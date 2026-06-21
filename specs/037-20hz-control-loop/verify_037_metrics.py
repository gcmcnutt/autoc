#!/usr/bin/env python3
# 037 metric cross-check.
#
# (A) Parses a training log's per-scenario variation table to confirm the
#     DEPLOYED run actually drew, within bounds:
#       - entry envelope (T006): cone half-angle <= 45 deg, speed delta <= 15%
#       - actuator-dynamics craft variations: servoTau in [0.005, 0.050] s,
#         servoSlew in [3, 9] /s, thrustTau in [0.050, 0.300] s (the clamps)
# (B) Reproduces the EXACT in-FDM servo filter math -- OLD (buggy slew x lag
#     compounding) vs NEW (1-exp lag + slew-cap-on-step) -- and reports the
#     time to reach a full-throw command, to confirm the fix (~6 full-throw/s,
#     not ~1.5) and that the lag is real (not a no-op like the thrust lag).
#
# Usage: python3 verify_037_metrics.py <training.log>
import sys, math, re

# ---- (A) variation-table parse -------------------------------------------
def parse_table(path):
    rows = []
    cols = None
    with open(path, errors="replace") as f:
        for line in f:
            # strip the "TS: <info> " prefix
            i = line.find("<info>")
            s = line[i+6:].strip() if i >= 0 else line.strip()
            if s.startswith("Scenario") and "ScenarioSeed" in s:
                cols = s
                continue
            if cols is None:
                continue
            if s.startswith("-") or s == "":
                if rows:
                    break
                continue
            # data row: idx hexseed heading roll pitch speed% winddir N E D cg drag trim thrSc pitEff rolEff svTau svSlew thrTau craftSeed
            t = s.split()
            if len(t) < 19 or not t[0].isdigit() or not t[1].startswith("0x"):
                if rows:
                    break
                continue
            try:
                rows.append({
                    "heading": float(t[2]), "roll": float(t[3]), "pitch": float(t[4]),
                    "speed": float(t[5].rstrip("%")),
                    "svTau": float(t[16]), "svSlew": float(t[17]), "thrTau": float(t[18]),
                })
            except (ValueError, IndexError):
                continue
    return rows

def rng(rows, k):
    v = [r[k] for r in rows]
    return min(v), max(v), sum(v)/len(v)

def report_A(path):
    rows = parse_table(path)
    if not rows:
        print("  (no variation table found -- craft variations off, or wrong log)")
        return
    print(f"  parsed {len(rows)} scenario rows")
    checks = [
        ("entry cone |heading|", "heading", 45.0, "deg", abs),
        ("entry cone |pitch|",   "pitch",   45.0, "deg", abs),
        ("|speed delta|",        "speed",   15.0, "%",   abs),
        ("servoTau",  "svTau",  None, "s",  None),
        ("servoSlew", "svSlew", None, "/s", None),
        ("thrustTau", "thrTau", None, "s",  None),
    ]
    bounds = {"svTau": (0.005, 0.050), "svSlew": (3.0, 9.0), "thrTau": (0.050, 0.300)}
    for label, key, cap, unit, fn in checks:
        v = [(fn(r[key]) if fn else r[key]) for r in rows]
        lo, hi, mean = min(v), max(v), sum(v)/len(v)
        if cap is not None:
            ok = "OK" if hi <= cap + 1e-9 else "FAIL"
            print(f"  {label:22s} max={hi:7.3f} {unit:3s} (<= {cap} ? {ok})  [mean {mean:.3f}]")
        else:
            blo, bhi = bounds[key]
            ok = "OK" if lo >= blo - 1e-9 and hi <= bhi + 1e-9 else "FAIL"
            print(f"  {label:22s} [{lo:.4f}, {hi:.4f}] {unit:3s} (clamp [{blo},{bhi}] ? {ok})  [mean {mean:.4f}]")

# ---- (B) servo-filter reproduction ---------------------------------------
def servo_new(target, tau, slew, dt, n):
    # exact dt-invariant lag + slew cap on the STEP (the fix)
    state = 0.0
    cap = 0.5 * slew * dt           # crrcsim +/-0.5 surface units
    blend = 1.0 - math.exp(-dt / tau) if tau > 0 else 1.0
    traj = []
    for _ in range(n):
        step = (target - state) * blend
        step = max(-cap, min(cap, step))
        state += step
        traj.append(state)
    return traj

def servo_old(target, tau, slew, dt, n):
    # buggy: slew-clamp the error THEN multiply by the lag blend (compounded)
    state = 0.0
    cap = 0.5 * slew * dt
    blend = min(1.0, dt / tau) if tau > 0 else 1.0
    traj = []
    for _ in range(n):
        delta = target - state
        delta = max(-cap, min(cap, delta))
        slewed = state + delta
        state += (slewed - state) * blend
        traj.append(state)
    return traj

def t_to_frac(traj, target, frac, dt):
    thr = frac * target
    for i, v in enumerate(traj):
        if v >= thr:
            return (i + 1) * dt
    return None

def report_B():
    tau, slew, dt = 0.020, 6.0, 0.005   # nominal centers, FDM substep (200 Hz)
    target = 0.5                         # full one-way deflection (crrcsim units)
    n = 600                              # 3 s
    for name, fn in (("NEW (fixed)", servo_new), ("OLD (buggy)", servo_old)):
        tr = fn(target, tau, slew, dt, n)
        t90 = t_to_frac(tr, target, 0.90, dt)
        t63 = t_to_frac(tr, target, 0.632, dt)
        eff_rate = (target / t90) / 0.5 if t90 else float("nan")  # full-throw/s (full=0.5)
        print(f"  {name:12s} t63%={1000*t63:5.1f} ms  t90%={1000*t90:6.1f} ms  "
              f"=> eff full-deflect rate ~ {eff_rate:.1f} full-throw/s")
    # small-step (lag-dominated, no slew clip) should show tau ~= 20 ms on NEW
    tr = servo_new(0.02, tau, slew, dt, n)
    t63 = t_to_frac(tr, 0.02, 0.632, dt)
    print(f"  NEW small-step t63% = {1000*t63:.1f} ms (expect ~{1000*tau:.0f} ms = tau)")

if __name__ == "__main__":
    print("=== (A) deployed variation-table check (entry envelope + dynamics draws/clamps) ===")
    if len(sys.argv) > 1:
        report_A(sys.argv[1])
    else:
        print("  (no log given)")
    print("=== (B) servo filter step response: OLD buggy vs NEW fixed ===")
    report_B()
