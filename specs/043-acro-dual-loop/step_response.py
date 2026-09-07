#!/usr/bin/env python3
"""043 T046 — control-responsiveness measurement, ONE script for sim and real.

The whole point is comparability: the same step detection, the same
normalisation, the same averaging, the same reported numbers, whichever side
the data came from.

  real: a decoded INAV blackbox CSV   (rcCommand[0..1] counts, gyroADC deg/s)
  sim:  a Cntrl_StepTest CSV          (cmd_aileron/elevator +-0.5, rate_p/q rad/s)

⚠️ Reports what the 2026-09-07 flight measured, so the two are read side by side:
  ROLL  -- first order: steady gain, time to 63%, and whether it overshoots
  PITCH -- short period: peak/steady ratio, ring frequency, and its trend with
           airspeed (a real short period MUST rise with speed; if it does not,
           distrust the fit before believing the aircraft)
"""
import argparse, csv, math, statistics as st, sys

RAD2DEG = 57.29577951308232

def load(path):
    """-> list of dicts with a common schema, and which side it is."""
    rows = list(csv.DictReader(open(path)))
    if not rows: sys.exit(f"ERROR: {path} is empty")
    cols = {k.strip() for k in rows[0]}
    out = []
    if "rate_p" in cols:                      # sim (Cntrl_StepTest)
        side = "sim"
        for r in rows:
            # ⛔ The auto-trim integrator moves the elevator during the "trim"
            # phase. That is not a commanded step and must not be detected as
            # one -- mark those rows not-manual so the detector skips them.
            _live = r.get("phase") not in ("trim",)
            out.append(dict(
                t=float(r["t_s"]),
                # to blackbox units so every number below is directly comparable:
                # surface +-0.5 -> +-500 counts, rad/s -> deg/s
                # ⛔ PITCH SIGN: crrcsim's elevator is INVERTED relative to the
                # rate it produces (+elevator -> -rate_q), which is why
                # cntrl_inavfwrate carries `pitchCmd = -2*elevator`. The
                # blackbox has command and gyro same-sense. Negate here so both
                # sides express "command in the sense of the rate it commands";
                # without this the sign-normalised average cancels itself.
                cmd=[float(r["cmd_aileron"]) * 1000.0, -float(r["cmd_elevator"]) * 1000.0],
                rate=[float(r["rate_p"]) * RAD2DEG, float(r["rate_q"]) * RAD2DEG],
                # ⚠️ crrcsim carries velocities in FT/S; the real side is m/s.
                speed=float(r["v_rel_airmass"]) * 0.3048,
                manual=_live))
    else:                                     # real (blackbox)
        side = "real"
        rows = [{k.strip(): v for k, v in r.items()} for r in rows]
        for r in rows:
            try:
                sp = math.sqrt(sum(float(r[f"navVel[{i}]"])**2 for i in range(3))) / 100.0
            except Exception:
                sp = float("nan")
            try:
                out.append(dict(
                    t=float(r["time (us)"]) / 1e6,
                    cmd=[float(r["rcCommand[0]"]), float(r["rcCommand[1]"])],
                    rate=[float(r["gyroADC[0]"]), float(r["gyroADC[1]"])],
                    speed=sp,
                    manual=(r["flightModeFlags (flags)"] == "ARM|MANUAL")))
            except (KeyError, ValueError):
                continue
    return out, side

def detect(rows, ax, thresh, dt):
    """Step edges: |Δcmd| > thresh inside an 85 ms window, 0.5 s refractory."""
    W = max(1, int(round(0.085 / dt)))
    c = [r["cmd"][ax] for r in rows]
    out, i = [], W
    while i < len(c) - W:
        if not rows[i]["manual"]:
            i += 1; continue
        d = c[i + W] - c[i - 1]
        if abs(d) > thresh and all(rows[j]["manual"] for j in range(i - 1, i + W + 1)):
            k = max(range(i - 1, i + W), key=lambda j: abs(c[j + 1] - c[j]))
            out.append((k, d)); i += max(1, int(round(0.5 / dt)))
        else:
            i += 1
    return out

def average(rows, ax, steps, dt, keep=lambda s: True):
    """Mean step response, normalised per 100 command counts, baseline removed."""
    # 0.9 s post-window: at 2.2-3.3 Hz the short period needs ~2 cycles, and a
    # 0.6 s window put the trough on the edge and manufactured a fake period.
    PRE, POST = max(1, int(round(0.10 / dt))), int(round(0.90 / dt))
    stack = []
    for k, d in steps:
        if k - PRE < 0 or k + POST >= len(rows): continue
        if not keep(rows[k]["speed"]): continue
        base = st.mean(rows[k - PRE + j]["rate"][ax] for j in range(PRE))
        sgn = 1.0 if d > 0 else -1.0
        row = [(rows[k - PRE + j]["rate"][ax] - base) * sgn / abs(d) * 100.0
               for j in range(PRE + POST)]
        if not any(math.isnan(x) for x in row): stack.append(row)
    if len(stack) < 3: return None, 0, PRE
    return [st.mean(col) for col in zip(*stack)], len(stack), PRE

def report(avg, PRE, dt, axis_name):
    ms = lambda i: (i - PRE) * dt * 1000.0
    post = avg[PRE:]
    pk = max(range(len(post)), key=lambda i: post[i])
    # settle estimate: the last ~150 ms of the window, never fewer than 4 samples
    ntail = max(4, int(round(0.150 / dt)))
    final = st.mean(post[-ntail:]) if len(post) > pk + ntail else post[-1]
    if axis_name == "ROLL":
        tgt = 0.63 * max(post)
        i63 = next((i for i, x in enumerate(post) if x >= tgt), None)
        first = next((i for i, x in enumerate(post) if abs(x) > 0.05 * max(post)), None)
        print(f"    steady gain      {max(post):7.1f} deg/s per 100 counts")
        print(f"    response by      {ms(PRE + first) if first is not None else float('nan'):7.0f} ms  (first 5% of final)")
        print(f"    63% of final at  {ms(PRE + i63) if i63 is not None else float('nan'):7.0f} ms")
        print(f"    peak/steady      {post[pk] / final if final else float('nan'):7.2f}x   "
              f"{'⭐ first-order, no overshoot' if post[pk] / max(final, 1e-9) < 1.15 else '⚠️ OVERSHOOTS'}")
    else:
        tr = min(range(pk + 1, len(post)), key=lambda i: post[i]) if pk + 1 < len(post) else pk
        pk2 = max(range(tr + 1, len(post)), key=lambda i: post[i]) if tr + 1 < len(post) else tr
        # ⛔ Degenerate cases must NOT be reported as a frequency: if the trough
        # or the second peak sits on the window edge, the response had not rung
        # inside the window and any "period" is an artefact of the edge.
        edge = 2
        degenerate = (tr >= len(post) - edge) or (pk2 >= len(post) - edge) or (pk2 <= tr)
        per = (ms(PRE + pk2) - ms(PRE + pk)) / 1000.0
        print(f"    peak             {post[pk]:7.1f} at {ms(PRE + pk):5.0f} ms")
        print(f"    trough           {post[tr]:7.1f} at {ms(PRE + tr):5.0f} ms")
        print(f"    2nd peak         {post[pk2]:7.1f} at {ms(PRE + pk2):5.0f} ms")
        print(f"    peak/steady      {post[pk] / final if final else float('nan'):7.2f}x")
        # ⭐ ROBUST estimator: for a 2nd-order step the time to FIRST peak is
        # t_pk = pi/omega_d, so omega_d = pi/t_pk. It needs only the peak --
        # the single most reliable feature here -- and does NOT require the
        # window to contain two cycles or the input to still be held. Prefer it;
        # the peak-to-peak period below is the corroborating estimate.
        t_pk = ms(PRE + pk) / 1000.0
        if t_pk > 0.02:
            print(f"    ⭐ f from t_peak  {math.pi / t_pk / (2*math.pi):7.2f} Hz  "
                  f"(omega_d = pi/t_pk; robust, needs only the peak)")
        if degenerate or per <= 0.02:
            print(f"    ⚠️ no clear ring inside the window — response still decaying at the edge;"
                  f" no frequency reported (widen POST or gather more steps)")
        else:
            print(f"    ⭐ ring period   {per * 1000:7.0f} ms = {1 / per:.2f} Hz  <- the SHORT PERIOD")

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csvfile", help="blackbox CSV (real) or Cntrl_StepTest CSV (sim)")
    ap.add_argument("--thresh", type=float, default=250.0,
                    help="step detection threshold in command counts (default 250)")
    a = ap.parse_args()

    rows, side = load(a.csvfile)
    dt = st.median([rows[i + 1]["t"] - rows[i]["t"] for i in range(min(500, len(rows) - 1))])
    print(f"{a.csvfile}\n  side={side}  n={len(rows)}  dt={dt*1000:.1f} ms ({1/dt:.0f} Hz)  "
          f"duration={rows[-1]['t']-rows[0]['t']:.1f} s")
    sp = [r["speed"] for r in rows if not math.isnan(r["speed"])]
    if sp: print(f"  airspeed: min {min(sp):.1f}  median {st.median(sp):.1f}  max {max(sp):.1f} m/s")

    bands = [("ALL", lambda s: True), ("slow <12", lambda s: s < 12),
             ("mid 12-17", lambda s: 12 <= s < 17), ("fast >=17", lambda s: s >= 17)]
    for ax, nm in ((0, "ROLL"), (1, "PITCH")):
        steps = detect(rows, ax, a.thresh, dt)
        print(f"\n{'='*66}\n{nm}: {len(steps)} step events\n{'='*66}")
        for bn, f in bands:
            avg, n, PRE = average(rows, ax, steps, dt, f)
            if avg is None: continue
            print(f"  --- {bn} (n={n}) ---")
            report(avg, PRE, dt, nm)

if __name__ == "__main__":
    main()
