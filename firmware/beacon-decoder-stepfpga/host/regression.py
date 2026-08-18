#!/usr/bin/env python3
"""FULL bench regression: PSU-powered cold start -> decode -> dropouts -> AGC -> injection -> skew -> I-V ->
UVLO trip + UPDI recovery -> restore. Run with ~/.venvs/avr/bin/python. Policy (2026-07-18): this suite runs
after ANY emitter-firmware or decoder-gateware change.

Instruments: SPD1168X (psu.py, guardrailed), emitter cmd link (recovery_sweep.Emitter: R/F/C/D/P), decoder BCN
telemetry (monitor.sh -> COM3), pymcuprog UPDI reset (clears the UVLO latch that mEDBG trickle preserves —
supply-only power cycles CANNOT clear it with USB attached, see beacon-pod/SETUP.md).

Thresholds are envelopes from the 2026-07-16..18 characterization (optical-link-outcome.md, agc_step.log):
baseline-relative where geometry-dependent.
"""
import os, subprocess, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from psu import SPD1168X
from recovery_sweep import Emitter

HERE = os.path.dirname(os.path.abspath(__file__))
PYMCU = os.path.expanduser('~/.venvs/avr/bin/pymcuprog')
results = []

def capture(sec):
    out = subprocess.run([os.path.join(HERE, 'monitor.sh'), 'COM3', str(sec)],
                         capture_output=True, text=True, timeout=sec + 60)
    rows = []
    for line in out.stdout.splitlines():
        f = line.strip().split(',')
        if len(f) == 13 and f[0] == 'BCN' and f[1].isdigit():
            try:  # serial reads can hand back a truncated row (empty field) — skip, don't die
                rows.append(dict(adc=int(f[2]), lkB=int(f[7]), mB=int(f[8]), corrB=int(f[6]), rB=int(f[10])))
            except ValueError:
                continue
    return rows

def stats(rows):
    if not rows: return dict(n=0, lockpct=0, m_med=0, c_med=0)
    ls = sorted(r['mB'] for r in rows); cs = sorted(r['corrB'] for r in rows)
    return dict(n=len(rows), lockpct=100 * sum(1 for r in rows if r['lkB'] == 2) // len(rows),
                m_med=ls[len(ls)//2], c_med=cs[len(cs)//2])

def check(name, ok, detail):
    results.append((name, bool(ok), detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}", flush=True)

def updi_reset():
    subprocess.run([PYMCU, 'reset', '-d', 'attiny416'], capture_output=True, timeout=60)

def usb_preflight():
    """Bench replugs shuffle busids and drop attachments. Attach anything bound-but-detached before starting.
    Native-Linux bench (no usbipd interop): nothing to attach — devices are direct USB."""
    try:
        out = subprocess.run(['usbipd.exe', 'list'], capture_output=True, text=True, timeout=30).stdout.replace('\r', '')
    except FileNotFoundError:
        print("  (pre-flight: native Linux host, no usbipd — skipping)", flush=True)
        return
    for vidpid, name in (('f4ec:1410', 'SPD1168X'), ('03eb:2145', 'mEDBG')):
        for line in out.splitlines():
            if vidpid in line and 'Attached' not in line:
                busid = line.split()[0]
                print(f"  (pre-flight: attaching {name} at {busid})", flush=True)
                subprocess.run(['usbipd.exe', 'attach', '--wsl', '--busid', busid], capture_output=True, timeout=30)
                time.sleep(2)

def main():
    usb_preflight()
    psu = SPD1168X()
    print("== P0: cold start from PSU (output off -> on) ==")
    psu.set_volt(4.2); psu.set_curr(0.45); psu.output(True); time.sleep(2)
    em = Emitter('/dev/ttyACM0')
    alive = em.restore()
    if not alive:                                   # trickle-latched from a prior trip -> UPDI reset clears
        print("  (no echo -> UPDI reset to clear a trickle-preserved latch)")
        updi_reset(); time.sleep(2); alive = em.restore()
    check("P0 cold-start echo", alive, f"emitter alive={alive}")
    vm, im = psu.meas()
    check("P0 rail", 4.0 < vm < 4.3 and 0.01 < im < 0.45, f"{vm:.3f} V {im*1000:.0f} mA")

    print("== P1: decode baseline ==")
    time.sleep(3)
    base = stats(capture(8))
    check("P1 lock", base['lockpct'] >= 90 and base['m_med'] >= 6,
          f"lock {base['lockpct']}% margin {base['m_med']} corr {base['c_med']}")
    C0 = max(base['c_med'], 1)

    print("== P2: dropout mini-ladder (D31 hold; envelopes from the 5-geometry sweeps) ==")
    for d, must_ride in ((0.025, True), (0.155, True), (1.0, False), (4.0, False)):
        cap = subprocess.Popen([os.path.join(HERE, 'monitor.sh'), 'COM3', str(int(d + 8))],
                               stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        time.sleep(2.5); em.stop(); time.sleep(d); em.restore()
        out, _ = cap.communicate(timeout=int(d) + 60)
        rows = [l.strip().split(',') for l in out.splitlines()]
        rows = [r for r in rows if len(r) == 13 and r[0] == 'BCN']
        unlocked = sum(1 for r in rows if int(r[7]) < 2)
        endlock = rows and int(rows[-1][7]) == 2
        ok = (unlocked == 0 if must_ride else endlock)
        check(f"P2 dropout {d}s", ok, f"unlocked_frames={unlocked} relocked={endlock}")
        time.sleep(2)

    print("== P3: AGC pulse-width ladder + step response ==")
    prev = None; mono = True
    for v in (128, 32, 16):
        em.cmd(bytes([0x50, v]), f"P{v}"); time.sleep(2)
        s = stats(capture(6))
        if prev is not None and s['c_med'] >= prev: mono = False
        prev = s['c_med']
        check(f"P3 P={v}", s['lockpct'] >= 90 and s['m_med'] >= 5,
              f"lock {s['lockpct']}% margin {s['m_med']} corr {s['c_med']}")
    check("P3 corr monotonic", mono, "corr decreases with duty")
    em.restore(); time.sleep(1)
    # step response: down 255->16 then up; settle envelopes from agc_step.log (down<5s, up<3s)
    cap = subprocess.Popen([os.path.join(HERE, 'monitor.sh'), 'COM3', '20'],
                           stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    time.sleep(2.5); em.cmd(b'\x50\x10', 'P16'); time.sleep(8); em.restore()
    out, _ = cap.communicate(timeout=80)
    rows = [l.strip().split(',') for l in out.splitlines()]
    rows = [(i, int(r[8])) for i, r in enumerate(rows) if len(r) == 13 and r[0] == 'BCN']
    down_i = next((i for i, m in rows if m <= 3), None)
    settle = next((i for i, m in rows[ (down_i or 0):] if m >= 5), None) if down_i is not None else None
    ok = down_i is not None and settle is not None and (settle - down_i) * 0.025 < 1.0  # s7 gear-shift bar (s6 measured 1.23 s)
    check("P3 step settle", ok,
          f"down@{down_i} settle@{settle} ({(settle-down_i)*0.025:.2f}s)" if ok else f"down={down_i} settle={settle}")

    print("== P4: error injection + rate skew ==")
    em.cmd(b'\x43\x0a', 'C10'); time.sleep(2)
    s = stats(capture(6))
    check("P4 corrupt10 degrades", s['m_med'] <= base['m_med'] - 1 or s['lockpct'] < 90,
          f"margin {s['m_med']} (base {base['m_med']}) lock {s['lockpct']}%")
    em.restore(); time.sleep(3)
    s = stats(capture(5))
    check("P4 recover", s['lockpct'] >= 90 and s['m_med'] >= base['m_med'] - 1,
          f"lock {s['lockpct']}% margin {s['m_med']}")
    em.cmd(bytes([0x46, 128 + 16]), 'F+2.6%'); time.sleep(4)     # 0.16%/step @10 MHz
    s = stats(capture(6))
    check("P4 skew +2.6% holds", s['lockpct'] >= 85, f"lock {s['lockpct']}% margin {s['m_med']}")
    em.restore(); time.sleep(2)

    print("== P5: I-V profile 4.2->3.7 (above trip: no LOS allowed) ==")
    iv = []
    psu.ramp(4.2, 3.7, 0.1, 2, cb=lambda v, vm, im: iv.append((v, vm, im)))
    s = stats(capture(4))
    ok = s['lockpct'] >= 90 and all(im < 0.45 for _, _, im in iv)
    check("P5 decode across 4.2-3.7 V", ok,
          "; ".join(f"{v:.1f}V:{im*1000:.0f}mA" for v, _, im in iv) + f" | lock {s['lockpct']}%")

    print("== P6: UVLO trip + UPDI recovery (trickle-latch aware) ==")
    cap = subprocess.Popen([os.path.join(HERE, 'monitor.sh'), 'COM3', '30'],
                           stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    time.sleep(2)
    psu.ramp(3.65, 3.40, 0.05, 3)
    out, _ = cap.communicate(timeout=90)
    rows = [l.strip().split(',') for l in out.splitlines()]
    rows = [r for r in rows if len(r) == 13 and r[0] == 'BCN']
    tripped = rows and all(int(r[7]) < 2 for r in rows[-40:])
    check("P6 UVLO trip <=3.45 V", tripped, f"dark at end of ramp={tripped}")
    psu.set_volt(4.2); time.sleep(1)
    updi_reset(); time.sleep(3)                                  # the ONLY reliable un-latch with USB attached
    s = stats(capture(6))
    check("P6 UPDI recovery", s['lockpct'] >= 90 and s['m_med'] >= 6,
          f"lock {s['lockpct']}% margin {s['m_med']}")

    print("== P7: restore + final state ==")
    em.restore(); em.close(); psu.set_volt(4.2); psu.set_curr(0.45)
    s = stats(capture(5)); vm, im = psu.meas(); psu.close()
    check("P7 final", s['lockpct'] >= 90, f"lock {s['lockpct']}% margin {s['m_med']} | {vm:.2f} V {im*1000:.0f} mA")

    npass = sum(1 for _, ok, _ in results if ok)
    print(f"\n==== REGRESSION: {npass}/{len(results)} PASS ====")
    for n, ok, d in results:
        if not ok: print(f"  FAIL {n}: {d}")
    sys.exit(0 if npass == len(results) else 1)

if __name__ == '__main__':
    main()
