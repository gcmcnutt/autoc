#!/usr/bin/env python3
"""Dropout->recovery characterization: REAL emitter (ATtiny416 over /dev/ttyACM0) vs the s6 decoder (COM3 BCN).

Methodology (clean-state): every trial starts from a VERIFIED locked baseline at nominal frequency; one
perturbation per trial; no compound slews. Signal loss = 'D' 0x1F (blank all 31 chips -> DIM held low =
true loss of signal); restore = 'R'. The ladder sweeps loss duration from 1 chip (~5 ms) to 8 s, each
repeated N times (default 3) to expose ratchet patterns in recovery time.

Measurement: one isolated COM3 capture per trial; the BCN seq field (40 Hz -> 25 ms/tick) is the timebase.
  unlock_span = first frame lockB<2  ..  first of 2 consecutive frames lockB==2
  recovery    = unlock_span - commanded loss duration
Trials where lockB never leaves 2 rode through in the HOLD state (confidence ladder reports HOLD as 2) --
these report the marginB dip instead. NB the s6 coast window is ~10 s (COASTMAX=65 periods): all rungs
<= 8 s should WARM re-lock; a rung > ~10 s would go cold (see --ladder to add one).

Usage:  ~/.venvs/avr/bin/python recovery_sweep.py [--repeats 3] [--ladder "0.005,0.025,..."] [--port /dev/ttyACM0]
"""
import argparse, csv, os, re, subprocess, sys, time
import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ports import emitter_port

HERE = os.path.dirname(os.path.abspath(__file__))
TICK_MS = 25.0                    # BCN frame period (40 Hz)
SEQ_MOD = 10000

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default=None, help="emitter CDC; default = mEDBG resolved via /dev/serial/by-id")
    p.add_argument("--com", default="COM3")
    p.add_argument("--repeats", type=int, default=3)
    p.add_argument("--ladder", default="0.005,0.025,0.078,0.155,0.5,1,2,4,8",
                   help="comma-sep loss durations in seconds")
    p.add_argument("--csv", default=os.path.join(HERE, "results", "recovery_sweep.csv"))
    a = p.parse_args()
    if a.port is None: a.port = emitter_port()
    return a

ATTACH_SH = os.path.abspath(os.path.join(HERE, "..", "..", "beacon-pod", "attach-medbg.sh"))

class Emitter:
    """Holds the CDC open for the whole run (DTR asserted un-tri-states the mEDBG UART bridge).
    The mEDBG randomly drops off USB (seen 2026-07-16, twice): on any serial error, re-run
    attach-medbg.sh, reopen the port, and retry the command once before giving up."""
    def __init__(self, port):
        self.port = port
        self._open()
    def _open(self):
        self.s = serial.Serial(self.port, 38400, timeout=0.5)
        self.s.dtr = True
        time.sleep(0.3)
        self.s.reset_input_buffer()
    def _reattach(self):
        print("  !! emitter CDC lost -> re-attaching mEDBG…", flush=True)
        try: self.s.close()
        except Exception: pass
        for _ in range(2):
            subprocess.run([ATTACH_SH], capture_output=True, timeout=60)
            if os.path.exists(self.port):
                try: self._open(); return True
                except Exception: pass
            time.sleep(1)
        return False
    def cmd(self, b: bytes, label=""):
        for attempt in (1, 2):
            try:
                self.s.reset_input_buffer()
                self.s.write(b)
                echo = self.s.read(len(b))
                if echo == b: return True
                print(f"  !! echo mismatch on {label or b.hex()}: sent {b.hex()} got {echo.hex()}", flush=True)
                if attempt == 1 and not self._reattach(): break
            except (serial.SerialException, OSError):
                if attempt == 1 and not self._reattach(): break
        return False
    def stop(self):    return self.cmd(b"\x44\x1f", "D31(stop)")
    def restore_200(self): return self.cmd(b"\x52", "R")  # RAW 'R': clears knobs AND jumps to 200 Hz nominal
    def bench_rate(self): return self.cmd(b"\x48", "H")   # 120 Hz, the rig's rate of record
    def restore(self):
        # BENCH POLICY (operator 2026-08-19; completed 2026-08-20 with the 120 Hz retune). The rig is
        # bench-rate ONLY — pod, decoder gateware and camera receiver are all pinned to 120 Hz, and the
        # decoder has no 200 Hz template, so a 200 Hz emitter reads as NO LOCK *by design*.
        # 'R' is the only command that clears the corruption/dropout/pulse knobs, but it ALSO jumps the
        # chip clock to 200 — so every restore must put 'H' back. Folding that pairing into restore()
        # itself (rather than leaving it to each caller) is what fixes regression.py: its 12 failures on
        # 2026-08-20 were ALL "harness left the pod at 200 Hz and then asserted on lock". Callers that
        # genuinely want the 200 Hz flight nominal must say restore_200() explicitly.
        ok = self.restore_200()
        return self.bench_rate() and ok
    def close(self):
        try:
            self.restore()          # already R-then-H; no harness exit leaves the pod at 200 Hz
        except Exception: pass
        finally:
            try: self.s.close()
            except Exception: pass

def capture(com, seconds):
    """Blocking COM3 read via the proven Windows-side reader; returns parsed frames [(seq,lockB,marginB),...]."""
    out = subprocess.run([os.path.join(HERE, "monitor.sh"), com, str(seconds)],
                         capture_output=True, text=True, timeout=seconds + 60)
    frames = []
    for line in out.stdout.splitlines():
        f = line.strip().split(",")
        if len(f) == 13 and f[0] == "BCN" and all(re.fullmatch(r"\d+", x) for x in (f[1], f[7], f[8])):
            frames.append((int(f[1]), int(f[7]), int(f[8])))     # seq, lockB, marginB
    return frames

def unwrap(frames):
    """seq wraps at 10000 -> monotonic tick count."""
    out, off, prev = [], 0, None
    for seq, lk, mg in frames:
        if prev is not None and seq < prev - 5000: off += SEQ_MOD
        prev = seq
        out.append((seq + off, lk, mg))
    return out

GOOD = 5   # decoder's q>=GOOD lock threshold; margin<GOOD marks signal-loss ONSET (<=1 period after the stop)

def analyze(frames, loss_ms):
    """Onset-anchored: t_onset = first frame marginB<GOOD (margin collapses <=1 period after signal stop, well
    before HOLD gives up lockB). recovery = (t_relock - t_onset) - loss  (+/- ~1 period ~155 ms uncertainty).
    -> dict(outcome, hold_ms, unlock_ms, recovery_ms, min_marginB, n)"""
    fr = unwrap(frames)
    if not fr: return dict(outcome="NO-DATA", hold_ms=None, unlock_ms=None, recovery_ms=None, min_marginB=None, n=0)
    min_mg = min(m for _, _, m in fr)
    t_onset = next((t for t, _, m in fr if m < GOOD), None)
    t_bad   = next((t for t, lk, _ in fr if lk < 2), None)
    if t_onset is None:
        return dict(outcome="NO-IMPACT", hold_ms=0.0, unlock_ms=0.0, recovery_ms=0.0, min_marginB=min_mg, n=len(fr))
    # re-lock = 2 consecutive frames with lockB==2 AND margin back >= GOOD, after onset
    after = [(t, lk, m) for t, lk, m in fr if t > t_onset]
    t_relock = None
    for (t1, l1, m1), (t2, l2, m2) in zip(after, after[1:]):
        if l1 == 2 and m1 >= GOOD and l2 == 2 and m2 >= GOOD: t_relock = t1; break
    hold_ms   = None if t_bad is None else (t_bad - t_onset) * TICK_MS
    unlock_ms = None if t_bad is None else ((after[-1][0] if t_relock is None else t_relock) - t_bad) * TICK_MS
    if t_relock is None:
        return dict(outcome="NO-RELOCK", hold_ms=hold_ms, unlock_ms=unlock_ms, recovery_ms=None,
                    min_marginB=min_mg, n=len(fr))
    recovery = (t_relock - t_onset) * TICK_MS - loss_ms
    outcome = "RODE-THROUGH" if t_bad is None else "RECOVERED"
    return dict(outcome=outcome, hold_ms=hold_ms, unlock_ms=unlock_ms, recovery_ms=recovery,
                min_marginB=min_mg, n=len(fr))

def baseline_ok(com):
    fr = capture(com, 2)
    return len(fr) >= 3 and all(lk == 2 for _, lk, _ in fr[-3:])

def main():
    a = parse_args()
    ladder = [float(x) for x in a.ladder.split(",")]
    em = Emitter(a.port)
    os.makedirs(os.path.dirname(a.csv), exist_ok=True)
    rows = []
    try:
        em.restore(); time.sleep(4)
        for rep in range(1, a.repeats + 1):
            for d in ladder:
                loss_ms = d * 1000.0
                # --- verified clean baseline (resettle once if needed) ---
                if not baseline_ok(a.com):
                    print(f"  [rep{rep} d={d}s] baseline not locked -> resettle 6s", flush=True)
                    em.restore(); time.sleep(6)
                    if not baseline_ok(a.com):
                        rows.append(dict(rep=rep, loss_s=d, outcome="INVALID-BASELINE")); continue
                # --- capture spans the whole event; reader start latency covered by lead-in sleep ---
                cap_s = int(3 + d + (10 if d >= 2 else 8))
                proc = subprocess.Popen([os.path.join(HERE, "monitor.sh"), a.com, str(cap_s)],
                                        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
                time.sleep(2.5)                                   # powershell/reader spin-up
                if not em.stop():                                 # CDC died even after reattach -> invalid trial
                    proc.kill(); em.restore()
                    rows.append(dict(rep=rep, loss_s=d, outcome="CMD-FAIL")); continue
                time.sleep(d); em.restore()                       # the one perturbation
                out, _ = proc.communicate(timeout=cap_s + 60)
                frames = []
                for line in out.splitlines():
                    f = line.strip().split(",")
                    if len(f) == 13 and f[0] == "BCN" and f[1].isdigit():
                        frames.append((int(f[1]), int(f[7]), int(f[8])))
                r = analyze(frames, loss_ms)
                r.update(rep=rep, loss_s=d)
                rows.append(r)
                rm = "-" if r.get("recovery_ms") is None else f"{r['recovery_ms']:.0f}ms"
                print(f"  [rep{rep} d={d:>6}s] {r['outcome']:13s} hold={r.get('hold_ms')} unlock={r.get('unlock_ms')} "
                      f"recovery={rm} minMarginB={r.get('min_marginB')}", flush=True)
                time.sleep(2)                                     # settle before next baseline check
    finally:
        em.close()
    # --- summary table + ratchet view ---
    print("\n==== recovery vs loss duration (per repeat -> ratchet check) ====")
    print(f"{'loss':>8} | " + " | ".join(f"rep{r}" for r in range(1, a.repeats + 1)) + " | outcome")
    for d in ladder:
        rs = [next((x for x in rows if x['rep'] == r and x['loss_s'] == d), None) for r in range(1, a.repeats + 1)]
        cells = ["   n/a" if x is None or x.get("recovery_ms") is None else f"{x['recovery_ms']:5.0f}" for x in rs]
        oc = ",".join(sorted({x['outcome'] for x in rs if x}))
        print(f"{d:>7}s | " + " | ".join(cells) + f" | {oc}")
    with open(a.csv, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=["rep", "loss_s", "outcome", "hold_ms", "unlock_ms", "recovery_ms", "min_marginB", "n"])
        w.writeheader()
        for r in rows: w.writerow({k: r.get(k) for k in w.fieldnames})
    print(f"\ncsv -> {a.csv}")

if __name__ == "__main__":
    main()
