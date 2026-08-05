#!/usr/bin/env python3
"""Set C -- recovery ladder on the SYNTHETIC channel A. LINUX-NATIVE (bench-journal Set C).

WHY A NEW RUNNER. recovery_sweep.py/regression.py read telemetry through monitor.sh, which shells
out to powershell.exe against COM3 on the WINDOWS host. On the UBUNTU host the board is a plain
CDC and there is no interop (no powershell.exe, no usbipd.exe, no /mnt/c) -- so the MEASUREMENT
side of every existing harness is unreachable here. This drives and reads the CDC directly.

WHY CHANNEL A. Operator direction 2026-08-04: code B's optical amplitude is awkward to set
repeatably, while code A is the decoder's own injected test channel with a 12-bit amplitude knob
(s7.v: "code A = controlled/known test channel"). Same methodology, controllable stimulus.

⚠️ PORTS ARE INVERTED ON UBUNTU vs the WSL host (verified by udevadm + probe 2026-08-04):
      /dev/ttyACM0  ARM   DAPLink CMSIS-DAP  -> DECODER, telemetry AND command (was COM3)
      /dev/ttyACM1  ATMEL mEDBG   CMSIS-DAP  -> EMITTER pod cmd link (was /dev/ttyACM0)
  recovery_sweep.py defaults --port to /dev/ttyACM0, which HERE is the decoder.

⚠️ TRAP: cmd_reg powers up at 0, so sending '+' (REMOTE) ALONE disables BOTH correlators
  (enA=cmd_reg[0], enB=cmd_reg[1]). Always send the 0x80|mask FIRST, then '+'. Restore with '-'.

s7 knob protocol (s7.v ~line 296):
  '+' 0x2B REMOTE · '-' 0x2D LOCAL · 0x80|mask [0]enA [1]enB [3]inj1 [4]inj2 [5]weak
  'A' 0x41 <v> code-A amplitude (half-separation, v*6 -> 0..1530 about MID=2048)
  'K' 0x4B <v> code-A burst dropout span in CHIPS   <-- chip-exact, but see below
  'E' 0x45 <v> code-A clock skew

ARMS
  dark      -- RUNS TODAY. One-shot: clear enA for N chips, restore. injA falls to MID.
  saturate  -- CANNOT RUN. There is no rail-blank knob. Muting drives injA to MID (mid-scale),
               which is a clean signal removal with ZERO DC step; real saturation is a large DC
               step the tracker must chase. Needs a one-bit gateware change -- see the journal.

NOTE ON 'K': chip-exact but it is a REPEATING burst (blank the first v chips of every
BURST_WINDOW = 8*N = 248-chip window), not a one-shot. Good for steady-state occlusion duty, not
for single-event recovery. The one-shot ladder below is therefore host-timed (a few ms of jitter
against a 5 ms chip); telemetry is 40 Hz so measurement quantises to 25 ms = 5 chips regardless.

Timing of record: chip 5 ms · code N=31 chips · word 155 ms · BCN tick 25 ms (40 Hz).

Usage:  python3 set_c.py [--repeats 3] [--arm dark]
"""
import argparse, csv, os, sys, time
import serial

HERE = os.path.dirname(os.path.abspath(__file__))
TICK_MS, CHIP_MS, SEQ_MOD = 25.0, 5.0, 10000
MASK_AB, MASK_B_ONLY = 0x83, 0x82          # 0x80 | enA|enB , 0x80 | enB
LADDER_CHIPS = [16, 31, 62, 124, 196, 400]  # 16 = 60 deg/s sun transit; 196 = 10 deg/s


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--dev", default="/dev/ttyACM0")
    p.add_argument("--repeats", type=int, default=3)
    p.add_argument("--arm", default="dark", choices=["dark", "saturate"])
    p.add_argument("--settle", type=float, default=4.0)
    p.add_argument("--csv", default=None)
    return p.parse_args()


def frame(line):
    """BCN,seq,adc,corrA,lockA,marginA,corrB,lockB,marginB,rateA,rateB,recA,recB"""
    f = line.strip().split(",")
    if len(f) < 9 or f[0] != "BCN":
        return None
    try:
        return dict(seq=int(f[1]), adc=int(f[2]), corrA=int(f[3]), lockA=int(f[4]),
                    marginA=int(f[5]), corrB=int(f[6]), lockB=int(f[7]), marginB=int(f[8]))
    except ValueError:
        return None


def main():
    a = parse_args()
    if a.arm == "saturate":
        sys.exit("saturate arm needs a rail-blank gateware knob that does not exist yet "
                 "(see bench-journal Set C / C2). Muting only reaches MID, not a rail.")
    csv_path = a.csv or os.path.join(HERE, "results", f"set_c_{a.arm}_chA.csv")
    d = serial.Serial(a.dev, 115200, timeout=0.5)

    def send(b):
        d.write(b); d.flush(); time.sleep(0.05)

    send(bytes([MASK_AB])); send(b"+")           # mask FIRST, then REMOTE (see trap above)
    time.sleep(1.0)
    rows = []
    try:
        for chips in LADDER_CHIPS:
            dur = chips * CHIP_MS / 1000.0
            for rep in range(a.repeats):
                d.reset_input_buffer()
                caps, st, t_cmd, t_res, t0 = [], 0, None, None, time.time()
                while True:                      # capture CONTINUOUSLY across the whole event
                    fr = frame(d.readline().decode("ascii", "replace"))
                    now = time.time()
                    if fr:
                        fr["t"] = now; caps.append(fr)
                    if st == 0 and len(caps) >= 8:
                        send(bytes([MASK_B_ONLY])); t_cmd = time.time(); st = 1
                    elif st == 1 and now - t_cmd >= dur:
                        send(bytes([MASK_AB])); t_res = time.time(); st = 2
                    elif st == 2 and now - t_res >= a.settle:
                        break
                    if now - t0 > dur + a.settle + 20:
                        break
                ev = [c for c in caps if c["t"] >= t_cmd - 0.05]
                li = next((i for i, c in enumerate(ev) if c["lockA"] < 2), None)
                n = None
                if li is not None:
                    for i in range(li, len(ev) - 1):
                        if ev[i]["lockA"] >= 2 and ev[i + 1]["lockA"] >= 2:
                            n = (ev[i]["seq"] - ev[li]["seq"]) % SEQ_MOD
                            break
                mm = min((c["marginA"] for c in ev), default=None)
                ums = n * TICK_MS if n is not None else None
                rows.append(dict(arm=a.arm, chips=chips, rep=rep, dur_ms=round(dur * 1000, 1),
                                 unlock_frames=n, unlock_ms=ums, min_marginA=mm,
                                 rode_through=(li is None), frames=len(ev)))
                tag = ("RODE THROUGH" if li is None else
                       ("NO RELOCK in window" if n is None else
                        f"unlock={n:>3} fr ({ums:>6.0f} ms)"))
                print(f"  {chips:>4} chips ({dur*1000:>6.0f} ms) rep{rep}: {tag}  "
                      f"minMarginA={mm}", flush=True)
    finally:
        send(bytes([MASK_AB])); send(b"-")       # knobs back on, decoder back to LOCAL/switches
        d.close()

    os.makedirs(os.path.dirname(csv_path), exist_ok=True)
    keys = sorted({k for r in rows for k in r})
    with open(csv_path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys); w.writeheader(); w.writerows(rows)
    print(f"\nwrote {csv_path} ({len(rows)} trials)")


if __name__ == "__main__":
    sys.exit(main())
