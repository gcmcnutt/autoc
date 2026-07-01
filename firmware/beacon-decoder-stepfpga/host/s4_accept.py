#!/usr/bin/env python3
"""s4 acceptance — drive the FPGA (s4) over COM3. s4 = s3 + a REAL external ATtiny416 emitter muxed into code B.

Two groups, selected by the 'X' command (scenario.ps1 -Ext):
  SYNTHETIC (X0): code B = the internal synthetic emitter -> a regression that s4 kept s3's correlator behavior
                  (runs with NO external hardware).
  EXTERNAL  (X1): code B = the real 416 on P8 -> requires the emitter wired + running; verifies lock + that the
                  DPLL measures its (independent RC) chip rate, and that the lock survives noise.

Usage:  python3 s4_accept.py            # all
        python3 s4_accept.py synth      # synthetic regression only (no external HW)
        python3 s4_accept.py ext        # external-emitter only (needs the 416 wired)
"""
import subprocess, sys, statistics
from beacon_telemetry import TelemetryFrame
from beacon_telemetry.frame import chip_rate_hz

SB = "/mnt/c/fpga-build/stepfpga"
FRAME_MS = 25
CONFIRMED, TENTATIVE = 2, 1

def run(**kw):
    args = ["powershell.exe","-NoProfile","-ExecutionPolicy","Bypass","-File","scenario.ps1"]
    for k,v in kw.items(): args += [f"-{k}", str(v)]
    out = subprocess.run(args, cwd=SB, capture_output=True, text=True, timeout=120).stdout.splitlines()
    m = next((i for i,l in enumerate(out) if l.strip()=="MARK"), -1)
    return [f for f in (TelemetryFrame.parse(l) for l in out[m+1:]) if f]

def lk(fr,ch): return [(f.lockA if ch=='A' else f.lockB) for f in fr]
def held(fr,ch,lv): s=lk(fr,ch); return sum(v>=lv for v in s)/(len(s) or 1)
def first_at(fr,ch,lv):
    s=lk(fr,ch); i=next((i for i,v in enumerate(s) if v>=lv),None); return None if i is None else i*FRAME_MS

# ---- SYNTHETIC regression (X0): no external HW; proves the s4 correlator == s3 ----
def c_synth_time():
    fr=run(Ext=0,Mask=1,Action="flush",Settle=5,Watch=5); t=first_at(fr,'A',CONFIRMED)
    return f"synth cold→confirmed {t} ms", (t is not None and t<=600)
def c_synth_skewB():
    fr=run(Ext=0,Mask=2,FreqB=190,Action="flush",Settle=6,Watch=6); t=first_at(fr,'B',CONFIRMED)
    return f"synth B@+5% →confirmed {t} ms", (t is not None and t<=900)
def c_synth_floor():
    fr=run(Ext=0,Mask=0b100,AmpN=160,Settle=3,Watch=8); fa=held(fr,'A',CONFIRMED)+held(fr,'B',CONFIRMED)
    return f"synth noise-only false-lock {fa*100:.1f}%", (fa<=0.02)

# ---- EXTERNAL emitter (X1): the real 416 on P8 must be running (emits code B). Settle 8 s lets the DPLL fully
#      converge on the independent RC clock after the remote extsel switch (spread then ~telemetry resolution). ----
def c_ext_lock():
    fr=run(Ext=1,Mask=2,Settle=8,Watch=6); h=held(fr,'B',CONFIRMED)
    return f"external emitter lock {h*100:.0f}%", (h>=0.9)
def c_ext_rate():
    fr=run(Ext=1,Mask=2,Settle=8,Watch=6)
    rates=[chip_rate_hz(f.rateB) for f in fr if f.lockB>=CONFIRMED]
    if not rates: return "external rate: NO LOCK", False
    m=statistics.mean(rates)
    return f"external chip rate {m:.2f} Hz ({(m-200)/2:+.2f}% RC), spread {max(rates)-min(rates):.2f} Hz", (195.0<=m<=207.0)
def c_ext_noise():
    fr=run(Ext=1,Mask=0b110,AmpN=120,Settle=8,Watch=6); h=held(fr,'B',CONFIRMED)
    return f"external + noise lock {h*100:.0f}%", (h>=0.5)

CASES = [
  ("synth","Synthetic regression: time-to-signal",  c_synth_time),
  ("synth","Synthetic regression: B +5% skew",      c_synth_skewB),
  ("synth","Synthetic regression: no false lock",   c_synth_floor),
  ("ext",  "External emitter: lock",                c_ext_lock),
  ("ext",  "External emitter: rate measurement",    c_ext_rate),
  ("ext",  "External emitter: holds under noise",   c_ext_noise),
]

def main():
    sel=sys.argv[1:]
    cases=[c for c in CASES if not sel or any(s in c[0] or s in c[1].lower() for s in sel)]
    print(f"\n  s4 ACCEPTANCE   {len(cases)} cases   (ext cases need the 416 wired to P8 + running)\n  "+"-"*66)
    npass=0
    for tag,name,fn in cases:
        metric,ok,tries="",False,0
        for tries in range(1,4):
            try: metric,ok=fn()
            except Exception as e: metric,ok=f"ERROR {e}",False
            if ok: break
        npass+=ok
        note="" if (ok and tries==1) else f" ({tries} tries)"
        print(f"  [{'PASS' if ok else 'FAIL'}] {name:38s} {metric}{note}")
    print("  "+"-"*66); print(f"  {npass}/{len(cases)} passed\n")
    sys.exit(0 if npass==len(cases) else 1)

if __name__=="__main__": main()
