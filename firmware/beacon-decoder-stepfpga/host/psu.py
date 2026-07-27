#!/usr/bin/env python3
"""Siglent SPD1168X bench-supply driver + CLI (USBTMC via pyvisa-py; run with ~/.venvs/avr/bin/python).

Bring-up 2026-07-18: WSL has no usbtmc kernel driver -> userspace pyusb/pyvisa-py; device perms via
/etc/udev/rules.d/99-siglent.rules (idVendor f4ec MODE 0666); usbipd bind (admin, once) + attach per boot.

Quirks (all hit live):
  - The SPD parser is SLOW: pace ~0.5 s write->read (query_delay) or queries time out.
  - An abandoned/timed-out query WEDGES the USBTMC stack (even string descriptors stall) -> recover with a
    virtual replug: usbipd.exe detach --busid <b> && attach --wsl --busid <b>.
  - pyvisa-py lists VID/PID in DECIMAL: USB0::62700::5136::<serial>::INSTR (62700=0xF4EC).
  - status 0x10 bit = output ON; 0x01 bit = CC mode.

Usage:
  psu.py idn | meas | status
  psu.py set <volts> [amps]     # setpoint(s); does NOT touch output state
  psu.py out on|off
  psu.py ramp <v0> <v1> <step> <dwell_s>   # e.g. ramp 4.2 3.3 0.05 2  (UVLO sweeps; leaves output ON)
  psu.py profile <v0> <v1> <step> <dwell_s> <out.csv>
      # ramp + log the I-V curve: v_set,v_meas,i_meas,p_w per station. The emitter's input current profile:
      # boost CC load => I ~ 1/V (constant power); the UVLO trip shows as a current cliff at ~3.48 V.
      # NB SPD measurements are averaged -- the 200 Hz code envelope reads as its mean.
"""
import sys, time
import pyvisa

ADDR = 'USB0::62700::5136::SPD1XEAX4R0055::0::INSTR'

# ---- SAFETY GUARDRAILS (operator-set 2026-07-18; fire risk) ----
# The bench rig is 1S-LiPo-class hardware. Any commanded value beyond these limits raises — REFUSAL, not
# clamping, so a buggy script dies loudly instead of doing something subtly different. Raising the limits is a
# deliberate EDIT of these constants (field-power 306 mA runs need MAX_CURR ~1.3 A — change it consciously).
MAX_VOLT = 4.5    # V
MAX_CURR = 0.5    # A

class SPD1168X:
    def __init__(self, addr=ADDR):
        self.rm = pyvisa.ResourceManager('@py')
        self.i = self.rm.open_resource(addr)
        self.i.read_termination = '\n'; self.i.write_termination = '\n'
        self.i.timeout = 5000; self.i.query_delay = 0.6
    def q(self, cmd, tries=2):
        for _ in range(tries):
            try:
                time.sleep(0.4)
                r = self.i.query(cmd).strip()
                if r: return r
            except Exception: pass
        raise IOError(f"no reply to {cmd} (SPD wedged? -> usbipd detach/attach)")
    def w(self, cmd): time.sleep(0.4); self.i.write(cmd)
    # --- api ---
    def idn(self):            return self.q('*IDN?')
    def set_volt(self, v):
        if not (0.0 <= v <= MAX_VOLT):
            raise ValueError(f"REFUSED: {v} V outside guardrail 0..{MAX_VOLT} V (edit MAX_VOLT deliberately)")
        self.w(f'CH1:VOLT {v:.3f}')
    def set_curr(self, a):
        if not (0.0 <= a <= MAX_CURR):
            raise ValueError(f"REFUSED: {a} A outside guardrail 0..{MAX_CURR} A (edit MAX_CURR deliberately)")
        self.w(f'CH1:CURR {a:.3f}')
    def get_volt(self):       return float(self.q('CH1:VOLT?'))
    def meas(self):           return (float(self.q('MEAS:VOLT? CH1')), float(self.q('MEAS:CURR? CH1')))
    def output(self, on):     self.w(f'OUTP CH1,{"ON" if on else "OFF"}')
    def status(self):         return int(self.q('SYST:STAT?'), 16)
    def ramp(self, v0, v1, step, dwell, cb=None):
        """Step v0 -> v1 (inclusive-ish) holding `dwell` s per station; cb(v, vmeas, imeas) per station."""
        for v_ in (v0, v1):                                    # fail BEFORE moving anything
            if not (0.0 <= v_ <= MAX_VOLT):
                raise ValueError(f"REFUSED: ramp endpoint {v_} V outside guardrail 0..{MAX_VOLT} V")
        if step <= 0: raise ValueError("step must be > 0")
        v = v0; sgn = 1 if v1 >= v0 else -1
        while (sgn > 0 and v <= v1 + 1e-9) or (sgn < 0 and v >= v1 - 1e-9):
            self.set_volt(v)
            time.sleep(dwell)
            if cb:
                vm, im = self.meas(); cb(v, vm, im)
            v = round(v + sgn * step, 4)
    def close(self):
        try: self.i.close()
        except Exception: pass

def main():
    a = sys.argv[1:]
    if not a: print(__doc__); return
    p = SPD1168X()
    try:
        if   a[0] == 'idn':    print(p.idn())
        elif a[0] == 'meas':   vm, im = p.meas(); print(f"{vm:.3f} V  {im:.3f} A")
        elif a[0] == 'status': s = p.status(); print(f"0x{s:02X}  output={'ON' if s & 0x10 else 'OFF'}  mode={'CC' if s & 1 else 'CV'}")
        elif a[0] == 'set':
            p.set_volt(float(a[1]))
            if len(a) > 2: p.set_curr(float(a[2]))
            print("setpt:", p.get_volt())
        elif a[0] == 'out':    p.output(a[1].lower() == 'on'); print("status:", hex(p.status()))
        elif a[0] == 'ramp':
            v0, v1, st, dw = map(float, a[1:5])
            p.ramp(v0, v1, st, dw, cb=lambda v, vm, im: print(f"  {v:.3f} V set -> {vm:.3f} V {im:.3f} A", flush=True))
        elif a[0] == 'profile':
            v0, v1, st, dw = map(float, a[1:5]); path = a[5]
            with open(path, 'w') as f:
                f.write("v_set,v_meas,i_meas,p_w\n")
                def cb(v, vm, im):
                    f.write(f"{v:.4f},{vm:.3f},{im:.3f},{vm*im:.4f}\n"); f.flush()
                    print(f"  {v:.3f} V -> {vm:.3f} V  {im*1000:5.0f} mA  {vm*im:6.3f} W", flush=True)
                p.ramp(v0, v1, st, dw, cb=cb)
            print("csv ->", path)
        else: print(__doc__)
    finally:
        p.close()

if __name__ == '__main__':
    main()
