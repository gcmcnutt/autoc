#!/usr/bin/env python3
"""beacon_track.py -- real-time single-beacon detector/tracker on Pi + OV9281 (031 A8-6 prototype, v2).

Two-stage (the tracker-bank idea in miniature), sized to a Pi 3A+ at 250 fps (4 ms/frame budget):
  per frame  : 2x2 SUM to 320x200 in uint16 (~1.5 ms) -> raw ring of the last WORD frames. Nothing else.
  per report : ACQUIRE = full-field matched filter (~1.1 s on Pi 3 -> only when unlocked, and it runs on a
               snapshot so the ring keeps filling); TRACK = matched filter on a small ROI around the last fix
               (~1024 px -> ~20 ms). DC removal = subtract each pixel's ring-mean (exact HP over one word).
               Lock/unlock hysteresis on the scale-free quality q. Emits t code x y cep q az el fps.
Chip rate must match the emitter (bench 'H' ~121 Hz measured; 'R' ~209). fpc = fps/chip.
"""
import argparse, subprocess, time
import numpy as np

CODES = {"A": "0000000100011011000011001110011", "B": "0100011001100111100101001011110"}
W, H = 640, 400; RW, RH = W//2, H//2
F_PX = 750.0/4.0                 # measured f≈750 px @1280 wide -> 187.5 px in the 320-wide frame
CX, CY = RW/2.0, RH/2.0
ROI = 16                          # half-size of the tracking window (px, reduced frame)

def templates(fpc, M):
    t = np.arange(M)/fpc; out = {}
    for name, bits in CODES.items():
        c = np.array([1.0 if b == "1" else -1.0 for b in bits], dtype=np.float32)
        Tm = np.stack([c[(np.floor(t).astype(int)+p) % 31] for p in range(31)])
        out[name] = (Tm - Tm.mean(axis=1, keepdims=True)).astype(np.float32)
    return out

def correlate(R, T):
    """R: (M x P) raw ring (oldest first). Returns best (q, code, pixel-index, qmap, peak)."""
    Rz = R - R.mean(axis=0, keepdims=True)                          # per-pixel DC removal over the word
    energy = np.sqrt((Rz*Rz).sum(axis=0)) + 1e-3
    best = None
    for name, Tm in T.items():
        C = Tm @ Rz                                                 # (31 x P) all code phases
        pk = C.max(axis=0); q = pk/energy/np.sqrt(Rz.shape[0])
        i = int(np.argmax(q))
        if best is None or q[i] > best[0]: best = (float(q[i]), name, i, q, float(pk[i]), int(np.argmax(C[:, i])))
    return best

def centroid_cep(qmap2d, x, y):
    y0,y1 = max(0,y-2),min(qmap2d.shape[0],y+3); x0,x1 = max(0,x-2),min(qmap2d.shape[1],x+3)
    w = qmap2d[y0:y1,x0:x1]; w = np.clip(w-w.min(),0,None)+1e-6
    yy,xx = np.mgrid[y0:y1,x0:x1]; s=w.sum(); cx=(w*xx).sum()/s; cy=(w*yy).sum()/s
    cep = float(np.sqrt(((w*((xx-cx)**2+(yy-cy)**2)).sum()/s)))
    return cx, cy, max(cep, 0.7)

def bearing(cx, cy):
    dx,dy = cx-CX, cy-CY; r = float(np.hypot(dx,dy)); th = np.degrees(r/F_PX)
    return (th*dx/r, -th*dy/r) if r > 1e-6 else (0.0, 0.0)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fps", type=float, default=250); ap.add_argument("--chip", type=float, default=121.0)
    ap.add_argument("--report", type=float, default=0.5); ap.add_argument("--shutter", type=int, default=2000)
    ap.add_argument("--gain", type=float, default=1.0); ap.add_argument("--secs", type=float, default=0)
    ap.add_argument("--qlock", type=float, default=0.62, help="lock threshold on q"); ap.add_argument("--qdrop", type=float, default=0.55)
    ap.add_argument("--json", action="store_true", help="emit JSON lines (for the DGX display)")
    ap.add_argument("--hold", type=int, default=6, help="reports of low q to ride through before dropping lock")
    ap.add_argument("--agc", action="store_true", help="adapt shutter to keep the beacon's lit level in band (restart capture)")
    ap.add_argument("--agc-lo", type=int, default=90, help="lit-level below which shutter doubles")
    ap.add_argument("--agc-hi", type=int, default=230, help="lit-level above which shutter halves")
    a = ap.parse_args()
    import json
    fpc = a.fps/a.chip; M = int(round(31*fpc)); T = templates(fpc, M)
    M_ring = max(M, int(round(31*a.fps/115.0)))          # ring long enough for the slowest bench mode (115 Hz)
    print(f"# fps={a.fps} chip={a.chip} fpc={fpc:.2f} word={M}fr ({M/a.fps*1000:.0f}ms) shutter={a.shutter}us gain={a.gain} roi=±{ROI}", flush=True)
    shutter=a.shutter
    def start(sh):
        return subprocess.Popen(["rpicam-raw","-n","-t",str(int(a.secs*1000) if a.secs else 0),"--mode",f"{W}:{H}:8",
                          "--framerate",str(int(a.fps)),"--shutter",str(sh),"--gain",str(a.gain),"-o","-"],
                         stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=W*H*8)
    p = start(shutter); agc_cool=0
    ring = np.zeros((M_ring, RH*RW), dtype=np.uint16); k=0; n=0; t0=time.time(); last=t0
    locked=False; lx=ly=0; FS=W*H
    chip=a.chip; last_ph=None; last_n=None; ph_acc=0.0   # DPLL-lite state
    hold=0; good=None                                     # HOLD: ride through bad words; last good fix
    rep_frames=max(1,int(round(a.report*a.fps))); next_rep=M_ring   # report on FRAME count, not wall clock
    while True:
        buf = p.stdout.read(FS)
        if len(buf) < FS: break
        f = np.frombuffer(buf, dtype=np.uint8).reshape(H, W)
        ring[k] = (f[0::2,0::2].astype(np.uint16)+f[1::2,0::2]+f[0::2,1::2]+f[1::2,1::2]).reshape(-1)
        k=(k+1)%M_ring; n+=1
        now=time.time()
        if n >= next_rep:
            next_rep = n + rep_frames
            R = np.concatenate((ring[k:], ring[:k]), axis=0).astype(np.float32)   # snapshot, oldest first (M_ring rows)
            ta=time.time()
            if not locked:
                # ACQUIRE across candidate chip rates: the emitter's 'H' bench mode is VOLATILE (any reset ->
                # 200 Hz nominal); a wrong template scores the beacon at noise. Try {--chip, 200-mode, 115-mode}
                # each x(1, 1.05) RC skew, keep the best. (Costs a few full-field passes only while unlocked.)
                cands = sorted({round(a.chip,1), round(a.chip*1.05,1), 200.0, 210.0, 115.0, 121.0})
                best = None
                for cr in cands:
                    fpc_c = a.fps/cr; M_c = int(round(31*fpc_c))
                    if M_c > R.shape[0]: continue
                    T_c = templates(fpc_c, M_c)
                    r_ = correlate(R[-M_c:], T_c)
                    if best is None or r_[0] > best[0][0]: best = (r_, cr, fpc_c, M_c, T_c)
                (q,name,i,qmap,pk,ph), cr, fpc_c, M_c, T_c = best
                if q >= a.qlock and abs(cr - chip) > 0.5:
                    chip = cr; fpc = fpc_c; T = T_c          # adopt the winning rate (ring length M stays max)
                y,x = divmod(i, RW); mode="ACQ"
            else:
                y0,y1 = max(0,ly-ROI),min(RH,ly+ROI); x0,x1 = max(0,lx-ROI),min(RW,lx+ROI)
                idx = (np.arange(y0,y1)[:,None]*RW + np.arange(x0,x1)[None,:]).reshape(-1)
                Mc = T[next(iter(T))].shape[1]
                q,name,ii,qroi,pk,ph = correlate(R[-Mc:,idx], T)                  # ROI only (fast, track), current word length
                yy,xx = divmod(ii, x1-x0); y,x = y0+yy, x0+xx
                qmap = np.zeros(RH*RW, dtype=np.float32); qmap[idx]=qroi; mode="TRK"
            dt_corr = time.time()-ta
            held=False
            if q >= a.qlock:
                if locked and last_ph is not None:
                    d = ((ph - last_ph + 15) % 31) - 15
                    dn = n - last_n
                    words = dn / (31*fpc)
                    if 0 < dn and words < 8 and abs(d) <= 2:
                        err = d / (dn/fpc)
                        ph_acc = 0.7*ph_acc + 0.3*err
                        if abs(ph_acc) > 0.002:
                            step = max(-0.005, min(0.005, ph_acc))
                            newchip = chip * (1.0 + step)
                            if abs(newchip/a.chip - 1.0) <= 0.08:
                                chip = newchip; fpc = a.fps/chip; T = templates(fpc, int(round(31*fpc)))
                            ph_acc = 0.0
                locked=True; lx,ly = x,y; last_ph=ph; last_n=n; hold=0; good=(x,y,qmap)
            elif locked:
                hold += 1
                if hold <= a.hold:            # HOLD: keep the last good fix, keep tracking the ROI
                    held=True; x,y,qmap = good
                else:                         # sustained loss -> drop to search (full-field acquire next)
                    locked=False; last_ph=None; hold=0
            cx,cy,cep = centroid_cep(qmap.reshape(RH,RW), x, y); az,el = bearing(cx,cy)
            # lit level at the fix (raw 2x2-summed units /4 -> ~8-bit scale) from the ring snapshot
            lit = float(np.percentile(R[:, y*RW+x], 90))/4.0
            new_sh = shutter
            if a.agc and agc_cool == 0:
                # locked: hold the lit level in band. unlocked: open up (dim field) or back off (saturated field)
                if lit < a.agc_lo and shutter < 3900: new_sh = min(3900, shutter*2)
                elif lit > a.agc_hi and shutter > 50: new_sh = max(50, shutter//2)
            if new_sh != shutter:
                shutter = new_sh; p.kill(); p = start(shutter); agc_cool = 3; k = 0; n = 0; next_rep = M_ring; t0 = time.time()
            elif agc_cool > 0: agc_cool -= 1
            if a.json:
                print(json.dumps({"t":round(now-t0,2),"mode":mode,"lock":locked,"code":name,"x":round(cx,1),"y":round(cy,1),
                                  "cep":round(cep,2),"q":round(q,3),"peak":round(pk),"az":round(az,1),"el":round(el,1),
                                  "fps":round(n/(now-t0),1),"chip":round(chip,2),"corr_ms":round(dt_corr*1000),"held":held,"lit":round(lit),"shutter":shutter}), flush=True)
            else:
                print(f"t={now-t0:7.2f} {mode} {'HELD' if held else ('LOCK' if locked else 'search')} code={name} x={cx:6.1f} y={cy:6.1f} cep={cep:4.2f}px q={q:5.3f} peak={pk:7.0f} az={az:6.1f} el={el:6.1f} chip={chip:6.2f} lit={lit:4.0f} sh={shutter} fps={n/(now-t0):5.1f} corr={dt_corr*1000:4.0f}ms", flush=True)
            last=now
    p.wait()

if __name__ == "__main__":
    try:
        main()
    except BrokenPipeError:      # display went away -- exit quietly
        import os, sys; sys.stderr.close(); os._exit(0)
    except KeyboardInterrupt:
        pass
