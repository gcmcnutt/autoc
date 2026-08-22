#!/usr/bin/env python3
"""candidate_census.py — T072: measure K, the candidate-detection count per frame.

K has never been measured, because `acquire_pass()` caps its RETURN at 3 seeds by policy — so the
underlying detection count is invisible from outside. K is the number that decides whether the O(K²)
proto-track stage in T050 is viable at all (compute-budget.md: needs K ≲ 1000–2000; every pixel is 64 000).

This replicates acquire_pass's statistic exactly rather than inventing one: reduce4 to the coarse plane,
temporal-difference against the previous pass, and count cells whose |diff| clears **4× the field's mean
|diff|** — the same self-normalising threshold, which is why it moves with ambient light.

It is also the right instrument for comparing two bench conditions. First-lock time is high-variance
(measured 2026-08-22: medians 2.95 vs 3.80 s but Mann-Whitney U=9 against a critical 2 — indistinguishable
at n=5), whereas K gives one sample per frame, i.e. thousands per clip.

    candidate_census.py <clip.bcnr> [--every N] [--stride S]
"""
import argparse, struct, sys
import numpy as np

MAGIC=0x42434E52; HDR=52; FRAME_HDR=40

def read_header(f):
    b=f.read(HDR); magic,ver,hdr_b,w,h=struct.unpack_from("<IHHHH",b,0)
    if magic!=MAGIC: raise SystemExit("not BCNR")
    if ver!=1: raise SystemExit(f"container v{ver}, this tool implements 1 — refusing")
    if hdr_b!=HDR: f.seek(hdr_b)
    return w,h

def frames(f,w,h,every):
    npx=w*h; i=0
    while True:
        head=f.read(FRAME_HDR)
        if len(head)<FRAME_HDR: return
        rec_b,_s,t,_e,_g,_fl=struct.unpack_from("<IIQIHH",head,0)
        pay=f.read(rec_b-FRAME_HDR)
        if len(pay)<npx: return
        if i%every==0: yield np.frombuffer(pay[:npx],np.uint8).reshape(h,w)
        i+=1

def main():
    ap=argparse.ArgumentParser(); ap.add_argument("clip")
    ap.add_argument("--every",type=int,default=4)
    ap.add_argument("--stride",type=int,default=3,help="pass separation in sampled frames (acquire throttles ~1/25)")
    a=ap.parse_args()
    with open(a.clip,"rb") as f:
        w,h=read_header(f)
        prev=None; Ks=[]; hist=[]
        for k,img in enumerate(frames(f,w,h,a.every)):
            # reduce4, as acquire_pass does
            p=img[:h//4*4,:w//4*4].astype(np.int32)
            r4=p.reshape(h//4,4,w//4,4).sum(axis=(1,3))
            if prev is not None and k%a.stride==0:
                d=np.abs(r4-prev)
                thr=4.0*d.mean()                      # the same self-normalising gate
                K=int((d>thr).sum())
                Ks.append(K)
                if len(hist)<3: hist.append((float(d.mean()),float(thr),K))
            if k%a.stride==0: prev=r4
    Ks=np.array(Ks)
    print(f"{a.clip}")
    print(f"  passes analysed {len(Ks)}   coarse plane {h//4}x{w//4} = {h//4*w//4} cells")
    print(f"  K per pass: min {Ks.min()}  p50 {int(np.percentile(Ks,50))}  p90 {int(np.percentile(Ks,90))}  max {Ks.max()}  mean {Ks.mean():.1f}")
    print(f"  as a fraction of the plane: p50 {100*np.percentile(Ks,50)/(h//4*w//4):.2f}%")
    print(f"  O(K^2) proto-track pairs at p50: {int(np.percentile(Ks,50))**2:,}  (budget needs <~1-4M)")

if __name__=="__main__": main()
