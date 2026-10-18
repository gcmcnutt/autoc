#!/usr/bin/env python3
"""
set_eval_scale.py — copy an NN01 weights file with its stored variation_scale replaced (043 T122, 2026-10-18).

Eval mode flies a genome at the variation_scale stored in its weights file (autoc.cc: gEvalVariationScaleOverride =
genome.variation_scale) — the ramp value of the generation it was saved at. So genomes from different generations are, by default,
evaluated in DIFFERENT environments (gen 370 at 9/19, gen 752 at 18/19). To compare candidates on equal footing, write eval-only
copies with the same scale (normally 1.0 = the full envelope). The weights themselves are untouched; never fly the copy (the xiao
ignores the field anyway, but keep provenance clean).

NN01 layout (src/nn/serialization.cc): "NN01" | u32 version | u32 num_layers | u32[num_layers] topology | u8[num_layers] recurrent |
u32 num_weights | f32[num_weights] | f64 fitness | u32 generation | f32 mutation_sigma | f32 variation_scale | u32 len | source.

Usage: set_eval_scale.py in.dat out.dat [--scale 1.0]
"""
import argparse, struct, sys


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("inp"); ap.add_argument("out"); ap.add_argument("--scale", type=float, default=1.0)
    a = ap.parse_args()
    b = bytearray(open(a.inp, "rb").read())
    assert b[:4] == b"NN01", "not an NN01 weights file"
    off = 8
    (nl,) = struct.unpack_from("<I", b, off); off += 4
    off += 4 * nl + nl
    (nw,) = struct.unpack_from("<I", b, off); off += 4 + 4 * nw
    fit, gen, sig, scale = struct.unpack_from("<dIff", b, off)
    struct.pack_into("<f", b, off + 8 + 4 + 4, a.scale)
    open(a.out, "wb").write(bytes(b))
    print(f"{a.inp}: gen {gen}, fitness {fit:.3f}, stored variation_scale {scale:.4f} -> {a.scale:.4f}  => {a.out}")


if __name__ == "__main__":
    sys.exit(main())
