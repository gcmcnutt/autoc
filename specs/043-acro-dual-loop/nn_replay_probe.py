#!/usr/bin/env python3
"""
nn_replay_probe.py — OPEN-LOOP REPLAY OF A GENOME ON ITS OWN RECORDED INPUTS, WITH COUNTERFACTUALS (043 t6, 2026-10-15)

The linear surrogate (pitch_input_attribution.py) says the 1.5–4 Hz pitch command is computed from the target-direction
slots at 0/50/100 ms, barely from the gyro. A linear fit of a tanh RNN is only a hint; this replays the actual network
(weights from the nn2cpp .cpp, forward pass per src/nn/evaluator.cc: h_t = tanh(W_xh x + W_hh h_{t-1} + b), hidden state
= the recurrent layer's output) over the recorded per-tick inputs of a dmp CSV, and reports:

  0. fidelity: replayed out_pt vs recorded out_pt (R²) — the replay must reproduce the record before anything else counts
  1. no-memory: hidden state forced to 0 every tick — how much of the output (and of its 1.5–4 Hz band) needs the RNN
  2. frozen-lead: target slots at 50/100/200/400/800 ms replaced by the 0 ms value (no history at all) — what the
     history slots contribute to the band
  3. no-gyro: gyro slots zeroed — what the instantaneous rate contributes
  4. autonomous: inputs held at their per-scenario mean — does the recurrent layer oscillate on its own (spectral
     radius ≈ 3 would allow it), and at what frequency

Band shares are of the replayed pitch output's variance in the 1.5–4 Hz band, relative to the baseline replay.

Usage: nn_replay_probe.py --nn genome.cpp tick.csv [--scenarios N] [--band 1.5 4]
"""
import argparse, csv, re, sys
import numpy as np
from collections import defaultdict

FS = 20.0
# 044 FR-016 — the input block is located BY NAME, never by a literal width (a 45 here
# silently read the wrong columns once the layout grew to 53).
_CRAFT_DISPLAY = {
    "closing_rate": "dd/dt", "quat_w": "qw", "quat_x": "qx", "quat_y": "qy", "quat_z": "qz",
    "airspeed": "vel", "gyro_p": "gyrP", "gyro_q": "gyrQ", "gyro_r": "gyrR",
    "accel_x": "acX", "accel_y": "acY", "accel_z": "acZ", "specific_energy": "Es",
    "boundary_closure_rate": "bClR", "dist_to_boundary": "dBnd",
    "inward_body_x": "inX", "inward_body_y": "inY", "inward_body_z": "inZ",
    "score_grad_x": "sgX", "score_grad_y": "sgY", "score_grad_z": "sgZ",
}


def display_name(col):
    """Decoded-flight column (lowercased slot enum name) -> the dmp-dump display name, so sim and
    flight CSVs index the same way: target_x_tm5 -> tgX-5, target_x_now -> tgX0, dist_tm1 -> ds-1,
    gyro_q -> gyrQ, gyro_q_tm2 -> gyrQ-2 (044 history slots)."""
    m = re.fullmatch(r"target_([xyz])_(?:tm(\d)|now)", col)
    if m:
        return f"tg{m.group(1).upper()}" + (f"-{m.group(2)}" if m.group(2) else "0")
    m = re.fullmatch(r"dist_(?:tm(\d)|now)", col)
    if m:
        return f"ds-{m.group(1)}" if m.group(1) else "ds0"
    m = re.fullmatch(r"(.+)_tm(\d)", col)
    if m and m.group(1) in _CRAFT_DISPLAY:
        return f"{_CRAFT_DISPLAY[m.group(1)]}-{m.group(2)}"
    return _CRAFT_DISPLAY[col]


def input_block(h):
    """The NN input columns of a tick CSV, BY NAME: a dmp-dump sim CSV writes them from the metadata
    table between `rbHhd` and `Es_m`; a decoded flight CSV names them from the log header between
    `tick_counter` and `out_pitch`. Returns (i0, i1, display_names) — names in dmp-dump spelling."""
    if "tick_counter" in h and "out_pitch" in h:
        i0, i1 = h.index("tick_counter") + 1, h.index("out_pitch")
        return i0, i1, [display_name(c) for c in h[i0:i1]]
    i0, i1 = h.index("rbHhd") + 1, h.index("Es_m")
    return i0, i1, list(h[i0:i1])


def parse_nn_cpp(path):
    txt = open(path).read()
    topo = [int(x) for x in re.findall(r"-?\d+", re.search(r"nn_topology\[\]\s*=\s*\{([^}]*)\}", txt).group(1))]
    rec = [t.strip() == "true" for t in re.search(r"nn_recurrent\[\]\s*=\s*\{([^}]*)\}", txt).group(1).split(",") if t.strip()]
    w = np.array([float(x) for x in re.findall(r"-?\d+\.?\d*(?:[eE][-+]?\d+)?", re.search(r"nn_weights\[\d+\]\s*=\s*\{([^}]*)\}", txt, re.S).group(1))], np.float32)
    return topo, rec, w


class Net:
    def __init__(self, topo, rec, w):
        self.topo, self.rec = topo, rec
        self.layers = []; off = 0
        for l in range(len(topo) - 1):
            i, o = topo[l], topo[l + 1]
            W = w[off:off + i * o].reshape(o, i); off += i * o
            B = w[off:off + o]; off += o
            self.layers.append([W, B, None])
        for l in range(len(topo)):
            if l < len(rec) and rec[l]:
                s = topo[l]; self.layers[l - 1][2] = w[off:off + s * s].reshape(s, s); off += s * s
        assert off == len(w), (off, len(w))

    def run(self, X, keep_memory=True, h0=None):
        h = {l: (np.zeros(self.topo[l + 1], np.float32) if h0 is None else h0.copy()) for l, (W, B, Whh) in enumerate(self.layers) if Whh is not None}
        out = np.zeros((len(X), self.topo[-1]), np.float32); H = []
        for t in range(len(X)):
            x = X[t]
            for l, (W, B, Whh) in enumerate(self.layers):
                pre = W @ x + B
                if Whh is not None:
                    pre = pre + Whh @ h[l]
                x = np.tanh(pre)
                if Whh is not None:
                    h[l] = x if keep_memory else np.zeros_like(x)
                    H.append(x.copy())
            out[t] = x
        return out, np.array(H)


def band_var(y, lo, hi):
    Y = np.fft.rfft(y - y.mean()); f = np.fft.rfftfreq(len(y), 1 / FS)
    m = (f >= lo) & (f <= hi); return float((np.abs(Y[m]) ** 2).sum() / len(y) ** 2 * 2)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("csv"); ap.add_argument("--nn", required=True); ap.add_argument("--scenarios", type=int, default=24)
    ap.add_argument("--band", nargs=2, type=float, default=[1.5, 4.0]); a = ap.parse_args(); lo, hi = a.band
    net = Net(*parse_nn_cpp(a.nn))
    r = csv.reader(open(a.csv)); h = next(r)
    i0, i1, names = input_block(h); iS, iP = h.index("scenario"), h.index("out_pt")
    sc = defaultdict(list)
    for row in r:
        if len(row) < len(h): continue
        s = int(row[iS])
        if s % max(1, 294 // a.scenarios) == 0 and len(sc) <= a.scenarios:
            sc[s].append([float(row[iP])] + [float(v) for v in row[i0:i1]])
    tg = [j for j, n in enumerate(names) if n.startswith(("tgX", "tgY", "tgZ", "ds"))]
    tg_now = {n[:-1] if n.endswith("0") else n.split("-")[0]: j for j, n in enumerate(names) if n in ("tgX0", "tgY0", "tgZ0", "ds0")}
    gy = [names.index(n) for n in ("gyrP", "gyrQ", "gyrR")]
    res = defaultdict(list); auto_peaks = []
    for s, rows in sc.items():
        A = np.array(rows, np.float32); y_rec, X = A[:, 0], A[:, 1:]
        if len(X) < 100: continue
        base, H = net.run(X)
        yb = base[:, 0]
        res["fidelity R²"].append(1 - ((yb - y_rec) ** 2).mean() / y_rec.var())
        vb = band_var(yb, lo, hi); vt = yb.var()
        nomem, _ = net.run(X, keep_memory=False)
        res["no-memory: R² vs baseline"].append(1 - ((nomem[:, 0] - yb) ** 2).mean() / vt)
        res["no-memory: band var ratio"].append(band_var(nomem[:, 0], lo, hi) / vb)
        Xf = X.copy()
        for j, n in enumerate(names):
            if n.startswith(("tgX", "tgY", "tgZ", "ds")) and not n.endswith("0") or n in ("tgX-5", "tgY-5", "tgZ-5", "ds-5"):
                key = n.split("-")[0]; Xf[:, j] = X[:, tg_now[key]]
        froz, _ = net.run(Xf)
        res["frozen-history: R² vs baseline"].append(1 - ((froz[:, 0] - yb) ** 2).mean() / vt)
        res["frozen-history: band var ratio"].append(band_var(froz[:, 0], lo, hi) / vb)
        Xg = X.copy(); Xg[:, gy] = 0
        nog, _ = net.run(Xg)
        res["no-gyro: R² vs baseline"].append(1 - ((nog[:, 0] - yb) ** 2).mean() / vt)
        res["no-gyro: band var ratio"].append(band_var(nog[:, 0], lo, hi) / vb)
        Xc = np.repeat(X.mean(0, keepdims=True), 400, axis=0)
        auto, Ha = net.run(Xc, h0=H[len(H) // 2])
        ya = auto[100:, 0]
        res["autonomous: pitch-out std (const inputs)"].append(float(ya.std()))
        if ya.std() > 1e-3:
            Y = np.abs(np.fft.rfft(ya - ya.mean())); f = np.fft.rfftfreq(len(ya), 1 / FS); auto_peaks.append(float(f[1:][Y[1:].argmax()]))
    print(f"replay of {len(res['fidelity R²'])} scenarios, band {lo}-{hi} Hz (medians [p10, p90])")
    for k, v in res.items():
        v = np.array(v); print(f"  {k:42s} {np.median(v):7.3f}  [{np.percentile(v,10):.3f}, {np.percentile(v,90):.3f}]")
    if auto_peaks:
        print(f"  autonomous oscillation in {len(auto_peaks)} scenarios, peak Hz median {np.median(auto_peaks):.2f} [{np.percentile(auto_peaks,10):.2f}, {np.percentile(auto_peaks,90):.2f}]")
    else:
        print("  autonomous: hidden state settles — no self-sustained oscillation with constant inputs")


if __name__ == "__main__":
    sys.exit(main())
