#!/usr/bin/env python3
"""Block-average a high-rate blackbox CSV down by factor N (boxcar anti-alias), keeping
numeric columns averaged and flag/string columns taken from the block's first row.
Output has the same column names so pitch_spectrum.py runs unchanged."""
import csv, sys
src, dst, n = sys.argv[1], sys.argv[2], int(sys.argv[3])
with open(src) as fh, open(dst, "w", newline="") as out:
    rd = csv.reader(fh)
    hdr = [h.strip() for h in next(rd)]
    wr = csv.writer(out); wr.writerow(hdr)
    block = []
    for row in rd:
        block.append(row)
        if len(block) == n:
            o = []
            for j, h in enumerate(hdr):
                vals = [r[j] for r in block]
                try:
                    o.append("%.4f" % (sum(float(v) for v in vals) / n))
                except ValueError:
                    o.append(vals[0])
            # keep flags/mode from the first row verbatim (averaging would corrupt them)
            for j, h in enumerate(hdr):
                if h in ("mspOverrideFlags", "flightModeFlags (flags)", "stateFlags (flags)",
                         "failsafePhase (flags)", "navState", "navFlags", "loopIteration"):
                    o[j] = block[0][j]
            wr.writerow(o); block = []
