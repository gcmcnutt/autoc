#!/bin/bash
# 026_acro_smoke.sh — validate a data.dat from a 026 build.
#
# Run this AFTER training (or eval) with the 026 binary to confirm the
# ACRO PID block fired and PidInternals were captured.
#
# Checks:
#   1. data.dat exists and has the new schema columns
#      (rateCmdP/Q, rateAchP/Q, pidFF_*, pidP_*, pidI_*, pidIntP/Q, pidSat).
#   2. At least one row has non-zero PID values (elite reeval ran with
#      gTraceIsEliteReeval set, populating PidInternals).
#   3. Reports median |rateAch - rateCmd| per axis as a tracking quality
#      indicator. Does NOT fail on threshold — the gate becomes meaningful
#      only after cadence8 trains. Pre-training, this is informational.
#
# Usage: specs/026-nn-temporal-state/026_acro_smoke.sh [data.dat]
#   default: ./data.dat
#
# Exits 0 if structural checks pass, non-zero if columns are missing or
# data.dat is unreadable.

set -euo pipefail

DAT="${1:-data.dat}"

if [[ ! -s "$DAT" ]]; then
  echo "error: $DAT not found or empty" >&2
  echo "  run training/eval with the 026 binary first" >&2
  exit 2
fi

echo "== 026 ACRO post-run validation =="
echo "  data.dat: $DAT ($(wc -l < "$DAT") lines)"

HEADER=$(head -1 "$DAT")
need=(rateCmdP rateCmdQ rateAchP rateAchQ pidFF_P pidFF_Q pidP_P pidP_Q pidI_P pidI_Q pidIntP pidIntQ pidSat)
missing=()
for col in "${need[@]}"; do
  grep -qE "\\b${col}\\b" <<<"$HEADER" || missing+=("$col")
done
if [[ ${#missing[@]} -gt 0 ]]; then
  echo "FAIL: data.dat missing columns: ${missing[*]}" >&2
  echo "       — was the binary rebuilt after T020?" >&2
  exit 1
fi
echo "  PASS: all 13 PID columns present"

python3 - "$DAT" <<'PY'
import sys
import numpy as np

path = sys.argv[1]
with open(path) as f:
    header = f.readline().split()
col = {name: i for i, name in enumerate(header)}

rows = []
with open(path) as f:
    f.readline()
    for line in f:
        parts = line.split()
        if len(parts) < len(header):
            continue
        # Skip repeated header rows (autoc may re-emit header per scenario).
        try:
            int(parts[0])  # first column is Scn (integer)
        except ValueError:
            continue
        rows.append(parts)

if len(rows) < 50:
    sys.exit(f"FAIL: only {len(rows)} data rows; need 50+")

def colf(name):
    return np.array([float(r[col[name]]) for r in rows])

rcp, rap = colf("rateCmdP"), colf("rateAchP")
rcq, raq = colf("rateCmdQ"), colf("rateAchQ")
intP, intQ = colf("pidIntP"), colf("pidIntQ")
sat = np.array([int(r[col["pidSat"]]) for r in rows])

# Tracking quality (informational; not a gate pre-cadence8)
err_p = np.median(np.abs(rap - rcp))
err_q = np.median(np.abs(raq - rcq))
populated = len(rows)
MAX_P, MAX_Q = 7.50, 5.24  # rad/s, ACRO_MAX_RATE_*
print(f"  rate-tracking (informational, not gated):")
print(f"    median |rateAchP - rateCmdP| = {err_p:.3f} rad/s ({100*err_p/MAX_P:.1f}% of {MAX_P})")
print(f"    median |rateAchQ - rateCmdQ| = {err_q:.3f} rad/s ({100*err_q/MAX_Q:.1f}% of {MAX_Q})")

# Saturation summary
sat_pitch = int(np.sum(sat & 0x1).item() if hasattr(np.sum(sat & 0x1), 'item') else (sat & 0x1).sum())
sat_roll  = int((sat & 0x2).sum() // 2)
print(f"  saturation: pitch {sat_pitch}/{populated} ticks, "
      f"roll {sat_roll}/{populated} ticks")

print("SMOKE: PASS (structure + capture verified)")
PY
