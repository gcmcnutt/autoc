#!/usr/bin/env bash
# generate_pngs.sh — standardized per-run report PNGs from a logfile (038 P0-C).
#
#   scripts/generate_pngs.sh <m1|m2> <logfile> [options]
#
# Options:
#   --out DIR            output dir (default: feature dir specs/<NNN>-*/ from the name, else cwd)
#   --compare NAME:LOG   overlay an earlier run on the evolution fitness-over-time chart
#                        (repeatable; passes through to plot_evolution_progress.py --compare)
#   --total-gens N       x-axis span for evolution/time-series plots (default 800)
#   --stride N           dynamics_progress per-gen sampling (default 5)
#   --full-summary       force a full run-summary refetch (ignore the cache)
#   --no-cache           don't read or write the run-summary cache
#
# Derives from the run .log: run-id ("Run ID: ..."), gen (last "#NNGen gen=N"),
# bucket (m1→autoc-m1, m2→autoc-m2), config (m1→autoc.ini, m2→autoc-tracker.ini),
# name (basename minus .log = the plot --label).
#
# Reports: m1 = evolution_progress, per_axis_aggressiveness, per_axis_time_series,
#          dynamics_progress (4).  m2 = those + gen_diag + intercept_analysis +
#          gen_runtime (log-only diversity/collapse proxy) + rnn_capacity (needs
#          nnextractor+nn2cpp) + tactics (needs a --compare run) (9).
#
# INCREMENTAL run-summary: the per-gen aggregate is the slow part (one S3 dmp
# fetch per gen). We cache it per run-id under $CACHE_DIR and use `dmp-dump
# --run-summary --since-gen N` to fetch ONLY new gens, then merge+dedup+sort —
# so re-plotting a still-growing run doesn't re-download earlier generations.
set -euo pipefail

die() { echo "generate_pngs: $*" >&2; exit 1; }

MODE="${1:-}"; LOG="${2:-}"; shift 2 2>/dev/null || true
[[ "$MODE" == "m1" || "$MODE" == "m2" ]] || die "usage: $0 <m1|m2> <logfile> [options]"
[[ -f "$LOG" ]] || die "logfile not found: $LOG"

OUT_ARG=""; COMPARE=(); TOTAL_GENS=800; STRIDE=5; USE_CACHE=1; FORCE_FULL=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --out)         OUT_ARG="$2"; shift 2 ;;
    --compare)     COMPARE+=( --compare "$2" ); shift 2 ;;
    --total-gens)  TOTAL_GENS="$2"; shift 2 ;;
    --stride)      STRIDE="$2"; shift 2 ;;
    --full-summary) FORCE_FULL=1; shift ;;
    --no-cache)    USE_CACHE=0; shift ;;
    *) die "unknown option: $1" ;;
  esac
done

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DMP="$REPO/build/dmp-dump"
AN="$REPO/src/analytics"
CACHE_DIR="${GENERATE_PNGS_CACHE:-/tmp/generate_pngs_cache}"
[[ -x "$DMP" ]] || die "dmp-dump not built at $DMP (cmake --build build --target dmp-dump)"
# dmp-dump build signature (mtime_size): a run-summary math change ⇒ new signature
# ⇒ a fresh cache file, so old-binary rows never splice with new-binary rows.
DSIG="$(stat -c '%Y_%s' "$DMP" 2>/dev/null || echo nostat)"

if [[ "$MODE" == "m1" ]]; then BUCKET=autoc-m1; INI="$REPO/autoc.ini";         else
                               BUCKET=autoc-m2; INI="$REPO/autoc-tracker.ini"; fi
[[ -f "$INI" ]] || die "config not found: $INI"

# Control cadence (sec/tick) from the run's config — feeds intercept_analysis's
# closing-rate m/s so it's correct at 20 Hz (50 ms) vs 10 Hz (100 ms).
CTRL_MSEC="$(grep -E '^[[:space:]]*ControlIntervalMsec' "$INI" | head -1 | sed -E 's/.*=[[:space:]]*([0-9]+).*/\1/')"
[[ "$CTRL_MSEC" =~ ^[0-9]+$ ]] || CTRL_MSEC=50
TICK_SEC="$(awk "BEGIN{printf \"%.4f\", $CTRL_MSEC/1000.0}")"

NAME="$(basename "$LOG" .log)"
RUNID="$(grep -E 'Run ID:' "$LOG" | tail -1 | sed -E 's/.*Run ID:[[:space:]]*//')"
[[ -n "$RUNID" ]] || die "no 'Run ID:' line in $LOG"
GEN="$(grep -E '#NNGen gen=' "$LOG" | tail -1 | grep -oE 'gen=[0-9]+' | cut -d= -f2)"
[[ -n "$GEN" ]] || die "no '#NNGen gen=' line in $LOG"
RUN="s3://$BUCKET/$RUNID/"

if [[ -n "$OUT_ARG" ]]; then OUT="$OUT_ARG"; else
  FNUM="$(echo "$NAME" | grep -oE 'autoc-[0-9]{3}' | grep -oE '[0-9]{3}' || true)"
  OUT="$(ls -d "$REPO"/specs/${FNUM}-*/ 2>/dev/null | head -1 || true)"
  [[ -n "$OUT" ]] || OUT="$PWD"
fi
mkdir -p "$OUT"

echo "generate_pngs: mode=$MODE name=$NAME gen=$GEN bucket=$BUCKET"
echo "  run=$RUN"
echo "  out=$OUT  config=$(basename "$INI")  compare=$(( ${#COMPARE[@]} / 2 ))  tick=${TICK_SEC}s"

TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT
TICK="$TMP/tick.csv"; SUMMARY="$TMP/summary.csv"

echo "  [dmp] per-tick CSV (latest gen) ..."
"$DMP" "$RUN" --csv-only -i "$INI" >"$TICK" 2>"$TMP/tick.err" \
  || { cat "$TMP/tick.err" >&2; die "dmp-dump --csv-only failed"; }

# --- run-summary, incremental against a per-run-id cache ------------------------
SAFE="${RUNID//[:\/]/_}"
# Cache key = run-id + dmp-dump build signature. Run-id (unique seed+timestamp)
# keeps different runs apart; DSIG keeps different dmp-dump builds apart. So we
# never splice gen-N of one run/build with gen-M of another.
CACHE="$CACHE_DIR/${SAFE}__dmp${DSIG}_summary.csv"
full_summary() {
  echo "  [dmp] run-summary FULL (all gens — slow) ..."
  "$DMP" "$RUN" --run-summary -i "$INI" >"$SUMMARY" 2>"$TMP/sum.err" \
    || { cat "$TMP/sum.err" >&2; die "dmp-dump --run-summary failed"; }
}
if [[ "$USE_CACHE" == 1 && "$FORCE_FULL" == 0 && -s "$CACHE" ]]; then
  LAST="$(tail -1 "$CACHE" | cut -d, -f1)"
  if [[ "$LAST" =~ ^[0-9]+$ ]] && [[ "$LAST" -eq "$GEN" ]]; then
    echo "  [dmp] run-summary cache complete @ gen $LAST — no fetch"
    cp "$CACHE" "$SUMMARY"
  elif [[ "$LAST" =~ ^[0-9]+$ ]] && [[ "$LAST" -lt "$GEN" ]]; then
    echo "  [dmp] run-summary incremental since gen $((LAST+1)) (cache @ gen $LAST) ..."
    cp "$CACHE" "$TMP/merged.csv"
    "$DMP" "$RUN" --run-summary --since-gen "$((LAST+1))" -i "$INI" >>"$TMP/merged.csv" 2>"$TMP/sum.err" \
      || { cat "$TMP/sum.err" >&2; die "dmp-dump --run-summary --since-gen failed"; }
    head -1 "$TMP/merged.csv" >"$SUMMARY"                                    # header (from cache)
    tail -n +2 "$TMP/merged.csv" | grep -vE '^gen,' | sort -t, -k1,1n -u >>"$SUMMARY"  # drop stray headers; dedup+numeric-sort by gen
  else
    [[ "$LAST" =~ ^[0-9]+$ ]] && \
      echo "  [cache] cached gen $LAST > log gen $GEN — full refetch (cache ahead/suspect)"
    full_summary
  fi
else
  full_summary
fi
if [[ "$USE_CACHE" == 1 ]]; then mkdir -p "$CACHE_DIR"; cp "$SUMMARY" "$CACHE"; fi

# dynamics_progress fetches one --csv-only dmp per (stride) gen — the other slow
# S3 path. Give it its own per-(run-id + dmp-build) cache so it too only fetches
# new gens on a re-plot.
DYN_CACHE=()
if [[ "$USE_CACHE" == 1 ]]; then
  mkdir -p "$CACHE_DIR"
  DYN_CACHE=( --cache "$CACHE_DIR/${SAFE}__dmp${DSIG}_dynamics.csv" )
fi

run() { echo "  [plot] $1"; python3 "$AN/$@"; }

# --- common (m1 + m2) ---
run plot_evolution_progress.py --focus "$NAME:$LOG" --run-name "$NAME" \
    --crash-log "$LOG" --total-gens "$TOTAL_GENS" "${COMPARE[@]}" \
    --out "$OUT/${NAME}_evolution_progress.png"
run per_axis_aggressiveness.py "$TICK" --label "$NAME" --gen "$GEN" \
    -o "$OUT/${NAME}_per_axis_aggressiveness.png"
run plot_per_axis_time_series.py "$SUMMARY" --label "$NAME" --total-gens "$TOTAL_GENS" \
    -o "$OUT/${NAME}_per_axis_time_series.png"
run dynamics_progress.py --run "$RUN" --gens "1-$GEN" --stride "$STRIDE" -i "$INI" \
    "${DYN_CACHE[@]}" --label "$NAME" -o "$OUT/${NAME}_dynamics_progress.png"

# --- m2 (tracker) adds: gen_diag, intercept_analysis, rnn_capacity, tactics ---
if [[ "$MODE" == "m2" ]]; then
  run plot_gen_diag.py --in "$LOG" --label "$NAME" --out "$OUT/${NAME}_gen_diag.png"
  run intercept_analysis.py --csv "$TICK" --label "$NAME" --gen "$GEN" \
      --tick-sec "$TICK_SEC" -o "$OUT/${NAME}_intercept_analysis.png"

  # gen_runtime — per-gen wall-clock curve (proxy for population diversity/collapse:
  # rising = learning to fly full scenarios; flat-low/dropping = early-death/collapse).
  # Log-only (no S3); overlays the current run + every --compare run's log.
  GR_ARGS=( --run "$NAME:$LOG" )
  for ((i=1; i<${#COMPARE[@]}; i+=2)); do GR_ARGS+=( --run "${COMPARE[i]}" ); done
  run gen_runtime.py "${GR_ARGS[@]}" -o "$OUT/${NAME}_gen_runtime.png"

  # Resolve the FIRST --compare run once (run-id from its log) — shared by the
  # rnn_capacity overlay (its elite NN) + the tactics overlay (its per-tick CSV).
  CNAME=""; CLOG=""; CRUNID=""
  if [[ ${#COMPARE[@]} -ge 2 ]]; then
    CSPEC="${COMPARE[1]}"; CNAME="${CSPEC%%:*}"; CLOG="${CSPEC#*:}"
    CRUNID="$(grep -E 'Run ID:' "$CLOG" 2>/dev/null | tail -1 | sed -E 's/.*Run ID:[[:space:]]*//')"
  fi

  # rnn_capacity — W_hh utilization (pure weight SVD; extract elite via
  # nnextractor→nn2cpp, no sim/sensor data). Overlays the --compare run's elite
  # too when given, so the report shows the A/B (eff-rank/spectrum) not just one run.
  if [[ -x "$REPO/build/nnextractor" && -x "$REPO/build/nn2cpp" ]]; then
    extract_nn() {  # <run-id> <out.cpp> ; returns 0 on success
      "$REPO/build/nnextractor" -k "$1" -o "$TMP/$2.dat" -i "$INI" >"$TMP/$2.err" 2>&1 \
        && "$REPO/build/nn2cpp" -i "$TMP/$2.dat" -o "$TMP/$2.cpp" >/dev/null 2>&1
    }
    if extract_nn "$RUNID" elite; then
      CAP_ARGS=( --nn "$NAME:$TMP/elite.cpp" )
      if [[ -n "$CRUNID" ]] && extract_nn "$CRUNID" cmp_elite; then
        CAP_ARGS=( --nn "$CNAME:$TMP/cmp_elite.cpp" "${CAP_ARGS[@]}" )  # baseline first
      fi
      run rnn_capacity.py "${CAP_ARGS[@]}" -o "$OUT/${NAME}_rnn_capacity.png"
    else
      echo "  [nn ] rnn_capacity skipped (nnextractor/nn2cpp failed):" >&2; tail -2 "$TMP/elite.err" >&2
    fi
  else
    echo "  [nn ] rnn_capacity skipped (build nnextractor + nn2cpp to enable)" >&2
  fi

  # tactics A/B — overlay this run vs the --compare run on the policy/encounter axes.
  if [[ -n "$CRUNID" ]] \
     && "$DMP" "s3://$BUCKET/$CRUNID/" --csv-only --gen "$GEN" -i "$INI" >"$TMP/cmp_tick.csv" 2>"$TMP/cmp.err"; then
    run intercept_compare.py --a "$CNAME:$TMP/cmp_tick.csv" --b "$NAME:$TICK" --gen "$GEN" \
        -o "$OUT/${NAME}_tactics.png"
  elif [[ -n "$CRUNID" ]]; then
    echo "  [plot] tactics skipped (compare run CSV @gen $GEN unavailable):" >&2; tail -1 "$TMP/cmp.err" 2>/dev/null >&2
  fi
fi

echo "generate_pngs: done — wrote to $OUT:"
ls -1 "$OUT/${NAME}"_*.png | sed 's/^/  /'
