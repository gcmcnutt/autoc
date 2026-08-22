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
#          dynamics_progress, energy_progress (5).  m2 = those + gen_diag + intercept_analysis +
#          predictor_analysis (038 US3 span/closure-predictor: per-horizon +
#          closure-rate error over gens + latest-gen calibration/regime detail) +
#          gen_runtime (log-only diversity/collapse proxy) + mode_progress (per-gen
#          perception/track/range/reacquire competence) + score_by_path (per-path
#          tracking-score + error distance) + rnn_capacity (needs nnextractor+nn2cpp) (11).
#          With --compare, intercept_analysis overlays the compare run's B/C policy
#          curves + E encounter histogram (A/B) — this replaced the separate tactics.png.
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

# Control cadence (sec/tick) — feeds intercept_analysis's closing-rate m/s AND
# time-denominates the tick-based streak metrics (avgMaxStreak, maxLost →
# seconds) so 20 Hz (50 ms) vs 10 Hz (100 ms) compare.
#
# 041 T009 — read it from the LOG, not the ini. autoc prints its resolved config
# at startup, so the log states the cadence the run actually used; the ini states
# the cadence the NEXT run would use. Re-plotting a historical 10 Hz log with
# today's 20 Hz ini silently halved every duration in the report. The ini is only
# a fallback for pre-config-print logs, and it says so out loud.
CTRL_MSEC="$(grep -oE 'ControlIntervalMsec[ =:]+[0-9]+' "$LOG" | head -1 | grep -oE '[0-9]+$')"
if [[ ! "$CTRL_MSEC" =~ ^[0-9]+$ ]]; then
  CTRL_MSEC="$(grep -E '^[[:space:]]*ControlIntervalMsec' "$INI" | head -1 | sed -E 's/.*=[[:space:]]*([0-9]+).*/\1/')"
  [[ "$CTRL_MSEC" =~ ^[0-9]+$ ]] || CTRL_MSEC=50
  echo "  [warn] $LOG does not state ControlIntervalMsec; falling back to ${INI##*/} = ${CTRL_MSEC}ms." >&2
  echo "         If this log predates the config print, confirm that is its real cadence." >&2
fi
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
PRED_CACHE=()
ENERGY_CACHE=()
if [[ "$USE_CACHE" == 1 ]]; then
  mkdir -p "$CACHE_DIR"
  DYN_CACHE=( --cache "$CACHE_DIR/${SAFE}__dmp${DSIG}_dynamics.csv" )
  PRED_CACHE=( --cache "$CACHE_DIR/${SAFE}__dmp${DSIG}_predictor.csv" )
  ENERGY_CACHE=( --cache "$CACHE_DIR/${SAFE}__dmp${DSIG}_energy.csv" )
fi

run() { echo "  [plot] $1"; python3 "$AN/$@"; }

# --- common (m1 + m2) ---
run plot_evolution_progress.py --focus "$NAME:$LOG" --run-name "$NAME" \
    --crash-log "$LOG" --total-gens "$TOTAL_GENS" --tick-sec "$TICK_SEC" "${COMPARE[@]}" \
    --out "$OUT/${NAME}_evolution_progress.png"
run per_axis_aggressiveness.py "$TICK" --label "$NAME" --gen "$GEN" \
    -o "$OUT/${NAME}_per_axis_aggressiveness.png"
run plot_per_axis_time_series.py "$SUMMARY" --label "$NAME" --total-gens "$TOTAL_GENS" \
    -o "$OUT/${NAME}_per_axis_time_series.png"
# regime_control (041, 2026-08-21) — the per-axis panels above pool every tick,
# which cannot tell "quiet on patrol, works hard tracking" from "thrashes
# uniformly"; same pooled mean, opposite verdicts. Groups pitch+roll as the bank
# pair (they trade geometrically; roll alone changes no trajectory) and keeps
# throttle separate, and measures attitude FRAME-FREE because ~30% of flight is
# knife-edge or inverted where Euler bank angle is meaningless.
run regime_control.py "$TICK" --label "$NAME" --gen "$GEN" \
    -o "$OUT/${NAME}_regime_control.png"
run dynamics_progress.py --run "$RUN" --gens "1-$GEN" --stride "$STRIDE" -i "$INI" \
    "${DYN_CACHE[@]}" --label "$NAME" -o "$OUT/${NAME}_dynamics_progress.png"

# input_investment (041) — per-input first-layer weight norm over gens, plus
# W_hh effective rank over gens and the curriculum ramp on a shared axis.
# Answers "is evolution investing in the inputs we added?" continuously, where
# the T068 ablation answers it definitively but only post-bake for one elite.
# Mode-agnostic (pure weight analysis), so it lives in the COMMON set.
# 2026-08-21 — NOW PER-GEN (stride 1), via an incremental cache. The old
# stride*8 existed because one nnextractor+nn2cpp per gen is ~2.8 s, essentially
# all S3 fetch. That is prohibitive once per RUN but trivial once per NEW GEN,
# which is exactly the trick the run-summary already used. First report on a run
# pays; every later one fetches only what has appeared since.
#
# ⭐ The cache also REPLACES the old --csv into $OUT. That wrote a recomputable
# intermediate into the committed report directory, where it looked like a
# deliverable; no other report does that with its internal metrics.
if [[ -x "$REPO/build/nnextractor" && -x "$REPO/build/nn2cpp" ]]; then
  # ⚠️ ramp-step is read with the same sed idiom as ControlIntervalMsec above,
  # NOT by stripping non-digits from the line. The old awk did
  # gsub(/[^0-9-]/,"",$2) over everything after the '=', so any digit in the
  # trailing COMMENT was concatenated into the value: a dated note turned
  # "0" into "00410-52026-08-18--1". Take the first number after '=' and stop.
  # Cache key mirrors the run-summary's: run-id + dmp-dump build signature, so a
  # tooling change invalidates rather than silently mixing old and new maths.
  INVCACHE="$CACHE_DIR/${SAFE}__dmp${DSIG}_investment.csv"
  run input_investment.py --run "$RUN" --gens "1-$GEN" --stride 1 \
      -i "$INI" --label "$NAME" --total-gens "$TOTAL_GENS" \
      --ramp-step "$(grep -E '^[[:space:]]*VariationRampStep' "$INI" | head -1 | sed -E 's/.*=[[:space:]]*(-?[0-9]+).*/\1/')" \
      --cache "$INVCACHE" --tick-csv "$TICK" \
      -o "$OUT/${NAME}_input_investment.png"
else
  echo "  [plot] input_investment skipped (needs nnextractor + nn2cpp)" >&2
fi

# energy_progress (041 P2-5) — the Es/Ps STATE over generations, not just the
# scalar objective. plot_evolution_progress's energy panel plots one summed
# number per gen, which cannot distinguish "energy improved because the policy
# flies better" from "energy improved because the policy was MUTED" -- and those
# look identical in a scalar. That distinction is AC-1's third part and it is
# what 035 got wrong. Mode-agnostic (Es/Ps are in the shared craft block), so it
# lives in the COMMON set.
#
# ⛔ Post-041 runs only: a pre-P2-4 dmp has no Es_m column and the script exits
# loudly rather than plotting a gap that would read as "no energy destroyed".
run energy_progress.py --run "$RUN" --gens "1-$GEN" --stride "$STRIDE" -i "$INI" \
    "${ENERGY_CACHE[@]}" --label "$NAME" --total-gens "$TOTAL_GENS" \
    -o "$OUT/${NAME}_energy_progress.png"

# --- m2 (tracker) adds: gen_diag, intercept_analysis (A/B-capable), rnn_capacity ---
if [[ "$MODE" == "m2" ]]; then
  run plot_gen_diag.py --in "$LOG" --label "$NAME" --out "$OUT/${NAME}_gen_diag.png"

  # Resolve the FIRST --compare run once (run-id from its log) — shared by the
  # intercept_analysis A/B policy overlay + the rnn_capacity elite overlay.
  # Fetch its per-tick CSV @ this gen for the intercept A/B (this folds in the
  # retired intercept_compare/tactics report — the B/C/E overlay now lives in
  # intercept_analysis itself).
  CNAME=""; CLOG=""; CRUNID=""; INTERCEPT_CMP=()
  if [[ ${#COMPARE[@]} -ge 2 ]]; then
    CSPEC="${COMPARE[1]}"; CNAME="${CSPEC%%:*}"; CLOG="${CSPEC#*:}"
    CRUNID="$(grep -E 'Run ID:' "$CLOG" 2>/dev/null | tail -1 | sed -E 's/.*Run ID:[[:space:]]*//')"
    if [[ -n "$CRUNID" ]] && "$DMP" "s3://$BUCKET/$CRUNID/" --csv-only --gen "$GEN" -i "$INI" >"$TMP/cmp_tick.csv" 2>"$TMP/cmp.err"; then
      INTERCEPT_CMP=( --compare "$CNAME:$TMP/cmp_tick.csv" )
    elif [[ -n "$CRUNID" ]] && "$DMP" "s3://$BUCKET/$CRUNID/" --csv-only -i "$INI" >"$TMP/cmp_tick.csv" 2>"$TMP/cmp2.err"; then
      # Compare run has no gen $GEN (it stopped earlier) — fall back to its
      # LATEST gen so the B/C/E overlay still renders (its final policy).
      echo "  [plot] intercept A/B overlay: compare has no gen $GEN — using its latest gen" >&2
      INTERCEPT_CMP=( --compare "${CNAME}-final:$TMP/cmp_tick.csv" )
    elif [[ -n "$CRUNID" ]]; then
      echo "  [plot] intercept A/B overlay skipped (compare CSV unavailable):" >&2; tail -1 "$TMP/cmp2.err" 2>/dev/null >&2
    fi
  fi
  run intercept_analysis.py --csv "$TICK" --label "$NAME" --gen "$GEN" \
      --tick-sec "$TICK_SEC" "${INTERCEPT_CMP[@]}" -o "$OUT/${NAME}_intercept_analysis.png"

  # predictor_analysis — 038 US3 span/closure-predictor: per-horizon (+50/100/150 ms)
  # + closure-rate prediction error over generations (own gen-sweep cache, like
  # dynamics) plus latest-gen calibration/regime detail from the tick.csv already
  # fetched. Answers "is the aux head learning + does accuracy track depth".
  run predictor_analysis.py --run "$RUN" --gens "1-$GEN" --stride "$STRIDE" -i "$INI" \
      "${PRED_CACHE[@]}" --tick "$TICK" --tick-sec "$TICK_SEC" --label "$NAME" \
      -o "$OUT/${NAME}_predictor_analysis.png"

  # gen_runtime — per-gen wall-clock curve (proxy for population diversity/collapse:
  # rising = learning to fly full scenarios; flat-low/dropping = early-death/collapse).
  # Log-only (no S3); overlays the current run + every --compare run's log.
  GR_ARGS=( --run "$NAME:$LOG" )
  for ((i=1; i<${#COMPARE[@]}; i+=2)); do GR_ARGS+=( --run "${COMPARE[i]}" ); done
  run gen_runtime.py "${GR_ARGS[@]}" -o "$OUT/${NAME}_gen_runtime.png"

  # mode_progress — per-gen mode competence (perception/track/range/reacquire) from #GenDiag;
  # log-only, same current+compares overlay. Plateau = competence ceiling.
  run mode_progress.py "${GR_ARGS[@]}" --tick-sec "$TICK_SEC" -o "$OUT/${NAME}_mode_progress.png"

  # score_by_path — per-path tracking-score distribution (M2 generalization proxy):
  # raw box/strip per path + per-step (path-length-normalized) histograms per path.
  # Needs per-scenario meta (path_variant + score + steps) → one --meta-only fetch.
  if "$DMP" "$RUN" --meta-only -i "$INI" >"$TMP/meta.yaml" 2>"$TMP/meta.err"; then
    # --csv adds the ground-truth per-tick tracking-error-distance panels (uses the tick.csv
    # already fetched); right panels show ‖chase−trail‖ per path instead of per-step score.
    run score_by_path.py --meta "$TMP/meta.yaml" --csv "$TICK" --label "$NAME" -o "$OUT/${NAME}_score_by_path.png"
  else
    echo "  [plot] score_by_path skipped (meta-only fetch failed):" >&2; tail -1 "$TMP/meta.err" >&2
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

fi

echo "generate_pngs: done — wrote to $OUT:"
ls -1 "$OUT/${NAME}"_*.png | sed 's/^/  /'
