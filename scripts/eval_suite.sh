#!/bin/bash
# eval_suite.sh — Automated regression eval for trained NN weights
#
# Usage: ./scripts/eval_suite.sh <weights_file> [tier] [master_seed]
#   weights_file: NN01 weight file from `nnextractor -g <gen>` (NOT a gen*.dmp --
#                 runNNEvaluation calls nn_detect_format and exit(1)s on a dmp)
#   tier: 0 (repro), 1 (quick), 2 (generalization), 3 (stress), all (default)
#   master_seed: REQUIRED for a meaningful tier 0. The bake's "Effective master
#                 seed" from its run log; without it tier 0 draws a novel table
#                 and cannot reproduce the stored fitness.
#
# IMPORTANT: autoc-eval.ini is the single source of truth for all sigmas/params.
# It MUST match autoc.ini training config exactly. Tier overrides only change
# path type, dimensions, and seed — never sigmas.
#
# Results stored in: eval-results/<timestamp>/
#   ├── config.txt          # combined config used
#   ├── tier0-repro/        # exact seed repro — must match stored fitness bitwise
#   ├── tier1-aeroStandard/ # 6 paths × 49 winds = 294 flights, novel seed (matches the bake)
#   ├── tier2-progressive/  # 1 path × 49 winds
#   ├── tier2-long/         # 1 path × 49 winds
#   ├── tier2-random/       # 12 paths × 12 winds = 144 flights
#   ├── tier3-stress/       # 12 paths × 12 winds at 120% sigmas
#   ├── tier3-quiet/        # 1 path × 1 wind, 12 m/s, no variation
#   └── summary.txt         # pass/fail + aggregate stats

set -euo pipefail

AUTOC=./build/autoc
BASE_INI=autoc.ini
EVAL_BASE_INI=autoc-eval.ini
RESULTS_ROOT=eval-results

usage() {
    echo "Usage: $0 <weights_file> [tier] [master_seed]"
    echo "  weights_file: NN01 file from nnextractor (NOT a gen*.dmp)"
    echo "  tier: 0 (repro), 1, 2, 3, all (default: all)"
    echo "  master_seed: the bake master seed — required for a real tier 0"
    exit 1
}

[[ $# -lt 1 ]] && usage
WEIGHTS_FILE="$1"
TIER="${2:-all}"
MASTER_SEED="${3:-}"

[[ ! -f "$WEIGHTS_FILE" ]] && { echo "Error: weights file not found: $WEIGHTS_FILE"; exit 1; }
[[ ! -x "$AUTOC" ]] && { echo "Error: $AUTOC not found or not executable"; exit 1; }

# Create results directory
TIMESTAMP=$(date -u +%Y-%m-%dT%H:%M:%SZ)
RESULTS_DIR="${RESULTS_ROOT}/${TIMESTAMP}"
mkdir -p "$RESULTS_DIR"

# Copy weights to results for reproducibility
cp "$WEIGHTS_FILE" "$RESULTS_DIR/weights.dmp"
echo "Weights: $WEIGHTS_FILE" > "$RESULTS_DIR/config.txt"
echo "Timestamp: $TIMESTAMP" >> "$RESULTS_DIR/config.txt"
echo "Tier: $TIER" >> "$RESULTS_DIR/config.txt"
echo "" >> "$RESULTS_DIR/config.txt"

# Generate a complete eval INI by copying the base eval config and patching keys.
# This ensures all required settings (NN topology, slew rates, etc.) are present.
# Args: output_file key=value pairs to override...
make_eval_ini() {
    local outfile="$1"; shift
    # Start from complete eval base
    cp "$EVAL_BASE_INI" "$outfile"
    # Patch the weights path
    sed -i "s|^NNWeightFile.*|NNWeightFile = ${RESULTS_DIR}/weights.dmp|" "$outfile"
    # Apply scenario-specific overrides (last write wins for duplicate keys)
    for kv in "$@"; do
        local key="${kv%%=*}"
        key="${key%% }"  # trim trailing space
        local val="${kv#*=}"
        # If key exists, replace it; otherwise append
        if grep -q "^${key} " "$outfile" 2>/dev/null; then
            sed -i "s|^${key} .*|${kv}|" "$outfile"
        else
            echo "$kv" >> "$outfile"
        fi
    done
    echo "# Auto-patched by eval_suite.sh — $TIMESTAMP" >> "$outfile"
}

# Run a single eval scenario
# Args: tier_name overlay_ini
run_eval() {
    local name="$1"
    local overlay="$2"
    local outdir="${RESULTS_DIR}/${name}"
    mkdir -p "$outdir"

    echo "=== Running: ${name} ==="
    cp "$overlay" "$outdir/overlay.ini"

    # Run autoc in eval mode, capture log
    # TODO: once config overlay (-i -i) is implemented, use:
    #   $AUTOC -i $BASE_INI -i "$overlay" > "$outdir/console.log" 2>&1
    # For now: use standalone eval ini with overlay values manually merged
    $AUTOC -i "$overlay" > "$outdir/console.log" 2>&1 || true

    # Archive volatile data files
    [[ -f eval-data.dat ]] && mv eval-data.dat "$outdir/data.dat"
    [[ -f eval-data.stc ]] && mv eval-data.stc "$outdir/data.stc"

    # Extract S3 key for renderer visualization
    local s3_key
    s3_key=$(grep "S3 upload:" "$outdir/console.log" 2>/dev/null | head -1 | awk '{print $NF}' | sed -E 's|/gen[0-9]+\.dmp(\.zst)?$||')

    # Extract summary stats from log
    local ok_count=0 crash_count=0 total=0
    ok_count=$(grep -c " OK " "$outdir/console.log" 2>/dev/null || true)
    crash_count=$(grep -c " CRASH " "$outdir/console.log" 2>/dev/null || true)
    ok_count=${ok_count:-0}
    crash_count=${crash_count:-0}
    total=$((ok_count + crash_count))

    # Extract aggregate metrics from per-scenario log lines
    # Format: [N] OK score=-1234.56 maxStrk=42 strkSteps=180 maxMult=3.2
    local avg_score avg_maxStrk avg_strkSteps avg_maxMult
    eval_stats=$(grep -E "\] (OK|CRASH)" "$outdir/console.log" | awk '{
        for(i=1;i<=NF;i++) {
            if($i~/^score=/) sc+=substr($i,7)+0;
            if($i~/^maxStrk=/) ms+=substr($i,9)+0;
            if($i~/^strkSteps=/) ss+=substr($i,11)+0;
            if($i~/^maxMult=/) mm+=substr($i,9)+0;
        }
        n++
    } END {
        if(n>0) printf "%.1f %.1f %.1f %.2f", sc/n, ms/n, ss/n, mm/n;
        else printf "0 0 0 0"
    }')

    read avg_score avg_maxStrk avg_strkSteps avg_maxMult <<< "$eval_stats"

    # Determine pass/fail
    local completion_pct=0
    [[ $total -gt 0 ]] && completion_pct=$(echo "scale=1; $ok_count * 100 / $total" | bc)

    local result="PASS"
    # Tier-specific pass criteria
    # score is negative (lower = better), maxStrk is in ticks (higher = better)
    case "$name" in
        tier1-*) [[ $(echo "$completion_pct < 95" | bc -l) -eq 1 ]] && result="FAIL" ;;
        tier2-*|tier3-stress*) [[ $(echo "$completion_pct < 95" | bc -l) -eq 1 ]] && result="FAIL" ;;
        tier3-quiet*) [[ $crash_count -gt 0 ]] && result="FAIL" ;;
    esac

    # Write per-test summary
    cat > "$outdir/summary.txt" << SUMMARY
Test: $name
Result: $result
Flights: $total  OK: $ok_count  CRASH: $crash_count  Completion: ${completion_pct}%
Avg score: $avg_score  maxStrk: $avg_maxStrk  strkSteps: $avg_strkSteps  maxMult: $avg_maxMult
SUMMARY
    cat "$outdir/summary.txt"
    echo ""

    # Append to overall summary
    printf "%-25s %s  %3d/%3d (%5.1f%%)  score=%7.1f  strk=%5.1f  mult=%4.2f\n" \
        "$name" "$result" "$ok_count" "$total" "$completion_pct" \
        "$avg_score" "$avg_maxStrk" "$avg_maxMult" >> "$RESULTS_DIR/summary.txt"
    if [[ -n "$s3_key" ]]; then
        echo "  build/renderer -i $overlay -k ${s3_key}/" >> "$RESULTS_DIR/summary.txt"
    fi
}

# ============================================================================
# TIER 0: Reproducibility — exact match against stored training fitness
# ============================================================================
run_tier0() {
    echo ""
    echo "=============================="
    echo "  TIER 0: Reproducibility"
    echo "=============================="

    # autoc-eval.ini mirrors autoc.ini exactly, so the ONLY thing tier 0 must
    # pin is the scenario table — i.e. the bake's master seed. Without it the
    # base Seed=-1 draws a NOVEL table and "repro" compares two different
    # scenario sets, which can never match bitwise.
    if [[ -n "$MASTER_SEED" ]]; then
        make_eval_ini "$RESULTS_DIR/tier0-repro.ini" "Seed = $MASTER_SEED"
    else
        echo "  [warn] no master_seed given — tier 0 will run a NOVEL scenario table" >&2
        echo "         and CANNOT reproduce the stored fitness. Pass the bake's" >&2
        echo "         \"Effective master seed\" as arg 3 for a real determinism gate." >&2
        make_eval_ini "$RESULTS_DIR/tier0-repro.ini"
    fi

    local outdir="${RESULTS_DIR}/tier0-repro"
    mkdir -p "$outdir"
    echo "=== Running: tier0-repro ==="
    cp "$RESULTS_DIR/tier0-repro.ini" "$outdir/overlay.ini"
    $AUTOC -i "$RESULTS_DIR/tier0-repro.ini" > "$outdir/console.log" 2>&1 || true

    # Extract S3 key for renderer visualization
    local s3_key
    s3_key=$(grep "S3 upload:" "$outdir/console.log" 2>/dev/null | head -1 | awk '{print $NF}' | sed -E 's|/gen[0-9]+\.dmp(\.zst)?$||')

    # Extract eval vs stored fitness (035 FR-P05 format: autoc emits
    # "NN_EVAL_SAME: fitness=X" on a bitwise match, else
    # "NN_EVAL_DIFFERENT: eval=X stored=Y"). The bitwise match IS the gate.
    local eval_fitness="" stored_fitness="" result="FAIL"
    # `|| true` — these lines are absent when there's no stored fitness to
    # compare (e.g. tier0 on an extracted nn_weights.dat, not a gen*.dmp);
    # without the guard, grep's exit 1 trips `set -e` and the script dies.
    local same_line diff_line
    same_line=$(grep "NN_EVAL_SAME:" "$outdir/console.log" 2>/dev/null | head -1 || true)
    diff_line=$(grep "NN_EVAL_DIFFERENT:" "$outdir/console.log" 2>/dev/null | head -1 || true)
    if [[ -n "$same_line" ]]; then
        eval_fitness=$(echo "$same_line" | sed -n 's/.*fitness=\(-\?[0-9.]*\).*/\1/p')
        stored_fitness="$eval_fitness"
        result="PASS"
    elif [[ -n "$diff_line" ]]; then
        eval_fitness=$(echo "$diff_line" | sed -n 's/.*eval=\(-\?[0-9.]*\).*/\1/p')
        stored_fitness=$(echo "$diff_line" | sed -n 's/.*stored=\(-\?[0-9.]*\).*/\1/p')
        result="FAIL"
    fi

    cat > "$outdir/summary.txt" << SUMMARY
Test: tier0-repro
Result: $result
Eval fitness:   $eval_fitness
Stored fitness: $stored_fitness
Match: $([ "$result" = "PASS" ] && echo "EXACT" || echo "MISMATCH")
SUMMARY
    cat "$outdir/summary.txt"
    echo ""

    local ok_count=0 crash_count=0 total=0
    ok_count=$(grep -c " OK " "$outdir/console.log" 2>/dev/null || true)
    crash_count=$(grep -c " CRASH " "$outdir/console.log" 2>/dev/null || true)
    ok_count=${ok_count:-0}
    crash_count=${crash_count:-0}
    total=$((ok_count + crash_count))
    local completion_pct=0
    [[ $total -gt 0 ]] && completion_pct=$(echo "scale=1; $ok_count * 100 / $total" | bc)

    local avg_score avg_maxStrk avg_maxMult
    eval_stats=$(grep -E "\] (OK|CRASH)" "$outdir/console.log" 2>/dev/null | awk '{
        for(i=1;i<=NF;i++) {
            if($i~/^score=/) sc+=substr($i,7)+0;
            if($i~/^maxStrk=/) ms+=substr($i,9)+0;
            if($i~/^maxMult=/) mm+=substr($i,9)+0;
        }
        n++
    } END {
        if(n>0) printf "%.1f %.1f %.2f", sc/n, ms/n, mm/n;
        else printf "0 0 0"
    }' || echo "0 0 0")
    read avg_score avg_maxStrk avg_maxMult <<< "$eval_stats"

    printf "%-25s %s  %3d/%3d (%5.1f%%)  score=%7.1f  strk=%5.1f  mult=%4.2f  fitness=%s\n" \
        "tier0-repro" "$result" "$ok_count" "$total" "$completion_pct" \
        "$avg_score" "$avg_maxStrk" "$avg_maxMult" "$eval_fitness" >> "$RESULTS_DIR/summary.txt"
    if [[ -n "$s3_key" ]]; then
        echo "  build/renderer -i $RESULTS_DIR/tier0-repro.ini -k ${s3_key}/" >> "$RESULTS_DIR/summary.txt"
    fi
}

# ============================================================================
# TIER 1: Quick validation — same geometry as training, novel seed
# ============================================================================
run_tier1() {
    echo ""
    echo "=============================="
    echo "  TIER 1: Quick Validation"
    echo "=============================="

    # Same geometry and sigmas as training, but novel random seed.
    # 6 paths × 49 winds = 294 flights, matching the bake.
    # All sigmas come from autoc-eval.ini base (which mirrors autoc.ini).
    #
    # ⛔ PathGeneratorMethod is STATED, not inherited. It used to be inherited,
    # and when the base ini was repurposed for M2 novel-path work it silently
    # became `random` — so "tier1-aeroStandard / same geometry as training" was
    # in fact testing the un-trained patrol/intercept start for weeks, and read
    # as a catastrophic failure that was really a config drift.
    make_eval_ini "$RESULTS_DIR/tier1-aero.ini" \
        "PathGeneratorMethod = aeroStandard" \
        "Seed = -1"

    run_eval "tier1-aeroStandard" "$RESULTS_DIR/tier1-aero.ini"
}

# ============================================================================
# TIER 2: Generalization — novel paths never seen in training
# ============================================================================
run_tier2() {
    echo ""
    echo "=============================="
    echo "  TIER 2: Generalization"
    echo "=============================="

    # All sigmas inherited from autoc-eval.ini base (matches training).
    # Only override: path type, dimensions, and novel seed.

    # Progressive distance (novel geometry)
    make_eval_ini "$RESULTS_DIR/tier2-prog.ini" \
        "PathGeneratorMethod = progressiveDistance" \
        "SimNumPathsPerGeneration = 1" \
        "Seed = -1"
    run_eval "tier2-progressive" "$RESULTS_DIR/tier2-prog.ini"

    # Long sequential (duration/stability)
    make_eval_ini "$RESULTS_DIR/tier2-long.ini" \
        "PathGeneratorMethod = longSequential" \
        "SimNumPathsPerGeneration = 1" \
        "Seed = -1"
    run_eval "tier2-long" "$RESULTS_DIR/tier2-long.ini"

    # Random 12×12 (full generalization)
    make_eval_ini "$RESULTS_DIR/tier2-random.ini" \
        "PathGeneratorMethod = random" \
        "SimNumPathsPerGeneration = 12" \
        "WindScenarios = 12" \
        "RandomPathSeedB = 99999" \
        "Seed = -1"
    run_eval "tier2-random" "$RESULTS_DIR/tier2-random.ini"
}

# ============================================================================
# TIER 3: Stress / envelope
# ============================================================================
run_tier3() {
    echo ""
    echo "=============================="
    echo "  TIER 3: Stress / Envelope"
    echo "=============================="

    # Random at 120% of *037* training ENV sigmas (craft/servo left at training
    # values per operator 2026-06-15 — this tier stresses the env envelope only).
    # 037 training: ConeSigma=18, RollSigma=30, SpeedSigma=0.06, WindDirSigma=45,
    #               RabbitSpeedSigma=2.0, Position R/Alt=0 (disabled).
    # 120%:         Cone=21.6, Roll=36, Speed=0.072, WindDir=54, Rabbit=2.4.
    make_eval_ini "$RESULTS_DIR/tier3-stress.ini" \
        "PathGeneratorMethod = random" \
        "SimNumPathsPerGeneration = 12" \
        "WindScenarios = 12" \
        "RandomPathSeedB = 99999" \
        "Seed = -1" \
        "EntryConeSigma = 21.6" \
        "EntryRollSigma = 36" \
        "EntrySpeedSigma = 0.072" \
        "WindDirectionSigma = 54" \
        "RabbitSpeedSigma = 2.4"
    run_eval "tier3-stress" "$RESULTS_DIR/tier3-stress.ini"

    # Quiet throttle baseline — no wind, no variation, constant 12 m/s rabbit
    make_eval_ini "$RESULTS_DIR/tier3-quiet.ini" \
        "PathGeneratorMethod = longSequential" \
        "SimNumPathsPerGeneration = 1" \
        "WindScenarios = 1" \
        "Seed = 42" \
        "EnableEntryVariations = 0" \
        "EnableWindVariations = 0" \
        "EnableRabbitSpeedVariations = 0" \
        "RabbitSpeedNominal = 12.0"
    run_eval "tier3-quiet" "$RESULTS_DIR/tier3-quiet.ini"
}

# ============================================================================
# Main
# ============================================================================
echo "============================================"
echo "  autoc Eval Suite — $TIMESTAMP"
echo "  Weights: $WEIGHTS_FILE"
echo "  Tier: $TIER"
echo "============================================"

echo "" > "$RESULTS_DIR/summary.txt"
printf "%-25s %s  %7s  %11s  %7s  %5s  %4s\n" \
    "Test" "Pass" "OK/Total" "Completion" "Score" "Strk" "Mult" >> "$RESULTS_DIR/summary.txt"
echo "------------------------------------------------------------------------------------" >> "$RESULTS_DIR/summary.txt"

case "$TIER" in
    0)   run_tier0 ;;
    1)   run_tier1 ;;
    2)   run_tier2 ;;
    3)   run_tier3 ;;
    all) run_tier0; run_tier1; run_tier2; run_tier3 ;;
    *)   usage ;;
esac

echo ""
echo "============================================"
echo "  RESULTS SUMMARY"
echo "============================================"
cat "$RESULTS_DIR/summary.txt"
echo ""
echo "Full results: $RESULTS_DIR/"
