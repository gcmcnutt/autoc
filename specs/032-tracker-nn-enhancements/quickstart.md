# 032 Quickstart — operator protocol for phase 1 bake + attribution follow-on

How to bake phase 1 of 032 against the postdiag2 reference, read out the result, and route to either close-out or phase-1b attribution per spec Q7.

> **Audience**: operator running training bakes. Assumes 030 baseline tooling (rebuild-perf.sh, autoc-eval pipeline, S3 dmp upload) is in place and that `autoc-tracker.ini` is the active config path.

---

## Pre-bake — verify build + determinism

```bash
# From repo root.
bash scripts/rebuild-perf.sh
```

Expectations:
- Build succeeds (autoc + crrcsim, optimized).
- Regression gate passes: per-gen elite fitness numbers match the recorded eval-vs-training reference (`scripts/rebuild-perf.sh` documents the exact pass criterion).
- M1 (pathgen) fitness numbers MUST be bitwise-equal to the pre-032 baseline. Any deviation here is a phase 1 implementation bug — DO NOT proceed to bake.

If the regression gate fails on M1, that's the highest-priority bug to fix before bake. M2 numbers will change by design (input vector grew); operator does NOT run the bitwise gate on M2 outputs.

---

## Bake — phase 1 combined run

The combined-bake mode (A + B together, per spec Q1) is the default — the code IS the change, no runtime toggle.

```bash
# Inspect the config to confirm:
grep -A 3 '\[DerivedFeatures\]' autoc-tracker.ini
# Expected:
#   [DerivedFeatures]
#   CepGateThreshold                = 1.25
```

Launch the bake:

```bash
stdbuf -oL -eL ./autoc -i autoc-tracker.ini 2>&1 | tee bake.log
```

> `stdbuf -oL -eL` gives line-buffered logs per [reference_autoc_launch_command](../../.claude/projects/-home-gmcnutt-autoc/memory/reference_autoc_launch_command.md).

Watch:
- Gen 0 should print TrackerInput count = 54 in startup banner (or run a one-shot `autoc --print-config` if the implementation lands one).
- `#GenDiag` lines appear per gen; `avgInRamp` is the metric of interest.
- Bake to at least 322 generations. Operator may bake longer if elite fitness slope is still positive at 322; the plateau-read protocol (R8) does not require a hard stop.

---

## Read result — plateau-avgInRamp

Per spec Q6 + research.md R8:

```bash
# Extract the last 50 gens of avgInRamp from the bake log.
grep '^#GenDiag' bake.log | tail -50 | awk '{ ... extract avgInRamp column ... }' | awk '{ s += $1; n++ } END { print "plateau-avgInRamp =", s/n }'
```

(Exact awk column index depends on the `#GenDiag` line format — see `src/autoc.cc` for the current emission order. Plan a small helper script under `specs/032-tracker-nn-enhancements/` if needed; `scripts/` directory is for cross-version utilities only per [feedback_scripts_dir_scope](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_scripts_dir_scope.md).)

---

## Outcome decision — spec Q7

| `plateau-avgInRamp` | Decision | Next step |
|---|---|---|
| **≥ 0.15** | Phase 1 SUCCESS | Close-out: SMOKE_REPORT.md + outcome.md commit; operator decides next milestone (031 hardware bring-up, M3 perception research, or 030-related follow-ups) |
| **0.10 ≤ x < 0.15** | Phase 1 PARTIAL | Run B-off attribution bake (below) |
| **< 0.10** | Phase 1 MISS | Route to phase 2 (FOV-blindness / lost-sight patrol) or M3-grade perception. Document in outcome.md what was tried and what the avgInRamp landed at |

---

## Attribution bake — B-off mode (only if partial-band result)

Per research.md R7: in sim, the "A-only" mode is equivalent to the existing 030 baseline (postdiag2's ~0.07 avgInRamp), so a separate A-only re-bake adds no new signal. Only the **B-off bake** is run as an attribution test, and it runs from a **git-reverted pre-032 commit** — no runtime toggle exists for "disable derived features" because greenfield M2 means the code IS the change.

```bash
# Capture the current commit so you can return to it.
git rev-parse HEAD > /tmp/032-head.txt

# Check out the last pre-032 commit (the 030 closeout — adjust SHA if needed):
git checkout 8bfad02

# Build the same way as for the combined bake:
bash scripts/rebuild-perf.sh

# Bake with the same source dmp, pop, gens, seed as the combined bake. This
# is the "without derived features" attribution point because the pre-032
# code knows nothing of slots 45..53.
stdbuf -oL -eL ./autoc -i autoc-tracker.ini 2>&1 | tee bake_b_off.log

# When done, return to 032:
git checkout "$(cat /tmp/032-head.txt)"
bash scripts/rebuild-perf.sh
```

Read plateau-avgInRamp the same way. Decision:

| Combined plateau (032) | B-off plateau (pre-032) | Interpretation |
|---|---|---|
| 0.10–0.15 | < 0.10 | Derived features (B) are load-bearing — partial signal is real; consider refining B (e.g., add tilt-rate, span EMA, etc.) before escalating to phase 2 |
| 0.10–0.15 | ~ combined plateau | Identity-stable ordering (A) carried most of the lift; but since A is a sim no-op (research.md R1), this would imply the combined result is mostly noise. Document, escalate to phase 2 |
| 0.10–0.15 | between 030 baseline and combined | Both contribute; partial. Document the split, decide phase-2 routing based on which sub-feature looks more refinable |

---

## Closeout artifacts

Regardless of outcome, the phase 1 closeout includes:

1. **`specs/032-tracker-nn-enhancements/outcome.md`** — operator-written summary: combined plateau-avgInRamp, B-off plateau-avgInRamp (if run), decision, next milestone routing.
2. **dmp honesty audit** — per spec Q8 + research.md R5: confirm the dmp output captures all 54 NN inputs + 3 outputs. If any gap, fix before tagging the bake's dmp as a phase-1 reference artifact. (See [feedback_honest_dmp_recording](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_honest_dmp_recording.md).)
3. **Constitution VI grep audit** — `/speckit.implement` closeout reports the grep result over `src/eval/`, `src/nn/`, `include/autoc/eval/`, `include/autoc/nn/` for unannotated raw `float`/`double` hits in 032's diff.
4. **Bake archive** — `bake.log` (and `bake_b_off.log` if run) preserved alongside the closeout dmp; `.png` charts (evolution_progress, gen_diag, per_axis_*) generated via the existing `specs/030-tracker-mode/` plotting scripts (parameterize on bake name).

---

## Pre-flight check — DO NOT run a no-op bake

Before launching the long bake, validate end-to-end that the new input slots are actually being populated:

```bash
# Run a short smoke (e.g., 2-3 gens, pop = small) and inspect the dmp:
./autoc -i autoc-tracker.ini --gens=3 --pop=200
# Or whatever the existing smoke-config mechanism is.

# Then inspect the dmp:
./tools/inspect_dmp <smoke-output>.dmp | grep -E 'spn|dspn|tlt'
# Expected: 9 new fields appear in the per-tick records, with non-trivial values.
```

If the new slots are zero-filled across all ticks, the gather pipeline didn't pick up the new code — abort and debug. The 322-gen bake is too expensive to waste on a no-op.

## Determinism sanity (one-time per phase-1 build)

Run the bake at small scale twice with the same seed; verify identical per-gen `#GenDiag` lines. Per [project_v15_determinism_candidates](../../.claude/projects/-home-gmcnutt-autoc/memory/project_v15_determinism_candidates.md), determinism issues in 030 v1.5 have a known suspect (scenarioSequence-seeded crash-hull PRNG); confirm phase-1 hasn't regressed.
