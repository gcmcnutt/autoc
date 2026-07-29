# Quickstart — 040 Camera Redo

**Date**: 2026-07-28 | **Plan**: [plan.md](plan.md)

How to build, verify, run, and later calibrate the refreshed perception model.

> Not to be confused with `camera-hardware-phase/quickstart.md`, which is the parked beacon-pod /
> recorder build walkthrough for a different era. See [README.md](README.md).

---

## (a) Build and test

Ordinary source edits — incremental:

```bash
cd /home/gmcnutt/autoc/build && make -j
ctest --output-on-failure -R run_autoc_tests
```

**Any `CMakeLists.txt` change** (new test target, new source file) — clean rebuild, **not** an incremental
reconfigure, per Principle IV:

```bash
bash scripts/rebuild-perf.sh     # PERFORMANCE_BUILD, single-threaded, FP-deterministic
```

Correctness gate before committing (Principle II):

```bash
bash scripts/rebuild.sh          # compile + full test suite
```

**Type-domain audit — run at every milestone close** (Principle VI). Each hit must be annotated
`// raw-ok: <reason>` or converted:

```bash
grep -nE '\b(float|double)\b' src/eval/ src/nn/ include/autoc/eval/ include/autoc/nn/ \
  | grep -v -E '// raw-ok:'
```

---

## (b) Verify the perception model

Test files carrying the contract:

| File | Verifies |
|---|---|
| `beacon_projection_tests.cc` | pixel quantisation, axis isotropy, separation invariance, obstruction geometry |
| `signal_model_tests.cc` *(new)* | monotonic falloff with range, CDMA penalty, emission flat-top |
| `acquisition_state_tests.cc` *(new)* | acquisition timing, hold ride-through, determinism, scenario reset |
| `camera_variation_tests.cc` *(new)* | reproducibility from scenario id, zero-sigma bit-identity |
| `gather_tracker_inputs_tests.cc` | input assembly, quality regimes, 58-input stability |
| `contract_tracker_config_tests.cc` | ini key count + new signal keys |
| `tracker_dmp_roundtrip_tests.cc` | diagnostic field round-trip |

**Key properties to assert** (from the spec's success criteria):

- a fixed angular separation reads within **2%** of the same value anywhere in frame, at any orientation
  (SC-001)
- range inferred from separation matches truth within the pixel grid's resolution, no systematic bias
  (SC-002) — this is what catches a regression of the 0.772 m correction
- bearing reaches the design detection range while separation-derived range goes **explicitly unavailable**
  below the resolving limit (SC-011)
- identical scenario ⇒ bit-identical results; zero variation ⇒ bit-identical to baseline (SC-006)

---

## (c) Determinism check

Bit-determinism is a project gate, not a nicety. After any perception change:

```bash
bash scripts/rebuild-perf.sh
# then: eval a pinned elite and confirm the fitness matches the training-side value exactly
```

An `NN_EVAL_SAME` result is the pass condition. A mismatch means state is leaking across scenarios
(FR-020a) or a non-deterministic term entered the signal path (FR-020) — check the acquisition state
reset first, in **both** the production tick and the test-only reference.

---

## (d) Throughput benchmark (FR-037/038)

Measure **total evaluation throughput** against the **prior M2 run** — not a micro-benchmark of the
perception function, which is not the dominant per-tick term.

```bash
# baseline: prior M2 run's generations-per-hour from its logfile
# candidate: same ini, same population, same scenario count, new perception
```

Ceiling is **≤10% regression** (SC-013). A breach is an explicit accept-or-optimise decision, not
something to absorb silently — throughput converts directly into generations reached.

---

## (e) Retrain and evaluate

**Pre-run build gate** (Principle IX) — perception changes are determinism-affecting:

```bash
bash scripts/rebuild-perf.sh
ctest --output-on-failure -R run_autoc_tests
```

Launch **only** via the detached script (never a harness-tracked background task — that reaping killed
runs t4 and t5 mid-flight with no error in the log):

```bash
bash scripts/train.sh autoc-tracker.ini logs/autoc-040-t1-perception.log
```

Naming follows the artifact convention: `autoc-<feature>-t<N>-<details>`, lexicographically ordered.

Then evaluate on paths the controller never trained against — repoint `autoc-eval-tracker.ini` at a novel
M1 eval source and compare on the established comparators. Report the **aggregate delta** (SC-008): the
question is *are we in the right room, and is this more honest?* — not which term cost what. **A
competence drop attributable to more honest perception is a valid outcome**, and no floor gates the
feature.

Artifact lifecycle (Principle VIII): dumps upload tagged `retain=expire`. If the controller becomes a
baseline, pin it `retain=keep` and record the S3 prefix in the outcome document.

---

## (f) Calibrating later — the plumbing-first payoff

This is what the feature is really for (FR-036). When real-to-simulation measurement arrives:

1. Open [contracts/config-surface.md](contracts/config-surface.md) and find every value classified **A**
   (assumed).
2. Replace it with the measured figure and reclassify to **M**, recording the basis.
3. Rebuild and re-run. **No structural change should be required.**

If step 3 demands a code change, the plumbing-first contract has been broken — that is the feature's
central failure condition, and SC-012 is the rehearsal that proves it in advance by substituting plausible
alternatives for every assumed value.

**Values expected to change first**: entrance pupil and exposure (need article 1), ambient floor and
optics gain (need field measurement), flux constant (pick between the measured 1.1 and 1.6), propeller
attenuation (needs a bench measurement or a mount that puts the propeller back in frame).
