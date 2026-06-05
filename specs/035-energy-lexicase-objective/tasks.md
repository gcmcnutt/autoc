---
description: "Task list for 035 — Energy as a Lexicase Secondary Objective"
---

# Tasks: Energy as a Lexicase Secondary Objective

**Input**: Design documents from `/specs/035-energy-lexicase-objective/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: INCLUDED — the project is Testing-First (Constitution I) and the spec explicitly requires
re-enabling the Selection027 suite (FR-004 / SC-004) plus round-trip / reproducibility gates.

**Organization**: One user story (US1 — energy investigation). The large **pre-work** is the
Foundational phase, ending in the operator-run **verification GATE** that must be green before US1.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependency on an incomplete task)
- **[Story]**: US1 for energy-investigation tasks; Setup/Foundational/Polish carry no story label

## Path Conventions
Single project: `src/`, `tools/`, `tests/`, `include/autoc/` at repo root.

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: dependency + new build targets so later tasks compile.

- [ ] T001 Add libzstd to top-level `CMakeLists.txt` (`find_library`/`find_package` for zstd) and link it into `autoc_common` so crrcsim `mod_inputdev` inherits it transitively (Principle IV; R4).
- [ ] T002 [P] Register new build targets in `CMakeLists.txt`: `tools/dmp_dump.cc` → `dmp-dump` binary; `src/util/s3_run_selector.cc` → `autoc_common`; new test files (`tests/s3_run_selector_tests.cc`, `tests/dmp_dump_tests.cc`, `tests/energy_metric_tests.cc`).

---

## Phase 2: Foundational (Pre-work — BLOCKS US1)

**⚠️ CRITICAL**: US1 (energy) cannot begin until the GATE (T025–T028) is green.

### A1 — Shared S3 selector + uniform run-id naming (FR-P07 / FR-P07b)

- [ ] T003 [P] Write `tests/s3_run_selector_tests.cc` (MUST fail first): `extractGenNumber` inverts `10000−N` for `.dmp` and `.dmp.zst`, rejects malformed keys; latest-run / latest-gen over a synthetic key list; uniform-prefix matching (legacy `tracker-` key NOT matched). Per `contracts/s3-selector.md`.
- [ ] T004 Implement `src/util/s3_run_selector.{h,cc}`: `extractGenNumber` (inverts encoding, `.zst`-aware), `findLatestRun(bucket)`, `findLatestGenKey(...)` — bucket-relative, prefix `"autoc-"`, fail-loud on empty.
- [ ] T005 Route `tools/nnextractor.cc` (≈47, ≈105) through `s3_run_selector`; delete its local `extractGenNumber` + `SetPrefix("autoc-")` duplication.
- [ ] T006 Route `tools/renderer.cc` (≈1778, ≈1898) through `s3_run_selector`; **fix the `extractGenNumber` invert bug** by deleting the local (non-inverting) copy.
- [ ] T007 Collapse `autoc::runIdPrefixForMode` in `include/autoc/util/run_id.h` to uniform `"autoc-"` for ALL modes (retire the `tracker-` branch); update the header comment and `tests/run_id_prefix_tests.cc` (FR-P07b).

### A2 — data.dat retirement + dmp schema + zstd + dmp-dump (FR-P01–P06, P09)

- [ ] T008 [P] Write a zstd round-trip unit test (in `tests/dmp_dump_tests.cc`, MUST fail first): compress→decompress a cereal blob is byte-identical; assert + report ratio (R4 spike-as-test).
- [ ] T009 Add zstd compress (level 19) at the cereal-write boundary in `src/autoc.cc` (both PutObject sites ≈1448 eval, ≈1640 per-gen); key suffix becomes `gen<N>.dmp.zst`.
- [ ] T010 Add zstd inflate on read in `src/eval/source_dmp_loader.cc` (≈71): `.zst` → `ZSTD_decompress`, legacy `.dmp` → passthrough; fail-loud on corrupt/missing (Principle V/VII).
- [ ] T011 Add `.zst` inflate to the dmp fetch paths in `tools/nnextractor.cc` and `tools/renderer.cc` (via the shared selector + decompress).
- [ ] T012 Remove ALL `data.dat` writer plumbing from `src/autoc.cc`: `fout` ofstream (≈2114–2117), `logEvalResults` + `logEvalResultsScenarioTracker` (≈738–884, ≈989–1052), `strOutFile` open, pathgen+tracker header emission. Clean cut, no dual-write (FR-P05, Constitution III).
- [ ] T013 [P] Update `.gitignore` (`*.dat` rule), the `include/autoc/eval/eval_logger.h` comment, and any spec/doc reference to `data.dat` as a live artifact (FR-P06).
- [ ] T014 Implement `tools/dmp_dump.cc` per `contracts/dmp-dump-cli.md`: S3 URI (primary) + local input, `.zst` auto-inflate, YAML metadata block + CSV per-tick body, recompute derived columns (`dhome/dist/along/stpPt/mult/rampSc`, `hull` in tracker mode) via shared `autoc_common` math; fail-loud on load error (FR-P01/P02).
- [ ] T015 Write `tests/dmp_dump_tests.cc` (column-parity + format): derived columns match the retired-writer math on a fixture dmp; YAML/CSV blocks segregated; fail-loud on missing key.
- [ ] T016 Repoint `specs/03[2-5]*/*.py` plot scripts **and `specs/029-no-future-arch/plot_per_axis_time_series.py`** (the gate consumer + FR-005 per-axis energy comparator) from `data.dat` to `dmp-dump` CSV/YAML (subprocess/pipe, new column names — no byte-compat with legacy format) (FR-P03).
- [ ] T017 Update the `scripts/` rebuild-perf gate documentation: the bit-replay gate compares **per-scenario `ScenarioScore` bytes**, not whole-dmp (dmps carry non-deterministic provenance timestamps) (FR-P04 / R3).

### A3 — Per-mode buckets + tagging + lifecycle + ini (FR-P08, P10–P12)

- [ ] T018 Set object `Tagging: retain=expire` on both PutObject sites in `src/autoc.cc` (≈1448, ≈1640) (FR-P10). *(Sequence after T009 — same code region.)*
- [ ] T019 [P] Add `S3Bucket` (per-mode) + `LexicaseEpsilonMode` keys to the `AUTOC_CONFIG_FIELDS(X)` macro in `include/autoc/util/config.h` + `src/util/config.cc`; the 034 config-dump test (T028-era) covers them automatically (FR-P08 decl; FR-003 decl).
- [ ] T020 [P] Add a repo note documenting the `autoc-pin` manual one-liner (`aws s3api put-object-tagging … retain=keep`) and the pinned-prefix-in-outcome-doc rule (FR-P12, Principle VIII). `contracts/lifecycle-policy.json` already committed.
- [ ] T021 **[Admin prereq — non-code, tracked]** Create buckets `autoc-m1` / `autoc-m2` / `autoc-eval`; grant `s3:PutObjectTagging` + `s3:GetObjectTagging` to IAM user `autoc-generator`; apply `contracts/lifecycle-policy.json` to each (+ legacy `autoc-storage`); re-tag the milestone runs below `retain=keep` BEFORE lifecycle goes live. Gates the bucket-cutover only — code/GATE can run on old buckets first (R6).

  **Milestone-pin list (must NOT expire — from the 2026-06-02 admin retention pass):**
  | S3 prefix | run | why keep |
  |---|---|---|
  | `autoc-9223370259105171692-2026-05-02…` | pastonly3 (029 M1) | first M2 pass; flew 5/3 & 5/17; source for 032-phase1 |
  | `autoc-9223370257807536859-2026-05-17…` | 032-phase1 (M2) | big tracker milestone; baked from pastonly3; **035 M2 baseline (FR-005b)** |
  | `autoc-9223370256935631488-2026-05-27…` | 033-pop8000-wind36-r1 (M1) | live `autoc-tracker.ini` source (gen9432.dmp) |
  | `autoc-9223370259246861370-2026-05-01…` | pastonly2 (029 M1) | 029 baseline, gen 800 |
  | `autoc-9223370259505713290-2026-04-28…` | gen-800 orphan | no reference found; kept provisionally, may be tossed |
- [ ] T022 Flip `S3Bucket` per ini to `autoc-m1` / `autoc-m2` / `autoc-eval` (`autoc.ini`, `autoc-basic-m1.ini`, `autoc-tracker.ini`, `autoc-eval.ini`, `autoc-eval-tracker.ini`, `autoc-eval-visual.ini`) per `contracts/ini-config.md` — config-only, after T021.

### A4 — Correctness fixes / verifications (FR-P13, P14)

- [ ] T023 [P] Repoint `TrackerSourceRun` in `autoc-eval-tracker.ini:42` to the live M2 source (the `autoc-tracker.ini` source, in `autoc-m2` post-migration); confirm the loader fails loud on a missing key (FR-P13). **End-to-end check:** run `autoc-eval-tracker.ini` once so the repointed source actually loads (GATE-3 runs `autoc-tracker.ini`, a different file, so FR-P13 is otherwise unexercised by the gate) (closes analyze C2).
- [ ] T024 [P] Verify eval Bug 3 (rabbitSpeedConfig set, `src/autoc.cc:1248-1252`) and Bug 2 (genome.fitness overwrite, `:1370`/`:1425`) still hold — already fixed in 034; covered by the basic-eval GATE, no code change expected (FR-P14).

### ✅ VERIFICATION GATE (operator-run — quickstart §GATE)

- [ ] T025 GATE-1 — basic M1 train (`autoc-basic-m1.ini`): zero `data.dat` written; dmps land as `.dmp.zst` under `autoc-m1`; each object tagged `retain=expire`.
- [ ] T026 GATE-2 — basic eval equivalent: clean `scripts/rebuild-perf.sh`, then eval reproduces training **per-scenario `ScenarioScore` bytes** (`NN_EVAL_SAME`); confirms rabbit-speed (Bug 3) on the configured profile.
- [ ] T027 GATE-3 — basic M2 (`autoc-tracker.ini`): `TrackerSourceRun` resolves (no dangling fail); dmps land under `autoc-m2` with identical naming to M1.
- [ ] T028 GATE-4 — `dmp-dump` reads dmps: YAML+CSV parse; a plot script consumes the CSV with no `data.dat`; `.zst` auto-inflates and a legacy `.dmp` still reads.

**Checkpoint**: GATE green ⇒ US1 may begin. (This is the user's load-bearing pre-condition.)

---

## Phase 3: User Story 1 — Energy as a lexicase secondary objective (Priority: P1) 🎯 MVP

**Goal**: Determine whether energy works as a real lexicase secondary objective (not a scalar
penalty) for both M1 and M2 — energy materially improves without collapsing tracking, or a
documented failure mode.

**Independent Test**: Run an energy-lexicase bake to convergence per mode and compare against the
mode's tracking-only baseline on (a) tracking quality (per-scenario score + avgMaxStreak — no
material regression) and (b) `energy_score` (material improvement). A clear yes or a clear no
(documented) both satisfy the story.

### Tests for User Story 1 (write FIRST, ensure they FAIL)

- [ ] T029 [P] [US1] Write `tests/energy_metric_tests.cc`: convex `energy_score = Σ((out_th+1)/2)²` is ≥0 and monotone-increasing in throttle; matches data-model/R1 formula on hand-computed fixtures.
- [ ] T030 [P] [US1] Re-enable the 4 `DISABLED_` Selection027 tests in `tests/selection_tests.cc` (`:157` EnergyBreaksTrackingTie, `:180` TradeoffBothSurvive, `:219` StabilityBreaksTrackingTie, `:239` ThreeWayTradeoffAllSurvive); update assertions for the active energy axis (FR-004 / SC-004).
- [ ] T031 [P] [US1] Add a MAD-epsilon test to `tests/selection_tests.cc`: MAD-relative pass/fail behavior, and `LexicaseEpsilonMode=constant` reproduces the fixed-0.5 path bit-for-bit (FR-003 / SC-003).

### Implementation for User Story 1

- [ ] T032 [US1] Change `energy_score` computation in `src/eval/fitness_decomposition.cc:174-181` to the convex throttle integral `Σ((out_th+1)/2)²` (`gp_fitness`), replacing the linear sign-wrong placeholder (FR-001b / R1).
- [ ] T033 [US1] Activate the energy lexicase axis in `src/eval/selection.cc:69` (uncomment, gen-0 unconditional pool entry); keep `stability_score` (`:68`) commented OFF (FR-001 / FR-002 trivially satisfied — energy is throttle-separate; FR-008 OFF).
- [ ] T034 [US1] Implement MAD-relative epsilon in `src/eval/selection.cc` (per-axis MAD over surviving candidates) gated by `LexicaseEpsilonMode`; preserve the constant-0.5 path exactly (FR-003 / R2). *(After T031 turns green.)*
- [ ] T035 [US1] Run the Principle-VI type-domain grep on the touched `src/eval/` paths; annotate `// raw-ok:` or convert any raw `float`/`double` in the energy/MAD math.

### Bakes & verdict (operator-run)

- [ ] T036 [US1] M1 energy bake (`autoc.ini` as-is = pop=5000/wind=49, the 034-validated config that tracks pastonly3 — NOT 8000/36, which throttle-pegs; `LexicaseEpsilonMode=mad`, energy axis on); budget 2–3 bakes for the basin lottery (watch throttle-σ=0.000 stuck tell); compare vs the M1 tracking-only baseline using the **per-axis aggressiveness distributions** (`plot_per_axis_time_series.py` → `per_axis_aggressiveness` PNGs) — variation/config-stable comparator that isolates energy from the MAD-ε change; expect movement toward goal on **all 3 controls (pitch/roll/throttle)** (FR-005).
- [ ] T037 [US1] M2 energy bake (`autoc-tracker.ini`, same objective: on-point + min energy); budget 2–3 bakes; compare vs the **pinned M2 tracking-only baseline = 032-phase1** (`autoc-9223370257807536859-2026-05-17…`) or a fresher confirmed non-stuck climber (M2 is lottery-prone — stalls / dead neurons — so the baseline MUST be a confirmed climber); use the same per-axis comparator; capture `#GenCrash hullStrike=N` escalation (seeds the future hull-crash feature) (FR-005b).
- [ ] T038 [US1] Write the outcome doc `specs/035-energy-lexicase-objective/outcome.md`: classify EACH mode as energy-works / tracking-collapses / energy-unmoved with per-scenario evidence AND the per-axis aggressiveness distribution shift on all 3 controls (SC-001/002, FR-006); record the total-energy (PE/KE) go/no-go (FR-007); pin milestone runs `retain=keep` and record their S3 prefixes (Principle VIII).

**Checkpoint**: US1 complete — energy verdict delivered for M1 and M2.

---

## Phase 4: Polish & Cross-Cutting

- [ ] T039 [P] crrcsim `mod_inputdev`: link `autoc_common` instead of the cherry-picked `evaluator.cc`/`serialization.cc`/`sensor_math.cc` (closes the BACKLOG cleanup; ensures libzstd + telemetry symbols resolve). `crrcsim/src/mod_inputdev/CMakeLists.txt`.
- [ ] T040 [P] Update `CLAUDE.md` + project memory references for 035 (per-mode buckets, zstd, `dmp-dump`, energy axis, retired `data.dat`).
- [ ] T041 Run full `quickstart.md` validation end-to-end (gate + US1 unit tests) as the feature acceptance pass.

---

## Dependencies & Execution Order

### Phase dependencies
- **Setup (P1)**: T001 → T002. No external deps.
- **Foundational (P2)**: depends on Setup. Internal order: A1 (T003–T007), A2 (T008–T017), A3 (T018–T022), A4 (T023–T024) → **GATE T025–T028**. A1/A2/A3/A4 are largely independent groups; the GATE depends on all of them.
- **US1 (P3)**: depends on the **GATE being green** (T025–T028).
- **Polish (P4)**: after US1 (T039 may run earlier once libzstd lands, but grouped here).

### Critical sequencing (same-file / data-flow)
- T001 before any zstd include (T008–T011, T014).
- T009 (zstd compress at upload) before T018 (tagging at upload) — same `src/autoc.cc` PutObject region.
- T004 (selector) before T005/T006/T011 (callers route through it).
- T021 (admin buckets/IAM/lifecycle) before T022 (ini bucket flip) and before the bucket-cutover aspect of the GATE.
- T031 (MAD test) before T034 (MAD impl); T029 before T032; T030 expects T032/T033 active.
- T032/T033/T034 all touch `selection.cc`/`fitness_decomposition.cc` — sequence within US1.

### Parallel opportunities
- Setup: T002 ∥ (after T001).
- A1: T003 (test) ∥ writing, then T004; T005/T006/T007 touch different files — parallel after T004.
- A2: T008 ∥ T013 ∥ T016 (different files); T009/T012/T018 serialize on `src/autoc.cc`.
- A3: T019 ∥ T020 ∥ T023 ∥ T024.
- US1 tests: T029 ∥ T030 ∥ T031 (T030/T031 same file — coordinate edits).
- Polish: T039 ∥ T040.

---

## Parallel Example: Foundational A1

```bash
# After T004 (selector implemented), route callers in parallel (different files):
Task: "T005 route tools/nnextractor.cc through s3_run_selector"
Task: "T006 route tools/renderer.cc through s3_run_selector + fix invert bug"
Task: "T007 collapse runIdPrefixForMode in include/autoc/util/run_id.h"
```

## Parallel Example: User Story 1 tests

```bash
Task: "T029 energy_metric_tests.cc convex integral"
Task: "T030 re-enable 4 Selection027 tests"   # same file as T031 — coordinate
Task: "T031 MAD-epsilon + constant-repro test"
```

---

## Implementation Strategy

### MVP = GATE + US1
1. Phase 1 Setup (T001–T002).
2. Phase 2 Foundational through the **GATE** (T003–T028) — the factory works on the new
   buckets/zstd/dmp-dump with no `data.dat`.
3. **STOP at the GATE.** Do not start energy until T025–T028 are green (user's explicit ask).
4. Phase 3 US1 (T029–T038) — energy verdict for M1 and M2.
5. Phase 4 Polish.

### Incremental delivery
- Foundational A1+A2 deliver dmp-dump + selector value even before bucket cutover.
- A3 (buckets/lifecycle) starts cutting the AWS bill immediately on cutover (T021/T022).
- US1 is the substantive deliverable; the bakes (T036/T037) are the long-pole, budgeted 2–3 each
  for the basin lottery.

---

## Notes
- [P] = different files, no incomplete-task dependency. Same-file tasks (e.g. `src/autoc.cc`
  T009/T012/T018; `selection.cc` T033/T034) are intentionally NOT [P].
- Tests precede implementation within each group (Constitution I).
- Admin task T021 is the only non-code item; everything else is operator/LLM-executable.
- Commit after each task or logical group; never rebuild during a live bake
  ([feedback_no_rebuild_during_training]).
