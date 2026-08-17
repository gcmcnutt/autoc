# Implementation Plan: 041 — M2 Depth (observation-side objectives)

**Branch**: `041-m2-depth` | **Date**: 2026-08-07 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/041-m2-depth/spec.md`
**Design derivation**: [hypothesis.md](hypothesis.md) — the pre-spec memo carrying the corrections,
retractions and evidence behind every decision below. Read it for *why*; this file is *how*.

## Summary

041 adds **observations to match an objective that already exists**, rather than adding objectives. The
streak multiplier is 5× of fitness, ramped over 5 s, against a 0.8 s perceptual window — invisible to the
policy. The feature gives the NN an in-envelope flag plus a duration accumulator, completes the 6-DOF
craft-state block with an accelerometer, and rebakes M1 with them. **041 adds no load objective**: the
per-regime load findings are measured and reported, then handed to a follow-on feature.

Separately it settles by cheap offline measurement whether the M2 predictor head can be re-targeted to
something learnable — **a continuous current-bearing estimate**, horizon-free, with the prediction *error*
fed back as an input — or should be retired.

Technical approach: one research phase that retires the index-coupled-collection failure class
structurally; one contract-break commit carrying every model- and schema-incompatible change (including the
M2 camera vertical field 90° → 75° to match ordered hardware); two instruments (input ablation, offline
regime/load study) built before any compute is spent; one production M1 bake; one M2 bake scoped to the
predictor question.

## Technical Context

**Language/Version**: C++17 (autoc, crrcsim, tools), Python 3.11 (analytics only)
**Primary Dependencies**: Eigen (vec3/quat), cereal (NN + `EvalResults` + dmp serialization), inih (ini),
GoogleTest, CRRCSim LaRCSim FDM (the sole worker path)
**Storage**: file-based — `data.dat` (per-tick trace), `data.stc` (per-gen), per-mode S3 buckets
`autoc-m1` / `autoc-m2` / `autoc-eval` (`<run-id>/gen<N>.dmp.zst`)
**Testing**: GoogleTest unit + contract suites via `run_autoc_tests`; the zero-answer test pattern
(construct data whose correct answer is exactly 0, assert 0, plus a shifted-input-scores-worse companion)
**Target Platform**: Linux training host (24-core class); xiao nRF52840 / PlatformIO arduino-mbed for the
deployed forward pass; **INAV flight controller (fork `gcmcnutt/inav`, `~/inav`)** — a third build surface
added by the 2026-08-07 clarification, since `MSP2_AUTOC_STATE` must carry accel. Builds routinely here
(vendored ARM toolchain in-tree); **two established target variants** — bench = `MAMBAF722_2022A`, flight =
`MATEKF722MINI`, both at `63cffaf4` — bench deployed first. Precedent: 021 T041 built both. ⚠️ Disconnect the
GPS module before flashing.
**Project Type**: research trainer + simulator + embedded target — single unified CMake build
**Performance Goals**: no throughput regression beyond ~10% (040 baseline ≈ 6,200 sims/s); FP determinism
preserved (eval-vs-training bitwise match is the regression gate)
**Constraints**: no backward compatibility required this phase (whole-M2-initiative clean slate, operator
2026-08-07); NN input layout is a serialization contract shared with xiao codegen; nothing
fitness-affecting may land while a bake is live
**Scale/Scope**: M1 production bake at pop 8000 / 49 winds (~800 gens); M2 bake ~27 h; two new input
slots plus accelerometer slots in both input structs; one dmp schema break; one M2 camera-fidelity change
(vertical field 90° → 75°, matching ordered hardware — research.md R14)

**Unknowns carried into Phase 0 — all now resolved or explicitly assigned in
[research.md](research.md) R1–R14**: accelerometer slot count and framing (R1: 3 slots, specific force,
instantaneous, fallback ladder recorded); envelope-accumulator reset rule (R2: envelope exit only — and the
*flag*, not the accumulator, is the primary input); M2 envelope estimator (R3: direct-perception, built and
used in the D bake); load-axis form (R4: **none in 041** — Study A is report-only, findings feed the
follow-on); grouped-record migration of the pre-loop initial state (R5: named field beside the list);
version-bump decision (R6: bump, no migration, fail loud); predictor target (R10: continuous current-bearing
estimate); hardware accel source (R13: INAV `MSP2_AUTOC_STATE`, transformed field); camera model (R14:
V = 75°). **Operator-owned thresholds still open**: non-regression band (R7) and retry budget (R8).

## Constitution Check

*GATE: evaluated before Phase 0, re-evaluated after Phase 1 (below).*

| Principle | Status | How this feature satisfies it |
|---|---|---|
| **I. Testing-First** | ✅ | FR-003 makes the zero-answer test mandatory for every paired-series fitness term — the pattern that would have caught all four index bugs. New inputs get layout/count tests; the ablation tool gets an empty-mask identity test (SC-004) so the instrument is validated before it is trusted. |
| **II. Build Stability** | ⚠️ **scope grew 2026-08-07** | **Three** build surfaces, not two: `rebuild.sh` (autoc+crrcsim), `pio run -e xiaoblesense_arduinocore_mbed` (xiao), and **INAV** for the `MSP2_AUTOC_STATE` accel extension — the last needing **two variants** (bench = `MAMBAF722_2022A`, flight = `MATEKF722MINI`). Both targets are established and were built together in 021 T041, so this is scope, not risk. The INAV surface is exercised **only in Phase 6** (bring-up T001/T001a deferred there 2026-08-10) — the T045 landing gate is autoc/crrcsim + xiao. |
| **III. No Compatibility Shims** | ✅ | Explicitly the operating mode. FR-005 lands all breaks at once; FR-009 forbids version negotiation; every call site updated directly. Reinforced by [[feedback_m2_no_fallback_patterns]] — no defaults whose purpose is preserving unchanged callers. |
| **IV. Unified Build** | ⚠️ **action required** | Two `CMakeLists.txt` touches are expected (the ablation tool target, its test registration). Those MUST be built with a clean `scripts/rebuild-perf.sh`, not an incremental reconfigure. **The operator drives the clean rebuild** ([[feedback_operator_runs_regression_gate]]) — the plan must not schedule an agent-run one. |
| **V. Versioned Persistence Artifacts** | ⚠️ **decision forced — see below** | 041 breaks the dmp schema. V's write-side contract says a committed transition SHOULD **bump the version field** while *not* maintaining shims, and readers MUST fail loud naming both versions. |
| **VI. Type-Domain Discipline** | ✅ | New inputs are NN byte-format buffers, so raw `float` with `// raw-ok:` annotation is the correct choice at those declaration sites. Step-score / fitness accumulation stays `gp_fitness`; geometry `gp_vec3`. *(No load/energy accumulation is added — FR-019.)* The closing report runs the audit grep on touched paths. |
| **VII. No Silent Fallback Defaults** | ✅ | New config knobs and new worker-primed values follow the constructor-initializer rule. This principle is *load-bearing* here: the whole feature is about a value the policy cannot see, and its cautionary example (`cepGateThreshold` falling back to a hardcoded default) is the same failure shape. |
| **VIII. Retention** | ✅ **satisfied in advance** | Both comparators verified retained object-by-object 2026-08-07: `autoc-m1/…2026-07-06T01:35:46.579Z/` and `autoc-m2/…2026-08-04T03:43:24.586Z/`, 800/800 `retain=keep` each. FR-010 adds weight archival, closing the gap that made the 038 baseline unloadable. Prefixes recorded in spec + outcome per VIII.2/VIII.3. |
| **IX. Detached Training Launch** | ✅ | All bakes via `scripts/train.sh <ini> <logfile>`. No `run_in_background`, no foreground job. Pre-run build gate before each bake. |
| **X. Single Ordered Backlog** | ✅ | 041 *consumes* backlog items; anything deferred out returns to `specs/BACKLOG.md` in order at wrap. No per-item files. |

### Principle V — the decision this gate forces

There is a genuine conflict between the constitution and recorded habit, and the constitution wins
(Principle IV states agent memory is advisory, the constitution is the single source):

- **Habit** ([[feedback_no_cereal_versioning]]): "don't bump `CEREAL_CLASS_VERSION`; greenfield only."
- **Constitution V**: no shims, *and* "version transitions to which the project is fully committed should
  bump the version field directly", with fail-loud reads naming both versions.

These are compatible once separated: the habit is about **not maintaining compatibility**, V is about
**declaring identity**. 041 does both — bump the version, keep zero migration paths, fail loud on older
files. This is a change in practice and is called out here so it is a decision rather than a drift.
Resolved in [research.md](research.md) R6.

### Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|---|---|---|
| Grouped per-tick record (schema break + every consumer touched) | Retires the index-coupled failure class at the root — four instances found in one session, three live, one of them the M2 objective since 030 | Asserting/testing the parallel-index invariant leaves it *possible* to violate. With no compat tax this phase, the structural fix costs only work, and it is the last time the class can bite (operator 2026-08-07) |
| New ablation tool target | The only rigorous instrument in a feature with no control bake | Reusing `nnextractor` was considered; a separate target keeps the eval-parity binary untouched. Scope held to `--zero-input` only — no weight-block masking |

## Project Structure

### Documentation (this feature)

```text
specs/041-m2-depth/
├── README.md            # 040-wrap seed (pre-existing)
├── hypothesis.md        # pre-spec design memo — the derivation of record
├── spec.md              # feature specification
├── plan.md              # this file
├── research.md          # Phase 0 output — unknowns resolved
├── data-model.md        # Phase 1 output — entities, layouts, schema deltas
├── quickstart.md        # Phase 1 output — how to run each phase
├── contracts/           # Phase 1 output
│   ├── nn-input-layout.md      # input slot contract (both modes) + xiao/codegen surface
│   ├── dmp-schema.md           # grouped per-tick record + version bump + fail-loud read
│   ├── ablation-cli.md         # genome ablation tool interface + report fields
│   └── offline-study.md        # regime/load study inputs, outputs, baselines
└── tasks.md             # Phase 2 output (/speckit.tasks — NOT created here)
```

### Source Code (repository root)

```text
include/autoc/
├── nn/
│   ├── nn_inputs.h              # US4: +envelope occupancy, +specific force; both enums + meta tables
│   └── topology.h               # input counts, weight counts, topology strings; US5 output count
├── eval/
│   ├── aircraft_state.h         # US2: wind_velocity set at record; grouped-record participation
│   ├── fitness_decomposition.h  # US1/US4: paired-series terms, regime/load terms
│   └── camera_projection.h      # (read-only here — 040 closed perception)
├── rpc/
│   └── protocol.h               # US1/US2: grouped per-tick record, config block, version field
└── util/
    └── config.h                 # US2/US4: new knobs via AUTOC_CONFIG_FIELDS X-macro

src/
├── eval/
│   ├── fitness_decomposition.cc # US1 FR-004 pairing fix; US4 envelope + load terms
│   ├── selection.cc             # US5 predictor axis gate (no load axis in 041 — FR-019)
│   └── fitness_computer.cc      # streak machinery (source of the envelope definition)
├── nn/
│   └── evaluator.cc             # gather_inputs / gather_tracker_inputs — new slots
├── autoc.cc                     # worker meta, config print, per-gen dump
└── analytics/
    └── (new) regime_load_study.py  # US3 offline study

tools/
├── nn_ablate.cc                 # US3 new — input-mask ablation + eval harness
├── dmp_dump.cc                  # US3 physics columns; US2 config-block read
├── nn2cpp.cc                    # US4 regenerate for new input layout
└── renderer.cc                  # US2 config-block read

crrcsim/src/mod_inputdev/inputdev_autoc/
├── inputdev_autoc.cpp           # US2 wind set + exact tick stamp; grouped record push
└── crrcsim_tracker_helper.cpp   # US4 tracker-side envelope estimator (M2)

crrcsim/src/SimStateHandler.cpp  # US2 FR-007 tick stamping (submodule — pointer bump FIRST)

tests/
├── fitness_decomposition_tests.cc   # zero-answer + shifted-worse patterns
├── contract_evaluator_tests.cc      # input counts / layout
├── nn_evaluator_tests.cc            # slot semantics
├── contract_config_tests.cc         # field count
├── contract_tracker_config_tests.cc # decouple from mutable production values
├── selection_tests.cc               # re-enable the 4 DISABLED_ multi-objective tests
└── (new) nn_ablate_tests.cc         # empty-mask identity (SC-004)

xiao/src/generated/nn_program_generated.cpp   # regenerated from new layout
```

**Structure Decision**: no new top-level structure. 041 is a change to existing eval / nn / rpc paths
plus one new tool (`tools/nn_ablate.cc`) and one new analytics script. The crrcsim tick-stamp fix is a
**submodule** change and follows pointer-bump-first ordering ([[feedback_submodule_merge_order]]).
Feature-local scripts stay under `specs/041-m2-depth/`; only cross-version utilities go in `scripts/`
([[feedback_scripts_dir_scope]]).

## Phase sequencing and the hard ordering constraints

```text
A0  research scan + inventory ──► A1  ONE contract-break commit ──► B  M1 bake ──► (repeat if stuck)
         │                              │                               │
         └─ decides grouped-record       │  grouped record · new inputs  └─ H1a ablation read
            migration shape              │  step-score→tick loop · wind    (input ablation matrix
                                         │  config block · tick stamp       + control-input calibration)
                                         │  version bump · camera V=75
                                         └─ nothing else lands until B completes
A2  ablation tool  ─┐
A3  offline study  ─┴─► REPORT ONLY — feeds the follow-on aggressiveness feature, does not gate A1

C   predictor offline feasibility + E1 ablation ──► C3 design or retire ──► D  M2 bake
```

Binding constraints:

1. **Land objective changes between baselines, never between a run and its comparator.** The entire A1
   bundle is one commit, before B starts.
2. **Never rebuild while a bake is live** — workers re-exec `build/autoc`
   ([[feedback_no_rebuild_during_training]]).
3. **A3 no longer gates A1** (clarified 2026-08-07 — report-only, no load axis in 041). It must complete
   before the outcome is written, since its findings are a deliverable and the input to the follow-on.
4. **FR-004 (prediction pairing) must land before C's Δspan control study**, or that study is confounded.
5. **Submodule pointer bump before the parent merge.**

## Post-Design Constitution Re-Check

Re-evaluated after Phase 1 artifacts (research.md, data-model.md, contracts/, quickstart.md):

- **I / II** — unchanged and satisfied; contracts name the specific tests per surface.
- **III** — Phase 1 confirmed no shim is needed anywhere: the grouped record replaces the parallel lists
  outright rather than wrapping them.
- **IV** — `CMakeLists.txt` touches confirmed (one target, one test registration) → clean
  `rebuild-perf.sh`, operator-driven. **Carried as an explicit task, not an assumption.**
- **V** — resolved: bump the version field, no migration path, fail loud naming both versions
  ([contracts/dmp-schema.md](contracts/dmp-schema.md)).
- **VI** — new input slots are byte-format buffers → raw `float` + `// raw-ok:`; load accumulation
  `gp_fitness`. Audit grep in the closing report.
- **VII** — the accelerometer framing decision (specific force, not kinematic) is precisely a
  no-silent-fallback concern: the wrong choice is invisible and produces a constant 1 g error. Contract
  states it and a test pins it.
- **VIII** — comparators pinned and verified; weight archival is FR-010. One accepted limitation recorded
  (second novel-geometry source deliberately left to expire).
- **IX / X** — unchanged.

**Gate result: PASS**, with two carried actions — the operator-driven clean `rebuild-perf.sh` on the
CMake touch, and the Principle V version-bump practice change now made explicit rather than habitual.

## Open items owned by the operator

Not blockers for `/speckit.tasks`, but each shapes a task's acceptance:

1. ~~**"Calibration efforts"**~~ — ✅ **RESOLVED 2026-08-10 (operator): it means re-establishing the
   COMPARATOR SET** — new M1 baseline pinned, weights archived beside the dmp (FR-010), novel-geometry
   source re-run so generalization stays measurable (T097). Already covered by T065 / FR-010 / T097, so this
   closes with **no new work**. Explicitly NOT the perception model's 15 assumed physical values: 040
   SC-012 proved those are *substitutable*, not *wrong*, no new measurements exist to inform them, and the
   camera hardware that would produce some is still weeks out.
2. **Non-regression band** — which historical runs define it, and how much per-axis movement counts as
   no-regression (research.md R7 proposes a default).
3. **Retry budget** for the production M1 bake, declared before the first attempt (research.md R8).

---

# AMENDMENT PLAN 2026-08-17 — containment + step-wise cost (M1 and M2)

Implements the spec amendment (FR-033…FR-039). Sequenced so the two **zero-bake** steps come first,
because either can change everything after them.

## Sequencing rationale

| phase | bake cost | why here |
|---|---|---|
| **A. Read what we already have** | none | the gen-608 elite and its dmps exist; both open questions are answerable without spending compute |
| **B. Instrument** | none | recording changes are free ONLY while no bake runs — that window is open now and closes the moment we start one |
| **C. Reshape** | one M1 bake | only after A and B say what to reshape |
| **D. Per-tick advantage** | later | needs B's data and C's result; see the 042 backlog entry |

⚠️ **Phase B requires a rebuild.** It was deferred throughout t1 for that reason
([feedback_no_rebuild_during_training](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_no_rebuild_during_training.md)).
The constraint is lifted; do not re-open a bake until B has landed, or it defers again.

## Shared-by-construction

Everything here lands in code both modes already share — `FlightArena` / `arena.h` for containment,
`nn_inputs.h` + both gathers for observations, `fitness_*` for cost. **No M1-only or M2-only variant is
acceptable**: a containment rule learned by M1 and not M2 means the M2 chase re-learns it against a
different objective, which is how the two modes drift. This is the same "one definition" discipline the
index-coupling inventory exists to enforce.

## Risks, and what each would look like

1. **The gradient hands over the answer.** ∂score/∂position is close to telling the policy where to go; it
   may learn greedy gradient-following and lose the anticipation the recurrent layer was building. **Tell**:
   the accelerating late-run curve shape disappears; effective rank falls further.
2. **Boundary shaping produces a centre-hugger.** Potential-based shaping cannot change the optimum, but a
   badly-chosen Φ can still slow learning badly. **Tell**: egress drops AND tracking drops together.
3. **`Ps_max(state)` is awkward to obtain** from the FDM without a fit or lookup. **Fallback**: a learned
   baseline, which costs the critic first — i.e. it promotes phase D ahead of C.
4. **Shaping does not fix strategy.** Potential-based shaping preserves the optimum, so if the tight spiral
   is genuinely objective-optimal, this makes learning faster without changing the target. Removing the
   spiral needs the objective or the airframe, and there is a standing decision to let a real flight
   trigger that call.

## Gates

- Phase B is a **format break**: `EvalResults` version bump, one owed re-bake, everything in one commit —
  the same FR-005 discipline the A1 bundle used.
- Constitution IX pre-run gate before any bake in phase C.
- Phase A's ablation must run **before** any input is dropped (SC-015).
