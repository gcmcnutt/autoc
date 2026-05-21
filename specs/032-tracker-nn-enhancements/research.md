# 032 — Research (Phase 0)

Investigations resolving every NEEDS CLARIFICATION from [plan.md](./plan.md) Technical Context and locking down the design decisions that the contracts + data-model depend on. Each finding follows Decision / Rationale / Alternatives.

---

## R1 — Feature A in sim is already satisfied; the only change is the contract

**Decision**: Feature A (port/starboard identity-stable beacon ordering) requires **NO code change in minisim or crrcsim** in phase 1. The change is to document the invariant in `docs/COORDINATE_CONVENTIONS.md` + `docs/sensor-pipeline.md`, add a contract file under `contracts/identity_invariant.md`, and add a contract test that asserts the invariant.

**Rationale**: The spec's Feature A framing — "Today the NN labels beacons 'left' and 'right' by NDC ordering" — is inaccurate for the existing sim pipeline:
- `tracker_stepper.cc:117-158` (`projectAndShiftHistory`) explicitly projects `beacon_left_` (mount body `-y` per [beacon_config.h:25](../../include/autoc/eval/beacon_config.h#L25)) into the `left` variable, then writes `history_.left_*[5] = left.*`. Same for `beacon_right_` (body `+y`) → `right` → `history_.right_*[5]`.
- `crrcsim_tracker_helper.cpp:77-114` mirrors the same pattern.
- `gather_tracker_inputs` in `src/nn/evaluator.cc:430-443` then copies `history.left_*` → `out.beacon_l_*` and `history.right_*` → `out.beacon_r_*` without any NDC re-sort.
- Port-keyed identity is therefore preserved end-to-end in both sim paths, regardless of which image-plane side each beacon lands on at any given tick.

The "NDC ordering" failure mode the spec describes is a **future-xiao** concern: when the FPGA emits two detected blobs (sorted by image position, or by Gold-code), the xiao firmware must map them back to port/starboard by Gold-code identity before populating NN input slots. That work belongs to the xiao tracker-mode milestone (deferred, NOT phase 1).

**Implication for the spec's "old 030 weights aren't portable across this reorder" claim**: That claim is wrong about Feature A in sim (there's no reorder). It's still correct that 032 phase 1 breaks 030 weight portability — but the breakage comes entirely from Feature B (the 9 new input slots shift the topology weight count), not from Feature A.

**Alternatives considered**:
- Add a runtime assertion in `gather_tracker_inputs` that checks identity-stability — rejected as redundant (no code path violates it; testing the invariant in `gather_tracker_inputs_tests.cc` is sufficient).
- Re-derive the invariant in xiao firmware now as future-proofing — rejected as out of phase 1 scope (xiao tracker-mode port is deferred entirely).

---

## R2 — CEP threshold value: reuse `kCepSentinelThreshold = 1.25f`

**Decision**: The phase-1 CEP-gating threshold for substituting derived-feature neutral values is the existing constant `kCepSentinelThreshold = 1.25f` defined in [camera_projection.h:31](../../include/autoc/eval/camera_projection.h#L31). The new `[DerivedFeatures]` section in `autoc-tracker.ini` exposes a `CepGateThreshold` knob (default = `1.25`) for operator override, but the default and recommended value matches the existing sentinel threshold.

**Rationale**:
- The 030 perception pipeline already defines an "invisible" boundary at CEP ≥ 1.25 (any out-of-FOV / out-of-emission-cone / occluded beacon ships with `cep = kCepSentinelFloat = 1.5f` per [camera_projection.h:32](../../include/autoc/eval/camera_projection.h#L32) and the post-int8-dequantize path).
- Reusing this threshold means "derived-feature neutral substitution fires exactly when at least one beacon is invisible by the existing definition." That's the simplest and most defensible default — no new perception semantics, just a downstream consumer of an existing signal.
- A lower threshold (e.g., 0.5) would gate more aggressively on noisy-but-present detections; but pre-tuning that without bench data risks throwing away good signal during normal-operation degradation. The config knob preserves the option to lower it later without code change.

**Alternatives considered**:
- Per-beacon threshold (separate L vs R) — rejected as over-parameterized for phase 1; no first-principles reason the wingtip beacons should gate at different thresholds.
- Time-domain hysteresis (gate only after N consecutive ticks above threshold) — rejected as scope creep; phase 1 keeps the substitution per-tick to avoid coupling with the existing 6-tick history machinery.

---

## R3 — History init at scenario start: span[6] mirrors NDC replicate-first-valid

**Decision**: At scenario start, `tracker_stepper.cc:initScenario()` (and the crrcsim helper equivalent) pre-fills `beacon_pair_span[6]` by computing span from the first valid (CEP-passing) projected sample's left+right NDC positions, then replicating that scalar across all 6 history slots. If the first tick is CEP-gated (either beacon invisible at tick 0), the neutral value (span = 0) is replicated across all 6 slots. This mirrors the existing NDC point-history convention (replicate-first-valid per spec Q5).

**Rationale**:
- Spec Q5 locks: "derived feature histories mirror whatever the existing 030 NDC point histories do at scenario start."
- Current NDC pattern: `for (int r = 0; r < 6; ++r) { projectAndShiftHistory(source_.samples[0]); }` — projects source[0] six times, propagating into all slots. Span is a function of the projected (left, right) NDC pair, so replicating the projection naturally replicates the span at no extra cost.
- CEP-gating layers on top: if at tick 0 the projection comes back with one beacon invisible, the neutral substitution applies during the pre-fill loop just as it would during normal-operation ticks. Single convention covers both — no special-case "scenario-start neutral fill" branch.

**Alternatives considered**:
- Zero-fill the span[6] history at scenario start — rejected because it would produce a step-change discontinuity at tick 1 (zero → first real span value), an artifact that contaminates the NN's gradient signal during the first ~6 ticks of every scenario.
- CEP-gated neutral-fill at scenario start (always (0, 0, 0, 0, 0, 0) regardless of tick-0 visibility) — rejected because tick 0 visibility is the normal case; replicate-first-valid is what the surrounding NDC slots already do, and divergence would be a phase-1 surprise.

---

## R4 — Tilt computation convention and degenerate-pair handling

**Decision**: Tilt θ is the image-plane angle of the directed line from **port beacon center (red)** to **starboard beacon center (green)**, measured as `atan2(y_r − y_l, x_r − x_l)` in NDC. Convention: θ = 0 when chase + target wings are level relative to each other (port→starboard line projects horizontally with port on chase-image-left, i.e., `x_l < x_r` and `y_l = y_r`). Encoded as `(sin θ, cos θ)` to remove the ±π wraparound discontinuity.

Degenerate case: when both beacons project to nearly identical NDC positions (`‖(x_r,y_r) − (x_l,y_l)‖ < 1e-4` in screen-fraction units, e.g., target dead-ahead at extreme range), tilt is undefined — substitute the neutral `(sin θ, cos θ) = (0, 1)`. This case typically co-occurs with high CEP (range past detection threshold), so CEP-gating from R2 already covers it; the geometric guard is belt-and-suspenders for sim-edge cases where CEP might pass but projection produces a near-coincident pair.

**Rationale**:
- `atan2` with sin/cos encoding is the standard NN angle-input pattern; values stay bounded in [-1, 1] and the wrap discontinuity disappears (sin ±π = 0, cos ±π = -1; both smooth).
- The 1e-4 epsilon is chosen against NDC's [-1, +1] coordinate range — 1e-4 ≈ 5 pixels at typical 4k camera resolution, well below the smallest tilt the NN could meaningfully consume.
- Convention "θ = 0 = wings level relative" complements the existing absolute-world `quat_w/x/y/z` inputs (chase-attitude in world frame); the NN sees both signals and learns to separate "I'm rolled in the world" from "I'm rolled relative to target."

**Alternatives considered**:
- Raw radians as a single input — rejected due to ±π wraparound discontinuity (NN activations don't like discontinuities in input space; gradients across the wrap are meaningless).
- Tilt sign convention flipped (starboard → port instead of port → starboard) — rejected as arbitrary; the chosen direction matches the natural "from red to green" reading order in the renderer's mini-panel splat display.

---

## R5 — dmp schema growth without version bump (M2 policy reinforcement)

**Decision**: 032 grows the cereal-serialized `TrackerInputs`-bearing dmp shape from 45 inputs to 54, **without** bumping the dmp version field. Pre-032 M2 dmps become unreadable through the post-032 `TrackerInputs` deserialization path; the cereal binary archive will fail loudly on the length mismatch (Constitution V's read-side fail-loud safety net). M1 dmps deserialize via the unchanged `PathgenInputs` path and remain fully readable.

**Rationale**:
- Operator policy (2026-05-16): "we don't revise schema version numbers during m2 experiments, just forget backward compat in the m2 realm (m1 however must still work)." Codified in [feedback_no_cereal_versioning](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_no_cereal_versioning.md).
- Constitution V exemption: "version transitions to which the project is fully committed should bump the version field directly (not maintain compatibility shims) — fail-loud on read is the safety net for older files." 032 phase 1 is the canonical case: greenfield M2 schema growth, no migration path, rely on fail-loud.
- M1 / M2 separation: `PathgenInputs` struct (`NNInputs` in `nn_inputs.h:35-51`) and `PathgenInput` enum are NOT touched by 032. M1 dmps continue to deserialize cleanly; only M2 (tracker-mode) dmps are affected by the schema change.
- The dmp-honesty invariant (per [feedback_honest_dmp_recording](../../.claude/projects/-home-gmcnutt-autoc/memory/feedback_honest_dmp_recording.md)): the 9 new derived-feature input slots MUST appear in both `data.dat` and dmp recordings. 032's schema bump is the audit boundary at which we verify dmp captures all NN inputs + outputs — any gap (or rationale for one) gets reconciled in phase 1.

**Alternatives considered**:
- Bump cereal class version + ship a v1-reader fallback path — rejected per the M2 policy (no backward compat in M2 realm; the compat-shim machinery is exactly what the policy avoids).
- Keep 45-slot struct + store the 9 new inputs in a sidecar — rejected as architectural ugliness for no benefit; the NN forward pass consumes a single contiguous float buffer, and splitting the storage would only complicate the cereal contract.

---

## R6 — ini schema for phase 1: minimal `[DerivedFeatures]` section

**Decision**: Add one new section to `autoc-tracker.ini`:

```ini
[DerivedFeatures]
CepGateThreshold                = 1.25       # CEP value at/above which derived features substitute neutral values; default matches kCepSentinelThreshold
```

No new section needed for Feature A (no-op in sim per R1). No section needed for the topology (16r is the existing baseline; phase 1 does not introduce a topology knob — the input count grows automatically via `TrackerInput::COUNT`).

**Rationale**:
- Q4 of the spec explicitly says the CEP threshold is "a config knob — set during /plan or implementation." R6 lands the knob with a default matching the existing sentinel threshold (R2).
- The new section is colocated with other tracker-mode knobs in `autoc-tracker.ini`. No M1 (`autoc.ini`) change needed — derived features are tracker-mode-only.
- inih parser pattern follows existing tracker sections (e.g., `[CrashHull]`, `[Camera]`) — no parser changes required.

**Alternatives considered**:
- Skip the ini knob entirely; hardcode 1.25 — rejected because it would force a recompile to retune the threshold during phase 1 attribution-bake follow-ups (Q7 partial-band path).
- Expose more tilt / span parameters (epsilon, smoothing) — rejected as scope creep; the operational design has zero knobs at the math level (raw `atan2`, raw one-tick diff), and exposing them would invite premature tuning.

---

## R7 — Attribution-bake methodology (phase-1b contingency from spec Q7)

**Decision**: If the phase-1 combined bake lands in the 0.10–0.15 partial band (per spec Q7), the **B-off attribution bake** runs from a **git-reverted pre-032 commit**, NOT via a runtime feature flag. No `EnableDerivedFeatures` knob exists in the M2 codebase or ini; greenfield M2 means the code IS the change.

- **B-off attribution bake**: operator checks out the pre-032 commit (e.g., `git checkout 8bfad02`, the 030 closeout), runs the same bake protocol against the same source dmp + same pop / gens / seed budget, and reads plateau-avgInRamp. That number is the "without derived features" attribution point. Result: same plateau as 032 combined → derived features added noise; collapsed to ~030 baseline (~0.07) → derived features were doing the work in the combined bake; in between → both contribute.
- **A-only attribution**: NOT applicable in sim, since identity-stable ordering is already the sim default (R1 — sim's mount-keyed pipeline preserves identity by construction; the pre-032 commit produces the same identity-stable inputs). The pre-032 bake IS effectively A-only-without-B as well as nothing-at-all. If real-flight rollout later shows A-only is the load-bearing piece, that attribution will come from xiao deployment (where Gold-code identity preservation is non-trivial), not from sim.

**Rationale**:
- **No compat shims** (Constitution III, operator policy 2026-05-16): the M2 codebase doesn't carry runtime toggles to undo the feature. If we want "without derived features", git already provides that — `git checkout` to a pre-032 commit, bake, done.
- Adding a runtime flag means the `gather_tracker_inputs` function carries a conditional branch + zero-fill path forever, plus an ini knob + loader code + test, all in service of a hypothetical follow-on bake that may never run. The git-revert approach has zero ongoing code cost.
- Sim Feature A is a no-op (R1) — the pre-032 commit already runs sim with port/starboard identity-stable ordering. Re-baking against pre-032 directly answers "what does identity-stable + no derived features look like" (= the 030 baseline ≈ -17k / avgInRamp ≈ 0.07).
- The git-revert workflow is also less error-prone than a flag: there's no risk of accidentally leaving `EnableDerivedFeatures = 0` set in a config that later gets committed and used for a "phase-1 combined" bake by mistake.

**Alternatives considered**:
- Runtime `EnableDerivedFeatures` flag (the path this research originally took) — rejected per the above; greenfield M2 + Constitution III rule out the toggle.
- Implement an `EnableIdentityStableOrdering` flag that synthetically un-stabilizes sim ordering via NDC-x-sort post-projection — rejected as adding dead code (sim's mount-keyed pipeline can't be un-done cleanly without restructuring the projection flow), AND because pre-032 git revert delivers the same attribution point at zero ongoing code cost.
- Skip B-off attribution entirely and treat partial-band result as inconclusive → escalate to phase 2 — rejected because the git-revert bake is cheap (~ same compute as the combined bake) and gives the operator a clean signal on derived-feature load-bearingness.

---

## R8 — Bake protocol parameters (recap, no new decisions)

**Decision**: Phase-1 bake parameters inherit directly from postdiag2 (the most recent baked 16r M2 reference):
- Population: 5000 (per `autoc-tracker.ini:42`, current value)
- Generations: at least 322 (matches smoke15 reference length per spec Q6); operator may bake longer if elite-fitness slope is still positive at gen 322
- Plateau read: average avgInRamp over the last 50 gens (per spec Q6)
- Source dmp: same as postdiag2 (currently `autoc-9223370259105171692-2026-05-02T19:20:04.115Z/gen9200.dmp` per `autoc-tracker.ini:27`)
- Variation ramp, crash-hull, arena, fitness shape: unchanged from postdiag2
- Topology: 32→16r→3 (unchanged — T-102 falsified 32r as worse)
- Seed: -1 (per ini, auto-randomized for the bake; reseed manually if reproducibility required)

**Rationale**:
- Holding all knobs constant except the input vector growth maximizes the attribution signal: any avgInRamp delta vs postdiag2 is attributable to phase-1's input change.
- Per [project_late_run_fitness_interpretation](../../.claude/projects/-home-gmcnutt-autoc/memory/project_late_run_fitness_interpretation.md), variation-ramp pressure makes cross-run best-fitness comparisons noisy; avgInRamp is the variation-stable comparator (spec §2 reaffirms this).

**Alternatives considered**: None — these are protocol recaps, not design decisions.

---

## R9 — Test coverage budget

**Decision**: Phase 1 adds three test artifacts:

1. **`tests/derived_features_tests.cc`** (NEW): unit tests for the pure math — span scalar from two NDC pairs, span-rate from two consecutive spans, tilt sin/cos from two NDC pairs, degenerate-pair handling, CEP-gated neutral substitution. Fully isolated; no `AircraftState` / `FlightArena` dependency.
2. **`tests/gather_tracker_inputs_tests.cc`** (EXTENDED): existing test file gains assertions that the new slots are populated correctly end-to-end (project a synthetic target sample → run `gather_tracker_inputs` → verify span/span-rate/tilt slots match expected values + CEP-gate fires when expected).
3. **`tests/nn_sensor_interface_tests.cc`** (EXTENDED): existing TrackerInput enum tests gain `static_assert(TrackerInput::COUNT == 54)` and round-trip name↔index for the new slot names.

End-to-end coverage comes from the rebuild-perf.sh regression gate: M1 pathgen bakes must remain bitwise-equal pre/post 032, and the M2 sim path must remain deterministic across worker counts (variation-PRNG signature).

**Rationale**:
- TDD per Constitution I: tests-first per phase 1 milestone.
- The math layer (test 1) is the most likely site for subtle bugs (sin/cos sign, span formula, degenerate cases) and warrants isolated coverage.
- The integration layer (test 2) catches structural wire-up bugs (slot index mismatch, history alignment).
- The contract layer (test 3) catches enum/struct-size drift at compile time + builds the "the enum and the meta array and the COUNT all agree" invariant chain.

**Alternatives considered**:
- Roll all three into one test file — rejected as muddying the pure-math layer with integration setup boilerplate.
- Add a property-test (e.g., sin² + cos² ≈ 1) — accepted as a quick add inside test 1; not a separate file.

---

## Phase-0 exit gate

All NEEDS CLARIFICATION items in plan.md's Technical Context are resolved:
- Schema bump policy: R5
- M1 isolation: R5
- Identity-stable ordering scope in sim: R1
- CEP threshold default + ini knob: R2 + R6
- History init: R3
- Tilt math + degenerate handling: R4
- Attribution-bake mechanism: R7
- Bake parameters: R8
- Test plan: R9

Phase 1 (data-model.md + contracts/ + quickstart.md) follows.
