# Contract: Tracker-mode NN sensor interface (extended 45 → 54)

**Producer**: `gather_tracker_inputs` (sim-side, both minisim and crrcsim paths)
**Consumer**: NN forward-pass (`evaluator.cc::evaluateTracker`), `data.dat` emission (`src/autoc.cc`), data.dat parser (`specs/019-improved-crrcsim/sim_response.py`), dmp serializer, [genome ablation tool](../../../BACKLOG.md)

Extends [030's nn_sensor_interface contract](../../030-tracker-mode/contracts/nn_sensor_interface.md) — all 030 invariants carry forward; this contract adds the 9 phase-1 derived-feature slots and the related compile-time / runtime invariants.

## Surface (extension only)

The `TrackerInput` enum in `include/autoc/nn/nn_inputs.h` gains 9 trailing entries (before `COUNT`):

```cpp
enum class TrackerInput : uint16_t {
  // ... 45 existing 030 entries (unchanged) ...
  DIST_TO_BOUNDARY_ALONG_VEL = 44,

  // ----- 032 PHASE 1 NEW SLOTS -----
  BEACON_PAIR_SPAN_TM5 = 45,
  BEACON_PAIR_SPAN_TM4,
  BEACON_PAIR_SPAN_TM3,
  BEACON_PAIR_SPAN_TM2,
  BEACON_PAIR_SPAN_TM1,
  BEACON_PAIR_SPAN_NOW,
  SPAN_RATE,
  TARGET_TILT_SIN,
  TARGET_TILT_COS,
  COUNT  // = 54
};
```

Parallel additions to `kTrackerInputMeta` (display_name + header_width). See [data-model.md §1.3](../data-model.md#13-ktrackerinputmeta-extension).

`TrackerInputs` struct gains 9 new fields after `dist_to_boundary_along_vel` — see [data-model.md §1.2](../data-model.md#12-full-enum--struct-shape).

## Compile-time invariants (newly enforced)

```cpp
static_assert(static_cast<int>(TrackerInput::COUNT) == 54,
              "TrackerInput::COUNT must equal 54 per 032 phase 1");

static_assert(sizeof(TrackerInputs) == 54 * sizeof(float),
              "TrackerInputs layout must be contiguous float[54] with no padding");

static_assert(static_cast<size_t>(TrackerInput::COUNT) ==
              sizeof(kTrackerInputMeta) / sizeof(SensorInputMeta),
              "TrackerInput enum count must match kTrackerInputMeta length");
```

The chain `TrackerInput::COUNT == kTrackerInputMeta.length == sizeof(TrackerInputs)/sizeof(float) == 54` is the structural integrity contract. Any slot add/remove/reorder must update all three in lockstep or the build fails.

## Runtime invariants (asserted in tests)

1. **Range conformance**:
   - `beacon_pair_span[i] ∈ [0, 2√2 + ε]` for all i ∈ [0, 6)
   - `target_tilt_sin² + target_tilt_cos² ∈ [1 − ε, 1 + ε]` (sin/cos identity holds to fp32 precision)
   - `span_rate` is signed; magnitude bound is data-dependent (typically `≤ |span_max − 0| = ~2.83` for visibility transitions)

2. **CEP-gate substitution** (when `beacon_l_cep[5] ≥ CepGateThreshold || beacon_r_cep[5] ≥ CepGateThreshold`):
   - `beacon_pair_span_now == 0.0f` (exact)
   - `target_tilt_sin == 0.0f` (exact)
   - `target_tilt_cos == 1.0f` (exact)
   - History slots `beacon_pair_span[0..4]` carry their pre-gate values (the gate fires per-tick on the "now" computation; history is not retroactively gated)
   - `span_rate` derives mechanically from `span[5] - span[4]` (no separate gate — see [data-model.md §3.1](../data-model.md#31-open-question-note-on-span_rate-under-cep-gate-transitions))

3. **Identity-stable ordering**: `beacon_l_*` slots ALWAYS carry the port-beacon (target body `-y` mount) projection; `beacon_r_*` slots ALWAYS carry the starboard-beacon (target body `+y` mount) projection. This is true regardless of which beacon lands on which image-plane side at any given tick. See [identity_invariant.md](./identity_invariant.md).

4. **Deterministic computation**: same `(state, history, arena)` inputs MUST produce bitwise-identical `TrackerInputs` output across worker counts, replays, and machine re-runs (per [project_variation_design_principles](../../../.claude/projects/-home-gmcnutt-autoc/memory/project_variation_design_principles.md)). The rebuild-perf.sh regression gate enforces this end-to-end.

## Cross-platform mirroring

| Path | Status |
|---|---|
| autoc desktop training (`src/eval/tracker_stepper.cc` + `src/nn/evaluator.cc`) | PRIMARY producer — phase 1 implementation site |
| crrcsim FDM helper (`crrcsim/src/mod_inputdev/inputdev_autoc/crrcsim_tracker_helper.cpp`) | MIRROR — must populate identically; wire-equivalent obligation |
| xiao firmware | NOT touched in phase 1 (tracker-mode port deferred). When xiao tracker-mode lands, the contract MUST be honored byte-for-byte: same enum, same struct layout, same gating semantics, same identity ordering. |
| data.dat header walk (`src/autoc.cc`) | Auto-extends via `kTrackerInputMeta` walk; no code change |
| Python parser (`sim_response.py`) | Auto-extends via header-name match; no code change |
| dmp serializer (cereal on the `EvalData`/`TrackerInputs`-bearing struct) | Auto-extends via cereal `serialize` template; NO version bump (M2 policy per [research.md R5](../research.md#r5--dmp-schema-growth-without-version-bump-m2-policy-reinforcement)) |

## Test coverage

`tests/nn_sensor_interface_tests.cc` (extension):
- `TrackerInput::COUNT == 54` (count assert)
- Round-trip `nameOf(TARGET_TILT_COS) == "TARGET_TILT_COS"` etc. for the 9 new entries
- `kTrackerInputMeta[BEACON_PAIR_SPAN_NOW].display_name == "spn0"` (display-name canonicality)
- `sizeof(TrackerInputs) == 54 * sizeof(float)` (covered by static_assert, but also asserted at runtime for the test report)

`tests/gather_tracker_inputs_tests.cc` (extension):
- Synthetic AircraftState + projected history → run `gather_tracker_inputs` → verify slot population matches expected math
- CEP-gated case: set `history.left_cep[5] = 1.5f` → verify span_now == 0, tilt_sin == 0, tilt_cos == 1
- Deterministic across consecutive calls with identical inputs

`tests/derived_features_tests.cc` (NEW):
- Pure-math tests for span / span-rate / tilt sin-cos (no integration deps); see [data-model.md §3](../data-model.md#3-per-tick-data-flow-extended) and [research.md R9](../research.md#r9--test-coverage-budget)

## Backward-compatibility note (Constitution III)

This is a clean-cut extension of the 030 contract. No shims, no parallel `TrackerInputsV1` struct, no version-gated reader fork. Post-032 binaries cannot read pre-032 M2 dmps (cereal fails loudly on archive length mismatch). M1 (pathgen) dmps remain fully readable through the unchanged `PathgenInputs` path.

## Citations

- [spec.md](../spec.md) §1.5 Feature B (derived-feature design)
- [spec.md](../spec.md) Clarifications Q3 (span-rate formula), Q4 (CEP-gating), Q5 (history init), Q8 (dmp honesty)
- [research.md](../research.md) R2 (CEP threshold), R4 (tilt math), R5 (dmp schema policy)
- [Constitution III](../../../.specify/memory/constitution.md) — no compatibility shims
- [Constitution V](../../../.specify/memory/constitution.md) — versioned persistence (fail-loud safety net)
- [Constitution VI](../../../.specify/memory/constitution.md) — type-domain discipline (raw-ok annotations)
