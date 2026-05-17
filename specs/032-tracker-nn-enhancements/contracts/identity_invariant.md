# Contract: Port/Starboard Identity-Stable Beacon Ordering (Feature A)

**Status in phase 1 sim**: ALREADY SATISFIED. Phase 1 work for Feature A is **documentation + contract test**, not new code in minisim or crrcsim. See [research.md R1](../research.md#r1--feature-a-in-sim-is-already-satisfied-the-only-change-is-the-contract) for the finding.

**Future xiao port**: This contract MUST be honored by the xiao firmware when tracker-mode lands there (deferred). The doc updates in `docs/COORDINATE_CONVENTIONS.md` and `docs/sensor-pipeline.md` exist precisely to keep the xiao port wire-equivalent.

## Invariant

For every tick at which the chase NN is asked to evaluate, the input slots populated into `TrackerInputs` MUST satisfy:

- `beacon_l_x[i]`, `beacon_l_y[i]`, `beacon_l_cep[i]` for all i ∈ [0, 6) reflect the projected observation of the **PORT** beacon — the beacon physically mounted at target body `-y` (left wingtip).
- `beacon_r_x[i]`, `beacon_r_y[i]`, `beacon_r_cep[i]` reflect the **STARBOARD** beacon (target body `+y`, right wingtip).

This identity holds **regardless** of:
- Which beacon lands on which side of the image plane at any given tick (e.g., when target faces away from chase, port and starboard appear on swapped image sides)
- Which beacon has the lower NDC `x` value
- Which beacon has the higher CEP (more uncertain detection)

The NN sees "I see target's port wing at NDC (x, y)" — not "the leftmost detected blob is at (x, y)."

## How it's enforced in each platform

### autoc minisim (`src/eval/tracker_stepper.cc`)

`projectAndShiftHistory(target)`:
1. Constructs a `ProjectionInput` with `beacon_mount_target_body = beacon_left_.mount_body` (= body `-y` per [BeaconConfig default](../../../include/autoc/eval/beacon_config.h#L25))
2. Calls `projectBeacon(proj)` and stores result in local `left`
3. Constructs second `ProjectionInput` with `beacon_mount_target_body = beacon_right_.mount_body` (= body `+y`)
4. Calls `projectBeacon(proj)` and stores result in local `right`
5. Writes `history_.left_*[5] = left.*` and `history_.right_*[5] = right.*`

The identity is preserved by virtue of the static mapping `beacon_left_` → `left` variable → `history_.left_*`. No post-projection NDC sort.

### crrcsim helper (`crrcsim/src/mod_inputdev/inputdev_autoc/crrcsim_tracker_helper.cpp`)

Mirrors the autoc minisim flow exactly. Same `projectBeacon` calls, same `left` / `right` variable mapping, same history writes.

### xiao firmware (FUTURE — NOT in phase 1)

When xiao tracker-mode lands, it will consume FPGA-emitted blob observations. The FPGA emits per-detection (x, y, CEP, code_id) tuples. The xiao firmware MUST:
1. Map `code_id → {port, starboard}` via the Gold-code-to-physical-port mapping (configured at xiao build time or via MSP setting)
2. Populate `TrackerInputs::beacon_l_*` from the port-keyed detection and `beacon_r_*` from the starboard-keyed detection
3. NEVER sort by image-plane `x` or by `code_id` numeric value
4. When a code is undetected for the current frame, populate the corresponding slot with the CEP-sentinel pattern (`screen_x = 0, screen_y = 0, cep = kCepSentinelFloat`) — same convention as sim's invisible-beacon case

## Contract test (phase 1)

`tests/gather_tracker_inputs_tests.cc` gains a synthetic test:

1. Construct a target oriented at chase by heading 90° (target's port wing physically on chase's image-plane right)
2. Project both beacons → verify that the `left_x[5]` slot carries the port-beacon NDC (which has positive x in this geometry), not the starboard-beacon NDC
3. Construct a target oriented away from chase (target's port wing on chase's image-plane left)
4. Verify identity-stable holds: `left_x[5]` again carries port-beacon NDC, which now has negative x

The test is a regression guard: any future refactor that re-introduces NDC-x sorting in sim breaks the test.

## Doc deliverables (phase 1)

### `docs/COORDINATE_CONVENTIONS.md` (modified)

Add a section "Beacon Identity-Stable Ordering" with:
- The mount convention (port = body -y, starboard = body +y)
- The identity-stable invariant text from this contract
- The pointer to this contract file and the failure-mode story (xiao port preservation)

### `docs/sensor-pipeline.md` (modified)

Add a section "Identity-stable beacon slot mapping" cross-referencing `COORDINATE_CONVENTIONS.md`, plus the CEP-gating + neutral-substitution rule from spec Q4.

## Why this contract exists (rationale recap)

The NN learns to associate "wing-tip pair pose" with target maneuvering. If port and starboard could swap slots between ticks (NDC-x sorting), the NN would see an apparent +180° tilt flip between consecutive ticks even when target is flying steadily — that contradictory gradient would defeat any tilt-based learning. Identity-stable ordering preserves the geometric continuity the NN's recurrent state can build on. The sim has always done this by construction; the contract codifies it so the xiao port can't accidentally break it.
