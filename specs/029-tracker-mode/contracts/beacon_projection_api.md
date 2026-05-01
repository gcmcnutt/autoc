# Contract: Beacon projection API

**Producer**: `src/eval/beacon_projection.cc` (NEW for 029)
**Consumer**: `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp` (per-tick input gathering for tracker mode)

**Source**: per FR-005 and [research.md R3](../research.md#r3--camera-projection-math-implementation-strategy). Hand-rolled in pure Eigen (~30-50 LOC). No new dependencies.

## API

```cpp
// include/autoc/eval/beacon_projection.h

#include <Eigen/Geometry>
#include "autoc/eval/camera_config.h"

struct BeaconProjectionResult {
    float screen_x;       // [-1, +1] normalized to camera FOV; 0 = center
    float screen_y;       // [-1, +1] normalized
    float visible;        // 1.0 = beacon detected, 0.0 = not detected
};

/// Project a single beacon point through the camera, given the training craft's
/// pose. Returns the 3-float collapsed result the NN sees, mirroring the deployed
/// FPGA centroid-extractor's output (per spec §FR-005).
///
/// Args:
///   beacon_world      — beacon position in world frame (meters)
///   beacon_emission_dir — beacon emission cone center axis in world frame (unit vec)
///   beacon_emission_cone_deg — beacon emission cone half-angle (>180 in v1 = omnidirectional)
///   beacon_wavelength_nm — for color-filter response (v1: 850 = L-channel, 940 = R-channel)
///   training_pos      — training craft position in world frame
///   training_orient   — training craft body-to-world quaternion
///   camera            — camera config (mount offset/orient, FOV, projection geometry, etc.)
BeaconProjectionResult project_beacon(
    const Eigen::Vector3f& beacon_world,
    const Eigen::Vector3f& beacon_emission_dir,
    float beacon_emission_cone_deg,
    float beacon_wavelength_nm,
    const Eigen::Vector3f& training_pos,
    const Eigen::Quaternionf& training_orient,
    const CameraConfig& camera);
```

## Algorithm (planar pinhole, v1)

```
Step 1 — World to body frame (training craft):
    beacon_body = training_orient.inverse() * (beacon_world - training_pos)

Step 2 — Body to camera frame:
    beacon_camera = camera.mount_orientation.inverse() *
                    (beacon_body - camera.mount_offset_body)

Step 3 — Behind-camera test:
    if (beacon_camera.x() <= 0):    // crrcsim convention: camera looks +X
        return {0, 0, 0};            // not visible

Step 4 — Pinhole projection:
    fov_h = camera.fov_horiz_deg * PI / 180
    fov_v = camera.fov_vert_deg * PI / 180
    half_tan_h = tan(fov_h / 2)
    half_tan_v = tan(fov_v / 2)
    screen_x = (beacon_camera.y() / beacon_camera.x()) / half_tan_h
    screen_y = (-beacon_camera.z() / beacon_camera.x()) / half_tan_v
    // Note: y-component sign flip because crrcsim Z-axis points down (NED)

Step 5 — FOV test:
    if (|screen_x| > 1 || |screen_y| > 1):
        return {0, 0, 0};            // out of FOV → not visible

Step 6 — Beacon emission cone test:
    los_world = (training_pos + camera.mount_offset_body) - beacon_world
    los_world_unit = los_world.normalized()
    cone_half_rad = (beacon_emission_cone_deg / 2) * PI / 180
    if (acos(beacon_emission_dir.dot(-los_world_unit)) > cone_half_rad):
        return {0, 0, 0};            // beacon facing away → not visible

Step 7 — Channel response (v1: binary IR-color filter):
    if (camera.color_filter == DUAL_PASS_IR):
        // Beacon must match one of the two IR wavelengths
        if (beacon_wavelength_nm == 850 || beacon_wavelength_nm == 940):
            visible = 1
        else:
            visible = 0  // wavelength filtered out
    else:
        visible = 1  // no filter, all beacons visible if other tests passed

Step 8 — Aberrations (v1: identity, all skipped):
    // Future: apply radial / tangential / chromatic distortion to (screen_x, screen_y)

return {screen_x, screen_y, visible};
```

## Validation rules

The function MUST:

| Rule | Why |
|---|---|
| Return `{0, 0, 0}` (with visible=0) when any visibility test fails | FR-007 — NN gets explicit no-signal indicator, not garbage coords |
| Be deterministic — same inputs produce identical outputs across calls | Project determinism invariant |
| Not allocate memory | Called per-tick per-beacon per-camera × ~10⁶+ times per gen — must be alloc-free |
| Use only float arithmetic on Eigen primitives | Match project's gp_fitness type discipline (no doubles in eval path) |
| Handle near-zero `beacon_camera.x()` gracefully | Avoid div-by-zero singularity at the camera plane; behind-camera test catches `≤ 0` already |

## Future-friendly extensibility points

Aberrations + non-pinhole geometries land at Step 8 / Step 4 respectively without touching call sites:

```cpp
// Future Step 4 dispatch by geometry:
switch (camera.geometry) {
    case PLANAR_PINHOLE:    /* current code */
    case FISHEYE_SPHERICAL: /* lat/lon-style projection */
    case CYLINDRICAL:       /* future */
}

// Future Step 8 chain (composable):
apply_radial_distortion(screen_x, screen_y, camera.radial_k1, camera.radial_k2, camera.radial_k3);
apply_tangential_distortion(screen_x, screen_y, camera.tangential_p1, camera.tangential_p2);
apply_chromatic_shift(screen_x, screen_y, beacon_wavelength_nm, camera);
// vignetting affects intensity / detection threshold, not coords
// motion blur is a kernel applied in the FPGA detection stage, not modeled in v1
```

## Test surface

`tests/beacon_projection_tests.cc` (NEW):

| Test | Assertion |
|---|---|
| `BeaconAtCameraOrigin_VisibleAtCenter` | Beacon at training craft's camera position → screen_x=0, screen_y=0, visible=1 |
| `BeaconBehindCamera_NotVisible` | Beacon directly behind training craft → visible=0, screen=(0,0) |
| `BeaconOutsideFOV_NotVisible` | Beacon at 95° angle from camera forward (FOV is 120° → half-fov 60°; beacon outside) → visible=0 |
| `BeaconAtFOVEdge_VisibleAtEdge` | Beacon exactly at FOV edge → visible=1, screen_x≈±1 (within float tolerance) |
| `BeaconNearField_LargeAngularDisplacement` | Target close to camera; beacons project to large displacement; both still in FOV → both visible=1 |
| `BeaconFarField_ScreenCoordsConverge` | Target far from camera; both beacons project to nearly same screen point | visible=1 for both, screen distance < 0.05 |
| `EmissionConeBackside_NotVisible` | Target facing away (beacon emission cone points away from camera) → visible=0 |
| `WrongWavelength_NotVisible` | Beacon at 550 nm (visible green, not in dual-pass IR filter) → visible=0 |
| `Determinism_RepeatCalls` | Same args → byte-identical output across 1000 calls |
| `NoAllocation_HotPath` | Use a custom allocator hook to assert zero allocations during 1M projection calls |

## Performance budget

Per R5 research finding:
- ~30 FLOPs per `project_beacon` call
- 8 calls per NN tick (4 history slots × 2 beacons × 1 camera in v1)
- 240 FLOPs per NN tick for projection
- ~10^6+ calls per gen at 5000 pop × 245 scenarios × ~350 ticks/scenario
- Total per-gen: ~10^9 FLOPs ≈ ~0.3 seconds at 3 GFLOPS effective single-thread → noise compared to per-gen training time

Target: ≤ 5 % per-gen wall-clock overhead vs more-rnn3 baseline.

## Open contract decisions

1. **Beacon emission direction**: in v1, set to "match target body's +Z (up)" so beacons point upward as the aircraft banks. Plausible default; pin in implementation. Future: explicit emission-direction config per beacon.
2. **Channel-response interpretation**: v1 binary (1 channel responds, others 0). Future: continuous response curve (e.g., 850 nm beacon at 0.7 sensitivity in IR-A channel, 0.05 leakage into IR-B channel). Architecture supports the extension by changing `visible` from float (binary 0/1) to per-channel response curve — but the wire format is bound by FR-005's "(x, y, visible)" contract for the NN, so additional per-channel data is internal-only.
