# Contract: Beacon projection API (FR-005 + FR-007 + FR-017)

**Producer**: `src/eval/camera_projection.cc` (NEW — M5 deliverable)
**Consumer**: `TrackerStepper` per-tick path (M6); recorder for `cameraViewList` in M2 dmp (M8)

Refreshed 2026-05-04 for the `(x, y, CEP)` perception interface (was `(x, y, visible)` in the prior version).

## Surface

```cpp
namespace autoc::eval {

struct BeaconObservation {
  // NN-facing dequantized values (fp32 at the boundary):
  float screen_x;       // [-1, +1] uncalibrated screen-relative; arbitrary if invisible
  float screen_y;       // same
  float cep;            // [0, 1.0] for visible, == kCepSentinelFloat (1.5f) for invisible

  // For dmp / renderer:
  int8_t raw_x_int8;
  int8_t raw_y_int8;
  int8_t raw_cep_int8;  // INT8_MIN ⇔ invisible
};

struct ProjectionInput {
  gp_vec3 chase_position_world;
  gp_quat chase_orientation_world;       // body→world
  gp_vec3 target_position_world;
  gp_quat target_orientation_world;
  // beacon-mount in target body frame:
  gp_vec3 beacon_mount_target_body;
  gp_vec3 beacon_emission_axis_target_body;
  // camera mount on chase:
  gp_vec3 camera_mount_chase_body;
  gp_quat camera_orientation_chase_body;
  // configs:
  CameraConfig camera;
  BeaconConfig beacon;
  // self-occlusion:
  AirframeProxy chase_airframe;          // coarse body shape for self-occlusion check
};

BeaconObservation projectBeacon(const ProjectionInput& input);

// Helper: int8 quantization round-trip primitives (per R7)
int8_t quantize_xy(float v_in_minus_one_plus_one);
int8_t quantize_cep(float cep_in_zero_one_or_sentinel);
float dequantize_xy(int8_t q);
float dequantize_cep(int8_t q);

constexpr float kCepSentinelThreshold = 1.25f;
constexpr float kCepSentinelFloat = 1.5f;

}  // namespace autoc::eval
```

## Projection math (per R3 carry-forward + R6 + R7)

```
1. Compute beacon position in world frame:
     beacon_world = target_position
                  + (target_orientation_world * beacon_mount_target_body)

2. Transform to chase body frame:
     beacon_in_chase_body = chase_orientation_world.inverse() * (beacon_world - chase_position_world)

3. Transform to camera frame (apply camera mount offset + orientation):
     beacon_in_camera = camera_orientation_chase_body.inverse()
                        * (beacon_in_chase_body - camera_mount_chase_body)

4. Sentinel checks (each → invisible):
     a. behind camera: beacon_in_camera.z <= 0       → cep = sentinel
     b. occluded by self-airframe: ray (camera origin → beacon_in_camera)
        passes through chase_airframe proxy box      → cep = sentinel
     c. beacon emission cone: angle between beacon_emission_axis and
        (chase_position - beacon_world) > 270°/2     → cep = sentinel
     d. behind FOV: |u| > tan(fov_h/2) OR |v| > tan(fov_v/2)
        where u = bx/bz, v = by/bz                   → cep = sentinel

5. Otherwise compute screen coordinates:
     u = beacon_in_camera.x / beacon_in_camera.z
     v = beacon_in_camera.y / beacon_in_camera.z
     fov_limit_h = tan(fov_h_deg * π / 360)
     fov_limit_v = tan(fov_v_deg * π / 360)
     screen_x = u / fov_limit_h            // normalized [-1, +1]
     screen_y = v / fov_limit_v

6. Compute CEP (R6 v1 linear, deferred refinement):
     base_cep = 0.0  // ideal centroid
     edge_factor = max(|screen_x|, |screen_y|)  // 0 at center, 1 at edge
     cep = clamp(base_cep + edge_factor * 0.3, 0.0, 1.0)
     // Future v2+: add aberration zones, motion blur estimate, etc.

7. Quantize to int8 (FR-017, R7):
     raw_x_int8 = quantize_xy(screen_x)
     raw_y_int8 = quantize_xy(screen_y)
     raw_cep_int8 = quantize_cep(cep)        // INT8_MIN if sentinel

8. Dequantize for NN-facing fp32 output:
     screen_x = dequantize_xy(raw_x_int8)
     screen_y = dequantize_xy(raw_y_int8)
     cep = dequantize_cep(raw_cep_int8)
```

## Round-trip property (contract test)

For any visible `(x_in, y_in, cep_in)` in valid range:
- `dequantize_xy(quantize_xy(x_in))` is within `1/127.0 ≈ 0.0079` of `x_in` (one int8 step).
- `dequantize_cep(quantize_cep(cep_in))` is within `1/127.0` of `cep_in` provided `cep_in < kCepSentinelThreshold`.

For invisibility:
- `dequantize_cep(quantize_cep(any_value >= 1.25))` exactly equals `kCepSentinelFloat (1.5f)`.

## Determinism contract (FR-009)

Same `ProjectionInput` ⇒ bit-identical `BeaconObservation` across invocations. Math uses `gp_scalar` (Eigen) operations; no PRNG, no system clock, no thread-dependent state.

## Self-occlusion proxy (D10)

`AirframeProxy` v1 = axis-aligned box in chase body frame approximating fuselage + wing extents. Ray from `camera_mount_chase_body` to `beacon_in_chase_body` tested for box intersection; if it hits, beacon registers as occluded (sentinel).

```cpp
struct AirframeProxy {
  gp_vec3 box_min_chase_body;
  gp_vec3 box_max_chase_body;
};

bool rayHitsProxy(const gp_vec3& ray_origin_chase_body,
                  const gp_vec3& ray_target_chase_body,
                  const AirframeProxy& proxy);
```

V1 default proxy: hb1 dimensions, calibrated against operator's reference video of actual occlusion footprint.

## Test coverage

`tests/beacon_projection_tests.cc` (M5 deliverable):
- Target dead-ahead at various distances → `cep` near 0, `(x, y)` near `(0, 0)`.
- Target left-edge / right-edge of FOV → `cep` elevated, `screen_x` approaching `±1`.
- Target behind chase camera → sentinel.
- Target occluded by airframe proxy → sentinel.
- Target outside emission cone (270° outward, target tail-on aspect) → sentinel.
- int8 round-trip determinism: 10 random `(x, y, cep)` triples; assert round-trip within tolerance.
- Sentinel exactly equals `kCepSentinelFloat`.

## Citations

- 030 spec FR-003 (camera config)
- 030 spec FR-004 (beacon config)
- 030 spec FR-005 (projection output)
- 030 spec FR-007 (sentinel handling)
- 030 spec FR-017 (int8 quantization)
- 030 spec D10 (camera v1 baseline + self-occlusion)
- research.md R3 carry-forward (projection math)
- research.md R6 (CEP encoding)
- research.md R7 (int8 quantization math)
- `src/nn/evaluator.cc:343-356` (existing inverse-quat * vec3 body-frame transform pattern)
