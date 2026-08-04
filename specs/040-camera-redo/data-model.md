# Data Model — 040 Camera Redo

**Date**: 2026-07-28 | **Plan**: [plan.md](plan.md) | **Spec**: [spec.md](spec.md)

Entities in the chase-side perception chain. Classification column follows FR-035:
**M** = measured (traceable to a recorded measurement) · **D** = derived (computed from measured) ·
**A** = assumed (placeholder awaiting calibration).

---

## 1. CameraSensor

Replaces the current implicit "two independent FOV numbers" description. FOV becomes **derived**, so field
of view and resolution can no longer disagree (FR-003).

| Field | Unit | Default | Class | Notes |
|---|---|---|---|---|
| `pixels_h`, `pixels_v` | px | 320, 240 | A | 4:3 grid |
| `deg_per_px` | deg | 0.375 | A | the single resolution knob |
| `fov_h_deg`, `fov_v_deg` | deg | **derived** = pixels × deg_per_px ⇒ 120, 90 | D | never set directly |
| `entrance_pupil_mm` | mm | 8.0 | A | pre-031 assumption; feeds propeller attenuation only |
| `exposure_ms` | ms | 1.04 | A | 50% of a 2.08 ms frame; sensitivity, not a fixed result |
| `mount_offset_body` | m | LE, 8″ outboard, ~1″ above thrust line | M | sketch 2026-07-28 |
| `mount_orientation_body` | quat | identity (boresight ∥ thrust line) | M | operator |

**Validation**: `deg_per_px > 0`; `pixels_* > 0`; mount must not lie inside or on the boundary of any
obstruction volume (the defect in the current default — see `AirframeObstruction`).

**Retired**: `frame_rate_hz` (was 30, predates the 20 Hz decision), `latency_ms` (was 0; perception latency
now emerges from acquisition timing), `Projection` enum (representation is fixed — see `BeaconObservation`).

---

## 2. BeaconEmitter

| Field | Unit | Default | Class | Notes |
|---|---|---|---|---|
| `mount_body` | m | ±0.386 (⇒ **0.772 m** separation) | M | 30″ span + 5 mm enclosure offset per tip. **Was ±0.45 — a ~17% error in the sole range channel** |
| `enclosure_size_m` | m | 0.01 | M | 1 cm cube, one face against the tip |
| `emission_axes` | unit vec ×5 | outboard, fore, aft, up, down | M | matches `beam_axes_cube_minus_base()` in the 031 analysis script |
| `flat_half_angle_deg` | deg | 45 | M | flat-top region |
| `half_power_half_angle_deg` | deg | 75 | M | ⇒ 150° FWHM, Lumileds DS190 Table 1 |
| `drive_current_ma` | mA | 306 | M | field; bench is 51 |

**Emission pattern** (FR-019): flat-top with shoulders — near-constant within `flat_half_angle`, falling
through half power at `half_power_half_angle`. **A cosine power law MUST NOT be used.** Total output is the
sum over the five emitters.

**Note**: with 45° flat regions and faces 90° apart, the flat regions *tile* the outboard hemisphere, so
illumination toward the chase is near-uniform — which is why off-axis emission is low-priority.

---

## 3. AirframeObstruction

Replaces `AirframeProxy` (a single AABB). Three primitives, all in chase body frame.

| Primitive | Shape | Source | Class |
|---|---|---|---|
| Wing | thin slab — 30″ span × 7″ chord × 1″ thick, LE at station 6″ | sketch | M |
| Pod nose | box forward of the wing LE | sketch | A (dimensions pending) |
| Propeller disc | static annulus, `r_tip` 2.75″, disc plane at station 0 | photo + sketch | M |

| Field | Unit | Default | Class |
|---|---|---|---|
| `prop_attenuation` | fraction | ~0.15–0.20 | A | representative blade-over-aperture duty |
| `enabled` | bool | **true** | — | was `false`; the old proxy was unusable |

**Critical validation**: the camera origin MUST NOT lie on a primitive boundary. The current default places
the mount at z = −0.05 exactly on `box_min_z`, and a surface touch counts as a hit — so obstruction as
shipped would obstruct nearly every forward ray. **Fix before anything depends on obstruction.**

**Behaviour**: wing and nose are opaque (hard gate). The propeller is a **static angular region applying
partial attenuation** — no engine speed, no blade phase (FR-009).

**At the baseline mount**: wing contributes nothing (nothing sits ahead of the leading edge); propeller
shadow spans ≈41–61° inboard.

---

## 4. SignalBudget *(new)*

Per beacon, per tick. Deterministic; no PRNG anywhere (FR-020).

```
received  = drive × emission(aspect) × (1/r²) × obstruction_attenuation × optics_gain
snr_chip  = received / (ambient_floor + noise_floor)
snr_chip -= cdma_penalty        when both beacons share a detector element
```

| Field | Unit | Default | Class |
|---|---|---|---|
| `flux_constant` | µA·m² | 1.1–1.6 | M | 031 bench; pick one, record which |
| `ambient_floor` | µA | per-scenario draw | A | overcast → direct sun |
| `detection_threshold` | dB | ~0 per chip | M | 031 decode floor ≤10 nA |
| `cdma_penalty` | dB | ≈1 SNR tier | M | 031 §4 |
| `optics_gain` | × | 1.0 | A | collection optics if fitted |

**Known uncertainties, carried explicitly** (not silently absorbed): the bench used five co-aimed emitters
where flight aims five directions (~1.4× range overstatement in the source doc); and the 100 m link budget
assumes a narrow camera, leaving a 120° optic ~40 dB down. Both are FR-035 calibration targets.

---

## 5. AcquisitionState *(new)*

Per beacon. **Internal to perception — never a controller input** (FR-017b). Recorded for diagnostics only.

**States**: `SEARCHING` → `ACQUIRING` → `TRACKING` → `HOLDING` → (`TRACKING` | `SEARCHING`)

Mirrors the shipped gateware FSM rather than inventing one
([`SIM-FEATURES.md`](../../firmware/beacon-decoder-stepfpga/SIM-FEATURES.md)). All timings are
**hardware-measured at N=31** via the decoder's recovery counter.

| Transition | Condition | Timing | ticks @ 20 Hz | Class |
|---|---|---|---:|---|
| SEARCH → ACQUIRING | SNR above threshold | immediate | 0 | M |
| ACQUIRING → LOCKED (**cold**) | rate stale ⇒ needs `MINLOCK` | **308 ms** (≈2 words) | 6.2 | M |
| **HOLD → LOCKED (warm)** | signal returns **inside the coast window** ⇒ flywheel still holds the rate, re-lock on the first good period, **ACQUIRING skipped** | **154 ms** (≈1 word) | 3.1 | M |
| LOCKED → HOLD | signal lost | immediate | 0 | M |
| HOLD → SEARCH | `HOLDMAX` 2 bad periods elapse | **308 ms** | 6.2 | M |
| coast expiry | `time_since_loss` > **10 s** ⇒ next re-acquire is **true-cold** | wallclock | 200 | M |

| Field | Unit | Notes |
|---|---|---|
| `time_in_state` | ms | advanced **analytically** once per 50 ms controller tick — no sub-stepping |
| `time_since_loss` | ms | gates warm vs true-cold at the coast window |
| `q` | 0–9 | `\|corr\|/energy` — **signal-level independent** (AGC-normalised); GOOD ≥ 5. Source of the quality value |

### Why the coast window dominates

The coast window is **wallclock-driven** (emitter↔receiver oscillator stability), *not* code length. The
documented M2 worst-case blind window is **~8 s**, which sits **inside** the ~10 s coast — so in practice
**most M2 reacquisitions are warm (154 ms), not cold (308 ms)**. "31 chips triples acquisition" is true only
of the cold path, which M2 rarely takes. Cold is still modelled: operator reports it does occur in flight
(sun, reflections) at losses beyond ~10 s.

### Advance rule

Chip credit accrues linearly in time, so a 50 ms tick adds 10 chips' worth at the current SNR — computed in
one arithmetic step rather than 24 frame sub-steps. This is exact for constant SNR, **phase-free** (a 154 ms
word against a 50 ms tick is 3.08 — not commensurate, so the code boundary drifts relative to the tick
exactly as free-running hardware does), and keeps perception inside the FR-038 throughput ceiling.
Quantisation is ±1 tick, which SC-005 already states as the tolerance.

**Reset (FR-020a)**: every field resets at each scenario boundary, in **both** the production path and the
test-only reference. Unreset state leaks across scenarios and breaks the bitwise gate — the existing
situational-awareness state carries an explicit warning to this effect.

---

## 6. BeaconObservation *(modified)*

The per-tick perception output. **Controller-facing fields are unchanged in count** — bearing pair plus
quality, so the 58-input vector holds (FR-006).

| Field | Was | Now | Class |
|---|---|---|---|
| bearing x, y | int8 NDC, per-axis normalised | **radians**, isotropic, quantised on the pixel grid | D |
| quality (CEP) | `0.3 × max(\|x\|,\|y\|)` — position only | **signal-derived**: small = confident, large = tentative, sentinel = not visible | D |
| `raw_x_px`, `raw_y_px` | — | **NEW** int16 pixel indices (diagnostic) | D |
| `raw_margin` | — | **NEW** correlation-margin proxy (diagnostic) | D |
| `lock_state` | — | **NEW** tracking state (diagnostic) | D |

**Quality regimes** (FR-017a): confident · tentative (bearing reported, large variance) · not-visible
(distinct sentinel). The sentinel remains distinguishable from any in-range value.

**Serialization**: append at the end of the v≥2 block, no `CEREAL_CLASS_VERSION` bump, old dmps orphaned —
the established in-code convention. Diagnostic fields are cereal byte-format ⇒ `// raw-ok:` annotated
(Principle VI).

---

## 7. CameraVariationDraw *(new)*

Per scenario, from the reserved `camera` sub-seed (slot 5). Modelled on the craft-variation pattern.

| Field | Unit | σ | Clip | Class | Consumer |
|---|---|---|---|---|---|
| `boresight_error` (2 DOF) | deg | 10 | **hard 20** | M | bearing |
| `roll_error` | deg | 10 | **hard 20** | M | bearing — **biases target tilt one-for-one** |
| `mount_translation` | m | within a 1 cm box | ±5 mm | M | **obstruction only** — negligible for bearing (0.03° at 10 m) but swings propeller clearance ~15% |
| `wing_thickness` | in | TBD | — | A | obstruction — folded foam board is "highly variable" |
| `ambient_level` | µA | TBD | — | A | signal budget |

**Rules**: reproducible from the scenario identifier alone; **chase-specific** even when environment seeds
are shared with the target (FR-022); with all sigmas zero, results bit-identical to the no-variation
baseline (FR-023). Raw pre-scale draws recorded in `ScenarioMetadata` so variation is verifiable
ramp-independently.

**Highest-impact term**: roll error. Boresight error lands as a near-constant additive offset under the
angular representation — clean and learnable — whereas roll rotates the image plane and therefore biases
the port→starboard tilt cue degree-for-degree, and tilt feeds the roll command.

---

## Entity relationships

```
CameraVariationDraw ──┬─→ CameraSensor (mount pose)
                      ├─→ AirframeObstruction (translation, wing thickness)
                      └─→ SignalBudget (ambient level)

BeaconEmitter ──→ SignalBudget ──→ AcquisitionState ──→ BeaconObservation
                       ↑                                       │
AirframeObstruction ────┘                                      ↓
                                              controller inputs (58, unchanged)
CameraSensor ──→ bearing quantisation ────────────────────────┘
```

**Envelope rule (FR-033)**: bearing and separation-derived range have **different reach**. Bearing extends
to the design detection range (~100 m); separation subtends ≈1 px there, so range degrades to explicitly
unavailable somewhere around 25 m. Neither may be reported as usable outside its own envelope.
