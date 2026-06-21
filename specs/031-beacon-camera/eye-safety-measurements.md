# Eye-Safety Measurements

**FR ref**: FR-1.7 (eye-safety analysis + manual on/off switch) + FR-3.4 (eye-safety bench verification).
**Phase-1 decision** (Clarifications Session 2026-05-17): formal calibrated-NIR-power-meter IEC 62471 measurement is **deferred** to a future spec (031-integration or any milestone with spectator-exposure / regulatory context). Phase 1 relies on **design-time link-budget analysis + qualitative substitutes + an operator-only 1 m viewing rule**.

This document is the operator's evidence record per FR-3.4 #3 and the US6 pre-flight gate.

---

## 1. Design-time link-budget basis (FR-1.7 table — recap)

At 300 mA per LED, 50% duty, Lumileds Luxeon IR Compact L1IZ-0850000000000 (130° HPBW):

| Distance | Time-avg irradiance | Source radiance | Classification |
|---|---|---|---|
| 200 mm direct viewing | ~3 W/m² | ~5×10⁴ W/m²/sr | RG0 for ≤10 s; RG1 for sustained ≥1 min direct staring |
| 500 mm | ~0.5 W/m² | (same source radiance) | RG0 for ≤100 s; borderline for extended viewing |
| 1 m direct viewing | ~0.12 W/m² | (same source radiance) | **RG0 unconditionally** |
| ≥10 m (flight distance) | <10 mW/m² | trivially below any threshold | **RG0 unconditionally** |

The FR-1.7 link-budget calculation IS the design-time eye-safety analysis. Path C's 300 mA single-mode is **5× below** the prior 1 A design, giving ~10× exposure-time safety margin at any distance.

---

## 2. Operator-only 1 m minimum viewing rule

**Rule**: during all bench + flight work in Phase 1, the operator + any other observers SHALL maintain ≥1 m minimum distance from any pod with batteries inserted.

At ≥1 m the design is **unconditionally RG0** per the FR-1.7 table — for any exposure time, by anyone.

**Implementation**:
- Pre-flight: insert pod batteries while pods are on a workbench at arm's length, then immediately step back to ≥1 m before observing the LED. Do NOT lean over the pod with batteries inserted.
- Battery swap between flights: same protocol — handle pod close-range only when pulling battery (LED off).
- No spectators within 1 m of the active flight area during Phase-1 flights.

This eliminates the close-range exposure regime entirely; the FR-1.7 RG0-unconditional-at-1m result is binding.

---

## 3. Smartphone-IR-camera qualitative check (FR-3.4 #3 substitute)

**Protocol** (per-pod, performed once at pod hand-build acceptance per US1 task T037):

1. Place pod on workbench. Insert charged 1S battery.
2. Hold a typical smartphone (any model from ~2018+ has IR-camera sensitivity) ~200 mm from the pod, with the rear camera pointing at the apex LED face.
3. Open the smartphone's camera app. Observe the pod through the live viewfinder.

**Expected results**:
- ✅ **PASS**: pod apex LED appears as a bright (but not pixel-saturated) spot in the smartphone viewfinder. The 5 LEDs may visibly modulate at 100 Hz (the chip rate) — the smartphone's framerate (typically 30 fps) aliases against the chip rate producing a visible flicker pattern.
- ❌ **FAIL — too dim**: LED barely visible. Indicates the LED is not emitting at the design power level. INVESTIGATE: check sense-resistor value, boost-driver output, LED orientation (could be misaligned with the indent and emitting sideways).
- ❌ **FAIL — saturated**: LED produces a pixel-saturated blob the size of a fist in the smartphone viewfinder. Indicates the LED is emitting WAY above the design current. INVESTIGATE: check for sense-resistor short, wrong R value, boost-driver mis-config.

A PASS confirms the LED is emitting at approximately the expected radiant flux — not a calibrated measurement, but a strong qualitative check that the link-budget assumptions hold.

---

## 4. Per-pod measurement log

| Pod ID | Code ID | Build date | Operator | Smartphone-IR result | 1m-rule attestation | Notes |
|---|---|---|---|---|---|---|
| (template — populate at T037 per pod) | | | | PASS / FAIL / N/A | YES / NO | |

---

## 5. When the calibrated measurement SHALL be performed

Procure a calibrated NIR power meter (Thorlabs S130C + PM100D, Newport 1830-R, or equivalent — $500-2000) and perform the formal IEC 62471 measurement at 200 mm BEFORE any of the following:

- Spectators at uncontrolled flying-field distances
- Spectators viewing the active pod at <1 m
- Children or eye-vulnerable populations in the operational area
- Any regulatory compliance certification (FAA, FCC, CE, etc.)
- Commercial deployment

These conditions trigger 031-integration's eye-safety re-evaluation. Phase 1's operator-only context exempts the formal measurement.

---

## 6. Cross-references

- [spec.md FR-1.7](./spec.md) — eye-safety analysis table + UVLO contract
- [spec.md FR-3.4](./spec.md) — bench verification procedure
- [spec.md Clarifications Session 2026-05-17](./spec.md) — Phase-1 deferral rationale
- [spec.md US6 pre-flight safety gate](./spec.md) — operational use of this document
