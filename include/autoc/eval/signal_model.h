#pragma once

// 040 T057-T059 (FR-014, FR-015, FR-016, FR-019) — the per-beacon link budget.
//
// WHAT THIS REPLACES. Through 039 range did not enter perception at all. The
// only thing modulating confidence was an edge factor — `0.3 × max(|x|, |y|)` —
// so a beacon at 5 m and one at 500 m produced identical observations provided
// they landed on the same pixel. `1/r²` is the single highest-value physical
// term available, and the 031 bench provides a measured flux constant to anchor
// it.
//
// THE CHAIN (data-model.md §4):
//
//   received  = flux_constant × emission(aspect) × (1/r²) × obstruction × optics
//   transfer  = ambient_knee / (ambient_knee + ambient_floor)   [031 field #4]
//   snr_chip  = (received × transfer) / (ambient_floor + noise_floor)
//   snr_chip -= cdma_penalty          when both beacons share a detector element
//
// DETERMINISM (FR-020): pure functions, no PRNG, no clock, no carried state.
// The 031 probability curves enter as thresholds, never as draws — bit-replay is
// a project gate, so sampling is disqualified outright.
//
// ---------------------------------------------------------------------------
// CALIBRATION HONESTY — read this before trusting any absolute number here.
// ---------------------------------------------------------------------------
//
// Three of these values are ASSUMED, and one of them is assumed in a way that
// deserves to be stated plainly rather than buried in a table:
//
// 1. `flux_constant` is PER-EMITTER. The 031 bench figure (1.1-1.6 µA·m²) was
//    measured with FIVE CO-AIMED emitters; the flight enclosure aims its five
//    faces in five directions, so the bench number is not the flight number.
//    We take the midpoint 1.35 and divide by five. R4 records the resulting
//    ~1.4× range overstatement in `optical-link-outcome.md` as a known
//    discrepancy carried as stated uncertainty, not silently absorbed.
//
// 2. `ambient_floor + noise_floor` is BACK-SOLVED, not measured. It is set so
//    that 0 dB SNR lands exactly at `detection_range_m` on beam peak.
//
//    Why not use the measured floor? The 031 decode floor is ≤10 nA. Taken
//    literally against a per-emitter flux of 0.27 µA·m², detection would end at
//    ~5 m and EVERY range past that would sit equally at the floor — a model
//    that is monotonic and completely uninformative. That is the same ~40 dB
//    wide-field shortfall R4 records: the 100 m link budget assumes a narrow
//    camera, and a 120° optic is far down on it.
//
//    FR-033a resolves the tension by ASSERTING the detection envelope rather
//    than deriving it: the sensor is taken as good to the configured range, and
//    the budget shapes QUALITY WITHIN it rather than cutting visibility short.
//    Back-solving the floor is what makes the gradient span the asserted
//    envelope instead of collapsing into its first 5 m. The coupling is pinned
//    by `SignalModel.NoiseFloorIsCoherentWithTheAssertedDetectionRange`, because
//    in the ini the two values look independent and changing one alone would
//    silently move the envelope edge.
//
//    This is the FR-035 calibration target of record for this module. When real
//    optics are chosen and a photon budget exists, this is the number that moves
//    — and per FR-036 it must move WITHOUT structural change.
//
// 3. `q_saturation_db` sets where the hardware's 0-9 quality metric tops out.
//    The 0 and 9 ends are anchored (decode floor, AGC saturation); the span
//    between them is assumed linear-in-dB.

#include "autoc/types.h"

namespace autoc::eval {

// Per data-model.md §4. NO in-class default initializers: this travels in
// WorkerInit, and Constitution VII exists because a stale default that silently
// survives a field addition is exactly the bug class that motivated it. Adding a
// field here must be a compile error at every construction site.
struct SignalConfig {
    // ----- Emitter -----------------------------------------------------------
    gp_scalar flux_constant;    // µA·m², PER EMITTER — see note 1 above
    gp_scalar optics_gain;      // ×, collection optics if fitted

    // Flat-top emission profile (FR-019). Datasheet beam width for the Lumileds
    // DS190: near-constant to the flat angle, half power at the shoulder angle.
    gp_scalar emission_flat_deg;        // gain ≡ 1 inside this
    gp_scalar emission_half_power_deg;  // gain ≡ 0.5 here

    // ----- Receiver ----------------------------------------------------------
    gp_scalar ambient_floor;  // µA, per-scenario draw in US6 (overcast → sun)
    gp_scalar noise_floor;    // µA, fixed sensor term

    // AMBIENT COMPRESSION (031 field test #4, 2026-08-02). µA — the ambient
    // photocurrent at which signal TRANSFER halves.
    //
    // Ambient does two distinct things to a photodiode, and through 040 t1 this
    // model captured only the second:
    //   1. it forward-biases the junction, collapsing dynamic resistance, so
    //      beacon current is SHUNTED AT THE SENSOR  <-- transfer loss, was missing
    //   2. it adds shot noise                        <-- the additive floor above
    //
    // Why it matters: with an additive floor alone, more `flux_constant` or
    // `optics_gain` always buys SNR at any ambient — the model would predict you
    // can OUT-POWER THE SUN. The bench says otherwise. At ~6x emitter current
    // with a bare PD, a shaded sensor locks at ~20 ft and a sun-exposed one
    // fails at any distance; shadow alone flips it at fixed emitter and
    // distance. The loss sits upstream of every downstream multiplier and is far
    // larger than any realistic current increase recovers.
    //
    // ⚠️ ANCHORED TO THE WRONG SENSOR, and knowingly so. Every 031 measurement
    // to date — field test #4 included — is from a SINGLE PHOTODIODE, which
    // integrates the whole field onto one junction. This model is of a 320×240
    // ARRAY, where ambient spreads over 76,800 pixels while the beacon
    // concentrates onto a few: ~48 dB of spatial ambient rejection a PD simply
    // does not have (optics-record.md §3a). Junction forward-bias from
    // whole-field flux is a PD failure mode; an array saturates wells instead.
    //
    // So the STRUCTURE here is right — ambient can compress, and modelling it as
    // additive noise alone let the model claim you can out-power the sun — but
    // this MAGNITUDE is very likely far too pessimistic for an array. It is an
    // ASSUMED value and the upcoming CAMERA trials (not the PD bench) are what
    // will set it. Harmless meanwhile: ambient variation ships at sigma 0 and
    // the knee sits 100× above the nominal floor, so the shipped contribution is
    // −0.09 dB.
    gp_scalar ambient_knee;

    // ----- Decode ------------------------------------------------------------
    gp_scalar cdma_penalty_db;  // dB, cost of sharing a detector element (031 §4)
    gp_scalar q_floor_db;       // dB, SNR at which the 0-9 quality metric reads 0
    gp_scalar q_saturation_db;  // dB, SNR at which it reads 9

    // ----- Envelopes (FR-033) ------------------------------------------------
    // ASSERTED, not emergent (FR-033a). Bearing stays available out to here.
    gp_scalar detection_range_m;
    // Below this the beacon pair no longer resolves as two blobs, so
    // separation-derived range becomes unavailable. Detection is unaffected —
    // FR-016 is emphatic that a shared detector element is field-proven.
    gp_scalar separation_min_px;
    // At or under this pixel gap the two beacons share a detector element:
    // the cdma penalty applies and identity becomes unresolved (FR-017d).
    gp_scalar shared_element_px;

    // 040 — cereal serialize for the WorkerInit RPC carry. RPC-only, never
    // persisted to a dmp, so this is same-rebuild safe.
    template <class Archive>
    void serialize(Archive& ar) {
        ar(flux_constant, optics_gain, emission_flat_deg, emission_half_power_deg,
           ambient_floor, noise_floor, ambient_knee, cdma_penalty_db, q_floor_db,
           q_saturation_db, detection_range_m, separation_min_px,
           shared_element_px);
    }
};

// The shipped hb1 values. A named factory rather than in-class defaults, so the
// values live in ONE place and the struct keeps its Constitution VII property.
SignalConfig hb1SignalConfig();

// ---------------------------------------------------------------------------
// Emission (FR-019).
// ---------------------------------------------------------------------------

// Gain of a SINGLE emitter at `angle_deg` off its own axis.
//
// Flat-top with shoulders — explicitly NOT `cos^m`, which under-reads the flat
// region ~16% and over-reads the skirt. The shoulder is a raised cosine placed
// so gain is exactly 1 at the flat angle, exactly 0.5 at the half-power angle,
// and reaches exactly 0 at twice the shoulder width beyond the flat top.
// Symmetric in sign; identically zero past the zero crossing.
gp_scalar emissionProfile(gp_scalar angle_deg, const SignalConfig& cfg);

// Gain of the whole ENCLOSURE toward `dir_target_body` (need not be unit).
//
// The enclosure is a 1 cm cube with one face against the wingtip, so it emits on
// FIVE faces — outboard, fore, aft, up, down (cube minus base), matching
// `beam_axes_cube_minus_base()` in the 031 analysis script. Contributions sum
// because the emitters are incoherent sources.
//
// THIS MATTERS MOST IN THE GEOMETRY M2 ACTUALLY FLIES. A tail chase sees the
// beacon from aft, which is 90° off the outboard face — deep in the skirt. Model
// only the outboard axis and the dominant case reads ~10× too dim, when in
// reality an aft-facing emitter points straight down the chase's throat.
//
// `outboard_axis_body` is the wingtip's outward normal (±y); the other four
// faces are the remaining body axes.
gp_scalar emissionGain(const gp_vec3& dir_target_body,
                       const gp_vec3& outboard_axis_body,
                       const SignalConfig& cfg);

// ---------------------------------------------------------------------------
// Budget.
// ---------------------------------------------------------------------------

struct SignalResult {
    gp_scalar received_ua;  // µA at the detector, AFTER ambient compression
    gp_scalar transfer;     // ambient-compression factor in (0, 1]; 1 = uncompressed
    gp_scalar snr_db;       // per-chip SNR, cdma penalty already applied
    // The hardware's own quality metric, 0-9, GOOD ≥ 5. Derived from snr_db by a
    // monotonic map. Using the hardware's scale rather than inventing one is
    // what lets a bench capture and a sim playback be compared on the same
    // number (R12 decision 5).
    gp_scalar q;
};

// `emission_gain` comes from emissionGain() above; `obstruction_attenuation` is
// the propeller-disc term the obstruction pass produces (1.0 = clear). The
// attenuation multiplies — it NEVER gates, per FR-009.
SignalResult computeSignal(gp_scalar range_m,
                           gp_scalar emission_gain,
                           gp_scalar obstruction_attenuation,
                           bool shares_detector_element,
                           const SignalConfig& cfg);

// snr_db → the 0-9 metric. Exposed because the acquisition machine consumes it
// directly and must not re-derive its own map.
gp_scalar qFromSnrDb(gp_scalar snr_db, const SignalConfig& cfg);

}  // namespace autoc::eval
