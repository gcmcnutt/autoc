#pragma once

// 040 T060-T062, T064a (FR-013, FR-017, FR-017a, FR-017b, FR-017d, FR-018,
// FR-020, FR-020a) — the per-beacon acquisition state machine.
//
// WHAT THIS REPLACES. Through 039, acquisition was instantaneous: a beacon that
// became geometrically visible was fully trusted on the same tick. That erased
// the reacquisition cost, which 037's M2 de-risking identified as the actual
// tracking bottleneck — M2 depth is architecture/perception-capped rather than
// reward-limited, and the named next lever is reacquire-through-blindness.
//
// WE MIRROR THE SHIPPED GATEWARE, we do not invent an FSM. States and timings
// come from firmware/beacon-decoder-stepfpga/SIM-FEATURES.md, hardware-measured
// at N=31 via the decoder's recovery counter (research R12). It is already the
// right shape and it is what the hardware does.
//
// ---------------------------------------------------------------------------
// THE BEHAVIOUR-DEFINING PROPERTY
// ---------------------------------------------------------------------------
//
// The coast window is WALLCLOCK-driven — set by emitter↔receiver oscillator
// stability — NOT code-length driven. It is ~10 s, and the documented M2
// worst-case blind window is ~8 s, which sits INSIDE it. Therefore:
//
//   most M2 reacquisitions are WARM (154 ms ≈ 3 ticks), not cold (308 ms ≈ 6)
//
// "N=31 triples acquisition" is true only of the cold path, which M2 rarely
// takes. Sim difficulty hinges on the COAST WINDOW far more than on code length,
// so that is the parameter to vary if reacquisition cost turns out to matter.
//
// ---------------------------------------------------------------------------
// ANALYTIC ADVANCE — why there is no sub-stepping
// ---------------------------------------------------------------------------
//
// Chip credit accrues linearly in time, so one 50 ms controller tick adds 10
// chips' worth at the current SNR in a single arithmetic step rather than 24
// frame sub-steps. This is exact for constant SNR, PHASE-FREE (a 154 ms word
// against a 50 ms tick is 3.08 — not commensurate, so the code boundary drifts
// relative to the tick exactly as free-running hardware does), and 24× cheaper,
// which matters against the FR-038 throughput ceiling. Quantisation is ±1 tick,
// which is the tolerance SC-005 already states.
//
// ---------------------------------------------------------------------------
// WHAT IS AND IS NOT EXPOSED
// ---------------------------------------------------------------------------
//
// FR-017b: the state machine is INTERNAL. No categorical state reaches the
// controller — its effect arrives solely through the quality value, so the
// network infers tracking state from how quality behaves across its history
// window rather than being handed a category. This also keeps the input vector
// at 58 (FR-006). `LockState` is recorded for diagnostics only (FR-028).

#include <cstdint>

#include "autoc/eval/signal_model.h"
#include "autoc/types.h"

namespace autoc::eval {

// The gateware's four states. Values are pinned because they are written to the
// dmp as a diagnostic field; renumbering would silently reinterpret history.
enum class LockState : int8_t {
    SEARCHING = 0,  // no signal, no rate estimate worth keeping
    ACQUIRING = 1,  // signal present, integrating toward lock — TENTATIVE
    TRACKING = 2,   // locked and confirmed
    HOLDING = 3,    // signal lost, flywheel coasting on the held rate
};

// NO in-class default initializers (Constitution VII) — this travels in
// WorkerInit. hb1AcquisitionConfig() is the single home for the values.
struct AcquisitionConfig {
    gp_scalar code_word_ms;     // one N=31 code word — the WARM relock budget
    gp_scalar cold_acquire_ms;  // rate stale, needs MINLOCK — two words
    gp_scalar hold_max_ms;      // HOLDMAX: bad periods before dropping to SEARCH
    gp_scalar coast_window_ms;  // COASTMAX: how long the rate estimate survives

    // Quality regimes (contracts/perception-interface.md §1). Small = confident,
    // large = tentative-but-usable, sentinel = not visible.
    gp_scalar confident_cep;  // cep at q = 9
    gp_scalar tentative_cep;  // cep at q = 0
    // FR-017d — unresolved identity floors quality into the large regime. Tilt
    // SIGN depends on beacon identity while separation does not, so a swapped
    // pair flips tilt by 180°, and tilt drives the roll command. Without this,
    // two cleanly-detected but unidentified blobs would carry LOW cep —
    // confidently wrong tilt with nothing flagging it.
    gp_scalar identity_uncertain_cep;

    // 040 — cereal serialize for the WorkerInit RPC carry. RPC-only, never
    // persisted to a dmp, so this is same-rebuild safe.
    template <class Archive>
    void serialize(Archive& ar) {
        ar(code_word_ms, cold_acquire_ms, hold_max_ms, coast_window_ms,
           confident_cep, tentative_cep, identity_uncertain_cep);
    }
};

AcquisitionConfig hb1AcquisitionConfig();

// Per beacon. Every field resets at each scenario boundary (FR-020a) — unreset
// state leaks across scenarios and breaks the bitwise gate.
struct AcquisitionState {
    LockState state;
    gp_scalar time_in_state_ms;
    gp_scalar time_since_loss_ms;  // gates warm vs true-cold at the coast window
    gp_scalar credit_ms;           // chip credit accrued toward the current lock
    gp_scalar q;                   // 0-9, the hardware's own AGC-normalised metric
    bool ever_locked;              // a fresh scenario has no rate to coast on
    // Was a blob visible on the tick just advanced? Distinct from `state`, and
    // the distinction is load-bearing: HOLDING with signal RETURNED is
    // re-integrating on a blob that is plainly there, so it must report a
    // tentative bearing (FR-017c), whereas HOLDING while still blind has nothing
    // to report. Keying quality off `state` alone conflates the two.
    bool blob_present;

    void reset();
};

// Advance one controller tick.
//
// `signal_present` is GEOMETRIC visibility — in field, not blocked, inside the
// asserted detection range. It deliberately does NOT consult SNR: FR-033a
// asserts the detection envelope rather than deriving it, so signal strength
// shapes quality WITHIN the envelope and never cuts visibility short.
//
// `signal_q` is the 0-9 metric from the link budget (qFromSnrDb).
//
// Deterministic: thresholds and time constants only, no draws (FR-020).
void advanceAcquisition(AcquisitionState& s,
                        bool signal_present,
                        gp_scalar signal_q,
                        gp_scalar dt_ms,
                        const AcquisitionConfig& cfg);

// The interface-facing quality value (FR-014). Returns the CEP sentinel whenever
// there is no blob to report a bearing for — HOLDING is an internal coast, not a
// claim that the beacon is visible.
//
// A TENTATIVE lock reports a real value in the large regime, not the sentinel:
// the blob centroid exists before the code decodes, so a bearing IS available
// pre-lock and the controller is expected to learn the discount rather than be
// handed a gap (FR-017a / FR-017c).
gp_scalar qualityFromAcquisition(const AcquisitionState& s,
                                 bool identity_uncertain,
                                 const AcquisitionConfig& cfg);

}  // namespace autoc::eval
