// 040 T060-T062, T064a — per-beacon acquisition state machine. See
// acquisition_state.h for the timing model and why the coast window, not the
// code length, is the parameter that decides sim difficulty.

#include "autoc/eval/acquisition_state.h"

#include <algorithm>

#include "autoc/eval/camera_projection.h"  // kCepSentinelFloat

namespace autoc::eval {

namespace {

// A fresh scenario has never locked, so its "time since loss" must read as
// beyond any coast window — otherwise the first acquisition of a scenario would
// be warm, and every scenario would inherit a rate estimate it never earned.
constexpr gp_scalar kNeverLocked = static_cast<gp_scalar>(1e9);

// Whether the flywheel still holds a usable rate estimate. This single
// expression is the warm/cold gate, and it is deliberately the SAME test from
// both HOLDING and SEARCHING: the rate survives on wallclock, so dropping to
// SEARCH after HOLDMAX does not by itself make the next lock cold.
bool coastStillValid(const AcquisitionState& s, const AcquisitionConfig& cfg) {
    return s.ever_locked && s.time_since_loss_ms <= cfg.coast_window_ms;
}

gp_scalar requiredCreditMs(const AcquisitionState& s, const AcquisitionConfig& cfg) {
    return coastStillValid(s, cfg) ? cfg.code_word_ms : cfg.cold_acquire_ms;
}

}  // namespace

AcquisitionConfig hb1AcquisitionConfig() {
    AcquisitionConfig c;
    // N=31 at 200 Hz chips. (N=63 reads 315/629 ms — a VALUE change, not a
    // re-derivation, which is why code length is configured rather than baked.)
    c.code_word_ms = static_cast<gp_scalar>(154.0);
    c.cold_acquire_ms = static_cast<gp_scalar>(308.0);
    c.hold_max_ms = static_cast<gp_scalar>(308.0);   // HOLDMAX = 2 bad periods
    c.coast_window_ms = static_cast<gp_scalar>(10000.0);  // COASTMAX = 65

    c.confident_cep = static_cast<gp_scalar>(0.02);
    c.tentative_cep = static_cast<gp_scalar>(1.0);
    c.identity_uncertain_cep = static_cast<gp_scalar>(0.75);
    return c;
}

void AcquisitionState::reset() {
    state = LockState::SEARCHING;
    time_in_state_ms = static_cast<gp_scalar>(0);
    time_since_loss_ms = kNeverLocked;
    credit_ms = static_cast<gp_scalar>(0);
    q = static_cast<gp_scalar>(0);
    ever_locked = false;
    blob_present = false;
}

void advanceAcquisition(AcquisitionState& s,
                        bool signal_present,
                        gp_scalar signal_q,
                        gp_scalar dt_ms,
                        const AcquisitionConfig& cfg) {
    s.blob_present = signal_present;

    if (!signal_present) {
        // ----- Blind ---------------------------------------------------------
        switch (s.state) {
            case LockState::TRACKING:
                // FR-018: loss passes through a hold interval before tracking is
                // reported lost. The flywheel keeps the rate; the clock starts.
                s.state = LockState::HOLDING;
                s.time_in_state_ms = static_cast<gp_scalar>(0);
                s.time_since_loss_ms = static_cast<gp_scalar>(0);
                s.credit_ms = static_cast<gp_scalar>(0);
                break;
            case LockState::ACQUIRING:
                // Lost before lock: there is nothing to coast on, so the partial
                // integration is discarded rather than banked.
                s.state = LockState::SEARCHING;
                s.time_in_state_ms = static_cast<gp_scalar>(0);
                s.credit_ms = static_cast<gp_scalar>(0);
                break;
            case LockState::HOLDING:
            case LockState::SEARCHING:
                // Any partial re-integration is discarded on the next bad period.
                s.credit_ms = static_cast<gp_scalar>(0);
                break;
        }

        s.time_in_state_ms += dt_ms;
        // Saturating, so a long blind stretch cannot roll a float into nonsense
        // and accidentally revalidate the coast.
        s.time_since_loss_ms =
            std::min(s.time_since_loss_ms + dt_ms, kNeverLocked);

        if (s.state == LockState::HOLDING && s.time_in_state_ms >= cfg.hold_max_ms) {
            s.state = LockState::SEARCHING;
            s.time_in_state_ms = static_cast<gp_scalar>(0);
        }

        // Quality decays across the hold window. Not visible on the interface —
        // qualityFromAcquisition returns the sentinel while blind — but it is
        // what drives the renderer's fading HOLD ring, which is how a human
        // reads warm-coast-then-expire on playback.
        if (s.state == LockState::HOLDING && cfg.hold_max_ms > static_cast<gp_scalar>(0)) {
            const gp_scalar frac = std::clamp(
                static_cast<gp_scalar>(1) - s.time_in_state_ms / cfg.hold_max_ms,
                static_cast<gp_scalar>(0), static_cast<gp_scalar>(1));
            s.q *= frac;
        } else if (s.state == LockState::SEARCHING) {
            s.q = static_cast<gp_scalar>(0);
        }
        return;
    }

    // ----- Signal present ----------------------------------------------------
    switch (s.state) {
        case LockState::SEARCHING:
            // SEARCH → ACQUIRING is immediate (0 ticks) per the measured table.
            s.state = LockState::ACQUIRING;
            s.time_in_state_ms = static_cast<gp_scalar>(0);
            s.credit_ms = static_cast<gp_scalar>(0);
            break;
        case LockState::HOLDING:
            // Stay in HOLDING and integrate. The gateware re-locks on the first
            // good period with the flywheel still turning — ACQUIRING is
            // SKIPPED, which is precisely what makes warm cheaper than cold. The
            // hold timer freezes because HOLDMAX counts BAD periods.
            break;
        case LockState::ACQUIRING:
        case LockState::TRACKING:
            break;
    }

    if (s.state == LockState::TRACKING) {
        s.time_in_state_ms += dt_ms;
        s.time_since_loss_ms = static_cast<gp_scalar>(0);
        s.q = signal_q;
        return;
    }

    // ACQUIRING or HOLDING-with-signal: accrue chip credit analytically.
    s.credit_ms += dt_ms;
    s.time_in_state_ms += dt_ms;

    const gp_scalar required = requiredCreditMs(s, cfg);
    if (s.credit_ms >= required) {
        s.state = LockState::TRACKING;
        s.time_in_state_ms = static_cast<gp_scalar>(0);
        s.time_since_loss_ms = static_cast<gp_scalar>(0);
        s.credit_ms = static_cast<gp_scalar>(0);
        s.ever_locked = true;
        s.q = signal_q;
        return;
    }

    // FR-017a — a tentative lock is reported, with quality ramping toward
    // confirmed as the code integrates. The controller is not starved during the
    // acquisition window; it gets an early, explicitly untrusted fix.
    const gp_scalar ramp =
        (required > static_cast<gp_scalar>(0))
            ? std::clamp(s.credit_ms / required, static_cast<gp_scalar>(0),
                         static_cast<gp_scalar>(1))
            : static_cast<gp_scalar>(1);
    s.q = signal_q * ramp;
}

gp_scalar qualityFromAcquisition(const AcquisitionState& s,
                                 bool identity_uncertain,
                                 const AcquisitionConfig& cfg) {
    // No blob ⇒ no bearing. A blind HOLDING coasts a RATE, not a position:
    // reporting a bearing there would be inventing data the sensor does not
    // have. Note this keys off `blob_present`, NOT off the state — a HOLDING
    // beacon whose signal has RETURNED is re-integrating on a visible blob and
    // must report it (FR-017c).
    if (!s.blob_present) return kCepSentinelFloat;

    // q (0-9, GOOD ≥ 5) → cep, linear between the two anchors.
    const gp_scalar t = std::clamp(s.q / static_cast<gp_scalar>(9),
                                   static_cast<gp_scalar>(0), static_cast<gp_scalar>(1));
    gp_scalar cep = cfg.tentative_cep + t * (cfg.confident_cep - cfg.tentative_cep);

    if (identity_uncertain) {
        // FR-017d: floor, not scale. A confident POSITION with an unknown
        // IDENTITY must not be able to present as confident, because tilt sign
        // rides on identity and tilt drives the roll command.
        cep = std::max(cep, cfg.identity_uncertain_cep);
    }

    // Must stay strictly inside the in-range band so the sentinel remains
    // distinguishable from every real value (contract §1).
    return std::clamp(cep, static_cast<gp_scalar>(0), static_cast<gp_scalar>(1));
}

}  // namespace autoc::eval
