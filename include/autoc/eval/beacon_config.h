#pragma once

// 030 M5 — Beacon configuration (FR-004).
//
// Per-beacon mount + emission parameters. v1 baseline: two beacons mounted
// at the wingtips (left = body -y, right = body +y), each with a 270°
// outward emission cone, distinct IR wavelengths.
//
// Coordinate convention: target body frame, NED (+x forward, +y right,
// +z down). `mount_body` is the beacon's position relative to the target
// craft body origin; `emission_axis_body` is the unit vector along which
// the beacon's emission cone is centered (outward at the wingtip = ±y).

#include <cstdint>

#include "autoc/types.h"

namespace autoc::eval {

// 040 T030 (FR-004) — MEASURED beacon separation, replacing the 0.9 m that
// shipped through 039.
//
// The article is a 30″ (0.762 m) span with the emitter enclosure adding ~5 mm
// outboard at each tip ⇒ 0.772 m between emitter centres.
//
// This single constant carried a ~17% error into the ONLY channel that
// conveys range: the controller has no rangefinder, and infers distance
// entirely from how wide the beacon pair subtends. Overstating the physical
// separation by 17% overstates every inferred range by 17%, uniformly, in a
// way no amount of training can correct — the network learned a consistent
// mapping to the wrong metre. Correcting it is the largest single fidelity
// fix in 040, and the reason US2 is the MVP.
constexpr gp_scalar kBeaconSeparationM = static_cast<gp_scalar>(0.772);
constexpr gp_scalar kBeaconMountY = kBeaconSeparationM / static_cast<gp_scalar>(2);  // 0.386

struct BeaconConfig {
    uint16_t wavelength_nm = 850;          // distinct per-beacon (e.g. 850 vs 940)
    gp_scalar emission_cone_deg = 270.0f;  // FULL angular extent (half-angle = 135°)

    // Mount in target body frame, at the wingtips: ±kBeaconMountY on body Y.
    gp_vec3 mount_body{0.0f, -kBeaconMountY, 0.0f};

    // Emission axis (unit) in target body frame. Outward at wingtip:
    //   left  beacon: (0, -1, 0)
    //   right beacon: (0, +1, 0)
    gp_vec3 emission_axis_body{0.0f, -1.0f, 0.0f};

    // 030 M6e — cereal serialize for EvalData wire-protocol carry.
    template <class Archive>
    void serialize(Archive& ar) {
        ar(wavelength_nm, emission_cone_deg, mount_body, emission_axis_body);
    }
};

}  // namespace autoc::eval
