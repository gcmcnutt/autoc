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

struct BeaconConfig {
    uint16_t wavelength_nm = 850;          // distinct per-beacon (e.g. 850 vs 940)
    gp_scalar emission_cone_deg = 270.0f;  // FULL angular extent (half-angle = 135°)

    // Mount in target body frame. hb1 wingtips: ±0.45 m on body Y.
    gp_vec3 mount_body{0.0f, -0.45f, 0.0f};

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
