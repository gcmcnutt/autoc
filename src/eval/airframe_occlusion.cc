// 040 T013 — chase-airframe obstruction model. See the header for why the
// single-AABB proxy this replaces was both wrong-shaped and degenerate.

#include "autoc/eval/airframe_occlusion.h"

#include <algorithm>
#include <cmath>

namespace autoc::eval {

namespace {

// Inches → metres. The measured station stack is imperial; the sim is metric,
// and doing the conversion once here beats scattering 0.0254 through the
// geometry.
constexpr gp_scalar kIn = static_cast<gp_scalar>(0.0254);

inline constexpr gp_scalar in(double inches) {  // raw-ok: literal-accepting helper; callers pass inch constants, result is gp_scalar
    return static_cast<gp_scalar>(inches) * kIn;
}

}  // namespace

bool rayHitsBox(const gp_vec3& a, const gp_vec3& b,
                const gp_vec3& lo, const gp_vec3& hi) {
    const gp_vec3 d = b - a;
    gp_scalar t_enter = static_cast<gp_scalar>(0);
    gp_scalar t_exit = static_cast<gp_scalar>(1);

    for (int i = 0; i < 3; ++i) {
        const gp_scalar o = a[i];
        const gp_scalar di = d[i];

        if (std::abs(di) < static_cast<gp_scalar>(1e-12)) {
            // Parallel to this slab: a hit is only possible if the origin is
            // already inside it on this axis.
            if (o < lo[i] || o > hi[i]) return false;
            continue;
        }

        gp_scalar t1 = (lo[i] - o) / di;
        gp_scalar t2 = (hi[i] - o) / di;
        if (t1 > t2) std::swap(t1, t2);

        t_enter = std::max(t_enter, t1);
        t_exit = std::min(t_exit, t2);
        if (t_enter > t_exit) return false;
    }

    return true;
}

bool rayCrossesPropDisc(const gp_vec3& camera, const gp_vec3& target,
                        const AirframeObstruction& airframe) {
    const gp_scalar dx = target.x() - camera.x();

    // Ray parallel to the disc plane never crosses it. (A ray lying exactly
    // IN the plane is treated as non-crossing: it grazes at most an edge, and
    // reporting a full disc attenuation for it would overstate the loss.)
    if (std::abs(dx) < static_cast<gp_scalar>(1e-12)) return false;

    // Where along the segment does it reach the disc plane?
    const gp_scalar t = (airframe.prop_plane_x - camera.x()) / dx;

    // Outside the segment — the plane is behind the camera or beyond the
    // target, so nothing is between them.
    if (t < static_cast<gp_scalar>(0) || t > static_cast<gp_scalar>(1)) {
        return false;
    }

    // Crossing point, measured from the thrust axis.
    const gp_scalar y = camera.y() + t * (target.y() - camera.y());
    const gp_scalar z = camera.z() + t * (target.z() - camera.z());
    const gp_scalar dy = y - airframe.prop_axis_y;
    const gp_scalar dz = z - airframe.prop_axis_z;

    return (dy * dy + dz * dz) <= (airframe.prop_radius * airframe.prop_radius);
}

ObstructionResult testObstruction(const gp_vec3& camera,
                                  const gp_vec3& target,
                                  const AirframeObstruction& airframe) {
    ObstructionResult out;
    out.blocked = false;
    out.attenuation = static_cast<gp_scalar>(1);

    if (!airframe.enabled) return out;

    // Opaque solids first — if the wing or nose blocks the ray, the propeller
    // is irrelevant and the beacon is simply not seen.
    if (rayHitsBox(camera, target, airframe.wing_min, airframe.wing_max) ||
        rayHitsBox(camera, target, airframe.nose_min, airframe.nose_max)) {
        out.blocked = true;
        return out;
    }

    // Propeller: partial, never a gate (FR-009).
    if (rayCrossesPropDisc(camera, target, airframe)) {
        out.attenuation =
            std::clamp(static_cast<gp_scalar>(1) - airframe.prop_attenuation,
                       static_cast<gp_scalar>(0),
                       static_cast<gp_scalar>(1));
    }

    return out;
}

AirframeObstruction buildAirframeObstruction(const gp_vec3& camera_mount_body,
                                             const AirframeDimensions& d) {
    AirframeObstruction a;

    const gp_scalar cx = camera_mount_body.x();
    const gp_scalar cy = camera_mount_body.y();
    const gp_scalar cz = camera_mount_body.z();

    // ---- Vertical stack (body NED: +z is DOWN, so "above" is -z) ----------
    // Measured 2026-07-28: wing bottom sits `wing_bottom_above_thrust_in`
    // above the thrust line, and the camera centre `camera_above_wing_bottom_in`
    // above the wing bottom. Working DOWN from the camera:
    const gp_scalar wing_bottom_z = cz + in(d.camera_above_wing_bottom_in);
    const gp_scalar wing_top_z = wing_bottom_z - in(d.wing_thickness_in);
    const gp_scalar thrust_z = wing_bottom_z + in(d.wing_bottom_above_thrust_in);

    // ---- Lateral ---------------------------------------------------------
    // The camera is outboard; the thrust axis is inboard of it. Sign follows
    // the camera's own side so the geometry is correct on either wing.
    const gp_scalar side = (cy >= static_cast<gp_scalar>(0))
                               ? static_cast<gp_scalar>(1)
                               : static_cast<gp_scalar>(-1);
    const gp_scalar thrust_y = cy - side * in(d.camera_outboard_in);

    // ---- Longitudinal ----------------------------------------------------
    // Stations are measured AFT from the disc, body +x is FORWARD, so a
    // station forward of the camera is at greater x.
    const gp_scalar prop_x = cx + in(d.camera_station_in);
    const gp_scalar wing_le_x =
        cx + in(d.camera_station_in - d.wing_le_station_in);
    const gp_scalar wing_te_x = wing_le_x - in(d.wing_chord_in);

    // ---- Propeller -------------------------------------------------------
    a.prop_plane_x = prop_x;
    a.prop_axis_y = thrust_y;
    a.prop_axis_z = thrust_z;
    a.prop_radius = in(d.prop_diameter_in / static_cast<gp_scalar>(2));
    a.prop_attenuation = d.prop_attenuation;

    // ---- Wing — THIN SLAB ------------------------------------------------
    // The thinness is the whole correction: the superseded single-AABB proxy
    // gave a 1" wing a 0.25 m z-extent and called it an airframe.
    //
    // The camera sits mid-thickness IN the leading edge, so the slab must be
    // strictly AFT of the lens or the mount lands on the LE face and every
    // forward ray reads as blocked — the exact degeneracy being removed. The
    // lens protrudes slightly from the foam, which is both physically true and
    // what keeps the geometry non-degenerate.
    const gp_scalar lens_protrusion = in(static_cast<gp_scalar>(0.1));
    const gp_scalar half_span = in(d.wing_span_in / static_cast<gp_scalar>(2));
    a.wing_min = gp_vec3(wing_te_x, thrust_y - half_span, wing_top_z);
    a.wing_max = gp_vec3(std::min(wing_le_x, cx - lens_protrusion),
                         thrust_y + half_span, wing_bottom_z);

    // ---- Pod nose --------------------------------------------------------
    // From the wing leading edge forward to the disc plane, hanging below the
    // thrust line. Dimensions ASSUMED pending the pod measurement, kept modest
    // so a wrong guess under-obstructs rather than silently eating the field.
    const gp_scalar pod_half_width = in(static_cast<gp_scalar>(1.5));
    a.nose_min = gp_vec3(wing_le_x, thrust_y - pod_half_width,
                         thrust_z - in(static_cast<gp_scalar>(1.5)));
    a.nose_max = gp_vec3(prop_x, thrust_y + pod_half_width,
                         thrust_z + in(static_cast<gp_scalar>(1.5)));

    a.enabled = d.enabled;
    return a;
}

AirframeDimensions hb1AirframeDimensions() {
    // Measured 2026-07-28 (operator sketch + photo). The camera is IN the wing
    // leading edge — station 6, not the station-8 max-thickness point an
    // earlier wing-top mount assumed.
    AirframeDimensions d;
    d.camera_station_in = static_cast<gp_scalar>(6.0);
    d.wing_le_station_in = static_cast<gp_scalar>(6.0);
    d.wing_chord_in = static_cast<gp_scalar>(7.0);
    d.wing_span_in = static_cast<gp_scalar>(30.0);
    d.wing_thickness_in = static_cast<gp_scalar>(1.0);
    d.wing_bottom_above_thrust_in = static_cast<gp_scalar>(0.75);
    d.camera_above_wing_bottom_in = static_cast<gp_scalar>(0.5);
    d.camera_outboard_in = static_cast<gp_scalar>(8.0);
    d.prop_diameter_in = static_cast<gp_scalar>(5.5);
    d.prop_attenuation = static_cast<gp_scalar>(0.18);
    // Ships DISABLED: Stage D (T043) moves the mount to the leading edge and
    // turns this on. Enabling it against the legacy centreline mount would
    // model geometry the aircraft does not have.
    d.enabled = false;
    return d;
}

}  // namespace autoc::eval
