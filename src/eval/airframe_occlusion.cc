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

AirframeObstruction hb1AirframeObstruction() {
    // Measured 2026-07-28 (operator sketch + prop photo), converted to metres
    // once, here. DATUM: prop axle / back of prop at (0, 0, 0); +x forward,
    // +y right wing, +z down; stations run aft (negative x), "up" is -z.
    //
    // The camera sits IN THE WING LEADING EDGE — station 6 aft, 8" outboard,
    // mid-thickness at 1.25" up (0.5" above the wing bottom, which is itself
    // 0.75" up from the axle). That is body (-0.1524, +0.2032, -0.03175).
    AirframeObstruction a;

    // Propeller — 5.5" two-blade (photo-confirmed) ⇒ r = 0.06985 m.
    a.prop_plane_x = static_cast<gp_scalar>(0.0);
    a.prop_axis_y = static_cast<gp_scalar>(0.0);
    a.prop_axis_z = static_cast<gp_scalar>(0.0);
    a.prop_radius = static_cast<gp_scalar>(0.069850);
    // Representative blade-over-pupil duty. CLASSIFIED ASSUMED — depends on
    // entrance pupil and exposure, neither measured (FR-035).
    a.prop_attenuation = static_cast<gp_scalar>(0.18);

    // Wing — THIN SLAB. 7" chord (LE station 6 → TE station 13), 30" span,
    // 1" thick, underside 0.75" up from the axle. The thinness is the whole
    // correction: the superseded single-AABB proxy gave a 1" wing a 0.25 m
    // z-extent and called it an airframe.
    a.wing_min = gp_vec3(static_cast<gp_scalar>(-0.330200),   // TE
                         static_cast<gp_scalar>(-0.381000),   // port tip
                         static_cast<gp_scalar>(-0.044450));  // top (up)
    a.wing_max = gp_vec3(static_cast<gp_scalar>(-0.152400),   // LE
                         static_cast<gp_scalar>(+0.381000),   // starboard tip
                         static_cast<gp_scalar>(-0.019050));  // underside

    // Pod nose — wing leading edge forward to the disc plane, straddling the
    // thrust line. ASSUMED pending the pod measurement; deliberately modest so
    // a wrong guess under-obstructs rather than silently eating the field.
    a.nose_min = gp_vec3(static_cast<gp_scalar>(-0.152400),
                         static_cast<gp_scalar>(-0.038100),
                         static_cast<gp_scalar>(-0.038100));
    a.nose_max = gp_vec3(static_cast<gp_scalar>(0.0),
                         static_cast<gp_scalar>(+0.038100),
                         static_cast<gp_scalar>(+0.038100));

    // Stage D (T043) moves the camera mount to the leading edge and turns this
    // on. Enabling it against the legacy mount would model geometry the
    // aircraft does not have.
    a.enabled = false;
    return a;
}

gp_vec3 hb1LeadingEdgeCameraMount() {
    // Station 6 aft, 8" outboard, 1.25" up — converted from the sketch once.
    //
    // x is 2 mm FORWARD of the wing LE plane (-0.152400), not on it. The lens
    // stands proud of the foam, which is physically true and is also what
    // keeps the geometry non-degenerate: an aperture exactly coincident with
    // the LE face reads as a hit (a surface touch counts), so every forward
    // ray would report blocked — the same failure that made the superseded
    // AABB proxy unusable.
    return gp_vec3(static_cast<gp_scalar>(-0.150400),
                   static_cast<gp_scalar>(+0.203200),
                   static_cast<gp_scalar>(-0.031750));
}

}  // namespace autoc::eval
