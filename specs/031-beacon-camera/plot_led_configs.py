"""
Compare LED-array radiation patterns for the 031-beacon-camera pod.

Configurations compared:
- 4-on-pyramid (45 deg facet tilt): 4 LEDs on the 4 sloped facets of a square pyramid,
  apex pointing outboard (+X). Beam axes 45 deg off +X, 90 deg apart in azimuth.
- 4-on-pyramid (60 deg facet tilt): same but flatter pyramid, beams more outboard-leaning.
- 5-on-cube (cube-minus-base): 1 LED on the outboard face (+X) plus 4 on the side faces
  (+Y, -Y, +Z, -Z). User's new proposal.

LED per-die intensity model: cos^m(theta) where theta is angle from the LED's beam axis.
For HPBW = 130 deg (Lumileds Luxeon IR Compact 850 nm), m approx 0.805. Approximates the
data-sheet polar curve adequately for design comparison. LEDs do not emit behind their
substrate, so cos(theta) < 0 -> intensity = 0.

The plots show RELATIVE intensity (peak normalized to single-LED on-axis = 1.0). Two
principal cuts are shown per configuration:
- "principal" cut: the great circle containing +X and one LED principal axis (best case)
- "diagonal" cut: the great circle bisecting two adjacent LEDs (worst case / seam)

Output: led_configs_polar.png
"""

import numpy as np
import matplotlib.pyplot as plt


HPBW_DEG = 130.0
M_EXPONENT = np.log(0.5) / np.log(np.cos(np.deg2rad(HPBW_DEG / 2.0)))


def led_intensity(beam_axis, direction):
    """cos^m(theta), clipped at 0. beam_axis and direction are unit 3-vectors."""
    cos_theta = np.dot(beam_axis, direction)
    cos_theta = np.maximum(cos_theta, 0.0)
    return np.power(cos_theta, M_EXPONENT)


def sum_intensity(beam_axes, direction):
    return sum(led_intensity(ax, direction) for ax in beam_axes)


def beam_axes_pyramid(facet_tilt_deg):
    """4 LEDs on a square pyramid, apex along +X. Facet tilt measured from +X axis."""
    t = np.deg2rad(facet_tilt_deg)
    cx, sx = np.cos(t), np.sin(t)
    axes = [
        np.array([cx, sx, 0.0]),
        np.array([cx, 0.0, sx]),
        np.array([cx, -sx, 0.0]),
        np.array([cx, 0.0, -sx]),
    ]
    return [a / np.linalg.norm(a) for a in axes]


def beam_axes_cube_minus_base():
    """5 LEDs: outboard face along +X, plus four side faces along +Y, -Y, +Z, -Z."""
    return [
        np.array([1.0, 0.0, 0.0]),
        np.array([0.0, 1.0, 0.0]),
        np.array([0.0, -1.0, 0.0]),
        np.array([0.0, 0.0, 1.0]),
        np.array([0.0, 0.0, -1.0]),
    ]


def cut_pattern(beam_axes, cut_dirs):
    """Compute intensity for each direction in cut_dirs."""
    return np.array([sum_intensity(beam_axes, d) for d in cut_dirs])


def great_circle(axis_a, axis_b, n=361):
    """Great-circle directions interpolating from axis_a to axis_b around their plane."""
    a = axis_a / np.linalg.norm(axis_a)
    b = axis_b / np.linalg.norm(axis_b)
    # gram-schmidt to get orthogonal b
    b_perp = b - np.dot(a, b) * a
    b_perp = b_perp / np.linalg.norm(b_perp)
    alphas = np.linspace(0.0, 2.0 * np.pi, n)
    return [np.cos(a_i) * a + np.sin(a_i) * b_perp for a_i in alphas], alphas


CONFIGS = [
    ("4-on-pyramid (45 deg tilt)", beam_axes_pyramid(45.0), "tab:blue"),
    ("4-on-pyramid (60 deg tilt)", beam_axes_pyramid(60.0), "tab:orange"),
    ("5-on-cube (square + apex)", beam_axes_cube_minus_base(), "tab:green"),
]


fig, axes = plt.subplots(1, 2, subplot_kw={"projection": "polar"}, figsize=(13, 6.5))


# Cut A: principal great circle through +X and one LED-principal-axis direction (+Y).
# For pyramid: an LED is at azimuth 0 in this cut.
# For cube: the +Y LED is in this cut.
dirs_principal, alphas = great_circle(
    np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])
)

# Cut B: diagonal great circle through +X and (+Y +Z)/sqrt(2) -- bisector of two
# adjacent LED principal axes for both configurations.
diag = np.array([0.0, 1.0, 1.0]) / np.sqrt(2.0)
dirs_diagonal, _ = great_circle(np.array([1.0, 0.0, 0.0]), diag)


for ax, dirs, title in [
    (axes[0], dirs_principal, "Principal cut (through outboard apex + one LED axis)"),
    (axes[1], dirs_diagonal, "Diagonal cut (through outboard apex + LED-seam bisector)"),
]:
    for label, beam_axes, color in CONFIGS:
        intensity = cut_pattern(beam_axes, dirs)
        ax.plot(alphas, intensity, label=label, color=color, linewidth=1.8)

    ax.set_theta_zero_location("E")  # angle 0 = +X (outboard apex) to the right
    ax.set_theta_direction(1)
    ax.set_rmax(2.0)
    ax.set_rticks([0.5, 1.0, 1.5, 2.0])
    ax.set_rlabel_position(135)
    ax.grid(True, alpha=0.5)
    ax.set_title(title, pad=18, fontsize=10)

    # Mark the 135 deg off-axis lines (the 270 deg-spherical-coverage edge).
    for edge_deg in [135.0, -135.0]:
        ax.axvline(
            np.deg2rad(edge_deg), color="red", linestyle="--", alpha=0.6, linewidth=0.8
        )

    # Mark useful-coverage threshold (relative intensity = 0.5, e.g., 50% of single-LED peak).
    threshold = 0.5
    th = np.linspace(0, 2 * np.pi, 361)
    ax.plot(th, np.full_like(th, threshold), color="gray", linestyle=":", alpha=0.6,
            linewidth=0.8)


axes[0].legend(loc="lower center", bbox_to_anchor=(1.1, -0.18), ncol=3, fontsize=9)


fig.suptitle(
    "LED-pod radiation patterns (Luxeon IR Compact, HPBW = 130 deg, m = %.2f)\n"
    "Outboard apex at 0 deg (right); inboard fuselage shadow at +/-180 deg.\n"
    "Red dashed lines = 135 deg off-apex (edge of 270 deg-spherical-coverage target).\n"
    "Dotted gray = relative intensity 0.5 (half single-LED peak)."
    % M_EXPONENT,
    fontsize=10,
)


plt.tight_layout(rect=[0, 0, 1, 0.92])
out = "/home/gmcnutt/autoc/specs/031-beacon-camera/led_configs_polar.png"
fig.savefig(out, dpi=140, bbox_inches="tight")
print(f"Wrote {out}")


def idx_at_deg(deg_value):
    """Index of the great-circle sample closest to the given angle (degrees)."""
    target = np.deg2rad(deg_value % 360.0)
    return int(np.argmin(np.abs(alphas - target)))


def report(angle_deg, label_text):
    print(f"\n{label_text} (alpha = {angle_deg:.0f} deg)")
    print(f"{'Configuration':<32}  {'principal':>10}  {'diagonal':>10}  {'min over sphere':>18}")
    for label, beam_axes, _ in CONFIGS:
        i_p = sum_intensity(beam_axes, dirs_principal[idx_at_deg(angle_deg)])
        i_d = sum_intensity(beam_axes, dirs_diagonal[idx_at_deg(angle_deg)])
        print(f"{label:<32}  {i_p:>10.3f}  {i_d:>10.3f}")


report(0.0, "Intensity at outboard apex centerline")
report(45.0, "Intensity at 45 deg off-apex (mid-shoulder)")
report(90.0, "Intensity at 90 deg off-apex (equator: top/bottom/fore/aft)")
report(120.0, "Intensity at 120 deg off-apex (just inside 270 deg rim)")
report(135.0, "Intensity at 135 deg off-apex (the 270 deg-spherical rim itself)")
report(150.0, "Intensity at 150 deg off-apex (15 deg into the unlit inboard zone)")


# Min-over-full-sphere within the 270 deg-spherical envelope. Sample directions
# on a dense fibonacci sphere, exclude the inboard 90 deg cone.
def fibonacci_sphere(n):
    indices = np.arange(0, n, dtype=float) + 0.5
    phi = np.arccos(1.0 - 2.0 * indices / n)
    theta = np.pi * (1.0 + 5.0 ** 0.5) * indices
    x = np.sin(phi) * np.cos(theta)
    y = np.sin(phi) * np.sin(theta)
    z = np.cos(phi)
    return np.stack([x, y, z], axis=1)


sphere = fibonacci_sphere(4096)
in_envelope = sphere[:, 0] >= np.cos(np.deg2rad(135.0))  # within 135 deg of +X
sphere_lit = sphere[in_envelope]

print("\nIntensity uniformity inside the 270 deg-spherical envelope")
print(f"{'Configuration':<32}  {'min':>8}  {'mean':>8}  {'max':>8}  {'min/max':>8}")
for label, beam_axes, _ in CONFIGS:
    intensities = np.array([sum_intensity(beam_axes, d) for d in sphere_lit])
    print(
        f"{label:<32}  {intensities.min():>8.3f}  {intensities.mean():>8.3f}  "
        f"{intensities.max():>8.3f}  {intensities.min() / intensities.max():>8.3f}"
    )
