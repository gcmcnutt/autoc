#!/usr/bin/env python3
"""16 mm M12 lens + 850 nm bandpass + PC-mount PD: link budget and range. (031, 2026-08-04)

WHY THIS EXISTS. Field test #4 established that ambient COMPRESSION at the photodiode,
not photon shortage, is the wall -- you cannot out-power the sun with a bare PD. The
consequence recorded in the bench journal is that the 850 nm bandpass is a GATE, not an
optimisation. This script asks the next question: with the lens + filter + PD rig now on
order, what range should we theoretically get, and what does the AGC have to cope with?

It also settles a specific emitter question: on a cube, how many emitters are effectively
visible at once, and how does ONE die at 306 mA compare with FIVE at 51 mA?

EVERYTHING IS ANCHORED TO MEASUREMENT, not to datasheet absolutes. The anchor is the
031 bench lock at 41 ft with 5 co-aimed L1IZ at 51 mA into a bare BPV10NF. Absolute
LED radiant flux is deliberately NOT used -- a datasheet cross-check of the implied
irradiance disagrees with the bench constant by ~37x (see NOTE below), so ratios
anchored on the bench are trustworthy where absolutes are not.

NOTE -- the 37x. flux_constant 0.27 uA*m^2 per emitter at 51 mA implies ~0.63 W/m^2 at
1 m for a bare 0.78 mm^2 PD at 0.55 A/W. A 51 mA L1IZ into its ~3.5 sr beam gives only
~0.017 W/m^2 -- more than the LED's entire electrical input. So either the detector has
internal gain (the BOM calls BPV10NF an "NPN phototransistor"; Vishay's part is a PIN
photodiode -- an unresolved conflict in our own docs), the drive is pulsed with a high
peak, or the constant folds in TIA gain. IT DOES NOT MATTER for range ratios, because
the optical gains below are all multiplicative on whatever the detector actually does.
Resolve it before trusting any ABSOLUTE current in section 5.

Usage:  python3 lensed_pd_range.py
"""

import math

# ---------------------------------------------------------------------------
# Measured anchors -- 031 bench + field. Change these only with new measurement.
# ---------------------------------------------------------------------------
K_5COAIMED = 1.35      # uA*m^2, five CO-AIMED L1IZ @ 51 mA (bench midpoint 1.1-1.6)
I_BENCH_MA = 51.0      # mA per die at that measurement
LOCK_INDOOR_M = 12.5   # 41 ft bare-to-bare indoor lock, bare BPV10NF
LOCK_OUTDOOR_M = 5.25  # field test #3 midpoint, 15-20 ft, bare PD, NO filter, full sun
I_FLIGHT_MA = 306.0    # flight cube drive per die (bench journal item 6)

# Back out the effective decode floor from the indoor lock rather than using the
# "<=10 nA" upper bound -- the lock distance IS the floor measurement.
FLOOR_INDOOR_UA = K_5COAIMED / LOCK_INDOOR_M**2
FLOOR_OUTDOOR_UA = K_5COAIMED / LOCK_OUTDOOR_M**2

# ---------------------------------------------------------------------------
# Emission profile -- exact port of src/eval/signal_model.cc emissionProfile().
# Lumileds DS190 flat-top: flat to 45 deg, half power at 75 deg, zero at 105 deg.
# ---------------------------------------------------------------------------
FLAT_DEG, HALF_DEG = 45.0, 75.0


def emission_profile(angle_deg):
    a, flat = abs(angle_deg), FLAT_DEG
    shoulder = HALF_DEG - flat
    if a <= flat:
        return 1.0
    t = (a - flat) / shoulder
    if t >= 2.0:
        return 0.0
    return 0.5 * (1.0 + math.cos(math.pi * t / 2.0))


def cube_gain(direction):
    """Sum of the 5 cube faces (outboard +y, plus +-x, +-z). Port of cubeEmissionGain."""
    axes = [(0, 1, 0), (1, 0, 0), (-1, 0, 0), (0, 0, 1), (0, 0, -1)]
    n = math.sqrt(sum(c * c for c in direction))
    d = [c / n for c in direction]
    total = 0.0
    for ax in axes:
        c = max(-1.0, min(1.0, sum(a * b for a, b in zip(d, ax))))
        total += emission_profile(math.degrees(math.acos(c)))
    return total


# ---------------------------------------------------------------------------
# Optics
# ---------------------------------------------------------------------------
F_MM = 16.0
PDS = {"BPV10NF": 0.78, "BPW34": 7.50}          # mm^2 active area
# C-14 (beacon-order-03.md) puts BPW34 at the focal plane: flat 2.65 mm chip, no dome.
# The domed BPV10NF is the BARE/finder sensor only -- its built-in optic fights an
# external lens. BPV10NF is kept below purely as a what-if on a narrower field.
PD_PRIMARY = "BPW34"
# CRITICAL: K_5COAIMED was measured on a BARE BPV10NF, so it already encodes that
# detector's 0.78 mm^2. Aperture gain must therefore be referenced to the ANCHOR
# detector, NOT to whichever PD is fitted now -- referencing it to BPW34 would
# double-count the area and understate range ~3x. PD_PRIMARY sets FOV and ambient;
# ANCHOR_PD sets signal gain. They are deliberately different detectors.
ANCHOR_PD = "BPV10NF"
FNUMS = [1.6, 2.0, 2.4]                         # C-14 orders "16 mm f/1.6-2.0"

# Measured bare-BPW34 ambient photocurrent (collector-schematic-daylight.md pedestal
# table). These are MEASUREMENTS -- prefer them over deriving ambient from first
# principles.
AMBIENT_BARE_UA = {"direct sun in FOV": (800.0, 1100.0),
                   "sunny, not sun-facing": (30.0, 300.0),
                   "indoor bench": (0.0, 1.0)}

# Option-C load resistor ceilings (collector-schematic-daylight.md): the PD compresses
# as V_ped -> 3.3 V, so I_ped_max = 3.3 / R1.
R_LOADS = {"47 k": 70.0, "22 k": 150.0, "10 k": 330.0}   # uA pedestal ceiling

T_LENS, T_FILT_PEAK = 0.85, 0.90                # in-band transmission
PD_BAND_NM = 270.0                              # 780-1050 nm PD/daylight-filter window
# The ELP lens ships an INTEGRATED "850 nm IR narrow pass" filter. Vendor does not
# publish FWHM; camera-market "narrow pass" is typically 40-60 nm, not the 10 nm of a
# scientific bandpass. Carried as a range until measured.
FILTERS = {"10 nm (Edmund/Thorlabs)": 11.0, "30 nm": 33.0,
           "50 nm (likely ELP)": 50.0, "none": PD_BAND_NM}

# Solar spectral irradiance at ground, AM1.5. ~0.9 W/m^2/nm at the 850 nm window;
# ~0.65 average across the PD's 780-1050 band (the 940 nm water band eats a chunk).
# Cross-check: 270 nm x 0.65 = 176 W/m^2 = 17.6 mW/cm^2, matching the bench journal's
# measured "full sun ~15-20 mW/cm^2 in the PD's wide band".
SOLAR_AT_850_W_M2_NM = 0.90
N_EFF_FILTER = 1.7      # effective index of a dielectric stack, for the AOI blue-shift


def pupil_area_mm2(fnum):
    return math.pi * (F_MM / fnum / 2.0) ** 2


def fov_deg(area_mm2):
    """Full FOV from detector extent. Square-equivalent side, and its diagonal."""
    side = math.sqrt(area_mm2)
    return (2 * math.degrees(math.atan(side / (2 * F_MM))),
            2 * math.degrees(math.atan(side * math.sqrt(2) / (2 * F_MM))))


def db(x):
    return 10.0 * math.log10(x)


# ---------------------------------------------------------------------------
print(__doc__.split("Usage:")[0])
print("=" * 78)
print("1. CUBE GEOMETRY -- how many emitters are effectively visible at once?")
print("=" * 78)
print(f"   profile: flat to {FLAT_DEG:.0f} deg, half power {HALF_DEG:.0f} deg (DS190 flat-top)")
print(f"   a face at 90 deg off-axis still contributes {emission_profile(90):.3f}\n")
for label, d in [("face-on   (outboard)", (0, 1, 0)),
                 ("edge-on   (45 deg)", (0, 1, 1)),
                 ("corner-on (54.7 deg)", (1, 1, 1)),
                 ("broadside (+x)", (1, 0, 0))]:
    print(f"   N_eff {label:22s} = {cube_gain(d):.3f}")
n_face, n_edge, n_corner = cube_gain((0, 1, 0)), cube_gain((0, 1, 1)), cube_gain((1, 1, 1))
print(f"\n   => range {n_face:.2f} .. {n_corner:.2f}, mean ~{(n_face+n_edge+n_corner)/3:.2f}")

print()
print("=" * 78)
print("2. EMITTER -- one die at 306 mA vs five at 51 mA")
print("=" * 78)
ratio_i = I_FLIGHT_MA / I_BENCH_MA
print(f"   current ratio {ratio_i:.2f}x; flux ~ I^gamma (droop). K per die @51 mA = "
      f"{K_5COAIMED/5:.3f} uA*m^2\n")
print(f"   {'gamma':>6} {'flux x':>8} {'1 die @306':>12} {'5 co-aim @51':>14} {'ratio':>8}")
for gamma in (1.00, 0.95, 0.90):
    fx = ratio_i ** gamma
    k_one = (K_5COAIMED / 5) * fx
    print(f"   {gamma:>6.2f} {fx:>8.2f} {k_one:>10.2f}   {K_5COAIMED:>12.2f} "
          f"{k_one/K_5COAIMED:>8.2f}x")

GAMMA = 0.95
FX = ratio_i ** GAMMA
K_ONE_306 = (K_5COAIMED / 5) * FX
print(f"\n   Taking gamma={GAMMA}: ONE die at 306 mA ~= FIVE co-aimed at 51 mA "
      f"({K_ONE_306/K_5COAIMED:.2f}x, {db(K_ONE_306/K_5COAIMED):+.1f} dB)\n")

print("   As BUILT (5 on a cube, not co-aimed) -- this is the real comparison:")
print(f"   {'configuration':<38} {'K (uA*m^2)':>12} {'vs bench cube':>15}")
ref = (K_5COAIMED / 5) * n_edge
rows = [("bench cube, 5 @ 51 mA, edge-on", (K_5COAIMED / 5) * n_edge),
        ("bench cube, 5 @ 51 mA, face-on", (K_5COAIMED / 5) * n_face),
        ("1 die @ 306 mA (single face only)", K_ONE_306),
        ("flight cube, 5 @ 306 mA, face-on", (K_5COAIMED / 5) * FX * n_face),
        ("flight cube, 5 @ 306 mA, edge-on", (K_5COAIMED / 5) * FX * n_edge),
        ("flight cube, 5 @ 306 mA, corner-on", (K_5COAIMED / 5) * FX * n_corner)]
for name, k in rows:
    print(f"   {name:<38} {k:>12.3f} {k/ref:>13.2f}x")

print()
print("=" * 78)
print("3. LENS -- FOV is set by the DETECTOR, aperture gain by the F-number")
print("=" * 78)
for pd, area in PDS.items():
    f_side, f_diag = fov_deg(area)
    print(f"   {pd:9s} {area:.2f} mm^2 -> FOV {f_side:.2f} deg across, {f_diag:.2f} deg diagonal")
print()
print(f"   {'F/#':>5} {'pupil D':>9} {'pupil A':>10} | signal gain vs bare PD")
print(f"   {'':>5} {'(mm)':>9} {'(mm^2)':>10} | {'BPV10NF':>10} {'BPW34':>10}")
for fn in FNUMS:
    ap = pupil_area_mm2(fn)
    g = {pd: ap * T_LENS * T_FILT_PEAK / a for pd, a in PDS.items()}
    print(f"   {fn:>5.1f} {F_MM/fn:>9.2f} {ap:>10.1f} | "
          f"{g['BPV10NF']:>7.1f}x {db(g['BPV10NF']):>+5.1f}dB {g['BPW34']:>6.1f}x")
print("\n   NOTE: with the lens the SIGNAL photocurrent is the same for both PDs")
print("   (fixed pupil collects a fixed power onto whatever die it lands on).")
print("   The bigger die does not gain signal -- it only widens FOV and adds ambient.")

print()
print("=" * 78)
print("4. AMBIENT REJECTION -- geometric (FOV) x spectral (bandpass)")
print("=" * 78)
print("   ambient power on PD / bare = T_amb / (4 F/#^2)   [image-plane irradiance]")
print("   signal / ambient improvement = signal_gain x ambient_rejection\n")
FN = 2.0
for fname, passband in FILTERS.items():
    t_amb = T_LENS * T_FILT_PEAK * (passband / PD_BAND_NM)
    rej = 1.0 / (t_amb / (4 * FN**2))
    sg = pupil_area_mm2(FN) * T_LENS * T_FILT_PEAK / PDS[ANCHOR_PD]
    print(f"   F/{FN} + {fname:12s}: ambient x1/{rej:>6.0f} ({db(rej):>5.1f} dB), "
          f"signal x{sg:.0f} => S/A {db(sg*rej):>5.1f} dB")

print()
print("=" * 78)
print("5. RANGE -- anchored on the 41 ft indoor lock (r scales as sqrt(K_eff))")
print("=" * 78)
print(f"   implied decode floor: indoor {FLOOR_INDOOR_UA*1e3:.2f} nA, "
      f"outdoor bare/no-filter {FLOOR_OUTDOOR_UA*1e3:.1f} nA "
      f"({db(FLOOR_OUTDOOR_UA/FLOOR_INDOOR_UA):.1f} dB ambient penalty)\n")
print(f"   {'configuration':<38} {'F/1.6':>9} {'F/2.0':>9} {'F/2.4':>9}")
for name, k in [("bench cube 5@51 edge (bare, no lens)", (K_5COAIMED/5)*n_edge),
                ("1 die @306 mA", K_ONE_306),
                ("flight cube 5@306 face-on", (K_5COAIMED/5)*FX*n_face),
                ("flight cube 5@306 edge-on", (K_5COAIMED/5)*FX*n_edge)]:
    cells = []
    for fn in FNUMS:
        g = pupil_area_mm2(fn) * T_LENS * T_FILT_PEAK / PDS[ANCHOR_PD]
        cells.append(math.sqrt(k * g / FLOOR_INDOOR_UA))
    bare = math.sqrt(k / FLOOR_INDOOR_UA)
    print(f"   {name:<38} " + " ".join(f"{c:>8.0f}m" for c in cells)
          + f"   (bare {bare:.1f} m)")
print("\n   r scales LINEARLY with entrance pupil diameter D = f/(F/#).")

print()
print("=" * 78)
print("6. AGC STUDY INPUTS -- the drive profile the loop must survive")
print("=" * 78)
FN_AGC, FWHM_AGC = 2.0, 50.0
k = (K_5COAIMED / 5) * FX * n_edge
g = pupil_area_mm2(FN_AGC) * T_LENS * T_FILT_PEAK / PDS[ANCHOR_PD]
# EXTENDED-source ambient scales as T_amb/(4 F#^2) off the MEASURED bare-BPW34
# pedestal. The sun DISC does not -- section 8 handles that separately.
t_amb = T_LENS * T_FILT_PEAK * (FWHM_AGC / PD_BAND_NM)
ext_factor = t_amb / (4 * FN_AGC**2)
print(f"   flight cube edge-on, F/{FN_AGC}, {FWHM_AGC:.0f} nm filter, {PD_PRIMARY} at focus")
print(f"   K_eff = {k*g:.1f} uA*m^2 (K {k:.2f} x aperture gain {g:.0f} vs bare {ANCHOR_PD})\n")
print("   DC pedestal (extended ambient, measured bare-BPW34 x lens factor "
      f"{ext_factor:.4f}):")
for cond, (lo, hi) in AMBIENT_BARE_UA.items():
    print(f"     {cond:<24} bare {lo:>6.0f}-{hi:<6.0f} uA -> lensed "
          f"{lo*ext_factor:>7.2f}-{hi*ext_factor:<7.2f} uA")
ped_sun = AMBIENT_BARE_UA["sunny, not sun-facing"][1] * ext_factor
print(f"\n   {'range':>7} {'AC signal':>12} {'vs decode floor':>17} "
      f"{'vs sunny pedestal':>19}")
for r in (5, 10, 25, 50, 75, 100, 150):
    sig = k * g / r**2
    print(f"   {r:>5d} m {sig*1e3:>10.1f} nA {db(sig/FLOOR_INDOOR_UA):>14.1f} dB "
          f"{db(sig/ped_sun):>16.1f} dB")
print(f"\n   dynamic range 5 m -> 150 m = {db((150/5)**2):.0f} dB of signal swing alone,")
print(f"   on top of a pedestal that moves {db(AMBIENT_BARE_UA['sunny, not sun-facing'][1]/AMBIENT_BARE_UA['sunny, not sun-facing'][0]):.0f} dB with attitude/shadow.")

print()
print("=" * 78)
print("7. FILTER PLACEMENT -- why a NARROW filter is WRONG behind a fast lens")
print("=" * 78)
print("   A dielectric bandpass blue-shifts with angle of incidence:")
print("      lambda(theta) = lambda0 * sqrt(1 - (sin theta / n_eff)^2)")
print("   In CONVERGING space the marginal ray angle is arctan(1/(2 F/#)) -- large.")
print("   In front of the lens (COLLIMATED space) it is only the field angle -- tiny.\n")
print(f"   {'F/#':>5} {'marginal ray':>13} {'blue shift':>12} {'passband needed (full width)':>30}")
for fn in FNUMS:
    theta = math.degrees(math.atan(1.0 / (2 * fn)))
    shift = 850.0 * (1 - math.sqrt(1 - (math.sin(math.radians(theta)) / N_EFF_FILTER) ** 2))
    print(f"   {fn:>5.1f} {theta:>12.1f}d {shift:>10.1f}nm {2*shift:>28.0f} nm")
fov_half = fov_deg(PDS[PD_PRIMARY])[0] / 2
shift_front = 850.0 * (1 - math.sqrt(1 - (math.sin(math.radians(fov_half)) / N_EFF_FILTER) ** 2))
print(f"\n   Front-mounted at our {2*fov_half:.2f} deg FOV: shift {shift_front:.2f} nm -- negligible.")
print("   => An INTERNAL filter must be wide (>=30 nm). The ELP integrated filter is")
print("      therefore the RIGHT part, and the $140-180 10 nm BOM line (D5) is wrong")
print("      for internal use. Keep a narrow filter ONLY as a front-mounted option.")

print()
print("=" * 78)
print("8. SUN IN THE FIELD -- blanking, not damage")
print("=" * 78)
SUN_DEG = 0.53
for fn in [2.0]:
    ap_m2 = pupil_area_mm2(fn) * 1e-6
    print(f"   F/{fn}, pupil {pupil_area_mm2(fn):.1f} mm^2, direct solar disc in FOV:\n")
    print(f"   {'filter':<24} {'in-band E':>12} {'pupil power':>13} {'photocurrent':>14}")
    for fname, passband in FILTERS.items():
        e_band = SOLAR_AT_850_W_M2_NM * min(passband, PD_BAND_NM)
        p = e_band * ap_m2 * T_LENS * T_FILT_PEAK
        print(f"   {fname:<24} {e_band:>9.0f} W/m2 {p*1e3:>10.2f} mW {p*0.55*1e6:>11.0f} uA")

    # Geometry of the focused solar image -- is this a burning glass?
    d_sun_mm = F_MM * math.radians(SUN_DEG)
    a_sun_mm2 = math.pi * (d_sun_mm / 2) ** 2
    e_band = SOLAR_AT_850_W_M2_NM * 50.0
    p = e_band * ap_m2 * T_LENS * T_FILT_PEAK
    print(f"\n   focused solar image: {d_sun_mm*1e3:.0f} um dia, {a_sun_mm2:.4f} mm^2")
    print(f"   peak irradiance {p/(a_sun_mm2*1e-6)/1e3:.0f} kW/m^2 -- BUT total power is only "
          f"{p*1e3:.1f} mW")
    # Disc source on a half-space: dT = P / (4 k r), silicon k ~ 150 W/m/K
    dt = p / (4 * 150 * (d_sun_mm / 2 * 1e-3))
    print(f"   junction rise ~{dt*1e3:.1f} mK (silicon spreads it) => NOT a damage risk.")
    print(f"   It is a SATURATION risk: {p*0.55*1e6:.0f} uA against a nA-scale signal.")

print("\n   vs the Option-C pedestal ceilings -- NOTE the daylight doc's table predates")
print("   the lens. For an EXTENDED source the lens REDUCES ambient (T/4F#^2), but the")
print("   SUN DISC in the FOV is collected by the whole pupil, so it scales with AREA.")
ap_ratio = pupil_area_mm2(2.0) / PDS[PD_PRIMARY]
print(f"   pupil/die area ratio at F/2.0 = {ap_ratio:.1f}x\n")
print(f"   {'filter':<24} {'sun pedestal':>14}  " +
      "  ".join(f"{'vs '+r:>10}" for r in R_LOADS))
for fname, passband in FILTERS.items():
    if passband >= PD_BAND_NM:
        continue
    e_band = SOLAR_AT_850_W_M2_NM * passband
    i_ua = e_band * pupil_area_mm2(2.0) * 1e-6 * T_LENS * T_FILT_PEAK * 0.55 * 1e6
    cells = "  ".join(f"{'OK' if i_ua < c else f'{i_ua/c:.0f}x over':>10}"
                      for c in R_LOADS.values())
    print(f"   {fname:<24} {i_ua:>11.0f} uA  {cells}")
print("\n   => NO load resistor survives the sun disc in the field. Dropping R1 to 22 k")
print("      (the C-14 40 nm pairing rule) buys linearity for DIFFUSE sun, NOT for a")
print("      direct hit. Direct sun must be RIDDEN THROUGH, not designed around.")

print("\n   Transit time -- how long is the blanking?")
print(f"   PD FOV {fov_deg(PDS[PD_PRIMARY])[0]:.2f} deg; sun disc {SUN_DEG} deg; array pixel 0.375 deg\n")
print(f"   {'body rate':>10} {'PD blanked':>12} {'array px hit':>14} {'vs 325 ms HOLD':>16}")
for rate in (10, 20, 60, 120):
    t_pd = fov_deg(PDS[PD_PRIMARY])[0] / rate
    t_px = SUN_DEG / rate
    print(f"   {rate:>7d} d/s {t_pd*1e3:>9.0f} ms {t_px*1e3:>11.0f} ms "
          f"{'rides through' if t_pd < 0.325 else 'EXCEEDS':>16}")
print("\n   => at LOW body rates the 9.8 deg field blanks LONGER than the 300-325 ms")
print("      occlusion absorption the s7 decoder survives -- lock is lost, not ridden")
print("      through. A narrower field would shorten this linearly. Either way AGC")
print("      WINDUP compounds it: the DC tracker")
print("      (alpha=1/256 locked, tau=533 ms) would chase a 100000x pedestal and then")
print("      need ~533 ms to unwind -- an order of magnitude worse than the event.")
print("      Fix shape: a MAX-energy gate that freezes AGC + asserts HOLD, mirroring")
print("      the existing min-energy gate that kills dark-input false locks.")

