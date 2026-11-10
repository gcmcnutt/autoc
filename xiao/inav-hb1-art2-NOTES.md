# Article 2 FC — config notes and bench findings

Companion to `xiao/inav-hb1-art2.cfg`, which is now a raw `dump all` taken from the
board (INAV/HGLRCF722LITE 8.0.0, build `c412bfd7`). That dump overwrote the annotated
version, so the derivations live here.

Hardware: HGLRC Specter F722 Lite, autoc-fork backport target `HGLRCF722LITE`.
Harness: `cad/article2-harness/harness-spec.md`.

---

## 1. OUTSTANDING — two fixes never reached the board

The config was pasted before two later corrections. Both are still wrong on the
hardware. Apply and `save`:

```
set align_mag_yaw = 3600
set vbat_scale = 1110
save
```

**`align_mag_yaw`** is currently 0, and that is not cosmetic. INAV takes the
external-compass path only if one of `align_mag_roll/pitch/yaw` is NON-ZERO
(`sensors/compass.c:312`). With all three at zero it falls through to the on-board
branch, which calls `applyBoardAlignment()` on the magnetometer — rotating the
GPS-mast compass by the flight controller's 90° yaw and 7.5° tilt. Heading will read
roughly 90° out.

3600 decidegrees is 360°, i.e. the identity rotation, but it is non-zero so it selects
the external path and leaves the compass unrotated. Verified: the setting's range is
−1800..3600, and `rotationMatrixFromAngles(0,0,360°)` is exactly the identity.
On that path the `align_mag` enum is ignored.

The existing compass calibration does NOT need redoing — `magZero`/`magGain` are
applied before the alignment branch, so the calibration is in raw sensor frame and
stays valid. Only the rotation was wrong.

**`vbat_scale`** should be 1110, from the 2026-10-22 bench measurement: reported
12.49 V on a 12.60 V supply at scale 1100, so 1100 × 12.60/12.49 = 1109.7. One scale
unit is 11.4 mV, so 1110 is as close as the setting gets.

---

## 2. Board orientation — why roll carries the tilt

Mounting: arrow points to the aircraft's LEFT, board tilted so its +Z leans AFT
(it sits against the inside of the wing's upper skin). Arrow face UP, not inverted.

```
set align_board_roll  = 75     # tilt_degrees * 10
set align_board_pitch = 0
set align_board_yaw   = 900    # arrow LEFT == CW90
```

Two things that are easy to get backwards:

**`align_board_yaw = 900` is exactly `CW90_DEG`.** Confirmed against the firmware,
not the naming. `rotationMatrixFromAngles` (maths.c:235) builds the matrix, but
`rotationMatrixRotateVector` (vector.h:62) multiplies by its TRANSPOSE. Deriving this
without noticing the transpose gives CW270 and a 180° error.

**The aft tilt lands on ROLL, not pitch.** Because the board is yawed 90°, the
airframe's pitch axis IS the board's roll axis. Solved numerically against a port of
the same code: for a tilt of T degrees the exact answer is roll = T×10, pitch = 0,
yaw = 900, verified at T = 0, 5, 7.5 and 10 with every test vector round-tripping.

Article 1 independently confirms this. It ran `align_board_roll = 1700` where 1800 was
intended, alongside `align_board_yaw = 900`, and its flight data showed roughly a 10°
**pitch** bias. Same 10°, appearing on the other axis, exactly as the swap predicts.

7.5° is an estimate of the 5–10° mount angle. Refine with the Configurator's board
level calibration, in situ, at the aircraft's true level-flight attitude —
`updateBoardAlignment` already folds the measurement through the 90° yaw correctly.
Do NOT run that calibration with the board loose; it writes straight into
`align_board_roll` and would overwrite the 75 with a measurement of the bench.

---

## 3. Blackbox rate — currently too fast

Measured from `eval-results/bench-20261025`:

| | current (denom 8) |
|---|---|
| rate | 240 Hz |
| per sample | 105 bytes |
| throughput | 25.3 kB/s |
| 16 MB flash fills in | ~11 min |

The policy runs at 20 Hz, so 240 Hz is 12× oversampled. Options:

| denom | rate | throughput | 16 MB lasts |
|---|---|---|---|
| 8 (now) | 240 Hz | 25.3 kB/s | 11 min |
| 20 | 96 Hz | 10.1 kB/s | 28 min |
| 40 | 48 Hz | 5.1 kB/s | 55 min |

`set blackbox_rate_denom = 20` is the recommendation: ~100 Hz is 5× the policy rate
and still ~20× the 2–5 Hz oscillation band that 043 was chasing, with room for the
50/100 ms gyro history 044 wants. Keep denom 8 for a dedicated short vibration run,
where high rate actually buys something.

---

## 4. Bench run 2026-10-25 — what it showed

23.3 s armed, 5608 blackbox samples, 149 xiao ticks.

**The autoc override path works end to end.** Blackbox shows `ARM|MSPRCOVERRIDE` for
7.45 s with the servos and motor driven; the xiao log shows ENGAGE→DISENGAGE spanning
7.435 s. The two independently-recorded clocks agree to 15 ms. Disengage was
`DISENGAGE_REASON=1` (servo switch), i.e. clean and operator-commanded, not a fault.

**xiao timing has headroom.** Against a 50 ms tick: total 12.5 / 22.2 / 35.7 ms
(min/mean/max), fetch 7.6/14.0/25.5, eval 0.9/1.7/8.3, send 3.9/6.6/9.0.
0 overruns, 0 resyncs, 0 dropped ticks, maxLate 31 ms. Fetch dominates and the worst
case sits at 71% of the budget.

**Sensors are healthy.** `hwHealthStatus = 341` decodes to GYRO, ACC, COMPASS, BARO
and GPS all OK; rangefinder and pitot absent, as expected.

**`gyroADC` logs as all-zero — this is NOT a fault.** It was worth checking, because
article 1's stationary log shows ±4. The field is `lrintf(gyro.gyroADCf[i])`, i.e.
integer DEGREES PER SECOND. The board was genuinely motionless here (attitude moved
0.3° across 23 s, `accVib` 3–5), so sub-0.5 °/s rounds to zero. Article 1's ±4 came
with `accVib` up to 1044 — real motion, not a fair comparison. The all-zero `debug[]`
under `debug_mode = VIBE` has the same cause: no vibration to report.

**The board was inverted on the bench.** `accSmooth[2]` ≈ −2018 and attitude roll
≈ 175.5°, versus ≈ +2000 for article 1. Expected with the board loose, but worth
stating because `align_board_roll = 75` assumes the arrow face is UP when installed.

**The policy on the xiao is generation 20.** The log names
`gen9980.dmp.zst`, and dmp filenames encode 10000 − generation. Fine for a wiring
test — and the outputs look like it, with `out_roll` never going below +0.14 and
averaging +0.83, i.e. pinned hard to one side. Not a flight-capable policy; load the
043 t6 gen-752 baseline (or whatever 044 produces) before flying.

---

## 5. Other config observations

- **`feature TX_PROF_SEL` is ON**, where article 1 had it off and the pasted config
  did not enable it. It lets the transmitter switch control/battery profiles, which is
  not wanted for a deterministic policy test. No aux is bound to it, so it is latent
  rather than active. Suggest `feature -TX_PROF_SEL`.
- `current_meter_scale` moved 179 → 400 (target default). Harmless; `current_meter_type`
  is NONE because the CR pad wants a 4-in-1 ESC signal we do not have.
- `gyro_to_use` is absent on this target — the Matek had two IMUs, this board has one.
- Everything else took correctly: ports, timer output modes, mixer, aux modes, CRSF,
  `acc_hardware = ICM42605`, `baro_hardware = DPS310`, `msp_override_channels = 47`.

---

## 6. Still to do

1. Apply the two fixes in §1.
2. `set blackbox_rate_denom = 20` (§3).
3. `feature -TX_PROF_SEL`.
4. **Verify motor and servo directions** — the last item before install.
5. After install: board level calibration at true level attitude (§2), then compass
   calibration with the mast in final position.
6. Load a real policy before flight (§4).
