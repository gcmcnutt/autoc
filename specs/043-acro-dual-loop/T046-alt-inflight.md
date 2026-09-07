# T046-alt — in-flight actuator identification: minimum log set and maximum rate

Operator proposal (2026-09-06): instead of the camera/load bench rig, fly **one short flight** with a
**stripped blackbox field set at a much higher rate**, doing a few deliberate **MANUAL bang-bang** inputs.
Servo command plus raw gyro/accel then give the response under real aero load.

⭐ **This is the better instrument for what the sim actually needs, and it makes the bench rig optional.**

---

## 1. Why this beats the bench rig for this purpose

⛔ **The honest limitation first**: blackbox `servo[0..1]` is the **commanded** position, not the achieved
one. There is no surface feedback in flight. So this measures **command → body rate**, i.e. the
**actuator and airframe composite** — it cannot isolate the servo the way the camera rig would.

⭐ **But the composite is exactly what the sim is judged on.** `cntrl_inavfwrate` + the servo model + the
FDM exist to reproduce command → rate. §10 of `flight-analysis-20260906.md` measured that composite and
found it wrong per-axis (real roll 0.58 vs sim 0.74; real pitch 0.70 vs sim 0.43). Splitting it into
"servo" and "aero" only matters for *attribution*, not for making the sim right.

And a step response is **partially separable by shape anyway**: the **dead-time** is dominated by the
actuator (PWM latch + servo processing), while the **settling curve** is dominated by the airframe.

⭐⭐ **Airspeed is the load sweep, for free.** Hinge moment scales with q = ½ρV², so flying the same step
at **10 / 15 / 20 m/s** gives a 4× range of hinge moment with *real* aerodynamic loading — which is
precisely what Phase C of the bench procedure was trying to fake with hanging weights, and it needs no
`Ch` estimate.

⚠️ It does not settle `kServoTransitSecPer60Deg` (0.055 vs 0.070) on its own — that is a property of the
servo alone. But if the composite matches the sim after correction, that constant stops mattering as much.

## 2. What INAV lets you strip — `blackbox <FLAG>` / `blackbox -<FLAG>`

INAV 8 has 15 selectable field groups (`blackbox.h:24-40`, CLI names in `cli.c:172`). Current flight
config and what each costs, by field count in the decoded frame:

| flag | now | fields | keep for this test? |
|---|---|---:|---|
| `NAV_POS` | ON | **18** | ⛔ **DROP** — the single biggest group, and irrelevant to a step response |
| `MAG` | ON | 3 | ⛔ drop |
| `QUAT` | ON | 4 | ⛔ drop |
| `RC_DATA` | ON | 4 | ⛔ drop (keep `RC_COMMAND`, which is post-expo/deadband) |
| `MOTORS` | ON | 1 | ⛔ drop |
| `GYRO_RAW` | ON | 3 extra | ⚠️ **drop** — `gyroADC` is core and already filtered-as-flown; `gyroRaw` doubles the cost |
| **`ACC`** | ON | 4 | ✅ **KEEP** — raw accel is half the point |
| **`SERVOS`** | ON | 2 | ✅ **KEEP** — the command |
| **`RC_COMMAND`** | ON | 4 | ✅ **KEEP** — the pilot's step |
| `NAV_ACC`/`NAV_PID`/`ATTI`/`PEAKS_*` | already OFF | — | leave off |

⇒ **drops ~33 of 92 fields, and they are among the most *active* ones.** ⭐ This matters more than the
count: blackbox is **delta-encoded**, so constant or zero fields (the PID terms in MANUAL, temperatures,
flags) cost almost nothing. The expensive fields are the ones that change every frame — `navPos`/`navVel`
(18), `quaternion` (4), `magADC` (3), `gyroRaw` (3). Those are exactly what this list removes.

**CLI:**

```
blackbox -NAV_POS
blackbox -MAG
blackbox -QUAT
blackbox -RC_DATA
blackbox -MOTORS
blackbox -GYRO_RAW
save
```

⛔ **This is a test-only configuration — revert it before any tracking flight.** `NAV_POS` and `QUAT` are
what the sim↔real position and attitude comparison rides on, and `flight-analysis` would be blind without
them.

## 3. Rate ladder

`looptime = 500 µs` (2 kHz), so the rate is `2000 / blackbox_rate_denom`:

| `blackbox_rate_denom` | rate | at ~105 B (today) | at ~55 B (stripped, est.) |
|---:|---:|---:|---:|
| **32** (today) | 62.5 Hz | 8.4 kB/s *(measured)* | — |
| 16 | 125 Hz | ~13 kB/s | ~7 kB/s |
| **8** | **250 Hz** | ~26 kB/s | **~14 kB/s** |
| 4 | 500 Hz | ~52 kB/s | ~28 kB/s |
| 2 | 1 kHz | — | ~55 kB/s |

⭐ **250 Hz on the stripped set is roughly today's total data rate** — the strip pays for a 4× rate
increase. 500 Hz is ~3× today's rate and is where it gets interesting.

⚠️ `blackbox_device = SPIFLASH`, which is the bandwidth-limited path (page-program stalls, not capacity).
⛔ **Do not guess the ceiling — measure it.** It is self-diagnosing: `blackbox_decode` reports
`frames failed to decode` and missing loop iterations.

## 4. ⭐ Bench qualification first (~30 min, no flying)

Find the real ceiling before spending the flight:

1. Apply the stripped field set. Note the new bytes/frame from a short decode.
2. For `denom` = 16, 8, 4, 2: arm on the bench **with the motor running at flight-ish throttle** (props
   off or restrained) so the FC is realistically loaded, log ~30 s, disarm, download, decode.
3. Read off `Data rate`, `frames failed to decode`, and whether loop iterations are missing beyond the
   denominator's own decimation.
4. ⭐ **Pick one notch below the first setting that shows losses.** Do not fly at the edge.

ⓘ Today's 59 Hz already shows 4–5 failed frames per flight (0.04%), so "zero failures" is not the bar —
"no worse than today, and no gaps" is.

## 5. Flight card (~one short sortie)

**MANUAL throughout. No NN, no autoc engagement.** Props on, normal flying.

| # | manoeuvre | why |
|---|---|---|
| 1 | trim out at ~**10 m/s**, wings level, then **sharp full roll**, hold ~0.5 s, sharp return. ×5 | low hinge moment |
| 2 | same at ~**15 m/s** (cruise). ×5 | ⭐ the reference point — matches the flown regime |
| 3 | same at ~**20 m/s**. ×5 | high hinge moment; with (1) and (2) this is the load sweep |
| 4 | repeat 1–3 in **pitch** | pitch is where the sim is most wrong (0.43 vs 0.70) |
| 5 | one **slow ramp** roll and pitch, stop to stop over ~3 s | separates rate-limit from lag |

⭐ **Bang-bang matters**: a sharp stick step is a much cleaner excitation than anything the NN produces,
and the pilot can place it at a known airspeed, wings level, repeatably. Hold each step long enough for
the rate to settle — a rate that is still rising when the stick returns gives no plateau to fit.

⚠️ Note the airspeed for each set (OSD or GPS groundspeed into wind, and log the wind).

## 6. What comes out

| measurement | from | replaces |
|---|---|---|
| dead-time, command → first rate change | `rcCommand`/`servo` → `gyroADC` step | `kCraftServoPwmFrameSec`, and part of `COMPUTE_LATENCY` |
| rate-limit (slew) vs airspeed | plateau slope at 10/15/20 m/s | `kCraftServoSlewCenter` + the `[16,32]` clamp, **with real aero load** |
| settling shape | step response tail | whether v2's "pure slew, no tau" holds |
| per-axis command → rate gain | plateau amplitude | ⭐ the roll-too-strong / pitch-too-weak split in §10 |

⇒ **This is the cheaper path to the same pre-bake corrections**, and the bench rig becomes optional —
worth building only if the flight data leaves the actuator and the aero unseparable.

⚠️ Still `n = 1` airframe. The operator's point stands: a second article is what turns these into a
*range* rather than a point, and that is a build-time task that can run while the bake does.
