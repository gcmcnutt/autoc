# T046 / T273g — DSM-44 loaded servo step-response: bench protocol

**Why this exists**: the same measurement has been requested by three features — 018 `T273g`, 037 t11
(*"Still open: a loaded bench step test"*), 043 `T046` — and never taken. It settles a live conflict inside
the repo (`craft_variation.h` **0.055** s/60° vs `finding.md`'s recorded datasheet **0.070**) and sets the
`craftServoSlew` clamp, which today is an assumed load envelope.

**Servo under test**: **Power HD DSM-44 digital micro**. 1000–2000 µs ⇒ **90° mechanical span** (confirmed
2026-06-11). Driven today at `servo_pwm_rate = 50` Hz.

---

## 0. What actually needs measuring, and why it is not just "slew"

⭐ **The typical NN command is NOT slew-limited; the tail is.** From the 2026-09-05 flight, per-tick
command deltas against the travel available in one 50 ms tick:

| channel | mean \|Δ\| | p90 | p99 | ticks >1.21 u (@24.2/s) | >0.64 u (@12.7/s) |
|---|---:|---:|---:|---:|---:|
| pitch | 0.234 | 0.754 | 1.385 | 2.4% | **13.5%** |
| roll | 0.416 | 1.023 | 1.557 | 4.1% | **24.7%** |
| throttle | 0.362 | 1.692 | 1.923 | 17.1% | 21.0% |

⇒ Moving the slew constant from 24.2 to 12.7 changes how often the **actuator, not the policy**, decides
the surface position — roll **4.1% → 24.7%**, a 6× swing. That is what makes this worth a bench day.

⇒ ⛔ **Measure three things, not one**: dead-time, large-step slew *as a function of load*, and the
**small-step** response (0.1–0.5 units), because v2 models pure slew with **no tau** and 037 left tau as
*"an upper-bound estimate at 20 ms"* that was never checked.

## 1. Instrumentation — reuse what 031/042 already built

| role | hardware | why |
|---|---|---|
| position sensor | **Pi 3A+ + OV9281 @ 250–280 fps**, 850 nm filter | 4 ms sampling, **global shutter** (no rolling-shutter skew on a moving surface), and the centroid tracker already exists |
| surface marker | one 850 nm beacon LED on the elevon horn / trailing edge | it is the same target the tracker was written for — no new vision code |
| **command marker** | a second 850 nm LED on a **xiao GPIO**, in frame | ⭐ this is the trick: it removes the clock-join entirely |
| command source | the **xiao**, via its existing MSP override at 20 Hz | exercises the *flown* command path, not a synthetic one |

⭐ **Why the xiao drives it**: it can toggle its own LED **in the same statement** that issues the stepped
command, so the video contains both the command instant and the resulting motion **on one time base**. No
cross-device sync, no ppm fit, no residual. Dead-time is then read straight off the frame count.

⚠️ Camera looking **along the hinge line** so deflection is a clean arc in-plane. Put a printed angular
scale in frame for absolute calibration; the tracker gives sub-pixel centroid, so 90° across a decent
pixel span resolves well under a degree.

## 2. Command sequence (bench-mode firmware, ~40 lines)

Hold, step, hold — repeated, with the LED marking each step edge:

| step size (autoc units) | what it isolates |
|---|---|
| **±1.0** (full span, rail to rail) | slew ceiling; directly comparable to the 0.055 / 0.070 s/60° conflict |
| **±0.5** | mid regime; is transit time linear in step size (pure slew) or not (tau present)? |
| **±0.25**, **±0.1** | ⭐ the regime the NN actually lives in (mean \|Δ\| 0.23–0.42) |
| ±1.0 with **randomised phase** vs the 50 Hz frame | measures the 0–20 ms latch distribution directly |

≥20 repeats per size. Randomise the order so servo warming does not alias onto step size.

## 3. Load — produce a CURVE, not a point

⛔ 037's own note: *"svSlew clamp [16,32] is the **load envelope**, not manufacturing spread."* So the
deliverable is **slew vs hinge moment**, and the clamp is then read off that curve at the aero-load range —
rather than the current assumed bracket.

Apply known static torque at the horn (calibrated weights on a lever arm, or a characterised spring),
sweeping 0 → beyond the estimated cruise hinge moment. Record slew at each.

To place the cruise point on that curve: `q = ½ρV² = 0.5 × 1.225 × 15² = 138 Pa` at the flown 15 m/s;
hinge moment needs elevon area, mean chord aft of the hinge, and typical deflection — take the geometry
from `hb1_streamer.xml` / the airframe rather than guessing. ⚠️ An airflow load (fan) is *more* faithful
but much harder to calibrate; the static-torque curve is the better first instrument because it is
repeatable and it produces the envelope directly.

## 4. Also settle `servo_pwm_rate` while the rig is up

The servo is **digital**, so 50 Hz is a config choice, not a constraint (§19 of `flight-analysis.md`).
With the rig already built this is nearly free:

1. Confirm the DSM-44's **accepted frame rate** — datasheet, then empirically (watch for buzz, heating,
   current draw, or missed frames). ⚠️ Do not assume 333 Hz; digital micros vary and an over-driven servo
   browns out.
2. Re-run the ±1.0 and ±0.1 sets at 50 / 200 / 333 Hz.
3. Expected: dead-time **0–20 ms → 0–5 ms**, i.e. ~7.5 ms mean off the 81.6 ms phase budget. Slew should
   be unchanged — if it is not, that is itself a finding.

⛔ If `servo_pwm_rate` moves, it moves **identically in the sim** (`kCraftServoPwmFrameSec`, currently
0.020) and gets bench-verified before any bake — FR-012a discipline.

## 5. Deliverables → where each number lands

| measured | replaces | in |
|---|---|---|
| slew at cruise hinge moment | `kCraftServoSlewCenter` ≈ 24.24 (from the disputed 0.055) | `craft_variation.h` |
| slew-vs-load curve endpoints | `kCraftServoSlewMin/Max` = 16 / 32 (assumed envelope) | `craft_variation.h` |
| dead-time distribution | `kCraftServoPwmFrameSec` = 0.020 (0–20 ms latch) | `craft_variation.h` |
| small-step behaviour | v2's **"pure slew, no tau"** assumption, and 037's unverified 20 ms tau bound | servo v2 model |
| accepted frame rate | `servo_pwm_rate = 50` | `inav-hb1.cfg` + sim |

Write results to `specs/043-acro-dual-loop/actuator-pin.md` (the file T046 already names), and record
**which of 0.055 / 0.070 the measurement supports** — that single line closes a three-feature-old question.

⚠️ **`n = 1` servo.** This pins *this* article. Per `2c691aa`, unit-to-unit spread still wants the second
airframe; what this test gives is the **centre and the load envelope**, which is what the model is missing.
