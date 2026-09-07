# T046 — DSM-44 servo step-response: step-by-step bench procedure

Design rationale: [`servo-bench-protocol.md`](servo-bench-protocol.md). This file is the **executable
sequence**. Results go to `specs/043-acro-dual-loop/actuator-pin.md`.

**What it settles** — three things, all currently assumed:

| constant | today | source of doubt |
|---|---|---|
| `kServoTransitSecPer60Deg` | **0.055** | `finding.md:80` records the datasheet as **0.070** |
| `kCraftServoSlewMin/Max` | **16 / 32** | an *assumed* load envelope, never measured |
| `kCraftServoPwmFrameSec` | 0.020 (50 Hz) | the servo is **digital** — 50 Hz is a choice, not a limit |

⭐ Ordered so each phase is usable on its own. If the day runs short, **Phase B alone settles
0.055 vs 0.070**, which is the single most valuable number.

---

## Phase A — rig and calibrate (~45 min, once)

1. **Secure the airframe** so it cannot move; the elevon must swing freely through its full ±45°.
2. **Marker LED** — one 850 nm beacon LED on the elevon, as far from the hinge as practical (longer arc =
   better angular resolution). Tape is fine; it only has to survive the session.
3. ⭐ **Command LED** — a second 850 nm LED on a xiao GPIO, **in the same camera frame**, positioned so it
   cannot be confused with the marker (opposite corner). *This is what removes the clock-join entirely.*
4. **Camera** — Pi 3A+ + OV9281, **850 nm filter**, 250 fps, looking **along the hinge line** so the
   deflection is a clean in-plane arc. Global shutter matters here: a rolling shutter would skew a surface
   moving 90° in ~100 ms.
5. **Angular reference** — tape a printed protractor/scale in frame, or record the surface at three known
   deflections (−45°, 0, +45°) using a digital angle gauge. ⇒ pixels → degrees.
6. **Verify the tracker sees both LEDs** and that neither saturates or drops out through the full sweep.
   ⚠️ Fix this now; it is the failure that wastes a session.
7. **Record a 10 s static clip.** Centroid noise on a stationary marker is your angular noise floor —
   note it, it bounds every slope fit that follows.

## Phase B — no-load steps ⭐ (~45 min) — settles 0.055 vs 0.070

Servo powered from the **flight battery/BEC**, not a bench supply, so the voltage sag is real.

8. Run the bench step sequence (Phase F firmware) with the elevon **unloaded**:

| step size (autoc units) | what it isolates | reps |
|---|---|---:|
| **±1.0** full span | ⭐ slew ceiling — the 0.055 vs 0.070 answer | 20 |
| ±0.5 | is transit time linear in step size (pure slew) or not (tau present)? | 20 |
| **±0.25**, **±0.1** | ⭐ the regime the NN actually lives in (mean \|Δ\| 0.23–0.42) | 20 each |

9. ⚠️ **Randomise the order** of step sizes. A servo warms up over a session; sequential blocks alias that
   onto step size.
10. From each step extract, and log per rep:
    - **dead-time** = command-LED edge → first marker motion (frames × 4 ms)
    - **slew** = slope of the linear portion, °/s → autoc units/s via `2.0 units / 90°`
    - **settle** = first time within 2% of final, and whether it overshoots

⭐ **Decision point**: full-span transit of **82.5 ms ⇒ 0.055 confirmed**; **105 ms ⇒ 0.070**, and
`kCraftServoSlewCenter` must drop 24.2 → 19.0.

## Phase C — loaded sweep (~60 min) — sets the clamp

⛔ 037 established the clamp is a **load envelope**, not manufacturing spread. So produce a **curve**.

11. Attach a lever at the elevon horn, arm length **measured** (10–20 mm typical). Hang known weights so
    the torque **opposes** the commanded direction.
12. **Target hinge moment at cruise ≈ 5–30 mN·m** (derived from `hb1_streamer.xml`: chord 0.178 m,
    span 0.762 m, q = 138 Pa at 15 m/s, elevon ~25–30% chord × ~60% span, `Ch` 0.10–0.20). ⚠️ `Ch` is a
    rough plain-flap figure — **the sweep is the deliverable, not this point estimate.**

    | arm | 5 mN·m | 10 | 20 | 30 mN·m |
    |---|---:|---:|---:|---:|
    | 10 mm | 51 g | 102 g | 204 g | 306 g |
    | 15 mm | 34 g | 68 g | 136 g | 204 g |
    | 20 mm | 26 g | 51 g | 102 g | 153 g |

13. Repeat the **±1.0** and **±0.1** sets at **0 / 5 / 10 / 20 / 30 mN·m**. Record slew at each.
14. ⭐ Record **both directions** — against the load and with it. The "with" case is the assisted slew and
    bounds the fast end of the envelope for free.
15. Plot slew vs hinge moment ⇒ read `kCraftServoSlewMin/Max` off the curve at the cruise range, replacing
    the assumed [16, 32].

## Phase D — frame rate (~30 min) — the free phase win

16. **Confirm the DSM-44's accepted frame rate** — datasheet first, then empirically: raise
    `servo_pwm_rate`, and watch for **buzz, heating, current draw, or missed frames**. ⚠️ Do not assume
    333 Hz; digital micros vary and an over-driven servo browns out.
17. Re-run **±1.0** and **±0.1**, no load, at `servo_pwm_rate` = **50 / 200 / 333 Hz**.
18. Expect **dead-time 0–20 ms → 0–5 ms**; slew should be **unchanged**. ⚠️ If slew changes too, that is
    itself a finding — say so rather than averaging it away.

## Phase E — record and route (~30 min)

19. Write `actuator-pin.md` with, per condition: dead-time (mean/spread), slew, settle, overshoot.
20. ⭐ State plainly **which of 0.055 / 0.070 the measurement supports** — that one line closes a question
    open across 018 T273g, 037 t11 and 043 T046.
21. Map each result to the constant it replaces:

    | measured | replaces | in |
    |---|---|---|
    | slew at cruise hinge moment | `kCraftServoSlewCenter` 24.24 | `craft_variation.h` |
    | slew-vs-load curve ends | `kCraftServoSlewMin/Max` 16 / 32 | `craft_variation.h` |
    | dead-time distribution | `kCraftServoPwmFrameSec` 0.020 | `craft_variation.h` |
    | small-step behaviour | v2's **"pure slew, no tau"** assumption | servo v2 model |
    | accepted frame rate | `servo_pwm_rate = 50` | `inav-hb1.cfg` **and** sim |

22. ⛔ If `servo_pwm_rate` moves it moves **identically in the sim** (`kCraftServoPwmFrameSec`), gets
    bench-verified, and is folded into the config of record — FR-012a discipline.

## Phase F — the bench firmware (write before the session)

A xiao bench mode, ~40 lines, that:

- drives the step through the **existing MSP override path** (so the *flown* command path is exercised,
  not a synthetic one);
- ⭐ toggles the command LED **in the same statement** that issues the step;
- holds each level ≥ 1 s so the surface fully settles;
- steps through the size list with randomised order and ≥20 reps;
- prints the schedule to the console so the video can be indexed afterwards.

## ⚠️ Standing caveats

- **`n = 1` servo.** This pins *this* article's centre and load envelope. Per `2c691aa`, unit-to-unit
  spread still wants the second airframe — but the centre and envelope are what the model is missing.
- **Measure, do not eyeball.** Same lesson as `rc_expo`: the arm-C full-deflection check looked identical
  either way and would have hidden a 20% action-space error.
- ⛔ **Do not skip Phase B's small steps.** Mean per-tick command is 0.23–0.42 units; full-span slew is the
  *tail* behaviour. If v2's "pure slew, no tau" is wrong, it is wrong in the small-step regime where the
  policy actually operates.
