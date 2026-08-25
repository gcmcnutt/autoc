# Contract — the action space

⭐ **One definition, shared by sim and firmware.** Spec FR-016 exists because a disagreement here is
invisible in every artifact that survived the change and would only appear in the air.

## The chain

```
NN output      tanh ∈ [-1, +1]          (3 outputs: roll, pitch, throttle — UNCHANGED)
   ↓ ×500 + 1500
RC channel     [1000, 2000] µs          (UNCHANGED range, UNCHANGED polarity)
   ↓ INAV rcCommand
rcCommand      [-500, +500]
   ↓ pidRcCommandToRate — scaleRangef(stick, ±500, ±rate×10)
rate setpoint  ±360 °/s roll · ±120 °/s pitch · (yaw: no actuator)
   ↓ Cntrl_InavFwRate / INAV pidApplyFixedWingRateController
axisPID        ±500
   ↓ servo mixer × smixRate/100            ⭐ IDENTICAL path in MANUAL and ACRO
surface
```

⭐ **`servos.c` feeds `rcCommand` (±500) in MANUAL and `axisPID` (±500) in stabilized modes into the same
mixer at the same rate.** Only what *fills* the ±500 changes. The 037 servo model sits downstream of this
point in both cases and needs no change.

## Invariants

| | |
|---|---|
| **count** | 3 NN outputs — unchanged. No yaw output (no rudder). |
| **magnitude** | tanh [-1,+1] → 1000–2000 µs — unchanged. |
| **polarity** | ⛔ unchanged, and **MUST be verified end to end on the bench**: NN sign → PWM about 1500 → `rcCommand` sign → commanded rate sign → achieved body-rate sign. `msplink` already flips y/z at the FRD↔FLU boundary for quat/gyro/accel, so a frame convention is genuinely in play. |
| **throttle** | direct command, not a rate — no inner-loop equivalent (FR-017). |
| **saturation** | ±360 °/s roll is far beyond the airframe (measured mean \|p\| = 88 °/s). The sim must express the shortfall as *the aircraft does not follow*, not as a clip the real loop would not do. |

## ⚠️ The asymmetry, and why it is worse than it looks

`roll_rate 36` / `pitch_rate 12` gives a 3:1 reachable-rate asymmetry — **and** the Gaussian attenuation
width scales with `maxRate`, so σ is 61.2 °/s on roll against 20.4 °/s on pitch. Pitch goes essentially
pure-feed-forward above ~40 °/s while roll retains 36% of P at 88 °/s. ⛔ **The two axes run materially
different effective controllers.** 043 does not change the config (gains and rates as-is), but the plan
MUST document the per-axis effective gain curve — it is a likely contributor to the per-axis behaviour
differences 041 reported.
