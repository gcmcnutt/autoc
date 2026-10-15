# What the policy can see in time — history slots, recurrent memory, and the 1–4 Hz cycle (2026-10-15)

Asked by the operator at t6 gen 275: *the sim is now honest to ~Nyquist; the NN's sensor history inputs and their cadence have
not been analysed against the 1–4 Hz cycle.* This note does that from three sides — the input vector as declared, a linear
attribution of the pitch command on the elite's recorded inputs (sim and **flight**), and an exact open-loop replay of the
network with counterfactual inputs. Scripts: [`pitch_input_attribution.py`](pitch_input_attribution.py),
[`nn_replay_probe.py`](nn_replay_probe.py) (both take a `dmp-dump --csv-only` tick CSV; the first also takes a
`flightlog_decode.py` CSV).

## 1. The temporal apparatus, as declared

[`nn_inputs.h`](../../include/autoc/nn/nn_inputs.h) — 45 slots, 20 Hz tick, topology 45 → 32 → **16 recurrent** → 3:

| what | slots | time view |
|---|---|---|
| target direction in body frame `target_x/y/z` | 18 | lags **{800, 400, 200, 100, 50, 0} ms** (log-spaced, `kNNHistoryLagsMsec`) |
| target distance `dist`, closing rate | 7 | same six lags + an instantaneous rate |
| **craft state**: quaternion, airspeed, gyro p/q/r, accel x/y/z, Es, boundary closure, distance, inward vector, score gradient | 20 | **instantaneous only — no history of any craft sensor** |
| recurrent state `h` | 16 | whatever the network chooses to keep; W_hh spectral radius **2.9–3.1** (saturating regime, not a leaky integrator) |

So the only *explicit* history the policy has is of the target's bearing and range. Its own rotation's past is not an input;
it has to come from the recurrent state or be inferred from how the target bearing swings in body frame — and a bearing swing
is the same signal whether the target moved or the craft rotated.

**What a lag slot can see of a cycle.** A difference against a lag τ has gain |1 − e^{−j2πfτ}| — zero where τ is a whole
period:

| f (Hz) | 50 ms | 100 ms | 200 ms | 400 ms | 800 ms |
|---|---|---|---|---|---|
| 1.0 | 0.31 | 0.62 | 1.18 | 1.90 | 1.18 |
| 2.0 | 0.62 | 1.18 | 1.90 | 1.18 | 1.90 |
| 2.3 (flight cycle) | 0.71 | 1.32 | 1.98 | **0.50** | 0.96 |
| **2.5** | 0.77 | 1.41 | 2.00 | **0.00** | **0.00** |
| 3.0 | 0.91 | 1.62 | 1.90 | 1.18 | 1.90 |
| 4.0 | 1.18 | 1.90 | 1.18 | 1.90 | 1.18 |

The 400 and 800 ms slots are blind at 2.5 Hz and nearly so at the flight cycle's 2.0–2.3 Hz; the 200 ms slot reads the cycle
*inverted* (half a period); only the 50 and 100 ms slots carry a usable trend, and a 50 ms difference gives +68° of lead at
2.5 Hz, 100 ms gives +45°. Sampling itself is not the problem — 20 Hz puts 5–10 samples on a 2–4 Hz period — the problem is
that the available lead is small and the plant's lag is not.

## 2. What the pitch command is actually computed from (linear surrogate on recorded inputs)

`out_pt ~ 45 inputs`, pooled over scenarios/spans; ΔR² = loss of fit in the 1.5–4 Hz band when the group is removed.

| | t6 gen-275 elite (sim) | g488 on the honest plant (sim) | g488 on the t5 plant (sim) | **g488 in flight 10-11 #1** | **#2** |
|---|---|---|---|---|---|
| R² in band | 0.60 | 0.79 | 0.60 | **0.90** | **0.92** |
| target dir, all lags (alone) | 0.24 (0.55) | 0.15 (0.70) | 0.17 (0.48) | 0.13 (0.87) | 0.15 (0.89) |
| of which lags 0/50/100 ms | **0.19** | **0.14** | **0.16** | **0.09** | **0.08** |
| of which lags 200/400/800 ms | 0.01 | 0.01 | 0.02 | 0.02 | 0.00 |
| **gyro (now)** | 0.015 | 0.002 | 0.06 | **0.003** | **0.002** |
| accel (now) | 0.00 | 0.06 | 0.06 | 0.00 | 0.00 |
| quaternion (now) | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 |
| cmd → gyro peak correlation lag | +2 ticks (0.60) | +2 (0.59) | +2 (0.73) | **+2 (0.73), +3 (0.68)** | +2 (0.75), +3 (0.69) |
| phase out_pt vs gyrQ, 2–3 Hz | +108° | +80° | +62° | +91° | +84° |
| phase, 3–4 Hz | +162° | +114° | +104° | −2° | +105° |

Reading, and it is the same in the air as in the sim: **the 1.5–4 Hz pitch command is a function of the target bearing in body
frame and its 50/100 ms differences. The gyro contributes nothing to it.** The longer lags contribute nothing either. The
aircraft's gyro follows the command 100–150 ms later (peak at +2 to +3 ticks in flight, +2 in sim — the sim is ~25 ms quicker,
the remaining latency gap). The policy is therefore a **proportional-plus-short-lead attitude loop, closed through a rate
command with ~100–150 ms of command-to-rate lag**. Its phase budget at 2.5 Hz: 90° rate→attitude integration + 90–135° command
lag + ~22° ZOH = 200–250° of lag against 45–68° of lead from the 50/100 ms slots — a loop that is past 180° wherever its gain
is still ≥ 1, i.e. a limit cycle at the frequency where gain crosses 1, set by the plant's lag, not by the objective. That is
the 2–3.5 Hz cycle, in flight and now in the sim.

## 3. The recurrent memory (exact replay, counterfactual inputs)

`nn_replay_probe.py` reproduces the recorded pitch output to R² = 1.000 (24 scenarios each), then re-runs the same network with:

| counterfactual | t6 gen 275: R² vs baseline / band variance | g488: R² vs baseline / band variance |
|---|---|---|
| hidden state zeroed every tick | 0.24 / **×2.3** | −0.10 / **×0.2** |
| history slots frozen at the 0 ms value | 0.36 / ×2.0 | 0.67 / ×1.6 |
| gyro zeroed | 0.22 / ×1.7 | 0.88 / ×1.1 |
| inputs held constant (autonomous) | settles, no oscillation | settles, no oscillation |

- **The recurrent state is not an oscillator** — with constant inputs both networks settle. The cycle is a closed-loop
  property of policy + plant lag, not something the RNN generates.
- **In g488 the memory *carries* the cycle**: zeroing it removes 80 % of the band content and the output no longer resembles
  the record. The hidden state is the integrator of the attitude loop — it is where the "P" acts on accumulated bearing.
- **In the t6 gen-275 elite the memory *damps* it**: zeroing it more than doubles the band content. On the honest plant
  evolution has started to use the recurrent state as the damping term the input vector does not offer. The 50/100 ms slots
  are used the same way in both (freezing them raises band power — they are lead, not drive), and the gyro has become a
  small damping input in t6 (×1.7 without it) where g488 ignored it (×1.1).

## 4. What this says about the next move

1. **The objective is not the first lever.** The policy cannot be asked to stop a cycle whose phase budget it cannot
   observe: it has no craft-rate history, its bearing history is blind at the cycle's own period, and the one lag-free
   damping signal it has — the gyro — it has learned to ignore because the inner loop makes rate ≈ own last command. A
   reversal-rate price (T103/T104) would push on a policy that has only amplitude to give back; expect t5's pattern
   (amplitude off the rail, chatter stays) — which is exactly what gens 200–275 show.
2. **Give the policy the damping signal explicitly** — the cheapest, most honest change: a short history of its *own
   rotation* (gyro q at 50/100 ms, or its own last pitch command) in the craft block. This is the rate-feedback term a
   human designer puts in an attitude loop; it is lag-free (unlike the bearing lead) and it is exactly what the measured
   command→rate lag asks for. It also removes the ambiguity that currently forces the policy to read craft rotation out of
   target motion. Wire-format change (45 → 47–48 slots, xiao struct in lockstep) — the FR-057 extraction rule applies.
3. **Keep the objective as is for that run**: the honest plant already prices the cycle (g488 lost 8 tracking points to it),
   so if the policy *can* damp, plain tracking selection will take it there; T103/T104 are then a second step if needed.
4. Residual sim-real gap in this view: command→rate lag +2 ticks in sim vs +2–3 in flight (~25 ms); not the cause of the
   cycle, worth closing when the latency model is next touched.

t6 continues to 350 unchanged (the gen-275 call stands) — this analysis changes what the next bake carries, not this one.
