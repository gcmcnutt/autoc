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

## 5. Reference: the full M1 input vector (verified in code 2026-10-15)

Source of truth: `NNInputs` / `PathgenInput` in [`nn_inputs.h`](../../include/autoc/nn/nn_inputs.h); one gather,
`gather_pathgen_inputs` → `writeCraftCommonInputs` in [`evaluator.cc`](../../src/nn/evaluator.cc), called verbatim by the
nn2cpp-generated xiao program (sim and flight cannot differ in slot meaning). History ring: `AircraftState::recordErrorHistory`,
17 ticks deep, written once per 20 Hz tick by crrcsim (`inputdev_autoc.cpp`) and by the xiao (`msplink.cpp`) **before** the NN runs.
Lags in ticks: {16, 8, 4, 2, 1, 0}. σ = slot standard deviation over the t6 gen-275 elite's 294 scenarios.

| slots | input | units → NN scale | time view | σ |
|---|---|---|---|---|
| 0–5 | `target_x` — unit vector to the rabbit, body x (forward) | direction cosine, raw | lags 800/400/200/100/50/0 ms | 0.44 |
| 6–11 | `target_y` — body y (right) | direction cosine | same six lags | 0.34 |
| 12–17 | `target_z` — body z (down) | direction cosine | same six lags | 0.29 |
| 18–23 | `dist` — range to the rabbit | m ÷ 26 | same six lags | 0.27 |
| 24 | `closing_rate` — (dist 50 ms ago − dist now) / 50 ms | m/s ÷ 16 | derived: a 50 ms difference of the history | 0.35 |
| 25–28 | `quat_w/x/y/z` — attitude q_EB | unit quaternion | now only | 0.32–0.63 |
| 29 | `airspeed` | m/s ÷ 13 | now only | 0.20 |
| 30–32 | `gyro_p/q/r` — body rates | rad/s ÷ 6 | **now only** | 0.33 / 0.27 / 0.18 |
| 33–35 | `accel_x/y/z` — body specific force incl. gravity | g ÷ 8 | now only | 0.07 / 0.01 / 0.25 |
| 36 | `specific_energy` — h above the deck + v²/2g | m ÷ 145 | now only | 0.09 |
| 37 | `boundary_closure_rate` — outward radial speed | m/s ÷ 13 | now only | 0.71 |
| 38 | `dist_to_boundary` — along the velocity ray | tanh(m ÷ 20) | now only | 0.03 |
| 39–41 | `inward_body_x/y/z` — unit vector to the arena centre | body frame | now only | 0.48–0.61 |
| 42–44 | `score_grad_x/y/z` — ∂score/∂position × streak multiplier | tanh(‖g‖ ÷ 0.78), direction kept | now only | 0.13–0.22 |
| — | recurrent state h (layer 2, 16 units) | tanh | whatever it learns; ρ(W_hh) ≈ 3 | — |
| — | the policy's own previous command | — | **not an input** | — |

Two properties that matter for the history question:

- **The target history is stored in the body frame of the tick it was recorded in**, not re-rotated into the current frame. A
  difference such as `target_x[now] − target_x[−50 ms]` is therefore target motion *plus* the craft's own rotation over 50 ms —
  the policy's only view of its own recent rotation, and it is confounded with the target's.
- `closing_rate` uses `kNNHistoryRecentGapSec` = 50 ms since layout v3 (t10); the comment above it in the gather still says 100 ms
  (stale from v2) — the code is right, the comment is not.

**What each lag adds** — 1 − r² between the lag slot and the 0 ms slot, median over scenarios (the share of that slot's variance
the "now" slot does not already carry):

| signal | 50 ms | 100 ms | 200 ms | 400 ms | 800 ms |
|---|---|---|---|---|---|
| `target_x` | 0.03 | 0.08 | 0.19 | 0.41 | 0.72 |
| `target_y` | 0.03 | 0.11 | 0.33 | 0.71 | 0.94 |
| `target_z` | 0.06 | 0.17 | 0.40 | 0.66 | 0.91 |
| `dist` | 0.00 | 0.01 | 0.04 | 0.16 | 0.46 |
| `gyro_q` (hypothetical history) | **0.53** | **0.93** | 0.99 | 0.99 | 1.00 |
| `gyro_p` (hypothetical history) | **0.37** | **0.73** | 0.84 | 0.96 | 0.99 |

The history budget (24 of 45 slots) is spent on the slowest signals in the vector — the 50/100 ms target and range slots are
83–100 % redundant with "now" — while the fastest signal, pitch rate, which decorrelates within 100 ms because it *is* the 2–3 Hz
cycle, has no history at all.

**Cost of adding slots.** Topology 45 → 32 → 16r → 3 = 2307 weights; every input slot costs one row into the 32-unit layer,
**+32 weights (+1.4 %)**. Gyro is already delivered to the xiao over MSP every tick (`setGyroRates`), so a gyro history is a
local ring buffer — no link traffic, and the forward pass (1.6 ms measured in flight) grows by ~1 %. The real cost is the
**format break**: `NNInputs` on the xiao, nn2cpp, data.dat / dmp readers, the renderer, the flight-log decoder, and the FR-057
baseline extraction — paid once, the same whether 2 or 6 slots are added, so one break should carry everything wanted.
The M2 vector (66 slots) shares the identical 20-slot craft block, so the same gap carries into 044 unless closed here.

## 6. Which signals get history, at which lags — and does the memory cover the previous command? (2026-10-15, operator)

Tools: [`sensor_alias_budget.py`](sensor_alias_budget.py) (blackbox, ~1 kHz, engaged spans), [`history_lag_study.py`](history_lag_study.py)
(out-of-sample R² of predicting the body rate 100/150 ms ahead — when a command issued now lands — from inputs available before
the decision; flight = fit one 10-11 flight, test the other; sim = g488 on the t6 plant, even/odd scenarios).

**Aliasing is not the constraint for gyro or accel z.** The FC's filters (gyro PT1 25 Hz + dynamic notch ≥ 30 Hz; accel biquad 15 Hz)
remove the vibration, which lives above 100 Hz in the raw gyro. Share of folded (aliased) variance in what the policy's 20 Hz point
samples carry, both flights:

| channel | sample | 50 ms difference | 100 ms difference |
|---|---|---|---|
| gyro q / p / r (filtered, = what the xiao gets) | 0.1–0.3 % | 0.5–1.4 % | 0.1–0.3 % |
| gyro q **raw** (for contrast) | 5–9 % | 28–49 % | 7–15 % |
| accel z | 0.2 % | 1.3–1.6 % | 0.2 % |
| accel x | 2–3 % | 21–24 % | 7–9 % |
| accel y | 3–10 % | 17–35 % | 5–12 % |

Gyro pitch puts 83–87 % of its power in 1–4 Hz (the cycle). Accel x and y alias badly on differences and carry nothing for prediction
(below) — **no history for them**.

**Which lags.** Pitch, flight (sim in brackets), R² of the pitch rate 100 ms ahead from pitch-rate history alone:
now 0.11 (0.00) → 0/50 ms 0.54 (0.16) → 0/50/100 0.54 (0.17) → 0…150 0.54 (0.19) → 0…300 0.56 (0.18); 0/100/200 (skipping 50) 0.50
(0.08) — the worst; the target's log set 0/50/100/200/400/800 0.57 (0.17). Roll needs the 100 ms lag (0.09 → 0.19 flight).
Physics: for a 2.5–3 Hz mode a 50 ms lag is ~50° of phase, 100 ms is ~quadrature (the best-conditioned pair for amplitude + phase),
200 ms is a half period (an inverted copy) and 400 ms a full period (an identical copy — blind). **Uniform 50/100 ms is the set.**

**The short-period state is already observable today.** It is second-order (q, α), and accel z measures α through lift: pitch rate +
accel z **now** predict the rate 100 ms ahead at 0.52 flight (0.18 sim) — the same as all six sensors now (0.52) or two gyro samples
(0.54). Adding the previous command, which the recurrent layer holds exactly by construction (o_{t−1} = tanh(W3 h_{t−1} + b3)), lifts
it to **0.76 flight (0.32 sim)**. Whole proposals, each including that remembered command:

| proposal | + slots | + weights | pitch flt +100 / +150 | pitch sim +100 / +150 | roll flt +100 / +150 | roll sim +100 / +150 |
|---|---|---|---|---|---|---|
| today: six sensors now | 0 | 0 | 0.759 / 0.625 | 0.322 / 0.198 | 0.484 / 0.224 | 0.672 / 0.476 |
| A: gyro p, q, r at 0/50/100 | 6 | 192 | 0.761 / 0.619 | 0.416 / 0.230 | 0.498 / 0.295 | 0.681 / 0.527 |
| **B: A + accel z at 0/50/100** | **8** | **256 (+11 %)** | **0.770 / 0.636** | **0.441 / 0.259** | **0.500 / 0.299** | **0.682 / 0.529** |
| C: gyro + accel z at 0/50/100/150 | 12 | 384 | 0.775 / 0.646 | 0.441 / 0.257 | 0.503 / 0.300 | 0.681 / 0.531 |
| D: all six on the target's log lags | 30 | 960 (+42 %) | 0.799 / 0.668 | 0.468 / 0.268 | 0.540 / 0.318 | 0.702 / 0.557 |
| E: B + explicit own command t−2, t−3 | 12 | 384 | 0.777 / 0.644 | 0.476 / 0.286 | 0.503 / 0.308 | 0.695 / 0.538 |

**The memory and the previous command** — R² of decoding the policy's command at t−j from h(t−1), the state the recurrent layer
receives at tick t:

| genome | j = 1 | 2 | 3 | 4 | 6 |
|---|---|---|---|---|---|
| g488 pitch / roll | 1.00 / 1.00 | 0.65 / 0.53 | 0.31 / 0.16 | 0.14 / 0.09 | 0.14 / 0.01 |
| t6 gen 275 pitch / roll | 1.00 / 1.00 | 0.35 / 0.17 | 0.45 / −0.05 | 0.35 / 0.06 | 0.19 / 0.01 |

The previous command is there exactly; 100–150 ms back (the commands currently in flight through the rate loop) only partly. Making
them explicit (E vs B) is worth +0.03 in sim, +0.01 in flight — optional, not first.

**⚠️ Correction to § 4 item 1.** § 4 said the policy "cannot be asked to stop a cycle whose phase budget it cannot observe". That was
too strong: the short-period state is observable from today's inputs (pitch rate + accel z now) plus the memory's previous command,
at R² 0.76 in flight. What history adds is an *easier and more robust* estimate — modest in the clean flight air (+0.01–0.04), larger
in the turbulent training environment where learning happens (+0.12 at 100 ms, 0.32 → 0.44), and a one-layer path for rate-difference
(derivative / phase-lead) terms that today must be built through the recurrent layer. The policy's command does not use the gyro
(ΔR² 0.003, § 2) although the information is present, so history is an **enabler**, not by itself the fix; whether selection then
rewards damping is still the gen-350 question.

**Recommendation → T121: proposal B** — gyro p, q, r and accel z at 50 and 100 ms (+8 slots, +256 weights), a second lag table for
craft history (the target's stays), no history for accel x/y, no explicit command history in the first step. Companion fidelity item
for the same build: the sim still does not model the FC's sensor filter delays (gyro ~6.4 ms, accel ~21 ms — `sensor_lpf.h`, option (a):
filter at the 5 ms FDM sub-step). With accel z differences becoming inputs, the 21 ms accel lag is no longer negligible.
