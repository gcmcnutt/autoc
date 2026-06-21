# 037 Wrap — 20 Hz control loop + M2 de-risking → handoff to 038

**Date**: 2026-06-21. Companion to [outcome.md](outcome.md) (the cadence go/no-go verdict) and
[finding.md](finding.md). This wrap covers the full branch arc: the 20 Hz deliverable **plus** the M2
experiments (t11–t15) the 20 Hz GO unblocked, the analytics toolkit built, and the architecture findings
that hand off to 038.

## 1. Core deliverable — 20 Hz is GO

The headline question ("run the control loop faster, 10 → 20 Hz") is answered: **GO**, with the 0.8 s
history window + honest servo v2. **t10 = the best M1 controller in project history** (pctInStreak 39.6%,
294/294 crash-free, pinned in `autoc-m1`). 20 Hz is the operating config. Full verdict + the cadence/servo
analysis in `outcome.md`; the bundle (2-knob rate flip, time-based log-spaced history, env-only variation
ramp, fail-loud schema boundaries) is rate-generic and stays.

## 2. The branch grew an M2 de-risking arc (t11–t15)

Because 20 Hz unblocked M2, this branch became the home for de-risking the **M2 crash penalty + tracking
depth** before 038 formally opens. The arc (all on the t10 source, 20 Hz, optical-only tracker):

| run | change | result |
|---|---|---|
| **t11** | clean 20 Hz M2 retrain (penalty OFF) | baseline |
| **t12** | hull penalty `0.5^K` | hull→0, but **displaced** failures hull→OOB; ragged (landscape sparsification) |
| **t13** | + linear-clamp OOB penalty | **collapsed** the fresh population (clamp zeroed the majority); stopped gen 7 |
| **t14** | curriculum-ramped, smooth, never-clamp penalty | fixed collapse; suppressed **both** crash modes, no displacement; + `applyCrashPenalty` all-paths fix (elite-divergence bug) |
| **t15** | streak threshold 0.5→0.3 (widen streak cone) | pctInStreak "up" but threshold-confounded; **error distance unchanged** |

## 3. The headline M2 finding: tracking is ARCHITECTURE-capped, not reward-limited

Every reward knob across t11–t15 (penalty shape, ramp, streak threshold) only **relocated trade-offs**.
The reward-invariant ground truth never moved:

- **Close tracking ~11–13% of ticks** (within 5 m), **median error ~17 m** — flat while elite fitness
  climbed −15k → −17k.
- **Perception plateaus ~70% in-FOV** (≈30% blind); **reacquire fails for 8–10 s** (`maxLost` 160–200
  ticks). All reward-invariant across runs (`mode_progress`).
- **`rnn_capacity`: not width-saturated** (eff-rank ~10/16) → it's **structure, not size**.
- **The dominant failure (overrun)** is **87% prediction-driven** (vis-loss + target-turn), only 12% pure
  speed. The pitch-up/roll-loop is recovery from a *wrong prediction*.
- **Once close, track holds 96%** → the *track controller works*; the bottleneck is **acquire +
  reacquire-through-blindness**.

→ **Reward shaping is exhausted as a depth lever.** The next lever is architecture / perception.

## 4. Analytics toolkit built

`scripts/generate_pngs.sh m2 <log>` → **11 reports** (current + `--compare` overlays): evolution,
per-axis aggressiveness / time-series, dynamics, gen_diag, intercept_analysis, **gen_runtime**
(diversity/collapse proxy), **mode_progress** (per-gen perception/track/range/reacquire), **score_by_path**
(per-path score + per-tick **error-distance** panels), rnn_capacity (W_hh SVD), tactics (A/B). Plus
**cone_topo** (config-viz of the streak cone). Maintained home in `src/analytics/`. Constitution IX
**pre-run build gate** added (it caught a real ini parse bug before a launch).

## 5. Architecture directions (→ 038 / US4) — GENERIC vs M2-FOV-SPECIFIC

The evidenced next lever is **temporal memory + an explicit target predictor** (US4), with a
**visibility-maintenance** reward for "stay on track." Crucial split (operator 2026-06-21) for 038
sequencing — **some of this is generic and could be proven on M1 first (no FOV confound); some is strictly
for the tracker's limited camera FOV** (M1's rabbit is always visible):

**GENERIC — helps M1 *and* M2** (M1 also tracks a target → benefits from better temporal prediction):
- Deeper / **non-uniform (log/geometric) history buffer** (US4 #1).
- **Two-timescale recurrence** — a structural leaky-slow channel for trajectory memory (#4); `rnn_capacity`
  says structure, not width.
- **Auxiliary target-predictor head** (#3) — trained via a **lexicase prediction-accuracy objective**
  (evolution-native, no backprop) or a gradient-pretrain-then-evolve hybrid. The pivot that also unlocks
  planning.
- The crash/OOB **penalty machinery** (OOB generic; hull is tracker-only) and the whole analytics toolkit.

**M2-FOV-SPECIFIC — strictly the tracker's limited FOV** (does not apply to M1):
- **Visibility-maintenance reward** (fly to keep the target framed) — M1 never goes blind.
- **Prediction-through-blindness / reacquire** — the 30% blind, 8–10 s gaps are a tracker-FOV problem.
- **Camera variations** (038 US2); the **overrun-from-vis-loss** failure mode.
- Note: `#2 explicit last-known target *position*` does **not** translate to attitude/optical flight
  (a stale bearing drifts with ego-rotation) — only a `time-since-seen` scalar survives; the rest folds
  into deeper gyro/attitude history (#1).

**Implication**: prove the generic temporal/predictor architecture cleanly (M1 or both), then layer the
FOV-specific perception/visibility work on M2.

## 6. Handoff to 038 ("Accurate M2")

- **Crash penalty**: t14's curriculum-ramped `0.75^K_hull × exp(−w·scale·K_oob/N)` is the validated
  mechanism (038 T001 effectively done; weights tunable).
- **Depth lever**: US4 temporal-memory + predictor (generic) + visibility reward (FOV-specific) — the real
  swing at the architecture ceiling. **US5** reward-gradient shaping (negative-ahead) is a complementary
  near-term tracking lever.
- **MoE watch**: full patrol/intercept/track/evade experts wait for *mode interference* evidence
  (one mode caps while another climbs) — instrument via `mode_progress`; the predictor/control split is the
  first MoE-lite seam.
- **Open backlog carried**: `EvalVariationScaleOverride` (robustness eval), decouple unit tests from
  production inis (joined to the config-system item), camera/CEP realism.
- **xiao M1-flight** → **039** (already re-homed).

t15 is still running (the streak experiment; reward-exhaustion already confirmed) — finish it for the
record or stop; it won't change the verdict.
