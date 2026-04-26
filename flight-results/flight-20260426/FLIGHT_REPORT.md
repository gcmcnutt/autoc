# Flight 2026-04-26 — cadence7-redux (cadence7 replicate, new seed)

**Date**: 2026-04-26 ~13:55 local
**NN weights**: cadence7-redux gen 400, best=−33037 (~92 % of cadence7's
−35951 — within Seed=−1 run-to-run variance) — flown S3 artifact
[`gen9600.dmp`](https://s3/...) under run id
`autoc-9223370259677948992-2026-04-26T04:13:46.815Z`.
**Xiao binary**: 027 tree, RNN compiled but `NN_RECURRENT[]` all-false; 1667
weights (cadence7 topology).
**INAV**: same fork as 04-22 (world→body blackbox quat, MSP override
engage path).
**Wind**: less than 04-22; nominal/light (pilot reported).

**Overall**: **first successful path-tracking flight on a learned policy.**
All three engage spans ran to **path-end**, not pilot abort — first time
across any 027/cadence flight. Roll bang-bang dropped meaningfully vs
the 04-22 cadence7 flight; pitch and throttle behaviour roughly
unchanged. This is **a second cadence7-class flight on a different seed
in calmer wind** — not a test of the 027 RNN/multi-objective bet, which
was deliberately disabled in the binary (see "What flew" below).

## Engage spans

Three engage spans, all path-end terminated:

| Span | Duration | NN evals | Path | Avg airspeed | Behaviour |
|---|---|---|---|---|---|
| 1 | 19.5 s | 170 | 0 | 18 m/s | Path-end termination, full duration |
| 2 | 24.1 s | 219 | 2 | 16 m/s | Path-end termination, longest span |
| 3 | 23.2 s | 211 | 1 | 17 m/s | Path-end termination |

vs 04-22 cadence7 flight: spans were 8.4 s / 18.4 s / 13.5 s, terminated
by pilot disengage on 1 and 3. Today's spans are 2–3× longer with no
disengage. Lighter wind contributes; longer-running policy likely also
contributes.

## Convention / sensor audit

Per-span anchors via the new
[`specs/027-recurrent-nn/join_flight_analysis.py`](../../specs/027-recurrent-nn/join_flight_analysis.py)
(027-variant with build-aware polarity bracketing and dual-convention
quat compare). All three spans:

| Anchor | Status |
|---|---|
| A. Gravity (FRD body Z) | ✓ pass on all spans |
| B. xiao quat ↔ INAV blackbox | ~ ok, sample-time slop (RMS 0.27–0.37 with INAV stored as `world→body` and xiao as `body→world` — the script picks the right convention; the conjugate-aware delta is sample-rate noise) |
| C. cos(nose, vel) using xiao quat | +0.91–0.94 ✓ forward flight |
| D. Control chain (end-to-end rcData→gyro) | +0.78 to +0.82 @ 8–14 ms, all stages match build's expected polarity |

The "negative" correlations on `NN→rc pitch` and `rcData→servo*` are
**by design** (msplink `convertPitchToMSPChannel` intentionally inverts;
INAV elevon mix inverts both servos). They cancel through the chain;
the only assertion that pins reality — `rcData→gyro` — is positive on
all axes. The 027 join script now labels these in `[brackets]` per
stage so they don't false-flag.

## Control behaviour — vs cadence7 04-22

Per-span and pooled `|out|` / `|Δout|` / saturation:

| Metric | cadence7 04-22 | cadence7-redux 04-26 | Δ |
|---|---|---|---|
| `<\|out\|>` total / tick | 2.10 | 2.11 | flat |
| `<\|Δout\|>` per tick | 1.02 | 0.88 | **−14 % chatter** |
| `<\|pt\|>` | 0.60 | 0.76 | up (more pitch use) |
| `<\|rl\|>` | 0.60 | 0.40 | **−33 % roll** |
| `<\|th\|>` | 0.94 | 0.95 | flat (Vmax-pinned) |
| roll near-zero band ([−0.3, +0.3]) | 22 % | **52 %** | **roll quieted** |
| roll saturation (\|·\| > 0.95) | 24 % | 9 % | halved |
| pitch saturation | 17 % | 8 % | halved |
| throttle saturation | 88 % | 88 % | unchanged |

The roll-axis result is the most striking: today's policy spends
half its ticks with roll near zero, vs 22 % on the 04-22 flight.
Pitch saturation is also halved. Throttle stays pinned high (Vmax /
rabbit-speed envelope problem, identical to 04-22).

**Caveat**: this isn't from the 027 architectural change (RNN was
off in the binary). It's a **different 400-gen seed of the same
cadence7 architecture**. Two non-controlled variables changed
between flights: training seed and field wind. Today's data is
not strong enough on its own to attribute the smoothness gain to
either, but the run-to-run training variance (cadence7 final
−35951 vs cadence7-redux −33037) is small enough that wind is
likely the dominant factor.

## End-to-end physics

Cmd → gyro lag and correlation per span (from `join_flight_analysis`):

| Span | rcData[0] → gyro p (roll) | rcData[1] → gyro q (pitch) |
|---|---|---|
| 1 | +0.79 @ 12 ms | +0.80 @ 8 ms |
| 2 | +0.82 @ 12 ms | +0.76 @ 8 ms |
| 3 | +0.81 @ 14 ms | +0.78 @ 10 ms |

Flight FC running INAV MANUAL during MSP override (no inner ACRO
PID). The ~10 ms cmd→gyro lag is servo + aero response. Identical
shape to 04-22 — no regression; the chain just works.

## What flew vs what didn't

The 027 tree carries D-simple recurrent and 3-axis lexicase
infrastructure, but the binary built and flashed today is in
**diagnostic mode** ("cadence7-redux") — see commit `3972afc` for
the rationale and restore points (`git grep CADENCE7-REDUX`):

- `NN_RECURRENT[]` all-false → 1667-weight feedforward, identical
  topology to cadence7.
- `selection.cc` lexicase pool: tracking-only (245 test cases) —
  the v3 energy and v4 stability `pool.push_back` calls are
  commented out.
- Four `Selection027*` multi-objective tests renamed `DISABLED_`.

The three primary 027 training experiments (rnn1, rnn2, rnn3) all
failed to descend to cadence7's fitness plateau:

- **rnn1**: D-simple + C2 smoothness (Σ\|Δout\|) — best stalled at
  −8437 by gen 312 vs cadence7's −35951.
- **rnn2**: D-simple + C2 v3 energy (Σ(out_th−1)/2) — final −7152.
- **rnn3**: D-simple + C2 v3 energy + v4 stability (3-axis lexicase)
  — final −12626.

cadence7-redux (FF, tracking-only) recovered to −33037 at gen 400 —
clean infrastructure, no bug. The rnn1/2/3 outcomes are design
issues to be tackled in 028.

[027 evolution-progress for rnn1](../../specs/027-recurrent-nn/rnn1_evolution.png) ·
[rnn2](../../specs/027-recurrent-nn/rnn2_evolution.png) ·
[rnn3](../../specs/027-recurrent-nn/rnn3_evolution.png) ·
[cadence7-redux baseline](../../specs/027-recurrent-nn/cadence7redux_evolution.png)

## Implications

- **Iteration of cadence7, not validation of 027.** The flight is
  the second flight of the cadence7-class controller, with a
  different training seed and lighter wind. It validates the new
  build infrastructure (multi-objective `ScenarioScore`,
  `gp_fitness` migration, hidden-state scaffolding) by reproducing
  cadence7-class flight behavior when those features are toggled
  off.
- **027's primary bet (D-simple + C2-via-lexicase) was not
  validated** in sim and was not flown. Carries forward to **028
  — deeper-rnn**.
- **Bang-bang has structural ceiling that hardware tweaks alone
  won't break.** The roll-axis improvement here is most plausibly
  attributable to wind, not policy — so the underlying problem
  (architectural lack of memory and lack of action-rate
  regularization in selection) remains.

## Artifacts

All in this directory:

- `bangbang_flight_*.png` — per-span output time series and histograms
  (from `specs/024-sim-real-fidelity/plot_bangbang_flight.py`).
- `join_analysis_*.png` / `.csv` — per-span 10-panel diagnostic from
  the 027-variant `join_flight_analysis.py` (clean polarity brackets,
  dual-convention quat compare).
- `blackbox_log_2026-04-26_135542.{01,02}.csv` — the two flights;
  `.02` is the cadence7-redux flight; `.01` is an earlier flight on
  the prior 024 binary (cadence7 weights, same airframe, same field
  conditions earlier in the day) — included for reference.
- `flight_log_2026-04-26T21-03-30.txt` — xiao log paired with `.02`.
- `flight_log_2026-04-26T21-09-52.txt` — xiao log from a later
  power-cycle (same day, no engage).
