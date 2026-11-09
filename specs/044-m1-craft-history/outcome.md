# 044 M1 craft-sensor history + roll plant — outcome (closed 2026-11-08)

**Scope:** one more M1 before M2 — make the sim's roll mode honest (the 2026-10-18 flight showed it ~4.5× too slow), widen pitch
damping to the streamer, model the FC sensor filters, give the policy its own recent rotation (gyro p/q/r + accel z at 50 / 100 ms),
and fix the baseline defects (PRNG class independence, wind-draw logging, eval strength). Spec of record [spec.md](spec.md); run
record [artifacts-t1/MANIFEST.md](artifacts-t1/MANIFEST.md); gates and smokes [MANIFEST.md](MANIFEST.md); acceptance flight
[flight-analysis-20261108.md](flight-analysis-20261108.md).

**Result in one paragraph.** The roll correction worked in the air: the 3–5 Hz roll dither that rose on 10-18 fell **60 %** (101 →
41 °/s), the roll command came off the cap (3.3 → 0.1 %), and tracking improved (median distance 12.2 → 9.1 m) — on a plant whose
roll mode, re-measured from this flight, sits on the class centre. Pitch did not follow: content moved from 4–5 Hz into **2–3 Hz**, which
rose 35 % (52 → 70 °/s), and 3–5 Hz fell only 11 % against the 15 % required. **Exit verdict: partly met** — SC-001 / 003 / 004 pass,
**SC-002 misses**, SC-005 is level. The same flight's airframe fit shows **less pitch damping and a slower roll mode than on 10-18 with
the same streamer, no wind and no configuration change** — something physical changed (operator: the rubber bands holding the wing on
may be oxidized), so the pitch miss is not yet attributable to the policy. Operator decision 2026-11-08: **close 044, take the gates up
from the M2 side in [045](../045-m2-tracking/README.md)**; SC-002 is carried open (§ 4).

## 1. Results

| | 043-t6 g752 (flown 10-18, reference) | **044-t1 g800 (flown 11-08)** | criterion |
|---|---|---|---|
| roll 3–5 Hz RMS (xiao, same code) | 101 °/s | **41** (−60 %) | SC-001 < 82 / −15 % ✅ |
| pitch 3–5 Hz RMS | 56.6 | **50.4** (−11 %; blackbox −3 %) | SC-002 < 48 / −15 % ❌ |
| pitch 2–3 Hz RMS | 52.0 | **70.2** (+35 %; blackbox +30 %) | SC-002 ≤ 51 ❌ |
| median distance to rabbit | 12.2 m | **9.1 m** | SC-003 9–14 m ✅ |
| roll corner, flight FRF | 5.05 Hz | **2.97 Hz** | SC-004 inside the class (1.7–5.8 Hz) ✅ |
| 1.5–5 Hz load RMS | 1.91 g | **1.90 g** | SC-005 < 1.88 — level, not below |
| roll cmd at cap / flips/s | 3.3 % / 7.3 | **0.1 % / 5.8** | — |
| pitch cmd \|Δu\|/tick / flips/s | 0.18 / 3.8 | 0.25 / 4.3 | — |
| throttle at cap | 88 % | 78 % | — |
| sim, regiment gen 800 (`pctInStreak`) | 46.7 % (t6, its plant) | **49.5 %**, 0 crashes, 0 divergences | — |
| eval suite on the flown genome | all PASS, tier 0 bitwise | **all PASS, tier 0 bitwise** (tier1 294/294, tier3-stress 143/144) | — |

> **Addendum 2026-11-09 — span 1 was flown on a faulted nav estimate** (GPS fell below `gps_min_sats`; the horizontal estimate
> coasted, froze for 2.3 s and reset 105 m; altitude hard-reset 26 m — [flight-analysis § 6](flight-analysis-20261108.md#6-span-1-was-flown-on-a-faulted-nav-estimate-added-2026-11-09)).
> Gates without it ([§ 7](flight-analysis-20261108.md#7-the-gates-without-span-1)): roll 3–5 Hz 40 (PASS), distance 8.6 m (PASS),
> load **1.79 g — SC-005 PASS**, pitch 3–5 Hz 50.4 raw / 42.1 mean-removed (MISS / PASS by form), **pitch 2–3 Hz 70.7, +36 % — SC-002
> still MISS**. The exit verdict (partly met, SC-002 open) stands.

The operator's "moved bang-bang to roll" impression is the roll energy moving **down** (0.25–2 Hz +20 to +70 %: larger, slower swings,
very visible from the ground) while the roll *command* got calmer on every measure.

## 2. What worked — the chain

1. **The roll plant became honest** (US2): `craftClP` as a variation class centred on a 3 Hz corner with `Cl_da` scaled so steady roll
   rate per aileron is unchanged; the honesty ladder reproduced the flown genome's 3–5 Hz roll (112 vs flight 97 °/s) and closed-loop
   roll |H| before any training ([honesty.md](honesty.md)). The 11-08 flight measured its roll corner at 2.97 Hz — the class centre.
2. **One bake on it removed the roll dither in the air** — the same mechanism as 043's pitch result: the sim had low-passed dither the
   real airframe passes; once the plant passed it too, selection paid against it.
3. **Baseline fixes** (US4): class PRNG streams now independent (fmix32 sub-seeds), wind direction off the speed draw, wind draws in
   the scenario table, `EvalVariationScale` for equal-environment comparison, async elite-dmp upload (−25 % generation time,
   bitwise-neutral) ([profile.md](profile.md)).
4. **Format break done cleanly** (US1): layout v4 (53 M1 / 74 M2 inputs), an index-free layout gate (literal-index scan, layout
   signature in NN01 v2 / nn2cpp / flight log v6), decoders reading by slot name; the layout change touched only the layout header,
   topology, recording sites and the schema bump. On the aircraft the history slots are exact one/two-tick lags, pre-filled at engage.
5. **Equal-environment pick worked as a predictor for roll** (`gate_estimate.py`: g800 roll predicted 27 °/s, flew 41 — direction and
   pass both right) and **flagged the pitch risk** (2–3 Hz predicted borderline 54; flew 70).

## 3. What did not — and what is not known

- **History did not buy smoothness in simulation** (SC-006 not met: the history smoke was no smoother than plant-only in 200 gens). The
  bake kept it (clarification 1); the flight cannot separate history from the plant. The roll gain is most plausibly the plant.
- **Pitch 2–3 Hz rose.** Two candidate causes, not separated:
  (a) **the airframe on 11-08** — `airframe_id.py`, same code both days: pitch `Cm_q` −1.1 / ζ 0.38 (10-18: −3.5 / 0.55), |H| peak 2.21
  at 2.9 Hz (10-18: 1.61), roll corner 5.05 → 2.97 Hz; operator confirms same streamer, light / no wind, nothing changed — except
  possibly **oxidized wing-retention bands** (a compliant wing mount is a plausible way to lose apparent damping and move both modes);
  the size of the change (~37 % more 2–3 Hz amplification) is about the size of the rise;
  (b) **the policy** — `gate_estimate.py` already showed g800 no better than g752 at 2–3 Hz in sim, and the pitch command is busier.
- **Command transport delay unresolved.** The xiao → `rcCommand` correlation peaks at a +60–70 ms clock shift (bench and flight); the
  clock anchors are up to a tick stale, so this is an upper bound, not a measurement. The sim models 10 ms. Backlogged.

## 4. Carried open

| item | where |
|---|---|
| **SC-002 pitch** (2–3 Hz +35 %, 3–5 Hz −11 %) — re-fly g800 on fresh wing bands with an airframe re-ID (doublets) to separate airframe from policy, before any corrective bake | [BACKLOG](../BACKLOG.md) § 044 close |
| SC-005 load level, not below 1.88 g | same entry |
| Per-tick INAV sample time → measure the real command delay | BACKLOG § 044 deferrals |
| BLE flight-log downloader: resume + transport (300 kB in ~10 min) — for 045 | BACKLOG § 044 deferrals |
| Sim sensor-path gaps, accel x/y history trigger, roll step-gain coupling; training-throughput worker idle | BACKLOG § 044 implementation / T013-T014 |

## 5. Record (Constitution VIII.2)

| what | where |
|---|---|
| tags | `043-final` (`8cd5a5d`, reads every 043 artifact), `044-last-45in` (the 45-input baselines and smoke A) |
| the bake, t1 | `s3://autoc-m1/autoc-9223370244339188361-2026-10-20T16:59:47.446Z/` — 800/800 `retain=keep`, master seed 1792515587 |
| the flown genome | t1 g800 (created gen 730), `artifacts-t1/nn_weights-t1-g800.dat`, `a5ed3521ea1e70b7`, from `gen9200.dmp.zst` |
| eval dumps (`autoc-eval`) | 11 pinned `retain=keep`: the g800 suite (`eval-results/2026-10-30T15:08:39Z/`) + the four gate-estimate evals — artifacts-t1 § 6–7 |
| 043 preservation | t5 prefix `autoc-9223370245346926812-2026-10-09T01:04:08.995Z/` (gens 1 / 488 / 800 `keep`), t6 (043 artifacts-t6 § 6), 29 eval dumps — [MANIFEST.md](MANIFEST.md) § Preservation |
| smokes | A `autoc-9223370244385275114-2026-10-20T04:11:40.693Z/`, B `autoc-9223370244382121270-2026-10-20T05:04:14.537Z/` (`expire`; evidence, not milestones) |
| flights | bench `eval-results/bench-20261107/`; acceptance `flight-results/flight-20261108/` |
| tooling added | `gate_estimate.py`, `flight_gates.py`, `bench_join.py`; `nn2cpp` unrolled by default (`-p` for analytics) |
