# Phase 1 Data Model — 035

Structs / schemas touched. No cereal version bump (greenfield, fail-loud read per Principle V +
[feedback_no_cereal_versioning]).

---

## ScenarioScore (`include/autoc/eval/fitness_decomposition.h:68`)

The per-scenario score record used as the lexicase test-case currency.

| Field | Type | 035 change |
|---|---|---|
| `score` | `gp_fitness` | unchanged (tracking; negated points, lower=better) |
| `stability_score` | `gp_fitness` | unchanged value; axis stays **OFF** in selection (FR-008) |
| `energy_score` | `gp_fitness` | **semantics changed** — now convex throttle integral `Σ_t ((out_th+1)/2)²` (FR-001b/R1), replacing `Σ(out_th−1)/2`. Field name + type unchanged; only the computation in `fitness_decomposition.cc` changes. Lower=better. |
| `crashed`, `crashReason` | bool / enum | unchanged (crash_cost field **NOT** added — hull-crash deferred to future feature) |
| `steps_completed/total`, `maxStreak`, `totalStreakSteps`, `maxMultiplier` | int/gp_fitness | unchanged |
| `tracker_diag` | TrackerDiag | unchanged |

**Validation:** `energy_score ≥ 0` (sum of squares; was previously ≤ 0 — sign flip is the fix).
Comparable in magnitude to `score`'s per-tick contributions so MAD epsilon scales sensibly.

**Explicitly not added:** `crash_cost` — deferred (spec FR-008b → BACKLOG future feature).

---

## energy_score computation (`src/eval/fitness_decomposition.cc:174-181`)

```
// OLD (placeholder, sign-wrong):  energyAccum += (out[2] - 1.0) / 2.0;   // range [-1,0]
// NEW (FR-001b/R1):
gp_fitness thr = (static_cast<gp_fitness>(out[2]) + 1.0) / 2.0;          // tanh→[0,1]
energyAccum += thr * thr;                                                 // convex, ≥0
```

Reads `out[2]` (throttle) from `AircraftState` NN outputs (`aircraft_state.h:476-479`,
`setNNData`). Per-tick, all ticks with NN data; no streak multiplier, no normalization (parity
with prior accumulation shape).

---

## Lexicase test-case pool + epsilon (`src/eval/selection.cc:57-106`)

**Pool entries** (per scenario):
```
pool.push_back({s, &ScenarioScore::score,        eps_for(score, s)});   // tracking (unchanged)
pool.push_back({s, &ScenarioScore::energy_score, eps_for(energy, s)});  // FR-001: UNCOMMENTED, gen-0
// stability_score: stays commented (FR-008, OFF)
```

**Epsilon (FR-003/R2):** replace the literal `0.5` floor with:
```
LexicaseEpsilonMode == constant : eps = 0.5                       // historical reproducibility
LexicaseEpsilonMode == mad      : eps = MAD(field over surviving candidates)   // default
```
MAD = median(|x − median(x)|), per-axis, recomputed at each test case over the alive candidate
set. Existing relative term (`max(eps, |best|·epsilon)` at `selection.cc:92`) is reconsidered —
with MAD as the floor, the relative multiplier may be redundant; research/impl decides (keep
behavior identical under `constant`).

---

## EvalResults / dmp schema (cereal)

| Aspect | Change |
|---|---|
| per-tick `aircraftStateList` | unchanged (already carries pos/quat/vel/commands/NN I/O) |
| derived columns (dhome/dist/along/stpPt/mult/rampSc/hull) | **NOT stored** — recomputed by `dmp-dump` from existing fields (FR-P01 default) |
| container | now **zstd-compressed** at the serialization boundary (R4); on-disk key `gen<N>.dmp.zst` |
| provenance | `stampEvalResultsProvenance` timestamps remain → dmp bytes non-deterministic (drives FR-P04 gate basis) |
| version field | NOT bumped (greenfield); reader fails loud on structural mismatch |

`data.dat` text writer: **removed entirely** (FR-P05) — no schema, no dual-write.

---

## Config (`include/autoc/util/config.h`, `src/util/config.cc`) — via AUTOC_CONFIG_FIELDS X-macro

| Key | Type | Default | Purpose |
|---|---|---|---|
| `S3Bucket` | string | per-mode | `autoc-m1` / `autoc-m2` / `autoc-eval` (FR-P07/P08) — ini-level, no code default that masks |
| `LexicaseEpsilonMode` | enum string | `mad` | `mad` \| `constant` (FR-003/R2) |

Existing `Mode`, `PopulationSize`, `WindScenarios`, `EvaluateMode`, `TrackerSourceRun` unchanged.
`TrackerSourceRun` value repointed in `autoc-eval-tracker.ini` (FR-P13).

---

## Run-id naming (`include/autoc/util/run_id.h`)

| Before | After |
|---|---|
| `runIdPrefixForMode(mode)` → `"tracker-"` (M2) / `"autoc-"` (M1) | collapse to uniform `"autoc-"` for all modes (FR-P07b) — bucket is the discriminator |

Key shape (all buckets, all modes): `autoc-<INT64_MAX−ms>-<ISO8601>Z/gen<10000−N>.dmp.zst`.
Update `tests/run_id_prefix_tests.cc` + the header comment.

---

## S3 object tags (FR-P10)

Every `PutObject` (both sites `src/autoc.cc:1448`, `1640`) sets `Tagging: retain=expire`.
Lifecycle rule (per bucket) deletes `retain=expire` at 30 days; `retain=keep` preserved; untagged
never matched (fail-safe).
