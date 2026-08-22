# pre-break comparator extracts (T011a)

Extracted **2026-08-10**, before T044's `EvalResults` version bump makes these dmps unreadable to a 041
binary. From that commit onward this directory is the only route to either comparator.

| file | source | rows |
|---|---|---|
| `prior-m1-gen800-physics.csv.gz` | `s3://autoc-m1/autoc-9223370253553029228-2026-07-06T01:35:46.579Z/gen9200.dmp.zst` (gen 800, pathgen, 294 scenarios) | 132,462 |
| `040-t4-m2-gen800-physics.csv.gz` | `s3://autoc-m2/autoc-9223370251039771221-2026-08-04T03:43:24.586Z/gen9200.dmp.zst` (gen 800, tracker) | 131,802 |
| `*-meta.yaml` | the same dmps' metadata block — per-scenario seeds, crash reasons, scores | — |
| `*.stderr.txt.gz` | the extraction's own join diagnostics, kept as evidence | — |

Regenerated with (gen 800 is the latest in each run, so no `--gen` needed):

```sh
build/dmp-dump "s3://autoc-m1/autoc-9223370253553029228-2026-07-06T01:35:46.579Z/" \
    --csv-only --physics -i autoc.ini | gzip -9 > prior-m1-gen800-physics.csv.gz
```

⚠️ That command only works from a **pre-T044** binary. After the version bump it fails loud by design.

## ⚠️ What these files DO and DO NOT contain

**Fully preserved — 100% of ticks:** every column sourced from `aircraftStateList` / `cameraViewList` /
`targetTrajectoryList`. That is position, attitude, velocity, the three NN command outputs, wind, the
path/target geometry, `stpPt`, the beacon span/CEP/tilt sensors, and `tSee`. So both of T011a's stated
deliverables survive:

- the **prior M1's per-regime control profile** (SC-007a) — `dCtrl` and `⟨|u|⟩` per axis, per regime;
- the **blind-gap distribution** (FR-024b) — derivable from the M2 file's CEP / `tSee` columns.

**NOT preserved — 0.89% of ticks:** the physics columns (`accX/Y/Z`, `odbP/Q/R`, `alpha`, `vRelWind`, and
the derived `sfx_g/sfy_g/sfz_g/nz_g`). Coverage is **1,176 of 132,462 ticks** in each file: exactly the
first **4 control ticks (200 ms)** of each of the 294 scenarios. Empty fields elsewhere — deliberately
empty rather than carrying the stale 200 ms sample forward.

### Why, and what it costs

`crrcsim/src/mod_fdm/fdm_larcsim/fdm_larcsim.cpp:74` caps the trace:

```c
static const uint32_t MAX_TRACE_STEPS = 35;  // Capture first 35 steps (~105ms) to get just past
                                             // 100ms GP eval and identify RNG divergence point
```

It was built as an **RNG-divergence debugging aid**, not a flight recorder, and the cap has been there
since long before 041. T010's task note ("this data is already recorded for every elite reeval … it works
on current dmps") is true of the *plumbing* but not of the *coverage*.

Consequences, stated plainly:

1. The **load half of Study A cannot be run on the prior M1**, ever. That run is finished; 200 ms of entry
   transient per scenario is not a per-regime load profile, and the peak-load statistic (the
   damage-relevant one, SC-008) is exactly what a 200 ms window cannot see. Sampled over what does exist,
   `nz_g` spans −0.57 … +2.45 (mean 0.67) — plausible entry-transient numbers, and not comparable to the
   ±11 g excursions the flight reports quote.
2. The **control half of Study A is unaffected** and runs on these files as planned.
3. **RESOLVED 2026-08-10** (spec.md § Clarifications, session "where load data actually comes from"):
   `MAX_TRACE_STEPS` **stays as it is** — it was built for old determinism troubleshooting and still serves
   that; uncapped it would write ~1.01 GB per elite dmp against today's 32.9 MB, most of it time-degenerate.
   Load for the **new** M1 instead comes from the recorded **NN input** `ACCEL_Z`, which lands for free once
   T039 adds the slot, because `AircraftState` already serializes the whole input block every tick. 20 Hz
   samples only; no gravity and no world accel are stored, since flight has neither.

So the `nz_g` column in these files is **entry-transient only, permanently**. Nothing will improve it — the
runs are finished. Use these files for the **control** half of Study A (`dCtrl`, `⟨|u|⟩`, regime
classification, blind-gap distribution), which is complete and exact, and get every load number from the new
M1's own dmp.

---

## ⚠️ LIMITATION FOUND 2026-08-22 — the physics COLUMNS are 4 ticks per scenario

The `--physics` dumps here carry `alpha`, `vRelWind`, `sfx/y/z_g` and `nz_g` on
**ticks 1–4 of each scenario only** — 1,176 of 132,462 rows in the prior-M1
archive (0.89%), and all of it the **entry transient**.

⛔ **Taken at face value the archive says the prior M1 never exceeded 2.45 g.**
Its actual peak is **10.50 g**. Anyone reading the physics columns directly will
get a badly wrong answer with no indication anything is missing.

⭐ **The non-physics columns are fully populated** — `out_pt/rl/th`, `dist`,
`stpPt`, `qw..qz`, `vx/vy/vz`, `px/py/pz` are on every row. So throttle,
stability, regime and trajectory analysis off this archive are sound; only the
FDM-computed physics block is sparse.

**Workaround, implemented in `src/analytics/g_load.py`:** reconstruct load factor
from the velocity trace (`dv/dt − g`, rotated world→body, Z component, negated)
and validate against the sparse real `nz_g` rows. Measured **corr +0.9951,
MAE 0.028 g**; the code refuses to plot a reconstruction below corr 0.95.

⚠️ **Units gotcha settled by that validation**: `vx/vy/vz` are **m/s** while
`vRelWind` in the same file is **ft/s**. Assuming ft/s gives corr 0.751 /
MAE 0.230 g; m/s gives 0.995 / 0.028 g. Decided by measurement, not by reading.
