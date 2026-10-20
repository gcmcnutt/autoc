# 044 — MANIFEST (run record, preservation, baselines)

Living record for 044's gates, baselines and smokes. The bake gets its own `artifacts-t1/MANIFEST.md` (T048).

## § Preservation (T001, FR-005) — 2026-10-19

**Tags (local; operator pushes)**

| tag | commit | crrcsim | reads |
|---|---|---|---|
| `043-final` | `8cd5a5d` | `7f345c5` | every 043 artifact: dmp schema 4, NN01 v1 genomes, xiao flight log v5 |
| `044-last-45in` | (the T026 record commit) | `1cb45c3` | 044 smoke A and the 45-input baselines (dmp schema 5) |

**Training runs (`autoc-m1`)** — already pinned by 043, re-verified by sampling 2026-10-19:

| run | prefix | check |
|---|---|---|
| t5 (flown gen 488) | `autoc-9223370245346926812-2026-10-09T01:04:08.995Z/` | `gen9999` (gen 1), `gen9512` (gen 488), `gen9200` (gen 800): all `retain=keep` |
| t6 (flown gen 752) | see `../043-acro-dual-loop/artifacts-t6/MANIFEST.md` § 6 | 800/800 tagged, verified at the 043 close |

**Evaluation dumps (`autoc-eval`)** — 29 objects re-tagged `retain=keep` (0 failures; 6 sampled, all `keep`). Columns: local
`eval-results/<run>/`, genome, S3 key.

| eval run | genome | key |
|---|---|---|
| 2026-10-12T14:54:25Z | `nn_weights-t5-g488.dat` | `autoc-9223370245037909718-2026-10-12T14:54:26.089Z/gen9999.dmp.zst` |
| 2026-10-12T14:54:55Z | `nn_weights-t5-g488.dat` | `autoc-9223370245037880519-2026-10-12T14:54:55.288Z/gen9999.dmp.zst` |
| 2026-10-13T23:38:54Z | `nn_weights-t5-g800.dat` | `autoc-9223370244919954306-2026-10-13T23:40:21.501Z/gen9999.dmp.zst` |
| 2026-10-13T23:38:54Z | `nn_weights-t5-g800.dat` | `autoc-9223370244919970360-2026-10-13T23:40:05.447Z/gen9999.dmp.zst` |
| 2026-10-13T23:38:54Z | `nn_weights-t5-g800.dat` | `autoc-9223370244919984968-2026-10-13T23:39:50.839Z/gen9999.dmp.zst` |
| 2026-10-13T23:38:54Z | `nn_weights-t5-g800.dat` | `autoc-9223370244919988227-2026-10-13T23:39:47.580Z/gen9999.dmp.zst` |
| 2026-10-13T23:38:54Z | `nn_weights-t5-g800.dat` | `autoc-9223370244919997980-2026-10-13T23:39:37.827Z/gen9999.dmp.zst` |
| 2026-10-13T23:38:54Z | `nn_weights-t5-g800.dat` | `autoc-9223370244920019171-2026-10-13T23:39:16.636Z/gen9999.dmp.zst` |
| 2026-10-13T23:38:54Z | `nn_weights-t5-g800.dat` | `autoc-9223370244920041253-2026-10-13T23:38:54.554Z/gen9999.dmp.zst` |
| 2026-10-13T23:40:21Z | `nn_weights-t5-g488.dat` | `autoc-9223370244919866925-2026-10-13T23:41:48.882Z/gen9999.dmp.zst` |
| 2026-10-13T23:40:21Z | `nn_weights-t5-g488.dat` | `autoc-9223370244919882500-2026-10-13T23:41:33.307Z/gen9999.dmp.zst` |
| 2026-10-13T23:40:21Z | `nn_weights-t5-g488.dat` | `autoc-9223370244919898801-2026-10-13T23:41:17.006Z/gen9999.dmp.zst` |
| 2026-10-13T23:40:21Z | `nn_weights-t5-g488.dat` | `autoc-9223370244919902521-2026-10-13T23:41:13.286Z/gen9999.dmp.zst` |
| 2026-10-13T23:40:21Z | `nn_weights-t5-g488.dat` | `autoc-9223370244919912136-2026-10-13T23:41:03.671Z/gen9999.dmp.zst` |
| 2026-10-13T23:40:21Z | `nn_weights-t5-g488.dat` | `autoc-9223370244919932573-2026-10-13T23:40:43.234Z/gen9999.dmp.zst` |
| 2026-10-13T23:40:21Z | `nn_weights-t5-g488.dat` | `autoc-9223370244919953811-2026-10-13T23:40:21.996Z/gen9999.dmp.zst` |
| 2026-10-18T14:51:31Z | `t5-g488-s1.dat` | `autoc-9223370244519684015-2026-10-18T14:51:31.792Z/gen9999.dmp.zst` |
| 2026-10-18T14:52:00Z | `t6-g752-s1.dat` | `autoc-9223370244519655067-2026-10-18T14:52:00.740Z/gen9999.dmp.zst` |
| 2026-10-18T14:52:33Z | `t6-g621-s1.dat` | `autoc-9223370244519622541-2026-10-18T14:52:33.266Z/gen9999.dmp.zst` |
| 2026-10-18T14:53:03Z | `t6-g523-s1.dat` | `autoc-9223370244519592681-2026-10-18T14:53:03.126Z/gen9999.dmp.zst` |
| 2026-10-18T14:53:31Z | `t6-g500-s1.dat` | `autoc-9223370244519564253-2026-10-18T14:53:31.554Z/gen9999.dmp.zst` |
| 2026-10-18T14:54:01Z | `t6-g370-s1.dat` | `autoc-9223370244519534587-2026-10-18T14:54:01.220Z/gen9999.dmp.zst` |
| 2026-10-18T22:35:14Z | `nn_weights-t6-g800.dat` | `autoc-9223370244491770418-2026-10-18T22:36:45.389Z/gen9999.dmp.zst` |
| 2026-10-18T22:35:14Z | `nn_weights-t6-g800.dat` | `autoc-9223370244491784781-2026-10-18T22:36:31.026Z/gen9999.dmp.zst` |
| 2026-10-18T22:35:14Z | `nn_weights-t6-g800.dat` | `autoc-9223370244491799626-2026-10-18T22:36:16.181Z/gen9999.dmp.zst` |
| 2026-10-18T22:35:14Z | `nn_weights-t6-g800.dat` | `autoc-9223370244491802820-2026-10-18T22:36:12.987Z/gen9999.dmp.zst` |
| 2026-10-18T22:35:14Z | `nn_weights-t6-g800.dat` | `autoc-9223370244491812515-2026-10-18T22:36:03.292Z/gen9999.dmp.zst` |
| 2026-10-18T22:35:14Z | `nn_weights-t6-g800.dat` | `autoc-9223370244491834975-2026-10-18T22:35:40.832Z/gen9999.dmp.zst` |
| 2026-10-18T22:35:14Z | `nn_weights-t6-g800.dat` | `autoc-9223370244491861538-2026-10-18T22:35:14.269Z/gen9999.dmp.zst` |

The `/tmp/.../evalcand/*-s1.dat` genomes of the 10-18 T122 runs are the archived `artifacts-t6/` / `artifacts-t5/` genomes with
`variation_scale` patched to 1.0 by `set_eval_scale.py` (seed 20261018).

## § Starting point (T005) — 2026-10-19

Branch head `68ec066` (crrcsim `7f345c5`), still 45 inputs and the t6 plant.

| gate | result |
|---|---|
| clean `scripts/rebuild-perf.sh` | OK, 5 min 02 s |
| `ctest` | 53/53 passed |
| xiao `pio run` | SUCCESS (RAM 53.5 %, flash 45.5 %) |
| t6 g800 tier 0, seed 1791951530 | **PASS bitwise**: `-93924.534457` = stored, 294/294 |

## § Post-stream-fix reference (T012) — 2026-10-19

Build: US4 fixes in (fmix32 class sub-seeds, wind direction from its own wind-stream slots, `EvalVariationScale`, wind-draw columns),
45 inputs, t6 plant. Every scenario draw changed by design, so these **replace T005's numbers as the reference for any speed-up
judged before the plant change** (T013/T014). Each run twice: identical.

| genome | tier 0 seed | scale | fitness | completed |
|---|---|---|---|---|
| t6 g800 | 1791951530 | stored (1.0) | `-82017.470057` | 291/294 |
| t6 g752 | 20261018 | stored (0.9474) | `-77966.093110` | 290/294 |
| t6 g752 | 20261018 | `EvalVariationScale = 1.0` | `-75353.709790` | 290/294 |

`eval_suite.sh` gained a 4th argument `[eval_scale]` (FR-011) that sets `EvalVariationScale` on every tier; the log line
`EvalVariationScale: stored 0.9474, used 1.0000 (EvalVariationScale override)` confirms it.

**Second draw-reuse defect fixed alongside FR-009** (found while implementing T011): autoc drew the per-scenario wind *direction* from
a fresh `ClassPRNG(wind)`, i.e. wind-stream draws [0..1] — the same values the worker used as the gust seed [0] and the wind-**speed**
draw [1], so direction was a function of speed (shared Box–Muller input). The direction now comes from slots [5..], after the worker's
gust seed and four envelope draws (`drawWindClass`, `wind_variation.h`); test `ScenarioDrawsMatchTheWorkerChain_044`.

**Bitwise-neutral speed-up adopted (T014)**: async elite-dmp upload — smoke gens 1–14 best/avg/worst identical to the sync build,
generation time −25 %. Details in `profile.md`.

## § New baselines on the 044 plant (T026) — 2026-10-19

Build: US4 + US2 (fmix32 streams, wind slots, `EvalVariationScale`, async upload, craftClP class + `Cl_da` scaling, CmQ −2.25,
FC sensor filters at the sub-step), **45 inputs**, dmp schema v5. Clean `rebuild-perf.sh` OK; ctest 53/53; xiao `pio run` SUCCESS.
Honesty ladder: `honesty.md` (rungs 2–3 match the 2026-10-18 flight; rung-1 steady-gain line explained, not re-tuned).

| tier-0 baseline (stored scale) | seed | fitness | completed | repeat |
|---|---|---|---|---|
| t6 g800 | 1791951530 | `-69975.152740` | 290/294 | identical ×2 |
| t6 g752 | 20261018 | `-67058.096781` | 293/294 | identical ×2 |

**Eval-vs-training bitwise gate**: 3-generation training run of the smoke regiment on this build (`logs/autoc-044-T026-gate.log`,
seed 20261019, prefix `autoc-9223370244385741393-2026-10-20T04:03:54.414Z`; `NN_ELITE_SAME` gens 1–3), gen-3 elite extracted with
`nnextractor` and re-evaluated in eval mode with the smoke regiment and the run's seed: **`NN_EVAL_SAME: fitness=-70.392779`** = stored.

Tag `044-last-45in` = the commit that records this section (local; operator pushes).

## § Smoke A — plant only, 45 inputs (T027) — launched 2026-10-19 21:11 PDT

| field | value |
|---|---|
| tree | worktree `/home/gmcnutt/autoc-044-smokeA` at tag `044-last-45in` (`ca49edd`, crrcsim `1cb45c3`), own clean `rebuild-perf.sh` (ctest 53/53); the main tree's `build/` stays free for US1 |
| launch | `bash scripts/train.sh specs/044-m1-craft-history/smoke.ini logs/autoc-044-smokeA-plant.log` (from the worktree, detached) |
| regiment | smoke.ini: pop 3000, 6 paths × 16 winds = 96 scenarios, 200 gens, ramp step 10 (full strength from gen 191), seed **20261019** |
| S3 prefix | `s3://autoc-m1/autoc-9223370244385275114-2026-10-20T04:11:40.693Z/` (`retain=expire`; the smoke is evidence, not a milestone) |
| log | worktree `logs/autoc-044-smokeA-plant.log` → copy to the main tree's `logs/` at the end, then `git worktree remove ../autoc-044-smokeA` |

## § Index-free gate green on the 45-input layout (T028–T038) — 2026-10-19

- One `CMakeLists.txt` edit (ctests `input_layout_literals`, `flightlog_decode_py`; fixture dir for the round trip) + clean
  `rebuild-perf.sh` (with smoke A training from its worktree — every running binary verified to be the worktree's).
- Gate (`scripts/check_input_layout_literals.py`): first run 5 hits (both decoders' `45`, three 043 scripts); widened to slice/range
  bounds, it also found `row[i0:i0 + 45]` ×2 and **`A[:, 1 + 30:1 + 33]` — the gyro slots by position** in
  `rate_history_availability.py`. Now **clean (168 files)**.
- Layout signature: FNV-1a 64 over the slot **enum names** + `kNNHistoryLayoutVersion` (display labels are cosmetic). Layout v3
  (45 inputs) = **`20bd7c41f6a62220`**. NN01 v2 stores it at [8..15]; `nn_check_genome_layout` (one function) is called by
  `nnextractor`, `nn2cpp` (pathgen-only) and eval mode; `nn2cpp` emits `kGeneratedLayoutSignature` + `static_assert`.
- Flight log v6: header = type/magic/version, then `num_inputs`, `num_outputs`, `layout_version`, `layout_signature`, the slot-name
  table (24 B each), then the v5 body. Both decoders size every record from the header and verify the names hash to the signature;
  both still read v5 (a frozen description). **Decoded columns = lowercased enum names for v5 and v6 alike** — v5 values identical to
  the old decoder on both 10-18 logs; 24 target/dist columns renamed (`target_x_0` → `target_x_tm5`, …). Web decoder also fixed for
  the pitch-first output order (the 043 Python fix never reached it). Tested in node: same values as Python.
- ctest **55/55**; xiao `pio run` SUCCESS with `nn_program_generated.cpp` from t6 g752 restamped as NN01 v2 (layout v3 = its own
  layout; `weight_id` therefore differs from the 043 flight file). ⚠️ **Hardware bench log not taken** (no bench in this session):
  the decoders were proven on the v6 fixture written by the same `flight_log_format.h`; the operator's pre-flight bench (T052)
  confirms on hardware.
- 043 scripts ported to read the input block by name (`nn_replay_probe.input_block`): `rate_history_availability` output identical
  to the pre-port script; `pitch_input_attribution` runs on sim and flight CSVs.
- ⚠️ Consequence: NN01 v1 genomes inside the smoke-A / T026 dmps are refused by this build's tools (by design); analyse smoke A with
  the worktree's binaries (`dmp-dump` now prints `rampSc` NaN, not a silent 1.0, for an unreadable genome).

## § Layout v4 — the format break (T039–T046) — 2026-10-19

- **53 inputs (M2 74)**; layout v4 signature **`a505f1be760da67d`**; weights 2563 (M2 **3303** — 7 outputs; the data model's earlier
  3235 assumed 3, corrected); dmp schema **6**; `kNNHistoryLayoutVersion` 4. History slots are scalar fields beside each current value
  (`gyro_q_tm2`, `gyro_q_tm1`, `gyro_q`), so every existing `common.gyro_q` reference kept meaning "now".
- Recording: `AircraftState::CraftSample` (gyro rad/s, accel z g) in a 3-deep ring; `recordErrorHistory` takes the sample as a REQUIRED
  argument (crrcsim inputdev + xiao msplink); tracker paths call `recordCraftHistory` before their gather. Lagged reads clamp to the
  oldest sample since the reset, so the first tick after a reset reads the current value in every lag (FR-002) — verified in the dmp:
  tick 1 `gyrQ-2 = gyrQ-1 = gyrQ`, tick 2 `gyrQ-1` = tick 1's `gyrQ`.
- **"Just works" check** — files the layout change touched beyond `nn_inputs.h`, `topology.h`, the recording sites and the tests:
  1. `include/autoc/rpc/protocol.h` — the planned schema bump (not an index).
  2. `xiao/include/flight_log_format.h` — ONE enum-range start: the gyro scale loop began at `GYRO_P` and skipped the two P lags.
     Caught on the first build by the `scaleTableComplete` hardening (an unassigned scale = 0); fixed to `GYRO_P_TM2`. A pattern gate
     cannot see a range bound reliably; the completeness check (test + runtime `validateFileHeader`) is the guard for it.
  No other hand-written consumer needed an edit (dmp-dump, renderer, data.dat header, nn2cpp, nnextractor, both log decoders,
  `input_investment.py`, the ported 043 scripts). Tests that pinned positions (`nn_sensor_interface_tests` anchors, header text, count
  pins) moved deliberately; `gather_tracker_inputs_tests` had a literal `flat[59]` → now by enum name.
- Gates: clean `rebuild-perf.sh` (smoke A on worktree binaries); ctest **55/55**; input-layout gate clean (168 files);
  **eval-vs-training bitwise** on a 3-gen 53-input smoke-regiment run (`logs/autoc-044-T046-gate.log`, prefix
  `autoc-9223370244382345856-2026-10-20T05:00:29.951Z`): **`NN_EVAL_SAME: fitness=-5.443911`** = stored — the 53-input baseline
  until the bake supplies a real one. xiao `pio run` SUCCESS with a 53-input generated program (RAM 53.6 %, flash 45.8 %); `nn2cpp`
  refuses the 45-input g752 by name. Both decoders read the 53-input v6 fixture and the v5 10-18 logs. ⚠️ Hardware bench log still
  owed at the pre-flight bench (T052).
