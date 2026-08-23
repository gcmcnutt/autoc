# Tasks: 042 — camera receiver (Stage 1: bench tracker + ASCII scope)

**Input**: Design documents from `/specs/042-camera-receiver/`
**Prerequisites**: [plan.md](plan.md), [spec.md](spec.md), [research.md](research.md), [data-model.md](data-model.md), [contracts/](contracts/), [quickstart.md](quickstart.md)

**Tests**: **INCLUDED and binding.** Not an optional choice here — Constitution I is a gate in
[plan.md](plan.md) §Constitution Check, and R11 makes the scalar reference the permanent oracle for every
future SIMD change. Test tasks precede the implementations they cover.

**Scope**: **Stage 1 only** (plan.md §Stages) — bench tracker on existing hardware, exiting at the ASCII
scope animating at 10 Hz. Stages 2–4 are gated on the 1.56 mm lens, the Pi 5, and the hb1 cubes
respectively, and are deliberately **not** decomposed here; the plan records them as likely separate
features (043 / 044). Decomposing hardware you do not have produces tasks that rot.

## Bench reality at kickoff (2026-08-19) — read this first

**Only ONE code is available at the start.** The bench emitter is flashed as **code B (STARBOARD/green)**
(031 handoff). The **code-A source is a real flight cube the operator is building in parallel** — so plan
the early work single-code and let two-code arrive with the hardware, rather than blocking on it.

| what exists now | note |
|---|---|
| Bench emitter, **code B only**, **120.0 Hz** (measured 119.940) | ~~Trap: `'H'` is volatile~~ — **OBSOLETE 2026-08-20.** The platform is STRICTLY single-rate: `'R'` no longer jumps to 200 Hz, `BOOT_HALF_RATE` / `+define+CHIP200` / `BEACON_SAMPLE_HZ` are deleted, and `acquire_next_rate_q8()` returns nominal unconditionally. The rate cannot leave 120 Hz |
| Pi 3A+ + OV9281, Trixie arm64, mainline `ov9282` + patched 640×200 module | ~250–280 fps sustained; `pi/INSTALL.md` is the rebuild recipe |
| 1.8 mm fisheye, **unfiltered** | f·θ, 95° H × 61° V measured — final. 850 filter + 1.56 mm lens are Stage 2 |
| Pan/tilt on the airframe, INAV-commanded; **O3 removed for these tests** | repeatable slew available; sorties fly blind (spec §7.1.1) |
| `rpicam-raw` to file | the crude recorder to bootstrap clips before T022 exists |
| **Arriving in parallel** | code-A **flight cube** (operator building); 1.56 mm lens; Pi 5 |

**Consequence for sequencing**: US1, US2 and US4 are fully exercisable single-code. In US3, cold
acquisition (T048–T053) is single-code work and should proceed; only **T054, T057 and the near-far case
need the second code**. Do not let them gate the Stage 1 exit demo — a single-code scope animating at
10 Hz is still the exit criterion met, with two-code verification following the cube.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: parallelizable — different files, no dependency on an incomplete task
- **[Story]**: US1–US4 below

---

## Phase 1: Setup

**Purpose**: project skeleton, the three build paths, and the tiered build structure

- [X] T001 Create the source tree skeleton per plan.md §Structure under `firmware/beacon-receiver/`: `src/core/`, `src/io/`, `src/app/`, `tools/`, `tests/{unit,equivalence,golden}/`, `cmake/`
- [X] T002 Declare beacon-receiver targets in the top-level `CMakeLists.txt` (Constitution IV — single source of truth): option `BEACON_RECEIVER`, targets `beacon_core` (C11), `beacon_tools`, `beacon_tests`; drive a clean rebuild, not an incremental reconfigure
- [X] T002a Expose the tier-0 / tier-1 build structure in the top-level `CMakeLists.txt` — option `WITH_EMBEDDED` (default OFF); tier 0 = native host targets, tier 1 = cross/embedded targets that need no hardware (plan.md §Build-tree principle)
- [X] T002b Add an `all-targets` meta-target in the top-level `CMakeLists.txt` that invokes each target build and **propagates failure** — wire the receiver in now; xiao/pod/fpga are retrofitted separately per `specs/BACKLOG.md`
- [X] T003 [P] Write the WSL2 cross toolchain file at `firmware/beacon-receiver/cmake/aarch64-linux-gnu.cmake` (research.md R7)
- [X] T004 [P] Document the three build paths in `firmware/beacon-receiver/README.md`, superseding the 031-era "Next (A8-6)" section

---

## Phase 2: Foundational (BLOCKING — no user story starts until this completes)

**Purpose**: the types, the versioned formats, and the config discipline everything else depends on

- [X] T005 Define `FrameView` and the `FrameSource` vtable in `firmware/beacon-receiver/src/core/frame.h` per data-model.md §1 — `t_us` is **source-supplied**; no `clock_gettime` anywhere in `core/` (R3)
- [X] T006 [P] Define the versioned 20 Hz record struct in `firmware/beacon-receiver/src/core/record.h` per `contracts/record-wire-format.md`, including the dual-clock fields (`inav_t_us`, `inav_read_age_us`, `gps_time_ms`) and the flags bit assignments
- [X] T006a [P] Add `static_assert` layout locks (struct size and field offsets) to `firmware/beacon-receiver/src/core/record.h` — catches an accidental field edit **within this implementation**; the cross-target mechanism is T006b's golden vectors, not a shared header (plan.md §Contracts at arm's length)
- [X] T006b [P] Emit **golden byte vectors** for the record contract into `firmware/beacon-receiver/tests/golden/record_vectors/` — canonical encoded records with known field values, so any future second implementation (xiao, analysis tooling) verifies against them with its own codec and no shared header (plan.md §Contracts at arm's length)
- [X] T007 [P] Define the versioned recording container structs in `firmware/beacon-receiver/src/core/container.h` per `contracts/recording-container.md`
- [X] T008 [P] Test: the config loader errors on **each** missing key, naming it, exiting 1 — `firmware/beacon-receiver/tests/unit/test_config.c` (Constitution VII; written before T010)
- [X] T009 [P] Test: record and container readers fail loudly on version mismatch, naming both versions — `firmware/beacon-receiver/tests/unit/test_versions.c` (Constitution V; written before T011)
- [X] T010 Implement the config loader in `firmware/beacon-receiver/src/core/config.c` + `config.h` per `contracts/config-schema.md` — no in-code defaults; annotate genuine zero-init with `/* default-ok: <reason> */`
- [X] T011 Implement the reader-side version-check helpers in `firmware/beacon-receiver/src/core/record.h` and `container.h` to satisfy T009
- [X] T012 Implement `config_hash` and `build_id` stamping in `firmware/beacon-receiver/src/core/config.c` (spec §16.2)
- [X] T013 Register GoogleTest + CTest for `tests/{unit,equivalence,golden}` in `firmware/beacon-receiver/CMakeLists.txt`
- [X] T014 [P] Implement the kernel micro-bench harness in `firmware/beacon-receiver/tools/bench.c`, reporting ns/px and GMAC/s per kernel (quickstart.md §4)

**Checkpoint**: `beacon_core` + `beacon_tests` build and pass on the dev box with no camera present.

---

## Phase 3: US1 (P1) — Record raw and replay it byte-identically

**Goal**: capture raw on the Pi, replay it on the dev box, get the same frames. Ships the flight recorder
as a side effect and **de-risks libcamera before any algorithm depends on it** (R8, spec §7.1).

**Independent test**: burst-capture 60 s on the Pi → `scp` → replay → container round-trip byte-identical;
a version-mismatched file is rejected with both versions named.

- [X] T015 [P] [US1] Test: container write→read round-trip preserves every field **including `seq` gaps** — `firmware/beacon-receiver/tests/unit/test_container.c`
- [X] T016 [P] [US1] Test: the replay source yields an identical `FrameView` sequence across two runs — `firmware/beacon-receiver/tests/unit/test_replay.c`
- [X] T017 [US1] Implement the container writer in `firmware/beacon-receiver/src/io/sink_record.c` — preallocated file, `O_DIRECT`, large aligned writes (R9)
- [X] T018 [US1] Implement the replay frame source in `firmware/beacon-receiver/src/io/src_replay.c` behind the `FrameSource` vtable
- [X] T019 [US1] Implement the bounded RAM ring with triggered dump in `firmware/beacon-receiver/src/io/sink_record.c` — the degraded-sink mode of spec §8.2, same code path as continuous
- [X] T020 [US1] Implement burst mode (N contiguous frames every M) with `burst_start` flags in `firmware/beacon-receiver/src/io/sink_record.c` — each burst ≥1 full word so it stays replayable (spec §16.4)
- [X] T021 [US1] Implement the libcamera frame source in `firmware/beacon-receiver/src/io/src_libcamera.cc` — Request-based, DMA buffers, per-frame exposure/gain metadata (R8)
- [X] T022 [US1] Implement the `beacon_record` CLI in `firmware/beacon-receiver/src/app/beacon_record.c` per `contracts/cli.md`
- [X] T023 [US1] Apply real-time hygiene in `firmware/beacon-receiver/src/app/beacon_record.c` — `SCHED_FIFO`, core pinning, `mlockall`, no allocation in the loop (R6)
- [X] T024 [US1] Write dual-clock fields as **zero-when-absent** in `firmware/beacon-receiver/src/io/sink_record.c` — no FC on the bench, and a reader must never mistake 0 for a valid INAV time (R10)
- [X] T025 [US1] Bench-verify on the Pi — 60 s burst capture to `/dev/shm`, replay on the dev box, byte-identical round-trip; record the run in `specs/031-beacon-camera/bench-journal.md` (quickstart.md §5)

**Checkpoint**: US1 is independently deliverable — the recorder can fly (spec §7.1) with nothing else built.

---

## Phase 4: US2 (P2) — Track a beacon across the field

**Goal**: the motion-compensated multi-scale correlator bank — the thing 042 exists for.

**Independent test**: a replayed pan/tilt slew clip is tracked through transit and scored against the
offline oracle, with both §3.1 rates reported.

- [X] T026 [P] [US2] Test: scalar `reduce` / `hipass` kernels against hand-computed vectors — `firmware/beacon-receiver/tests/unit/test_kernels.c` (written before T027)
- [X] T027 [P] [US2] Implement scalar reference kernels in `firmware/beacon-receiver/src/core/reduce.c` and `src/core/hipass.c` — separate passes, obviously correct, permanent (R11)
- [X] T028 [US2] Implement the fused per-tile NEON front end in `firmware/beacon-receiver/src/core/reduce.c` and `src/core/hipass.c` — L1-sized tiles, one pass over each (R4)
- [X] T029 [US2] Test: scalar-vs-NEON **bit-exact** equivalence per kernel — `firmware/beacon-receiver/tests/equivalence/test_kernels_neon.c` (R11)
- [X] T030 [US2] Run `tools/bench` on the Pi and record the measured GMAC/s against R5's 1–2 GMAC/s assumption in `specs/042-camera-receiver/research.md`
- [X] T031 [P] [US2] Implement Gold-31 template generation and frames→chips reduction in `firmware/beacon-receiver/src/core/corr.c`, fixed-point per R2
- [X] T032 [US2] Implement `corr_search()` (31 phases × 2 codes) and `corr_track()` (known phase/code) as **separate** functions in `firmware/beacon-receiver/src/core/corr.c` — they differ by ~60× and the cheap one is the common path
- [X] T033 [P] [US2] Test: `corr_track` reproduces `corr_search`'s answer at a known phase — `firmware/beacon-receiver/tests/unit/test_corr.c`
- [X] T034 [US2] Implement the `Track` struct and the alpha-beta centering loop in `firmware/beacon-receiver/src/core/track.c` per data-model.md §2
- [X] T035 [US2] Implement the multi-scale ladder (coarse / medium / fine) with q-driven scale selection in `firmware/beacon-receiver/src/core/track.c` (spec §2.2)
- [X] T036 [US2] Implement the DPLL (chip-rate and phase pull-in) in `firmware/beacon-receiver/src/core/track.c`
- [X] T037 [US2] Implement decision-directed chip-level `lock_health` in `firmware/beacon-receiver/src/core/track.c` — scale-free and field-position independent, normalised against the local noise floor (spec §2.6)
- [X] T038 [US2] Implement SNR-driven adaptive integration length in `firmware/beacon-receiver/src/core/agc.c` (spec §4)
- [X] T039 [US2] Implement the Bank — 16 slots, candidate lifecycle, guard+precision pairing, eviction priority — in `firmware/beacon-receiver/src/core/bank.c` per data-model.md §3
- [X] T040 [US2] Implement the **evidence-bounded** HOLD state in `firmware/beacon-receiver/src/core/bank.c`, reading the same `hold_max_*` keys the scorer uses so metric and state machine cannot drift (data-model.md §2)
- [X] T041 [P] [US2] Test: track lifecycle transitions including evidence-bounded HOLD exit — `firmware/beacon-receiver/tests/unit/test_bank.c`
- [X] T042 [US2] Implement acquisition-budget virtualisation in `firmware/beacon-receiver/src/core/sched.c` so replay completes acquires at the frame index live would have (R3)
- [X] T043 [US2] Test: two replay runs of one clip emit byte-identical record streams — `firmware/beacon-receiver/tests/golden/test_replay_parity.c` (quickstart.md §6)
- [ ] T044 [P] [US2] Implement the non-causal offline oracle in `firmware/beacon-receiver/tools/oracle.c`
- [ ] T045 [P] [US2] Implement `firmware/beacon-receiver/tools/inject.c` — coded point source on a known trajectory into real recorded background (spec §7)
- [ ] T046 [US2] Implement `firmware/beacon-receiver/tools/score.c` — one CSV row per envelope cell carrying §3.1's two rates, §3.2's three columns, and §11.1's deadline-miss rate
- [ ] T047 [US2] Bench-verify — a pan/tilt slew clip tracked through transit and scored against the oracle; envelope cell to `specs/042-camera-receiver/results/stage1-slew.csv`, summary in `specs/031-beacon-camera/bench-journal.md`

**Checkpoint**: the tracker works on recorded data. Nothing yet renders it.

---

## Phase 5: US3 (P3) — Acquire cold, and tell the two codes apart

**Goal**: find a beacon with no prior, at slew, and keep A and B distinct.

**Independent test**: cold acquisition on a clip with no seed; false-acquire rate over cluttered
background. **Two-code verification (T057) is deferred until the code-A flight cube lands** — see
§Bench reality; it does not gate the Stage 1 exit.

**Single-code first**: T048–T053, T055, T056 and T058 are all exercisable with the bench code-B emitter
alone. The mirror-pair rule (T055) is written and unit-tested against injected pairs (T045) rather than
waiting for real hardware.

- [ ] T048 [US3] Prototype acquisition in NumPy against recorded clips in `firmware/beacon-receiver/pi/acquire_proto.py` — **research spike, ships no tests** (R12; Constitution I exemption invoked narrowly, nothing promoted)
- [X] T049 [P] [US3] Implement blink detection (frame-to-frame temporal difference) in `firmware/beacon-receiver/src/core/acquire.c`
- [ ] T050 [US3] ~~Implement linear-motion RANSAC proto-track formation~~ → **RE-CUT 2026-08-22 to
  track-before-detect (TBD)**, in `firmware/beacon-receiver/src/core/acquire.c`. **RANSAC is the wrong
  model and the operator caught it**: it fits a straight line over a whole word, but a word is 258 ms and at
  a 500 °/s roll that sweeps 129° — cross-track plus roll with both platforms moving is an arc, and RANSAC
  would reject the target as an outlier from its own trajectory. Instead propagate hypotheses frame-by-frame
  through an **acceleration-bounded reachable cone**, accumulating code correlation along each path, with no
  global motion model. Operator's framing: *"mostly little triangle guesses where last known point is a
  vertex."*
  **Why it is affordable** (`compute-budget.md` arithmetic): with a velocity state the per-frame cone is set
  by ACCELERATION, not velocity, and is **sub-pixel even at 10 000 °/s²** (0.198 M2 px per 3.47 ms frame).
  Propagation is therefore nearly free — a beam of 100 k particles costs **0.26 G ops/s** against 15.2
  available, and 1 M costs 2.59. Compare the unpruned position×velocity lattice at 104–431 G ops/s, or the
  brute-force sweep at 5.5 s per attempt.
  **The actual design work is INITIALISATION**, not propagation: all the cost is in not-yet-knowing velocity.
  Seeding density, pruning rule and score gate are the parameters that matter.
- [ ] T051 [US3] Implement multi-rate decode-along-track in `firmware/beacon-receiver/src/core/acquire.c` — rate-agnostic, because the bench emitter's `'H'` mode is volatile (031 trap #2)
- [ ] T052 [P] [US3] Test: acquisition finds a known injected track at a known SNR — `firmware/beacon-receiver/tests/unit/test_acquire.c`
- [ ] T053 [US3] Thread acquisition off the capture path in `firmware/beacon-receiver/src/app/beacon_trackd.cc`, charged through the T042 budget model
> **RE-ORDERED 2026-08-22 after the pan2 measurement + compute budget.** T050/T051 are no longer "acquisition
> polish" — they are the critical path, and a new prerequisite (T071) sits in front of them. Evidence:
> [results/stage1-pan2-analysis.md](results/stage1-pan2-analysis.md) (lock collapses at 1–2 °/s; hand motion
> is 15–18 °/s; 22 s of continuous slow motion with **zero** reacquisitions) and
> [compute-budget.md](compute-budget.md) (brute-force velocity search is 14× over budget at bench rates and
> ~15 000× at flight rates; ego-motion registration removes the V² term and is worth ~75×, where NEON is
> worth ~4×). Do them in this order — each is independently testable against `pan2.bcnr`.

- [ ] T081 [US3] **SYNC-FIRST ACQUISITION** (operator, 2026-08-22) — see
  [sync-first-acquisition.md](sync-first-acquisition.md). Split the synced and unsynced regimes and make
  chip phase **receiver-global** rather than per-track (one emitter, one clock). Then:
  **synced** = full-field phase-known matched filter at **0.39 ms**, run EVERY tick, broad + narrow
  continuously; **unsynced** = one 24 ms full 31-phase × 2-code search, off-thread.
  Cold acquisition becomes **258 ms window + 24 ms search = 283 ms, deterministic**, against a 400 ms bar
  and today's measured 2.10–8.30 s lottery. Phase survives ~**65 s** blind (single-rate platform, pod 64 ppm
  off nominal), so a beacon leaving and re-entering the field costs ONE 0.39 ms correlation.
  **Evidence it works: `tools/oracle.py` is already this detector** and finds the beacon at q_rel ~5000 in
  the very scenes where the blink detector ranks it 126th–230th.
  ⚠️ Measure first: memory bandwidth (7.9 MB of bins touched per frame — `hipass M2` at 0.672 G op/s is the
  warning that this is memory-bound, not MAC-bound) and the full-field false-alarm rate (makes T058
  load-bearing). Does NOT fix the 1–2 °/s coherence limit; T050 still owns motion.
  **Largely supersedes T080** — ranking only matters while seeding is a lottery.
- [ ] T080 [US3] **Re-rank the acquisition detector: accumulated |Δ²| over ~8 frames** — the cheapest large
  win found so far, and it should be done BEFORE T071/T050 because it is ~30 lines and changes what those
  are measured against. Replace `acquire_pass`'s single inter-pass |Δ| on the reduce4 plane with a running
  sum of the temporal SECOND difference over 8 frames (3.3 chips). Measured offline on `pan2.bcnr`
  ([results/stage1-detector-ranking.md](results/stage1-detector-ranking.md)): beacon median rank **1842 → 1**
  still, **276 → 2** moving; top-3 seeding ~19–25 % → ~60 %, and the moving case becomes as good as the
  still case. Works because a second difference annihilates the linear ramp an edge makes as it sweeps a
  cell, which is the dominant ego-motion artefact. Window has a real optimum (flat 4–16 frames, collapses by
  72) set by the same smear limit as everything else. **Also raise `max_seeds`/candidate cap** (3 seeds, 4
  candidates today) — top-32 reaches ~71 %.
  ⚠️ **Scope**: this fixes SEEDING, not CONFIRMATION. A seeded candidate still needs a coherent word, and the
  code match still degrades 5–20× under motion — so expect a large gain static and only partial gain moving.
  It does **not** substitute for T071/T050.
- [ ] T071 [US3] **Ego-motion registration** — per-frame global shift estimate on a reduce4 plane, in
  `firmware/beacon-receiver/src/core/`. THE enabling piece: it turns the moving-camera case back into the
  static-camera case for detection, and removes the V² term from the velocity search (compute-budget.md).
  Independently testable: registered consecutive frames of `pan2.bcnr` must differ far less than unregistered
  ones over the 10–41 s motion segments.
- [ ] T072 [US3] **Measure K, the candidate-detection count per frame** — before and after T071, on
  `pan2.bcnr`. Cheapest high-value experiment available: the RANSAC stage is O(K²) and needs K ≲ 1000–2000,
  but K has never been measured because `acquire_pass` caps its return at 3 seeds by policy. This decides
  whether T050 is viable as designed.
- [ ] T073 [P] [US3] **NEON the correlator MAC** — `mac_i16` measures 5.07 G MAC/s SCALAR while the reduce
  kernels gain ×9.4 from NEON, so ~4× is sitting uncollected in the hot path. Cheap, no architecture, and it
  multiplies every budget in compute-budget.md.
- [ ] T074 [US3] **Re-measure cold-acquisition latency at the 53 µs bench exposure.** The ~8 s figure in the
  pan2 analysis was taken at the fiducial exposure (1499 µs) where the blink detector sees a very different
  scene — it is flagged, not banked.
- [ ] T075 [P] [US2] **Lens calibration pass** (`targets/checkerboard_8x11sq_20mm_inner7x10.png`, one static
  capture). Every °/s in this feature assumes a uniform 0.304 °/M2 px across a 97.3° ultra-wide field; that
  degrades toward the edge, so off-axis rates are not strictly comparable to on-axis ones. Owed before the
  envelope publishes absolute °/s (spec §3.2).
- [ ] T077 [US3] **Moving-target fixture — pendulum** (operator idea, 2026-08-22). Hang the emitter and
  swing it: a known analytic trajectory with REAL curvature, which is exactly what distinguishes TBD from a
  linear model, for the price of a piece of string. Sizing: a **1.0 m pendulum at 3 m range gives a 2.0 s
  period, peak 20.9 °/s and peak 65 °/s²** — dead centre of the band where the tracker currently scores
  **3 % locked** (10–20 °/s), and the acceleration is trivially inside the TBD cone.
  **One 60 s clip is ~30 swings = ~60 continuous sweeps of the whole rate axis**, which fixes the ragged
  binning that made `pan1`/`pan2` weak (a single sample in the decisive 0.5–3 °/s bin). Vary the RELEASE
  ANGLE across clips to move the peak — a sinusoid lingers near peak velocity and passes through zero
  quickly, so the low-rate bins need the small-amplitude clips. Camera STATIC for this, so it isolates
  target motion with no registration confound.
- [ ] T078 [US3] **Moving-target fixture — constant-speed loop** (operator: *"a racetrack… some sort of
  target track built in a large room back in the '70s"*). Complements T077 rather than replacing it: a
  pendulum's speed distribution is arcsine-shaped and never holds a rate, whereas a loop gives **sustained
  constant velocity on the straights and sustained constant curvature on the bends** — which separates
  "constant-velocity tracking" from "constant-acceleration tracking" cleanly. Cheap version: model-railway
  loop with the emitter on a flatcar. Do this only if T077 leaves the velocity/acceleration behaviours
  entangled.

- [ ] T079 [US3] **Field phase — preprogrammed quad flights** (operator idea, 2026-08-22): put the emitter
  and/or the receiver on large quads flying **waypoint missions**, for *"honest and somewhat repeatable"*
  field tests. Payload is not a problem — §7.1 already decoupled the verification airframe from the flight
  article, and the Pi 5 + camera + NVMe is ~100–130 g.
  **This is the stage that settles the one question the bench cannot**: it is the only way to get a **sky
  background**, and sky is the case where visual ego-motion registration (T071) has no texture to work with —
  precisely the regime where `compute-budget.md` shows brute force is four orders out, and therefore the
  test that decides whether §2.3's AHRS is optional feed-forward or a flight-article requirement.
  **Truth comes from the flown logs, never from the commanded path** — wind, GPS drift and timing offsets
  mean "repeatable" is approximate. And know what that truth is worth:
  | reference | 30 m | 100 m | vs the 0.3° RMS bearing bar |
  |---|---|---|---|
  | GPS, typical (3 m) | 5.73° | 1.72° | **6–19× too coarse** |
  | GPS, good (1 m) | 1.91° | 0.57° | 2–6× too coarse |
  | **RTK (2 cm)** | **0.038°** | **0.011°** | 8–26× margin |
  So plain GPS is fine for **lock duty, reacquire distribution and the rate axis** — and rate truth is better
  than position truth, since differencing removes common-mode error — but **the ≤1 px / 0.3° bearing-residual
  bar cannot be validated in the field without RTK.** That bar stays a bench measurement (fiducials give
  0.18°). Decide RTK on whether field bearing accuracy is worth the kit.
  Mission rates are **rated at 288 fps (spec §2.5.1)** — 10 m/s at 30 m is 19.1 °/s = 16× the
  acquisition knee and 0.2× the tracking limit, the right regime here; a 480-fps-designed profile would need
  ×0.600 before flying it against this chain. Useful missions, since apparent rate is set by v/R: a **constant-rate crossing** at fixed range (10 m/s at
  30 m = 19 °/s, at 100 m = 5.7 °/s — the whole band is reachable by choosing range), a **radial
  approach/recede** (≈0 °/s, isolates range/SNR from motion), and a **circle around the chase** (sustained
  constant rate, the racetrack at scale). Both circling = the real engagement.
  ⚠️ Has procurement/logistics lead time (airframes, mounts, possibly RTK) — start it early even though it
  runs late.

### The research ladder (operator, 2026-08-22)

Each stage isolates ONE new thing; do not skip, because a failure two stages up is uninterpretable.

| stage | isolates | fixture | status |
|---|---|---|---|
| 1. static target, moving camera | ego-motion is the *entire* apparent motion, residual ≈ 0 | pan/tilt mount | `pan2.bcnr` is the handheld version — **done, analysed** |
| 2. + camera vibration/ego disturbance | robustness of registration (T071) | same rig + disturbance | |
| 3. **moving target, static camera** | TBD with **no registration confound** | **T077 pendulum** | the cheap decisive one |
| 4. both moving | the real engagement | both rigs | |
| **5. field, preprogrammed quads** | **sky background** + real range/SNR; settles the AHRS question | T079 | long lead time |

- [ ] T076 [P] [US2] **Two `track.c` correctness fixes** (bugs, not tuning — do them opportunistically, do
  NOT schedule a tuning phase; the measured knee is 1–2 °/s and these move it to maybe 2–3, against 15–18 °/s
  of hand motion): (a) `hold_max_age_ms = 150` kills a track before the 258 ms needed to rebuild a
  `integration_min_chips = 31` window, so HOLD is arithmetically incapable of recovering; (b) widening the
  aperture `memset`s the bins, discarding evidence that is a strict subset of the wider aperture.

- [ ] T054 [P] [US3] Code-A source — **the operator is building a real flight cube in parallel** (031 A7); track its arrival in `specs/031-beacon-camera/bench-journal.md` and re-baseline the regression when it lands. *Fallback only if the cube slips past US3: a breadboard ATtiny412 + 2 LEDs is adequate at 2–5 m — but do not build it pre-emptively*
- [X] T055 [US3] Implement the same-code mirror-pair rule in `firmware/beacon-receiver/src/core/bank.c` — flag the lower `MULTIPATH_SUSPECT`, **keep it, do not delete** (spec §9)
- [X] T056 [P] [US3] Implement `extent` (q_fine/q_coarse) and `scintillation` outputs in `firmware/beacon-receiver/src/core/track.c` (spec §9 — free from the ladder, painful to retrofit)
- [ ] T057 [US3] Bench-verify — two codes tracked simultaneously, zero ID swaps, near-far case; results to `specs/042-camera-receiver/results/stage1-twocode.csv`. **GATED on the code-A flight cube (T054)** — does not block the Stage 1 exit
- [ ] T058 [US3] Bench-verify — false-acquire rate over a cluttered room with the rig slewing; results to `specs/042-camera-receiver/results/stage1-falsealarm.csv` (spec §3)

---

## Phase 6: US4 (P4) — See it working at 10 Hz  ← **Stage 1 exit criterion**

**Goal**: the visible end-to-end proof. Capture, correlation, tracking, the record and a transport, all at
once, on real beacons.

**Independent test**: the ASCII scope animates at 10 Hz over SSH against a live bench run.

- [X] T059 [P] [US4] Implement the binary record emitter in `firmware/beacon-receiver/src/io/emit_record.c` — versioned struct, transport-agnostic (R13)
- [X] T060 [P] [US4] Implement the JSON projection in `firmware/beacon-receiver/src/io/emit_json.c`, preserving A = PORT/red, B = STARBOARD/green
- [X] T061 [US4] Implement the `tcp:` and `serial:` sinks in `firmware/beacon-receiver/src/io/emit_record.c` for bench USB/WiFi streaming (`contracts/cli.md`)
- [X] T062 [US4] Implement deadline instrumentation (`deadline_margin_us`, miss-rate accounting) in `firmware/beacon-receiver/src/app/beacon_trackd.cc` (spec §11.1) — track p99 and max, not mean
- [X] T063 [US4] Implement the `beacon_trackd` CLI in `firmware/beacon-receiver/src/app/beacon_trackd.cc` per `contracts/cli.md`
- [X] T064 [P] [US4] Implement the 10 Hz ASCII scope in `firmware/beacon-receiver/tools/ascii_scope.py`, consuming any sink, no X required
- [X] T065 [US4] **STAGE 1 EXIT**: bench run with real beacons — ASCII scope animates at 10 Hz, deadline-miss rate reported, recorded to `specs/031-beacon-camera/bench-journal.md`

---

## Phase 7: Polish & cross-cutting

- [ ] T066 [P] Capture the first golden vector from `beacon_track.py` + oracle, then re-baseline it from `beacon_trackd`, in `firmware/beacon-receiver/tests/golden/` (R11 bootstrap)
- [ ] T067 Retire `firmware/beacon-receiver/pi/beacon_track.py` once golden vectors pass — **no fallback path survives** (Constitution III)
- [ ] T068 [P] Add a tracker regression entry point alongside `firmware/beacon-decoder-stepfpga/host/regression.py` per the CLAUDE.md policy
- [ ] T069 [P] Verify the WSL2 cross build produces a working `beacon_trackd` on the Pi and record the procedure in `firmware/beacon-receiver/README.md` (R7, quickstart.md §3)
- [ ] T070 [P] Update `firmware/beacon-receiver/README.md` and `specs/031-beacon-camera/bench-journal.md` with Stage 1 results

---

## Dependencies

```
Setup (T001–T004)
   └─> Foundational (T005–T014)   ← BLOCKS everything
          ├─> US1 (T015–T025)     ← independently deliverable; ships the flight recorder
          │      └─> US2 (T026–T047)   needs replay (T018) + container (T017)
          │             ├─> US3 (T048–T058)   needs corr + bank + sched
          │             └─> US4 (T059–T065)   needs Track output
          └─> Polish (T066–T070)  after US4
```

**Story order is a real dependency chain here, not a preference**: US2 has nothing to run against without
US1's replay path, and US4 has nothing to render without US2. US1 alone is still independently valuable.

**Within-story ordering that matters**: T027 (scalar) strictly precedes T028 (NEON), which strictly
precedes T029 (equivalence) — that is R11 and it is the whole verification strategy. T042 (scheduler
virtualisation) must land before T043 (parity), or parity will fail for a reason that looks like a
correlator bug.

## Parallel execution examples

- **Foundational**: T006, T007, T008, T009, T014 all run together (distinct files).
- **US1**: T015 and T016 (tests) together; then T017/T018 together; T021 is the long pole — start it early since libcamera bring-up is the schedule risk (R8).
- **US2**: T031, T033, T044, T045 are independent of the track-loop chain (T034→T040) and can run alongside it.
- **US3**: T054 (solder the interim emitter) is pure hardware and parallel to every software task in the phase — start it first, it has lead time.
- **US4**: T059, T060, T064 together.

## Implementation strategy

**MVP = US1.** The recorder is deliverable on its own, ships the flight recorder called for by spec §7.1,
and de-risks the single biggest unknown (libcamera bring-up on Trixie) before anything depends on it. If
libcamera fights back, that is discovered in week one with a fallback still available (R8).

**Stage 1 exit = US1 + US2 + US3 + US4, single-code.** Two-code verification (T057) follows the flight cube and is not part of the exit. An intermediate demo is possible at US1 + US2 + US4 by seeding a
candidate manually — worth doing to get the scope animating early, but it is not the exit criterion,
because "you have to tell it where to look" is precisely what US3 removes.

**Not decomposed here**: Stage 2 (optics/AGC/range, gated on the 1.56 mm lens), Stage 3 (flight speeds,
gated on the Pi 5), Stage 4 (ground track then airborne, gated on the hb1 cubes). See plan.md §Stages.
