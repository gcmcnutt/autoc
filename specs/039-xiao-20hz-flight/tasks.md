# Tasks: 039 Xiao 20 Hz Flight — embedded control-loop catch-up

**Input**: Design documents from `/specs/039-xiao-20hz-flight/`
**Prerequisites**: plan.md, spec.md (5 stories + clarifications), research.md (D1–D7),
data-model.md, contracts/ (flight-log-format, latency-memo, bench-validation), quickstart.md

**Tests**: included — Constitution I (testing-first) + the contracts explicitly demand
failing-first tests for the codegen equivalence, arena rule, and log round-trip.

**Sequencing note (operator 2026-07-10)**: "implement first and then study actual latency" — so
US3/US4 (log + 20 Hz) run BEFORE the US2 memo/review, even though US2 is P1. US5 is the capstone.
Hardware-in-the-loop tasks (bench, flight) are operator-driven; desktop tasks are assistant-executable.

**Status 2026-07-11 (end of implement session)**: T001–T019 + T029 DONE — firmware flashed and
bench-validated (T010 FR-002 PASS, first hardware span: 111 ticks @ 20 Hz, 0 overruns/drops,
outcome.md + autoc-039-t1-bench-latency.png). Remaining: T017/T020 soak (one combined bench
session), T021 baud (optional lever, can defer past the first flight), T022 memo (assistant, needs
soak numbers), T023 decision (flying the default t5 candidate = the "model stands" branch),
T025–T026 flight (planned 2026-07-12), T027/T028 report+verdict (assistant, post-flight), T030 wrap.

## Phase 1: Setup

- [X] T001 Extract the pinned t5 elite (`autoc-m1/autoc-9223370253553029228-2026-07-06T01:35:46.579Z/`, gen 800) via `build/nnextractor` to `nn_weights.dat`; verify reported topology 37→32→16r→3 / 2051 weights — operator pre-extracted; topology VERIFIED 37→32→16r→3 / 2051 w (run id corrected from the 07-02 draft value, operator-confirmed 2026-07-11)
- [X] T002 [P] Baseline gate: confirm the UNMODIFIED xiao tree compiles — `cd xiao && pio run -e xiaoblesense_arduinocore_mbed` (Constitution II reference point before any 039 edits) — RESULT: FAILS at `src/generated/nn_program_generated.cpp:284` (029-vintage generated file vs 038 `gather_pathgen_inputs(…, const FlightArena&, …)` signature); this is the known catch-up gap US1/T008-T009 closes; recorded as the baseline reference point

## Phase 2: Foundational — nn2cpp unrolled-recurrent (blocks US1 regen; benefits US4 budget)

- [X] T003 Write FAILING test `tests/nn2cpp_unroll_tests.cc`: generate unrolled + table-driven code from the same recurrent genome (16r layer), assert per-tick outputs identical across a multi-tick sequence incl. `nn_reset()` (same accumulation order ⇒ bit-comparable on desktop); register target in `CMakeLists.txt` — RED confirmed at the fallback marker, then green after T004; test drives the real nn2cpp binary + two g++ harness compiles (24 ticks, reset at t=12, bit-exact compare)
- [X] T004 Implement unrolled-recurrent emission in `tools/nn2cpp.cc` (D6): persistent static `h[]` state registers, straight-line `W_xh·x + W_hh·h + b` MACs, double-buffered h commit (no read-after-write), `nn_reset()` zeroing (as `generatedNNProgramReset()`), remove the `-u`+recurrent fallback; T003 green. ALSO (039 D5 + log provenance): both emissions now gather via extern `nnActiveArena()` (consumer-owned, link-fails if undefined) + expose `generatedNNProgramArenaTemplate()`; identity trailer baked (weight_id/firmware_id SHA-256 prefixes, topology string/counts) for the boot banner + FileHeader
- [X] T005 Clean `rebuild-perf.sh` gate (OPERATOR — CMakeLists changed, Constitution IV) + full test suite green — operator confirmed clean rebuild + pio clean build 2026-07-11

**Checkpoint**: `nn2cpp -u` emits working unrolled code for the t5 elite.

## Phase 3: User Story 1 — Firmware catch-up to the 038 contract (P1) 🎯 MVP

**Goal**: xiao runs the 37-in/2051-w elite with engage-centered arena; observational bench span passes.
**Independent Test**: stationary bench span; logs show the path moving around the craft, all 37 inputs + 3 outputs plausible, arena inputs at band center with the resolved ±47.5 m band in the EngageHeader (contracts/bench-validation.md FR-002).

- [X] T006 [P] [US1] Write FAILING test `tests/arena_recenter_tests.cc`: engage-centered pure ±K rule `ceiling_Z = z_engage − K`, `floor_Z = z_engage + K`, K = 47.5 m (NED, per `docs/COORDINATE_CONVENTIONS.md`, no min-elevation clamp per 2026-07-10 simplification); cases: bench z_e≈0 → band ±47.5 centered on bench; high engage z_e=−80 → floor −32.5 / ceiling −127.5; horizontal re-center at engage x/y — RED (compile: resolver absent), 7 tests green after T007; also covers virtual-frame band-centering + distanceToBoundary symmetry + egress kinds
- [X] T007 [US1] Implement the engage-scoped arena (D5): `EngageArena` + inline `resolveEngageArena(geometry, engage_pos)` in `include/autoc/eval/arena.h` (xiao-safe), invoked in the span-activation path in `xiao/src/msplink.cpp` BEFORE `rabbit_active`; `nnActiveArena()` defined in msplink returns the span's resolved virtual-frame arena; origin + resolved floor/ceiling exposed for logging; NO in-class defaults (Constitution VII); T006 green
- [X] T008 [US1] Regenerate `xiao/src/generated/nn_program_generated.cpp`: `build/nn2cpp -i nn_weights.dat -u -a 80,5,100 -o …` (arena literal = TEMPLATE only per D5); emitted header comment VERIFIED: "(unrolled)" + `37 -> 32 -> 16r -> 3` / 2051 (file is gitignored — regenerate per checkout)
- [X] T009 [US1] Update xiao call sites for the 038 gather signature (arena via `nnActiveArena()`) in `xiao/src/msplink.cpp` + `xiao/include/nn_program.h` (new decls: reset/template/ids); `generatedNNProgramReset()` now called at engage (fixes recurrent-state leak across spans); INTERIM text NN log line extended with `dBnd` + `inw[3]` + an Engage line with arena origin/floors/K; boot banner reports topology + weight_id/firmware_id + arena template; `src/eval/arena.cc` added to platformio build_src_filter (both envs — distanceToBoundary/inwardBodyDirection now linked); `pio run` GREEN (RAM 53.1%, Flash 44.4%)
- [X] T010 [US1] OPERATOR bench: FR-002 observational span per `contracts/bench-validation.md` §FR-002 (flash → ERASE:ALL → engage → download → checklist review incl. center-of-arena readings + resolved ±K band in the EngageHeader); record the reviewed log reference in `specs/039-xiao-20hz-flight/outcome.md` — PASS 2026-07-11: eval-results/bench-20260712/flight_log_2026-07-12T01-17-40_flight_001.bin (111 ticks, 0 gaps/drops/overruns; center-of-arena readings confirmed incl. on-axis inward=(0,0,0); full checklist in outcome.md)

**Checkpoint**: US1 delivers a flyable-at-10 Hz 038-contract firmware (MVP: the candidate runs on hardware).

## Phase 4: User Story 3 — Flight logging that sustains 20 Hz (P2, pulled before US2 per sequencing note)

**Goal**: versioned int16-packed binary log carrying all 37+3+aux; console = events + 2 Hz heartbeat.
**Independent Test**: sustained bench logging session decodes losslessly, fits the 2-flight budget, zero tick interference (contracts/flight-log-format.md tests + bench).

- [X] T011 [P] [US3] Write FAILING tests `tests/flightlog_roundtrip_tests.cc` per contracts/flight-log-format.md §Tests: encode→decode field-for-field ≤ quantization step; version+1 and corrupted scale-CRC loud-fail; saturation (no wrap); static assert TickRecord ≤ 100 B; register in `CMakeLists.txt` — RED first (header absent), then GREEN: 6 tests incl. stream-walk w/ padding skip + unknown-type/truncation loud-fail; registered in CMakeLists + run_autoc_tests
- [X] T012 [US3] Define the format as ONE shared self-contained header `xiao/include/flight_log_format.h` (FileHeader/EngageHeader/TickRecord/EventRecord structs, per-field scale tables, `format_version`, encode/decode inline fns — compilable by BOTH the xiao build and the desktop test target; wire-format fields `// raw-ok: hardware byte layout`); T011 compiles against it and goes green — DONE: self-contained packed structs (FileHeader 188 B w/ header-carried CRC-guarded scale table, EngageHeader 29 B, TickRecord 96 B, EventRecord 10 B, SpanSummary 103 B), symmetric-saturating i16 encode, StreamWalker; compiles xiao + desktop
- [X] T013 [US3] Implement the xiao write path: EngageHeader at span activation (origin + RESOLVED floors from T007), TickRecord per tick (post-gather input values — honest recording), span-summary EventRecord at disengage carrying `loopStats` (ticks/overruns/resyncs/maxLate/avgLate) + MSP pipeline stats, in `xiao/src/flash_logger.cpp` + `xiao/src/msplink.cpp`; drop/coalesce counter on buffer pressure (FR-008) — DONE: flashLoggerWriteBinary (whole-record append, drop-not-stall), raw BLE download (zero-strip filter REMOVED — would corrupt binary), .bin naming; flight_log.cpp writer (FileHeader at arm w/ baked ids, EngageHeader w/ resolved arena at engage, per-tick honest TickRecord, SpanSummary w/ loopStats+pipeline+DWT at disengage, kEventLogDrop accounting)
- [X] T014 [US3] Console split (FR-014) in `xiao/src/msplink.cpp` (+ logPrint call sites): DELETE the per-tick `NN:`/`Nav State:` text lines (incl. T009's interim extension — Constitution III, no parallel writers), keep events (arm/disarm/engage/disengage/errors) + ~2 Hz heartbeat — DONE: logPrint console-only (flash text path deleted incl. T009 interim NN line + Nav State line — Constitution III), ~2 Hz hb console line, events kept; span-end summaries stay (event-class)
- [X] T015 [P] [US3] Desktop decoder `src/analytics/flightlog_decode.py`: reads `.bin` → CSV/dataframe; loud-fail on version/CRC; reports tick_counter gaps + drop counts; unit-check against a bench-produced file — DONE: header-scale CSV decode, loud-fail verified (version+1 / CRC / unknown-type / truncation all exit 1), gap+drop reporting; unit-checked against a C++-generated .bin from the same format header (bench file check remains in T017)
- [X] T016 [P] [US3] Update `xiao/web/flight_logger.html`: download the `.bin` intact (rename/size handling), plus in-browser decode or CSV export implementing the same contract (no third format definition) — DONE: .bin saved intact (named flight_log_<ts>_flight_NNN.bin), in-browser decoder + tick-CSV export implementing the contract (validated under node against the synthetic .bin), decode summary panel
- [ ] T017 [US3] OPERATOR bench: sustained logging session — budget math holds (~95 B/tick ⇒ ~0.9 MB for 2×4 min), BLE download, `flightlog_decode.py` lossless, zero logging-induced overruns in the span summary

**Checkpoint**: complete honest flight record at 20 Hz volume, console quiet.

## Phase 5: User Story 4 — 20 Hz control loop on hardware (P2)

**Goal**: NN every tick at 20 Hz, cadence held with logging active; eval cost measured.
**Independent Test**: FR-011 soak — several consecutive 3–4 min spans, zero overruns, stats captured (contracts/bench-validation.md §FR-011).

- [X] T018 [US4] Set `MSP_NN_EVAL_DIVISOR = 1` in `xiao/include/main.h` (50 ms loop already 20 Hz); confirm re-entry guard behavior at every-tick eval in `xiao/src/msplink.cpp:43-44` — DONE: divisor=1; re-entry guard reasoning documented in main.h (single-threaded loop ⇒ no MSP overlap; over-budget ticks surface as loopStats overruns)
- [X] T019 [P] [US4] DWT cycle-count measurement of the unrolled eval on target (037 `eval-cycle-harness` design: `DWT->CYCCNT`), one number per firmware image, written into the span-summary/boot log — DONE (code): DWT cycle counter enabled at msplinkSetup, first eval after boot measured (gather+forward), reported on console + carried in every SpanSummary (dwt_eval_cycles); the NUMBER lands with the operator bench (T020)
- [ ] T020 [US4] OPERATOR bench: FR-011 cadence soak per `contracts/bench-validation.md` — several consecutive 3–4 min engaged spans at 115200 baud; loopStats + fetch/eval/send captured in the log
- [ ] T021 [US4] OPERATOR bench: baud-raise experiment — repeat one soak at 460800 (`xiao/src/msplink.cpp:342` + INAV side); record both baud's pipeline stats for the memo; keep whichever the operator picks after T023 (D7: latency lever, not bandwidth)

**Checkpoint**: 20 Hz holding on hardware; all memo inputs measured.

## Phase 6: User Story 2 — Latency ground truth → plan of action (P1, gated on US1/US3/US4 by design)

**Goal**: the decision memo + operator review; flight candidate locked.
**Independent Test**: memo exists with measured/modeled/gap + recommendation; operator decision recorded (contracts/latency-memo.md).

- [ ] T022 [US2] Assemble the latency memo per `contracts/latency-memo.md` into `specs/039-xiao-20hz-flight/latency-memo-results.md`: 10 Hz-era numbers (research.md R1), 20 Hz bench numbers at both bauds (T020/T021), DWT eval cost (T019), modeled values + mechanism (COMPUTE_LATENCY 30 ms, servo v2), tail-vs-50 ms-tick analysis, recommendation (stands / amend+retrain / amend-without-retrain); NO RC-filter modeling (verified transparent)
- [ ] T023 [US2] OPERATOR review: record the decision + FR-006 local-IMU verdict (research position: stays deferred) + baud choice in the memo results + outcome.md — this LOCKS the flight candidate
- [ ] T024 [US2] CONDITIONAL (only if T023 = amend+retrain): update `COMPUTE_LATENCY_MSEC_DEFAULT` in `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.h`, clean `rebuild-perf.sh` (OPERATOR), retrain M1 via `scripts/train.sh` (OPERATOR, Constitution IX), pin the elite (`retain=keep`) + record in outcome.md, regenerate firmware (repeat T008–T010 on the new weights)

**Checkpoint**: flight candidate + configuration frozen.

## Phase 7: User Story 5 — The M1 flight test (P1 capstone; gated on US1–US4 + T023)

**Goal**: fly, capture, and judge sim-grade smoothing (SC-005/SC-006).
**Independent Test**: completed flight with full 20 Hz logs + the per-axis comparison report.

- [X] T025 [US5] OPERATOR pre-flight (2026-07-13): `project_preflight_checklist` + FR-013 safety envelope check (arming/mode-flip/failsafe unchanged — bench SBUS/LOS sanity), ground `ERASE:ALL`
- [X] T026 [US5] OPERATOR flight day (2026-07-13 — one flight, 4 spans, all `path complete`; BLE field download; see wrap.md §2): 2 × 3–4 min flights on one flash fill, spans across the standard paths incl. the OOD random-intercept path (TX button 6); BLE download in the field (~7 s)
- [X] T027 [US5] Comparison report (t5 per-axis run by operator; command-domain metrics CLEAN — the observed pitch oscillation is plant-domain, invisible to dCtrl by construction: wrap.md §3 + flight_report.py three-regime numbers; sim-eval tracking comparison deferred to wrap §4.5) `src/analytics/flight_vs_sim_axes.py`: decoded flight CSV vs the SAME candidate's sim per-axis baseline (038 per_axis tooling) → per-axis dCtrl ⟨|Δu|⟩ + amplitude ⟨|out|⟩ with the ±25% band drawn, plus bang-bang saturation % vs the 2026-05-17 flight; PNG artifact named `autoc-039-t<N>-flight-vs-sim_*` in the feature dir
- [X] T028 [US5] SC-006 verdict → carried in `wrap.md` §3/§4 (roll: smooth, confirmed visually + 95°/s RMS; pitch: oscillation is closed-loop plant sensitivity on a neutral-CG article, NOT command roughness; throttle: 86% saturated, 035-line). Deferrals in wrap §4.5: smoothing exhibited yes/no per axis, residual gaps named; new deferrals appended to `specs/BACKLOG.md` (Constitution X)

## Phase 7b: Operator additions (2026-07-11, pre-first-flight)

- [X] T031 Flight-log format v2 — TickRecord gains pos[3]/vel[3]/rabbit[3] telemetry (114 B/tick,
  ~47% headroom) so the log is renderer/trajectory self-contained (no INAV-blackbox join); scale
  table 49 entries; writer + desktop tests + python + in-browser decoders updated in lockstep,
  all validated against a shared synthetic file; the v1 T010 bench artifact stays on record
  (decode via commit 94e5fde if ever needed)
- [X] T032 Renderer `-x` reads the binary flight log: `parseXiaoDataBinary()` (loud-fail on
  version/CRC, spans from explicit Engage/SpanSummary records, rabbit reconstruction + direct
  ground-truth rabbit), magic-sniff dispatch keeps legacy text logs rendering; headless smoke
  test parses the synthetic v2 file (1 span, states/vecs/path correct). NOTE: TickRecords are
  engagement-scoped, so the 'a' full-flight trace shows engaged segments only.

- [X] T033 FlightState breadcrumbs (operator: old -x log covered INAV-arm→disarm) — 25 B raw-frame
  pos/vel/quat record every tick while armed-not-engaged; renderer 'a' all-flight trace is
  continuous again (spans highlighted in world coords, per-span views stay virtual-frame so error
  bars are engage-relative); writer + tests + python (--flightpath CSV) + browser + renderer all
  updated and cross-validated (13-state synthetic: 5 breadcrumbs + 8 ticks, span indices 5-12)

## Phase 7c: Bench hardening + v3 (2026-07-12, waived-flight bench day)

- [X] T034 Alignment HardFault root-caused + fixed — packed `msp_autoc_state_t` at offset 11 in
  `State` put `q[]` on an odd address; `-Ofast` VLDR through the decayed `float*` faulted on the
  first valid MSP state (armed by 17f8c52's second call site de-inlining the quat helper). The
  "USB bus wedge" was this crash playing dead. `alignas(4)` on the member; on-chip J-Link
  confirmation (CFSR UNALIGNED, faulting PC in inavQuatToAerospaceEB)
- [X] T035 fault_guard — nRF WDT (30 s, armed end-of-setup after the console window, fed in loop +
  QSPI erase waits) + fault-vector capture to .noinit (pc/lr/cfsr/hfsr/bfar) reported on next
  boot. Console-only by design (operator: keep it simple). WDT=30s NOT 4s: the dog survives the
  1200-touch into the bootloader, which does NOT feed it — a 4 s dog killed DFU mid-write
  (erase-only "SUCCESS", RESETREAS DOG). Field recovery notes in memory reference_xiao_usb_dfu_quirks
- [X] T036 Flight-log v3 — arm→disarm self-containment (operator ask post console/flash gap
  review): FileHeader += program[96] (nn2cpp source string), events 8-11 = DISENGAGE_REASON
  (stopAutoc enum-ified, all 10 call sites), FAILSAFE + SERVO_SWITCH transitions (seeded after
  ARM), INAV_CLOCK anchor pairs at arm/engage/disengage/disarm. All five readers in lockstep
  (firmware, python, browser, renderer recompile, roundtrip tests 6/6); v3 loud-fails on v2 file
- [X] T037 Blackbox correlation `specs/039-xiao-20hz-flight/correlate_blackbox.py` — anchor-fit
  clock map + breadcrumb/tick vs navPos/quaternion comparison. Powered bench: drift +1409 ppm,
  pos err mean 4.3 cm / p95 9.1 cm, attitude mean 0.21°. Real-craft battery run: +1513 ppm
  (stable crystal offset), pos mean 4.8 cm / p95 13 cm, attitude 0.19°, 0 drops/gaps both runs
- [X] T038 Flight hardware recovered + v3 flashed — old firmware crash made USB unflashable
  in-airframe; recovery = FC in DFU mode (no MSP state → firmware stays alive) + touch → 24 s
  DFU + power cycle. Verified on live INAV: 53 s+ heartbeat through the exact state that killed
  it, WDT fed. Battery-powered 2-span bench flight logged, downloaded over BLE, correlated (T037)

**CHECKPOINT 2026-07-12: READY FOR FLIGHT** — flight xiao runs v3 (weight_id 32fb4398, gen9200
candidate), full chain proven cable-free (arm → spans → BLE download → decode → render →
blackbox-correlate). Remaining before takeoff: T025 operator pre-flight incl. ground `ERASE:ALL`
(flash holds bench flights), then T026 flight day.

**WRAP 2026-07-13**: flight complete, feature closed — see `wrap.md` (verdict SUCCESS; pitch
three-regime finding; operator decision: no sim recalibration until n>1 flight articles).

## Phase 8: Polish & Cross-Cutting

- [X] T029 [P] Constitution VI type-domain grep audit on 039-touched paths (`tools/nn2cpp.cc`, decoder boundary, any `src/`/`include/autoc/` touches): annotate `// raw-ok:` or convert — DONE: arena.h/nn2cpp gp_scalar-clean; flight_log_format.h records section + flight_log.h carry raw-ok annotations; python/js decoders are the gp_scalar re-entry boundary
- [ ] T030 Outcome hygiene: FR-002/FR-011 bench records, DWT number, memo decision, pinned-artifact prefixes all present in outcome.md; BACKLOG updated (delta+varint@50 Hz already filed)

## Dependencies

```
Setup (T001,T002)
  → Foundational (T003→T004→T005)                       [blocks T008]
    → US1 (T006→T007→T008→T009→T010)                    🎯 MVP
      → US3 (T011→T012→T013→T014; T015,T016 [P]; T017)  [T013 needs T007's resolved arena]
        → US4 (T018; T019 [P]; T020→T021)               [soak logs via US3 format]
          → US2 (T022→T023→[T024])                      [memo consumes T019/T020/T021]
            → US5 (T025→T026→T027→T028)                 [T024 loops back to T008-T010 if retrain]
Polish (T029,T030) — after US3 lands, finalize at end
```

## Parallel opportunities

- T002 ∥ T001 (different surfaces)
- T006 ∥ T003 (different test files) — then T007 waits on T006 only
- T011 ∥ (US1 bench T010) — desktop vs hardware
- T015 ∥ T016 (python decoder vs html) once T012 defines the header
- T019 ∥ T018 bring-up; T029 anytime after US3

## Implementation strategy

**MVP = Phase 3 (US1)**: an 038-contract firmware verified on the bench — flyable at 10 Hz even if
nothing else lands. Each later phase is an independently valuable increment: US3 (complete records +
quiet console), US4 (the 20 Hz claim), US2 (the deferred latency answer), US5 (the feature's
outcome). The operator-in-the-loop tasks (T005, T010, T017, T020, T021, T023, T024-run, T025, T026)
are the natural session boundaries; desktop tasks between them are assistant-executable.
