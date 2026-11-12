---
description: "Task list for 045 — M2 tracking: navigation integrity, a re-fly, then a perfect-signal M2 run"
---

# Tasks: 045 — M2 tracking: navigation integrity, a re-fly, then a perfect-signal M2 estimate

**Input**: `specs/045-m2-tracking/`: spec.md (governs; clarified through **2026-11-12**), plan.md (§ Revision 2026-11-12), research.md
(R1–R19), data-model.md, contracts/, quickstart.md, bench-checklist.md, bench-results-20261111.md.
**Tests**: required (Constitution I). Code tasks write their tests first and see them fail. Hardware behaviour is proven by bench
procedures (quickstart.md, bench-checklist.md § H), recorded in `bench-results-YYYYMMDD.md`.
**Organization**: by user story. ⛔ **Phase order follows spec § Execution order** (0a → 0b → G1 → re-fly → G2 → safety ladder
alongside the M2 sim phases), not the P1/P2/P3 labels.
**Revised 2026-11-12 (operator tasks review, spec § Clarifications 2026-11-12):**
- The companion-side state message v2 / flight log v7 / per-tick fault flags are **deferred**. The blackbox already carries those
  signals, so US5 is met by the B1 fields plus an offline flag tool. There is no Foundational phase.
- **The re-fly carries no structural safety change.** The safety work before it is analysis, a bench "before" record and a pilot
  brief. The changes form a **ladder after the re-fly, one change per step**: bench before and after, then a happy-path flight on
  that step alone, then the next step. The MSP traffic work (R-3) starts with a measurement.
- The dmp cache goes first and on its own: read-through, then write-through, each checked with a mini run.
- **The re-fly** (the next test flight) is hardened all-attitude flight, a better blackbox and the improved field download. It flies
  through attitude with nav dead-reckoning errors present, and should see fewer dropouts with more satellites. **The shadow IMU
  (US6) comes after it.** The xiao is left alone for the re-fly. With the TX link up, RC9 off makes INAV take over whatever the
  xiao does (verified in source: research R8).

⛔ **The first implementation pass ends at G2 (T068).** US6 and the safety ladder follow the re-fly at their own pace, each
structural change on its own flight. Phases
11–13 (US8–US10) are written at readiness level and are re-generated after the post-G2 `/speckit.plan` revision. US11 (phase 5)
is outside this pass.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: can run in parallel (different files, no dependency on an incomplete task)
- **[Story]**: US1 dmp cache · US2 BLE download · US3 drift report · US4 INAV config + role · US5 fault visibility (blackbox) ·
  US6 shadow frame · US7 safety prep + re-fly + safety ladder · US8 camera · US9 perfect-signal sim · US10 M2 run
- **(operator)**: needs the operator: the article, a flash, an FC write, a flight, `rebuild-perf.sh` or the regression gate.
  Everything else can be done at the desk.

### Standing rules (every task)

- ⛔ No rebuild of a tree while a run launched from that tree is training (workers re-exec `build/bin/autoc`).
- The operator drives `scripts/rebuild-perf.sh`, the regression gate, `git push`, every FC write, every flash and every flight.
  Commits on `045-m2-tracking` are fine. Interactive iteration uses incremental `cmake --build build`.
- FC writes go only through `specs/045-m2-tracking/inav_cli_apply.py` (whitelisted set → get read-back → save), with the operator's
  approval per change set. Close INAV Configurator before any scripted MSP or CLI. The `/dev/ttyACM*` numbers swap between sessions
  (identify by `udevadm` `ID_MODEL`). CLI `exit` / `save` reboots the FC.
- Disconnect the GPS before flashing an INAV controller. The motor is disconnected (or props off) for every [SERVO] item. Props go
  on only for checklist § D, with the article restrained.
- ⚠️ Article 1's FC already carries a change set that is **not** in `xiao/inav-hb1.cfg` (bench-results-20261111.md § Change set and
  § 2026-11-12). Do not write it into the repo cfg before T049.
- ⛔ **Little steps on anything that flies.** The happy path is proven in flight, and a structural change invalidates that. One
  structural change per flight. Fault-path behaviour and INAV failsafe settings change only through the safety ladder (Phase 10).
- Already done on the 2026-11-11/12 bench, not repeated here: checklist A1–A6, A9, C1, C2, the bench half of E1/E2, and the remote
  BLE baseline (F1). See bench-results-20261111.md.
- New backlog items go to `specs/BACKLOG.md` (Constitution X), not to new memory files.

---

## Phase 1: Setup

- [X] T001 Create `specs/045-m2-tracking/MANIFEST.md` with § Starting point, § Preservation, § Change sets, § Builds, § Gates and
  § Safety ladder. Later tasks append keys, tags, hashes, build results and verdicts to it
- [X] T002 (operator) Starting-point build gate on the branch head, recorded in `specs/045-m2-tracking/MANIFEST.md` § Starting
  point: clean `bash scripts/rebuild-perf.sh`; `cd build && ctest` (R15 saw 55/55 at fb4d3ba; record the actual count);
  `~/.platformio/penv/bin/pio run -e xiaoblesense_arduinocore_mbed -d xiao` (baseline RAM 53.6 % / flash 45.8 %). Also record the
  git hash of the xiao build flown on 11-08, which T062 diffs against. No hash was recorded at the time. The candidate is `78e7d6e`
  (2026-11-07), the last `xiao/` commit before the flight; nothing under `xiao/src` or `xiao/include` has changed since. Confirm
  it against the build line in the boot banner (T034)
- [X] T003 [P] Decode the existing xiao logs for N1. `flight-results/flight-20261108/flight_log_2026-11-09T00-14-08_flight_001.bin`
  (header: v6) goes through today's `src/analytics/flightlog_decode.py -o` into a `.csv` beside it. Both
  `flight-results/flight-20261018/flight_log_*.bin` (v5) go through the decoder from tag `043-final` (`git show
  043-final:src/analytics/flightlog_decode.py`, run from the scratchpad). The CSVs are regenerable, so they are **not** committed
  (`.gitignore`, per the 2026-10-04 rule for decoded flight artifacts). Record in MANIFEST § Preservation which tag reads which
  version (`044-final` → v6, `043-final` → v5) and the regeneration commands
- [X] T004 [P] Confirm the flight baseline of record. `specs/044-m1-craft-history/artifacts-t1/nn_weights-t1-g800.dat` must be
  genome `a5ed3521ea1e70b7`. In `autoc-m1`, `autoc-9223370244339188361-2026-10-20T16:59:47.446Z/gen9200.dmp.zst` (gen 800: dmp
  names encode 10000 − gen), gen 1 and the last gen must carry `retain=keep` (`aws s3api get-object-tagging`). Record in MANIFEST
  § Preservation

*(No Foundational phase: the companion-side wire/log change it held is deferred, and nothing else is shared across the stories.)*

---

## Phase 2: User Story 1: training-box reports stop paying egress (P1, phase 0a) 🎯 first, standalone

**Goal**: every dmp read and write on the training box goes through one local cache (contracts/dmp-cache.md, research R1). It
lands in two small steps, each checked with a mini run.
**Independent test**: one full report cycle on a run trained with the cache in place. The S3 access logs show zero GET.OBJECT by
the training box of keys it wrote itself.

### Step 1: read-through (fill on miss)

- [X] T005 [US1] Test first, in the already-registered `tests/s3_run_selector_tests.cc` (no CMake edit, so US1 merges alone).
  Test a read API with an injectable fetch, e.g. `autoc::dmpCacheFetch(const DmpCacheConfig&, bucket, key,
  std::function<std::string()> fetchRaw)`, on a temp dir:
  - hit ⇒ `fetchRaw` is not called;
  - miss ⇒ fetched once and the cache filled with tmp + rename, so the second call hits;
  - the path is `<dir>/<bucket>/<key>`;
  - no `.tmp.<pid>` is left behind, and a stray partial tmp file is never read;
  - a fill into an unwritable dir ⇒ throws with the path in `what()`;
  - a corrupt entry (bad zstd) ⇒ throws with the path, with no re-fetch;
  - disabled ⇒ always fetches, writes nothing
- [X] T006 [US1] Implement it in `include/autoc/util/s3_run_selector.h` + `src/util/s3_run_selector.cc`:
  - read `AUTOC_DMP_CACHE` once (function-local static) and log it once per process (`dmp cache: disabled (AUTOC_DMP_CACHE
    unset)` or `dmp cache: <dir>`);
  - `s3GetDmpBlob` (~line 126) checks the cache before the GET, and caches the compressed body before inflating.

  T005 passes
- [X] T007 [P] [US1] Document `AUTOC_DMP_CACHE` in `docs/toolchains.md` § training box: its purpose, the dir, the one-time full
  re-read after `dmp-dump` is rebuilt (the DSIG cache key in `generate_pngs.sh` changes), and the sweep (T012). `scripts/train.sh`
  and `scripts/generate_pngs.sh` print the effective value at start. The operator sets it in the training box's login environment
  (it is an env var because the Python reports spawn the tools, research R1)
- [X] T008 [US1] Incremental build + ctest. Then check the failure modes by hand (AS2 / AS3): a truncated cached entry ⇒ `dmp-dump`
  stops naming the path; an unwritable dir ⇒ the fill stops naming the path; unset ⇒ the one-line notice and today's behaviour
- [ ] T009 [US1] (operator) Mini run A, read-through. Create `specs/045-m2-tracking/cache-smoke.ini` (a copy of
  `specs/044-m1-craft-history/smoke.ini` with `NumberOfGenerations = 10`). Run `scripts/train.sh
  specs/045-m2-tracking/cache-smoke.ini logs/autoc-045-t0-cache-smoke-a.log` (the pre-run build gate per Constitution IX), then
  `scripts/generate_pngs.sh m1 logs/autoc-045-t0-cache-smoke-a.log` twice with the cache set. When the access logs land
  (`s3://autoc-s3-access-logs/m1/`), count GET.OBJECT per key with a new `specs/045-m2-tracking/s3_egress_check.py`. Expect ≤ 1
  per key (the first report fills; the second reads locally). Record in MANIFEST

### Step 2: write-through

- [X] T010 [US1] Test first, in `tests/s3_run_selector_tests.cc`: `autoc::dmpCacheStore(const DmpCacheConfig&, bucket, key, const
  std::string& compressed)` writes tmp + rename to `<dir>/<bucket>/<key>`, a following fetch hits without calling `fetchRaw`, an
  unwritable dir throws with the path, and disabled writes nothing
- [X] T011 [US1] `s3PutDmpBlob` (~line 142) writes the compressed bytes through `dmpCacheStore` after a successful put. A
  cache-write failure throws with the path: the run must not silently lose its cache. T010 passes
- [X] T012 [P] [US1] `scripts/dmp_cache_sweep.sh -d <dir> [--apply]`, dry-run by default. For each run prefix under
  `<dir>/<bucket>/`, read the retain tag of one object (`aws s3api get-object-tagging`). Delete the local entries of
  `retain=expire` runs older than 30 days. Never touch `retain=keep`. Print what it removes or would remove
- [ ] T013 [US1] (operator) Mini run B, write-through = SC-001: the same mini run as T009 (`logs/autoc-045-t0-cache-smoke-b.log`),
  then reports twice. Expect zero GET.OBJECT from the training box on that run's keys (`s3_egress_check.py`). Record in MANIFEST.
  The month-end egress figure is T113

**Checkpoint**: US1 merged and in use (no CMake edit, no format change).

---

## Phase 3: User Story 2: field log download is fast and survives a drop (P1, phase 0a) 🎯 front task

**Goal**: a ~300 kB log in < 60 s with the phone ~3 m from the aircraft, and an interrupted download that resumes byte-identical
(contracts/ble-download-v2.md, research R2).
**Independent test**: on the bench, download a full-size log, interrupt it mid-transfer, resume, and compare with a wired read.
**Flight safety**: every change here is on ground-only paths. BLE stops at arm, as today. The download-time MSP skip (T021) is gated
on *disarmed and downloading*.

### Step 1: measure first (FR-005: the cause is recorded before it is fixed)

- [X] T014 [US2] Firmware instrumentation in `xiao/src/bluetooth.cpp`, protocol unchanged:
  - status `MTU:<att_mtu>` at connect (`ATT.mtu(handle)`);
  - compare `writeValue`'s returned length with the bytes read (~line 232) and count short writes;
  - track the maximum `loop()` duration and the maximum gap between `NEXT`s;
  - status `STAT:<offset>:<short_writes>:<max_loop_ms>:<max_next_gap_ms>` every 16 chunks, and appended to `DOWNLOAD_DONE`;
  - `BLE.setDeviceName("XIAO-GP")`, so BlueZ stops caching the name "Arduino"
- [X] T015 [US2] Page instrumentation in `xiao/web/flight_logger.html`:
  - an in-memory log with a **Copy log** button;
  - READY t0; per chunk the send time, receive time, `byteLength` against the expected chunk, and offset;
  - errors with name and offset; the disconnect offset and elapsed time;
  - a summary (bytes, s, kB/s, CRC result);
  - the device filter also matches the service UUID `F1706000-…`
- [X] T016 [US2] A wired path, which the xiao lacks today. It is the byte reference for the comparison, and a field fallback with a
  laptop. Add a USB-serial `DUMP:<file>` console command (raw bytes framed with size + CRC32; refused while armed) and
  `specs/045-m2-tracking/xiao_serial_dump.py` to capture it
- [ ] T017 [US2] (operator) The measurement of record (quickstart § 8 steps 1–2, checklist F1). Build gate first: `pio run -e
  xiaoblesense_arduinocore_mbed -d xiao` SUCCESS (record RAM and flash in MANIFEST). Then flash T014–T016. The field phone
  downloads a full ~300 kB log at ~3 m (the SC-002 distance), plus one pass at a longer range for the record. Paste the **Copy log** output into
  `specs/045-m2-tracking/ble-measure-YYYYMMDD.md`, plus a `ble_cdp_probe.py` run from the training host. Record the measured cause
  (MTU, round trip, duplicate delivery, MSP-poll stalls) in research.md R2 "Measured" (US2 AS3). ⛔ T018–T022 wait for this record

### Step 2: protocol v2 (tests first)

- [ ] T018 [US2] Test first. Write a pure window state machine `xiao/include/ble_dl2.h`: next offset, acked offset, window, resend
  from the ack on a gap, 5 s inactivity ⇒ idle, short notify ⇒ `ERR:SHORT_NOTIFY`. Test it in the new `tests/ble_dl2_tests.cc`
  against a simulated channel that drops, duplicates and reorders notifications. A reference receiver in the test implements the
  contract's page rules: offset-tagged chunks, duplicates ignored, ACK = highest contiguous byte. Assert byte-identical reassembly
  and resume from any cut point. Also test the pure gate predicate `dl2ShouldSkipMspPoll(downloading, armed)` that T021 uses: true
  only when downloading and disarmed; false the moment `armed` goes true. Register the test in `CMakeLists.txt` (⇒ clean `rebuild-perf.sh` before the next
  determinism-affecting run, operator)
- [ ] T019 [US2] Firmware v2 in `xiao/src/bluetooth.cpp`, using `ble_dl2.h`:
  - `DL2:<file>:<offset>:<window>` → `READY2:<size>:<chunk>:<mtu>` (chunk = MTU − 3 − 4);
  - notifications `[u32 offset][payload]`, back to back, up to `window` unacknowledged;
  - `ACK:<offset>`, `ABORT`, `DONE2:<size>:<crc32>`, `ERR:<reason>:<offset>`;
  - request MTU 247 after connect. Check that `ATT.exchangeMtu` works from the peripheral role in ArduinoBLE 2.1.0
    (`ATT.cpp:168`); if it does not, size chunks from the MTU the central negotiated (`ATT.mtu`);
  - request a short connection interval and 2M PHY where available.

  T018 passes
- [ ] T020 [US2] Clear the download wedge: the 5 s inactivity timeout clears `transferInProgress` and returns
  `xiao/src/flash_logger.cpp` from `FLASH_DOWNLOADING` (~lines 547–556) to idle, so a new `DL2` is accepted without a reconnect
- [ ] T021 [US2] `xiao/src/controller.cpp` `controllerUpdate()`: skip MSP polling only when `dl2ShouldSkipMspPoll()` (from
  `ble_dl2.h`, tested in T018) says so, i.e. while a download runs **and** INAV reports disarmed. Arming still stops BLE, as
  today. Verified on the bench in T023
- [ ] T022 [US2] Page v2 client in `xiao/web/flight_logger.html`: `DL2` with resume; ACK every N chunks and on a gap; duplicates
  ignored by offset; the received buffer kept across reconnects, keyed by file name + size; CRC32 checked at `DONE2`. The T015
  instrumentation stays
- [ ] T023 [US2] (operator) SC-002 acceptance (quickstart § 8 step 3). Build gate first: `cmake --build build && (cd build &&
  ctest)` incl. `ble_dl2_tests`, and `pio run` SUCCESS. Then:
  - with the field phone ~3 m from the aircraft, ~300 kB downloads in < 60 s;
  - walk out of range mid-transfer, reconnect and resume, then `cmp` the result with the T016 wired dump;
  - stop acking mid-transfer for > 5 s without disconnecting: a new `DL2` is accepted without a reconnect (T020);
  - the xiao console shows MSP polling paused during the download and resumed after it (T021);
  - arm mid-download (TX on, motor disconnected): BLE stops and MSP polling resumes at once.

  Record in `ble-measure-YYYYMMDD.md` and MANIFEST

**Checkpoint**: US2 done. The re-fly's download path works.

---

## Phase 4: User Story 3: how wrong the navigation and attitude frame is (P1, phase 0b)

**Goal**: the N1 drift report and the attitude-stability budget (FR-006, FR-006a, SC-003), offline on the committed logs.
**Independent test**: the report reproduces from the committed tools and the 11-08 / 10-18 logs.

- [X] T024 [P] [US3] `specs/045-m2-tracking/span_attitude.py`: add the GPS-derived specific-force reference per window (the
  `nav_accel_fit.py` method: GPS-differentiated acceleration plus gravity in the earth frame, against the body specific force
  rotated by each attitude source). Report the attitude error of INAV and of gyro-only-from-engage at 5 / 10 / 17 s / end per span.
  State the reference's own uncertainty (10 Hz GPS differentiation noise, fitted lag). Take the flights and spans as CLI options,
  with today's two flights as the defaults. Write `specs/045-m2-tracking/n1/attitude_drift.csv`
- [X] T025 [P] [US3] `specs/045-m2-tracking/nav_dead_reckon.py`: add a `--span` mode. It propagates velocity and position from
  engage, span-local, with the body specific force rotated by INAV attitude and by gyro-only attitude, at the 240 Hz blackbox
  rate, against GPS, reported at 5 / 10 / 17 s / end. Write `specs/045-m2-tracking/n1/nav_drift.csv`
- [X] T026 [P] [US3] INAV fault inventory per span. Run `specs/045-m2-tracking/nav_span_scan.py` on the T003 CSVs, and add a
  **blackbox mode** to it that scans navPos / navVel / navEPH / navEPV / numSat: freezes, resets and steps, with times and
  magnitudes. Emit the per-tick residual distributions |Δpos − v̄·Δt| (horizontal and vertical) that set the STEP / FROZEN
  thresholds. Write `specs/045-m2-tracking/n1/faults.csv`. US5's T053 builds the flag output on this mode
- [X] T027 [P] [US3] Datasheet error terms into research.md R4 for MPU6000 (article 1 FC), ICM42688P (article 2 FC) and
  LSM6DS3TR-C (xiao): gyro scale tolerance and nonlinearity, cross-axis, bias instability and temperature drift, g-sensitivity,
  noise density, range and ODR. Fill the budget table per source and per term
- [X] T028 [US3] Write `specs/045-m2-tracking/drift-report.md`, the N1 report of record (data-model § Drift report):
  - per flight × span × source: the R3 columns, attitude drift, velocity and position drift, and INAV's faults;
  - the AS2 answer: the better source in sustained high-load manoeuvres, by how much, with the reference's uncertainty;
  - the attitude-stability budget for each candidate (INAV AHRS, xiao gyro + GNSS-velocity-aided filter, an on-chip-fusion part),
    whether < 1° over a span is reachable, and its error sources (FR-006a);
  - the R4 decision 4 revisit: keep < 1°, or "smooth and < 3°" with 1° kept as the design aim;
  - the reproduction commands (SC-003)
- [X] T029 [US3] Pin values from N1 into `drift-report.md` § Pinned values and `data-model.md`: the FROZEN / STEP thresholds
  (consumed by T053) and the `inav_max_eph_epv` value (research R5: 2000–3000 cm, set from the measured coast durations;
  consumed by T033)

**Checkpoint**: US3 reported. It sizes US6's decision and sets the offline flag thresholds.

---

## Phase 5: User Story 4: INAV configured for all-attitude flight, its role during autoc defined (P1, phase 0b)

**Goal**: one reviewed, recorded, read-back configuration change set per article (FR-007, 007a, 007b, 008, 008a, 009, 010, 027),
and INAV's role written down. This is the core of the re-fly: improved alignment, calibration and INAV sampling and tuning.
**Independent test**: on the bench, the recorded configuration loads and its read-back confirms each setting. The change set is
reviewable against the configuration of record.

### Tooling

- [X] T030 [P] [US4] `specs/045-m2-tracking/cal_verify.py` (research R6b), on a `blackbox_decode` CSV:
  - static-pose segmentation;
  - accel ellipsoid fit (offset, scale, cross-axis) and |f| per pose;
  - the gyro's static residual mean per axis;
  - turn-count gyro scale (∫gyro vs N·360°);
  - mag ellipsoid fit (hard and soft iron), |m| variation and dip;
  - heading at known bearings.

  It prints each R6b pass criterion with PASS / FAIL
- [X] T031 [P] [US4] `specs/045-m2-tracking/mount_survey.py` (research R7): per-axis spectra from a denom-1 blackbox (GYRO_RAW +
  ACC). It separates peaks that track motor RPM / throttle from fixed mount modes and writes a per-board table
- [X] T032 [P] [US4] `specs/045-m2-tracking/msp_bench_read.py`: add per-satellite C/N0 (`MSP_GPSSVINFO`) for checklist D4, and a
  `--watch` mode that logs the I2C error count from MSP status (A10)
- [X] T033 [US4] Create `specs/045-m2-tracking/config-changeset-art1.md` (data-model § Configuration change set). First, the rows
  already on the FC from bench-results-20261111.md (GNSS, AIR_4G, `init_gyro_cal`, `inav_w_acc_bias`, `ins_gravity_cmss`,
  `gyro_zero_*`, `align_board_*`, `align_mag_*`), each with class, evidence, read-back and trained-on effect, plus article 1's
  receiver record (SAM-M8Q, ROM 3.01, max 3 concurrent GNSS: FR-008, AS2). Then the pending rows: `inav_max_eph_epv` (T029),
  `inav_default_alt_sensor` (T043), the blackbox groups and rate (T051). Then the calibration rows, with date, article, level
  reference and conditions. Failsafe settings are **not** in this change set: they move only through the safety ladder (T095)

### Bench and outdoor, article 1 (operator; the scripts run at the desk and results are recorded)

- [ ] T034 [US4] (operator) [USB] Checklist A7 + A8. Record the SBUS receiver's own failsafe mode: it must be "no pulses" or a
  flagged failsafe, never "hold" (R17). Record the xiao boot banner (build, genome `a5ed3521ea1e70b7`, log format version, MSP poll
  rate) by listening only, without racing the 8 s boot window. Results in `bench-results-YYYYMMDD.md`
- [ ] T035 [US4] (operator) [USB] C3: a 6-point accel calibration on the level reference, then ≥ 12 static poses (blackbox ACC,
  10 s each) through `cal_verify.py`. Pass: |f| = 1.000 ± 0.005 g; cross-axis < 1 % (else record it). Then re-record
  `ins_gravity_cmss` level and still (bench-results 11-12 read |a| 0.986 g). Rows in `config-changeset-art1.md`
- [ ] T036 [US4] (operator) [USB] C4 + FR-007a. Record the gyro zero and gravity level and still, with `init_gyro_cal = ON` for one
  boot. Then set them with `inav_cli_apply.py`, with `init_gyro_cal = OFF` and `inav_w_acc_bias = 0` (both already on the FC), and
  read them back. Write the explanation of the stored 943.52 cm/s² into research.md R5 and `config-changeset-art1.md`: a boot
  calibration taken off level (909 / 943 / 979 on successive boots, bench-results 11-11). Repeat the read next session; a move of
  more than ~10 LSB ⇒ re-record every session
- [ ] T037 [US4] (operator) [USB] C5: the gyro scale factor, from 10 turns per axis on a jig with the blackbox at denom 1–2,
  through `cal_verify.py`. Record it per axis in `config-changeset-art1.md` and in the R4 budget. Above 0.5 % ⇒ noted (INAV cannot
  correct it)
- [ ] T038 [US4] (operator) [USB] C6 + FR-008a. On the level reference, measure the back-tilt of the FC panel and of the GPS panel,
  and confirm the module's arrow points forward. Set `align_mag_roll/pitch/yaw` from the measured GPS tilt (CW270FLIP + tilt,
  research R19) with `inav_cli_apply.py` and read it back. Row in `config-changeset-art1.md`. This must precede T044: calibrating a
  mis-aligned compass bakes the error in
- [ ] T039 [US4] (operator) [USB] C7: photograph the xiao board in the airframe and record its axes relative to the aircraft in
  `bench-results-YYYYMMDD.md` (feeds US6)
- [ ] T040 [US4] (operator) [PROP] D1 mount survey (quickstart § 5): denom 1, GYRO_RAW + ACC, a throttle sweep, then three steady
  points; analyse with `mount_survey.py`; results in `bench-results-YYYYMMDD.md`
- [ ] T041 [US4] (operator) [PROP] D2: heading drift across the throttle range, and with the servos sweeping and the motor off (MAG
  logged); the I2C error count under motor load (`msp_bench_read.py --watch`, A10). Results in `bench-results-YYYYMMDD.md`
- [ ] T042 [US4] (operator) [PROP] D3 + D4: the baro's prop-wash step and a 10 min static baro drift; per-satellite C/N0 via the CLI `gpssats` command (disarmed; leaving the CLI with `exit` reboots the FC) or u-center through `gpspassthrough`. INAV 8.0's `MSP_GPSSVINFO` is a stub that reports no satellites (found 2026-11-12); `msp_bench_read.py --svinfo` falls back to sats / HDOP / EPH, with the
  xiao powered vs unpowered and the motor off vs running. Results in `bench-results-YYYYMMDD.md`
- [ ] T043 [US4] (operator) [OUT] E1 + E2 + E3, outdoors:
  - after save + reboot, `status` shows GPS + Galileo + GLONASS active;
  - the GPS frame interval in a short blackbox `*.gps.csv` is ≈ 100 ms (200 ms ⇒ drop GLONASS);
  - a 10 min static: sats, HDOP, EPH, EPV, GPS-only vs multi-GNSS.

  That decides `inav_default_alt_sensor` (`GPS_ONLY` only if EPV ≤ ~5 m). Row in `config-changeset-art1.md`
- [ ] T044 [US4] (operator) [OUT] E4 + quickstart § 4b (after T038):
  - dip and |m| at N / E / S / W, level; expect dip ≈ 61° and equal |m|;
  - the outdoor compass calibration if they are not;
  - a second logged tumble through `cal_verify.py` (residual hard and soft iron);
  - heading at 8 known bearings < 5°.
  - N1 found an INAV heading offset of −8.6 to −18.8° in pre-engage cruise; its size is close to the site's 12.9° declination,
    so check the declination sign / auto-declination before blaming the compass (drift-report.md).

  Rows in `config-changeset-art1.md`

### Review and record

- [ ] T045 [US4] The FR-007 review table in `config-changeset-art1.md`. Class every changed setting as A / B-accuracy / B-delay /
  C / D / E / hardware, with evidence and its effect on what the policy was trained on. For example, the board-alignment change is
  B-accuracy: it removes the 3.7° nose-up bias in the quaternion inputs. Confirm there is no class A or B-delay change (FR-017)
  and no class D (failsafe) change. The board-alignment change (row 9) also rotates the rate loop's gyro axes by 3.6°
  (~6 % roll↔yaw coupling removed). **Operator 2026-11-12: an accuracy fix**: the sim already assumes aligned axes. Sensor-alignment
  variations in the sim (board + compass) are filed in `specs/BACKLOG.md` § 045 deferrals. `gps_min_sats` stays 6 (row 19). State the calibration policy (FR-007a)
- [ ] T046 [US4] The FR-007b mount review per board (FC, xiao, GPS / compass), from T040 plus the level-boot gravity: resonance
  frequency and axis, and the alignment result, in `config-changeset-art1.md`. A mount change is hardware hygiene, recorded as
  A/B-able on the next flight. It is not a re-fly hold unless it saturates a sensor
- [ ] T047 [US4] FR-010 / AS4. Confirm research.md R8 holds the per-dependency table: rate loop, state to the companion and
  fallback × "depends on INAV's nav estimate?" × the gates applied during autoc, plus the reduced-role design (built after G2). Add
  the table to R8 if any column is missing. File "build the reduced-role INAV" in `specs/BACKLOG.md`
- [ ] T048 [US4] Correct the wrong comment at `xiao/src/msplink.cpp:1093-1095` (comment only; no code change). MANUAL is on RC6,
  which the override holds; `aux 4` is NAV RTH on RC8; the pilot escape is RC9 override off (research R8; the B-8 decision). `pio run` SUCCESS
- [ ] T049 [US4] (operator) The final article 1 change set. Apply any remaining rows with `inav_cli_apply.py`, `save`, reboot, and
  read every row back (FR-009). Capture `diff all` into `specs/045-m2-tracking/bench-YYYYMMDD/`. Then, with the operator's
  approval, write it into `xiao/inav-hb1.cfg` (FR-027) and record the commit in MANIFEST § Change sets (SC-004)

### Article 2 (only if it will fly in the window; FR-019)

- [ ] T050 [P] [US4] (operator) Article 2 (`xiao/inav-hb1-art2.cfg`: HGLRCF722LITE, M10Q-5883). Apply and read back its
  outstanding fixes (`align_mag_yaw = 3600`, `vbat_scale = 1110`) and the GNSS, AIR_4G and calibration-policy rows. Run quickstart
  §§ 1–4 and 4b and record its receiver identity (A5). Record all of it in a new `specs/045-m2-tracking/config-changeset-art2.md`
  and update `xiao/inav-hb1-art2.cfg`

**Checkpoint**: the US4 change set is on article 1 and in the repo; INAV's role is recorded.

---

## Phase 6: User Story 5: estimator faults visible in every flight, from the blackbox (P1, phase 0b)

**Goal**: the B1 blackbox fields and an offline flag tool (FR-013, FR-014; FR-011 deferred, spec § Clarifications 2026-11-12). No
firmware change.
**Independent test**: outdoors, with autoc engaged on the bench, block the GPS antenna. The offline tool flags the interval in the
blackbox, and the policy keeps flying exactly as before.

- [ ] T051 [US5] (operator) B1 and the blackbox rate (FR-014; class E), applied with `inav_cli_apply.py` and read back:
  - enable the `MAG` and `NAV_ACC` field groups;
  - keep `debug_mode = VIBE`: it is the blackbox's only source of accWeight (research R9);
  - set `blackbox_rate_denom` from the write bandwidth. Bench-log with the motor running at 8, 16 and 32 and take the fastest with
    no dropped frames in the decode header. **1:32 (62.5 Hz) is acceptable** (operator 2026-11-12).

  Record the chosen rate and its cost in `config-changeset-art1.md`. At 62.5 Hz, gyro-only propagation is coarse (R4 sampling
  term; 240 Hz already shows a ~3° floor), so the re-fly's attitude check leans on the GPS specific-force reference. Mount content
  is a bench-only measurement (T040 at denom 1)
- [ ] T052 [US5] Test first: write `specs/045-m2-tracking/test_nav_flags.py` (plain asserts, `python3` to run) against a committed
  blackbox excerpt of 11-08 span 1 (`specs/045-m2-tracking/fixtures/bb_1108_span1.csv`, cut from
  `flight-results/flight-20261108/blackbox_log_2026-11-08_160651.01.csv`). It must flag the 2.3 s freeze and the 105 m reset, and
  a clean excerpt (span 2) must flag nothing
- [ ] T053 [US5] Extend the T026 blackbox mode of `specs/045-m2-tracking/nav_span_scan.py` into flags per engaged span:
  - INVALID: EPH / EPV at or above `inav_max_eph_epv`;
  - FROZEN; STEP_XY / STEP_Z, with the T029 thresholds;
  - LOW_SATS (< 6); GPS frame gaps from the `*.gps.csv`;
  - failsafe phase from the slow frames.

  It prints the `flight_gates.py --drop FILE.csv#SPAN` arguments for flagged spans (FR-013), so the 044 tools are not edited. T052
  passes
- [ ] T054 [US5] (operator) [OUT] SC-005, the satellite-block test (quickstart § 7, checklist E5): 3D fix and ≥ 8 sats, armed in
  bench mode, autoc engaged, motor disconnected, blackbox at the T051 settings. Cover the antenna for ~15 s. Then the tool flags
  LOW_SATS, the EPH growth, the freeze and the step on reacquire, and engagement is unchanged. Record in `bench-results-YYYYMMDD.md`
- [ ] T055 [P] [US5] `specs/BACKLOG.md`: per-tick companion fault flags deferred (state message v2, flight log v7 tick fields, an
  on-board derivation). The contracts are the design of record. A revived v2 also needs `failsafe_phase`, receiver-signal flags and
  box bits 32–63 (R16 B-5; bench A3: MSP RC OVERRIDE is bit 40)

**Checkpoint**: US5 bench-verified from the blackbox (a G1 item).

---

## Phase 7: User Story 7, part A: safety preparation, with no behaviour change (before the re-fly; FR-015b)

**Goal**: know exactly how today's system behaves in each fault, brief the pilot on it, and lay out the ladder of changes and the
failsafe debate's inputs. Nothing here changes firmware or a saved setting.
**Independent test**: the "before" record, the timelines and the pilot brief exist and agree with each other.

- [ ] T056 [US7] (operator) The **before** record: run bench-checklist B1–B10 and H1–H4, H6 and H7 on today's firmware and
  configuration, ×3 each (H8), motor disconnected. The candidate failsafe settings are measured **in RAM only, never saved**:
  `failsafe_delay` at 10 / 5 / 3 (B7), and LAND vs DROP (B6 / H1), with the pilot actions and time needed to regain control. This is
  also the bench test of **today's** in-flight re-arm paths (B6 after DROP, B9 emergency arming): each one timed, with its pilot
  action count. H5 (MSP
  flood) stays on hold until a verified hard power cycle exists. Results in `bench-results-YYYYMMDD.md`
- [ ] T057 [P] [US7] Trace ANGLE's recovery path from inverted and from vertical in the fork (`~/inav/src/main/flight/pid.c` level
  path, `fw_p_level`) and record it in research.md R17. Any change to surface behaviour after a fault needs this trace first
  (FR-015b (2))
- [ ] T058 [US7] The FR-015b timelines, in `specs/045-m2-tracking/fault-timelines.md`. For each of RX glitch / loss, TX reboot, FC
  reboot / hang, xiao wedge / reboot, GPS brownout and MSP flood:
  - the timeline to stabilised, pilot-controllable flight under **today's** configuration, measured in T056, against the R17 time
    budget (~2.9 s at 100 m);
  - what each candidate change would do to it;
  - throttle to 0 first, in each case;
  - the in-flight re-arm path and its pilot action count, including the transmitter-side option (R-6) and the limits of emergency
    arming (B9).

  This is the input to the failsafe-configuration debate
- [ ] T059 [US7] The pilot brief for **today's** firmware, `specs/045-m2-tracking/pilot-brief.md` (B-7):
  - escape = RC9 override off. With the TX link up, INAV stops merging the xiao's channels within about one RX frame, whatever
    the xiao is doing (verified in source, research R8);
  - RC6 MANUAL does nothing while engaged;
  - RC8 RTH / POSHOLD works only with a valid estimate;
  - after any stop, INAV holds the last command, **including throttle**, for ~1.4 s;
  - after an xiao glitch or reboot, autoc can re-engage by itself ~9 s later if RC9 is still high, so **flip RC9 off after any xiao
    glitch**. That is the whole mitigation for the re-fly: the xiao is left alone (operator 2026-11-12);
  - the DROP re-arm sequence
- [ ] T060 [US7] The safety ladder plan, `specs/045-m2-tracking/safety-ladder.md`. It holds the ordered steps (proposed order in
  Phase 10), the rule (one structural change per step: bench before and after, a happy-path g800 flight on that step alone, a short
  note, then the next step), and where the failsafe-configuration debate stands. The operator approves or reorders the steps. A
  pairing of two fault-path-only steps on one flight is the operator's call, recorded with its reason

**Checkpoint**: the safety picture is documented and the pilot is briefed. No safety behaviour has changed (a G1 item).

---

## Phase 8: User Story 7, part B: G1, the re-fly and G2 (P1, phase 1)

**Goal**: fly 044-t1 g800 on the Phase 0 configuration: hardened all-attitude flight (alignment, calibration, INAV sampling and
tuning, more satellites), a better blackbox and the improved field download. Get a log whose nav frame stays continuous through
all-attitude flight with dead-reckoning errors present, or whose faults are flagged and measured (SC-006, SC-007).
**Independent test**: the flight's own logs, analysed with the 11-08 tools plus the offline flags.

- [ ] T061 [US7] The G1 readiness record, `specs/045-m2-tracking/g1-readiness.md`: each spec G1 bullet with its evidence link.
  - US3 report + budget (T028);
  - US4 change set recorded, read back and on the aircraft, including the calibration policy and compass alignment (T049);
  - the reduced-role design written (T047);
  - accel calibration, board alignment and outdoor compass per article (T035–T044, T050);
  - US5 verified from the blackbox (T054), with the blackbox rate chosen (T051);
  - US2 verified on the bench (T023, SC-002): **required**; the re-fly waits for it;
  - no shadow IMU on the re-fly (US6 comes after it);
  - the safety before record, timelines and pilot brief (T056–T060);
  - FR-017: no class A, B-delay or D change (T045);
  - **no structural safety change on the re-fly**.

  (operator) signs it off. Record in MANIFEST § Gates
- [ ] T062 [US7] The pre-flight build (the project's preflight checklist: training → build → flash → configure → verify).
  - `git diff <the 11-08 flight build, T002>..HEAD -- xiao/` shows only US2's ground-only download paths, T016's dump and T048's
    comment. Anything else holds the flight;
  - `xiao/src/generated/nn_program_generated.cpp` is g800 from `specs/044-m1-craft-history/artifacts-t1/nn_weights-t1-g800.dat`
    (regenerate with `nn2cpp`, unrolled by default, only if it differs);
  - `pio run` SUCCESS; (operator) flash; the boot banner shows genome `a5ed3521ea1e70b7`;
  - the INAV firmware is the build flown on 11-08 (no fork change); `diff all` = `xiao/inav-hb1.cfg`
- [ ] T063 [US7] (operator) The session checklist (quickstart § 0, bench-checklist.md § G): erase the xiao log; re-record the gyro
  zero (`init_gyro_cal = OFF`); `diff all` against the repo; read the pilot brief. Tick § G
- [ ] T064 [US7] (operator) Fly the re-fly card (quickstart § 10): article 1, g800, streamer as on 11-08; ≥ 4 engaged spans; ≥ 60
  s of box or circle at altitude in ANGLE or MANUAL (≥ 4 course sectors); optional MANUAL doublets. Note wind and conditions in
  `flight-results/flight-YYYYMMDD/notes.md`. Article 2 flies in the same window if ready (it is the first dropped if the weather
  turns)
- [ ] T065 [US7] Retrieve the BLE v2 download (time it: SC-002 field evidence), the blackbox and the notes into
  `flight-results/flight-YYYYMMDD/`. Decode and commit
- [ ] T066 [US7] The analysis, `specs/045-m2-tracking/flight-analysis-YYYYMMDD.md`:
  - SC-006: the added blackbox fields over 100 % of engaged time, and the offline flags for every span (`nav_span_scan.py`
    blackbox mode);
  - SC-007: spans with a freeze or a reset > 10 m vs 11-08's 1 of 4; otherwise each fault flagged, measured and fed into the US6
    decision;
  - faulted spans listed and excluded (`flight_gates.py --drop …`, from T053's output);
  - tracking and control vs 11-08 as flown (FR-018, AS2);
  - the class-C effect on EPH, EPV, sats and dropouts vs 11-08;
  - the heading check (`nav_heading_check.py`) and alignment fit (`nav_accel_fit.py`), including the R6b in-flight checks;
  - the attitude check against the GPS specific-force reference at the chosen blackbox rate, with the rate's limit stated (T051);
  - the change set it flew (FR-027)
- [ ] T067 [P] [US7] If article 2 flew: `specs/045-m2-tracking/flight-analysis-YYYYMMDD-art2.md`, reported separately with its own
  configuration of record (FR-019, AS3)
- [ ] T068 [US7] The G2 verdict, in the analysis doc and MANIFEST § Gates. Update the active-feature block in `CLAUDE.md` and the
  memory pointer `project_045_state.md`. **End of the first implementation pass.** Next: the post-G2 `/speckit.plan` revision
  (phases 2–4), then `/speckit.tasks` again. The safety ladder (Phase 10) starts at its own pace

**Checkpoint G2**: US6, the safety ladder and the M2 phases start after T068.

---

## Phase 9: User Story 6: a span-local inertial frame on the xiao, in shadow (P2, after the re-fly: adopt, narrow or defer)

**Goal**: the decision, with numbers (FR-015), taken after the re-fly. If adopted, the shadow frame runs and logs on its own
flight, and never feeds the policy.
**Independent test**: replayed against a recorded flight's sensor stream, the frame's drift over 17 s is inside US3's budget, with
no discontinuity.

- [ ] T069 [US6] The decision (FR-015, AS1), **after G2** (operator 2026-11-12: the shadow IMU comes after the re-fly): adopt, narrow
  (e.g. INAV attitude + xiao position) or defer. Cite `drift-report.md` (T028), the budget, and the re-fly's analysis (T066), which
  shows how far more satellites and the INAV configuration got without it. If adopted, it flies as its own step, the only
  structural change on its flight. Its order against the safety ladder is the operator's call, recorded in `safety-ladder.md`.
  Record in research.md R4 / R10 § Decision and in MANIFEST § Gates. If deferred, decide whether the IMU driver (T075) still flies
  with raw logging, to collect xiao-IMU data; that needs its own log record, planned at the time. ⛔ T070–T079 run only for adopt
  or narrow
- [ ] T070 [US6] Test first. Write the new `tests/shadow_filter_tests.cc` against a portable `xiao/include/shadow_filter.h` (no
  Arduino):
  - static input ⇒ attitude holds within the bias term;
  - a constant-rate rotation at 416 Hz integrates to the analytic angle (coning);
  - a GPS-velocity update with a large innovation is gated ⇒ COASTING, with no state jump;
  - a handover from an INAV state 3–35 ms old is propagated through the gyro ring buffer to "now";
  - over a full replayed span, no step exceeds a continuity bound (no reset inside a span).

  Register it, and `tools/shadow_replay.cc` (T072), in `CMakeLists.txt` (⇒ clean `rebuild-perf.sh`, operator)
- [ ] T071 [US6] Implement `xiao/include/shadow_filter.h` (data-model § Span-local inertial frame):
  - an error-state filter: attitude quaternion (body → span-local NED), velocity, position, gyro bias; optional accel bias;
  - propagation at IMU rate;
  - 10 Hz GPS-velocity updates from the existing 64 B state message (position and velocity; no v2 needed);
  - innovation gating with a coast, and smooth corrections;
  - states HANDOVER / PROPAGATING / COASTING / FAULTED.

  T070 passes
- [ ] T072 [US6] Replay first (plan N4). `tools/shadow_replay.cc` runs the same filter on a `blackbox_decode` CSV + `*.gps.csv`,
  per span. `specs/045-m2-tracking/imu_shadow_replay.py` drives it over the 11-08 and 10-18 spans and adds § Shadow replay to
  `drift-report.md`: attitude drift against the GPS specific-force reference, and the discontinuity count (must be 0). ⚠️ The
  blackbox runs at 240 Hz, below the filter's design rate; state that limit in the report
- [ ] T073 [US6] Test first, the log change. In `tests/flightlog_roundtrip_tests.cc`: `kFormatVersion == 7`, where v7 = v6 plus a
  `ShadowStateRecord` (type `0x07`, 31 B) and nothing else; it round-trips; `recordSize(0x07) == 31`; a v6 header is refused. In
  `CMakeLists.txt`, rename the decoder fixture `flightlog_v6.bin` → `flightlog_v7.bin` and the fixture names (~lines 407–419)
- [ ] T074 [US6] Log v7 on every consumer, in one change (Constitution III / V):
  - `xiao/include/flight_log_format.h`: version 7, `kShadowState = 0x07` + the packed record, the budget assert for (148 + 31) B
    per tick;
  - `xiao/src/flight_log.cpp`;
  - `src/analytics/flightlog_decode.py`: accepts v7 only, a v5 / v6 file fails naming `043-final` / `044-final`, shadow rows go
    to a separate CSV through `--shadow-out`;
  - `xiao/web/flight_logger.html` (after US2's T022);
  - `tools/renderer.cc`;
  - `specs/044-m1-craft-history/bench_join.py`, `specs/044-m1-craft-history/flight_gates.py` and
    `specs/045-m2-tracking/nav_span_scan.py`.

  T073 passes
- [ ] T075 [US6] The xiao IMU driver, `xiao/src/imu_local.cpp` + `xiao/include/imu_local.h`:
  - LSM6DS3TR-C on Wire1: SCL P0.27, SDA P0.07, power-enable P1.08 driven high, INT1 P0.11;
  - a 416 Hz FIFO (208 Hz fallback) with a watermark, drained in small batches on every `loop()` pass and inside the MSP wait
    loop;
  - a FIFO overrun or an I2C error ⇒ FAULTED (logged, never feeds the NN)
- [ ] T076 [US6] Engage handover and logging. In `xiao/src/msplink.cpp`, initialise the filter once per span from INAV's state at
  engage. In `xiao/src/flight_log.cpp`, write one `ShadowStateRecord` after each TickRecord while engaged. Nothing in the NN path
  reads it
- [ ] T077 [US6] (operator) xiao IMU calibration (FR-007a, quickstart § 9): 6 static faces (mount rotation, accel offset and
  scale), 10 turns per axis (gyro scale), 60 s static (bias), and the temperature sensitivity. Commit it as
  `xiao/include/imu_local_cal.h`
- [ ] T078 [US6] (operator) Bench gate F3: 60 s static and 10 slow turns per axis on the shadow frame. Over a 5 min engaged bench
  run: **0 tick overruns**, and the tick p95 against the 21.6 ms baseline. Record pio RAM and flash in MANIFEST. Then fly it on
  its own flight. Analyse the shadow frame against INAV and the GPS specific-force reference (SC-003a), in
  `specs/045-m2-tracking/flight-analysis-YYYYMMDD.md` for that flight
- [ ] T079 [US6] AS4 / N5. Record in `specs/BACKLOG.md` that sim parity for the companion estimator (N5) gates any training run
  whose policy flies on it. Phase 4 runs on perfect navigation state (spec clarify), so N5 is not built in this pass

**Checkpoint**: US6 flown in shadow on its own flight, or deferred with numbers.

---

## Phase 10: User Story 7, part C: the safety ladder (after the re-fly; FR-015a, FR-015b; one change per step)

**Rule** (spec § Clarifications 2026-11-12): each step is one structural change. Write its tests first, implement it, run the build
gate, then repeat that step's B / H items from T056 ×3 as the after record. Then fly a happy-path g800 flight with that step alone,
and record a note in `safety-ladder.md` and MANIFEST § Safety ladder: the happy path is unchanged and the fault behaviour is as
designed. Only then the next step. The order below is the proposal. `safety-ladder.md` (T060) holds the operator's order.
Fault-path-only steps come first; steps that change every disengage, the MSP path or INAV come later.

**Shared files**: `xiao/src/msplink.cpp` (T048, T076, T081, T084, T087, T089, T093) and the new `tests/xiao_safety_tests.cc`
(T080, T083, T086, T093) are edited one step at a time.

### L1: mode bits by permanent id (B-3; also gives B-5 through the real FAILSAFE bit, with no state-message change)

- [ ] T080 [US7] Test first. Write a pure `xiao/include/box_map.h` that maps permanent ids (ARM, FAILSAFE, NAV RTH, NAV POSHOLD, NAV
  ALTHOLD; ids from `~/inav/src/main/fc/fc_msp_box.c`) to bit positions from an `MSP_BOXIDS` reply. Test it in the new
  `tests/xiao_safety_tests.cc` with the article 1 reply captured on the bench (ARM = bit 0, FAILSAFE = bit 29, ALTHOLD = bit 19)
  as `tests/fixtures/boxids_art1.bin`. A reply without ARM or FAILSAFE ⇒ the map is invalid. Register `xiao_safety_tests` in
  `CMakeLists.txt` (⇒ clean `rebuild-perf.sh`, operator)
- [ ] T081 [US7] Implement it in `xiao/src/msplink.cpp` (+ `xiao/src/MSP.cpp`). Request `MSP_BOXIDS` at boot. If the map is
  invalid, refuse engagement and say so on the console. Replace the fixed `MSP_MODE_ARM` / `MSP_MODE_FAILSAFE` bits
  (`xiao/include/MSP.h:77,96`) with the mapped bits, so the failsafe stop watches the real FAILSAFE bit, not NAV ALTHOLD
- [ ] T082 [US7] (operator) After record (B3, H1: the xiao now stops on a real failsafe and not on ALTHOLD), then the L1 flight,
  then the note

### L2: no resume after an in-flight xiao reboot (B-2a)

- [ ] T083 [US7] Test first, in `tests/xiao_safety_tests.cc`. Write a pure state machine `xiao/include/engage_latch.h`. If the
  first valid state after boot reports armed, this is an in-flight reboot: engagement is blocked until a disarm → arm cycle is
  seen, whatever RC9 does. A cold boot while disarmed is normal
- [ ] T084 [US7] Implement it in `xiao/src/msplink.cpp` (~lines 761–781) and `xiao/src/main.cpp`. Report the reboot with RESETREAS
  and the fault breadcrumb on the console, and in the log through the existing event record if `flightlog_decode.py` accepts a new
  event code without a layout change (otherwise it is a format change for its own step, Constitution V)
- [ ] T085 [US7] (operator) After record (H3: an xiao reset with RC9 high does not re-engage), then the L2 flight, then the note.
  Update `pilot-brief.md`

### L3: bounded run on bad output (B-4, first half)

- [ ] T086 [US7] Test first, in `tests/xiao_safety_tests.cc`: an NN-output validity check (NaN or out of range ⇒ invalid)
- [ ] T087 [US7] In `xiao/src/msplink.cpp`: fix the `flight_path.empty()` early return (~line 409) that skips the 60 s run
  timeout, and stop on invalid output with a new reason `kReasonNnOutputInvalid` (decoders updated in the same change)
- [ ] T088 [US7] (operator) After record, then the L3 flight, then the note

### L4: neutral, motor-off frames on every stop (B-1); this changes every disengage

- [ ] T089 [US7] In `stopAutoc` (`xiao/src/msplink.cpp` ~line 181), on every stop send zero-rate (roll = pitch = yaw = 1500),
  **minimum-throttle** (1000) `MSP_SET_RAW_RC` frames for ≥ 3 ticks (150 ms) so at least one lands, then go quiet. INAV then holds
  that neutral command, with the motor off through `motorstop_on_low`, until its override times out and hands the pilot's
  channels back. RC9 low needs none. Surface behaviour after the hold stays as decided in FR-015b (2) and T057. Update
  `pilot-brief.md` **before** the flight: the pilot will feel this on every span end
- [ ] T090 [US7] (operator) After record (B4, H7: the hold is zero-rate and motor-off), then the L4 flight with the pilot briefed,
  then the note

### L5: the companion's MSP traffic (R-3), in small steps

- [ ] T091 [US7] Measure first, without changing code. Audit every MSP send path in `xiao/src/msplink.cpp`, `xiao/src/MSP.cpp` and
  `xiao/src/controller.cpp`: commands, rates, the outstanding requests, whether any settings / config / reboot command or non-MSP
  byte can ever be sent. Measure it on the bench (B10: request rate, outstanding requests, bytes on the FC UART). Record it against
  the R-3 caps in research.md R17. If today's traffic is already within the caps with no path to exceed them, record that and close
  L5 here
- [ ] T092 [US7] Only if T091 finds a path over a cap: a guard on **that one path**, test first in `tests/xiao_safety_tests.cc`,
  then its after record, flight and note. Never a blanket rewrite of the MSP path; each further path is its own step

### L6: control-loop watchdog (B-4, second half)

- [ ] T093 [US7] Test first, in `tests/xiao_safety_tests.cc`: a pure stall detector (no completed 20 Hz tick for 1 s while engaged
  ⇒ trip). Then implement it in `xiao/src/msplink.cpp` / `xiao/src/main.cpp`: an mbed `Ticker` runs the check, and a trip writes a
  breadcrumb through `xiao/include/fault_guard.h` and calls `NVIC_SystemReset()`. L2's latch then blocks a re-engage. The 30 s
  board watchdog stays. Add a bench-only stall hook behind a build flag
- [ ] T094 [US7] (operator) After record (a forced stall resets the xiao and it does not re-engage), then the L6 flight, then the
  note

### L7: INAV failsafe configuration, once the debate settles (one setting per step)

- [ ] T095 [US7] (operator) For each decided setting (e.g. `failsafe_delay` 10 → 5; DROP vs LAND), one per step: bench it in RAM
  against T056 / T058 (H1, H2), save it with `inav_cli_apply.py` and read it back, add it to `config-changeset-art1.md` and
  `xiao/inav-hb1.cfg`, update `pilot-brief.md`, then fly that step and write the note. DROP stays until a tested alternative is
  adopted (FR-015b). Any change to a re-arm path (LAND's return on stick motion, the transmitter-side re-arm R-6, a shorter
  `failsafe_delay`) is bench-tested in the same step before it flies, timed and with its pilot action count against T058 (FR-015b's
  "designed and bench-tested" re-arm path)

### L8, L9: INAV fork changes, each its own step

- [ ] T096 [US7] L8 (R-7) in `~/inav` branch `autoc`: never evaluate non-MSP bytes (`reboot_character`, CLI `#`) on the
  companion's UART, even while disarmed (`src/main/fc/fc_tasks.c:113`; `src/main/msp/msp_serial.c` ~lines 425–447 and 478). Add a
  per-port setting in `src/main/fc/settings.yaml`, set to the xiao's UART (from the A6 `serial` capture). The USB VCP keeps today's
  behaviour. Build `MATEKF722MINI`, `HGLRCF722LITE`, `MAMBAF722_2022A`. (operator) Flash (GPS disconnected), re-apply the change set
  and read it back, run the bench check (disarmed: a stray `R` on the companion UART does nothing; USB CLI still works), then fly
  and write the note
- [ ] T097 [US7] L9 (R-4) in `~/inav`, a flight-controller watchdog:
  - enable `HAL_IWDG_MODULE_ENABLED` (`src/main/target/stm32f7xx_hal_conf.h:67`);
  - start the IWDG at the end of init (`src/main/fc/fc_init.c`) and refresh it from the scheduler's main loop
    (`src/main/scheduler/scheduler.c`), never from an ISR;
  - measure the worst-case blocking time of `save` (config flash sector erase) and of blackbox erase on the bench first. Either
    refresh around them or choose a timeout above it;
  - report an IWDG reset reason at boot (`status`);
  - add a bench-only CLI hang command behind a build flag.

  Build the three targets and the bench-hook variant
- [ ] T098 [US7] (operator) L9 after record (H4: a forced hang resets with the motor off; arming blockers after the reset), then
  flash the flight build, fly, and write the note
- [ ] T099 [US7] Close the ladder: `fault-timelines.md` gets the after column for every step flown, `pilot-brief.md` matches the
  firmware and configuration of record, and MANIFEST § Safety ladder lists each step with its flight

---

## Phase 11: User Story 8: camera model and variations reflect the field measurements (P2, phase 2) ⛔ after G2, readiness level

- [ ] T100 [US8] Test first. Update `tests/camera_variation_tests.cc` / `tests/beacon_projection_tests.cc` so the projection reads
  one camera definition. Consolidate `CameraDegPerPixel` (five definitions today, research R12) into
  `include/autoc/eval/camera_config.h`
- [ ] T101 [US8] The camera parameter record, `specs/045-m2-tracking/camera-record.md` (data-model). Every simulator camera value
  and every 040 variation (boresight σ 4°, roll σ 4°, mount translation σ 2 / 4 / 1.2 mm, wing thickness 0.8 mm, ambient, the
  ideal-projection lens model), each with source, class (M / D / A / provisional), fate (kept / updated / added / deferred) and
  phase (4 geometry / 5 signal), from `specs/042-camera-physics/README.md` and `specs/031-beacon-camera/bench-journal.md`. None of
  the previous lens's values remain (AS1)
- [ ] T102 [US8] Land the field-measured lens values in `include/autoc/eval/camera_config.h`, `autoc-tracker.ini` and
  `autoc-eval-tracker.ini`. Mark unmeasured values provisional, with their source (AS4)
- [ ] T103 [P] [US8] Record the signal measurements, with sources, for phase 5 (FR-020). Write the model gap list for the hardware
  side in `specs/045-m2-tracking/camera-gaps.md`. The R12 gaps: 100 m range vs locks at 4.5–12.5 m, the ambient knee, the 1.56 mm
  lens, warm relock, rotation-rate lock loss, the filter FWHM, prop attenuation computed but unused
- [ ] T104 [US8] Replay a recorded field geometry through the simulator and compare it with the recorded detections, within the
  measurement's stated uncertainty (US8 independent test). G3 in MANIFEST

## Phase 12: User Story 9: perfect-signal M2 sim with usable inputs (P2, phase 3) ⛔ after G2

- [ ] T105 [US9] Test first. In `tests/signal_model_tests.cc` / `tests/acquisition_state_tests.cc`: in PERFECT mode every visible
  beacon is TRACKING, q = 9, no CDMA penalty, CEP 0.02; FOV, behind-camera, obstruction and shared-element still apply (AS2); the
  emission-darkness gate and the 100 m cutoff are off. In `tests/contract_tracker_config_tests.cc`: a missing `TrackerSignalMode`
  fails at startup
- [ ] T106 [US9] Implement it (research R13):
  - a `TrackerSignalMode = PERFECT | REALISTIC` X-macro in `include/autoc/util/config.h`, with no default;
  - a `TickRuleConfig` field, and the branch in the `refine` lambda in `src/eval/tracker_tick_rule.cc` (~lines 82–118);
  - the signal gates at `camera_projection.cc:128` and `:139` skipped in PERFECT;
  - the WorkerInit field (`src/autoc.cc`, `crrcsim/.../crrcsim_tracker_helper.cpp`) and the startup log line;
  - the mode recorded in the as-run ini, **not in the dmp**. `src/eval/source_dmp_loader.cc` deserializes the whole `EvalResults`
    with cereal, so a schema change would orphan the `retain=keep` 044-t1 source that T107 points at (FR-026). Any later dmp schema
    change needs a source-extraction task first;
  - new sim code uses `gp_scalar`, with `// raw-ok:` annotations at byte-format boundaries (Constitution VI)
- [ ] T107 [US9] Repoint `autoc-tracker.ini:39` and `autoc-eval-tracker.ini:62` to 044-t1 g800
  (`autoc-9223370244339188361-2026-10-20T16:59:47.446Z/gen9200.dmp.zst`). Check the path × wind table matches 1:1 under
  `TrackerChaseUseSourceScenarioSeed = 1`. The 041 hand-over items (predictor head, 7 ↔ 3 topology, innovation channels) are
  decided in the post-G2 plan
- [ ] T108 [P] [US9] `specs/045-m2-tracking/input_domain.py` (+ a fixture test): spread, bounds and fraction at a bound for every
  input, from `dmp-dump --csv-only` (044-t1), `flightlog_decode.py -o` (11-08) and the smoke. Change a constant only for a
  saturated or dead input (FR-022). If a shared M1 input changes, re-evaluate the M1 source and g800 before either is used (spec
  edge case)
- [ ] T109 [US9] Create `autoc-basic-m2.ini`, the smoke (PERFECT, perfect navigation state)
- [ ] T110 [US9] (operator) Clean `rebuild-perf.sh` and the regression gate (FR-025, Constitution IX). Then `scripts/train.sh
  autoc-basic-m2.ini logs/autoc-045-t1-m2-smoke.log`. G4 = SC-009: no crash or schema failure, the log states PERFECT, and every
  tracker input is invested

## Phase 13: User Story 10: simulation-only M2 run (P2, phase 4) ⛔ after G4

- [ ] T111 [US10] (operator) `scripts/train.sh autoc-tracker.ini logs/autoc-045-t2-m2-perfect.log` after the pre-run build gate.
  It is never flown (FR-023)
- [ ] T112 [US10] The fixed-evaluation report in `specs/045-m2-tracking/outcome.md`: tracking occupancy, per-step error, reacquire
  time and crash rate, on the 040/041 definitions; the signal and navigation-state caveats (FR-024, AS2); each provisional camera
  value named (SC-008, SC-010). Pin the run (Constitution VIII: it is phase 5's comparison baseline, FR-024): tag its dmps
  `retain=keep` in `autoc-m2`, sample-verify gen 1, the evaluated gen and the last gen, and record the S3 prefix in `outcome.md`

US11 (phase 5, realistic signal) is outside this feature's passes. It starts from T103's record, with T112's run as its
comparison point.

---

## Phase 14: Polish and cross-cutting

- [ ] T113 (operator) SC-001 at month end: `USW2-DataTransfer-Out-Bytes` < 100 GB. Record in MANIFEST
- [ ] T114 [P] `specs/BACKLOG.md` (Constitution X):
  - the fork estimator fixes (reset smoothing on reacquire; the inertia-compensation fallback bug at `imu.c:687,694`);
  - R-5, the in-flight restart path;
  - B-2b (skip the 8 s boot delay without USB) and B-2c (a short watchdog from the first arm), not selected;
  - H5 MSP flood, once a hard power cycle exists;
  - the DROP → RTH-with-climb revisit (B-6);
  - GPS mast / EMI mitigations, if D2 / D4 show coupling;
  - soft-iron calibration
- [ ] T115 [P] Fold corrections back into the docs:
  - research.md: R2 (measured cause), R4 (datasheet numbers, the G1 decision), R5 (the 943.52 explanation, the pinned EPH cap),
    R17 (the L5 measurement, the ANGLE trace);
  - quickstart.md, wherever a procedure changed in practice;
  - bench-checklist.md, items ticked with links
- [ ] T116 Validate `specs/045-m2-tracking/quickstart.md`: every procedure run in this pass links its result (bench-results,
  change set or analysis doc)

---

## Dependencies and execution order

### Phase dependencies

```
Setup (T001–T004)
 ├─▶ US1 dmp cache (T005–T013)       FIRST, standalone: read-through + mini run A, then write-through + mini run B
 ├─▶ US2 BLE (T014–T023)             measure (T017) before any fix; ground-only paths
 ├─▶ US3 drift report (T024–T029)    offline; needs T003's CSVs
 ├─▶ US4 config + role (T030–T050)   bench and outdoor sessions; T033 takes T029's pinned values
 ├─▶ US5 blackbox side (T051–T055)   T053 builds on T026 and T029; no firmware change
 └─▶ US7-A safety prep (T056–T060)   bench before record + documents; no behaviour change
G1 (T061) ◀── US3, US4, US5, US7-A, and US2 for the field download
 └─▶ US7-B re-fly (T062–T067) ─▶ G2 (T068)  ⛔ end of the first pass
        ├─▶ US6 shadow IMU (T069–T079)       decided on T028 + T066; if adopted, built and flown as its own step
        ├─▶ US7-C safety ladder (T080–T099)  one step per flight; order vs US6 in safety-ladder.md
        └─▶ post-G2 sim work: US8 (G3) ─▶ US9 (G4) ─▶ US10 ─▶ outcome   (no flights; runs alongside the ladder)
```

### Within the stories

- Tests before implementation in every code group (T005 → T006; T010 → T011; T018 → T019; T052 → T053; T070 → T071; T073 → T074;
  T080 → T081; T083 → T084; T086 → T087; T093).
- `xiao/src/msplink.cpp`: T048, T076, T081, T084, T087, T089, T093, one at a time in ID order.
- `xiao/web/flight_logger.html`: T015 → T022 → T074 (only if US6 is adopted).
- `research.md` (T027 R4, T036 R5, T057 R17, T069 R4 / R10, T091 R17, T115) and `specs/BACKLOG.md` (T047, T055, T079, T114): each
  task edits its own section, but merge them one at a time. Their [P] markers mean parallel *work*, not parallel *commits*.
- CMake edits: T018, T070 + T073 (US6), T080. Each one ⇒ the operator's clean `rebuild-perf.sh` before the next
  determinism-affecting run. One clean rebuild covers every edit made before it.
- Calibration order on article 1: T038 (compass alignment) before T044 (compass calibration); T035 (accel) before T036 (gravity
  reference).
- **What flies when.** The re-fly carries only the US4 configuration, US5's blackbox settings and US2's ground-only firmware; the
  xiao is otherwise left alone. After it, US6 (if adopted) and each ladder step each get their own flight.

### Bench sessions (operator), grouped by setup class

| session | setup | tasks |
|---|---|---|
| 1 | [USB], level reference | T034, T035, T036, T037, T038, T039 |
| 2 | [SERVO], motor disconnected | T056 (before record, failsafe candidates in RAM) |
| 3 | [PROP], restrained | T040, T041, T042, T051 (blackbox rate under motor) |
| 4 | [OUT] | T043, T044, T054 |
| 5 | flying field | T063–T065 (the re-fly) |
| US6 flight, if adopted | [USB] + a flight | T077, T078 |
| per ladder step | [SERVO] + a flight | T082, T085, T088, T090, T092, T094, T095, T096, T098 |

## Parallel opportunities

```text
# Desk work that can run together once Setup is done:
T005–T008  US1 read-through           (src/util/s3_run_selector.cc, tests/s3_run_selector_tests.cc)
T014–T016  US2 instrumentation + dump (xiao/src/bluetooth.cpp, xiao/web/flight_logger.html, xiao_serial_dump.py)
T024–T027  US3 analysis scripts       (span_attitude.py, nav_dead_reckon.py, nav_span_scan.py, research.md R4)
T030–T032  US4 tooling                (cal_verify.py, mount_survey.py, msp_bench_read.py)
T057       ANGLE trace                (research.md R17)

# After T026 and T029: T052–T053 (US5 flags) while the operator runs bench session 1
# After G2: the safety ladder (operator + flights) alongside US8 → US9 → US10 (desk + training box)
```

## Implementation strategy

### First: the dmp cache, out of the way

1. Setup (T001–T004).
2. **US1** read-through (T005–T009), then write-through (T010–T013). Each is a small change, checked with a 10-gen mini run, and
   merged on its own.

### Then the platform clean-up, to the re-fly

3. **US2** (T014–T023): measure, then fix. The wired dump comes with it.
4. Phase 0b in parallel: desk work (US3, US5's tool, the safety documents) alongside the operator's bench sessions (US4
   calibration and configuration, T056's before record).
5. G1 (T061), then the re-fly: g800 on hardened all-attitude flight (improved alignment, calibration, INAV sampling and tuning,
   more satellites), a better blackbox and the improved field download, with **no structural safety change and no shadow IMU**
   (T062–T067). Then G2 (T068). **Stop and re-plan** phases 2–4 with the flight data in hand.

### After G2

6. The shadow IMU (T069–T079), decided with the re-fly's data and flown as its own step if adopted.
7. The safety ladder, one change per flight, at the pace the flying allows (T080–T099).
8. Dusting off M2 training, alongside it: US8 → G3 → US9 → G4 → US10 → `outcome.md`. Regenerate these tasks after the post-G2
   plan revision.

## Notes

- [P] = different files, no dependency on an incomplete task. Operator bench tasks on one article are never [P] with each other.
- Commit after each task or logical group. The operator pushes.
- Every FC setting counts as applied only after a read-back following a reboot (FR-009).
- One structural change per flight. A step that fails its after record or its flight is reverted before the next one starts.
