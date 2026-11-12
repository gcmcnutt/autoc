# Implementation Plan: 045 — M2 tracking: navigation integrity, a re-fly, then a perfect-signal M2 run

**Branch**: `045-m2-tracking` | **Date**: 2026-11-11 | **Spec**: [spec.md](spec.md)
**Input**: [spec.md](spec.md) (governs), [README.md](README.md) (derivation of record, incl. § Plan research scope), [research.md](research.md)

## Summary

The first implementation pass ends at the **re-fly gate (G2)**: fly 044-t1 g800 on a corrected, recorded INAV configuration, with
advisory estimator-fault flags in every tick and the companion's own inertial frame running in shadow. Before that come two front
tasks, the dmp cache and a BLE downloader that works. Then, after G2: field-measured camera geometry, a perfect-signal M2 simulator,
and a simulation-only "does M2 work at all" run.

Research changed the picture in five places:
- Un-aided gyro attitude cannot meet < 1° per span: the policy loops ~1000°+ net in pitch every span, so scale factor dominates.
  The companion frame must be **GNSS-velocity-aided**, and its corrections must be smooth (R4).
- The 11-08 freeze and reset are INAV code behaviour: a 1.5 s pull to a frozen GPS fix, an EPH cap, then a hard reset on
  reacquire. `inav_allow_dead_reckoning` played no part. No setting removes the reset (R5).
- The BLE download almost certainly runs at a **23-byte MTU** and silently truncates every chunk, in a strict stop-and-wait
  protocol with no recovery (R2).
- The MANUAL switch is on an **overridden channel**. The pilot escape is the RC9 override switch, not MANUAL (R8).
- The committed logs cannot show mount resonances; they are aliased at 240 Hz (R7).

## Technical Context

**Language/Version**: C++17 (autoc, crrcsim, tools), C (INAV 8.0 fork, branch `autoc`), C++ / Arduino-mbed (xiao, PlatformIO),
JavaScript (Web Bluetooth page `xiao/web/flight_logger.html`), Python 3.11 (analysis, `specs/045-m2-tracking/*.py`)
**Primary Dependencies**: Eigen, cereal, inih, GoogleTest, AWS SDK C++ (S3), CRRCSim LaRCSim FDM; ArduinoBLE 2.1.0 over mbed Cordio;
INAV MSP v2; numpy (analysis)
**Storage**: S3 per-mode buckets (archive of record) + **new local dmp cache** (`AUTOC_DMP_CACHE`); xiao QSPI flash log (**v6 → v7**);
INAV blackbox (SPI flash)
**Testing**: GoogleTest / ctest (autoc: cache, log v7 round-trip, perfect-signal switch, input-layout gate); PlatformIO build gate
(xiao); bench procedures for hardware ([quickstart.md](quickstart.md)); flight analysis scripts as acceptance
**Target Platform**: Linux training box (DGX Spark), xiao nRF52840 BLE Sense, MATEKF722MINI (article 1) and HGLRCF722LITE (article 2)
flight controllers, Android/desktop Chrome (Web Bluetooth)
**Project Type**: multi-surface embedded + simulation + training pipeline
**Performance Goals**: SC-001 zero self-written dmp GETs per report cycle; SC-002 ~300 kB log in < 60 s; shadow frame at 416 Hz
(208 Hz fallback) within ~3 % xiao CPU, without disturbing the 20 Hz tick (p95 21.6 ms, 0 overruns today)
**Constraints**: g800 must fly unchanged on the re-fly (no class A / B delay changes, FR-017); xiao log budget nearly full
(~47 KB slack in v6); INAV fork changes are limited to the state message before G2; training is detached via `scripts/train.sh`
(Constitution IX); the operator drives `rebuild-perf.sh` and the regression gate
**Scale/Scope**: 4 build surfaces (autoc/crrcsim, xiao, web page, INAV fork); 2 articles; 1 flight before G2; 1 M2 smoke and 1 M2
training run after G2

## Constitution Check

*GATE: checked before research, re-checked after design (below). No violations.*

| principle | applies to | status |
|---|---|---|
| I Testing-first | dmp cache (unit + integration with a temp dir), log v7 round-trip fixture, state-message decode test, perfect-signal switch test, the input-domain script on a fixture | PASS: tests named in the design |
| II Build stability | autoc `rebuild.sh`, xiao `pio run -e xiaoblesense_arduinocore_mbed`, INAV target builds for MATEKF722MINI + HGLRCF722LITE | PASS: gates per surface |
| III No compatibility shims | log v7 and state-message v2 are clean cuts: every consumer updated, old readers refuse new data (R9 consumer list) | PASS |
| IV Unified build | new test registrations and the v7 fixture touch `CMakeLists.txt` ⇒ clean `rebuild-perf.sh`, operator-driven | PASS (flagged in tasks) |
| V Versioned persistence | xiao log v6 → v7; the state message changes length (versioned by size check + a version byte); dmp schema **unchanged** (no cereal bump, per project practice) | PASS |
| VI Type-domain | new sim code (perfect-signal switch) uses gp_scalar; raw-ok annotations at byte-format boundaries | PASS |
| VII No silent fallback | `AUTOC_DMP_CACHE` unset ⇒ logged once, no cache; set but unwritable / corrupt ⇒ fail loud. The perfect-signal mode is an explicit ini key with no default and is logged at startup | PASS |
| VIII Artifact lifecycle | 044-t1 already `retain=keep` 800/800; cache eviction follows retain classes; the M2 run is tagged per policy | PASS |
| IX Detached training | the M2 smoke and run launch via `scripts/train.sh`; the pre-run build gate applies | PASS |
| X Single backlog | new items (fork estimator fixes, reduced-role build, MANUAL channel move) go to `specs/BACKLOG.md` if not done here | PASS |

## Project Structure

### Documentation (this feature)

```text
specs/045-m2-tracking/
├── spec.md            # governs
├── README.md          # derivation of record + plan research scope
├── plan.md            # this file
├── research.md        # R1–R15
├── data-model.md      # change set, fault flags, shadow record, log v7 budget, drift report, camera record, signal mode
├── quickstart.md      # bench + outdoor procedures, re-fly card, M2 smoke
├── contracts/
│   ├── msp-autoc-state-v2.md    # INAV → xiao state message (+ estimator fields)
│   ├── flight-log-v7.md         # xiao log v7 (fault flags, shadow record)
│   ├── ble-download-v2.md       # windowed, resumable download protocol
│   └── dmp-cache.md             # cache layout, env var, failure behaviour
├── checklists/requirements.md
├── nav_*.py           # N1 tools (existing)
└── (new) span_attitude.py, input_domain.py, imu_shadow_replay.py   # analysis scripts for this feature
```

### Source Code (repository root)

```text
src/util/s3_run_selector.cc          # F1: cache in s3GetDmpBlob / s3PutDmpBlob
tests/dmp_cache_tests.cc             # F1 (new)
xiao/src/bluetooth.cpp               # F2: instrumentation, MTU, windowed streaming, resume, inactivity timeout
xiao/web/flight_logger.html          # F2: instrumentation + copy log, windowed/resume client; v7 decoder
xiao/src/msplink.cpp                 # N3 flags, N4 engage handover, MANUAL comment fix, box-bit check
xiao/include/MSP.h                   # msp_autoc_state_t v2
xiao/src/imu_local.{cpp,h}           # N4 (new): LSM6DS3 FIFO driver + error-state filter (shadow)
xiao/include/flight_log_format.h     # v7: tick flags + estimator fields, ShadowStateRecord
xiao/src/flight_log.cpp
src/analytics/flightlog_decode.py    # v7
tools/renderer.cc                    # v7 reader
tests/flightlog_roundtrip_tests.cc   # v7 fixture
~/inav (branch autoc): src/main/fc/fc_msp.c   # N3: append estimator fields to MSP2_INAV_LOCAL_STATE
xiao/inav-hb1.cfg, xiao/inav-hb1-art2.cfg    # configuration change sets of record (N2)
# after G2 (phases 2–4):
src/eval/tracker_tick_rule.cc, include/autoc/eval/tracker_tick_rule.h   # perfect-signal branch
include/autoc/util/config.h, src/autoc.cc, crrcsim/.../crrcsim_tracker_helper.cpp  # ini key + WorkerInit + log
include/autoc/eval/camera_config.h, autoc-tracker.ini, autoc-eval-tracker.ini     # camera update; source repoint to 044-t1
autoc-basic-m2.ini (new)             # M2 smoke
```

**Structure Decision**: existing multi-surface layout. One new firmware module (`imu_local`) and one new test file. Everything
else edits in place.

## Design by phase

### 0a — front tasks

- **F1 dmp cache** — [contracts/dmp-cache.md](contracts/dmp-cache.md), research R1. Write-through after a successful S3 put;
  read-through with fill on miss; compressed bytes; tmp + rename; key includes the bucket; env var read once; eviction sweep script
  by retain class.
- **F2 BLE** — [contracts/ble-download-v2.md](contracts/ble-download-v2.md), research R2. **Step 1 is instrumentation only**, then
  one operator bench download with the field phone. Then MTU 247 + fail loud on short notify, windowed streaming with offset acks,
  resume by offset, inactivity timeout, connection-interval / PHY requests, MSP traffic paused while downloading on the ground.

### 0b — pre-tasks to G1

- **N1** (US3): the full drift report = the `nav_*` tools + `span_attitude.py` (gyro-only vs INAV) + a GPS-specific-force attitude
  check (the `nav_accel_fit.py` method per window) as the truth reference. Datasheet error terms for MPU6000 / ICM42688P /
  LSM6DS3TR-C fill the R4 budget table.
- **N2 + FR-007a/b + FR-008/8a** (US4): the class-C / calibration change set from research R5 (table), applied as **one recorded
  change set per article**, each setting read back after reboot. Bench and outdoor steps in quickstart.
- **N6** (FR-010): the reduced-role design written into research R8 (done); the pre-re-fly items are the MANUAL / escape
  correction, the box-bit check and the pilot brief.
- **N3 + B1** (US5): [contracts/msp-autoc-state-v2.md](contracts/msp-autoc-state-v2.md) +
  [contracts/flight-log-v7.md](contracts/flight-log-v7.md); blackbox MAG + NAV_ACC on. Bench-verified with the satellite-block test
  (outdoor).
- **N4** (US6, shadow): `imu_local` at 416 Hz FIFO; an error-state filter (attitude, velocity, position, gyro bias; GPS velocity
  update at 10 Hz from the state message; no resets inside a span; GPS innovation gating with a coast). Engage handover propagates
  INAV's state forward with a gyro ring buffer. Logged at 20 Hz in the ShadowStateRecord. **Replay-first**: the same filter code is
  built for desktop and run against recorded data (`imu_shadow_replay.py` drives it on the blackbox IMU) before it flies. Board
  orientation from a static + rotation calibration on the bench. **Adopt / narrow / defer is decided at G1** with N1's numbers.
  If deferred, the driver and logging may still fly to collect raw xiao-IMU data.
- **N5**: specified, not built before G2 (phase 4 runs on perfect navigation state, per clarify).

### 1 — re-fly (G2)

Quickstart § Re-fly card (research R11). Analysis: 11-08 tools + N3 flags + shadow-frame comparison + configuration effect.

### 2–4 — after G2 (readiness-level design; revised in a post-G2 plan update)

- Camera: consolidate `CameraDegPerPixel` (five definitions → one); field-measured lens values; the review of the 040 variations
  (R12 gap list) recorded as the camera parameter record.
- Perfect-signal mode: the `refine`-lambda branch (R13). Signal-class gates (emission darkness, 100 m cutoff) are off; geometry
  gates (FOV, behind-camera, obstruction, shared element) stay on. Navigation state is perfect.
- Input-domain check: `input_domain.py` on `dmp-dump --csv-only` (044-t1) + `flightlog_decode.py -o` (11-08) + the M2 smoke.
  Change constants only on saturation or dead inputs.
- Repoint `autoc-tracker.ini` / `autoc-eval-tracker.ini` to 044-t1 g800. Create `autoc-basic-m2.ini`. Smoke (G4), then the run.
  The 041 hand-over items (predictor head, 7 ↔ 3 topology, innovation channels) are decided in the post-G2 revision.

## Risks

| risk | effect | mitigation |
|---|---|---|
| Article 1's receiver can't do multi-GNSS | class C's main fix lost | firmware check / swap a receiver from stock (FR-008) |
| `init_gyro_cal = OFF` with a stale zero | gyro bias on the re-fly | re-record zero each session (quickstart) |
| Raising `inav_max_eph_epv` hides faults from INAV's own validity | RTH trusts a worse estimate | N3 logs EPH regardless; RTH is not the escape (R8); value pinned from N1 |
| Log v7 overflows the flash budget | lost ticks | data-model budget: fewer ticks per session + erase per session |
| Shadow IMU drain starves the 20 Hz tick | control-loop overrun | FIFO + small batches; measured overrun gate on the bench before flight |
| GPS_ONLY with poor EPV | worse Z | only if bench multi-GNSS EPV ≤ ~5 m; else stay on GPS |

## Complexity Tracking

No constitution violations to justify.

## Post-design Constitution re-check

Re-checked after data-model and contracts: log v7 and state v2 are clean cuts with all consumers listed (III, V); the cache's
failure modes are loud (VII); new CMake registrations are flagged for clean `rebuild-perf.sh` (IV). **PASS.**
