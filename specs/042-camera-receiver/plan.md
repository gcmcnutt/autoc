# Implementation Plan: 042 — camera receiver (a measured, robust beacon tracker)

**Branch**: `042-camera-receiver` | **Date**: 2026-08-19 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/042-camera-receiver/spec.md`

## Summary

Turn the 031 prototype — a **static-beacon** correlator that loses all processing gain above ~1–2 °/s —
into a receiver that tracks beacons crossing the field at flight-realistic rates, emits 20 Hz fixes with a
next-tick prediction, and **publishes a measured envelope** rather than an anecdote.

Technical approach: a bank of **movable multi-scale correlators**, each an alpha-beta centering loop (the
spatial twin of the DPLL), with **decision-directed chip-level lock re-affirmation** at ~121 Hz. Written as
a **zero-dependency C11 core** behind a thin C++17 libcamera shell, so the identical code runs live on the
Pi, in replay on the dev box, and cross-compiled from WSL2 for field updates. Truth comes from an
**offline oracle** over recorded raw plus **signal injection** into real backgrounds — no simulator.

Scope of record: **042 = A · B · C · E1 · D** (spec §13). Photometry, daylight, range, 453 fps and the
xiao/AHRS transport are 043.

## Technical Context

**Language/Version**: **C11** for `core/` (zero-dependency, no allocation after init, integer hot paths);
**C++17** for `io/` + `app/` only, where the libcamera API requires it. Python 3.11 for the acquisition
prototype and analysis/plot scripts.
**Primary Dependencies**: libcamera (live capture, per-frame metadata, request-level control); inih
(runtime config, already a repo dependency); GoogleTest (tests, already a repo dependency). **`core/` has
none** — that is a load-bearing property, not an aesthetic one (§16.1).
**Storage**: raw frame recordings — continuous full raw on the flight host (Pi 5 + NVMe, 116 MB/s of ~450
available); bounded RAM ring with triggered dump on the 3A+ bench host. Versioned binary 20 Hz record
stream. No database.
**Testing**: GoogleTest against `core/` with no hardware; **scalar-vs-NEON equivalence tests per kernel**;
**golden-vector replay tests** (fixed clip in → byte-compared record stream out); a tracker regression
entry point alongside the existing `firmware/beacon-decoder-stepfpga/host/regression.py` policy.
**Target Platform**: Pi 3A+ (Trixie arm64, 4×Cortex-A53, 512 MB) as the 042 bench/dev host; Pi 5 + NVMe as
the flight design point; aarch64 dev box (20 cores, ASIMD) for replay/oracle/tests; **WSL2 x86_64 →
aarch64 cross-compile for field updates**.
**Project Type**: real-time embedded-Linux signal-processing daemon + offline analysis tooling.
**Performance Goals**: 20 Hz fixes; front end ≤1 frame (4 ms at 250 fps) hard real-time; **delivery
deadline — the record predicting tick N on the wire ≥5 ms before tick N, miss rate <0.1 %** (§11.1);
16 tracker instances; cold full-field acquire threaded off the capture path.
**Constraints**: **bit-exact replay parity with live** (no wall-clock in `core/`, integer correlator paths,
virtualised acquisition scheduler); 512 MB RAM on the bench host; ~1 A from a 3S/4 Ah pack for the flight
host (~8 % of capacity over a 20 min sortie); read-only root + data partition so a brownout cannot corrupt;
field iteration must not require a compiler.
**Scale/Scope**: 640×400 @ 250 fps = 64 Mpx/s front end; 2 Gold-31 codes; 16 correlator slots; ~123 MMAC
per cold full-field acquire pass.

## Constitution Check

*Constitution v1.8.0. Several principles are scoped to the sim/training pipeline in `~/autoc` and do not
reach this feature; those are marked N/A with the reason.*

| Principle | Status | How this plan satisfies it |
|---|---|---|
| **I. Testing-First** | **GATE — binding** | Every `core/` kernel lands with (a) a unit test, (b) a scalar-vs-NEON equivalence test, before the NEON path is written. Golden-vector replay tests gate the integrated tracker. **Exemption invoked, narrowly**: the Python acquisition prototype (§Phase A) is an explicit research spike and ships no tests; nothing from it is promoted to mainline — the C implementation is written test-first against clips the prototype identified. |
| **II. Build Stability** | **GATE — binding** | New targets build under the top-level CMake; `core/` + tools + tests build on the dev box in CI-equivalent form, so a broken commit is caught without hardware. The libcamera app cannot build on the dev box (no libcamera) — mitigated by keeping `app/` thin and all logic in `core/`. |
| **III. No Compatibility Shims** | **GATE — binding** | `beacon_track.py` is **not** wrapped or preserved as a fallback. It is the reference for golden vectors during 042-B, then deleted when the C tracker passes them. The `rpicam-raw` pipe path survives only as the replay *source*, not as a duplicate live path. |
| **IV. Unified Build** | **GATE — binding** | All new targets declared in the top-level `CMakeLists.txt`; no second build system. Toolchain files for the WSL2 cross path are additive (`cmake/aarch64-linux-gnu.cmake`), not a parallel build. CMakeLists changes drive a clean rebuild per the principle. |
| **V. Versioned Persistence Artifacts** | **GATE — binding, and directly load-bearing** | Two new persistence formats — the **20 Hz record stream** and the **raw recording container** — both carry an explicit version field at a stable, parseable offset; readers attempt version detection and **fail loudly** naming artifact and reader version. Recordings additionally stamp **build ID + config hash** (§16.2). |
| **VI. Type-Domain Discipline** | **N/A with one carry-over** | `gp_scalar`/`gp_vec3`/`gp_fitness` are eval-pipeline aliases in `~/autoc`; this feature has no eval pipeline and `core/` is C11. **Carried over in spirit**: the correlator hot paths are deliberately fixed-point `int16`/`int32` with the unit documented at each declaration, and any float that survives is annotated with why. |
| **VII. No Silent Fallback Defaults** | **GATE — binding, adapted to C** | No hidden defaults for config-supplied values. Config structs are populated by an explicit loader that **errors on a missing key** rather than defaulting; sentinel/counter fields that legitimately zero-init are annotated `/* default-ok: <reason> */`. This is the 032 `cepGateThreshold` failure mode and it is exactly reachable here (thresholds, chip rates, ROI sizes). |
| **VIII. Training-Artifact Lifecycle** | **N/A** | No S3 training artifacts in this feature. *(Flight recordings are large and will need their own retention rule — flagged for 043, not invented here.)* |
| **IX. Detached Training Launch** | **N/A** | No training runs. |
| **X. Single Ordered Backlog** | **Satisfied** | 042 spin-offs already filed to `specs/BACKLOG.md` (ToF → M3/M4, multipath research). |

**Gate result: PASS.** No violations requiring justification; Complexity Tracking is empty.

**One risk the gates surface rather than block**: Principle II's build-stability guarantee is weaker here
than in `~/autoc`, because the libcamera-dependent `app/` layer cannot compile on the dev box or in any
hardware-free CI. The mitigation *is* the architecture — `core/` holds all logic and builds everywhere, and
`app/` is wiring thin enough to be reviewed by eye. This is the strongest single argument for the
core/shell split and should not be eroded.

## Project Structure

### Documentation (this feature)

```text
specs/042-camera-receiver/
├── spec.md              # what & why (shaped + clarified 2026-08-17..19)
├── plan.md              # this file
├── research.md          # Phase 0 — decisions with rationale
├── data-model.md        # Phase 1 — entities, lifecycles, versioned formats
├── contracts/           # Phase 1 — record wire format, config schema, CLI, recording container
├── quickstart.md        # Phase 1 — bring-up on all three build paths
└── tasks.md             # Phase 2 — NOT created by /speckit.plan
```

### Source Code (repository root)

```text
firmware/beacon-receiver/
├── src/
│   ├── core/                 # C11. Zero dependencies. Builds on every path. All logic lives here.
│   │   ├── frame.h           #   FrameView {ptr, stride, w, h, t_us, exposure_us, gain_q8}
│   │   ├── reduce.c/.h       #   2x2 / 4x4 software sums          (scalar + NEON)
│   │   ├── hipass.c/.h       #   per-frame spatial high-pass      (scalar + NEON)
│   │   ├── corr.c/.h         #   templates; chip-domain reduction;
│   │   │                     #   corr_search()  vs  corr_track()   <- 1922 vs 31 MAC/hypothesis
│   │   ├── track.c/.h        #   alpha-beta centering loop, scale ladder, DPLL, lock_health
│   │   ├── bank.c/.h         #   candidate pool, lifecycle, guard+precision pairing
│   │   ├── acquire.c/.h      #   blink detect -> proto-track RANSAC -> multi-rate decode
│   │   ├── agc.c/.h          #   exposure / gain / integration-length controllers
│   │   ├── sched.c/.h        #   acquisition budget virtualisation (replay determinism)
│   │   ├── config.c/.h       #   loader; errors on missing keys (Principle VII)
│   │   └── record.h          #   versioned 20 Hz wire record (POD)
│   ├── io/                   # C++17 only where libcamera forces it
│   │   ├── src_libcamera.cc  #   live source
│   │   ├── src_replay.c      #   file source — same FrameSource vtable
│   │   ├── sink_record.c     #   raw recorder: bounded ring + pluggable sink
│   │   └── emit_json.c       #   JSON lines for beacon_display.py
│   └── app/
│       ├── beacon_trackd.cc  #   the daemon
│       └── beacon_record.c   #   recorder only — flies before the tracker (spec §7.1)
├── tools/
│   ├── oracle.c              # non-causal offline truth over a recording
│   ├── inject.c              # coded point source into real recorded background
│   └── score.c               # envelope scoring -> CSV (§3.1 / §3.2 / §11.1 metrics)
├── tests/                    # GoogleTest; no hardware
│   ├── unit/                 #   per-kernel behaviour
│   ├── equivalence/          #   scalar vs NEON, per kernel
│   └── golden/               #   fixed clip -> byte-compared record stream
├── cmake/aarch64-linux-gnu.cmake   # WSL2 cross toolchain file
└── pi/                       # existing 031 prototype tools (beacon_track.py retires per Principle III)
```

**Structure Decision**: a **core/shell split with the dependency boundary as the primary architectural
line**. `core/` is C11 with zero dependencies so that it (a) cross-compiles from WSL2 with nothing but the
distro cross-gcc, (b) builds and tests on the dev box with no camera, (c) is the *same* code live and in
replay, and (d) can be exercised by the oracle, injector and scorer as a library. Every other property this
plan needs — replay parity, hardware-free CI, field updates, bit-exact NEON verification — is downstream of
that one boundary. `io/` and `app/` exist to be thin.

## Complexity Tracking

*No constitutional violations. Section intentionally empty.*
