# Tasks: NN Training Signal Improvement

**Input**: Design documents from `/specs/014-nn-training-signal/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/

**Tests**: Contract tests are included per spec clarification ("contract tests first, before ripping things out").

**Organization**: Tasks follow the refactor-first implementation order from the spec clarifications. Foundational phases (GP removal, Boost removal, source reorg) are prerequisites. User story phases (sigma floor, curriculum, CMA-ES, etc.) follow.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

---

## Phase 1: Setup

**Purpose**: Dependency analysis and contract tests that define surviving behavior

- [X] T001 Run `nm ~/GP/lib/libgp.a | grep ' T '` and document all 103 exported symbols used by autoc in specs/014-nn-training-signal/analysis/libgp-symbols.md
- [X] T002 Scan all `#include "gp.h"` and `#include "gpconfig.h"` across 11 autoc files, document transitive usage in specs/014-nn-training-signal/analysis/gp-includes.md
- [X] T003 Scan all `#include <boost/` across autoc files, document per-file usage in specs/014-nn-training-signal/analysis/boost-includes.md
- [X] T004 [P] Write contract test for NN evaluator (sensor-in/control-out: 22 inputs → 3 outputs) in autoc/tests/contract_evaluator_tests.cc
- [X] T005 [P] Write contract test for config parsing (load autoc.ini, verify key types and defaults) in autoc/tests/contract_config_tests.cc
- [X] T006 [P] Write contract test for NN evolution loop (1 generation: init → evaluate → select → reproduce → verify fitness improves) in autoc/tests/contract_evolution_tests.cc
- [X] T006a [P] Write contract test for RPC transport (serialize EvalRequest/EvalResponse, round-trip, verify fields match — little-endian wire format for x86↔ARM portability) in autoc/tests/rpc_transport_tests.cc
- [X] T007 Update autoc/CMakeLists.txt to add new contract test targets and ensure tests build into build/tests/
- [X] T008 Verify all existing + new tests pass: `cd build && ctest --output-on-failure`

**Checkpoint**: Dependency surface fully documented, contract tests pass, safe to begin surgery

---

## Phase 2: Foundational — Strip GP Dependencies (FR-012, FR-014)

**Purpose**: Remove all GP code and libgp.a dependency. BLOCKS all subsequent work.

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### 2a: Vendor inih config parser

- [X] T009 Vendor inih library via CMake FetchContent (r58 tag)
- [X] T010 Add inih to autoc/CMakeLists.txt as a static library dependency
- [X] T011 Rewrite autoc/config_manager.h: unified AutocConfig struct replaces GPVariables + ExtraConfig — removed `#include "gp.h"` and `#include "gpconfig.h"`
- [X] T012 Rewrite autoc/config_manager.cc to use INIReader::GetInteger/GetReal/GetString — updated all callers across autoc.cc, nnextractor.cc, renderer.cc, gpextractor.cc, threadpool.h, pathgen.cc
- [X] T013 Verify contract_config_tests pass with new parser

### 2b: Replace PRNG

- [X] T014 [P] Create autoc/rng.h with std::mt19937 wrapper replacing GPrand()/GPsrand()/GPRandomPercent()
- [X] T015 Replace GPrand() calls in autoc/pathgen.cc, autoc.cc, variation_generator.h, nn_population.cc, nn_evaluator_portable.cc with rng.h
- [X] T016 Replace GPrandGaussian/GPrandDouble in gp_math_utils.h with rng.h delegates — removed `#include "gp.h"`

### 2c: Break GP inheritance and remove GP code

- [X] T017 Rewrite autoc/autoc.h: remove `#include "gp.h"`, remove MyGP/MyGene/MyPopulation classes, ExtraConfig, Operators enum — NN-only header with fitness defines and scenario types
- [X] T018 Rewrite autoc/autoc.cc: remove MyPopulation class, MyGP::evaluate/evalTask, GP evolution loop, bytecode evaluation mode, GP initialization — NN-only with runNNEvolution/runNNEvaluation
- [X] T019 Delete autoc/autoc-eval.cc (GP tree evaluation — entirely replaced by nn_evaluator_portable)
- [X] T020 [P] Delete autoc/gp_bytecode.h and autoc/gp_bytecode.cc (bytecode interpreter)
- [X] T021 [P] Delete autoc/gp_evaluator_portable.h and autoc/gp_evaluator_portable.cc (GP portable evaluator). Sensor math (executeGetDPhi, executeGetDTheta, getInterpolatedTargetPosition) extracted to new sensor_math.h/cc with fastAtan2 LUT preserved for embedded platform compatibility.
- [X] T022 Remove GP tree deserialization from autoc/minisim.cc — complete rewrite, NN-only, uses sensor_math.h for temporal history
- [X] T023 Remove GP tree deserialization from autoc/renderer.cc — remove `#include "gp.h"`, NN-only fitness extraction
- [X] T024 Remove `#include "gp.h"` from autoc/nnextractor.cc, remove dummy MyGP implementations
- [X] T025 Delete autoc/gpextractor.cc, bytecode2cpp.cc, gp_evaluator_desktop.h/cc, gp_evaluator_embedded.h/cc, gp_evaluator_tests.cc
- [X] T026 Strip all GP/NN modal branches — NN is the only path in autoc.cc and minisim.cc

### 2d: Sever CMake dependency on parent repo

- [X] T027 Update autoc/CMakeLists.txt: remove `include_directories(../include)`, `link_directories(../lib)`, remove `gp` from all target_link_libraries
- [X] T028 Remove references to deleted source files from autoc/CMakeLists.txt, add sensor_math.cc/h, remove autoc_tests (GP-only)
- [X] T029 Full rebuild: clean build succeeds with zero GP references
- [X] T030 Run all tests: `cd build && ctest --output-on-failure` — 8/8 tests pass
- [X] T030a Integration smoke test: run `./build/autoc` for 3 generations with NN config, verify data.dat and data.stc are produced and fitness values are present in console output

**Checkpoint**: autoc builds with zero dependency on libgp.a or parent GP repo. All `#include "gp.h"` removed.

---

## Phase 3: Foundational — Replace Boost (FR-013, phases C/D/E)

**Purpose**: Remove Boost dependency entirely

### 3a: Trivial Boost replacements

- [X] T031 [P] Replace boost::thread with std::thread in autoc/threadpool.h
- [X] T032 [P] Replace boost::mutex/unique_lock/condition_variable with std:: equivalents in autoc/threadpool.h and autoc/autoc.cc
- [X] T033 [P] Replace boost::format with sprintf or std::stringstream in autoc/minisim.h and autoc/aircraft_state.h
- [X] T034 [P] Replace boost::date_time with std::chrono in autoc/autoc.cc logging setup
- [X] T035 Replace boost::process with fork()/execv() in autoc/threadpool.h

### 3b: Medium Boost replacements

- [X] T036 Create autoc/socket_wrapper.h with POSIX TCP socket wrapper (~150 LOC) replacing boost::asio usage
- [X] T037 Replace boost::asio in autoc/threadpool.h (tcp::acceptor, io_service) with socket_wrapper.h
- [X] T038 Replace boost::asio in autoc/minisim.cc (tcp::resolver, connect) with socket_wrapper.h
- [X] T039 Replace boost::asio in autoc/minisim.h (read/write) with socket_wrapper.h
- [X] T040 Rewrite autoc/logger.h and autoc/logger.cc: replace boost::log with simple custom logger (~30 LOC, severity levels, file + stderr output)
- [X] T041 Replace boost::iostreams with std::stringstream in autoc/minisim.h and autoc/autoc.cc

### 3c: Boost serialization replacement + unified transport

- [X] T042 Define EvalRequest and EvalResponse structs with manual serialize/deserialize methods per contracts/rpc-transport.md in autoc/rpc_protocol.h
- [X] T043 Rewrite sendRPC/receiveRPC in autoc/minisim.h to use manual binary serialization instead of boost::archive
- [X] T044 Remove BOOST_SERIALIZATION_ACCESS, BOOST_CLASS_VERSION, and all boost::serialization friend classes from autoc/minisim.h, autoc/aircraft_state.h, autoc/gp_bytecode.h (if still present)
- [X] T045 Remove all `#include <boost/` directives from all autoc source files
- [X] T046 Remove Boost from find_package and target_link_libraries in autoc/CMakeLists.txt
- [X] T047 Full rebuild: `bash scripts/rebuild.sh` — verify zero Boost references remain
- [X] T048 Run all tests: `cd build && ctest --output-on-failure`
- [X] T048a Integration smoke test: run `./build/autoc` for 3 generations, verify RPC communication works (autoc ↔ minisim), data.dat produced, fitness values in output

**Checkpoint**: autoc builds with zero Boost dependency. RPC uses plain binary protocol.

---

## Phase 4: Foundational — Source Reorganization (Phase F)

**Purpose**: Clean C++ project layout enabling standalone repo extraction

- [X] T049 Create directory structure: autoc/include/autoc/{nn,eval,util}/, autoc/src/{nn,eval,util}/, autoc/tools/
- [X] T050 [P] `git mv` NN headers to include/autoc/nn/: nn_topology.h, nn_genome.h (from nn_population.h), nn_population.h, nn_forward.h, nn_serialization.h
- [X] T051 [P] `git mv` eval headers to include/autoc/eval/: nn_evaluator_portable.h, aircraft_state.h
- [X] T052 [P] `git mv` util headers to include/autoc/util/: config_manager.h (→ config.h), logger.h, s3_archive.h, socket_wrapper.h, rng.h
- [X] T053 [P] `git mv` NN source files to src/nn/
- [X] T054 [P] `git mv` eval source files to src/eval/
- [X] T055 [P] `git mv` util source files to src/util/
- [X] T056 [P] `git mv` tool executables to tools/: nnextractor.cc, nn2cpp.cc, renderer.cc
- [X] T057 [P] `git mv` test files to tests/
- [X] T058 Update all `#include` directives across all moved files to use `"autoc/nn/..."`, `"autoc/eval/..."`, `"autoc/util/..."` style
- [X] T059 Rewrite autoc/CMakeLists.txt for new directory structure: include paths, source file lists, add_subdirectory(tests), test output to build/tests/
- [X] T060 Full rebuild and test: `bash scripts/rebuild.sh && cd build && ctest --output-on-failure`
- [X] T060a Integration smoke test: run `./build/autoc` for 3 generations, verify everything still works after file moves (same output as T048a)

**Checkpoint**: Clean source layout, all tests pass, ready for cross-repo updates

---

## Phase 5: Foundational — Cross-Repo & Output Cleanup (Phases G/H)

**Purpose**: Update CRRCSim and xiao-gp for cereal/POSIX protocol, clean up run output

**Direction**: autoc is the main repo. crrcsim and xiao-gp will become git submodules of autoc once it is extracted from GP.

### 5a: CRRCSim integration

- [X] T061 Update CRRCSim inputdev_autoc RPC: replace Boost serialization with cereal, Boost.Asio with POSIX sockets (matching autoc socket_wrapper.h protocol)
- [X] T062 Remove GP dead code from CRRCSim inputdev_autoc: drop `gp.h`, `gp_bytecode.h`, `gp_evaluator_portable.h` includes; NN-only
- [X] T062a Replace `boost::circular_buffer` in CRRCSim inputdev_autoc with std::array ring buffer
- [X] T063 Build CRRCSim: `cd crrcsim/build && cmake .. && make`

### 5b: xiao-gp integration

- [X] T064 Verify xiao-gp export pipeline: `./build/nnextractor` → `./build/nn2cpp` → `cd ~/xiao-gp && pio run`
- [X] T065 Remove GP dead code from xiao-gp: `gp_evaluator_embedded.cc`, `gp_program_generated.cpp`, `gp_program.h`
- [X] T065a Update xiao-gp includes to match renamed autoc headers (types.h, math_utils.h, etc.)

### 5c: Output cleanup

- [ ] T066 Add OutputDir config key to autoc.ini (default: current directory) (deferred to backlog)
- [ ] T067 Implement auto-created run subdirectory in autoc.cc: create OutputDir/{timestamp}/ at startup, route data.dat, data.stc, logs to it (deferred to backlog)
- [ ] T068 Remove hardcoded `eval-` prefix from data file naming in autoc.cc (deferred to backlog)

### 5d: Verification

- [X] T069 Run all 3 repo builds and verify: autoc, CRRCSim, xiao-gp
- [X] T069a Integration smoke test: run `./build/autoc` for 3 generations with OutputDir set, verify run artifacts land in auto-created subdirectory
- [X] T069b Cross-repo integration test: run autoc with CRRCSim FDM worker, verify cereal RPC works end-to-end
- [X] T069c Profile cereal serialization cost in large-scale CRRCSim run (vs Boost baseline)

**Checkpoint**: All 3 repos build, cross-repo contracts aligned, run output goes to subdirectories

---

## Phases 6-14: MOVED TO 015-nn-training-improvements

All remaining user story work (sigma floor, curriculum, fitness decomposition, sep-CMA-ES,
per-timestep streaming, segment scoring, checkpoint/resume, polish) has been migrated to
[specs/015-nn-training-improvements/tasks.md](../015-nn-training-improvements/tasks.md).

Feature 014 scope = replumbing (GP removal, Boost removal, source reorg, repo extraction).
Feature 015 scope = NN training signal improvements.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies — start immediately
- **Phase 2 (Strip GP)**: Depends on Phase 1 contract tests — BLOCKS all subsequent work
- **Phase 3 (Replace Boost)**: Depends on Phase 2 — can overlap with late Phase 2 work
- **Phase 4 (Source Reorg)**: Depends on Phases 2+3 — all code changes done before moving files
- **Phase 5 (Cross-Repo)**: Depends on Phase 4 — transport contract finalized, autoc is main repo
- **Phases 6-13 (User Stories)**: Depend on Phase 5 — can proceed in priority order or parallel
- **Repo Extraction (T200-T210)**: After user stories stabilize — crrcsim and xiao-gp become submodules of autoc

### User Story Dependencies

- **US1 (Sigma Floor, P1)**: Independent — can start after Phase 5
- **US2 (Curriculum, P1)**: Independent — can start after Phase 5
- **US3 (Fitness Decomposition, P2)**: Independent — can start after Phase 5
- **US4 (sep-CMA-ES, P2)**: Independent — can start after Phase 5
- **US7 (Per-Timestep Streaming, P3)**: Independent — moved early because US5 and US6 depend on it
- **US5 (Segment Scoring, P3)**: Depends on US7 (needs per-timestep data)
- **US6 (Behavioral Cloning, P3)**: Depends on US7 (for data collection format)
- **US8 (Checkpoint/Resume, P3)**: T119 depends on US4 (CMAESState must exist). Rest of US8 is independent.

### Within Each User Story

- Contract tests first → implementation → integration → verify

### Parallel Opportunities

- T004/T005/T006: All contract tests in Phase 1
- T031/T032/T033/T034: All trivial Boost replacements
- T050/T051/T052/T053/T054/T055/T056/T057: All git mv operations
- US1 and US2 can run in parallel (both P1, independent)
- US3 and US4 can run in parallel (both P2, independent)

---

## Parallel Example: Phase 2c (GP removal)

```bash
# These can run in parallel (different files, no dependencies):
Task T019: "Delete autoc/autoc-eval.cc"
Task T020: "Delete autoc/gp_bytecode.h and autoc/gp_bytecode.cc"
Task T021: "Delete autoc/gp_evaluator_portable.h and autoc/gp_evaluator_portable.cc"
```

## Parallel Example: User Stories 1 + 2 (both P1)

```bash
# After Phase 5, these can run in parallel:
Task T072-T075: "Sigma Floor implementation"
Task T079-T082: "Curriculum Ramp implementation"
```

---

## Implementation Strategy

### Status: COMPLETE — Feature 014 closed

Phases 1-5 (Setup, Strip GP, Replace Boost, Source Reorg, Cross-Repo Integration) and Repo Extraction (T200-T211) completed. autoc is a standalone repo with crrcsim and xiao-gp as submodules, zero GP/Boost dependencies, clean include/src/tools/tests layout, and cereal/POSIX RPC.

Output cleanup tasks T066-T068 deferred to [BACKLOG.md](../BACKLOG.md).

All user story work (Phases 6-14) migrated to [015-nn-training-improvements](../015-nn-training-improvements/tasks.md).

- [X] T200 Extract autoc/ to standalone git repo (git subtree split), preserve commit history
- [X] T201 Sort autoc/specs/: enumerate which stay with GP repo vs move to autoc repo
- [X] T202 Move relevant specs/ and docs/ artifacts to new autoc repo
- [X] T203 Add crrcsim as git submodule of autoc repo
- [X] T204 Update crrcsim CMakeLists.txt: replace `$ENV{HOME}/GP/include` with parent autoc include path
- [X] T205 Add xiao-gp as git submodule of autoc repo
- [X] T206 Remove `~/GP` symlink from xiao-gp, update platformio.ini lib_deps to use submodule path
- [X] T207 Update xiao-gp includes: `#include <GP/autoc/...>` → `#include <autoc/...>`
- [X] T208 Build + test all three from autoc repo root: autoc, crrcsim, xiao-gp
- [X] T209 Update CLAUDE.md in all repos for new submodule workflow
- [X] T210 Update .gitignore for standalone autoc repo (no longer under GP)
- [X] T211 Cross-platform verification: train on aarch64, pull repo on x86, build, run renderer/nnextractor/eval against S3 objects created on aarch64 — validates cereal binary portability end-to-end

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- All file moves use `git mv` to preserve history
- Happy path only — strip all defensive fallback branches
- Contract tests define surviving behavior — don't break them during refactoring
- Commit after each logical group of tasks
- Stop at any checkpoint to validate independently
