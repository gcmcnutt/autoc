# 014-nn-training-signal: Handoff Notes

**From**: Claude session on GP repo (014-nn-training-signal branch)
**To**: Claude session on autoc repo (after speckit init)
**Date**: 2026-03-16

## What Just Happened

We extracted `~/GP/autoc/` into a standalone `~/autoc/` repo via `git subtree split`. This repo is the new home for all NN aircraft controller evolution work. The GP genetic programming library stays behind in `~/GP/`.

### Repo Structure

```
~/autoc/
├── include/autoc/          # Headers (nn/, eval/, util/, rpc/)
├── src/                    # Core source (autoc.cc, nn/, eval/, util/)
├── tools/                  # minisim, renderer, nnextractor, nn2cpp
├── tests/                  # 88 tests (contract + unit)
├── crrcsim/                # Git submodule → github.com/gcmcnutt/CRRCsim
├── xiao/                   # Embedded target (copied from xiao-gp, not submodule)
├── specs/                  # Feature specs (ZZZ- = archived)
├── CMakeLists.txt          # Build (inih, cereal, gtest via FetchContent)
├── autoc.ini / autoc-eval.ini
├── rebuild.sh / rebuild-perf.sh / crrcsim.sh
```

### What Was Completed (Phases 1-5)

| Phase | What | Key Result |
|-------|------|------------|
| 1 | Contract tests + dependency analysis | 88 tests, dependency surface documented |
| 2 | Strip GP code, sever libgp.a | -6,718 lines, zero GP includes |
| 3 | Replace Boost with cereal/POSIX/std:: | +52% throughput (27k→41k sims/sec) |
| 4 | Source reorg (include/src/tools/tests) | Standard C++ layout, git mv preserved history |
| 5 | Cross-repo (CRRCSim, xiao-gp) | All 3 build clean, NN-only, smoke tested |

### What Was Smoke Tested

- **autoc**: 200 gen evolution, 2000 pop, 20 threads, 49 winds — deterministic
- **renderer**: Visualized results from S3
- **nnextractor**: Extracted weights, correct gen number (10000-key)
- **eval mode**: Reproduced training fitness exactly (287763.847597)
- **nn2cpp**: Generated embedded C++ with provenance
- **crrcsim**: Full FDM run, rendered, extracted, eval matched
- **xiao-gp**: PlatformIO build success (43.5% flash, 57.3% RAM)

## Remaining Steps (Do In This Order)

### Step 1: Fix paths for new repo layout

The CMakeLists.txt and rebuild scripts still have paths assuming `~/GP/autoc/`. Update:

- **CMakeLists.txt**: Already at top level, but `rebuild.sh` references `cd ~/GP/autoc`. Fix to `cd ~/autoc`.
- **rebuild.sh / rebuild-perf.sh**: Update crrcsim build path from `~/crsim/crrcsim-0.9.13` to `./crrcsim` (relative).
- **crrcsim.sh**: Update crrcsim binary path similarly.
- **crrcsim/CMakeLists.txt**: Replace `$ENV{HOME}/GP/autoc/include` with `${CMAKE_SOURCE_DIR}/include` (parent autoc).
- **crrcsim/src/mod_inputdev/CMakeLists.txt**: Same — source files from `$ENV{HOME}/GP/autoc/src/` become `${CMAKE_SOURCE_DIR}/src/`.
- **crrcsim FetchContent cereal**: Remove from crrcsim CMakeLists — share from parent autoc top-level CMake.
- **xiao/platformio.ini**: Replace `include/autoc/include` with `../include` and `include/autoc/src/` with `../src/`. Remove the symlink dependency entirely.
- **xiao/ includes**: Already use `<autoc/...>` style. The `-I` flag in platformio.ini just needs to point at `../include`.

### Step 2: Unified CMake build

Create or update top-level CMakeLists.txt to:
1. Declare shared deps once: cereal, googletest, inih, Eigen
2. Build autoc executables (autoc, minisim, renderer, nnextractor, nn2cpp)
3. `add_subdirectory(crrcsim)` for the FDM — crrcsim inherits cereal from parent
4. Tests via `add_subdirectory(tests)` or in the main CMakeLists
5. Single `mkdir build && cd build && cmake .. && make` builds everything

### Step 3: .gitignore

Create top-level .gitignore for autoc:
```
build/
log/
*.dat
*.stc
nn_weights.dat
.pio/
xiao/.pio/
```

### Step 4: Speckit init

Run `speckit init` in `~/autoc/`. This creates `.specify/` with constitution, memory, templates. The constitution should reflect:
- NN-only aircraft controller evolution system
- C++17, CMake, Eigen, cereal, GoogleTest
- Desktop (train) + embedded (deploy) dual-target
- Three components: autoc (evolution), crrcsim (FDM), xiao (embedded)

### Step 5: Migrate relevant specs

From `~/GP/specs/`:
- **014-nn-training-signal/**: Copy spec.md, plan.md, tasks.md, research.md, data-model.md, contracts/, analysis/, checklists/ → `~/autoc/specs/014-nn-training-signal/`
- These are the active work items for NN improvements

From `~/autoc/specs/`:
- **ZZZ-* files**: These are archived GP-era specs. They have historical value (decisions made) but are not active. Consider moving to a `specs/archive/` subdirectory or purging.
- **PROFILING.md**: Still relevant — keep.

From `~/xiao-gp/specs/` (now `~/autoc/xiao/specs/`):
- Review for anything still relevant to embedded deployment.

From `~/GP/.specify/`:
- Don't copy — fresh speckit init is cleaner. But review constitution for any principles worth carrying forward.

### Step 6: Backlog consolidation

Scattered TODO items exist in:
- `~/GP/specs/BACKLOG.md` — GP-era backlog, some items still relevant
- `~/autoc/xiao/TODO.md` — embedded-specific TODOs
- Various `// TODO` comments in source

Consolidate into a single `~/autoc/specs/BACKLOG.md` after speckit init.

### Step 7: Output cleanup (deferred T066-T068)

- OutputDir config key
- Auto-created run subdirectory
- Clean eval prefix naming

### Step 8: Cross-platform verification (T211)

- Train on aarch64, pull repo on x86
- Build, run renderer/nnextractor/eval against S3 objects created on aarch64
- Validates cereal binary portability end-to-end

### Step 9: Create GitHub repo and push

- Create `github.com/gcmcnutt/autoc` repo
- Push master (extracted history) and 014-nn-training-signal branch
- Verify crrcsim submodule resolves on clone

## Then: NN Improvements (Phase 6+)

These are the actual training signal improvements, in priority order:

1. **Sigma Floor** (Phase 6, US1, P1) — prevent search freeze, quick win
2. **Curriculum Ramp** (Phase 7, US2, P1) — progressive difficulty
3. **Fitness Decomposition** (Phase 8, US3, P2) — minimax/percentile aggregation
4. **sep-CMA-ES** (Phase 9, US4, P2) — replace GA with efficient optimizer
5. **Per-Timestep Streaming** (Phase 10, US7, P3) — enables segments + cloning
6. **Segment Scoring** (Phase 11, US5, P3) — per-segment credit assignment
7. **Checkpoint/Resume** (Phase 13, US8, P3) — crash recovery

Full task details are in `~/GP/specs/014-nn-training-signal/tasks.md` (copy to this repo in Step 5).

## Pre-Migration from ~/GP Config Dirs

**`.vscode/`** — Copy and adapt. Has autoc/minisim debug configs (launch.json), C++ intellisense (c_cpp_properties.json), build tasks. Update paths from `${workspaceFolder}/build/` (was GP/build) — should work as-is since autoc is now the workspace root.

**`.github/workflows/`** — Copy the Claude Code workflow. Update for autoc repo if desired.

**`.specify/`** — Do NOT copy. Fresh `speckit init` in ~/autoc. BUT read `.specify/memory/constitution.md` for core principles to carry forward:
- Testing-First (all changes need tests)
- Multi-repo build documentation
- The constitution was versioned 1.1.0

**`.claude/`** — Do NOT copy. Auto-generated per-project. The `commands/` has speckit commands that will be re-created by speckit init.

## Key Memories (for new session context)

- **No compatibility shims** — clean cut everything, update all callers directly
- **Every file must justify existence** after reorg
- **Profile cereal serialization** cost in large-scale crrcsim run (27k→41k was minisim-only)
- **eval_scalar rename** — gp_scalar→eval_scalar is deferred cosmetic rename
- **fastAtan2 LUT** — keep for embedded platform compatibility, matches training precision
- **NN01 format v1** — no version bumps until stable
- **autoc is the main repo** — crrcsim and xiao are subordinate
