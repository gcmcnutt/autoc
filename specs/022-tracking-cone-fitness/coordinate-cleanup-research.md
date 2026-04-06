# Coordinate Convention Cleanup — Research

**Date**: 2026-04-06
**Context**: 022-tracking-cone-fitness branch, prompted by originOffset bug

## Problem Statement

The codebase has an inconsistent, fragile handling of raw (world) vs virtual
(path-relative) coordinate systems. This leads to bugs (originOffset not
serialized, double-offset, forgotten offset) and makes the code hard to reason
about. We need a single, principled strategy.

## First Principles

1. **Virtual paths start at virtual origin (0,0,0 NED).**
   Paths are generated in canonical space. The rabbit starts at (0,0,0).

2. **Producers convert raw→virtual at the boundary, once.**
   CRRCSim and xiao each capture a raw origin offset at test start, subtract
   it from raw FDM/INAV position, and store virtual position in AircraftState.
   This mirrors the INAV sign convention pattern: convert once at boundary,
   standard everywhere downstream.

3. **All downstream code operates in virtual space.**
   NN sensor inputs, fitness evaluation, logging — all use `getPosition()`
   which IS virtual. No `getVirtualPosition()` indirection needed.

4. **Raw coordinates are used only for out-of-bounds detection** (sim-side,
   before storing virtual position) and optionally logged as metadata.

5. **Renderer translates virtual→display** by adding a display offset
   (e.g., SIM_INITIAL_ALTITUDE for Z). Both paths and aircraft get the
   same transform.

6. **Origin offset is metadata**, stored per-scenario (in ScenarioMetadata
   or EvalResults header), not per-AircraftState. Available for renderer
   "all paths in raw" mode and for correlating with real-world coordinates.

## Current State Audit

### Producer: CRRCSim (inputdev_autoc.cpp)

| Aspect | Current | Correct? | Notes |
|--------|---------|----------|-------|
| Path start (L511-540) | Capture raw FDM pos → `setOriginOffset()` on AircraftState | Partial | Offset set correctly but stored on wrong object |
| Each tick (L685) | `setPosition(p)` with raw FDM pos | **WRONG** | Should store virtual: `p - originOffset` |
| OOB check (L702-708) | Uses `aircraftState.getPosition()` (raw) | Works now | Must switch to local `p` before we change position to virtual |
| NN sensors (sensor_math.cc) | Uses `getVirtualPosition()` | Correct | Works because originOffset_ IS set on CRRCSim side |
| NN evaluator (evaluator.cc:213) | Uses `getVirtualPosition()` | Correct | Same reason |
| dhome (L972) | Uses `getVirtualPosition().norm()` | Correct | Same reason |

### Producer: Xiao (msplink.cpp)

| Aspect | Current | Correct? | Notes |
|--------|---------|----------|-------|
| Enable (L408) | Capture raw INAV pos → `test_origin_offset` | ✓ | Module-level variable |
| Each MSP (L647-659) | `position_rel = raw - offset; setPosition(position_rel)` | ✓ | **Already stores virtual!** |
| originOffset_ on AircraftState | Never set | OK by accident | getVirtualPosition() = virtual - 0 = virtual |
| Flash log | Logs both `pos_raw` and `pos` (virtual) | ✓ | Good practice |

### Transport: RPC Serialization (aircraft_state.h)

| Field | Serialized? | Notes |
|-------|-------------|-------|
| `position` | Yes | Currently RAW from CRRCSim, virtual from xiao |
| `originOffset_` | **NO** | "not serialized, set at runtime" — **ROOT CAUSE of bugs** |

### Consumer: autoc fitness (fitness_decomposition.cc)

| Aspect | Current | Correct? |
|--------|---------|----------|
| L43: `getVirtualPosition()` | Returns `position - originOffset_` | **WRONG**: originOffset_=0 after deserialization → returns RAW |

### Consumer: autoc logging (autoc.cc logEvalResults)

| Aspect | Current | Correct? |
|--------|---------|----------|
| L573: `originOffset = aircraftStates.at(0).getPosition()` | Manual recomputation | Correct but hacky |
| L591: `stepState.getPosition() - originOffset` | Virtual via manual subtraction | Correct but fragile |
| L678-680: `stepState.getPosition()[0,1,2]` (X,Y,Z columns) | Logs **RAW** position | **WRONG**: should log virtual |
| L627: dhome uses `(home - aircraftPosition).norm()` | Uses manually-computed virtual | Correct |

### Consumer: Renderer (renderer.cc)

| Mode | Current | Notes |
|------|---------|-------|
| Training playback: paths | Virtual (Z=0) → adds SIM_INITIAL_ALTITUDE → Z=-25 | ✓ |
| Training playback: aircraft | RAW (Z≈-25) → no transform → Z=-25 | Works by accident: raw Z happens to match display |
| Xiao single span | Virtual pos → adds SIM_INITIAL_ALTITUDE | ✓ |
| Xiao all-flight | Raw pos → translates by first origin | ✓ (uses raw for absolute positioning) |
| Rabbit reconstruction | Virtual → adds SIM_INITIAL_ALTITUDE | ✓ |

## Specific Bugs Found

### Bug 1: originOffset_ not serialized (CRITICAL)
- **Location**: aircraft_state.h:395, serialize() at L426
- **Impact**: On autoc side, `getVirtualPosition()` returns raw position
- **Affected**: fitness_decomposition.cc:43 uses `getVirtualPosition()` thinking
  it gets virtual, but gets raw. ~25m Z error in fitness scoring.
- **Current workaround**: autoc.cc:573 manually recomputes offset from first state

### Bug 2: data.dat logs raw aircraft position (MODERATE)
- **Location**: autoc.cc:678-680
- **Impact**: X,Y,Z columns in data.dat are in raw space while pathX/Y/Z and
  distance/along columns are in virtual space. Mixed coordinate frames in one file.
- **Confusion**: Analysing data.dat requires knowing that aircraft position is raw
  but metrics are virtual

### Bug 3: CRRCSim OOB uses AircraftState position (LATENT)
- **Location**: inputdev_autoc.cpp:703-707
- **Impact**: Currently works because position IS raw. Will break when we store
  virtual position. Must use local raw variable instead.

### Bug 4: Xiao doesn't set originOffset_ on AircraftState (LATENT)
- **Location**: msplink.cpp
- **Impact**: getVirtualPosition() happens to return virtual (offset=0) because
  position is already virtual. But code semantics are wrong — any code that
  checks originOffset_ would get Zero.

## Proposed Solution: Virtual-at-Boundary

### Change Summary

**CRRCSim (inputdev_autoc.cpp)**:
1. Add module-level `gp_vec3 pathOriginOffset` (like xiao's `test_origin_offset`)
2. At path start: `pathOriginOffset = initialPos` (raw FDM position)
3. Store initial state: `initialState.setPosition(initialPos - pathOriginOffset)` = (0,0,0)
4. Each tick: compute raw `p`, do OOB check with `p`, then `aircraftState.setPosition(p - pathOriginOffset)`
5. Remove `setOriginOffset()` calls on AircraftState
6. Store `pathOriginOffset` in ScenarioMetadata for downstream use

**AircraftState (aircraft_state.h)**:
1. Remove `originOffset_`, `setOriginOffset()`, `getOriginOffset()`, `getVirtualPosition()`
2. `getPosition()` now IS virtual — the only position accessor
3. All callers of `getVirtualPosition()` → `getPosition()`

**ScenarioMetadata (protocol.h)**:
1. Add `gp_vec3 originOffset` field (serialized via cereal)
2. Set by CRRCSim at path start, available to autoc and renderer

**autoc.cc logEvalResults()**:
1. Remove manual `originOffset = aircraftStates.at(0).getPosition()` hack
2. Use `stepState.getPosition()` directly (it's virtual now)
3. X,Y,Z columns in data.dat now log virtual position
4. Optionally log origin offset once per scenario in comment/header

**fitness_decomposition.cc**:
1. `getVirtualPosition()` → `getPosition()` (now virtual)

**sensor_math.cc, evaluator.cc, minisim.cc**:
1. `getVirtualPosition()` → `getPosition()` (now virtual)

**Renderer (renderer.cc)**:
1. `updateGenerationDisplay()`: Add SIM_INITIAL_ALTITUDE to aircraft positions
   (not just paths) — both are now virtual at Z≈0, need shift for display
2. `renderFullScene()` all-flight mode: Use originOffset from ScenarioMetadata
   to reconstruct raw positions for "all paths in raw" display
3. Xiao modes: Minimal/no change — already handles virtual

**COORDINATE_CONVENTIONS.md**:
1. Add "Virtual Frame" section documenting the principle and boundary

### Migration Risk Assessment

| Change | Risk | Mitigation |
|--------|------|------------|
| CRRCSim stores virtual | Medium — OOB must use raw | Move OOB check before setPosition, use local `p` |
| Remove getVirtualPosition() | Low | Compile-time: all callers must update or fail to build |
| Renderer aircraft positions | Medium | Both training and xiao paths affected | Test with existing data.dat files |
| data.dat format change | Low | X,Y,Z columns shift by ~constant offset | Python scripts need update |
| originOffset in ScenarioMetadata | Low | New serialized field, backwards compat via cereal versioning |

### Files Changed

| File | Change Type | Lines ~affected |
|------|-------------|----------------|
| `include/autoc/eval/aircraft_state.h` | Remove originOffset_ machinery | ~10 lines removed |
| `include/autoc/rpc/protocol.h` | Add originOffset to ScenarioMetadata | ~5 lines |
| `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp` | Store virtual, OOB with raw, set metadata | ~20 lines |
| `src/eval/fitness_decomposition.cc` | getVirtualPosition→getPosition | 1 line |
| `src/eval/sensor_math.cc` | getVirtualPosition→getPosition | 2 lines |
| `src/nn/evaluator.cc` | getVirtualPosition→getPosition | 1 line |
| `src/autoc.cc` | Remove manual offset, use getPosition directly | ~10 lines |
| `tools/renderer.cc` | Add SIM_INITIAL_ALTITUDE to aircraft positions from eval | ~15 lines |
| `tools/minisim.cc` | Virtual start, OOB raw reconstruction, remove originOffset hack | ~15 lines |
| `docs/COORDINATE_CONVENTIONS.md` | Add Virtual Frame section | ~30 lines |

### Minisim-Specific Issues

Minisim has the same structural bugs plus its own wrinkle: `minisimAdvanceState()`
advances `position` in-place (no separate FDM providing raw position).

| Issue | Location | Impact |
|-------|----------|--------|
| Starts at raw Z=-25 | L135: `initialPosition(0, 0, SIM_INITIAL_ALTITUDE)` | Hackaround: needs originOffset to compute virtual |
| setOriginOffset hack | L139: `setOriginOffset(initialPosition)` | Makes getVirtualPosition work locally, but wrong pattern |
| OOB uses position directly | L196-202: `getPosition()[2] > SIM_MIN_ELEVATION` | **After cleanup: virtual Z=0 > -7 → immediate OOB!** |
| getVirtualPosition() | L174: distance calculation | Works by accident (offset set locally, no RPC) |

**Fix**: Start at virtual (0,0,0). After physics advance, reconstruct raw for OOB:
`rawForOOB = getPosition() + vec3(0, 0, SIM_INITIAL_ALTITUDE)`. Check rawForOOB
against existing bounds. Same principle as CRRCSim: raw for OOB only, virtual everywhere else.

### NED / Body Frame Verification

Verified all coordinate conventions are correct and consistent:

| Convention | CRRCSim | Minisim | Xiao | Status |
|-----------|---------|---------|------|--------|
| World NED (+X=N, +Y=E, +Z=Down) | FDM native (ft→m) | Kinematic in NED | INAV NEU→NED at boundary | ✓ |
| Body FRD (+X=fwd, +Y=right, +Z=down) | EOM01 native | AngleAxis on UnitX/Y | INAV body frame | ✓ |
| Cockpit up = -Z world (level) | Level quaternion preserves Z | Rz(π) preserves Z mapping | Quaternion conjugate aligns | ✓ |
| Positive roll = right wing down | EOM01 RHR | AngleAxis(roll, UnitX) | INAV roll matches standard | ✓ |
| Positive pitch = nose up | EOM01 RHR | AngleAxis(pitch, UnitY) | INAV pitch NEGATED at boundary | ✓ |
| Positive yaw = nose right | EOM01 RHR | Rz(yaw) | INAV yaw NEGATED at boundary | ✓ |
| earth→body quaternion | EOM01 ZYX init + kinematics | Same Eigen convention | Conjugate from INAV body→earth | ✓ |
| Gyro rates aerospace RHR | FDM omega_body (rad/s) | Extracted from quat delta | Negated pitch/yaw from INAV | ✓ |

**No NED or body frame issues found.** All three producers (CRRCSim, minisim, xiao)
use consistent NED world frame and FRD body frame with standard aerospace right-hand
rule. Cockpit points up (-Z world) in level flight across all environments.

### Validation Plan

1. **Compile**: All `getVirtualPosition()` callers updated → clean build
2. **Unit tests**: fitness_decomposition_tests with synthetic trajectories verify correct distances
3. **data.dat check**: Start training, verify X,Y,Z columns near (0,0,Z_entry_offset) not (-25m)
4. **Renderer**: Load data.dat in renderer, verify paths and aircraft overlap correctly
5. **Python validation**: Recreate fitness from data.dat columns, zero mismatches
6. **Xiao**: Load xiao log in renderer, verify display unchanged (already virtual)
7. **Minisim OOB**: Verify minisim doesn't immediately OOB on first tick (virtual Z=0 with raw OOB reconstruction)
8. **NED consistency**: Spot-check data.dat: aircraft heading south should show velocity_body.x > 0, position.x decreasing
