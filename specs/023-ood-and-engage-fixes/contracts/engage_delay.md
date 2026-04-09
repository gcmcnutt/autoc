# Contract: CRRCSim Engage Delay Window (Change 1b)

**Feature**: 023 | **Phase**: 1 | **Scope**: CRRCSim `inputdev_autoc.cpp` timing + state contract

## Motivation

INAV has a measured ~750 ms delay between "NN switch enabled" and "commands
actually reaching servos" during MANUAL-mode handoff. The training pipeline
currently assumes NN commands take effect immediately, which produces a
sim-to-real mismatch: every real flight has ~7 ticks of free flight after
"engage" where INAV is still processing the mode change. Training must
simulate this so the NN sees the correct post-delay state distribution.

Clarified 2026-04-08 (Q2): during the window, the sim applies **centered
stick** (zero pitch/roll/throttle) — the craft coasts on FDM + wind, no
autopilot, no attitude hold, no stabilization. This matches the user's real
handoff technique: line up, release, throw the switch, let the craft coast
briefly on momentum.

## Contract

### Timing

```
t=0           engage transition (autoc_enabled: false → true)
              |
              |  window_active = true
              |  ticks_remaining = ceil(EngageDelayMs / SimTimeStepMs)
              |
t=tick_1      NN runs, updates history buffer; sim applies {0,0,0} stick
t=tick_2      same
...
t=tick_N      last "ignored output" tick (ticks_remaining hits 0)
              |
              |  window_active = false (transition)
              |
t=tick_N+1    NN runs, updates history buffer; sim applies NN_outputs * NNAuthorityLimit
t=...         normal operation until scenario end
```

With `EngageDelayMs = 750` and `SimTimeStepMs = 100`, `N = ceil(7.5) = 8` ticks.

### State variables (file-local to `inputdev_autoc.cpp`)

```cpp
struct EngageDelayState {
    bool window_active;
    int ticks_remaining;
};
static EngageDelayState g_delay_state;
```

### Transitions

| From | Event | To | Action |
|---|---|---|---|
| `{false, *}` | scenario start / engage | `{true, ceil(EngageDelayMs/dt)}` | history reset (Change 1) runs in the same transition |
| `{true, N}` where N > 1 | tick | `{true, N-1}` | apply `{0,0,0}` stick; NN runs, updates history |
| `{true, 1}` | tick | `{false, 0}` | apply `{0,0,0}` stick; NN runs; last ignored-output tick |
| `{false, 0}` | tick | `{false, 0}` | apply `NN_outputs * NNAuthorityLimit`; normal operation |
| any | scenario end | (reset to `{false, 0}` on next scenario start) | — |

### Invariants

1. **NN runs every tick regardless of window state.** The NN is computing
   forward passes and `nn_gather_inputs()` is populating `AircraftState::nnInputs_`
   every tick, including ticks where the window is active. Only the sim's
   stick-application step is gated by `window_active`.

2. **History buffer updates every tick regardless of window state.** The
   history-shift logic in `NNInputs` (P3 in `nn_interface.md`) runs on every
   tick during the window. By the end of the window, the history buffer is
   fully populated with real in-flight samples.

3. **`NNAuthorityLimit` does NOT apply during the window.** During the window,
   sim stick is hard-coded `{0, 0, 0}` — the authority limit is irrelevant.
   After the window, `stick = NN_outputs * NNAuthorityLimit`.

4. **Window duration is independent of `SimTimeStepMs`.** If we ever bump the
   sim tick rate (e.g., 10 Hz → 20 Hz, which is Out of Scope for 023 but
   possible later), `EngageDelayMs = 750` still means 750 ms of delay,
   automatically re-computed as `ticks_remaining = ceil(750 / 50) = 15` ticks
   at 20 Hz.

5. **Reset on scenario boundaries.** Each new scenario starts with
   `window_active = true` and `ticks_remaining` recomputed. Scenarios must
   not leak state across boundaries.

### Configuration

```ini
# autoc.ini / autoc-eval.ini
EngageDelayMs                   = 750       # ms
```

- Default: 750 (matches measured INAV delay from 2026-04-07 flight analysis)
- Range: [0, 5000]. `0` disables the window entirely (NN takes over from
  tick 0, matches pre-023 behavior for A/B testing).
- Type: `int`
- Read at: `ConfigManager::initialize()` in `src/util/config.cc`
- Global: `gEngageDelayMs` in `src/autoc.cc`, passed through `EvalData`
  to the CRRCSim RPC.

### Protocol additions (RPC)

`include/autoc/rpc/protocol.h` `EvalData` struct gains one new field:

```cpp
struct EvalData {
    // ... existing fields ...
    int engage_delay_ms;  // NEW — from gEngageDelayMs
};
```

Cereal serialization updated. Backward compat: old sim workers reading a new
protocol with this field will fail loud (cereal will error on the missing
field). No compat shim — per Constitution III, clean cut. Sim worker and
autoc must be rebuilt together.

## Interaction with other changes

### vs Change 1 (history reset)

Change 1's `resetHistory()` runs **at engage transition**, BEFORE the first
tick of the window. The pre-fill populates all history slots with the current
geometry at `t = engage`. During the window, history ticks advance normally,
so by `t = engage + EngageDelayMs`, the buffer contains real samples from the
delay window itself. The NN's first effective-output tick sees a fully-real
history buffer.

See `contracts/history_reset.md` for full `resetHistory()` semantics.

### vs Change 8 (authority limit)

Authority limit applies AFTER the window closes. During the window, sim stick
is `{0, 0, 0}` regardless of `NNAuthorityLimit`. After the window:
`stick = nn_outputs * NNAuthorityLimit`.

### vs xiao

**Xiao does NOT simulate the delay window.** On xiao, INAV's real delay is
what it is — we're not modeling it in xiao, we're modeling it in sim so
training matches reality. Xiao code path remains: engage → history reset
(Change 1) → NN runs every tick → outputs go to INAV → INAV takes whatever
time it takes. The sim's `EngageDelayMs` value should match the measured
INAV delay so training reflects what xiao actually experiences.

If the INAV audit (Phase 0 research) reveals the delay is significantly
different from 750 ms, update `EngageDelayMs` config default and the spec's
rationale section accordingly.

## Verification

### Unit test (`tests/engage_delay_tests.cc` NEW)

```cpp
// Synthetic minisim test — no full CRRCSim required.
TEST(EngageDelayTest, WindowSuppressesOutputs) {
    // 1. Set up a minisim scenario with EngageDelayMs=500, SimTimeStepMs=100
    // 2. Feed a mock NN that outputs {+1, -1, +0.5} every tick
    // 3. Run 10 ticks
    // 4. Assert:
    //    - Ticks 1-5 (window active): sim received {0, 0, 0}
    //    - Ticks 6-10 (window closed): sim received {+1, -1, +0.5}
    //    - NN was called every tick (including during window)
    //    - History buffer populated on every tick (check nnInputs_ progression)
}

TEST(EngageDelayTest, ZeroDelayDisablesWindow) {
    // EngageDelayMs=0 → no window, first tick applies NN outputs immediately
}

TEST(EngageDelayTest, WindowResetsOnScenarioBoundary) {
    // Run scenario A with window, end it mid-window, start scenario B:
    // scenario B must re-enter the window, not inherit scenario A's state
}
```

### Integration test (CRRCSim, Phase 3 Milestone A)

During the first CRRCSim training milestone (zero-variation baseline):
1. Log per-tick stick values alongside NN outputs to `data.dat`.
2. Spot-check a scenario: first ~7 ticks must have `stick_pitch == 0` and
   `stick_roll == 0` and `stick_throttle == 0`, while `nn_pitch`, `nn_roll`,
   `nn_throttle` have non-zero values.
3. Tick 8+ must have `stick_pitch == nn_pitch * NNAuthorityLimit` (etc.).
4. If this doesn't match, the delay window is bugged — stop and fix before
   proceeding.

## Files touched

| File | Change |
|---|---|
| `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp` | Add `EngageDelayState`, gate stick application, apply `NN_outputs * NNAuthorityLimit` after window |
| `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.h` | Declare config inputs |
| `include/autoc/rpc/protocol.h` | Add `engage_delay_ms` to `EvalData` |
| `include/autoc/util/config.h` | Add `engageDelayMs` field |
| `src/util/config.cc` | Parse `EngageDelayMs` key |
| `autoc.ini`, `autoc-eval.ini` | Add `EngageDelayMs = 750` |
| `tools/minisim.cc` | Apply same engage delay logic (contracts apply equally) |
| `tests/engage_delay_tests.cc` | NEW — unit tests as above |
