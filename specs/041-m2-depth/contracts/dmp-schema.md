# Contract — dmp schema (`EvalResults`)

**Surface**: `include/autoc/rpc/protocol.h`, `include/autoc/eval/aircraft_state.h`,
`crrcsim/.../inputdev_autoc.cpp` (push sites), `tools/dmp_dump.cc`, `tools/renderer.cc`,
`src/eval/source_dmp_loader.cc`, `src/eval/tracker_stepper.cc`, `src/eval/fitness_decomposition.cc`.

**Break class**: greenfield, no migration. Prior dmps become unreadable by the new binary **by design** —
which is why both comparator runs were pinned and verified before this lands.

---

## 1. Grouped per-tick record — replaces the parallel lists

### Before

```text
aircraftStateList[i][k]     length 1 + N     ← initial state pushed BEFORE the tick loop
cameraViewList[i][k]        length N
targetTrajectoryList[i][k]  length N
```

Consumers had to know `targets[j] ↔ states[j+1]`. Three of four known consumers had it wrong — including
**the M2 objective, since 030** — and it was undetectable in recorded data because neither
`CopiedTargetSample` nor `CameraViewSample` carries a tick index.

### After

```text
initialState                                       ← ONE named field, beside the list
tickList[i][k] = { state, cameraView, targetSample }   length N, single index
```

**Requirements**

1. There MUST be exactly one index across co-temporal per-tick data. No second collection may be indexed
   by the same loop variable across a producer/consumer boundary.
2. The pre-loop initial state MUST be a **named field**, not `tickList[0]` with sentinel members. Rationale:
   the bug being retired is precisely an extra element in one list; putting it back in the list with empty
   co-members recreates the hazard as "slot 0 is special", which is the same invariant-in-a-comment failure.
3. Consumers MUST NOT compute an index offset. The objective's `stepIndex - 1` clamp is **deleted**, not
   relocated — if a consumer still needs an offset, the grouping is wrong.
4. Tracker-only members are absent (not zero-filled) in pathgen records, and a pathgen consumer MUST never
   read them.

**Explicitly rejected**: a per-sample tick index. Operator direction 2026-08-02 — *"stick with proper
indexing, not additional storage"* — and it pays permanent bytes on every tick of every dmp to paper over a
structural problem.

---

## 2. Recorded config block

Each per-gen dump MUST carry the fitness + cadence configuration that produced it: cone angle, distance
scales, streak threshold / ramp / multiplier, control interval, and any 041-added fitness parameter.

**Read-side rule**: consumers MUST prefer the dmp-recorded config and MUST NOT silently fall back to
`ConfigManager`. Today `dmp_dump.cc` and `renderer.cc` read these from whatever ini is on disk, so a
replayed score is wrong whenever the ini has drifted — and 041 will have several inis pointed at several
sources, which makes the drift likely rather than theoretical.

The only remaining legitimate ini dependence is the S3 profile/bucket needed to *fetch* the dmp.

---

## 3. Realized wind

`AircraftState::wind_velocity` MUST carry the wind the FDM actually applied for that tick.

It is currently **serialized but never set** — zero in every dmp, M1 and M2 — while `dmp_dump` already
emits `wN,wE,wD` columns that print zeros. Getter and setter already exist. This is an honest-recording
violation ([[feedback_honest_dmp_recording]]) for an environment input the controller demonstrably
experiences.

---

## 4. Exact tick stamping

`simTimeMsec` MUST be stamped so a run at the configured cadence records exact intervals.

Current behaviour: `getSimulationTimeSinceReset()` truncates a 200 Hz step clock to integer ms, so a 20 Hz
run records 49/50/51 ms gaps. It feeds the **ms-based history-lag selection** and the **`span_rate` gap
denominator**, so the jitter is noise injected into exactly the rate inputs 041 reasons about.

Fix by rounding or by deriving from the integer step count.

⚠️ **crrcsim submodule change, determinism-affecting.** Submodule pointer bump **first**, parent merge
second ([[feedback_submodule_merge_order]]). Never mid-bake. After it lands, the M2 source-spacing check
may revert from an average-gap test to a strict single-gap test.

---

## 5. Version field — bump, do not migrate

Per Constitution V write-side and [research.md](../research.md) R6:

1. The version field **MUST be bumped** for this transition.
2. There MUST be **no migration path or compatibility shim** (Constitution III).
3. A reader encountering an older artifact MUST **fail loudly, naming both the artifact version and the
   reader version**, and MUST NOT silently truncate or default-initialize.

**Why the loud-fail message matters concretely**: the 038 baseline currently fails as
`vector::_M_default_append` — a schema mismatch presenting as a memory error, which cost real diagnosis
time at the 040 wrap. A message saying "artifact v=N, reader v=N+1" turns that into one line.

This is a **change in recorded practice** — the informal habit was "never bump" — and is called out as a
decision rather than a drift.

---

## 6. Physics trace — reader only, no schema change

`PhysicsTraceEntry` already carries per-tick `acc[3]`, `omegaDotBody[3]`, `alpha`, `beta`, `vRelWind`, is
populated for **every elite reeval** (`inputdev_autoc.cpp:1047`) and is already serialized into every gen
dmp — with **no consumer anywhere in `tools/` or `src/`**.

041 adds physics columns to `dmp-dump` so load becomes readable. **No recording change.** See
[offline-study.md](offline-study.md).

---

## 7. Tests required

| test | asserts |
|---|---|
| grouped-record round-trip | serialize → deserialize preserves every member and the tick count |
| **zero-answer objective** | data whose correct score is exactly 0 scores exactly 0 (FR-003) |
| **shifted-input-worse** | a deliberately one-tick-shifted input scores visibly worse — a test that passes either way would be worse than none |
| initial-state handling | the named field survives round-trip and is not confused with tick 0 |
| config-block preference | a reader with a deliberately-drifted ini still replays the recorded config's numbers |
| wind recorded | a run in non-zero wind records non-zero `wind_velocity` |
| tick exactness | consecutive gaps are exactly the control interval |
| version fail-loud | reading a prior-version artifact errors with both version numbers, and does not crash in the allocator |
