# Index-coupling inventory (US1 / A0)

**Deliverable of T012–T016.** Every entry is marked **fixed**, **structurally eliminated**, **covered by a
test**, or **accepted (with reason)**. SC-001 is met when no known instance is asserted only by a comment.

**Method** — grep-guided, then every hit read. The sweeps were:

```sh
grep -rn "List\.at(\|List\[" src/eval/ src/nn/ tools/ crrcsim/src/mod_inputdev/   # T012 — 108 hits
grep -rn "ScenarioMetadata" include/autoc/rpc/ include/autoc/eval/                # T013
grep -rn "hb1AirframeObstruction\|hb1AirframeDimensions" include/ src/ tests/     # T014
grep -c "runConfig" tools/*.cc ; grep -c "ConfigManager::getConfig" tools/*.cc    # T015
```

⚠️ **This is a survey, not a proof.** The pattern finds collections named `*List`; a parallel pair named
otherwise would be missed. The classes below are the shapes to look for, and the entries are what those
sweeps actually turned up.

T012 hit distribution: `renderer.cc` 65, `dmp_dump.cc` 13, `tracker_dmp_inspect.cc` 11,
`fitness_decomposition.cc` 8, `inputdev_autoc.cpp` 6, `source_dmp_loader.cc` 3, headers 2.

---

## Class A — per-TICK parallel lists (the class 041 retires)

### A1. `aircraftStateList` ↔ `cameraViewList` ↔ `targetTrajectoryList` — **LIVE OFFSET**

`include/autoc/rpc/protocol.h:439-440`. Declared as "parallel aircraftStateList's per-scenario per-tick
indexing" — but they are **not** 1:1. `aircraftStateList[i]` carries a pre-loop **initial state at index 0**;
the view/target lists begin at the first *stepped* tick. So tick `k`'s camera view is `cams[k-1]`.

Live consequence, already documented in code at `src/eval/fitness_decomposition.cc:300-304`:

> `stepIndex` is a STATE index and tick k's view is `cams[k-1]`. Observation-only, so this was cosmetic —
> but leaving one of the two sites wrong is how the next reader concludes the pairing is 1:1.

The prediction-score pairing (FR-004) is the same offset applied at a site where it is **not** cosmetic.

**Status: structurally eliminated** — T020 replaces the parallel lists with `tickList[i][k] = {state,
cameraView, targetSample}` and a **separately named** initial-state field (research.md R5: *not*
`tickList[0]` with sentinel members, which recreates the hazard as "slot 0 is special"). T022 **deletes** the
`stepIndex - 1` clamp rather than relocating it. Consumers migrated at T023–T027; round-trip test at T028;
zero-answer tests at T017–T019 written first, with T019 expected RED until T022.

### A2. `physicsTrace` ↔ `aircraftStateList` — **different tick RATES, not an offset**

`protocol.h:429`. Sits in the same struct and reads like another per-tick parallel list, but is captured
inside the FDM's 200 Hz substep loop (`fdm_larcsim.cpp:890`, gated by `MAX_TRACE_STEPS = 35`), while states
are recorded per 50 ms control tick. `physicsTrace[i][k]` is a **different moment in time** from
`aircraftStateList[i][k]`, and after 175 ms there is no physics row at all.

**Status: fixed at the only consumer (2026-08-10), coupling accepted.** `dmp-dump --physics` (T010) joins on
**recorded time** with a half-tick tolerance and reports match/miss counts per scenario; it was that
diagnostic that surfaced the 0.89% coverage. The producer keeps its cap by operator decision (spec.md
Clarifications, 2026-08-10) — it is a determinism-debugging aid, not a flight recorder. There is no second
consumer to get this wrong, and any future one now has a worked example.

---

## Class B — per-SCENARIO parallel lists ⚠️ **NOT addressed by the grouped record**

`EvalResults` carries **eight** collections indexed by the same scenario `i` (`protocol.h:423-442`):
`crashReasonList`, `pathList`, `aircraftStateList`, `scenarioList`, `debugSamples`, `physicsTrace`,
`arenaEgressCount`, `hullStrikeCount`. `fitness_decomposition.cc:122-124` reads three of them off one `i`.

The T020 grouped record groups the **tick** axis. It does nothing for the scenario axis, and after 041 the
scenario axis is where every remaining instance of this shape lives.

Two things keep this from being urgent, and they are worth stating rather than assuming:

- They are appended **in lockstep in one place** per scenario (`inputdev_autoc.cpp:1030-1054`), not built
  independently and joined later. A1's offset arose precisely because the two lists had *different start
  points*; these do not.
- `physicsTrace` and `debugSamples` are the exception — appended only `if (evalData.isEliteReeval)`, so on a
  non-elite dmp they are **empty while the others are full**. Any consumer indexing them by `i` without a
  size check reads out of range. Today's consumers all bounds-check.

**Status: accepted for 041, filed for the follow-on.** Grouping the scenario axis is a strictly larger
change than the one this feature is already taking, and it would land in the same commit as the M1 bake's
schema — raising the risk of the one commit 041 cannot afford to get wrong.

⚠️ **And the follow-on is not "group the scenario axis too."** Operator 2026-08-10: *"try not to sync
independent lists — rather we refactor the whole thing by time. Big ripple effect for later on."* The target
is to make **time the organising principle**, so there is no set of independent series to keep in step at
all — which subsumes Class B, Class A2's rate mismatch, and Class D's dual lifetime in one move. Filed with
full rationale in [`specs/BACKLOG.md`](../BACKLOG.md) → *"Reorganise `EvalResults` BY TIME"*; T101 confirms
it at wrap rather than filing it fresh.

## Class C — worker-init parallel lists

`WorkerInit::pathList` ↔ `scenarioMetaList` (`protocol.h:208-209`), documented at `:204` as
"parallel-indexed with". Built once at startup, read once per dispatch by the same index.

**Status: accepted.** Single producer, single consumer, no offset, no lifetime split. Recorded so the sweep
is complete rather than because it is a risk.

---

## Class D — one struct, two lifetimes

### D1. `ScenarioMetadata` as RPC payload *and* as persisted record

`EvalResults::scenario` (singular, the per-eval RPC field) and `EvalResults::scenarioList` (the persisted
per-scenario table) are the **same type** serving two lifetimes — `protocol.h:426-427`. This shape already
**cost a launch on 2026-08-02**.

**Status: needs coverage.** The grouped-record work does not touch it, and no test asserts which field a
given reader should be using. Cheapest sufficient guard: a round-trip test asserting that a deserialized
`EvalResults` has `scenarioList.size() == aircraftStateList.size()` and that `scenario` is *not* read by any
persistence consumer. Folded into T028's round-trip test rather than given its own task.

---

## Class E — one value, two definitions

### E1. `CameraConfig` / airframe defaults vs `hb1AirframeObstruction()`

`include/autoc/util/config.h:352` states outright: *"Defaults mirror `hb1AirframeObstruction()` so the two
cannot drift"*. The mirroring is guarded by `ContractTrackerConfig.AirframeObstructionKeysPresentInTrackerInis`
(all 18 keys must be **assigned** in every ini, not merely defaulted).

**Status: covered by a test — and this is the model to copy.** It is the only entry in this inventory where
the duplication is deliberate, documented, and asserted. Every other "two definitions" case should either
adopt this pattern or be collapsed.

---

## Class F — compiled-in default vs recorded config

`dmp-dump` reads the dmp's own `RecordedRunConfig` for the fitness cone and cadence (038 P0-B); it uses
`ConfigManager` only for S3 bucket/profile. **`tools/renderer.cc` has not made that move** — 5
`ConfigManager::getConfig()` reads against 1 `runConfig` reference. So the renderer replays a recorded run
using **today's ini**, and a knob edited since the bake silently changes what is drawn.

**Status: fixed at T043**, which serializes the fitness/cadence config block into `EvalResults` and flips
both `dmp_dump.cc` and `renderer.cc` to prefer the recorded config, with a test that a reader holding a
deliberately-drifted ini still replays the recorded numbers.

---

## Appendix — grouped-record migration list (T023–T027)

Every consumer that indexes the per-tick lists and must move to `tickList`:

| consumer | task | note |
|---|---|---|
| `src/eval/fitness_decomposition.cc` | T022 | the objective; **delete** the `stepIndex - 1` clamp |
| `tools/dmp_dump.cc` | T023 | coordinate with the T010 physics columns already added |
| `tools/renderer.cc` | T024 | 65 of the 108 sweep hits; also carries F above |
| `src/eval/source_dmp_loader.cc` | T025 | `:138-140`, scenario-axis reads — check both axes |
| `src/eval/tracker_stepper.cc` + `crrcsim_tracker_helper.cpp` | T026 | producer side |
| `src/analytics/` per-tick readers | T027 | CSV-column consumers, not struct consumers |
| `tools/tracker_dmp_inspect.cc` | **⚠️ not in the task list** | 11 sweep hits; add to T023's scope or it breaks at T045 |
