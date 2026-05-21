# 030 — Quickstart (smoke-test runbook)

End-to-end recipe for executing the 030 v1 smoke test (D13's four deliverables) once `/speckit.tasks` has produced `tasks.md` and the milestones M1–M9 have landed.

> **Scope**: this quickstart is the **smoke-test runbook**. Earlier development milestones (M1–M9) each have their own visible-checkpoint procedures inside [plan.md](./plan.md); use those during the build-up. This document is the M10 operator walkthrough — "how do I run the smoke test once the implementation milestones are green."

## Prerequisites

- M1–M9 all green per [plan.md](./plan.md). In particular:
  - `bash scripts/rebuild.sh` succeeds (autoc + crrcsim + tools).
  - `cd xiao && pio run -e xiaoblesense_arduinocore_mbed` succeeds.
  - All test suites green: `cd build && ctest --output-on-failure`.
- An M1 source dmp identified — recommended: `pastonly3` gen-N from a recent run, available at:
  `s3://<bucket>/autoc-storage/autoc-9223370259105171692-2026-05-02T19:20:04.115Z/gen<N>.dmp`
  The smoke slice (per Clarifications 2026-05-04) is `TrackerPathSubset = 0..5` × `TrackerWindSubset = 0..19` = 120 scenarios; ensure the source-run's path family covers paths 0-5 and that those scenarios are non-truncated (≥ `MIN_SCENARIO_TICKS = 30` per T020). Filtering of source scenarios with terminal crashes is per FR-013 (resolution pending C1 — see [plan.md M3](./plan.md#m3) and [tasks.md](./tasks.md)).
- AWS credentials configured — production load path is **S3-key-driven** (the `TrackerSourceRun = autoc-storage/<run-id>/gen<N>.dmp` form IS the S3 key, streamed directly). Local file paths are accepted as offline-test convenience but are not the canonical input.

## Steps

### 1. Configure `autoc-tracker.ini`

Copy the v1 baseline template and populate:

```ini
[Source]
TrackerSourceRun = autoc-storage/autoc-9223370259105171692-2026-05-02T19:20:04.115Z/gen9609.dmp
; v1 smoke-test slice (per Clarifications 2026-05-04): 6 paths × 20 winds = 120 scenarios.
; Cross-product subsetting; concrete syntax to be locked in plan phase.
TrackerPathSubset = 0,1,2,3,4,5
TrackerWindSubset = 0-19

[TrackingFitness]
TrailDistance = 3.048             ; 10 ft, FR-008 v1 default
LowSpeedTrailThreshold = 2.0      ; R10 v1 default
LowSpeedTrailHysteresis = 0.5

[CrashHull]
CrashHullShape = SPHERE
CrashHullRadius = 1.0             ; FR-008b v1 default
PCrashGen0 = 0.0                  ; R3 curriculum
PCrashGenRamp = 100
PCrashGenPlateau = 200
PCrashPlateau = 0.30

[Arena]
FlightArenaRadius = 80.0          ; R2 v1 default
FlightArenaFloorAGL = 5.0
FlightArenaCeilingAGL = 100.0

[Camera]                          ; D10 v1 baseline
CameraCount = 1
CameraFOVHorizontal = 120.0
CameraFOVVertical = 90.0
CameraFrameRateHz = 30
CameraLatencyMs = 0
CameraMountOffsetX = 0.0
CameraMountOffsetY = 0.0
CameraMountOffsetZ = -0.05

[Beacon]                          ; D1 v1 baseline (270° outward, hb1 wingspan)
BeaconLeftWavelength = 850
BeaconRightWavelength = 940
BeaconEmissionConeDeg = 270
BeaconLeftMountY = -0.45
BeaconRightMountY = +0.45

[Population]
PopulationSize = 5000             ; full diversity per Clarifications 2026-05-04
NumGenerations = 100              ; compressed from converged-run 600+
; Variation-curriculum step count = 10 (parameter name TBD in plan phase)
Seed = 42                         ; deterministic
```

### 2. Launch tracker-mode autoc

```bash
cd /home/gmcnutt/autoc
stdbuf -oL -eL build/autoc -i autoc-tracker.ini 2>&1 | tee logs/autoc-030-smoke-001.log
```

**Expected log lines** (first ~20 seconds):
```
Loading source dmp: autoc-storage/.../gen9609.dmp
  → 245 scenarios, slicing TrackerPathSubset=0,1,2,3,4,5 × TrackerWindSubset=0-19 → 120 scenarios
  → terminal-crashed source scenarios filtered out per FR-013 (count: N)
  → mean samples=312 ticks (~31.2s)
Tracker mode initialized.
Generation 0: pop=5000, scenarios=120
  best=...  avg=...  ...
```

### 3. Watch fitness progress

The smoke-test signal: **fitness curve does *something* — descends, plateaus, or fails informatively**.

Plausible outcomes:
- **Descending** (fitness improves over gens): the loop is closing. Promising.
- **Plateau early** (fitness flat-lines after ~5 gens): controller can't break out of initial random behavior. Could mean: arena too tight, p_crash too aggressive (despite curriculum), or tracker-mode fitness landscape pathologically flat.
- **Crashing fast** (most scenarios terminate within 1-2 seconds): `p_crash` might be firing too often despite curriculum, or arena is too tight, or initial chase pose puts the chase craft outside the arena.
- **NaN / Inf** (fitness diverges): math bug in projection or trail-rabbit; check the latest M5 / M7 contract test outputs.

### 4. Inspect the M2 dmp

Tracker-mode best-of-gen is uploaded to S3 with run-id specific to this smoke run. Find it:
```bash
aws s3 ls s3://<bucket>/autoc-storage/autoc-<NEW-RUN-ID>/
```

Pull a recent gen's dmp:
```bash
aws s3 cp s3://<bucket>/autoc-storage/autoc-<NEW-RUN-ID>/gen49.dmp /tmp/smoke-gen49.dmp
```

Quick sanity check via the per-tick dmp extractor (M11a):
```bash
build/aircraft_state_extractor --in /tmp/smoke-gen49.dmp --out /tmp/smoke-gen49.csv
head -5 /tmp/smoke-gen49.csv  # confirm columns include BEACON_L_X_NOW, BEACON_L_CEP_NOW, etc.
```

### 5. Renderer playback (the 4th deliverable)

```bash
build/renderer -i autoc-tracker.ini -k autoc-storage/autoc-<NEW-RUN-ID>/gen49.dmp
```

**Expected display**:
- 3rd-person view: chase craft + target craft both visible, target wingtip beacons rendered as colored points.
- Camera-POV mini-panel (in the HUD overlay area near throttle/control state): two dots (left / right beacons) at their projected screen positions; CEP visualized as ellipse spread per D15 v1 commit.
- Per-tick scrub controls work: pause / step-forward / step-backward.
- Switch to 1st-person camera-POV mode: full-screen render from chase camera with beacons visible.

**Smoke-test green** = the renderer animates the M2 result end-to-end without errors. Doesn't require the controller to be *good* — just that the loop closes.

### 6. Capture findings

The smoke-test deliverable is qualitative + the four boxes checked, not a benchmark number. Record:
- Source dmp identifier, scenario index, M2 run-id.
- Fitness curve shape (descending / plateau / pathological).
- Renderer screenshots (3rd-person + camera-POV).
- Any sentinel events: arena egresses, crash-hull strikes, NaN propagations, build issues.

Write a short `eval-results/030-smoke-<date>/SMOKE_REPORT.md` with the above. M10 is sim-only; `flight-results/` is reserved for real-flight artifacts.

### 7. Decide what's next

Per [plan.md M11+](./plan.md):
- If smoke green and signal interesting → **continue to M11a (per-tick dmp extractor) + M11b (eval fitness Bug 2 fix) + M11c (tracker-specific analytics)**. These are the post-smoke ramp items proposed for "030 done line."
- If smoke green but signal weird (e.g., fitness plateaus immediately) → **debug** using the per-tick extractor + renderer error bars. The bug is somewhere in M3–M9; the analytics tooling lets you narrow it down.
- If smoke red (loop doesn't close) → **fix the failing milestone**. The granular checkpoints (M1 through M9) make this localized rather than a forest-fire debug.

## What success looks like

Per D13 the four checkbox items:
- ✓ Autoc loaded an M1 dmp and ran tracker mode against it.
- ✓ Single scenario per `autoc-tracker.ini`.
- ✓ Results saved (M2 dmp + per-gen logs).
- ✓ Renderer animated the M2 result.

That's the floor. "030 done" sits some distance above it; plan-research has proposed `030 done = M10 + M11a + M11b + M11c`.

## Citations

- 030 spec D13 (smoke-test framing)
- 030 spec FR-001, FR-008, FR-008b, FR-011, FR-015, FR-016 (the v1-locked-in commitments this runbook exercises)
- [plan.md](./plan.md) milestones M1–M9 (smoke-test prerequisites)
- [research.md](./research.md) R1–R10 (architectural decisions baked into v1 defaults above)
