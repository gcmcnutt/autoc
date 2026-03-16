# AutoC Backlog

**Last Updated**: 2026-03-16 (consolidated from ~/old/GP/specs/BACKLOG.md + xiao/TODO.md)

## Legend

- `[NEXT]` - High priority, ready to start
- `[DEFERRED]` - Lower priority, will revisit
- `[BLOCKED]` - Waiting on dependencies

---

## NN Training Signal (014)

Active work tracked in [specs/014-nn-training-signal/tasks.md](014-nn-training-signal/tasks.md).

### [NEXT] Sigma Floor (Phase 6)
- Enforce minimum mutation sigma (e.g., 0.05) to prevent search freeze
- Quick win — band-aid for stalled runs while better optimizers are developed

### [NEXT] Curriculum Ramp (Phase 7)
- Progressive difficulty: start with fewer wind scenarios, expand over generations

### [DEFERRED] Fitness Decomposition (Phase 8)
- Minimax/percentile aggregation instead of sum-of-errors
- Pressures NN to fix worst behaviors first

### [DEFERRED] sep-CMA-ES (Phase 9)
- Replace isotropic Gaussian mutation with CMA-ES
- Learns covariance structure of 531-weight space

### [DEFERRED] Per-Timestep Streaming (Phase 10)
- Enables per-segment credit assignment and checkpoint cloning

### [DEFERRED] Segment Scoring (Phase 11)
- Per-segment credit assignment with delta-based scoring
- Score by error reduction relative to initial condition

### [DEFERRED] Checkpoint/Resume (Phase 13)
- Dump full state each generation for crash recovery
- Long runs (~9 hours) are vulnerable to crashes/OOM

---

## Infrastructure

### [DEFERRED] Make pathgen.h Portable for Embedded
- Single pathgen.h that works on both desktop and embedded
- Conditional compilation for std::vector vs fixed arrays
- Wait until path system stabilizes

### [DEFERRED] Output Cleanup (T066-T068)
- OutputDir config key
- Auto-created run subdirectory
- Clean eval prefix naming

### [NEXT] Batch and Cache Deterministic Scenarios
- With 36+ wind scenarios, serializing the full table per individual causes ~4x throughput hit
- Send scenario table once at generation start, cache in crrcsim
- Reduces per-eval serialization from O(scenarios x individual) to O(individual)

---

## Robustness

### [DEFERRED] Wind Speed Variation
- Currently only wind direction varies; speed is fixed at base value
- Add windSpeedOffset/windSpeedFactor to ScenarioMetadata
- Trains robustness to different wind regimes (complements turbulence)

### [DEFERRED] Demetic Mode Elite Preservation
- Best fitness jumps wildly when elite re-evaluated on single scenario
- Revisit when fitness ramp stabilizes

---

## Controller Architecture

### [DEFERRED] Upper-Level Intercept Director
- Higher-level controller for initial maneuver before engaging track mode
- Depends on entry variation training reaching maturity

### [DEFERRED] Future State Predictor NN
- Secondary NN that predicts short-horizon future sensor values
- Depends on validating temporal history inputs first

---

## Embedded / Hardware

### [NEXT] Training Record Consistency & Provenance
- S3 provenance gaps: several locations print S3 key without bucket/profile
- Fitness formatting inconsistency (exponent vs fixed-point)
- Add bucket+profile to provenance, use consistent formatting

### [NEXT] Export RC Commands to Xiao Log
- Log RC commands throughout entire flight for full playback visualization
- Location: xiao/src/msplink.cpp

### [DEFERRED] Xiao Safety Checks Pre-Arm
- Ensure mode flip is safe: RC failsafe, RC disarm, hold/RTH should disarm co-processor
- msp_status should send array of unsigned32 as packed bit field (currently 32 bits)

### [DEFERRED] Speed Up Logfile Download
- BLE download may be over-bucketed from prior troubleshooting

### [DEFERRED] GP to Autoc in INAV via Controls
- Selector and activate mechanisms

---

## Visualization

### [DEFERRED] Blackbox Rendering Improvements
- Select path + blackbox log for comparisons, FPV mode

### [DEFERRED] CRRCSim Display Dependency
- CRRCSim requires valid DISPLAY even in headless mode

### [DEFERRED] Clean CRRCSim Shutdown
- Polling loop for keepalive; clean exit when autoc exits

---

## Code Cleanup

### [DEFERRED] Memory Leak Investigation
- Small memory leak exists in autoc

### [DEFERRED] Cross-Platform Verification (T211)
- Train on aarch64, pull repo on x86
- Build and run renderer/nnextractor/eval against aarch64 S3 objects
- Validates cereal binary portability end-to-end
