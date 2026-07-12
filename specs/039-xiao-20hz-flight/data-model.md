# 039 Data Model

Entities for the versioned binary flight log, the latency decision memo, and the acceptance
comparison report. Byte-exact wire format lives in
[contracts/flight-log-format.md](contracts/flight-log-format.md); this file defines the logical
records and validation rules.

## 1. Flight candidate

| field | value |
|---|---|
| weight source | pinned t5 elite: `autoc-m1/autoc-9223370253553029228-2026-07-06T01:35:46.579Z/` gen 800 (`retain=keep`; operator-confirmed 2026-07-11 — supersedes the 07-02 run id in earlier drafts) |
| topology | 37 → 32 → 16r → 3 (2051 weights) |
| replacement rule | ONLY via FR-005: operator amends sim latency model → retrain → new elite pinned + recorded in outcome.md → re-verified per FR-002 before flash |

## 2. Flight-log file (per flight, on QSPI flash)

```
FileHeader (once) → { EngageHeader → TickRecord × N }* → (event records interleaved)
```

### 2.1 FileHeader

| field | type | notes |
|---|---|---|
| magic | u32 | file identity ('A','F','L','1'-style; exact value in contract) |
| format_version | u8 | **Constitution V**: reader loud-fails on unknown version |
| firmware_id | u8[8] | first 8 bytes of SHA-256 of the generated `nn_program_generated.cpp` (computed at regen time, baked as a constant) — ties log to the flashed firmware |
| weight_id | u8[8] | first 8 bytes of SHA-256 of the extracted weight file (`nn_weights.dat`) — candidate provenance |
| tick_ms | u16 | control tick (50 at 20 Hz) — makes the log rate self-describing |
| scale_table_crc | u32 | guards decoder/scale-table agreement (scales are format-versioned) |

### 2.2 EngageHeader (once per span activation)

| field | type | units | notes |
|---|---|---|---|
| engage_timestamp_ms | u32 | ms boot-relative | span start |
| span_id | u16 | — | flight-unique, monotonically increasing |
| arena_origin_ned[3] | f32×3 | m, NED rel. arm origin | the engage point; FR-001 re-centering provenance |
| arena_floor_z / ceiling_z | f32×2 | m NED | RESOLVED values after the pure ±K rule `floor=z_e+K`, `ceil=z_e−K` (K=47.5; no min-elevation clamp this phase) — logged resolved, not just derivable |
| path_index | i16 | — | selected path at engage |

### 2.3 TickRecord (every control tick while engaged)

Quantization: all `[-1,1]`-bounded NN values → i16 (scale 32767); distances/rates → i16 with
per-field scale from the contract's scale table. Raw types below are wire format
(`// raw-ok: hardware byte layout` domain — desktop reader converts to `gp_scalar`).

| group | fields | count | quantized size |
|---|---|---|---|
| framing | record type (u8) | 1 | 1 B |
| timing | tick_timestamp_ms (u32), tick_counter (u16) | 2 | 6 B |
| NN inputs — target dir history | target_x[6], target_y[6], target_z[6] (unit-vector components) | 18 | 36 B |
| NN inputs — range | dist[6] (raw metres, scale 1/32 m — what the NN actually consumes), closing_rate | 7 | 14 B |
| NN inputs — self state | quat[4] (q_EB), airspeed, gyro[3] (rad/s) | 8 | 16 B |
| NN inputs — 038 situational | dist_to_boundary (tanh), inward_body[3] | 4 | 8 B |
| NN outputs | roll, pitch, throttle (tanh) | 3 | 6 B |
| aux | recurrent_reset flag (u8), path_index (i8 = selected path 0-5), rc_sent[3] (u16×3), state_valid (u8) | 6 | 9 B |
| **total** | | **48 fields + framing** | **96 B/tick** (static-asserted ≤ 100) |

Validation rules:

- The 37 input fields MUST be the values actually fed to the NN this tick (post-gather, post-history
  shift) — honest-recording; no re-derivation on decode.
- `recurrent_reset = 1` exactly on the first tick after span activation (warm-up marker).
- `tick_counter` gaps ⇒ decoder reports dropped ticks (never silently interpolates).
- Budget check: 96 B × 20 Hz × 480 s (2×4 min) ≈ **0.92 MB** ≤ 2.04 MB usable — ~55% headroom.

### 2.4 EventRecord (console-class events into the log)

Sparse, small: event code (u8) + timestamp + optional payload (arm/disarm, engage/disengage,
fetch-timeout, log-buffer-overflow, heartbeat snapshot). Console itself prints only these +
~2 Hz heartbeat (FR-014). Includes a **span-summary event** at disengage carrying the loop-health
counters (ticks, overruns, resyncs, maxLate, avgLate — today's `loopStats`, msplink.cpp:163-168)
and the MSP pipeline stats (fetch/eval/send mean/95/max) — the latency memo's flight-side numbers
come straight from these records.

## 3. Latency decision memo (US2 deliverable — contracts/latency-memo.md)

| section | content |
|---|---|
| measured @10 Hz era | fetch/eval/send mean/95/max from flight-20260503/0517 (research.md R1 table) |
| measured @20 Hz bench | same components re-measured on regenerated firmware (post-unroll; at current + raised baud) |
| modeled | crrcsim COMPUTE_LATENCY (30 ms) + servo v2 (latch 0–20 ms, slew 24.2 u/s) mechanism |
| gap + recommendation | side-by-side; recommendation ONLY (no auto-trigger per clarification #4) |
| decisions (operator) | model stands / amend (+ retrain plan); local IMU verdict (FR-006) |

## 4. Sim-vs-real comparison report (US5 acceptance — SC-005)

| field | source |
|---|---|
| per-axis dCtrl ⟨\|Δu\|⟩: flight vs sim | flight: decoded TickRecords; sim: same elite's per-axis report (038 tooling) |
| per-axis amplitude ⟨\|out\|⟩: flight vs sim | same |
| pass band | each axis within ±25% of sim value (both metrics) |
| bang-bang qualitative | saturation % per axis vs the 2026-05-17 10 Hz-era flight |
| verdict | SC-006 documented statement + residual gaps |

## 5. State transitions

```
DISARMED → ARMED → [span: ENGAGE (reset hidden state, re-center arena, write EngageHeader)
                     → TICK×N (TickRecord each) → DISENGAGE] × M → DISARMED
Log lifecycle: ground ERASE:ALL → initialized → append-only across ≤2 flights → BLE download → clear
```
