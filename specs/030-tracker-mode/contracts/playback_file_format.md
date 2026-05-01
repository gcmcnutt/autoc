# Contract: Playback file format (`.crrclog`)

**Producer**: `tools/dmp_to_playback` converter (NEW for 029)
**Consumer**: `crrcsim/src/mod_robots/CRRC_AirplaneSim_Playback` (existing crrcsim class — unchanged)

**Format**: Existing crrcsim `.crrclog` binary format. No new format invented for 029. Per [research.md R1](../research.md#r1--crrcsim-playback-file-format-exact-byte-layout), reusing the format means zero crrcsim-side changes for the reader path.

**Native byte order**: little-endian (per crrcsim convention; documented at `crrcsim/src/mod_robots/robotfile.h:36`).

## File structure

```
+------------------------------------------+
| ASCII XML header                          |   Root element MUST be <CRRCSim_record>
| - airplane.file = hb1_streamer.xml        |   v1: same model as training craft
| - airplane.graphics = ...                 |
| - scenery.file = ...                      |
| - wind.* (seed, direction, etc.)          |   Inherited from source scenario
| - description (free text)                 |
+------------------------------------------+
| 0x0A (single newline byte)                |   Required separator (consumer skips one byte)
+------------------------------------------+
| Tagged binary record stream               |
| (one record after another, until EOF)     |
+------------------------------------------+
```

## Record types

### 0x00 — Position/attitude (load-bearing for tracker mode)

| Offset | Size | Type | Field | Notes |
|--:|--:|---|---|---|
| 0 | 1 | char | tag = 0x00 | |
| 1 | 8 | double (LE) | timestep | Seconds since previous 0x00 record. Writer stores `dt × multiloop` |
| 9 | 4 | float (LE) | pos_x | World-frame X (crrcsim units: ft, X-North) |
| 13 | 4 | float (LE) | pos_y | World-frame Y (Y-East) |
| 17 | 4 | float (LE) | pos_z | World-frame Z (Z-Down) |
| 21 | 2 | int16 (LE) | euler_phi_scaled | phi × ROBOT_EULER_TO_INT16 (= 32767/(2π)) |
| 23 | 2 | int16 (LE) | euler_theta_scaled | theta × ROBOT_EULER_TO_INT16 |
| 25 | 2 | int16 (LE) | euler_psi_scaled | psi × ROBOT_EULER_TO_INT16 |
| **27** | | | **end of record** | Total: 1 tag + 20 body bytes |

**Python writer template**:
```python
struct.pack('<B d 3f 3h', 0x00, timestep, pos_x, pos_y, pos_z, phi_scaled, theta_scaled, psi_scaled)
```

**Cadence**: one record per FDM tick. Constant timestep matching crrcsim's `Global::dt × multiloop`.

### 0x01 — Control inputs (NOT USED in tracker mode)

Reader has no case for 0x01 in `fdm_playback.cpp:46-94`. Consumer would error. **Converter MUST NOT emit 0x01 records.**

### 0x02 — F3F sync marker (NOT USED in tracker mode)

Body: 4 bytes `int32 marker_id`. Used for F3F-style sync points (irrelevant for tracker mode). Reader passes through state machine but ignores. **Converter MAY omit; if emitted, MUST be a 4-byte int32.**

### 0x03 — Variable-length XML metadata (recommended for 029)

| Offset | Size | Type | Field |
|---|---|---|---|
| 0 | 1 | char | tag = 0x03 |
| 1 | variable | ASCII | XML body |
| (end) | 1 | char | trailing 0x0A |

Used to embed scenario provenance:

```xml
<scenario_metadata
    source_run_id="more-rnn3-2026-04-26T..."
    source_gen="600"
    source_scenario_index="42"
    path_variant="2"
    wind_variant="11"
    wind_seed="12345"
    craft_variation="0"
    entry_cone_phi_deg="3.4"
    entry_cone_theta_deg="-1.7"
    entry_roll_deg="2.1"
    entry_speed_factor="1.05" />
```

**Cadence**: ONE per file, at end-of-file (after the last 0x00 record).

## Validation rules

The converter (producer) MUST:

| Check | Failure mode |
|---|---|
| XML root element is `<CRRCSim_record>` | Reader at `fdm_playback.cpp:143-146` errors out |
| Header is followed by exactly one 0x0A byte | Reader skips first post-header byte unconditionally |
| All numeric fields little-endian | Reader uses native memcpy; non-LE corrupts silently |
| 0x00 records have monotonic-non-decreasing accumulated timestep | Consumer interpolation breaks with negative/zero deltas |
| Phi / theta / psi within [-π, +π] before scaling | int16 overflow on out-of-range angles |
| File ends cleanly (no truncated records) | Reader EOF detection at `fdm_playback.cpp:118-126` |

## Consumer (crrcsim) — invariant preservation

`CRRC_AirplaneSim_Playback` (`crrcsim/src/mod_robots/fdm_playback.cpp`) consumes the file unchanged from existing behavior. 029 introduces no consumer-side changes. Multi-aircraft instantiation per scenario uses the existing `Robots::AddRobot()` API.

## Test surface

`tests/dmp_to_playback_tests.cc` (NEW for 029):

| Test | Assertion |
|---|---|
| `Roundtrip_KnownDmp_PlaybackMatches` | Convert a fixture `.dmp` to `.crrclog`, then have a Python parser read back position/attitude per tick. Each parsed value matches the source dmp's `aircraftStateList[s][t].pos / quat` within float-precision tolerance. |
| `Determinism_SameDmpSameOutput` | Convert the same `.dmp` twice with identical args; resulting `.crrclog` files are byte-identical. |
| `XmlRootValidation_HeaderRequired` | Output file's XML header parses and root element is `CRRCSim_record`. |
| `EulerScaling_RoundTrip` | A known phi/theta/psi roundtrips through int16 scaling within ~0.01° (limit of int16 quantization). |
| `EmptyTrajectory_HandledGracefully` | A source dmp with a crashed scenario (truncated trajectory) produces a valid `.crrclog` with the partial trajectory — does not crash the converter. |

## Open contract decisions (resolve in `/speckit.tasks`)

1. **Crashed-scenario handling** (FR-013): truncate `.crrclog` at crash tick, OR omit the scenario entirely from the library. Recommendation: truncate + flag in `library_metadata.json`; gives the operator visibility into degraded scenarios without forcing a non-245 scenario count.
2. **Provenance XML schema details**: which fields exactly land in the 0x03 record. Above is a starting point; finalize during implementation.
