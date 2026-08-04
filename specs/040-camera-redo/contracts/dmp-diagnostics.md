# Contract — dmp Diagnostics

**Scope**: what the M2 dump records so the new perception model can be diagnosed after a run (FR-028),
and the boundary that keeps diagnostics from becoming controller inputs (FR-029).

---

## Convention

Follow the practice documented in-code at `include/autoc/rpc/protocol.h:428-447`:

- **append at the end of the v≥2 block**
- **no `CEREAL_CLASS_VERSION` bump**
- **old dmps are orphaned** — they fail to parse rather than mis-parsing into plausible-but-wrong values

Orphaning is what satisfies Principle V's fail-loud requirement. A silent mis-parse producing subtly wrong
flight behaviour weeks later is precisely the failure mode the principle exists to prevent; a hard parse
error prompts an explicit operator decision.

**The M1 source format is untouched.** M1 dmps remain readable and no rebake is triggered — this is what
keeps 040 parallel to flight-test work (SC-009).

---

## Fields added to `BeaconObservation`

| Field | Type | Purpose |
|---|---|---|
| `raw_x_px`, `raw_y_px` | int16 | pixel indices — the quantisation actually applied. **int16 because centred coordinates reach ±160, exceeding int8.** Replaces `raw_x_int8` / `raw_y_int8`, which are removed outright (Principle III — clean cut, no shim) |
| `raw_margin` | int8 | correlation-margin proxy; the quantity a real front-end reports (`marginA`/`marginB` on the 031 bench) |
| `lock_state` | uint8 | tracking state — searching / acquiring / tracking / holding |

All are cereal byte-format ⇒ `// raw-ok: <reason>` annotated at the declaration site (Principle VI).

**Safe because tracker mode is desktop-only** — `gather_tracker_inputs` is `#ifndef ARDUINO` and tracker
mode has never shipped to firmware, so the int8→int16 widening carries no embedded exposure.

---

## The diagnostics boundary

| Recorded | Fed to the controller |
|---|---|
| bearing (radians) | ✅ yes |
| quality | ✅ yes |
| pixel indices | ❌ diagnostic only |
| correlation margin | ❌ diagnostic only |
| tracking state | ❌ **diagnostic only** (FR-017b) |

The tracking state machine is internal. Recording it is what makes the model reviewable; feeding it would
change the interface, grow the 58-input vector, and move inference out of the network where the operator
wants it.

---

## Already available — no new fields required

Verified present, so propeller and range instrumentation costs nothing:

| Quantity | Source |
|---|---|
| throttle command / applied | `DebugSample.throttleCommand`, `.throttleSim` |
| chase position | `DebugSample.position` |
| target position | `CopiedTargetSample.position` |
| ⇒ chase-to-target range | derived from the two |

---

## Consumers to update

| Consumer | Change |
|---|---|
| `tools/dmp_dump.cc` | emit the new diagnostics |
| `tools/renderer.cc` | POV panel in radians; effective FOV including obstructed regions; quality regime visible (FR-030) |
| `tests/tracker_dmp_roundtrip_tests.cc` | round-trip the new fields |

**Renderer note**: the reticle ticks and FOV overlay currently assume the ±1 normalised convention. That
convention is retired — the panel must be rescaled to radians and must draw the *effective* field, not the
nominal rectangle.
