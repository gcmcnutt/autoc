# Contract — the 20 Hz record stream

**Consumers**: `beacon_display.py` (JSON), `tools/score` (envelope metrics), 041 (later, via 042-E2
transport), any post-hoc analysis. **Constitution V applies**: explicit version, fail-loud readers.

## Framing

Fixed-size little-endian struct, one per control tick, emitted **every tick without exception** — a tick
with nothing tracked emits a record with `n_tracks = 0`, never silence. Consumers detect loss by `seq` gap.

```
offset  size  field
0       4     magic            0x42434E31  ("BCN1")
4       2     format_version   starts at 1
6       2     header_bytes     for forward skip
8       8     t_us             frame-clock time of the tick
16      4     seq              monotonic record counter
20      4     tick_index       control-tick index since start
24      1     n_tracks         populated entries in tracks[]
25      1     n_slots_used     bank occupancy (diagnostic)
26      2     reserved
28      4     deadline_margin_us   SIGNED; negative = deadline missed
32      8     build_id
40      8     config_hash
48      -     tracks[MAX_TRACKS]   fixed count from config; unused zeroed, VALID clear
```

Per-track layout is `Track` from `data-model.md` §2, fixed-point as declared there.

## Read-side contract (binding)

A reader MUST verify `magic`, then `format_version`. On mismatch it MUST **fail loudly**, naming both the
artifact's version and its own, and MUST NOT attempt partial interpretation. Unknown-but-newer versions are
an error, not a best-effort read.

## Semantics consumers may rely on

- **Fixed shape every tick.** Never a gap; absence is expressed by `VALID` clear, not by a missing record.
- **`x_pred/y_pred` target the NEXT tick**, not this one. A consumer steering at tick N uses the record
  delivered before tick N, whose prediction is *for* N.
- **`age_ms` is informational** (how stale the measurement behind the prediction is), not a constraint.
  The constraint is `deadline_margin_us` (§11.1).
- **`VALID` and `MEASURED_FIX` are the two scored bits** (spec §3.1). Everything else is diagnostic and
  may gain meanings within a `format_version`; **no bit is ever repurposed** without a version bump.

## JSON projection

`emit_json.c` renders the same fields as JSON lines for `beacon_display.py`, preserving the existing
convention: **code A = PORT (red), code B = STARBOARD (green)**. The JSON is a *projection*, never a
separate source of truth — it is generated from the binary record, not alongside it.
