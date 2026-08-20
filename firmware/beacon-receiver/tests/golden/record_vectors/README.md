# Golden byte vectors — the 20 Hz record contract

These files **are** the contract (`specs/042-camera-receiver/contracts/record-wire-format.md`). Each is
exactly 832 bytes: a 64-byte header plus 16 × 48-byte track slots.

| vector | what it pins down |
|---|---|
| `v1_empty.bin` | the common case — a tick with nothing tracked, emitted as a record with `n_tracks = 0` and **never as silence**, so consumers can detect loss by `seq` gap |
| `v1_two_tracks.bin` | one track per code, with negative `x`/`vy`, a **negative** `deadline_margin_us`, both clocks populated, and the `MULTIPATH_SUSPECT` / `HOLD` / `EXTRAPOLATED` flags |
| `v1_saturated.bin` | `SATURATED` + `AGC_SETTLING`, and `deadline_margin_us == 0` (exactly on the deadline is **not** a miss) |

## How to use them from another implementation

Read the bytes, decode with **your own** codec, and assert the field values documented in the contract.
Do not include this project's `record.h` — that is the whole point (plan.md §Contracts at arm's length).
A shared header only delivers its `static_assert` guarantee if both sides compile the same header, which
is what breeds the `#ifdef` thicket; the ATtiny412's 8-bit `int` settles it alone.

## Regenerating

Only on a `format_version` bump:

```bash
cmake --build build --target beacon_gen_record_vectors
build/firmware/beacon-receiver/beacon_gen_record_vectors \
    firmware/beacon-receiver/tests/golden/record_vectors
```

**Never regenerate to make a failing test pass.** A failure here means the layout changed without a
version bump, or a codec drifted — both are the mechanism doing its job. `tests/golden/test_record_vectors.c`
states the expected field values independently of the generator, so the two must agree.
