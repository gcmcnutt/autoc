# 042 — Phase 1 data model

Entities are C11 PODs in `core/`. **No allocation after init**: every collection below is a fixed-capacity
array sized at config load. Field types name their fixed-point domain (R2); `q8` means 8 fractional bits.

---

## 1. `FrameView` — the only thing `core/` knows about a camera

| field | type | notes |
|---|---|---|
| `data` | `const uint8_t *` | not owned; valid only for the call |
| `stride`, `w`, `h` | `uint16_t` | 640×400 or 640×200 |
| `t_us` | `uint64_t` | **source-supplied**, never `clock_gettime` inside `core/` (R3) |
| `seq` | `uint32_t` | monotonic frame counter; gaps are an error, not a warning |
| `exposure_us` | `uint32_t` | from libcamera metadata; **normalises the sample series** (spec §4) |
| `gain_q8` | `uint16_t` | ditto |

**Rule**: identical `FrameView` sequences must produce identical outputs whether they came from libcamera
or from a replay file. This is the parity contract.

---

## 2. `Track` — one confirmed beacon

| field | type | notes |
|---|---|---|
| `code_id` | `uint8_t` | 0 = A/PORT, 1 = B/STARBOARD |
| `x, y` | `int32_t` q8 | M2 grid px (320×200 @ 0.304°/px, centre (0,0), +x right / +y down) |
| `vx, vy` | `int32_t` q8 | px/s — the alpha-beta velocity state; the "momentum" of spec §2.1 |
| `x_pred, y_pred` | `int32_t` q8 | position at the **next** control tick |
| `cep` | `uint16_t` q8 | px |
| `q` | `uint16_t` q8 | slow, full-word correlation quality (scale-free) |
| `lock_health` | `uint16_t` q8 | **fast, chip-rate** decision-directed statistic (spec §2.6) |
| `extent` | `uint16_t` q8 | `q_fine / q_coarse` — point-source vs glitter-path (spec §9) |
| `scintillation` | `uint16_t` q8 | q variance over a window (spec §9) |
| `chip_hz` | `uint32_t` q8 | per-track DPLL state |
| `chip_phase` | `uint8_t` | 0..30 |
| `t_int_chips` | `uint8_t` | current adaptive integration length (spec §4) |
| `scale` | `uint8_t` | coarse / medium / fine (spec §2.2) |
| `flags` | `uint16_t` | see §5 |
| `age_ms` | `uint16_t` | staleness of the measurement behind the prediction — **reported, not constrained** (§11.1) |

### Track lifecycle

```
                 acquisition emits proto-track
                            |
                            v
  [CANDIDATE] --lock_health sustained--> [CONFIRMED] --+
      |                                       |        | guard partner allocated
      | q never rises / slot evicted          |        v
      v                                  chip re-affirmation fails
   [DEAD]                                     |
                                              v
                                          [HOLD] --bounded: age>150ms or CEP>3px--> [DEAD]
                                              |
                                        re-affirmed
                                              |
                                              v
                                        [CONFIRMED]
```

**HOLD is evidence-bounded, not a countdown.** The 031 prototype's "ride 6 low-q reports" is removed
(spec §2.6). The HOLD→DEAD bound is exactly §3.1's validity bound, so the metric and the state machine
cannot drift apart.

---

## 3. `BankSlot` / `Bank` — 16 movable correlators

| field | type | notes |
|---|---|---|
| `slots[16]` | `BankSlot` | fixed capacity; **eviction by priority, never realloc** |
| `role` | `uint8_t` | `PRECISION` \| `GUARD` \| `CANDIDATE` |
| `partner` | `int8_t` | index of the paired slot; every CONFIRMED track owns a `PRECISION`+`GUARD` pair (spec §2.4) |

**Validation rules**
- A `GUARD` may exist only with a live `PRECISION` partner; orphaned guards are freed the same tick.
- Two CONFIRMED tracks sharing a `code_id` trigger the **mirror-pair rule** (spec §9): the geometrically
  *upper* one keeps `CONFIRMED`; the lower is flagged `MULTIPATH_SUSPECT` and **kept, not deleted**.
- Eviction priority: `CANDIDATE` < `GUARD` < `PRECISION`; within a class, lowest `lock_health` goes first.

---

## 4. `Record` — the 20 Hz wire artifact (**versioned; Constitution V**)

Fixed size, fixed shape **every tick, never a gap**. See `contracts/record-wire-format.md`.

| field | type |
|---|---|
| `magic`, `format_version` | `uint32_t`, `uint16_t` |
| `t_us`, `seq`, `tick_index` | `uint64_t`, `uint32_t`, `uint32_t` |
| `n_tracks`, `n_slots_used` | `uint8_t`, `uint8_t` |
| `deadline_margin_us` | `int32_t` | signed: negative = **missed** (§11.1) |
| `tracks[MAX_TRACKS]` | `Track` | unused entries zeroed with `VALID` clear |
| `build_id`, `config_hash` | `uint64_t`, `uint64_t` |

**Read-side contract**: a reader MUST check `magic` + `format_version` and **fail loudly** naming both the
artifact's version and its own. No silent coercion (Constitution V).

---

## 5. `flags` bit meanings

| bit | name | set when |
|---|---|---|
| 0 | `VALID` | this track slot carries meaning this tick |
| 1 | `LOCK` | CONFIRMED, measurement-backed this tick |
| 2 | `HOLD` | extrapolating, still inside the §3.1 bound |
| 3 | `EXTRAPOLATED` | position came from prediction, not measurement |
| 4 | `MULTIPATH_SUSPECT` | lower member of a same-code mirror pair (spec §9) |
| 5 | `SATURATED` | peak railed → centroid switched to flat-top estimator (spec §5) |
| 6 | `MEASURED_FIX` | chip re-affirmation covered this tick — **this is §3.1's second metric** |
| 7 | `AGC_SETTLING` | exposure/gain changed within the integration window |

`VALID` and `MEASURED_FIX` are the two bits the envelope scorer counts. Everything else is diagnostic.

---

## 6. `Config` — loaded by `config.c`, **errors on missing keys** (Constitution VII)

Grouped: `[camera]` mode/fps/exposure bounds · `[code]` chip rates, N, code table · `[bank]` slot count,
scale extents, alpha-beta gains, thresholds · `[agc]` the three controllers' bounds · `[record]` sink, path,
ring seconds, trigger conditions · `[sched]` acquisition cost model for R3's virtualisation.

**No in-C defaults for any of these.** A missing key is a startup error naming the key. Counters and
sentinels that legitimately zero-init are annotated `/* default-ok: <reason> */`. `config_hash` is computed
over the resolved config and stamped into every record and recording.

---

## 7. `RecordingContainer` — raw frames on disk (**versioned; Constitution V**)

Header (magic, format version, sensor mode/geometry, `build_id`, `config_hash`, start `t_us`) followed by
length-prefixed frame records (`seq`, `t_us`, `exposure_us`, `gain_q8`, payload). Preallocated,
`O_DIRECT`, large aligned writes (R9). See `contracts/recording-container.md`.

**Modes**: `continuous` (flight host — Pi 5 + NVMe) · `ring` (bench host — bounded RAM, triggered dump) ·
`burst` (N contiguous frames every M — proves the toolchain per spec §16.4, and unlike uniform decimation
each burst is a replayable word).
