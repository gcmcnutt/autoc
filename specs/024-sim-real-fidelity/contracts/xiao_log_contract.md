# Contract: Xiao Flight Log

**Reader role**: `cmd_response_scatter_lagged.py`, `gyro_vs_quat.py`,
`sensor_self_check.py` (WI1), renderer `-x` mode.
**Producer**: xiao `msplink.cpp` writes flash and serial log lines.

## Format

Text lines. Each line starts with `#<seq> <xiao_ms> <inav_ms>` then an
indicator letter `i`, then a named record type, then space-separated fields.

**Record types** used by analysis:

- `Nav State` — per sample (~10 Hz)
- `NN` — per NN evaluation (every 10 Hz tick; skipped during engage-delay
  window)
- `NN Control: Switch enabled/disabled` — engage state transitions
- `Boot banner`, `Config dump`, `Telemetry stats` — other; ignored for audit

## `Nav State` line format

```
#<seq> <xiao_ms> <inav_ms> i Nav State: pos_raw=[x,y,z] pos=[x,y,z] vel=[vx,vy,vz] quat=[w,x,y,z] gyro_raw=[px,py,pz] armed=Y fs=N servo=Y autoc=Y rabbit=Y path=N
```

| Field | Units | Frame | Transform |
|---|---|---|---|
| `xiao_ms` | ms | — | `millis()` at packet processing; local Xiao clock |
| `inav_ms` | ms | — | INAV timestamp from MSP packet |
| `pos_raw` | m | NED world (raw, no offset) | (none — already canonical) |
| `pos` | m | NED virtual (after offset) | (none) |
| `vel` | m/s | NED world | (none) |
| `quat` | unit quat | claimed q_EB aerospace | (WI5 verifies) |
| `gyro_raw` | raw INAV | body | debugging only |
| `armed`, `fs`, `servo`, `autoc`, `rabbit` | Y/N | — | flags |
| `path` | int | — | active path index |

## `NN` line format

```
#<seq> <xiao_ms> <inav_ms> i NN: idx=<N> tX=[..6..] tY=[..6..] tZ=[..6..] d=[..6..] cr=<cr> q=[w,x,y,z] as=<as> g=[p,q,r] out=[pt,rl,th] rc=[r,p,t]
```

`idx` is the NN scenario/run index. `tX/tY/tZ/d` are 6-element history
arrays (oldest first to newest, or newest first — TBD, document with WI1).

| Field | Units | Frame | Canonical |
|---|---|---|---|
| `tX[0..5]`, `tY[0..5]`, `tZ[0..5]` | unitless | body FRD direction cosines | TARGET |
| `d[0..5]` | m | scalar distances | TARGET distance history |
| `cr` | m/s | scalar closing rate (+ = approaching) | — |
| `q` | unit quat | q_EB aerospace (claimed) | QUAT |
| `as` | m/s | groundspeed | — |
| `g[0..2]` | rad/s | aerospace RHR (p, q, r) | GYRO |
| `out[0..2]` | [−1, +1] | pitch, roll, throttle | CMD |
| `rc[0..2]` | µs PWM 1000–2000 | — | RC payload to INAV |

## Timestamp semantics

- `xiao_ms` = `millis()` at `mspUpdateState()` call (msplink.cpp:339).
- `inav_ms` = `state.autoc_state.timestamp_us / 1000` (from INAV's packet).
- **Not synchronized.** Xiao and INAV run on independent clocks with ~20–30 ms
  of round-trip transit. Xiao log engage span detection should use
  `xiao_ms`; for cross-source join against blackbox, use `inav_ms`.

## Cadence

- Target: 100 ms between `NN` lines.
- Observed: median 101 ms, range 92–122 ms. Soft-polling loop (`millis()` +
  threshold); no hardware timer.
- Not drift-free but stable.

## Error modes

- **Missing record types** — tolerated; reader skips unparseable lines.
- **Malformed field syntax** — reader logs + skips.
- **Engage span detection** — transitions on `autoc=Y/N`; spans are consecutive
  runs of `autoc=Y`. Last `Nav State` in a span sets `end_t`.

## Current drift from canonical (pre-WI3)

- `quat=[..]` in log is **claimed** to be q_EB aerospace by msplink but
  `gyro_vs_quat.py` analysis shows pitch-axis sign inversion. WI3 fixes this
  at msplink; when fixed, readers can trust `q` as canonical QUAT without
  further transform.

## Testing

- Parse `flight-results/flight-20260417/flight_log_2026-04-18T00-36-37.txt`
  and count: 3 engage spans, N `NN` lines within, M `Nav State` lines overall.
  (Reference numbers documented in quickstart.)
