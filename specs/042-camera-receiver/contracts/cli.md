# Contract — command-line surface

## `beacon_record` — recorder only (flies before the tracker; spec §7.1)

```
beacon_record --config <ini> [--mode continuous|ring|burst] [--out <path>]
              [--ring-seconds N] [--burst-frames N --burst-every M]
              [--duration S] [--trigger manual|gpio]
```
Exercises the entire libcamera path with no tracker attached — deliberately the **first** thing built, so
capture, thermals, storage and the container format are proven before any algorithm depends on them.

## `beacon_trackd` — the daemon

```
beacon_trackd --config <ini> [--source live|replay:<file>]
              [--emit <sink>[,<sink>...]] [--record <path>] [--duration S]

    <sink> ::= binary:<path> | json:- | tcp:<host>:<port> | serial:<dev>[:<baud>] | uart:<dev> | i2c:<addr>
    The record is a versioned STRUCT; every sink carries the same bytes (R13). USB/serial and TCP/WiFi are
    the bench-test transports; uart/i2c are the xiao link; the struct outlives all of them.
```
`--source replay:<file>` runs the **identical `core/`** over a recording, with the acquisition scheduler
virtualised (R3). Live and replay over the same frames must produce byte-identical record streams; that
equality is the golden-vector test.

## Tools (dev box)

```
oracle --in <recording> --out <truth.bin>       # non-causal, unlimited compute
inject --in <recording> --out <recording> --traj <csv> --code A|B --amplitude <n>
score  --records <stream> [--truth <truth.bin>] --out <csv>
```
`score` emits one CSV row per envelope cell with **all of**: valid-output rate, measured-fix rate (spec
§3.1), the §3.2 invariant plus bench and flight-scaled °/s columns, deadline-miss rate (§11.1),
false-acquire rate, and relock times.

## `ascii_scope` — the 10 Hz terminal display (Stage 1 exit criterion)

```
ascii_scope --source tcp:<host>:<port>|json:-|binary:<path> [--hz 10]
```
Renders the M2 grid in the terminal at 10 Hz — beacon position, code identity (A=PORT, B=STARBOARD),
lock/hold state, CEP. Works over SSH with no X. **This animating at 10 Hz on real beacons is the Stage 1
exit criterion** (plan §Stages) — the first end-to-end proof that capture, correlation, tracking, the
record and a transport all work together.

## Exit codes

`0` clean · `1` config error (names the missing/invalid key — Constitution VII) · `2` version mismatch on a
persistence artifact (names both versions — Constitution V) · `3` camera/source failure · `4` deadline
budget exceeded beyond the configured tolerance.
