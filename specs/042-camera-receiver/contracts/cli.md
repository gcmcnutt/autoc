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
              [--emit binary:<path>|json:-|both] [--record <path>] [--duration S]
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

## Exit codes

`0` clean · `1` config error (names the missing/invalid key — Constitution VII) · `2` version mismatch on a
persistence artifact (names both versions — Constitution V) · `3` camera/source failure · `4` deadline
budget exceeded beyond the configured tolerance.
