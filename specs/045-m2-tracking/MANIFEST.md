# 045 — MANIFEST

Record of keys, tags, hashes, builds and gate verdicts for 045. Tasks append to it ([tasks.md](tasks.md)).

## § Starting point (T002)

Branch head at the start of implementation: **`e808cd5`** (2026-11-12, the tasks checkpoint; the last code commit is `78e7d6e`).

| check | result |
|---|---|
| incremental `cmake --build build -j8` + `cd build && ctest` | **55 / 55 passed** (2026-11-12) |
| xiao `pio run -e xiaoblesense_arduinocore_mbed -d xiao` | **SUCCESS**, RAM 53.6 % (127 332 B), flash 45.8 % (371 372 B) |
| clean `bash scripts/rebuild-perf.sh` (Constitution IV / IX) | ⏳ **owed: operator** |
| INAV fork `~/inav` `autoc` @ `1fc3b257c`: `make MATEKF722MINI`, `make HGLRCF722LITE` | ⏳ not run (no fork change before the re-fly; needed before ladder L8 / L9) |

**The xiao build flown on 11-08** (the base for T062's diff gate). No hash was recorded at the time. The candidate is **`78e7d6e`**
(2026-11-07, "nn2cpp: unrolled is the default"), the last commit touching `xiao/` before the flight; nothing under `xiao/src` or
`xiao/include` has changed since. The generated program in the tree is g800:
`generatedNNWeightId = a5ed3521ea1e70b7`, `generatedNNFirmwareId = cdf4efb6bb6e120b`, source
`autoc-m1/autoc-9223370244339188361-2026-10-20T16:59:47.446Z/gen9200.dmp.zst`, topology `53->32->16r->3`. ⏳ Confirm against the
boot banner (T034).

## § Preservation (T003, T004)

**Flight baseline of record (T004)**, verified 2026-11-12:
- `specs/044-m1-craft-history/artifacts-t1/nn_weights-t1-g800.dat`: SHA-256 prefix **`a5ed3521ea1e70b7`** = the xiao's
  `generatedNNWeightId` (nn2cpp's weight_id is the first 8 bytes of the weight file's SHA-256).
- `autoc-m1` / `autoc-9223370244339188361-2026-10-20T16:59:47.446Z/`: `gen9999` (gen 1) and `gen9200` (gen 800, the last gen) both
  carry **`retain=keep`**.

**xiao flight logs decoded for N1 (T003)**. The CSVs are regenerable from the tracked `.bin`, so they are **not committed**
(`.gitignore`, per the operator's 2026-10-04 rule for decoded flight artifacts). Which decoder reads which version:

| log | format | decoder | ticks / spans |
|---|---|---|---|
| `flight-results/flight-20261108/flight_log_2026-11-09T00-14-08_flight_001.bin` | v6 | `src/analytics/flightlog_decode.py` (today; tag `044-final`) | 1635 / 4 |
| `flight-results/flight-20261018/flight_log_2026-10-18T19-56-02_flight_001.bin` | v5 | tag `043-final` | 1367 / 4 |
| `flight-results/flight-20261018/flight_log_2026-10-18T20-20-32_flight_002.bin` | v5 | tag `043-final` | 1693 / 3 |

Regenerate:

```bash
B=flight-results/flight-20261108/flight_log_2026-11-09T00-14-08_flight_001.bin
python3 src/analytics/flightlog_decode.py "$B" -o "${B%.bin}.csv"
git show 043-final:src/analytics/flightlog_decode.py > /tmp/flightlog_decode_043.py
for B in flight-results/flight-20261018/flight_log_*.bin; do python3 -I /tmp/flightlog_decode_043.py "$B" -o "${B%.bin}.csv"; done
```

## § Change sets

*(T033 / T049: article 1; T050: article 2.)*

## § Builds

| date | task | what | result |
|---|---|---|---|
| 2026-11-12 | T005–T008 | US1 read-through cache (`src/util/s3_run_selector.cc`, 9 new tests in `tests/s3_run_selector_tests.cc`) | incremental build + ctest **55 / 55**; `s3_run_selector_tests` 17 / 17 |

**T008 hand checks** (real `dmp-dump … --meta-only` on 044-t1 `gen9200.dmp.zst`, 52.9 MB):
unset ⇒ `dmp cache: disabled (AUTOC_DMP_CACHE unset)` on stderr; miss ⇒ 3.3 s, entry filled (52 898 609 B, compressed as in S3);
hit ⇒ 0.49 s; a truncated entry ⇒ `rc=1`, `dmp cache: corrupt entry <path> (zstd decompress failed …)`, no re-fetch; an
uncreatable dir ⇒ `rc=1`, `dmp cache: AUTOC_DMP_CACHE=<path> cannot be created: Not a directory`.

| date | task | what | result |
|---|---|---|---|
| 2026-11-12 | T010–T012 | US1 write-through (`s3PutDmpBlob` → `dmpCacheStore` after a successful put) + `scripts/dmp_cache_sweep.sh` | incremental build + ctest **55 / 55**; `s3_run_selector_tests` 20 / 20; sweep dry-run/apply checked on fake entries (pinned run kept, gone-from-S3 run + stale tmp removed) |
| 2026-11-12 | T014–T016 | US2 step 1: firmware instrumentation (`MTU:`, `STAT:`, short-notify count, loop / NEXT gaps, GAP device name), page instrumentation + **Copy log**, USB-serial `LIST` / `DUMP:<file>` + `xiao_serial_dump.py` | xiao `pio run` SUCCESS, RAM 53.7 % (127 588 B), flash 45.9 % (372 620 B); page JS `node --check` OK. ⏳ not flashed (operator, T017) |

**US1 mini run A (T009, read-through)**: `logs/autoc-045-t0-cache-smoke-a.log`, run `autoc-9223370242347031545-2026-11-12T18:22:24.262Z`
(10 gens, ~20 s/gen, dmps ~13.8 MB). `AUTOC_DMP_CACHE=~/autoc-dmp-cache`. Report pass 1: 32 s, filled 10 entries (138.1 MB).
Pass 2 with `--no-cache` (every dmp re-read): 19 s, the cache byte-identical before and after ⇒ every read a local hit, zero
GETs by construction. ⏳ External confirmation from the S3 access logs: `s3_egress_check.py --profile <admin> --run <run-id>
--day 2026-11-12` (the training box's `autoc-generator` user cannot read `autoc-s3-access-logs`).

**US1 mini run B (T013, write-through = SC-001's local half)**: `logs/autoc-045-t0-cache-smoke-b.log`, run
`autoc-9223370242346471150-2026-11-12T18:31:44.657Z`, trained with `AUTOC_DMP_CACHE` set. Training wrote all 10 entries through.
Report pass 1 (25 s) and pass 2 with `--no-cache` (19 s) left the cache byte-identical ⇒ **zero fetches of the run's own dmps**.
⏳ **Held open (operator 2026-11-12):** the access-log confirmation for T009 / T013 and the month-end egress (T113) wait until
the operator is at a host with the admin profile; the training box cannot read `autoc-s3-access-logs`. Sweep on the real cache: `--days 30` keeps both
runs (fresh `retain=expire`); `--days 0` (dry run) would remove both (20 files, 0.28 GB).

## § Gates

*(G1 T061, G2 T068, US6 decision T069, G3, G4.)*

## § Safety ladder

*(one row per step: change, after record, flight, note.)*
