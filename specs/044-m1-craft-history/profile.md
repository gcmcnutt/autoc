# 044 — training-cost profile (FR-015, research R10, tasks T013 / T014)

Question: where did training time go? 043 generation time rose 310 → 465 → 577 s (t2 → t3 → t5). t3 → t5 is explained by the
course change; **t2 → t3 is +47 % per tick at equal flight length and is unexplained.** The parent ran at 161 % CPU with a ~25 s
serial phase per generation.

Rule: adopt only changes that leave the **post-stream-fix reference replays** (MANIFEST § Post-stream-fix reference, T012) bitwise
identical; everything else goes to `specs/BACKLOG.md`.

## Method

Kernel `perf` is unavailable on this host (`perf_event_paranoid` = 4) and `ptrace_scope` = 1 forbids attaching gdb to a
`train.sh`-launched (setsid) run, so the profile is CPU accounting from `/proc` (per-process and per-thread utime + stime, sampled)
plus a new per-generation timing line `#GenTiming` (bitwise-neutral instrumentation, kept).

## (a) Per-tick cost — rate-loop controller node A/B (T013a)

Fixed eval: t6 g800, tier 0, seed 1791951530, post-stream-fix build, 294 scenarios; same build, `hb1_streamer.xml` stock vs a
temporary copy with the `<controllers>` node removed (restored, `git status` clean).

| model | worker CPU (20 crrcsim) | ticks | worker µs / tick | parent CPU |
|---|---|---|---|---|
| stock | 3.4 s | 167 367 | **20.3** | 16.8 s |
| no controller node | 3.2 s | 167 610 | **19.1** | 16.8 s |

The INAV rate-loop node costs **~6 %** of worker time per tick — not the +47 %. (The parent's 16.8 s is eval-mode overhead:
startup and, mostly, the zstd-19 compression of the 87 MB eval dump — see (d).)

## (b) Per-tick cost — `EvalTick` payload (T013b)

Not built as a throwaway: the payload is measured in place instead. A full-regiment eval dump is **87 MB uncompressed for 167 k ticks
= ~520 B / tick**, and every training evaluation ships its full tick records to the parent, where one RPC thread per worker
deserializes them and runs `computeScenarioScores`. In the smoke regiment those 20 threads use **~9 % of a core each (~1.7 cores)**;
the worker waits for its thread to finish before its next job arrives (request/response per job), which is most of the remaining
worker idle in (d). Trimming it needs worker-side scoring (`EvalResultsLite`) — not bitwise-neutral by construction to verify cheaply
→ BACKLOG.

## (c) Accounting for the +47 % (T013c)

No per-tick code change separates 043-t2 from t3 (artifacts-t3/MANIFEST § 4: `OobCrashPenaltyWeight` 0 → 10, compute latency
30 → 10 ms, the sensor-LPF back-out). The per-generation cost scales with the **population's** mean flight length, not the elite's:
the smoke's first generations average **79 ticks per flight** while the elite flies ~570. A crash penalty that lengthens the
population's flights raises cost per generation with no change in cost per tick, which fits a +47 % "per tick" figure normalised by
elite flight length. **Likely explanation, not verified retroactively** (t2/t3 logged no population tick counts). `#GenTiming` now
logs `popTicks`, `meanTicksPerSim` and `evalUsPerTick`, so the bake measures it directly. Unexplained remainder: none attributable to
code; recorded here and in BACKLOG.

## (d) Parent profile (T014)

`specs/044-m1-craft-history/smoke.ini` (3000 × 96 scenarios), launched with `scripts/train.sh`, sampled over 50–90 s windows.

| | before | after the async upload |
|---|---|---|
| generation time (gens 2–14) | 24.6–28.3 s | **18.8–20.4 s** (−25 %) |
| serial gap, elite re-eval → gen summary | 6–9 s | 0.6 s (`reevalSec` 0.55 + `dmpSerializeSec` 0.05) |
| workers busy | 10.5–12.6 of 20 cores | **16.4 cores** |
| parent main thread | ~20 % of wall | — |
| per-gen fitness (best / avg / worst, gens 1–14) | — | **identical** to the sync build; 0 `NN_ELITE_DIVERGED` |

**Cause**: the elite dmp was compressed with zstd **level 19 on the main thread every generation** while every worker sat idle —
measured on a full-regiment elite dump: level 19 **23.8 s**, level 9 1.0 s, level 3 0.27 s (87 MB). On the 800-gen bake (~577 s
generations) that is ~4 % of wall; on short early generations it dominated.

**Adopted** (bitwise-neutral, verified): the compress + upload now runs on one background slot (`std::async`), overlapping the next
generation's evaluation; the previous upload is joined before the next starts and its exception rethrown (fail-fast preserved); the
last one is joined before the run reports done. Stored bytes unchanged. `#GenTiming gen= totalSec evalSec reevalSec dmpSerializeSec
uploadWaitSec popTicks meanTicksPerSim evalUsPerTick` added.

## Adopted / backlogged

- ✅ Async elite-dmp upload (above).
- ✅ `#GenTiming` per-generation timing + population tick count.
- ⏭ BACKLOG: pipeline the per-worker request/response (send job n+1 before deserializing/scoring result n) or score worker-side
  (`EvalResultsLite`) — the remaining ~3.6 idle worker cores in the smoke; also cuts the ~520 B/tick payload.
- ⏭ BACKLOG: confirm the t2 → t3 explanation on the bake's `meanTicksPerSim` trajectory.
