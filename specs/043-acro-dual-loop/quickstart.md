# 043 — quickstart

## Read first, in this order

1. [spec.md](spec.md) **§ What ACRO is** — the definition; everything else is downstream of it.
2. [spec.md](spec.md) **§ Execution order** — ⛔ *not* the story-priority order.
3. [research.md](research.md) R1 — what INAV's loop actually computes (it is **not** a PID).
4. [contracts/inav-fw-rate-loop.md](contracts/inav-fw-rate-loop.md) — the thing being built.

## The three facts that catch people out

- ⛔ **ACRO has no attitude feedback.** Zero command = *stop rotating*, not *hold attitude*. Building
  attitude hold as a goal produces ANGLE mode and trains the policy on a safety net the aircraft lacks.
- ⭐ **The gains are not constant.** P and D are multiplied by a Gaussian in the *setpoint*
  (σ = 61.2 °/s roll, **20.4 °/s pitch**). Loop gain is highest at zero commanded rate.
- ⭐ **It is feed-forward dominant.** `kFF` 1.61 / 2.26 against `kP` 0.484. At 88 °/s roll, FF alone is 142
  of the ±500 budget.

## Do this first — ⛔ before touching `ScenarioMetadata`

```bash
# FR-057: the pinned 041-t7 baseline becomes unreadable the moment the wire format changes.
# s3://autoc-m1/autoc-9223370249590214474-2026-08-20T22:22:41.333Z/  (retain=keep, 800 dmps)
# Extract per-tick CSVs / whatever the comparisons need, verify readable, THEN change the format.
```

## Build gates

```bash
bash scripts/rebuild.sh                              # autoc + crrcsim, tests           (Constitution II)
bash scripts/rebuild-perf.sh                         # ⛔ REQUIRED after CMakeLists edits (Constitution IV)
~/.platformio/penv/bin/pio run -e xiaoblesense_arduinocore_mbed   # from xiao/
# INAV: build BOTH targets — bench MAMBAF722_2022A first, then flight MATEKF722MINI.
# ⚠️ Disconnect the GPS before flashing.
```

## The model's acceptance test — run it before anything else believes the model

```
1. constant rate setpoint      → achieved rate converges to it
2. mis-trimmed craft, zero cmd → body rate settles to ZERO and stays              (SC-012)
3. displaced to bank, zero cmd → held ~1 s, then DRIFTS (expected — no attitude ref)
   run from +30 and -30: drift must be UNCORRELATED with bank sign  (SC-012 converse)
   ⛔ a sign-correlated restoring trend = ANGLE built by accident
4. attenuation sweep           → aP matches exp(−r²/2σ²) at r ∈ {0, σ, 2σ}
5. known-good genome           → still trains                                     (SC-004)
```

Run the attitude sweep **before autoc is connected** (FR-019), so a failure is attributable to the model
rather than surfacing later as a policy that behaves oddly in one corner.

## Launching the bake

```bash
# Constitution IX — detached, never via a harness background task.
# Pre-run gate first: clean build + relevant tests.
bash scripts/train.sh autoc.ini <unique-logfile>
```
