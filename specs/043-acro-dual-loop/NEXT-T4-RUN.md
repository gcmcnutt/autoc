# 043-t4 — the next bake must calm the pitch bang-bang

⭐ **Written for a FRESH CONTEXT.** Everything needed is here or linked.

**Operator's call (2026-10-05), after the 10-04 bench + all-flights study:** t4 focuses on **calming the
pitch bang-bang**. It is not helping tracking and it is the parent of most of the damage below.

---

## 1. The one thing to know

⭐ **The 2–3 Hz pitch oscillation is a ±7 g loading cycle, and it is the common parent of nearly every
anomaly in this feature.** Measured across all engaged flights: `|accSmooth[2]|` p99 **5.2–7.5 g**, max
**8.4–11.4 g** — and that is the *15 Hz-filtered* value, so raw transients are higher, against a
`ACC_CLIPPING_THRESHOLD_G` of **15.9 g**.

What it causes, all measured in
[`vibration-analysis-20261004.md`](vibration-analysis-20261004.md) § 6:

| consequence | evidence |
|---|---|
| **Destroys the static source** | `BaroAlt` moves **3–17× further than physically possible** between updates while engaged (t3: p95 759 cm vs a 46 cm bound, max **26 m** in one 17 ms sample); roughly physical when unengaged. |
| **Pushes the accel toward its rail** | ~2× away on filtered data, closer on raw. Unconfirmed — needs `debug_mode = VIBE`. |
| **Feeds the NN garbage** | the estimator's vertical position reaches the NN via a custom `MSP2_AUTOC_STATE` (`pos[3]`, NEU cm) → `setPosition()`, so every reset lands in `pos_d`. t3 span 2 handed the NN an **86.69 m** step in one tick. |
| **Buys nothing** | tracking error 24.5–33.6 m median across t3's spans; the oscillation is unchanged in frequency, band share and amplitude from 041-t7, which flew with the rate loop **bypassed**. |

⛔ **It is not the rate loop.** t7 flew `MANUAL` (servos straight from `rcCommand`) and t3 flew ACRO with
INAV's loop fully in command; the oscillation is identical in both — see
[`flight-analysis-20260913.md`](flight-analysis-20260913.md) § 5. The only loop common to both is
**NN ↔ airframe at 20 Hz**. ⇒ it cannot be tuned out in INAV; it has to come out of the **policy**.

## 2. What has been ruled out — do not re-investigate

Each of these was chased and closed. Full working in
[`vibration-analysis-20261004.md`](vibration-analysis-20261004.md).

| hypothesis | verdict | where |
|---|---|---|
| Prop imbalance | ⛔ No — it is 2/rev **blade-pass**; shaft order is ~8 % of it. Balancing buys nothing. | § 2, § 4 |
| Structural resonance | ⛔ No — nothing throttle-independent in 40–487 Hz. | § 3 |
| Vibration / `accVib` / `accWeight` | ⛔ Not the cause of the Z jumps: the two **highest**-vibration logs were never engaged and had **zero** jumps; 09-05 had 3 jumps at the **lowest** vibration. | § 5.2 |
| Accel FSR mis-set | ⛔ No — hardcoded `INV_FSR_16G` = ±16 g, the **widest** the MPU6000/6500 offers. | § 5.2 |
| Quantisation / accel LPF | ⛔ No — quantum is 0.49 mg against a 6–11 g signal. | § 6.3 |
| MSP polling starving the FC | ⛔ No — frame `dt` identical engaged vs unengaged. | § 5.2 |
| Static port as the *cause* of jumps | ⛔ Real and severe (§ 6.2) but **does not discriminate** — t7 is clean with 15.5× over-physical steps vs t3's 16.5×. | § 6.5 |
| GPS quality | ⛔ Does not discriminate — clean flights include worse `epv` (572 cm) than any jump flight (419 cm). | § 6.5 |

⭐ **The only marker that separates, 23/23 with no exceptions:** `navEPV` reaching the
`inav_max_eph_epv = 1000` cm cap. Jump logs all peak 999; clean logs peak 463–936.

⚠️ **Read § 6.5 before hunting for a root cause.** Four confirmed-but-non-discriminating factors plus one
perfect threshold marker is the signature of a **threshold-crossing race**, not a missing cause. Bad
conditions are present on *every* engaged flight; whether `navEPV` crosses 1000 decides whether it
becomes a reset. Removing the ±7 g excitation is the lever — not more forensics.

## 3. Suggested t4 shape — ✅ CONFIRMED 2026-10-05/06 (operator)

⭐ **Objective-side change chosen: T102, the excess-rotation lexicase axis** — the craft's rotation minus what
the path demands, per scenario, co-equal from gen 0, no tunable parameter. Derivation and the history of the
alternatives (015's `Σ|Δu|` exploit, 041's Es-destroyed exploit, fixed g-caps = tuning, lookahead = cheating)
in [tempered-control-first-principles.md](tempered-control-first-principles.md). Implemented and gated
2026-10-06 (53/53 tests; t3 reproduces bitwise with the key absent). ⚠️ The sim-visibility caveat below
is why the **jerk** term (item 1) was not chosen: the sim's pitch command has ~10% of its power at 2–3 Hz
against 56–81% in flight (t3 ADDENDUM A1) — until D1 lands, a reversal term trains on a tone the sim does
not make. The rotation measure has signal in today's sim: the t3 genome rotates **5.1×** the path's demand
(straight-and-level 6.1×).

### What changed, t3 → t4 (the delta of record; details in tasks.md Phase 10b)

| | 043-t3 (flown 09-13) | **043-t4** |
|---|---|---|
| **objective** | tracking + throttle energy; OOB crash price 10.0 | **+ excess-rotation axis** (`EnableExcessRotationAxis = 1`, T102); energy + crash price unchanged |
| **wind speed** | 3.66 m/s on every tick | **uniform 0–8 m/s per scenario** (T087), ramped |
| **wind direction** | σ 45° about 330° | **σ 60°** about 330° |
| **turbulence** | Dryden, TI fixed 15.7%, L fixed | **TI ×0.5–2.0** (T087a), **length scale ×0.25–1.0** (T088) per scenario |
| **thermals** | 0–5 cells, 2.0±0.5 m/s, ~10% exposure | **count_max 8, strength ×0.5–1.5** (T087b) — 25–47% exposure in smoke |
| **courses** | 5 analytic + 1 random (slot 5), 45° loop in slot 3 (7.7 s) | **slot 3 = second random course** `SeededRandomA` (T100): 2/6 random-entry, 45 s |
| **entry geometry** | latched 0.1–1.1 m on 5 paths | unchanged (dither deferred to t5+, T085/T086) |
| **pitch plant (D1)** | no short period in sim | **unchanged unless T096 lands before launch** — t4 flight not to be judged on pitch if so |
| **wire / dmps** | — | no format change; t2/t3 dmps readable; all new keys OFF in code |
| **what is NOT comparable** | — | raw fitness (new regiment); compare crash rate, `pctInStreak` by wind bin, `rotRatio` vs 5.10×, cold-start completion vs 84.7% |

Aircraft side is unchanged from the t3 flight except the logging/estimator config in § 3 below (GPS_ONLY,
VIBE, rate 1/2) — none of which the bake sees.


**Objective side (the real fix).** The bake currently prices tracking error and crashes but nothing for
**control thrash**. Candidates, in rough order of directness:

1. ⭐ **Penalise pitch-rate reversal / command jerk.** A bang-bang policy alternates the sign of
   `out_pitch` at ~2–3 Hz; a term on sign-flip rate or on `d(out_pitch)/dt` prices that directly.
   Check [`variation-inventory.md`](variation-inventory.md) and
   [`fitness_decomposition.cc`](../../src/eval/fitness_decomposition.cc) for where this slots in.
2. **Penalise normal load.** `accSmooth[2]` excursion is the physical damage; a g-load term maps straight
   onto the measured harm. There is already a `g_load` plot in the bake telemetry.
3. **Throttle**: t3 ran the `+rail` for **85–90 %** of engaged ticks. Also worth a term — it is what puts
   the airframe in the worst vibration band, though § 5.2 says vibration is not the jump cause.

⚠️ The sim must be able to *see* the oscillation for any of this to train. Confirm the 20 Hz ZOH plant
and `COMPUTE_LATENCY_MSEC_DEFAULT = 10` still reproduce a 2–3 Hz limit cycle before trusting a penalty
to suppress one — if the sim is smooth where the aircraft rings, the term trains on nothing.

**Config side (cheap, independent of the bake).**

```
set inav_default_alt_sensor = GPS_ONLY   # baro currently outweighs GPS: w_z_baro_p 0.35 > w_z_gps_p 0.20
set debug_mode = VIBE                    # debug[0..2]=accVibeLevels, debug[3]=accClipCount
set blackbox_rate_denom = 2              # 1/32 is NOT enough -- see below
save
```
Then `flash_erase`. `GPS_ONLY` keeps baro as a fallback; it removes the 3–17× garbage from the estimator
that feeds the NN. It is **not** the fix for the oscillation.

## 4. ⛔ Logging traps that have already cost measurements

- **`blackbox_rate_denom` does not persist.** Three instances now (t3 § 1 documents two). Likely
  mechanism: the **Configurator's Blackbox tab writes the whole blackbox config over MSP**, overwriting a
  CLI `save`. ⇒ set it in the CLI, `save`, **do not reopen that tab**, and verify by reading
  `P interval` back out of the log.
- **1/32 (59 Hz) is useless above 30 Hz** and `gyroRaw` does not rescue it — "raw" means *pre-filter*,
  not *pre-decimation*. Use **1/2** (973 Hz) for anything spectral; it kept up fine at 0.02 % missing
  iterations and *improved* looptime (513 vs 519 µs).
- ⛔ **`flash_erase` before every flight.** It was skipped before 07-20 and 09-07 `103927`, so **log
  index 1 in those files is the previous flight.** Decode **every** `--index`, not just 1 — six sessions
  in the repo had never been analysed because `.01.csv` only ever holds log 1.
- Decoded CSVs are **not tracked** any more (regenerable; gitignored 2026-10-04). The `.TXT` is the
  source of truth.
- ⚠️ `escRPM` is garbage (no ESC telemetry) ⇒ no RPM filter. Doesn't matter: the 25 Hz gyro LPF already
  leaves < 1.2 °/s.
- ⚠️ Gyro amplitudes above ~256 Hz are understated and **unfixable from the log side** — INAV 8 hardcodes
  the MPU DLPF to 256 Hz and the `gyro_lpf:0` header line is a literal, not a setting.

## 5. Reproduce the analysis

```
blackbox_decode --index 1 flight-results/flight-20261004/blackbox_log_2026-10-04_141959.TXT
specs/043-acro-dual-loop/vibration_order.py  <decoded>.01.csv      # prop orders, S2
specs/043-acro-dual-loop/static_port_aoa.py  <decoded>.01.csv      # static port, S6
specs/043-acro-dual-loop/pitch_spectrum.py   <decoded>.01.csv      # oscillation, t3 S5
```

All three are **stdlib-only on purpose** — there is no numpy on the bench host. Do not "improve" them
into a numpy dependency.
