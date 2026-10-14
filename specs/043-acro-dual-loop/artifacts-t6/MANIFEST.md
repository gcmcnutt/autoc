# 043-t6 — `autoc-043-t6-m1-plant` — MANIFEST (written BEFORE launch, 2026-10-13)

Pattern of record: [artifacts-t5/MANIFEST.md](../artifacts-t5/MANIFEST.md). Why t6 exists: five ACRO flights showed a 2 Hz, ±200 °/s,
±7 g pitch cycle that no sim genome exhibited, because the sim's pitch short period sat at 6.9 Hz / ζ 0.81 (model `I_yy 0.0013`) where the
aircraft's is ~3 Hz / ζ 0.2–0.3. t6 is **t5's regiment, objective and pool on an honest plant**. Evidence and derivation:
[sim-pitch-fidelity-proposal.md](../sim-pitch-fidelity-proposal.md) (§ 1, § 2b "applied"), [actuator-pin.md](../actuator-pin.md)
(§ Inertia measurement, § The t6 plant), [tempered-control-first-principles.md § 11](../tempered-control-first-principles.md).

## 1. The run

| Field | Value |
|---|---|
| S3 prefix | `s3://autoc-m1/autoc-9223370244903245464-2026-10-14T04:18:50.343Z/` |
| Master seed | **1791951530** (`Seed = -1`; logged as "Effective master seed" — the only record of it) |
| Regiment / pool / ini | **identical to t5**: pop 5000 × 800 gens × 294 scenarios, wind envelope 0–8 m/s, `randomA` slot 3, `ExcessRotationAxisMode = conditional`, ramp 40. The only ini change is `CraftCmQSigma 0.32 → 0.6` (class re-centred in code). |
| Launch command | `bash scripts/train.sh autoc.ini logs/autoc-043-t6-m1-plant.log` (Xvfb `:2` up) |
| Launched | **2026-10-13 21:18 PDT**, pid 2146100, detached via `scripts/train.sh`; startup log confirms `ExcessRotationAxisMode: conditional`, `AeroStandardPath3: randomA`, 20 workers, no errors; `autoc.ini` sha `344330b996958ab3` = as-run |

## 2. What changed vs t5 — the plant, and only the plant

| | t5 | **t6** |
|---|---|---|
| `Mass` / `I_xx` / `I_yy` / `I_zz` | 0.515 / 0.024 / 0.0013 / 0.026 | **0.510 / 0.0069 / 0.0094 / 0.016** (measured bare, bifilar; `I_zz` flat-body estimate) |
| pitch `Cm_a` / `Cm_q` / `Cm_de` | −0.55 / −4.2 / −0.36 | −0.55 / **−1.0** / −0.36 — `Cm_a`, `Cm_de` unchanged; the FRF of gen 488 rejected −1.0/−0.65 |
| pitch short period (analytic, 14 m/s) | 6.9 Hz / ζ 0.81 | **2.6 Hz / ζ 0.4** (aircraft ~3.0 / 0.2–0.3) |
| roll `Cl_*` / yaw `Cn_*` | — | × 0.2875 / × 0.615 — roll and yaw dynamics **held constant** across the inertia change (roll FRF of gen 488 verified) |
| `kCraftCmQCenter` / clamp / `CraftCmQSigma` | −4.2 / [−5.0, −3.6] / 0.32 | **−1.0 / [−2.0, −0.2] / 0.6** (pitch damping stays a craft variation) |
| NN→surface latency | configured 10 ms, **honoured as one 50 ms frame** (frame-top apply) | **honoured at the 5 ms sub-step** (`applyPendingSubstep`) |
| `Global::craftCmQ` default | hard-coded −4.2, overrode the xml in non-autoc runs | NaN (unset) + xml fallback |
| objective, pool, courses, wind, seeds policy | | **unchanged** |

Commits: autoc **`290b098`** (+ the record commit that follows it), crrcsim **`7f345c5`**. `autoc.ini` as-run snapshot [`autoc.ini.as-run`](autoc.ini.as-run), sha256 `344330b996958ab3…`.
Build: clean `scripts/rebuild-perf.sh` 2026-10-13 21:06 (`PERFORMANCE_BUILD=ON`), ctest **53/53**.

## 3. Gates run before launch (actuator-pin § "The t6 plant")

| gate | result |
|---|---|
| step test (plant only, same binary A/B) | pitch peak **135–155 ms** (old plant 50 ms; aircraft 133–166), overshoot 1.3–1.7× (aircraft ~2.5×), no FDM overflow after the harness trim retune |
| **gen 488 reproduces its own flights** | RMS **105–123 °/s** (flight 124–153), peak **1.9–2.3 Hz** (2.0–2.3), 2–3 Hz share 24–47 % (31–63 %), closed-loop \|H\| 1.34 at 2.5 Hz (1.5–1.6), tracking 48 → 40 %, 3 crashes — the sim now exhibits the cycle |
| roll preserved | gen 488 roll FRF t5 vs t6 plant within ~15 % at 0.5–2.5 Hz |
| new eval baselines (plant changed; expected) | t3 genome **−54922.067792** (was −88026.619367); gen 488 **−65928.363124** (was −95067.305736) |
| cross-build determinism | ✅ incremental build vs clean `rebuild-perf.sh`: t3 genome **−54922.067792**, gen 488 **−65928.363124** — identical to the last digit |
| **T092 trainability** | ✅ **passes, ~2× slower start** — basic-m1 smoke (pop 3000, 16 winds, full variations from gen 1, Seed 4242, same binary): old plant takes off at gen 13 (−835 at 25); new plant flat to gen 26, then −621 (40) → −799 (50) → −1029 (64–80), 16/16 complete from gen 42, `pctInStreak` 10.4 % at 80 |

## 4. ⛔ What is and is not comparable

- Fitness is not comparable to any earlier run (plant changed).
- **Comparable to t5 at matched gens**: per-path score/step, `pctInStreak`, crashes, `rotRatio`, population average (`#NNGen avg=`) — t5 is the
  dashed reference now (`generate_pngs.sh … --compare 043-t5-condrot:logs/autoc-043-t5-m1-condrot.log` builds its per-path cache once).
- `sim_pitch_spectrum.py` on the elite's dmp is the new primary read: on this plant a genome that dithers pays in pitch rate.

## 5. Early calls (same discipline as t5) and the exit

| when | healthy | stop-and-rethink |
|---|---|---|
| gen 2–5 | `#GenSel` bite ≫ 0 (same pool as t5) | inert tie-break |
| gen 5–40 | elite replaced every few gens; pop-avg moving (t5: −1.1k @5, −4.3k @40; a slower start is expected on the harder plant) | elite frozen 40 gens (the t4 signature) **or** pop-avg flat as in the smoke — then T092's options |
| gen ≥ 100 | `sim_pitch_spectrum` of the elite: pitch RMS and 2–3 Hz share **falling** with gens while `pctInStreak` rises | RMS stuck at g488's 105–123 °/s = the objective still cannot see it |
| exit (T118) | the t6 genome **in flight**: pitch-rate RMS well under 124–153 °/s, 2–3 Hz share well under 31–63 %, setpoint off the cap, tracking within noise of g488 | |

## 6. Post-run

⏳
