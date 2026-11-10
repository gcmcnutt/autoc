> 📘 **[spec.md](spec.md) governs from 2026-11-10** (execution order, requirements, acceptance). This README is the derivation of
> record and is superseded where it conflicts.
>
> ▶ **044 CLOSED 2026-11-08** ([outcome](../044-m1-craft-history/outcome.md)) — 045 is next. Inherits layout v4 (53 M1 / 74 tracker
> inputs, craft history in the shared block), the honest pitch + roll plant, flight log v6 and NN01 v2.
> ⛔ **M2 does not proceed until [PHASE 0](#-phase-0--pre-m2-general-improvement-and-a-re-fly-operator-2026-11-10) closes**: two front tasks
> (dmp cache, BLE downloader), the navigation & attitude pre-tasks, and **a re-fly of the current model** with the GPS / blackbox / INAV
> changes, for a cleaner signal. Phase 1 flies a virtual target, so it depends on the nav frame exactly as M1 does.
> The 044 SC-002 pitch 2–3 Hz miss is **backlog, not a 045 gate** (operator 2026-11-10: close enough; M2 fidelity work comes before
> fine-tuning basic control).

## ⛔ PHASE 0 — pre-M2: general improvement and a re-fly (operator 2026-11-10)

*"The pre-work is general improvement and we probably need another flight on the most recent model with GPS, blackbox, INAV
adjustments to get a better signal … all this is before the M2 proceeds."*

Order:
1. **Front tasks**: [F1 dmp cache](#-front-task--write-through-dmp-cache-on-the-training-box-filed-2026-10-25) and
   [F2 BLE downloader](#-front-task-f2--ble-flight-log-downloader-filed-2026-11-08-promoted-2026-11-10).
2. **Pre-tasks**: [N1–N6](#-pre-task--navigation--attitude-integrity-filed-2026-11-09), plus **extra blackbox fields** (B1, below), whatever N1
   shows is missing.
3. **Re-fly the flight baseline of record (044-t1 g800)** with N2's GPS / INAV config, N3's advisory logging and B1's blackbox
   fields. The goal is a clean nav and attitude signal on a known policy. **Article 2** (`xiao/inav-hb1-art2.cfg`,
   `cad/article2-harness/`) flies in the same window if the weather allows.
4. Then the **M2 core**: camera parameters from 042 ([Gate 1](#gate-1--the-042-camera-parameters-must-have-landed)), lock down the
   signal-realism model, and [Gate 2](#gate-2---audit-the-46-tracker-specific-input-scales) (⚠️ **needs discussion** before it is acted on).

- **B1 — Additional blackbox fields (scope from N1).** Log the estimator internals that N1 has to infer today: at least the nav
  position and velocity estimate and its validity, EPH/EPV, accWeight and sat count. These must be in the re-fly's log.


# 045 — M2 tracking (seed)

> ⚠️ **RENUMBERED 044 → 045 on 2026-10-18** (operator, at the 043 wrap): one more M1 feature goes first —
> [044](../044-m1-craft-history/README.md), craft-sensor history in the input vector plus the roll-plant correction the
> 2026-10-18 flight measured. M2 inherits both (the craft block is shared with the tracker vector) and the honest pitch plant
> from 043. Nothing below was started.

> ⚠️ **RENUMBERED 043 → 044 on 2026-08-23.** The 043 slot was repurposed for
> [ACRO dual-loop](../043-acro-dual-loop/README.md), which must land first: the 041-t7 flight oscillates at
> 2-5 Hz and M2 would inherit whatever control architecture M1 ends on. Everything below is unchanged and
> still governs M2 — including the two gates (042 camera params; audit the 46 tracker input scales).

**Created 2026-08-17** when 041 was scoped down to *"a fresh full M1 toolchain, flown"*. This holds the M2
work moved out of it. **Not started.**

## ⛔ FRONT TASK — write-through dmp cache on the training box (filed 2026-10-25)

**Must land in the build that runs 045's first smoke/bake.** It's independent of the M2 content, so it can merge on its own
ahead of everything else.

**Why.** Egress is the biggest item on the AWS bill: **Oct 2026 = 378 GB / $25.04** out of $32.33 (Aug was the same, 380 GB).
S3 access logs for October attribute **all of it** to `autoc-generator` `GET`s on `autoc-m1` from the home IP, using
`aws-sdk-cpp`. That's `scripts/generate_pngs.sh` running **on the training box**, about twice a day. The box uploads each
dmp and then downloads it again **2–7 times** to plot it. Each report pass keeps its own incremental CSV cache, but each one
does its own S3 fetch. Every gen: `run-summary` + `input_investment`. Every 5th gen: + `dynamics`, `energy`,
`tracking_by_path`, `pitch_cycle`, plus the latest-gen `--csv-only` / `--meta-only`. **150 GB of unique dmps → 422 GB
transferred (2.8×)**, ~143 MB per gen against a ~51 MB `.dmp.zst`. ⚠️ M2 is worse: `predictor_analysis` adds another
per-stride pass, and `rnn_capacity` / `source_dmp_loader` also read from S3.

**Design (operator 2026-10-25).** S3 stays the **archive of record and is written immediately**, so other hosts (renderer
on the WSL box, etc.) can read any gen as soon as it exists. Reporters on the training box read the **local copy**.
- **One chokepoint.** Every C++ reader/writer goes through `s3GetDmpBlob` / `s3PutDmpBlob` in
  `src/util/s3_run_selector.cc`. The Python reports shell out to `dmp-dump` / `nnextractor`, which use the same calls.
  Change those two functions and nothing else needs to change.
- **Write-through (`s3PutDmpBlob`).** After the S3 put succeeds, write the same **compressed** bytes to
  `<cache>/<bucket>/<key>` (tmp file + rename, so a reader never sees a partial file).
- **Read-through (`s3GetDmpBlob`).** Local hit → read it. Miss → S3 `GET`, then fill the cache, so other hosts and
  `--compare` reference runs also fetch each key at most once.
- **No invalidation needed.** Keys are unique per run-id + gen and never rewritten.
- **Constitution VII.** The cache dir is configured explicitly (one name, e.g. `AUTOC_DMP_CACHE`, which needs to be an
  env var because the Python reports spawn the tools). Unset = no cache, and startup logs that once. Configured but
  unwritable, or a cached entry that fails to inflate = **fail loud with the path**, never silently go back to S3.
- **Eviction.** A sweep matching the S3 lifecycle: drop `retain=expire` runs after 30 days, keep `retain=keep`
  milestones. One 800-gen run is ~40 GB; the box has 3.2 TB free.
- ⚠️ Rebuilding `dmp-dump` changes `generate_pngs.sh`'s `DSIG` cache key and forces one full re-read of the current run.
  Runs written before the cache existed aren't in it, so that re-read costs one S3 `GET` per key (once).

**Accept.** For one report cycle on a run baked with the cache: S3 access logs show **zero** `GET.OBJECT` from the training
box for gens that host wrote, and ≤ 1 per key for anything else. The month's `USW2-DataTransfer-Out-Bytes` stays under
the 100 GB free tier. Logs: `s3://autoc-s3-access-logs/{m1,m2,eval}/` (90-day expiry).

## ⛔ FRONT TASK F2 — BLE flight-log downloader (filed 2026-11-08, promoted 2026-11-10)

Field workflow, needed before the Phase 0 re-fly. A 300 kB xiao flight log took **~10 minutes** to download in the field
(~0.5 kB/s). BLE should move that in seconds, so the transport is broken, not just slow. (1) **Resume** a dropped download from the
last acknowledged offset instead of restarting. (2) **Measure, then fix the dominant cost**: negotiated ATT MTU, connection interval,
notify vs read-per-chunk, write-with-response round trips, QSPI read size, any per-chunk delay or flush. Surfaces:
`xiao/web/flight_logger.html` and the xiao BLE service. Filing record: [BACKLOG](../BACKLOG.md) § 044 deferrals.

## ⛔ PRE-TASK — navigation & attitude integrity (filed 2026-11-09)

**Gates the first 045 flight; N4/N5 decide what the 045 bake trains on.** Pulled in from BACKLOG (each source item is marked
promoted there).

### Why — the frame the policy flies in is not trustworthy

**Operator 2026-11-09:** *"during M1 we fly against a virtual path all inside the system. This assumes some level of stability of
attitude/position is accurate. Over 17 seconds dead reckoning should be easy to do with built-in IMUs in these boxes. Sure the GPS
can be used as a cross check, but once we get to M2, which is tracking a real, not virtual target, relative direction is clean and
the IMU really devolves to a stability engine with rough position tracking as a safety mechanism. Right now the problem is our
self-referential virtual path assumes the craft attitude is stable, and any offset from that/drift winds up tilting the whole target
path. Eventually not using INAV other than as a safety fallback seems good."*

- **M1 and 045 phase 1 are self-referential.** The path or target is defined in the nav frame anchored at engage, so a *smooth* drift
  of the estimate is harmless: the path drifts with it. What hurts is **discontinuities** (resets, freezes) and **tilt** (an
  attitude error rotates the path against gravity, which the energy objective and the floor both see). Phase 2 (a real target)
  takes its direction from the camera, and the inertial frame becomes stability plus a rough position for the safety envelope.
- **INAV's estimator has failed under the NN on every ACRO-era flight**: 09-05 85 m, 09-06 58 m, 09-13 93 m vertical divergence
  ([flight-analysis-20260913 A4](../043-acro-dual-loop/flight-analysis-20260913.md)); 10-18 flight 1 span 1 −82 / +101 / −52 m
  vertical steps; 11-08 span 1 a 2.3 s horizontal freeze, a 105 m horizontal reset and a 26 m altitude reset
  ([flight-analysis-20261108 § 6](../044-m1-craft-history/flight-analysis-20261108.md#6-span-1-was-flown-on-a-faulted-nav-estimate-added-2026-11-09)).
  **The sim feeds the NN perfect state**, so none of this is visible in training.
- **Why INAV's coast failed on 11-08 is attitude under load, not the accelerometer.** No clipping; no fixed misalignment (best-fit
  heading offset 0° median over the flight). But the GPS-out window averaged **4.4 g**: at that load each 1° of attitude error puts
  ~0.75 m/s² into the wrong horizontal axis, and the IMU-integrated velocity change came out ~85° rotated from GPS (+14 m/s east vs
  +23 m/s north). Dead-reckoning error grows with manoeuvre severity (2.3 s windows, p50 2.9 → 11 m/s as max pitch goes < 30° →
  > 70°; best-fit offset scatter ±10–25°). That points at INAV's AHRS during sustained pulls (`ahrs_acc_ignore_rate 15` ⇒ gyro + mag
  only) and the under-calibrated QMC5883. ⚠️ The reference is 10 Hz GPS velocity from a 5–7-sat receiver, so the evidence is
  indirect. Two INAV design points made it worse: when GPS stops correcting, horizontal velocity **decays to zero** at
  `inav_w_xy_res_v` 0.5/s **÷ accWeight**, and Z vibration holds accWeight at its 0.30 floor in every span (≈ 1.7/s decay); with
  `inav_allow_dead_reckoning = OFF` the estimate then freezes.
- ⚠️ **The quaternion is 4 of the policy's inputs.** If attitude is off 10–25° in hard manoeuvres, the policy sees that exactly when
  it matters.
- ⚠️ **"Dead reckoning should be easy" holds for attitude and velocity over 17 s, not for double-integrated position at these
  loads.** Gyro bias ~0.1 °/s ≈ 1.7° over a span; 1° of tilt at 4 g sustained ≈ 100 m of position over 17 s. For a virtual path
  that is acceptable **if it is smooth**, which is the design target: **span-local, continuous, gravity-correct; never reset inside a
  span.** N1 measures whether the IMUs deliver it.

### The tasks

- **N1 — Measure first (offline, existing logs, no flight).** From the 250 Hz blackbox gyro/accel on 11-08 and 10-18: propagate
  attitude **gyro-only from engage** (initialized from INAV's attitude) and compare it with INAV's attitude and with the GPS-derived
  specific-force direction over each span. Report attitude drift at 17 s, then velocity and position drift for a span-local inertial
  propagator, against INAV's estimate including its faults. This sizes N4 and tells whether INAV's AHRS or the raw IMU is the
  better attitude source in manoeuvres. Starting tools, from the 11-09 analysis (numpy; `blackbox_decode` first):
  [`nav_span_scan.py`](nav_span_scan.py) (xiao CSV: frozen / stepped estimates per span),
  [`nav_accel_fit.py`](nav_accel_fit.py) (earth-frame IMU vs GPS: sign / lag fit, heading-offset sweep, vibration and accWeight;
  writes `align.npz`), [`nav_dead_reckon.py`](nav_dead_reckon.py) (windowed dead reckoning binned by manoeuvre, and the INAV
  no-GPS rule replay), [`nav_heading_check.py`](nav_heading_check.py) (heading vs GPS course, for N2's mag check). 11-08 reproduces
  with `nav_accel_fit.py DIR --air 355 535 --window 378.4 383.2` then `nav_dead_reckon.py DIR --replay 378.47 380.76`.
- **N2 — INAV configuration review for all-attitude flight (operator 2026-11-10: "a LOT").** The config of record
  ([`xiao/inav-hb1.cfg`](../../xiao/inav-hb1.cfg), frozen at 91abd4d) is mostly INAV's defaults for a cruising airplane. The 043/044
  analyses show it does not hold up in sustained high-g, all-attitude flight. This is a full review, not just the GPS items. It is
  sorted by what each setting touches, because that decides whether the g800 re-fly is still a test of the same policy:

  | class | what changing it does | settings (current value) | 044/043 evidence |
  |---|---|---|---|
  | **A — inside the ACRO rate loop** | changes the **plant g800 was trained on**; the sim must change with it | `gyro_main_lpf_hz` 25 PT1 (sim matches: `gyroLpfHz`), dynamic notch (2D, Q 250, ≥ 30 Hz), `fw_*` rate gains | 25 Hz is aggressive for a fixed wing (6.4 ms, 11° at 5 Hz) |
  | **B — NN observation path** | changes **what the policy sees**; N5 must match it | `acc_lpf_hz` 15 BIQUAD (21 ms; no sim model found), AHRS: `ahrs_dcm_kp/ki` 2000/50, `ahrs_acc_ignore_rate` 15 °/s / `_slope` 5, `ahrs_inertia_comp_method` ADAPTIVE, `ahrs_gps_yaw_weight` 100, mag cal + `align_mag_*` | quaternion off 10–25° in pulls; at ≥ 15 °/s the accelerometer is ignored, so gyro + an under-calibrated QMC5883 carry attitude through every manoeuvre |
  | **C — nav estimator / GPS only** | changes the **nav frame** (path anchor, position/velocity inputs), not the plant | constellations (Galileo/GLONASS/BeiDou all OFF), `gps_dyn_model` AIR_2G, `gps_min_sats` 6, `inav_max_eph_epv` 1000 cm, `inav_allow_dead_reckoning` OFF, `inav_allow_gps_fix_estimation` OFF, `inav_default_alt_sensor` GPS with `inav_w_z_baro_p` 0.35 > `inav_w_z_gps_p` 0.20, `inav_w_xy_res_v` / `_z_res_v` 0.5, `inav_w_acc_bias` 0.01, `inav_baro_epv` 100 | 5–7 sats, every drop in a pull above 2.7 g; EPV/EPH at the 1000 cm cap marks every jump flight; 2.3 s freeze, then a 105 m XY reset; 26 m Z reset through the baro/GPS fade; baro is the heaviest Z input |
  | **D — fallback / safety** | what INAV does **after** autoc (N6) | `failsafe_procedure` DROP, RTH block, `nav_extra_arming_safety` | DROP needs no altitude estimate; RTH needs the full estimator |
  | **E — logging** | evidence only | blackbox 1/8, `-NAV_ACC -MAG -QUAT -RC_DATA` (B1) | N1 has to infer EPH/EPV and accWeight today |
  | **hardware** | — | Z vibration / prop balance (BACKLOG § 043 hardware), mag tumble calibration (BACKLOG § compass; 4.5 % gain spread, 16 % after the 09-07 retry) | vibration pins accWeight at its 0.30 floor; the 09-13 survey showed it is not the Z-divergence cause |

  - ⭐ **For the g800 re-fly: class C, D, E and hardware are free to change. A and B are not**, unless the sim changes with them (N5)
    and g800 is re-evaluated, or retrained, on the new plant. Otherwise the re-fly no longer tests the policy it is meant to measure.
    B is the hard one. Better attitude is the point of the work, but it moves the quaternion the policy has learned on.
  - ⚠️ **Article 1's GPS receiver is not recorded anywhere in the repo**, and multi-constellation support is not confirmed for it.
    Article 2's Matek M10Q-5883 (u-blox M10) does support it. First step, on the bench: CLI `status` prints the u-blox
    `HW Version`, the protocol version and a `GNSS Capabilities` block (`Galileo/BeiDou/Glonass active/default`, `Max concurrent`).
    Record it here. ⛔ **INAV fails silently on this**: if the receiver NAKs the GNSS config, `gps_ublox.c` (~L1108) resets all
    three `gps_ublox_use_*` back to their defaults. So setting `gps_ublox_use_galileo = ON` proves nothing. Confirm with `status`
    after a reboot, and with the sat count. If article 1 cannot use more than GPS, the sat-count fix is a receiver swap
    (e.g. the same M10Q as article 2), which is hardware, not config.
  - Candidate C changes already argued: Galileo (± GLONASS), receiver permitting (above); AIR_4G (above 2 g for 45 % of airborne time, above 4 g for 14 %);
    `GPS_ONLY` (BACKLOG § 043 close; needs enough sats for a usable GPS vertical); review the EPH/EPV cap and dead reckoning together,
    since together they produce the freeze and the reset.
  - Each setting is read in the INAV 8.0 fork source before it changes (units, and what it actually gates). The result is one
    recorded change set per class, A/B-able against the 11-08 and 10-18 logs.
- **N3 — Detection, ADVISORY ONLY (operator 2026-11-10: no change to operations).** Add EPH / EPV and the XY / Z valid bits to
  `MSP2_AUTOC_STATE` (INAV fork + `msp_autoc_state_t` + flight log v7). The xiao **flags the tick** on invalid, frozen (stale) or
  stepped estimates. **No new disengage.** The goal is the xiao's own IMU during autoc (N4), so INAV's estimate is being measured
  here, not made into a flight gate. Today `state_valid` only means *the fetch succeeded*. Absorbs the old *GPS dropout handling
  during NN control* item. `flight_gates.py` / `bench_join.py` flag such spans automatically (`--drop` exists).
- **N4 — A span-local inertial frame owned by the xiao (the architecture step).** At engage, take INAV's attitude / velocity /
  position once. Then propagate on the **xiao's own IMU** (the LSM6DS3 on the board: 021's never-built "AHRS cross-check",
  revived as the primary). It must run at IMU rate, not the 20 Hz MSP tick. GPS becomes a **soft cross-check**: a slow
  complementary correction, or a consistency monitor that flags and disengages. **No hard resets inside a span.** INAV stays the
  safety fallback (arming, failsafe, pilot override, return). Owed: IMU-to-airframe alignment calibration (the xiao board's mount),
  boot/arm gyro bias calibration, vibration isolation, temperature drift. Scope from N1's numbers. The same IMU is what a future
  xiao-hosted inner loop would use (BACKLOG § *dedicated fast inner loop*; keep that a separate step).
- **N6 — INAV's role during autoc: a simpler mode with sensible gates (study; operator 2026-11-10).** Autoc still depends on INAV
  for ACRO, so N4 alone does not take INAV's estimator out of the loop. Separate what each dependency actually needs:
  - **The ACRO rate loop** needs gyro rates only. `pidLevel` does not run in ACRO and ACRO never reads `attitude.raw[]` (043 spec
    § What ACRO is), so it should be immune to nav-estimator faults. *Verify this in the fork*, including anything the autoc
    MSP-override path pulls from nav or attitude.
  - **The nav estimator → xiao** (the MSP state) is replaced by N4.
  - **The safety fallback** (failsafe, RTH, pilot override, any nav-mode gates) is where INAV still consumes its own estimate.
  The study question: configure INAV during autoc as a gyro rate loop plus simple, estimator-independent gates (e.g. attitude or
  rate limits, an altitude floor from baro, link-loss failsafe), so a nav fault cannot change how the aircraft flies. Keep its full
  estimator only for the fallback after disengage. N2's config changes feed this.
- **N5 — Sim parity.** Whatever estimator flies, the sim must hand the policy *that estimator's* errors (span-local drift at N1's
  measured rates, attitude error under load), not perfect FDM state. If N4 is adopted, this lands **before the 045 bake**.

**Accept (before the Phase 0 re-fly):** N1 reported; N2 config set and recorded; N3 logging (fault → flagged tick, verified on the
bench by blocking GPS); B1 fields in the blackbox; the N6 dependency map written. N4 / N5 / N6 adopted or explicitly deferred, with the
N1 numbers that justify the choice.

## Why M2 moved

⚠️ **M2 was not blocked by M2 work — it was blocked by M1.** Three findings inside 041:

1. **The M1 source was not good enough to chase with.** t1 reached `pctInStreak` 16.1% against the prior
   M1's 30.9% tracking occupancy, and converged on a tight spiral that is objective-optimal under
   no-future inputs but is not the behaviour we want a tracker built on.
2. **The objective was missing its main term.** The policy cannot observe its own energy — the input vector
   carries `AIRSPEED` but no altitude — so every prior energy objective muted the whole regiment. M2
   inherits that verbatim.
3. **The datum chain is unverified** — eleven hops, four references, at least one open reconciliation.

Building a tracking problem on that foundation would have measured the foundation, not the tracking.

## What moved here from 041

| item | 041 refs | state |
|---|---|---|
| Predictor head — offline verdict, re-target or retire | T081–T088, FR-024…FR-027 | ⚠️ **T082's blind-gap distribution is a LOWER BOUND** — measured at 120°×90°, the real lens is 97.3°×60.8° |
| M2 bake, innovation channels (FR-025c–f), output topology 7↔3 | T089–T094, FR-005a | not started |
| Repoint tracker at the new M1 source; novel-geometry read | T095–T098 | **blocked on 041 producing a pinned M1** |
| M2 smoke | T060 | needs a v-current M1 source dmp |

## What 041 hands over

- A pinned M1 source in the current schema — **the actual dependency**.
- The energy observation + `Ps` axis, which M2 inherits (`CraftCommonInputs` is shared).
- A validated datum chain, proven by an M1 flight.
- The measured camera model (97.3° × 60.8°, equidistant, 0.304 °/px).

## ⚠️ The sensor model is NOT settled — a 1.56 mm lens is on order

Operator 2026-08-18: *"042 has ordered a 1.56mm lens which is close to 120deg fov. So that part of m2 is
dependent on range and response findings anyway."*

So M2's perception scope cannot be fixed yet. The tree currently carries the **1.8 mm** measurement
(97.3° × 60.8° derived from 320×200 @ 0.304 °/px), and 031 retired the single-fisheye-at-120° assumption
**for that lens**. A 1.56 mm near 120° would partially reverse that, which changes:

- the blind-gap distribution — 041's T082 figure is a **lower bound** measured at 120°×90°, and a genuinely
  120° lens moves reality back toward it;
- the **predictor's value**, since its whole justification is bridging blind gaps (fewer/shorter gaps ⇒ less
  to predict);
- whether the **birded pair** is needed at all — its main justification was reaching 120°.

⛔ **Do not spec M2's perception against either lens until 042 measures the 1.56 mm and reports range +
response.** Waiting is cheap; specifying twice is not.

## Sequencing

**041 (better M1, flown) → 042 (physics/camera, child of 031) → 043.**

042 sits between because the camera constants it produces feed M2's sensor model, and measuring them
depends on neither. ⚠️ Do not start 043 before 041 pins an M1 — the source dependency is hard, and the
`EvalResults` version bump means an older source dmp cannot be read at all.

## ⭐ Phase 1 inherits M1's fidelity — the hard perception work is phase 2

Operator 2026-08-18: *"Score grad in Xiao in m1 is virtual so we can do that. In m2 it'll proxy from camera
range estimation. Should be an interesting 043 study… Wait. First m2 flights will also be virtual so roughly
the same fidelity as today's m1 virtual paths."*

Because **M2 phase 1 is a virtual target with a synthetic camera**, the `SCORE_GRAD_*` input stays **exact**
there — the xiao computes it from a target it knows, exactly as M1 flight does. Nothing new is owed to fly
phase 1.

**The camera-proxy gradient — estimating ∂score/∂position from span-based range — is a phase-2 study.** That
is a genuine piece of research (range from span is the same inference the whole M2 sensor model rests on),
and it is now **deferred behind a flyable phase 1** instead of gating it.

⚠️ Read this together with the lens block above: phase 2's perception scope depends on 042's range and
response findings *and* on which lens is chosen, so specifying the proxy before 042 reports would be
specifying against an unknown sensor.

## Known traps carried forward

- **`FR-005a`** permits the tracker innovation channels as the one post-A1 layout change — legal because
  `TrackerInputs` has a separate genome and 040's T023 serialize split means no M1 source re-bake.
- **M2 has no direct distance or bearing** — only two beacon bearings and their separation, range inferred
  from span. That asymmetry is the whole M1↔M2 difference and should shape the input work.
- **Contribution/weight screens mis-rank inputs in both directions** (041 TA01 got it wrong three times).
  Ablate the set you intend to remove; ablation is non-monotonic.

---

## ⛔ TWO GATES BEFORE ANY 043 RUN (added 2026-08-22)

### Gate 1 — the 042 camera parameters must have LANDED

Operator 2026-08-22: *"we will have some new camera params from the 042 modelling that occurs in parallel,
so be sure before we start any 043 runs we've received the camera updates."*

⛔ **Do not launch a 043 bake against the current camera model.** 042 is running in parallel and its output
is a changed perception front-end. A bake started before those land is spent: its results are attributable
to a camera model that no longer exists, and it cannot be compared to anything after.

Check before launching: the 042 camera constants are merged, and `beacon_config.h` / the camera-projection
parameters are the 042 values, not 040's.

### Gate 2 — ⚠️ AUDIT THE 46 TRACKER-SPECIFIC INPUT SCALES

> 💬 **Needs discussion (operator 2026-11-10)** before it is acted on. Layout v4 now has 74 tracker inputs, so the 46 / 20 split
> below predates 044's craft history.

⭐ **M2 inherits the shared block's normalization automatically and completely.** `TrackerInputs` embeds
`CraftCommonInputs` (the same struct M1 uses), and `gather_tracker_inputs` calls the same
`writeCraftCommonInputs`. So all 20 shared slots arrive with the 041 P2-8 scaling for free — quat,
airspeed, gyro, accel, Es, boundary-closure, dist-to-boundary, inward-body, score-grad.

⛔ **But P2-8 never touched the 46 tracker-specific slots, and at least two are explicitly raw.**
`nn_inputs.h` says of `beacon_pair_span`: *"in RADIANS … with no scaling, no normalization, no clipping."*
`span_rate` is raw rad/s.

⚠️ **The magnitudes are in the danger band.** With the measured 0.772 m beacon separation, the pair
subtends 0.154 rad at 5 m and 0.011 rad at 70 m — an implied spread of roughly **0.02–0.03**. That sits
alongside `DIST_TO_BOUNDARY` (0.036), `SCORE_GRAD` (0.062) and `SPECIFIC_ENERGY` (0.090) — precisely the
quiet band that 041 proved was **never selected on at all** until P2-8 rescaled it. On 041-t5, every
input's weight investment sat flat at ~1.0 for 475 generations and the network never differentiated.

⛔ **This is not a prediction; it is the same defect, one milestone over.** 041 spent three runs
(t4/t5/t6) blaming the objective and the ramp before measuring the inputs. Do not repeat that. **Measure
the tracker-input spreads on a real M2 tick set BEFORE the first 043 bake** and rescale by measured p95,
the derivation already used for `kEnergyScale_m`, `kScoreGradScale` and the four P2-8 constants.

⚠️ **Unmeasured as of this note.** The 040-t4 pre-break archive does not carry the beacon input columns
(its `spP1/2/3`/`spdR` are the aux span-PREDICTOR OUTPUTS, not inputs), so the figures above are a physical
estimate from the beacon geometry, not a measurement. Getting the real numbers needs a tracker `--csv-only`
dump — cheap, and it is the first thing to do here.

⭐ The durable fix is the backlog entry *"Formal input normalization — measured statistics, not
hand-derived constants"* (specs/BACKLOG.md). If that lands first, both gates collapse into it and the
tracker block can never drift raw again.

---

## M2 input-normalization strategy (2026-08-22) — reasoned, not yet measured

⭐ **The shared 20 are already correct and PROVEN identical.** `TrackerInputs` embeds `CraftCommonInputs`;
`writeCraftCommonInputs` is a single unconditional path with no mode branch, called identically from
`gather_pathgen_inputs` (`evaluator.cc:464`) and `gather_tracker_inputs` (`:652`), and nothing overwrites a
`common.*` slot afterwards. M1 and M2 cannot diverge on those without editing one function. A guard test
pins it.

**So the work is the 46 tracker-specific slots.** Current state and proposal:

| slot(s) | n | today | conditioning | proposal |
|---|---:|---|---|---|
| `beacon_l/r_cep` | 12 | `[0,1]`, 1.5 = invisible sentinel | ✅ already unit | **leave alone** |
| `target_tilt_sin/cos` | 2 | ±1 by construction | ✅ | **leave alone** |
| `time_since_seen` | 1 | `tanh(ticks·scale)`, `[0,1)` | ✅ | **leave alone** |
| `beacon_l/r_x` | 12 | raw rad, ±0.849 (half-FOV H) | ⚠️ ~0.4 spread | scale — **see the isotropy trap** |
| `beacon_l/r_y` | 12 | raw rad, ±0.531 (half-FOV V) | ⚠️ ~0.3 spread | scale — **same constant as x** |
| `beacon_pair_span` | 6 | raw rad, 0.011–0.154 | ⛔ **~0.02–0.03** | **the real problem** |
| `span_rate` | 1 | raw rad/s | ⛔ unmeasured | follows span |

### ⛔ TRAP 1 — do NOT normalize x and y by their own half-FOVs

The obvious move is `x /= 0.849`, `y /= 0.531`. **That breaks a stated invariant.**
`camera_projection.h:90-93`: *"Both axes carry the SAME angular scale, so a given angular separation reads
identically at any orientation (FR-002)."* Dividing by different constants makes a fixed angular separation
read differently depending on whether the pair is horizontal or vertical — which is exactly the property the
beacon-pair geometry depends on.

⭐ **Scale both axes by ONE constant — the larger half-FOV (0.849).** H maps to ±1, V to ±0.63, isotropy
preserved. ⭐ Since FOV-transfer is explicitly not required (see the governing principle), tying the
constant to the optic is fine and gives a guaranteed unit range; a new optic means a retrain, which is
accepted. A measured joint p95 of `|bearing|` is the alternative if the beacons rarely reach the frame
edge and the p95 is well inside the FOV.

### ⭐ THE GOVERNING PRINCIPLE (operator 2026-08-22)

> *"we want good searchable dynamic range, not human units of measure — so, if we change FOV we retrain
> around that. no problem."*

⛔ **This demotes an argument made below.** Proposal B was partly sold on *"dimensionally identical to M1's
`DIST`"* — that is aesthetics, not conditioning, and it is not a reason to choose a transform. The only
criterion is **does the GA get uniform, searchable resolution across the operating range**.

⭐ **And it removes a constraint**: FOV-transfer is NOT a requirement. A genome may be tied to the FOV it
trained on; changing the optic means retraining. That frees the bearings to be scaled by the half-FOV
itself (see Trap 1), which guarantees unit range — the cost is only a retrain we have already accepted.

### ⛔ TRAP 2 — `beacon_pair_span` is a 1/distance cue, so linear rescaling barely helps

With the measured 0.772 m separation, span is `kBeaconSeparationM / range`:

| range | 5 m | 10 m | 20 m | 40 m | 70 m |
|---|---:|---:|---:|---:|---:|
| span (rad) | 0.154 | 0.077 | 0.039 | 0.019 | 0.011 |

**Half the dynamic range is spent inside 10 m.** Divide by a p95 of ~0.15 and the 20–70 m band still lives
in 0.07–0.13 of unit scale — better than today, but the far field, which is where reacquisition happens,
stays compressed. And the network has to learn `1/x` internally to recover range at all.

⭐ **PROPOSAL A (safe, P2-8-shaped)**: `span /= kSpanScale_rad`, p95-derived. Keeps semantics, fixes the
worst of the conditioning, low risk. Do this if 043 wants one variable changed.

⭐ **PROPOSAL B (better conditioned, changes semantics)**: reparametrize to **range**:
`r̂ = kBeaconSeparationM / span`, then scale by `kTargetDistScale_m` (26.0) — **the same constant M1's
`DIST` uses**. Likewise `span_rate` → **range-rate**, scaled by `kClosingRateScale_mps` (16.0), the same
constant M1's `CLOSING_RATE` uses.

⭐ **Why B is attractive beyond conditioning**: it makes M2's depth channel *dimensionally identical* to
M1's `DIST` / `CLOSING_RATE` pair. The tracker stops having to learn the reciprocal, and the two milestones
express range the same way — which matters if weights or intuition are ever carried across.

⚠️ **B's cost, stated honestly**: a singularity at `span → 0`, which is exactly the CEP-gated
invisible case (span is substituted to 0.0 when either CEP ≥ `CepGateThreshold`). B needs an explicit
sentinel — "no range estimate" must be representable and distinguishable from "very far". `time_since_seen`
already carries visibility, so the pairing is natural, but it must be designed, not left to a divide.

⭐ **PROPOSAL C (measured best on the stated criterion) — LOG SPACE.** Span covers a factor of 14 (1.15
decades) over 5–70 m. Map `log(span)` to `[-1, +1]`. Measured resolution uniformity, max step ÷ min step
across the working range (1.0 = perfectly uniform):

| transform | uniformity | comment |
|---|---:|---|
| **A** linear `span / 0.154` | **14.7×** | most resolution spent inside 10 m |
| **B** `range / 26` | **7.5×** | better, still biased to the far field |
| **C** `log(span)`, normalised | **1.5×** | ⭐ uniform per **octave** of distance |

⭐ **C is the right answer under the operator's criterion.** A 1/x cue spanning a decade wants a log
transform: equal input deltas then correspond to equal *fractional* range changes, which is what tracking
accuracy actually scales with — being 10 % off at 50 m and 10 % off at 5 m are comparable errors, and only
C gives them comparable input deltas. A and B both spend most of their dynamic range on one end.

⚠️ **C shares B's singularity** — `log(0)` at the CEP-gated invisible case — so the sentinel design is
required either way, and is not a reason to prefer one over the other.

⛔ **B and C are REPRESENTATION changes, not normalization changes. Do not bundle either with A.** 041's
lesson was that two changes at once make a run unattributable — t4 moved the objective and the inputs
together and cost three runs to untangle. If the appetite is there, go **straight to C** as a single
variable and skip B: B was motivated by a dimensional-symmetry argument the governing principle above
retires, and it measures worse than C on the criterion that survives.

### ⚠️ ALL OF THE ABOVE IS UNMEASURED

The spreads quoted for the bearings are inferred from FOV limits and the span figures from beacon geometry —
**not** from a tick set. The 040-t4 pre-break archive does **not** carry beacon input columns (its
`spP1/2/3` + `spdR` are the aux span-*predictor outputs*). ⛔ **First action here is a tracker
`--csv-only` dump and a spread table**, exactly as P2-8 did for M1 — 041 spent three runs blaming the
objective and the ramp before anyone measured the inputs.

⭐ If the *"Formal input normalization — measured statistics, not hand-derived constants"* backlog entry
lands first, all of this collapses into it: the stats would be measured and serialized with the weights, and
the tracker block could never drift raw again.
