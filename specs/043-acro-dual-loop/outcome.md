# 043 — outcome (⚠️ IN PROGRESS — feature not closed)

This file is the running record of what landed, what deferred, and why. It is completed at close-out
(T081). Until then it accumulates the deferral/decision records the tasks require (T026, etc.).

## US6 (Phase 4) housekeeping — status & deferrals (T026 · FR-076)

The FR-07x "housekeeping on the opened surfaces" items, each done or recorded deferred:

| FR | task | status |
|---|---|---|
| **FR-072** | T021 — `mod_inputdev` links `autoc_common` (no cherry-pick) | ✅ **Already satisfied** (done in 030 M1, `mod_inputdev/CMakeLists.txt:26-30`). Verified no autoc-source cherry-picking remains. |
| **FR-071** | T022 — `nnextractor -g` vs `dmp-dump --gen` footgun | ✅ **DONE**. `nnextractor -g` now takes the actual generation (via shared `extractGenNumber`), agreeing with `dmp-dump --gen`; `--help` records the meaning CHANGED. BACKLOG entry marked resolved. |
| (build) | T021a — two suites missing from the `make` ALL target | ✅ **DONE**. `shared_input_block_tests` + `nn_input_scaling_tests` added to both the `run_autoc_tests` `DEPENDS` list and the `COMMAND` chain — `make` now builds+runs them (verified locally). The 49/49 gate self-check is re-confirmed by the operator's T027 clean `rebuild-perf.sh`. |
| **FR-070** | T023 — formal **measured** input normalization (not hand-derived constants) | ⚠️ **DEFERRED** — already filed in `specs/BACKLOG.md` (*"[041 P2-8 follow-up … HIGH VALUE, LOW COST] Formal input normalization — measured statistics"*). It changes the NN input scaling → **bake-affecting** and needs its own validation (041's P2-8 rescale took multiple runs to trust); it is also cut-list item #1. Not pulled into 043's bake. |
| **FR-073** | T024 — type-safe NN sensor interface (name input columns by enum at the sites the new axes touch) | ⚠️ **RESEQUENCED to Phase 5** — the "sites the new axes touch" are the **observation-path** sites whose IMU transform is itself deferred to Phase 5 (see T015). Doing the enum-naming there keeps it with the code it hardens, rather than refactoring sites that do not yet exist. |
| **FR-074** | T025 — simulator sampling-time variation (20 Hz tick dither) | ⚠️ **DEFERRED** — new determinism-affecting variation feature (per-scenario seeded, replayable), not open-file housekeeping; cut-list item. Filed in `specs/BACKLOG.md` (043 T025 entry). Owner call on whether it rides the 043 bake. |

⭐ **Net**: the true housekeeping (build coherence + the tooling footgun) is done and verified; the three
bake-affecting / Phase-5-coupled items (T023/T024/T025) are deferred with rationale, exactly the cut order
the 043 implementation strategy names.

## Decisions of record so far

- **2026-08-25 (T003)**: as-run FDM substep is **200 Hz** (`Global::dt = 0.005 s`), not the assumed
  ~333 Hz. Phase-5 discrete constants use 5 ms. See `baseline.md` / `research.md`.
- **2026-08-25 (T006)**: the dynamic gyro notch is **modelled as absent** — effective Q = 2.5, ≥30 Hz
  floor ⇒ < 4° phase at 5 Hz. See `research.md` addendum D.
- **2026-08-25 (T015 split, operator)**: `craftCmQ → Cm_q` + Global carriers landed; the IMU
  **observation-transform** is **folded into Phase 5** (needs a sensed copy distinct from the truth
  fitness uses; also feeds `Cntrl_InavFwRate`). See `variation-inventory.md`.
- **2026-08-25 → superseded 2026-08-30 (T037)**: the node first went in the **global** config because
  `crrc_fdm.cpp:38` reads controllers there and `fdm_larcsim` had no per-model path (the literal task
  placement would have created a controller that never loads). ⭐ **Superseded by the operator decision
  below** — the per-model path now exists.
- **⭐ 2026-08-30 (operator): `<controllers>` MOVED to the model file** (`hb1_streamer.xml`, inside
  `<config>`), and `fdm_larcsim` gained the per-model load following the `fdm_mcopter01` pattern
  (`data-model.md` §3's named recipe). Rationale — *"we will likely make changes and even run this on
  different models as we go — retrain and all"*:
  * ⛔ **Kills a real footgun.** Under the global config, loading ANY other airframe silently inherited
    hb1's INAV gains. Verified fixed: `hb1_streamer.xml` → `[FDM] model-local controllers loaded: 1`;
    stock `zagi-xs.xml` → `none (no <config><controllers> node) — direct stick→surface`.
  * **Cohesion**: one file is the whole plant — aero, mass, `Cm_q`, *and* the FC tune it flies behind.
  * **Unblocks per-scenario/per-craft gain variation** later: the list is per-FDM-instance, so gains can
    vary like `craftCmQ` already does. (Impossible under the load-once global env.)
  * ⭐ **Makes the load VISIBLE.** `fdm_larcsim` now logs the outcome either way and **fail-louds** if a
    `<controllers>` node is present but constructs nothing (unknown name / missing gain key). That closes
    the one invisible-failure mode of this feature — previously a bad node meant `LoadList` printed an
    XMLException and the sim silently flew MANUAL. The node stays optional, because "no controllers" is
    the correct pre-043 behaviour for every stock model (not a silent fallback default).
  * Owned/reset correctly: rebuilt on every `LoadFromXML` (no `ReloadParams` duplication), deleted in the
    destructor, and `Reset()` at each `initAirplaneState` — ⛔ required for determinism, or a worker's
    second scenario would start from the previous scenario's integrator/filter state.
  * Runs per substep after the global `ControllerCallback` and **before** the 037 servo model, so servo
    lag still lands inside the rate loop.
- **2026-08-25 (T040/T041)**: no `getInputData` change — `ControllerCallback` auto-routes
  `pInputsFromUser`→controller, so the NN→rate-setpoint conversion lives in the adapter/core. ACRO is
  **always-on**, replacing MANUAL outright (Constitution III, no dual path).

## ⭐ Model validation against the 041-t7 flight (2026-08-30)

The 2026-08-23 flight was launched and landed in **ACRO**, so its blackbox carries INAV's own
rate-loop internals — `axisRate` (setpoint), `axisP/axisI/axisD/axisF` (per-term contributions),
`gyroADC`, `rcCommand`, `servo[]`. That makes it ground truth for the model, not just a behaviour
comparison. Analysis over the ACRO-only samples (`flightModeFlags == ARM`, 6,271 samples @ 59 Hz;
`ARM|MANUAL` segments excluded — MANUAL bypasses the PID).

### 1. Every contract constant confirmed from the aircraft's own log

`rollPID:15,3,7,50` and `pitchPID:15,5,5,70` → applying the contract's divisors (/31, /4, /1905, /31)
reproduces **all eight gains exactly**: kP 0.484 both; kI 0.750/1.250; kD 0.003675/0.002625;
**kFF 1.613/2.258**. Also `rates:36,12,4` (⇒ maxRate 360/120 ✓), `dterm_lpf_hz:10` + `type:2` (PT2 ✓),
`tpa_rate:0` ✓, `axisAccelerationLimitRollPitch:0` ✓ — i.e. every "deliberately not modelled" item
(T039) is confirmed off on the real aircraft.

### 2. Term-by-term: the implementation is correct

Feeding the real `rcCommand` + `gyroADC` into the model and comparing against INAV's own recorded terms:

| | roll | pitch |
|---|---|---|
| setpoint chain (`rcCommand/500 × maxRate` vs `axisRate`) | max err **0.96 °/s**, r = 0.99997 | 0.96 °/s, r = 0.99992 |
| **FF** vs `axisF` | mean err **0.21**, r = **0.99998** | 0.46, r = **0.99994** |
| **P** (incl. Gaussian attenuation) vs `axisP` | mean err **0.53**, r = 0.99917 | 0.52, r = 0.99848 |

Residuals are at blackbox integer-quantisation level. ⭐ The 0.999 P-correlation independently
confirms the **Gaussian setpoint attenuation** is right — a wrong attenuation could not track `axisP`.

### 3. The pitch authority ceiling is HARDWARE-REAL, not a sim artifact

INAV's own `axisF[1]` maxes at **270** of the ±500 budget on the real aircraft (model: 271). So at full
pitch stick ACRO commands only **54%** of available elevator, versus roll's 100% (FF 581 → clipped to
500). This is `maxRate × kFF`: 120×2.258 = 271 vs 360×1.613 = 581. Nothing rescues it — P and D are
attenuated to ~0 at full stick, and the I-term is **locked**: INAV's `pid.c:767` refreshes
`targetOverThresholdTimeMs = millis()` every iteration while `|rateTarget| > 0.2·maxRate`, so *holding*
the stick keeps the lock engaged permanently (`aI = 0`). Verified against the fork's source; the model
matches this behaviour.

⚠️ Pitch **authority per unit command** is nearly right (sim 0.368 vs real 0.439 °/s per PID unit,
**0.84×**), so full pitch command still rotates the sim at ~100 °/s vs real ~119. The airframe is not
the problem.

### 4. ⛔ The finding that matters for the bake: WHERE the policy operates

σ scales with maxRate, so the attenuation depends **only on stick fraction** `f`:

> **aP = aD = exp(−17.33 · f²)** — half authority at **f = 20%**; `aP ≈ 0` at full stick, *regardless of
> the `rates` setting*. (Lowering `rates` would make a given physical rate a LARGER fraction and
> attenuate MORE — the obvious lever is backwards.)

| | \|cmd rate\| | \|achieved\| | saturated | **aP** | in linear band |
|---|---:|---:|---:|---:|---:|
| **human pilot, real ACRO** | 16.5 / 13.4 °/s | 31.7 / 25.6 | 0.0 / 2.6% | **0.909 / 0.816** | **91.6 / 81.2%** |
| 043 smoke NN, gen 63 | 311 / 108 °/s | 206 / 103 | 60.8 / 72.6% | 0.037 / 0.035 | 2.9 / 2.7% |
| ratio (roll/pitch) | 18.9× / 8.0× | 6.5× / 4.0× | | | |

⭐ **The human flies where the loop has ~85% of its P/D authority — fully closed and actively damping.
The policy commands 8–19× more rate and lives at the attenuation floor, where the loop is pure
feed-forward.** Bang-bang commanding is endemic in autoc (037, 041), but its *consequence* is new: under
MANUAL, pegging meant full surface; under ACRO, pegging **bypasses the inner loop**.

⛔ **Open pre-bake question (owner decision).** 043's premise is that the fast inner loop damps the
2–5 Hz oscillation; that damping lives in P/D. If the baked policy operates at aP ≈ 0.03, ACRO becomes a
bang-bang *rate* command and the oscillation may well survive. The lever is **ours** — the action space
is FR-016, needing no INAV config change and so no FR-012a bench burden: map the NN tanh onto a
*fraction* of maxRate (`×0.25` ⇒ f ≤ 0.25, aP ≥ 0.34, max 90/30 °/s — still ~3× what the pilot used, and
achieved rates land in the real aircraft's measured range). Proposed test: a short A/B smoke,
current mapping vs 0.25×, judged on fitness climb **and** the rate-spectrum/aggressiveness measures.
This also supplies direct evidence for T061/FR-030's leading candidate (rate-tracking error as an NN
input): the policy achieves 57% of what it commands and has no channel that reports it.

### 5. Subjective validation — the operator flew it

Manual flight in the sim (video config, USB stick, ACRO loop active) initially felt wrong — *"no pitch
for climb at all"*. Root cause was **input binding, not the controller**: crrcsim's shared axes parser
(`inputdev.cpp:710-731`) defaults ELEVATOR polarity to **−1 for mouse but +1 for joystick**, and every
axis other than aileron/elevator defaults to `-1` (unmapped). ⚠️ `bindings->getChild("axes", true)`
creates the node when absent, so a missing binding fails *silently* into these defaults. After
recalibration the operator's verdict: **"NOW it feels about right"** — the model reproduces real ACRO
feel to a pilot who has flown this airframe in ACRO. Command/surface ranges confirmed against
`docs/COORDINATE_CONVENTIONS.md:315-317` (pitch `−pitch/2`, roll `roll/2`, throttle `(thr/2)+0.5`).

## The action-space experiment — A/B/C (2026-08-30)

Three arms, each flown manually by the operator (who has flown this airframe in real ACRO) and
scored against the 041-t7 flight data. The knob added for the experiment is `commandScale`
(per-axis XML attribute on `<InavFwRate>`, adapter-side — the action space is FR-016, ours; the
validated `inav_fw_rate.h` core was deliberately left untouched).

| arm | change | full-stick pitch | aP at a useful 58 °/s pitch | operator verdict |
|---|---|---:|---:|---|
| **A** | baseline (`commandScale` 1.0, `pitch_rate` 12) | 120 °/s / **54%** elevator | 0.016 | marginal — "no pitch for climb" |
| **B** | `commandScale` 0.35 | 42 °/s / **19%** | — | ⛔ **"more sluggish"** — rejected |
| **C** | `pitch_rate` 12 → **24** (maxRate 120 → 240) | 240 °/s / **100%** | **0.363** | ✅ **"feels about right … ACRO is felt for sure"** |

### Why B failed and C worked — the coupling, stated once

Surface authority is `FF = commanded_rate × kFF`; damping is `aP = exp(−17.33·f²)` where `f` is the
commanded *fraction* of maxRate. **`commandScale` lowers the rate, so it lowers authority and
damping together** — it can only trade one for the other (arm B: damping bought with 19% elevator).
⭐ **`maxRate` is the only knob that decouples them**: raising it leaves `FF` at a given physical rate
unchanged while shrinking `f`, so authority and damping both improve. Arm C gets 100% elevator
*and* 22× the P/D authority at normal flying rates.

### The root cause, localised

`pitch_rate = 12` is **below INAV's own default of 20** (allowed range 4–180) and caused both symptoms
at once. Full-stick FF = 120 × 2.258 = 271 of the **fixed** ±500 `pidSumLimit` (hard-coded in
`getPidSumLimit`, not configurable), and σ ∝ maxRate put every useful pitch rate at the attenuation
floor. ⭐ **Roll was never broken**: at 360 °/s a useful 95 °/s is already f = 0.26 → aP = 0.30.
Design note — full stick exactly fills the ±500 budget at `rate = 500/(10·kFF)`: **roll 31, pitch 22**.
We run roll 36, so the top **14% of roll stick is clipped dead range** (operator 2026-08-30: *"the roll
is a bit hot"*). Lowering roll toward 31 would remove the dead band at a small damping cost
(aP 0.30 → 0.20 at 95 °/s) — not done, recorded as an option.

### ⛔ Gate before the bake — the sim now models an aircraft that does not exist

Arm C is **sim-only**: `xiao/inav-hb1.cfg` still has `pitch_rate = 12`. Baking against arm C and then
flying `pitch_rate 12` would diverge *precisely in the axis this feature exists to fix*. So before the
27 h bake, one of:
1. **Commit to the INAV change** — `pitch_rate 12 → 24` on the flight FC, changed identically in sim,
   bench-verified, folded into the config of record (FR-012a discipline). ⚠️ This is a deviation from
   the spec's "gains and rates stay as-is" and is an operator decision (2026-08-30: *"xiao and inav
   need updating for this so we will consider"*); **or**
2. bake on arm A's config and accept the 54% pitch ceiling.

⚠️ **T050a also applies**: arm C is a material plant change, so the T044 trainability gate should be
re-run on arm C (a ~70 min smoke) before committing 27 h. Arm A's trainability result (best −1576.9
at gen 328, 16/16 scenarios, past the basic-m1 400-gen baseline of −1320) was measured on a plant the
bake would no longer use.

## Phase-5 model gates (US2)

- **T037a ✅ (2026-08-25)**: second clean `rebuild-perf.sh` after the `mod_cntrl`/top-level CMakeLists
  changes — **50/50 suites, 0 failures**, binaries present. Constitution IV satisfied with the controller
  compiled in.
- **T043 ✅ (SC-014 part 1)**: all-attitude zero-command sweep, model standing alone
  (`tests/inav_fw_rate_tests.cc::AllAttitudeZeroCommandSweep`). Across a bank×pitch grid over the sphere,
  zero command **nulls body rate** at every attitude (|residual rate| < 2 °/s from a 20 °/s disturbance)
  and **holds attitude WITHOUT self-levelling** (mean restore fraction < 0.15). A contrast ANGLE outer
  loop on the same core+plant restores bank (> 0.8), and the ACRO residual is **not** sign-correlated with
  bank — so the FR-019a discriminator is proven, not trivially true. ⚠️ This is the model-level sweep; the
  full crrcsim-FDM attitude behaviour is additionally exercised by the operator's T044 training run.
- **T044 ⏳ trainability (SC-004)** — operator-driven: seed a short run from a known-good genome and
  confirm the GA improves rather than stalls. **Not yet run.**
- **T045 ⏳ determinism (FR-015)** — operator-driven: identical seed+config reproduce identical
  trajectories, and the eval-vs-training bitwise ScenarioScore gate holds. The `rebuild-perf.sh` clean
  build passed, but the eval-vs-training comparison itself is a separate run. **Not yet run.**
