# 026 — Implementation Plan (ACRO delegation)

Companion to [`spec.md`](./spec.md) (scope + intent) and
[`research.md`](./research.md) (analysis + escalation ladder). This
doc orders the work and resolves the open questions the spec left for
the plan phase.

## Plan ordering principle

**Sim signal first, flight prep last.** Flight hardware does not get
touched until the sim retrain measurably beats cadence7 on the
bang-bang metrics. If the retrain does not move the needle, we
escalate on the fitness/input side *before* spending a flight-day's
effort. Measurement reuses tools already in place from 024:

- [`plot_control_aggressiveness.py`](../024-sim-real-fidelity/plot_control_aggressiveness.py)
  on training `data.dat` — dCtrl / |out| plateau.
- [`plot_fitness_ramp.py`](../024-sim-real-fidelity/plot_fitness_ramp.py)
  comparing new run to cadence7.
- [`plot_bangbang_flight.py`](../024-sim-real-fidelity/plot_bangbang_flight.py)
  on a tier1 eval's data.dat (using it for sim data works — it reads
  `out=[...]` histograms regardless of source).

## Go/no-go gate (Phase 2 → Phase 3)

| Metric | cadence7 value | 026 target | Interpretation |
|---|---|---|---|
| `<|Δout|>` per tick (dCtrl, all-gen mean, late) | 1.0 | **< 0.8** | ≥ 20 % reduction in chatter |
| `<|out|>` per tick (amplitude, all-gen mean, late) | 2.2 | drop preferred, flat acceptable | Magnitude relief is a bonus |
| Best fitness at gen 400 | −35951 | **≥ −25000** | ≥ 70 % of cadence7, i.e. training did not collapse |
| Output value histogram at final gen | bimodal at ±1 | spread toward 0 | visual proof of regime change |

All three quantitative bars must hit; histogram is qualitative. If any
quantitative bar misses, we enter Phase 4 escalation before flight.

## INAV knobs on the flight FC

Audit of current [`xiao/inav-hb1.cfg`](../../xiao/inav-hb1.cfg) against
what ACRO-delegated autoc needs. Summary: **very few knob changes**;
the switch from MANUAL to ACRO is mostly operational (CH5 middle
instead of low) and most existing PID + filter settings are already
correct.

### Must change (config edits)

| Knob | Current | New | Why |
|---|---|---|---|
| `rc_expo` (profile 1) | 20 | **0** | Expo applies to MSP-override channels (verified in `inav/src/main/rx/rx.c`; override substitutes at `rcChannels[]`, before expo). Non-zero expo means NN output 0.5 doesn't map to 50 % of max rate. Flatten for linear sim-match. |
| `rc_yaw_expo` (profile 1) | 30 | **0** | Same reason. Yaw is held at 1500 neutral by xiao so this is belt-and-suspenders. |

That's it for hard edits.

### Verify (no change expected, but check before flight)

| Knob | Current | Needs | Notes |
|---|---|---|---|
| `msp_override_channels` | 47 (= roll, pitch, throttle, yaw, AUX2) | unchanged | Bits set on channels xiao needs to drive. Covering yaw with a neutral 1500 keeps the pilot out of yaw during autoc. Could trim to 7 (just main three) but not necessary. |
| `rc_filter_lpf_hz` | 250 | unchanged | 250 Hz LPF at 10 Hz NN command rate is essentially no filtering. Benign. |
| `rc_filter_auto` | OFF | OFF | Keep off. |
| `gyro_main_lpf_hz` | 25 | unchanged | Sim will match this value on its inner loop. |
| `dterm_lpf_hz` | 10 | unchanged | Standard PID D-term LPF. |
| `roll_rate` | 36 | unchanged | Flight-measured max ~430 °/s — matches `ACRO_MAX_RATE_ROLL = 430` in [`crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.h`](../../crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.h). |
| `pitch_rate` | 12 | unchanged | Flight-measured ~300 °/s → `ACRO_MAX_RATE_PITCH = 300`. |
| `yaw_rate` | 4 | unchanged | Flight-measured ~180 °/s → `ACRO_MAX_RATE_YAW = 180`. |
| `fw_p_pitch / fw_i_pitch / fw_d_pitch / fw_ff_pitch` | 15 / 5 / 5 / 70 | unchanged | Current flight PID. Sim tunes to produce matching rate step-response shape, not matching gain numbers. |
| `fw_p_roll / fw_i_roll / fw_d_roll / fw_ff_roll` | 15 / 3 / 7 / 50 | unchanged | Same — proven in flight. |
| `fw_p_yaw / fw_i_yaw / fw_d_yaw / fw_ff_yaw` | 20 / 0 / 0 / 100 | unchanged | Yaw PID; xiao holds yaw at 1500 so this only affects pilot yaw. |

### xiao-side change (small code edit, not pilot behaviour)

**Today's setup**: autoc engage force-selects MANUAL. xiao drives the
mode-select channel itself:

- [`xiao/src/msplink.cpp:676`](../../xiao/src/msplink.cpp#L676):
  ```cpp
  // CH6 (index 5) = 1000 → forces MANUAL mode (no INAV stabilization)
  state.command_buffer.channel[5] = 1000;
  ```
- `msp_override_channels = 47` includes bit 5, so this override is
  delivered.
- `aux 2 12 1 900 1200` maps MANUAL (permanentId 12) to AUX2 (CH6)
  range 900–1200. xiao's 1000 falls in that range → MANUAL is
  active during engage.

**For 026**: change xiao to drive CH6 to mid-range (1300–1700). No
aux entry maps to that range, so INAV falls back to its default mode
— which for fixed-wing is **ACRO**. The pilot's CH6 position is
irrelevant during engage because xiao overrides it.

```cpp
// Proposed for 026:
// CH6 (index 5) = 1500 → ACRO mode (INAV rate PID active)
state.command_buffer.channel[5] = 1500;
```

Audit for ambiguity on CH6 in the current aux table:

| aux entry | Mode | CH6 range |
|---|---|---|
| `aux 1 1 1 1800 2100` | ANGLE | 1800–2100 |
| `aux 2 12 1 900 1200` | MANUAL | 900–1200 |
| (no entry) | **ACRO (default)** | **1300–1700** |

Any value 1300–1700 is unambiguously ACRO. 1500 picked as a safe mid.

The xiao ARM/engage switch (CH9 via `aux 6 50 4 1600 2100` →
`BOXMSPRCOVERRIDE` on AUX5) is **not** in the override mask (bit 8
clear in 47). Pilot retains physical control of engage/disengage via
that channel. Only the mode-select channel is xiao-controlled.

### Throttle — what to do (TLDR: nothing)

INAV's throttle-related controllers:

- **ACRO throttle**: pure passthrough. No PID, no controller. This is
  what autoc-engage throttle will be under 026. Same as MANUAL today.
- **Nav-mode throttle (`CRUISE`, `POSHOLD`, `RTH`)**: INAV holds
  airspeed / altitude via `nav_fw_pitch2thr_*`. Not engaged by
  autoc; out of 026 scope.
- **TPA (`tpa_rate`, `tpa_breakpoint`)**: throttle-PID-attenuation
  scheduler. Currently `tpa_rate=0` (disabled). Leave disabled — no
  reason to vary PID gains with throttle for autoc.

**Disposition**: `outTh` stays a direct throttle command in 026. The
throttle saturation visible in flight-20260422 (NN pinning at +1) was
an envelope issue (rabbit groundspeed too fast for wind + HB1 Vmax),
not a control architecture issue. Adding a throttle PID on top does
not help with that. If bang-bang persists on throttle after 026's
pitch/roll succeed, revisit as a separate item (airspeed-PID in sim +
INAV CRUISE mode on flight — but that's 027+).

## NN output semantic update

`outPt`, `outRl`, `outTh` keep their **field names and storage type**
(float in [-1, +1]). The *interpretation* at the consumer changes for
pitch/roll; throttle unchanged.

| Output | Pre-026 meaning | Post-026 meaning |
|---|---|---|
| `outPt` | Elevator deflection fraction (-0.5 to +0.5 after sim bridge) | Desired pitch rate fraction (× `ACRO_MAX_RATE_PITCH` rad/s) |
| `outRl` | Aileron deflection fraction | Desired roll rate fraction (× `ACRO_MAX_RATE_ROLL` rad/s) |
| `outTh` | Throttle fraction | Throttle fraction (unchanged — INAV ACRO passes through) |

**Why not rename**: renaming the field forces cascading changes across
xiao, msplink logging, all analysis scripts, and data.dat parsers for
a cosmetic win. Keeping `out*` stable means code churn stays contained
to the places that *actually* change behaviour (the sim PID, the xiao
engage-mode selection, and the data.dat new diagnostic fields). Rename
is reversible later if it becomes a readability problem.

**How the semantic shift is communicated**:

- Code comments and docstrings at the sites that now interpret `out*`
  as rate (CRRCSim `inputdev_autoc.cpp`, `msplink.cpp`
  `convertPitchToMSPChannel`, etc.).
- New diagnostic fields in data.dat (below) carry the post-scaling rate
  values explicitly — `rateCmdP`, `rateCmdQ`, `rateCmdR` (rad/s).
- `docs/COORDINATE_CONVENTIONS.md` "Control Command Polarity & Scaling"
  section updated — pitch/roll command table gets a new "under ACRO"
  column.
- `sensor_self_check_lib.py` and renderer comments updated.

**No backward compatibility**. Old data.dat files from cadence7 and
earlier will not parse cleanly with 026 tools. They don't need to —
we have the artifacts we need from cadence7 already. Pre-026 files can
still be read with pre-026 scripts (git history).

## data.dat schema additions

All new fields, appended after the current columns (so existing
column names stay at their current indices for scripts that skip
unknown trailing fields). Flat-text format (no schema version), one
line per tick.

Proposed new columns (per axis p, q, r where applicable):

| Field | Units | What |
|---|---|---|
| `rateCmdP`, `rateCmdQ`, `rateCmdR` | rad/s | Desired body rate = `out* × ACRO_MAX_RATE_*` |
| `rateAchP`, `rateAchQ`, `rateAchR` | rad/s | Achieved body rate (FDM `getOmegaBody`) |
| `pidFF_P`, `pidFF_Q` | norm | FF contribution to surface, pre-SCALE |
| `pidP_P`, `pidP_Q` | norm | P contribution |
| `pidI_P`, `pidI_Q` | norm | I contribution |
| `pidIntP`, `pidIntQ` | rad·s | I-term integrator state (for windup diagnosis) |
| `pidSat` | bits | Saturation bitmask: bit0=pitch, bit1=roll (1=output clamped) |

Total added: 12 numeric + 1 bitmask. No throttle rate fields
(throttle is direct). Yaw PID fields omitted (xiao holds yaw at 1500;
sim can follow suit or keep yaw direct — decide in Phase 1.1).

Renderer, sensor_self_check_lib, cmd_response_scatter, bangbang_flight
will read through the new columns by name (all these scripts already
parse by header, not fixed position).

## S3 payload / AircraftState serialization

Renderer visualization of PID behaviour is deferred (design to come
later), but we serialize everything we'd want for it now so we don't
have to regenerate training data later:

- Extend `AircraftState` (serialized via cereal) with optional fields:
  `pidInternals_{P, Q}` — a small POD with the same fields as the
  data.dat additions above.
- These fields populate only during elite re-eval (matching the
  existing `gTraceIsEliteReeval` gate for `gCurrentPhysicsTrace`). No
  cost on per-individual training evals.

## Phases

### Phase 1 — Sim infra (autoc + crrcsim + renderer + analysis scripts)

Sim + tools only. No flight hardware touched. Target: a sim binary
that runs ACRO delegation with matching filters, writes the new
data.dat fields, and downstream scripts parse them.

- **1.1 CRRCSim ACRO PID re-enable**
  [`crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp`](../../crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp).
  Port the rate PID block from commit 9809dd6, adapt to the current
  post-024 code (cadence-fix already landed, so timing is clean). Use
  the existing `ACRO_MAX_RATE_*`, `ACRO_FF/P/I_*`, `ACRO_PID_SCALE`
  constants from the header. Enable the integral anti-windup
  already in the original design (±10 rad). Reset integrators on span
  start (the `gAcroLastTimeMsec` + `gAcroIntegral*` globals exist,
  already reset).
- **1.2 Inner-loop filters**: 25 Hz single-pole LPF on gyro rate
  before PID error computation (matches INAV `gyro_main_lpf_hz`); 10 Hz
  PT2 on D-term (matches `dterm_lpf_hz`). Parameters as compile
  constants in `inputdev_autoc.h` next to the ACRO constants.
- **1.3 data.dat diagnostic fields**: append the 12+1 new columns per
  the schema above. Update [`src/autoc.cc`](../../src/autoc.cc) header
  print and sprintf in the elite-reeval path.
- **1.4 AircraftState serialization**: add `pidInternals_*` fields,
  populated during elite reeval only. Cereal schema update is a
  breaking change — no backward compat needed; old binary data.stc /
  scenario snapshots won't reload cleanly.
- **1.5 Renderer data path**: update data.dat parser and span-loaded
  state to absorb the new fields. Visualization panel to come later
  (renderer can read and store without displaying for now).
- **1.6 Analysis scripts**: update
  [`flight-results/flight-20260417/sensor_self_check_lib.py`](../../flight-results/flight-20260417/sensor_self_check_lib.py),
  [`specs/024-sim-real-fidelity/plot_bangbang_flight.py`](../024-sim-real-fidelity/plot_bangbang_flight.py),
  [`specs/024-sim-real-fidelity/cmd_response_scatter.py`](../024-sim-real-fidelity/cmd_response_scatter.py),
  [`specs/023-ood-and-engage-fixes/sim_polar_viz.py`](../023-ood-and-engage-fixes/sim_polar_viz.py),
  and `tools/nn2cpp` (if it parses data.dat — check) for the new
  columns.
- **1.7 minisim**: leave it on MANUAL (direct-deflection) semantics.
  Add a one-line docstring note at
  [`tools/minisim.cc`](../../tools/minisim.cc) top: "minisim does not
  emulate ACRO; use for MANUAL-mode analysis only since 026." No code
  change.
- **1.8 Semantic docs**: update
  [`docs/COORDINATE_CONVENTIONS.md`](../../docs/COORDINATE_CONVENTIONS.md)
  "Control Command Polarity & Scaling" with a post-026 "under ACRO"
  column. Update in-code comments at the interpretation sites.
- **1.9 Smoke test**: sim runs; step NN rate command from 0 to +1,
  log achieved rate — should track to within rate-PID step-response
  time (~20–40 ms). Also test NN rate=0 holds attitude in a wind
  gust. Write `scripts/026_smoke_test.sh` that runs a short scenario
  and compares achieved rate to commanded rate offline.

### Phase 2 — Training + measurement (sim only)

- **2.1 Rebuild**: autoc, crrcsim, renderer, xiao (full `scripts/rebuild-perf.sh`).
- **2.2 Short training** (50–100 gens) to confirm convergence shape
  hasn't broken: fitness rises, no NaN, no early collapse. If it
  fails here we revisit Phase 1.
- **2.3 Full training** (400 gens) → **cadence8** (tentative name).
- **2.4 Eval suite** (`scripts/eval-suite.sh`): all tiers pass. Tier 0
  is against a freshly captured post-026 reference (determinism
  invariant carries over but bitwise reference is new).
- **2.5 Measure**:
  - `plot_fitness_ramp.py` — cadence8 line added to the existing
    comparison PNG (hb1-adjust4, test7, cadence7).
  - `plot_control_aggressiveness.py` on cadence8's data.dat —
    plateau values vs cadence7.
  - `plot_bangbang_flight.py` on a tier1 eval's data.dat —
    histogram spread.
- **2.6 Go/no-go**: compare against the gate table at top of this
  plan. If ≥ 20 % dCtrl drop and ≥ 70 % cadence7 fitness and visible
  histogram spread → Phase 3. Else → Phase 4.

### Phase 3 — Flight-deployment sprint (GO path)

Only if Phase 2 gate passes. Similar cadence to 024's Phase 8
sprint — flight FC config and xiao rebuild are the only flight-side
code paths.

- **3.1 INAV flight config**: `rc_expo = 0`, `rc_yaw_expo = 0` in
  profile 1 (autoc profile). No other config changes expected. Commit
  updated `xiao/inav-hb1.cfg`. Apply via INAV CLI on the flight FC.
- **3.1b xiao mode-select change**: edit
  [`xiao/src/msplink.cpp`](../../xiao/src/msplink.cpp#L676)
  `performMspSendLocked()` to drive `state.command_buffer.channel[5] = 1500`
  (ACRO default) instead of 1000 (MANUAL). Update the adjacent comment.
  Belongs in Phase 1 or here; placing here because it's the last
  flight-side change before flash and keeping it close to the INAV config
  edit makes the MANUAL→ACRO transition visible as a single reviewable unit.
- **3.2 Weights → nn2cpp → xiao rebuild**: extract cadence8 final
  weights, regenerate `nn_program_generated.cpp`, rebuild xiao.
- **3.3 Flash flight FC** with 026 xiao binary. Verify boot banner.
- **3.4 Bench rate-response characterization** (new, 026-specific):
  command a step on each axis via the bench test harness, record
  gyro rate over time. Sim side: same step in crrcsim. Compare rise
  times, overshoot, settling. If off by > 30 %, tune sim PID
  parameters (compile constants) and re-sim — NOT new training. Goal
  is sim-to-real match on rate response.
- **3.5 Bench preflight** (carry forward from 024 T114 checklist):
  polarity sanity, `ctl loop:` summary clean, compound-attitude holds.
- **3.6 Pilot ops update**: CH5 mid (ACRO) during autoc engage
  instead of CH5 low (MANUAL). Brief the pilot. TX trainer switch
  config verified.
- **3.7 Flight test**: same profile as flight-20260422 — short
  engage spans, varied path selections, preferably calmer wind to
  isolate the 026 change from the envelope issues.
- **3.8 Post-flight analysis**:
  - New: **`plot_rate_tracking_flight.py`** (write in Phase 1 if
    possible, or start of Phase 3.8) — per-axis scatter of
    commanded rate vs achieved rate from the flight blackbox. INAV
    ACRO PID tracking quality live.
  - Rerun `sensor_self_check`, `cmd_response_scatter`,
    `bangbang_flight`, `gravity_check` on the new flight data.
  - Write a `FLIGHT_REPORT.md` in
    `flight-results/flight-NNNNNNNN/` matching the 20260422 format.

### Phase 4 — Escalation (NO-GO path)

Only if Phase 2 gate misses. Flight hardware still untouched.

- **4.1 Escalation A — previous-output feedback inputs**. Add
  `outPt_prev`, `outRl_prev`, `outTh_prev` (or a HIST_PAST-sized
  window) to the NN input vector. Topology grows by 3 (or 3×N).
  Retrain as **cadence9** / **rate2**.
  - Re-run Phase 2.5 measurement. If gate passes → Phase 3.
- **4.2 Escalation E — Pareto selection on (tracking, effort)**.
  Structural change to `src/nn/selection.cc` (or wherever lexicase
  lives). Keep both "aggressive-and-accurate" and "quiet-and-moderate"
  champions. Retrain as **cadence10**.
  - Re-run Phase 2.5 measurement. If gate passes → Phase 3.
- **4.3 If both escalations fall short**: post an analysis doc in 026
  explaining what was tried and what we learned. 025 remains blocked.
  This is a real possibility and shouldn't be treated as project
  failure — the data will tell us what's missing from the
  architecture.

### Phase 5 — Feature close

- **5.1** Post-flight analysis doc (from 3.8) merged to `flight-results/`.
- **5.2** Update [`specs/025-craft-variations/spec.md`](../025-craft-variations/spec.md)
  Status to unblocked (if 026 succeeded).
- **5.3** Summary commit documenting 026's actual outcome. Merge 026
  into main.
- **5.4** `BACKLOG.md`: move any open follow-ups (throttle airspeed
  PID, rename `out*` to `rate_cmd*`, per-tick renderer PID panel) to
  the backlog.

## Task ordering within Phase 1

Authoritative decomposition lives in
[`tasks.md`](./tasks.md) Groups 1.a–1.e under User Story 1.
High-level intent: each group is a commit-sized chunk that leaves
a working build. The ordering is:

- **Group 1.a** (T010–T015) — CRRCSim ACRO PID re-enable + filters.
- **Group 1.b** (T020–T023) — data.dat schema + cereal serialization.
- **Group 1.c** (T030–T035) — analysis-script + renderer parser updates.
- **Group 1.d** (T040–T050) — docs + PID unit test.
- **Group 1.e** (T060–T061) — full rebuild + smoke test.

Analysis tools temporarily lose the new columns between Group 1.b
and Group 1.c but always build and pass their own parsing
gracefully. `build/autoc` works throughout.

## Open questions for implementation-time

1. **Sim PID tuning**: the 021-era `ACRO_FF/P/I_*` constants were
   empirical for that era's FDM + cadence. After 024's dt=0.005 and
   canonicalization, the numbers likely need a small re-tune to keep
   step response matching flight. Plan: bring up ACRO, run a step-
   input sim, check rise time & overshoot, adjust FF/P/I to match
   a reference target. Part of 1.9's smoke test.
2. ~~**Yaw in sim**~~: **RESOLVED 2026-04-23 (spec clarification Q3)**.
   Leave yaw passive in sim — no PID, no rudder input. HB1 has a
   tail fin for stability but no controllable rudder; INAV's yaw PID
   has no actuator on this airframe either. The `ACRO_FF_YAW / P_YAW
   / I_YAW` header constants stay as documentation but are not wired.
3. **NN input vector**: 021's plan removed `previous command` inputs
   on the theory that ACRO made them redundant. The current
   cadence7 topology inherited 024's input set which does NOT include
   previous-output feedback. Do we add previous-output feedback
   preemptively as part of 026, or save it for Phase 4 escalation?
   Recommendation: **save it**. 026 primary hypothesis is "ACRO alone
   suffices"; adding feedback inputs at the same time conflates two
   experiments. Phase 4 exists for exactly this escalation.
4. **Sim rc_filter model**: INAV's `rc_filter_lpf_hz = 250` is so
   high relative to 10 Hz NN that sim probably doesn't need to model
   it. Recommendation: **skip rc_filter modelling**. If post-026
   bench rate-response shows tracking lag we don't explain, revisit.

## Stopping rules

Clear when-to-stop-and-reconsider triggers:

- Short training (2.2) fails to converge: revisit PID sign/gain or
  filter stability. Don't proceed to full 2.3 until short run behaves.
- Full training (2.3) stalls below cadence7 baseline by > 30 %: stop
  at ~gen 100, investigate before spending the remaining ~4 hours of
  compute.
- Eval suite (2.4) fails determinism: probably a cereal schema issue;
  re-check 1.4.
- Bench rate-response (3.4) shows sim PID diverges from INAV by > 30 %:
  tune sim constants, re-sim, do NOT proceed to flight until match
  improves.
