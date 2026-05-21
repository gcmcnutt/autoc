# Feature Specification: 031 Beacon-Camera Optical Perception — Phase 1

**Feature Branch**: TBD (likely `031-beacon-camera` when this is unparked; currently being drafted inside the `030-tracker-mode` working branch alongside the 030 v1 wrap)
**Created**: 2026-05-10
**Revised**: 2026-05-12 — operator firmed the beacon + camera + recording substrate; added a Beacon Test Flight 1 use case (paired-craft field recording) so the optical chain gets its first real-air test before any FPGA work
**Status**: DRAFT — spec + clarify only, no plan/tasks yet. Bench experiments will iterate this heavily once hardware lands.
**Input**:
- [docs/aircraft_tracker_handoff.md](../../docs/aircraft_tracker_handoff.md) — architectural handoff doc; source of truth for phase-1 decisions until bench evidence re-litigates them.
- [camera_considerations.md](camera_considerations.md) — sensor selection + link budget + Gold-code acquisition/tracking math; consumed by this spec for the camera + code-length decisions.
- User direction 2026-05-10: focus 031 on **LED + pyramid design**, **camera + filter**, **raw recording for sim analysis** — defer FPGA/DSP/wire-protocol to follow-on phases (031-fpga, 031-protocol).
- User direction 2026-05-12: firm beacon to **4× 1W LEDs on a 4-sided pyramid (~2.5 cm base) covering 270° spherically with the unlit cone pointed INBOARD toward the craft centerline (visible from outboard, above, below, ahead, behind — only the fuselage-shadow direction is blind), wired together as one Gold-code-modulated drive group, powered from the main 3S LiPo via in-wing-tip flight hardware**; firm camera to **320×240 @ 240 fps, 120° FOV (wide side), 850 nm bandpass**, recording **raw frames to onboard SD flash for ground analysis**; first field experiment is a **paired-craft test flight** (beacons on target craft, camera+SD recorder on tracker craft, both auto-record, no live link, no NN engagement) — field data feeds a follow-on sim-noise / FPGA-pipeline tuning spec.

> **Context: where 031 sits in the program arc.**
>
> 030 trained a chase NN against a synthetic beacon-projection front-end ([src/eval/camera_projection.cc](../../src/eval/camera_projection.cc)). The NN's input vector is `(x_i8, y_i8, cep_u8)` per beacon plus history slots. The simulator's beacon projection is analytic — perfect detection, no clutter, no sensor noise. That's exactly the right tool for evolving a controller, but it cannot be flown until a real hardware front-end emits the same `(x, y, CEP)` triples at the same wire contract.
>
> 031 is the hardware/firmware/optics work that produces those triples from real photons. The handoff doc lays out an end-to-end architecture (sensor → MIPI → FPGA five-stage pipeline → I²C → autopilot). **031 phase 1 (this spec) covers only the front of that chain**: the emitters (LEDs + pyramid pods on the target's wingtips), the receiver optics (sensor + lens + 850 nm bandpass filter on the tracker), and a **raw-frame capture/recording capability** so the bench team can collect real footage and iterate on the simulator's noise model + the FPGA pipeline design with ground-truth data — all before any FPGA gateware is written.
>
> Follow-on 031 specs (NOT this one):
> - **031-fpga** — Stages 1-5 of the FPGA pipeline (background subtract → CCA → track table → code correlator → output formatter). Specs the gateware + Python golden model + HDL co-sim.
> - **031-protocol** — I²C register map, sentinel CEP encoding, host-side polling library. Locks the wire contract once the simulator-side input variants have been compared (per handoff §6.11).
> - **031-integration** — Real-flight integration: stress + sim-to-real comparison runs, on-aircraft NN-in-the-loop, then the deploy of 030's trained weights against real beacons.

## Clarifications

### Session 2026-05-12

- Q: How should the spec handle IR eye-safety classification given 4× 1 W 850 nm LEDs × 2 pods at ~50% duty, with operators + spectators at meters during bench + flying field? → A: **Dual-mode beacon operation: low-power ground/handling mode + high-power flight-armed mode**, selected by a PWM input from a spare RX channel on the airframe via a standard 3-wire RC servo connector (V_bus + GND + PWM signal). MCU decodes pulse width: <1.5 ms → low-power, ≥1.5 ms → high-power. Fail-safe: absent / invalid PWM defaults to **low-power**. Operator workflow ties high-power mode to the airframe's arming switch on the TX. Eye-safety classification (IEC 62471) bench-measured separately for **both modes**, documented in bench-log; low-power mode targets unconditional Class 1 (eye-safe at zero distance), high-power mode requires a documented safe-viewing distance from the flying-field perimeter.
- Q: How should the SD recorder signal its state to the pre-flight operator so silent-failure modes (no card, FS error, sensor enumeration failure, ring-buffer overrun, etc.) don't waste flight sessions? → A: **Mandatory blinking-LED status indicator on the recorder housing**, following the xiao NN-controller convention (see [xiao/src/util.cpp:15-32](../../xiao/src/util.cpp#L15) `heartBeatLED()` + [xiao/include/main.h:14-20](../../xiao/include/main.h#L14)). Most early-flight builds use eval/dev-kit boards with onboard R/G/B LEDs already wired; use them. The contract: "writing" must be a recognizable blinking pattern distinguishable from "fault" (steady or off) and "init" (different cadence). Documented blink-code table goes in the operator quick-reference alongside this spec.
- Q: How durable should the SD clip-file format be against mid-flight interruption (SD-full, power-loss, brown-out, crash)? → A: **Pre-allocated clip file + chunked direct-sector writes** (~1 s blocks, ~18.5 MB each). Each chunk is independently parseable by the FR-4.3 Python loader so surviving chunks recover whatever ran before the fault. On detected fault (SD-full, write error, brown-out signal): immediate-truncate the current chunk + finalize the file. Worst-case loss: ~1 second of trailing frames — acceptable per operator direction ("the final frames before a hull crash, well..."). **Raw-LBA block writes (skip the FS entirely) reserved as a fallback** if bench-benchmarking shows FS bookkeeping bottlenecks the 18.5 MB/s sustained budget. Per-frame fsync explicitly rejected — likely incompatible with sustained 240 fps on most platforms.
- Q: What is the US6 (Beacon Test Flight 1) acceptance floor — what counts as "a successful first flight"? → A: **Infrastructure validation**, not detection-performance validation. US6 passes when the field session produces a **standard-format raw recording that downstream DSP / FPGA-sim / visualization tools can ingest** end-to-end. The optical chain produced photons → pixels → standard file → Python loader → downstream tool successfully. Specifically: (a) clip is loadable via the FR-4.3 Python utility without errors, (b) clip plays through at least one detection/correlation DSP producing a sane output (even if 0 beacons are detected — the *flow* is what's validated), (c) clip is loadable as input to a visualization tool (e.g., per-frame display + per-pixel time-series). **Beacon count, tracking duration, and detection robustness are explicitly NOT acceptance criteria** — those characterize the operating envelope and belong to the follow-on noise-cal/DSP-tuning spec (Q10). The corollary: the recording's information content must be **lossless raw** (FR-2.5 + FR-4.2) so it remains a valid input to whatever DSPs / sims / visualizations get built on top of it later.
- Q: Where should the canonical recording-file-format definition live, and what is its bit-depth contract? → A: **Dedicated [`data-format.md`](data-format.md) doc alongside this spec**, with versioned chunk-header schema (`format_version` uint16 per chunk). FR-4.3 Python loader is the reference implementation; mismatched `format_version` is a hard load error. **Bit depth is selectable: 8-bit baseline OR 10-bit option for post-hoc AGC-dynamics simulation** (camera moving fast through sun→shadow + sun→dark transitions is one of the primary phase-1 study targets — extra 2 bits of dynamic range lets the ground analysis simulate AGC responses that the live camera couldn't capture). Data-rate budget scales: 18.4 MB/s @ 8-bit unchanged; **10-bit packed (1.25 B/pixel) = 23.0 MB/s**; 10-bit unpacked-16-bit (2 B/pixel) = 36.9 MB/s — bumps the SD-card-class requirement to **V60 minimum if 10-bit is selected**, V90 preferred. **Architecturally**: the flight recorder build is naturally a small **FPGA-eval-board program** (Lattice CrossLink-NX-EVN or similar) doing camera-config + bulk-record to onboard SD — building this **also proves camera→FPGA ingest**, the prerequisite for 031-fpga proper. Offload paths: (a) USB download from the eval-board, OR (b) pull the SD card and read directly on the PC. Both produce identical files.

### Session 2026-05-14 (5-on-cube design + Topology A + chip-rate + clock-drift)

- Q: 4-on-pyramid vs 5-on-cube — which LED arrangement gives "fairly even spread" across the 270° envelope? → A: **5-on-cube wins** per polar-plot analysis ([plot_led_configs.py](plot_led_configs.py)) — 0.41 min/max uniformity vs 0.00 (full nulls) for 45°-tilt pyramid or 0.10 for 60°-tilt pyramid. 5-on-cube has no nulls, leaks coverage past the design rim into the inboard direction, costs 25% more LEDs + total power. **Half-cube enclosure**: 2.5 × 2.5 × ~1 cm, mounted in **45° diamond orientation** on wing-tip top surface for reduced pressure drag (wedge frontal profile vs flat-face). LED part firmed: **Lumileds Luxeon IR Compact 850 nm — L1IZ-0850000000000** (DigiKey 7243418).
- Q: 1 A per LED is too much power for the 5 V BEC ESC rail; can we cut power and/or run from raw battery? → A: **Yes — drop to 300 mA per LED** per the link-budget analysis (100 m daylight retains ~23 dB margin even at 100× real-world derating). Cuts per-pod power 5× (~2.85 W peak / ~1.43 W avg vs prior 14.5 W / 7.25 W). Power source: **dedicated 2-wire JST cable from main 3S pack** (not BEC, not shared with servos). EMI isolated from RC signal harness.
- Q: With 15-bit Gold code, what's the sensible chip rate considering the 10 Hz FC sample rate? → A: **100 Hz chip rate (firm)** — 150 ms code period = ~1.5 FC samples cold acquisition. 240:100 = 12:5 non-integer ratio sidesteps the prior 48 Hz / 240 fps 5:1 integer-aliasing edge case. 2.4 frames-per-chip oversampling preserves matched-filter margin. Ongoing position-fix updates at 240 fps regardless (only CEP confidence is gated by code-period rate).
- Q: Are there Microchip parts that manage PWM AND control a series of LEDs from outputs directly? → A: **No general-purpose MCU sources 300 mA per pin** — switching elements (FETs or LED driver ICs) are always required for power-class LEDs. **Topology A**: integrated LED driver IC (**Microchip MIC3232** primary / Diodes AL8807 alternate) handles 9-12.6 V input + CC switching to 5 parallel LEDs at 300 mA; ATtiny412 MCU does small-signal PWM control of the driver IC's DIM pin (Gold code + mode-select scaling). Single chip per duty (driver + MCU) keeps part count + PCB area + EMI sources minimal. Topology C (5 per-LED FETs gated by MCU GPIOs) noted as listed-alternate prototype path. **→ SUPERSEDED later same day (2026-05-14) by the "path C standalone-battery" clarification below**: Topology A's 9-12.6 V tethered-from-3S premise was dropped in favor of a 1S-battery boost driver (TI LM3410); MIC3232 is no longer the primary part. Topology A is preserved here only as design-history context.
- Q: What's the beacon-channel reliability when MCU clock drifts ±5% across operating range? → A: **Plan-time analysis required (NFR-4)**: simulation across -5% to +5% chip-rate drift, computing post-correlation SNR loss + cold-acquisition probability + tracking probability at each step. The simulation picks the implementation: (a) external crystal on the beacon MCU (~$0.30, eliminates problem), (b) receiver-side multi-hypothesis matched filter (031-fpga gate cost), (c) factory-calibrated internal RC only if simulation shows 99% acquisition is met, or (d) both crystal + multi-hypothesis (defense in depth). Bench-verification follows the plan-time choice.
- Q: M2 beacon effort is transitory (path forward is scene-recognition, not coded beacons). Can we drop the tethered + dual-mode design entirely and go with a standalone-battery pod? → A: **YES — path C**: standalone 1S-LiPo-powered pod with single-mode operation at 300 mA per LED (IEC 62471 RG0 at typical distances, no dual-mode safety contract needed). Manual on/off switch on the pod body for field activation. **2-bit solder jumpers or DIP switch** for Gold-code selection (supports 4 codes; phase 1 uses A/B). **1S 100 mAh battery**: ~2 g, ~14 min runtime per charge — single-flight runtime. Drop the tethered variant entirely from the spec; if a future heavier-power spec needs it, retrieve from git history. Eye-safety analysis at 300 mA: **RG0 at ≥30 cm for momentary glances, RG0 at ≥1 m unconditionally**. 5× power cut vs original 1 A design buys ~10× exposure-time safety margin.
- Q: Does 15-bit at 240 fps give enough codes / error margin, or should we go to 31-bit + 480 fps now? → A: **Stay at 15-bit / 240 fps for this transitory M2 effort.** N=15 gives 17 codes in the family (4 used via 2-bit code-select, 13 reserved) and 1-bit error tolerance — sufficient at our 30+ dB derated link-budget margin. **31-bit + 480 fps is documented as the clean upgrade path** in FR-1.3 (5-bit error tolerance, 31 codes, same ~150 ms acquisition latency, doubled SD bandwidth). Future multi-beacon or harder-noise scenarios trigger the upgrade.
- Note (operator 2026-05-12, post-Q5): **The phase-1 raw recordings are the input to acquisition-time / DSP / FPGA-pipeline simulations** for actual flight-DSP design — not just visualization or noise-cal. The Q10 follow-on will replay these clips through simulated correlators, AGC curves, and FPGA-pipeline Python models to characterize: acquisition latency under realistic photon flux, correlation SNR vs aspect angle, FOV-coverage utilization, beacon-intensity sufficiency at design range, pyramid-dimension trade-offs (e.g., is the 2.5 cm base too small to dissipate heat / spread the unlit cone correctly?), frame-rate adequacy under body-rate-induced motion blur. The "early phase is proving the beacon→camera chain" is **expected to be non-trivial** — the optical, mechanical, electrical, and timing variables interact in ways the link-budget math can't fully predict, and the recordings are how we surface that interaction.
- Note (operator 2026-05-12, post-Q5): **The Lattice eval-board flight-recorder is fine for early eval on a larger carrier craft** (e.g., the hb1 or a larger trainer-class airframe with margin to lift the eval-board weight). It is **over the weight budget for the eventual target tracker craft**. When the perception loop moves toward production hardware (likely 031-integration or a dedicated small-form-factor follow-on), we will pursue **smaller setups or custom builds** (bare camera + small FPGA on flex PCB, or the bare-die OG-series sensor + custom carrier from camera_considerations.md). Phase 1 explicitly accepts the eval-board weight cost as the price of getting first-flight data fast and reusing the FPGA work for 031-fpga.

### Session 2026-05-17 (post-/analyze + post-/plan refinements)

- Q: Is the Phase 1 bench-mode (FR-4.1) USB-tethered recording path built as a custom FPGA gateware variant, or satisfied by a different mechanism? → A: **Multi-eval-board de-risk strategy.** Bench mode is satisfied by an **off-the-shelf USB-camera eval board** (Arducam B0264 UVC USB shield + OV9281 B0162 module, or equivalent UVC-compliant NIR-capable sensor module) feeding a host PC over UVC. This gives **live frame display + per-pixel inspection during first-light, optical-chain bring-up, EMI debugging, exposure tuning** — without requiring any Lattice FPGA gateware work. The bench-mode recording path (FR-4.1 + FR-4.2 file output) is a **Python script consuming UVC frames and writing the canonical `.clip` format**, so the loader contract is exercised end-to-end on the bench-mode path before flight gateware lands. The Lattice CrossLink-NX-EVN + bare-die camera is the **flight-mode** path (FR-4.1b), built in parallel and de-risked by the bench-mode optical proof. **Similarly**, the beacon firmware is brought up on a **Microchip ATtiny412 Curiosity Nano dev board** (~$10, USB-UPDI built-in) with breadboard wiring to the LED string + boost driver BEFORE the half-cube enclosure or hand-soldered pod exists. Each chunk of work proves out on its own eval board, then integrates.
- Q: Does Phase 1 require procuring a calibrated NIR power meter (~$500–2000) to perform the FR-3.4 #3 IEC 62471 measurement, or can the measurement be deferred / substituted? → A: **Defer the formal calibrated measurement; use design-analysis + qualitative substitutes + an operator-only 1 m viewing rule.** The path C 300 mA design has ~10× exposure-time safety margin vs the prior 1 A design and is unconditionally RG0 at ≥1 m per the FR-1.7 link-budget table; the 200 mm bench measurement only matters for sustained close-range staring, which doesn't occur in operator-only Phase 1 work. Substitutes: (a) the FR-1.7 link-budget calculation IS the design-time eye-safety analysis; (b) a smartphone IR-camera qualitative check confirms the LED is emitting at expected brightness ("look bright but not saturating through the phone camera at 200 mm" passes); (c) a written 1 m minimum operator viewing rule enforced during bench + flight, eliminating the close-range exposure regime entirely. Document the deferral, the link-budget basis, the qualitative checks, and the operator rule in `eye-safety-measurements.md`. Phase 1 + US6 proceed without a calibrated meter. If 031-integration introduces spectator exposure or regulatory context, procure the meter then.
- Q: How should the recorder detect / signal a soft-CPU hang mid-flight so the operator doesn't end up with a stale-green LED while no chunks are landing? → A: **Tie the status LED's "recording" blink directly to the main-loop chunk-flush event** — no hardware watchdog needed. Specifically: on every successful SDIO `CMD25` completion (the "chunk durable" event), the soft-CPU emits a short GREEN pulse (~100 ms ON) on the status LED. With ~1-sec chunks at 240 fps (1 Hz pulse rate) and ~0.5-sec chunks at 480 fps (2 Hz pulse rate), the operator sees a **visible heartbeat blink in the 1–4 Hz range** during normal recording. If the main loop hangs, the pulses stop firing — LED freezes at whatever state it was last set to (probably solid GREEN or solid off, but importantly *no blink*). Operator's visual cue is "blink = alive, no blink = problem", which is the simplest possible liveness signal and requires zero new hardware. Combines cleanly with the existing fault patterns (YELLOW 10 Hz blink = transient fault, RED solid = unrecoverable, etc.) which override the heartbeat pulse during fault states.

## Overview

The 031 phase-1 deliverable is a **bench- and field-runnable beacon-camera setup that records raw video of two coded-IR beacons at flight-relevant ranges and dynamics**. Specifically:

- **A pair of LED beacon pods** sized for wingtip-top mount on an RC aircraft (~5 g each including a self-contained 1S LiPo battery), built from **5× 850 nm Lumileds Luxeon IR Compact LEDs wired in series as one Gold-code-modulated drive group at 300 mA constant current**, arranged on a 3D-printed **half-cube** of **~2.5 × 2.5 × ~1.3 cm**, mounted in **45° diamond orientation** (wedge frontal profile for reduced drag) with the apex face pointing OUTBOARD for **270° spherical coverage with the unlit ~90° cone pointed INBOARD toward the craft centerline** — i.e., visible from outboard, above, below, ahead, behind regardless of relative attitude. **Standalone-battery design** (path C, 2026-05-14): no wires to the airframe at all. Each pod runs from its own 1S LiPo (100 mAh, ~14 min runtime per charge) with a single externally-accessible on/off switch and 2-bit code-select jumpers. Drop on any wing-tip with tape, flip the switch, you're flying. (M2 effort is transitory — beacons are a stepping-stone to scene-recognition tracking in later phases, so simplicity beats integration depth here.)
- **A single mounted camera** with a wide-angle (~120° H FOV) M12 lens, **850 nm bandpass filter** (≤30 nm FWHM) integrated in the lens housing, and a global-shutter mono sensor at **320×240 @ 240 fps**.
- **A raw-frame recording capability with two modes**: (a) **bench mode** — USB-tethered host PC for early bring-up and bench scenario sweeps; (b) **flight mode** — onboard SD flash card recording at the **18 MB/s** sustained data rate (320 × 240 × 1 B × 240 fps), V30 minimum / V60 preferred class, auto-start on power-up so a flight-only operator does not touch the recorder mid-mission. Both modes produce the same file format + JSON sidecar.
- **Beacon Test Flight 1 (new in 2026-05-12 revision)**: a paired-craft field experiment. Target craft carries the beacon pods (powered up, broadcasting); tracker craft carries the camera + SD recorder (powered up, recording). Both craft are RC-flown by the operator. The deliverable is real-air raw footage — to be **analyzed on the ground** to confirm or revise code lengths/rates, FOV utilization, exposure settings, and EMI tolerance **before any FPGA gateware is committed**.

This phase is **deliberately not** producing a working real-time `(x, y, CEP)` front-end. That's 031-fpga's job. Phase 1 is the optical front + recording substrate + first paired-craft field session.

A **follow-on feature (likely 031-noise-cal or similar)** will consume the field-recorded clips to drive the simulator's noise-model calibration and the FPGA pipeline's Python golden-model test vectors. It is **not** in this spec, but the recording substrate and clip-file format here are explicitly designed to support it.

## Stakeholders

- **Operator (single-developer project)** — designs, builds, benches, iterates. Reads the handoff doc as source-of-architecture; this spec is the working memory of "what was decided and why" once we start.
- **030's evolved controller (downstream consumer)** — already trained against synthetic beacon projections. Treats 031's outputs (eventually) as the same `(x, y, CEP)` wire contract. Phase-1 work doesn't touch 030's controller; phase-2 (FPGA) and phase-3 (integration) do.
- **Future maintainer** — assumes nothing about the project except what's in the repo. Reads this spec to understand why specific LEDs/lens/sensor/filter parts were picked and what bench-result iterations changed those decisions.

## User Stories

Phase-1 has bench-developer-facing user stories only. No end-user-facing UI changes.

### US1 — Build and verify a single beacon pod (priority: P1)

**As** the bench operator,
**I want to** assemble a **5-LED half-cube standalone-battery pod** (per FR-1.1 + FR-1.2) with code-modulated LED-string output,
**so that** I can verify the optical emission pattern, mass budget, EMI signature, and code orthogonality before committing to a paired-beacon flight build.

Acceptance: a single pod emits the canonical **15-bit** Gold code at the canonical **100 Hz** chip rate, runs on its own **1S LiPo** per FR-1.2, weighs within the FR-1.4 mass budget (≤6 g), produces a visible-on-scope modulated NIR signal at >1 m distance through an **ND attenuator** (to bring the scope-photodiode in-range without saturating; confirms code is recoverable optically), and passes the FR-3.3 bench EMC sanity check (with the pod sitting ~10 cm from a representative FC running INAV, the FC's gyro noise floor and RC RSSI show **no degradation > 3 dB** vs the beacon-OFF baseline).

### US2 — Build the paired-beacon target rig (priority: P1)

**As** the bench operator,
**I want to** integrate two pods on a small target rig (workbench mock-up of wingtips at correct spacing, then onto a real RC airframe for outdoor bench),
**so that** I can collect realistic two-beacon footage for simulator + downstream tooling.

Acceptance: two pods running simultaneously with code A and code B respectively, mounted at hb1-wingtip spacing (~0.9 m), **each powered from its own 1S LiPo battery per FR-1.2** (no airframe connection), surviving moderate vibration. No firmware-level concerns yet — just photons in space.

### US3 — Build the receiver: camera + lens + filter, hand-held first (priority: P1)

**As** the bench operator,
**I want to** mount the chosen global-shutter sensor with the chosen ~120° NIR-bandpass M12 lens on a dev board capable of capturing raw frames,
**so that** I can collect real footage of the paired-beacon rig at varied range, angle, and ambient.

Acceptance: **320×240 raw mono frames at 240 fps** captured to disk for a configurable duration, with per-frame microsecond timestamps. Sun/ambient/range manually logged. Existence of two distinguishable IR blobs in the recorded frames at design ranges confirms the optical chain works end-to-end.

### US4 — Raw-frame recording substrate for downstream (priority: P1)

**As** the bench operator (and future 031-fpga / sim-tuning teams),
**I want to** record N-second raw-frame clips with provenance metadata (sensor settings, exposure, gain, timestamps, env conditions),
**so that** the simulator's noise model can be calibrated against real footage, and the FPGA Python golden model can be validated against the same footage offline.

Acceptance: a recorded clip is loadable by a Python utility that produces `numpy` arrays + a JSON sidecar with the per-clip metadata. The simulator (currently fully synthetic, [src/eval/camera_projection.cc](../../src/eval/camera_projection.cc)) gets a hook to ingest a real clip as a noise-source reference in a follow-on calibration pass (NOT this spec — 031-fpga or a sim-tuning sub-feature).

### US6 — Beacon Test Flight 1: paired-craft field recording (priority: P1)

**As** the operator,
**I want to** fly a target craft (with beacon pods broadcasting) and a tracker craft (with camera + onboard SD recorder) **simultaneously, both auto-recording from power-up**, with no live link, no NN engagement, no autonomous tracking,
**so that** I can collect real-air paired-beacon footage at flight-relevant ranges, aspects, and body rates — the substrate for confirming or revising the code length, chip rate, FOV, exposure, and EMI assumptions before any FPGA gateware is committed.

Operational model (revised 2026-05-14 for path C standalone-battery pods):

- **Target craft** (hb1 or equivalent): each beacon pod is **electrically self-contained** with its own 1S LiPo battery. **No wires to the airframe.** Tape the half-cube to each wing-tip in diamond orientation with double-sided tape (FR-1.1 / Q4). Operator **inserts a charged battery into each pod at flight prep** — pod begins emitting its Gold code within 100 ms, runs for ~14 min on one charge. Diagnostic visible-light LED on the pod blinks at chip rate to confirm "alive". To power-off, pull the battery. No telemetry, no airframe integration.
- **Tracker craft**: camera mounted nose-on or canopy-on with ~120° FOV pointed forward. SD recorder auto-starts at power-up, records continuously, auto-stops at power-down or SD-full. Operator does not interact with the recorder mid-flight.
- **Flight pattern**: tracker chases target at varying ranges (10 m → 100 m), aspects (head-on, side, trailing, banked), and body rates (gentle pan, hard pull). No automated guidance; pilot judgment only.
- **Post-flight**: pull SD card from tracker, ingest raw clip via FR-4 Python utility, qualitatively inspect for beacon visibility / blob shape / dropouts, log in bench-log per FR-5.2. Pull spent batteries from beacon pods, charge them externally on any standard 1S USB-LiPo charger (the battery leaves the pod for charging — no charge port on the pod itself).

Pre-flight safety gate (path C — simplified):
- FR-3.3 bench EMC sanity check PASS (no airframe-side coupling — mostly a formality);
- FR-3.4 eye-safety verification PASS — specifically: (a) **Phase 1 eye-safety substitutes per FR-1.7 #3** — link-budget design analysis on file + smartphone-IR qualitative check done + operator-only 1 m minimum viewing rule in effect (formal IEC 62471 calibrated measurement deferred), (b) battery insert/remove behavior per FR-1.7 #1-2, (c) firmware UVLO at 3.5 V real + topological failsafe + WDT bench-verified per FR-1.7 #4 (revised 2026-05-20 per R11);
- Two charged 1S batteries available (≥4.0 V each, per cell-checker reading) before flight.

Pre-takeoff per-flight checklist:
- Recorder status LED on tracker craft shows **GREEN heartbeat blink (~1–2 Hz pulse)** BEFORE throttle-up (= actively writing chunks to SD; per FR-2.6). A frozen LED with no observed blink = main-loop hung; do not throttle up.
- Each beacon pod's diagnostic LED blinks at chip rate, confirming code generation after battery insertion.

Acceptance (revised 2026-05-12 per clarification — infrastructure validation, not detection-performance):

US6 passes when **one** completed paired-craft flight session (≥5 min in air with both craft simultaneously airborne, both auto-recording successfully) produces a **standard-format raw recording that downstream DSP / FPGA-sim / visualization tools can ingest end-to-end**. Specifically:

1. **Clip is loadable** via the FR-4.3 Python utility — surviving chunks per FR-2.5 chunk-write contract are parsed without errors; per-frame timestamps are monotonic; JSON sidecar metadata is well-formed.
2. **Clip plays through at least one downstream DSP** — e.g., a hand-rolled correlator against the canonical Gold-code template, or a blob detector against per-frame imagery. The DSP output must be *sane* (not crash; produce per-frame outputs of the expected type) — even if zero beacons are detected, the *flow* (raw → DSP) is what's validated.
3. **Clip is loadable as input to a visualization tool** — e.g., per-frame imagery display, per-pixel time-series plot, range-over-time plot if range telemetry is available.

**Beacon count, tracking duration, and detection robustness are explicitly NOT US6 acceptance criteria.** Those characterize the operating envelope and belong to the follow-on noise-cal / DSP-tuning spec (Q10). The point of US6 is to make sure the chain works at all — emit, collect, record, retrieve, ingest, process, visualize — so the *follow-on can be planned*. A US6 clip with zero beacon detections is still a pass if it validates the rest of the chain (the failure mode is then "investigate the optics / pod power / code-select" — different debug paths than "investigate the DSP").

**Corollary**: the recording's information content must be **lossless raw** (no codec, no lossy compression, no quantization beyond the sensor's native 8-bit). The clip is the input to *every future DSP and sim* — its fitness for the next bench-tool needs no requalification because it's bit-identical to what the sensor produced.

Out of scope for US6 (deferred to 031-integration):
- Live telemetry link from tracker → ground
- NN-in-the-loop (the simulator's evolved chase controller running on the tracker)
- Any autonomy on the tracker; pilot flies the chase manually
- Multi-tracker / multi-target / mid-air handoff scenarios

### US5 — Side-by-side: range, ambient, body-rate scenarios (priority: P2)

**As** the bench operator,
**I want to** run a set of canonical bench-experiment scenarios (close range / far range, indoor / outdoor / direct sun, stationary / hand-induced rotation / fast pan to simulate body rate, dim ambient / bright ambient),
**so that** the operating envelope (acquisition range, sun-rejection performance, dynamics handling) is grounded in real measurements rather than the handoff doc's link-budget estimates.

Acceptance: a written bench-experiments log notes each session's setup + recorded-clip references + qualitative observations. Quantitative analysis (SNR floor estimation, etc.) is a follow-on, **not phase-1 acceptance**.

## Functional Requirements

### FR-1 LED beacon pod

#### FR-1.1 Optical emission pattern

Each pod SHALL include **5 LEDs arranged on the 5 outward-facing faces of a 3D-printed half-cube** ("half-cube" = a square base + 4 vertical side faces + 1 outboard apex face, like a cube with its inboard face removed and used as the mounting base). The half-cube SHALL be sized **2.5 cm × 2.5 cm square base × ~1.3 cm tall** (the box only protrudes ~1.3 cm from the wing-tip surface; the apex face is parallel to the wing-tip surface). Height of 1.3 cm accommodates the internal 1S LiPo battery + PCB stack-up per FR-1.2.1; the prior ~1 cm height was for the tethered variant without an internal battery. (Revised 2026-05-14: 4-LED pyramid → 5-LED half-cube; tethered → standalone-battery — see "Why 5-on-cube replaces 4-on-pyramid" below.)

The 5 LEDs SHALL be positioned so the combined emission pattern covers **270° spherically with the ~90° unlit cone pointed INBOARD toward the craft centerline** (behind the half-cube, into the battery + wing/fuselage stack):

1. **One LED on the outboard apex face** — beam axis along **+Y, the outboard direction**, pointing directly away from the wing.
2. **Four LEDs on the four side faces** of the half-cube, with the cube rotated **45° about the +Y outboard axis** (diamond orientation, see Mounting below). The resulting side-LED beam directions (in body frame, +X forward / +Y outboard / +Z up) are:
   - **Upper-forward** (≈ +45° pitch with forward bias) — operator shorthand: "45° pitch up"
   - **Lower-forward** (≈ −45° pitch with forward bias) — operator shorthand: "45° pitch down"
   - **Upper-aft** and **Lower-aft** — operator shorthand: "the other two facing back"

From any external aspect — outboard, above, below, ahead, behind — the beacon SHALL be visible regardless of the tracker's relative attitude; only the inboard fuselage-shadow direction is blind, and that direction is occluded by craft structure anyway.

**Why 5-on-cube replaces 4-on-pyramid** (decision 2026-05-14):

Polar-plot analysis ([led_configs_polar.png](led_configs_polar.png), generated by [plot_led_configs.py](plot_led_configs.py)) compared configurations using the Luxeon IR Compact 130° HPBW pattern:

| Config | Min in 270° envelope | Mean | Max | Uniformity (min/max) |
|---|---|---|---|---|
| 4-on-pyramid 45° tilt (prior spec) | 0.000 | 1.30 | 3.03 | 0.00 — has full nulls at the rim |
| 4-on-pyramid 60° tilt | 0.24 | 1.28 | 2.29 | 0.10 — 10× variation |
| **5-on-cube** | **0.79** | **1.49** | **1.93** | **0.41 — only 2.4× variation** |

The 5-on-cube has **no coverage nulls** anywhere in the 270° envelope. Trade-off: one more LED (25% more drive current + power + mass), lower peak intensity at the outboard apex (1.0 vs 3.0 relative — but apex peak gain only matters for direct-chase aspects, which 5-on-cube still illuminates strongly enough), and a slightly larger pod footprint. "Fairly even spread" is the load-bearing design goal.

**Mounting** (revised 2026-05-17 to capture wing-tip-flat geometry):

The target craft (hb1-class) has wing tips that present a **flat vertical outboard surface ~2.5 cm tall**. The pod assembly mounts to this surface as a **stacked battery-then-cube column** extending outboard:

1. **The 1S LiPo battery sits flat against the wing-tip outboard face**, installed by **slide-in from the aft direction** into the pod enclosure (FR-1.2.1). The battery's broad flat side lies parallel to (and in contact with) the wing-tip face; the battery's footprint (~4 × 22 mm projected onto the wing-tip face) fits well within the 2.5 cm wing-tip height.
2. **The half-cube enclosure sits directly outboard of the battery** — the cube's inboard face is the outboard wall of the battery cavity. The apex of the half-cube points **directly outboard (+Y)**.
3. **The half-cube is rotated 45° about the +Y outboard axis** (diamond orientation). The cube edges of the square base align fore-up / aft-up / fore-down / aft-down rather than fore-aft-up-down. This produces the LED beam directions listed above (apex outboard + 4 LEDs at 45° pitch-up-forward / 45° pitch-down-forward / upper-aft / lower-aft).
4. **Total outward protrusion**: battery thickness (~6 mm) + enclosure walls (~2 mm) + cube depth (~13 mm) ≈ **2-2.5 cm outboard** of the wing-tip surface.

Rationale:

- **Aerodynamics**: presenting a 45°-diamond edge (rather than a flat square face) into the airstream gives a wedge-shaped frontal profile and substantially lower pressure drag.
- **Wing-tip face is the load-bearing mount surface**: gluing/taping the pod-enclosure base against the flat outboard tip face (rather than the top surface) gives a stiff symmetric mount, no asymmetric torque, and natural alignment of the +Y apex axis with the wing's outboard normal.
- **Slide-in battery from aft**: allows battery swap between flights without removing the cube or disturbing the wing-tip mount. The aft direction is also the natural operator-access direction when the craft is parked.

**LED part** (FR-1.1 firm choice, 2026-05-14): **Lumileds Luxeon IR Compact 850 nm — [L1IZ-0850000000000](https://www.digikey.com.mx/en/products/detail/lumileds/l1iz-0850000000000/7243418)**. Per-die: 130° HPBW, 1.3 W radiant @ 1 A drive (T_j = 25 °C), 2.0 × 1.6 mm SMT package, ~30 mg/die. OSRAM SFH 4725S (150° HPBW, ~1 W) is the mechanical-similar fallback if Lumileds availability slips.

**Wavelength binning (firm 2026-05-14)** — load-bearing for end-to-end optical chain consistency:

- Lumileds Luxeon IR Compact parts ship in wavelength bins (typical bin width ±5 nm or ±10 nm). The base part number "0850" indicates **nominal 850 nm**, but real-world parts fall within a bin (e.g., 845-855 nm).
- **Required**: order the **850 ± 5 nm bin** specifically. The trailing digits of the full Lumileds order code encode the wavelength sub-bin; verify the bin code at order time with the distributor / Lumileds direct.
- **Common adjacent bins to avoid ordering by accident**: 830, 840, 860, 870, 880, 940 nm — none match the bandpass filter spec'd in FR-2.2 and would degrade or destroy the optical link.
- OSRAM SFH 4725S alternate has its own wavelength binning (typical ±10 nm at 850 nm) — same care required.
- The bandpass filter spec in FR-2.2 (CWL = 850 ± 5 nm, ≤30 nm FWHM) is matched to the 850 nm LED bin; **deviation in either direction breaks the design's link budget**.

**Mechanical integration — form factor is battery-driven** (path C, 2026-05-14): the pod enclosure is **designed around three primary form-factor elements**:

1. The **1S LiPo battery** (FR-1.2.1) — physical dimensions drive the inboard half of the enclosure.
2. The **half-cube LED housing** (5 LED-mounting indents, one per outward face) — drives the outboard half of the enclosure.
3. The **driver PCB** — sits between the battery cavity and the LED housing.

The 3D-printed shell SHALL provide:

- **5 LED-mounting indents** (one per outward face) with through-holes for the LED solder pads; LEDs are glued + soldered into the indents from outside the cube, with their leads routed back to the PCB inside.
- **A battery-shaped cavity** with retention features (clip, friction-fit ribs, or strap loop) so the battery can be **slide-in / pull-out** without tools.
- **An internal JST-PH 2.0 mm socket** on the PCB; the battery's standard JST-PH pigtail mates with this socket on insertion. The mate/demate of this connector **IS the power switch** — no separate slide-switch or push-button is needed (operator inserts the battery to power on, pulls it out to power off). See FR-1.2 for the electrical implications.
- **An external opening** matching the battery's footprint, on the inboard mounting face, so the battery can be inserted/removed without disassembling the cube. The opening doubles as the **charge port**: the battery is removed from the pod for charging in any standard 1S USB-LiPo charger, then re-inserted.
- **Code-select access** (FR-1.3): a small slot in the inboard wall exposes either solder jumpers (assembly-time) or a 2-position DIP switch (field-changeable) on the PCB.

The exact battery dimensions + PCB layout + retention mechanism are **plan-time dependencies** — see "Plan dependencies" at the end of this spec.

Even illumination across the 270° envelope SHALL be bench-validated per FR-3.1 (full-rotation azimuth + elevation sweep of recovered signal) before the design freezes.

#### FR-1.2 Driver topology — Standalone battery-powered (path C, 2026-05-14 simplification)

The spec is **firmly committed to a standalone-battery pod** (per operator direction 2026-05-14 — "this m2 effort... is transitory. Keep it simple."). No wires to the airframe. The tethered-to-main-pack and dual-cable variants from earlier drafts are dropped entirely.

Power + drive chain:

```
1S LiPo battery (3.0-4.2 V, 100 mAh, 20C — Tinywhoop-class pouch)
   │  battery's JST-PH 2.0 mm pigtail mates with internal socket on PCB:
   │  insertion = power-on, removal = power-off (no separate switch)
   ▼
internal JST-PH 2.0 mm socket → 1S rail (V_BAT)
   │
   │  *(revised 2026-05-20 per FR-1.7 #4 / R11: supervisor IC removed; UVLO is now
   │   firmware ADC + topological failsafe + WDT — see truth table below)*
   │
   ├── boost LED driver IC (TI LM3410X, SOT-23-5) ────────────────┐
   │      - V_BAT input (2.7-5.5 V), single-stage step-up         │
   │      - drives 5 LEDs IN SERIES (group Vf ~9.5 V at 300 mA)   │
   │      - constant-current output (sense resistor sets 300 mA)  │
   │      - DIM = SHUTDOWN control (no separate EN pin on LM3410):│
   │           HIGH → driver active + boost runs + 300 mA LEDs    │
   │           LOW  → full IC shutdown (~80 nA quiescent)         │
   │      - Soft-start = 20 µs (datasheet SU) — negligible vs 10 ms chip period
   │      - 2.8 A typ switch current limit, ~90% efficiency       │
   │                                                              ▼
   │                                          LED1 → LED2 → LED3 → LED4 → LED5
   │                                          (apex)  (side)  (side)  (side)  (side)
   │
   ├── ATtiny412 MCU runs directly off V_BAT (1.8-5.5 V tolerant — no LDO)
   │      - 1 µF + 100 nF ceramic decoupling at MCU VCC to filter boost-switching transients
   │      - ADC0 samples internal 1.1 V bandgap every 100 ms (V_BAT = 1.1 × 1024 / raw)
   │      - PA3 driven LOW + POWER_DOWN sleep on 5× consecutive < 3.6 V samples
   │      - WDT enabled (~250 ms timeout), petted in main loop
   │      │
   │      │ ─── 2-bit code-select read at boot (solder jumpers OR DIP switch) ────
   │      │ ─── Gold-code LUT bit at 100 Hz chip rate, GPIO push-pull active-HIGH:
   │      │       chip = 1 → drive PA3 HIGH (DIM HIGH, LM3410X running)
   │      │       chip = 0 → drive PA3 LOW  (DIM LOW, LM3410X shutdown)
   │      ▼
   │
   └─────────── DIM line (topological failsafe via pull-DOWN) ───────────┐
        MCU PA3 (push-pull active-HIGH, LSB of code LUT) ────────────────┤
        10 kΩ pull-DOWN to GND  ─────────────────────────────────────────┤
                                                                          ▼
                                                              LM3410X DIM pin
       
      Truth table on DIM (revised 2026-05-20 per FR-1.7 #4 / R11):
        MCU PA3 = HIGH (chip=1)                  → DIM=HIGH → LEDs ON
        MCU PA3 = LOW  (chip=0)                  → DIM=LOW  → LEDs OFF
        MCU PA3 = high-Z (reset / boot / hang)   → DIM=LOW  → LEDs OFF  (topological failsafe)
        MCU drives HIGH but firmware hung        → ≤250 ms WDT reset → PA3 high-Z → LEDs OFF
        V_BAT < 3.5 V real (firmware ADC trip)   → MCU drives PA3 LOW + sleeps → LEDs OFF
        V_BAT collapses (battery removed)        → all rails dead → LEDs OFF
       
      (PA3 is push-pull active-HIGH; R2 pull-down to GND guarantees the safe state
       in every MCU-offline condition. Schematic IS the failsafe — no supervisor
       chip required. WDT bounds the only failure-mode gap (hang while driving HIGH)
       to ≤250 ms ≈ ≤0.02 % of a 100 mAh pack worst-case.)
```

Key topology choices:

- **Single-stage boost LED driver IC** — **TI LM3410X** (primary, 1.6 MHz fsw — NOT LM3410Y which is 525 kHz) or **TI TPS61169** (alternate). Takes 1S LiPo directly, steps up to drive 5 LEDs **in series** at 300 mA constant current. Output voltage = 5 × LED Vf = ~9.5 V (well under the LM3410's 24 V V_SW absolute max). Single chip handles input regulation + boost + CC switching. DIM pin is the only shutdown/control input — no separate EN exists on LM3410.
- **5 LEDs in series, not parallel**: boost LED drivers natively drive series strings — simpler topology, no per-LED ballast needed, ~90% efficiency single-stage. Failure mode: one LED open → all 5 go dark on that pod. Acceptable trade; pod is a replaceable unit.
- **Battery-as-switch**: insertion of the 1S LiPo's JST-PH connector into the on-PCB socket powers the pod; removal disconnects all rails. No mechanical switch. Operator workflow: insert battery → pod runs the FR-1.3 Gold code for ~14 min until depletion; remove battery → pod off.
- **No dual-mode safety contract**: at 300 mA the eye-safety analysis (FR-1.7) shows IEC 62471 RG0 at typical bench + flight distances. Single-mode operation at 300 mA is sufficient.
- **No wires to airframe**: pod is electrically self-contained. No EMI envelope to the airframe at all. FR-3.3 bench EMC check is reduced to "verify no radiated emissions reach the FC at flight-typical mounting distance."
- **All 5 LEDs blink synchronously** as a single series-current load carrying one Gold code. No per-LED independent control.

The "drop on any craft" use case is the deployment model: tape the pod to a wing-tip, slide a charged battery in, you're flying. To stop: pull the battery. To re-fly: swap a fresh battery in.

#### FR-1.2.1 Driver BOM + dimensional fitment (revised 2026-05-14 for path C standalone-battery)

The driver + battery SHALL fit inside the **2.5 × 2.5 × ~1.3 cm half-cube enclosure plus an inboard battery cavity** (FR-1.1). Internal PCB area ~20 × 20 mm; component-stack height budget ~6 mm above the PCB. The **1S 100 mAh Tinywhoop-class battery (~4 × 22 × 6 mm typical)** sits flat against the wing-tip outboard face in its own enclosure pocket on the inboard side of the PCB, slide-in from the aft direction (FR-1.1 Mounting).

**Component BOM** (path C, standalone-battery, 2026-05-14):

| Block | Part | Package | Footprint | Height | Notes |
|---|---|---|---|---|---|
| Battery | **1S LiPo, ~100 mAh, 20C** (e.g., generic Tinywhoop pack [Amazon B083NWXLTK](https://www.amazon.com/dp/B083NWXLTK) or equivalent — 100 mAh size) | flat pouch | ~4 × 22 × 6 mm | 6 mm | **~2 g**; **~14 min runtime per charge** at 50% duty (FR-1.4); standard 1S JST-PH 2.0 mm charge connector exits through inboard base. 20C × 0.1 Ah = 2 A continuous capability — well above the 0.86 A peak draw |
| Boost LED driver IC | **TI LM3410X (LM3410XMF-NOPB)** primary, 1.6 MHz fsw / **TI TPS61169** alternate | SOT-23-5 | 3 × 3 mm | 1.1 mm | 2.7–5.5 V input, up to 24 V V_SW abs max (datasheet §6.3); 2.8 A typ switch current limit (datasheet §6.5); current-mode PWM (internal boost regulator switching — NOT a dim PWM); FB pin regulates LED current via external R_sense. **DIM pin is the ONLY shutdown/dimming control — LM3410 has no separate EN pin.** *(revised 2026-05-20 per FR-1.7 #4 / R11)* DIM is wired to MCU PA3 (push-pull active-HIGH) with a 10 kΩ pull-down (R2) to GND. MCU drives DIM HIGH for chip=1 (LEDs ON), LOW or high-Z for chip=0 (LEDs OFF). Any MCU-offline state → PA3 high-Z → R2 pulls DIM LOW → LM3410X enters ~80 nA shutdown. DIM-driven shutdown puts the entire IC in ~80 nA quiescent (datasheet §6.5 I_Q-shutdown). Soft-start = 20 µs (datasheet §6.5 SU) — 0.2 % of the 10 ms chip period, no waveform impact. **LM3410X** (1.6 MHz) variant enables the smaller 22 µH inductor (NOT LM3410Y which is 525 kHz per datasheet §6.5 fsw table). Built-in V_IN UVLO (~2.3 V per datasheet §6.5) is **insufficient** as primary LiPo protection — the firmware ADC + topology + WDT defense per FR-1.7 #4 holds the 3.5 V cutoff |
| LED current sense resistor | **0.62 Ω 1% 1206** (between LED string and FB pin) | 1206 | 3.2 × 1.6 mm | 0.6 mm | Sets LED current via the LM3410's 190 mV FB reference: I_LED = 190 mV / R_sense ≈ 300 mA. Dissipates ~57 mW peak / ~30 mW avg — easy 1206 |
| Driver inductor | **22 µH shielded SMD, I_sat ≥ 1.5 A** | 4×4 mm | 4 × 4 mm | 1.8 mm | Peak inductor current ≈ 1.2 A at 1S → 9.5 V boost at 300 mA, 1.6 MHz fsw; **verify saturation rating** on the specific part before ordering |
| Schottky diode | **1 A 30 V Schottky** (e.g., MBR130) | SOD-123 | 3.5 × 1.6 mm | 1.1 mm | Boost-mode rectifier diode; low V_F minimizes efficiency loss |
| Bulk cap (V_LED output) | **4.7 µF / 25 V low-ESR ceramic** | X7R 1206 | 3.2 × 1.6 mm | 1.5 mm | Output stabilization at the 9.5 V LED-string rail; **verify after DC-bias derating** that effective capacitance meets the LM3410X stability requirement (datasheet specifies minimum effective C_out) |
| Bulk cap (V_bat input) | **22 µF / 10 V ceramic** | X7R 1210 | 3.2 × 2.5 mm | 1.5 mm | Input bypass for the 1S rail; protects against current spikes during switching |
| MCU | **Microchip ATtiny412** | SOIC-8 | 5 × 4 mm | 1.5 mm | 8 pins: V+, GND, **code-out GPIO (push-pull active-HIGH to LM3410X DIM at 100 Hz chip rate; no PWM peripheral required)**, 2× code-select inputs, UPDI, reset, 1 spare. **MCU IS the UVLO cutoff path** *(revised 2026-05-20 per FR-1.7 #4 / R11)* — samples V_BAT via internal 1.1 V bandgap on ADC0, drives PA3 LOW at 3.6 V firmware trip, enters POWER_DOWN sleep. WDT (≤ 250 ms timeout) bounds firmware-hang exposure. Topological failsafe (DIM pull-down to GND) covers all other MCU-offline cases. **Runs directly off V_BAT** (1.8-5.5 V operating range covers the 3.0-4.2 V LiPo swing); no LDO needed. Internal 20 MHz RC oscillator — see NFR-4 clock-drift requirements |
| DIM-line pull-DOWN | **10 kΩ ±5 % 0603 to GND** *(revised 2026-05-20 per FR-1.7 #4 / R11 — was pull-up to V_BAT in original spec)* | 0603 | 1.6 × 0.8 mm | 0.5 mm | Holds DIM LOW (= LM3410X shutdown) when MCU PA3 is high-Z (reset, boot, brown-out, hang in input mode). MCU drives PA3 push-pull HIGH to enable LED string. **Schematic-level failsafe** — no firmware path between MCU-offline and LEDs-off; the topology guarantees it. R2 dissipation when PA3 driven HIGH: V_BAT² / 10 kΩ ≈ 1.4 mW @ 3.7 V nominal — negligible vs LED drive. 100 kΩ alternative for slightly lower active-state draw — only matters if every 100 µA counts for runtime |
| Battery socket | **JST-PH 2.0 mm 2-pin SMT or THT socket** (battery's pigtail mates here) | THT or SMT | 5 × 2 mm | 5-6 mm above PCB | Mate/demate is the pod's power switch (FR-1.2). Battery is removed for charging in any standard 1S USB-LiPo charger |
| Code-select jumpers | **2× solder jumpers** (PCB pads, no parts) OR **CUI DSM-02 SMT DIP switch** (2-position) | PCB / SMT | n/a / 4 × 5 mm | 0 / 2.5 mm | 2 bits → 4 Gold codes; MCU reads at boot. Jumpers cheaper/lighter (assembly-time); DIP switch operator-friendly in field |
| MCU decoupling | **1 µF + 100 nF ceramic** at MCU VCC pin | 0603 | 1.6 × 0.8 mm | 0.5 mm | Filters boost-switching transients on V_BAT (no LDO PSRR to fall back on); place close to the MCU |
| Boost-driver decoupling | 2.2 µF + 100 nF ceramic at boost driver VIN pin | 0603 | 1.6 × 0.8 mm | 0.5 mm | Local input bypass for the boost driver IC |
| Diagnostic LED (**mandatory** per US6 pre-flight check) | 0603 green visible-light LED | 0603 | 1.6 × 0.8 mm | 0.5 mm | Blinks at the Gold-code chip rate (100 Hz) for bench- and field-side verification that the pod is alive + emitting; uses ~3 mA. Visible through a small light-pipe slot on the pod enclosure's inboard face (operator-visible at pre-flight walk-around) |
| Battery retention (mechanical, not PCB) | Friction-fit ribs in 3D-printed cavity OR elastic strap loop OR magnetic latch | n/a | n/a | n/a | Retains battery during flight vibration; plan-time decision based on bench iteration |

**PCB layout sketch** (top-down view inside the half-cube — outboard apex is "up"):

```
       OUTBOARD APEX (LED1 mounts here, glued in from outside)
       ┌─────────────────────────────────────┐
       │   ┌─────────┐  ┌────┐                │
       │   │ LM3410  │  │L1  │  ┌──────┐      │
       │   │ boost   │  │22µH│  │schott│      │
       │   │ LED drv │  │    │  │ky D  │      │
       │   └─────────┘  └────┘  └──────┘      │
LED5 ──│                                       │── LED2
side   │   ┌────┐ ┌─────┐ ┌─────┐              │   side
       │   │MCU │ │super│ │4.7µF│              │
       │   │T412│ │visor│ │ out │              │
       │   └────┘ └─────┘ └─────┘              │
       │   (MCU runs direct off V_BAT)         │
       │                                       │
       │   [code-select jumpers ▪▪]            │
       │                                       │
LED4 ──│   ┌─────────┐                         │── LED3
side   │   │ JST-PH  │  ← battery pigtail       │   side
       │   │ socket  │    mates here            │
       │   │ (power) │                          │
       │   └─────────┘                          │
       │                                       │
       │  ┌─── battery cavity below ───┐        │
       │  │ 1S LiPo (slide-in / pull)  │        │
       │  │   ~4 × 22 × 6 mm pouch     │        │
       │  └───────────────────────────┘        │
       └───────────────────────────────────────┘
       INBOARD (mounting base, wing-tip surface)
       battery slides in/out through an opening on this face
```

**Component-stack height check** (path C with 100 mAh battery):
- PCB thickness: 0.8 mm
- Tallest SMT component: driver inductor 1.8 mm
- 1S 100 mAh battery (6 mm thick) below PCB, in its own enclosure pocket
- LED leads protrude back ~2 mm from each cube face
- Stack: PCB + components on top = 0.8 + 1.8 + 2 (LED clearance) = 4.6 mm above PCB
- Below PCB: 6 mm battery
- **Total interior height needed: ~11 mm — slight overrun on the original ~9 mm half-cube interior**; bump enclosure height to **~1.3 cm** (still well under the wing-tip-thickness budget at hb1 scale). The half-cube dimensions in FR-1.1 are updated to **2.5 × 2.5 × ~1.3 cm** to accommodate.

**Total per-pod mass estimate** (path C standalone-battery, 100 mAh, no-LDO, 2026-05-14):
- 5 × Luxeon LED dies: 5 × 30 mg = 150 mg
- PCB (20×20 mm × 0.8 mm FR4): ~250 mg
- All SMT components (boost driver + MCU + caps + jumpers): ~150 mg (saves ~20 mg from dropping the LDO, ~10 mg from dropping the supervisor IC + its decoupling cap per FR-1.7 #4 revision)
- **1S LiPo 100 mAh battery**: **~2 g**
- 3D-printed half-cube + battery compartment (PLA, ~30% infill): ~1.8 g
- Mounting tape/glue: ~0.3 g
- **Total: ~4.5–5 g per pod** (~10 g for the pair)

For comparison: the tethered variant from earlier drafts came in at ~3 g per pod; the standalone-battery design adds ~2 g per pod (mostly the battery) to eliminate all airframe wiring + EMI envelope. Net cost: ~4 g for the pair. Negligible on any RC airframe ≥500 g.

**Prototyping path** (before custom PCB spin):

The path C circuit fits on a 25 × 25 mm proto perfboard using off-the-shelf modules:
- **Boost LED driver module**: LM3410-based breakout, ~10 × 15 mm — Adafruit / Sparkfun stock a few options
- **ATtiny412 breakout** (~10 × 12 mm)
- **1S LiPo battery** with JST-PH 2.0 mm pigtail
- **Slide switch** (e.g., C&K JS-series) hand-soldered to interrupt the battery + tail
- **2× solder-jumper pads** on the proto board for code-select
- Wires soldered direct to the 5 LEDs in series (LEDs mounted in their 3D-printed indents from outside the cube)

This proto fits the same half-cube enclosure (allowing for the battery compartment) and lets the operator iterate on dimming linearity, EMI signature, and battery-life behavior **before** committing to a custom-PCB design.

#### FR-1.3 Gold-code generation (the modulation pattern the FPGA correlates against)

Each pod's MCU SHALL generate a **15-bit Gold code** modulating the LED group. (Revised 2026-05-12 from 7-bit to 15-bit per user direction and the camera-considerations analysis: at the 240 fps operating point we have enough margin to absorb the longer code, and N=15 gives 1-bit worst-case acquisition error tolerance vs N=7's 0-bit. See [camera_considerations.md §Coding & Acquisition Strategy](camera_considerations.md) for the full Gold-code length tradeoff.)

The code generation contract (revised 2026-05-14 — chip rate firmed to 100 Hz; per-LED current cut to 300 mA per FR-1.4):

- **Chip rate**: **100 Hz (firm).** 15-chip code period = 15 / 100 = **150 ms** = 1.5 FC samples (at the 10 Hz FC sample rate per the xiao NN controller) for cold-acquisition latency. Frames-per-chip oversampling at 240 fps camera = **2.4 fpc** (240 / 100 = 12:5 non-integer ratio — see "Aliasing avoidance" below).
- **Why 100 Hz** (revised from prior 48 Hz default — see [decisions-locked entry "Chip rate: 100 Hz"]):
  - **Cold-acquisition latency**: 150 ms = ~1.5 FC samples. The FC's NN controller transitions through `no_lock → tentative_lock → confirmed_lock` within 2 FC reads, which feels instantaneous to the operator. (Prior 48 Hz default gave 312 ms = 3.1 FC samples — slow for re-acquisition scenarios.)
  - **Oversampling**: 2.4 frames-per-chip is comfortable Nyquist margin and tolerates motion blur + frame drops. (Prior "≥5 fpc" rule was conservatively over-budgeted given the camera_considerations.md SNR analysis showing 70+ dB margin at 100 m daylight.)
  - **Aliasing avoidance**: 240:100 = 12:5 non-integer ratio means camera-frame edges never sit at fixed offsets relative to chip-transition edges. (Prior 5:1 integer ratio at 48 Hz had a theoretical persistent phase-alignment edge case.)
- **Acquisition tolerance**: with 2.4 fpc oversampling, each chip is sampled 2-3 times. Bit-error-rate per chip after per-chip integration is essentially zero at 100 m daylight (per FR-1.4 link-budget margin of 30+ dB even with 100× real-world derating).
- **Hard floor (Nyquist)**: chip rate ≤ camera_fps / 2 = 120 Hz. The 100 Hz default sits comfortably below this. Bench-iteration range: 50 Hz to 200 Hz, firmware-tunable.
- **Code length**: **15 chips** → one full code period = 15 × chip_period (= 150 ms at the 100 Hz default). Family of 17 N=15 Gold codes available, far more than the 2 the pair needs; cross-correlation bounded by 9/15 ≈ −4.4 dB peak vs −15 dB nominal (verify per pair selection offline). Acquisition error tolerance: 1 bit worst case at N=15 vs 0 bits at N=7 (camera_considerations.md table). This is the headline reason for the bump to 15.
- **Code source**: a small set (4) of pre-selected N=15 Gold codes baked into the MCU firmware as LUTs. The N=15 family contains 17 mutually-orthogonal members; pre-select the 4 with the lowest worst-case cross-correlation (verified offline at code-design time; pod just plays back the LUT).
- **Code ID assignment** (revised 2026-05-14 — path C): **2-bit code-select** read at MCU boot from a pair of GPIO pins tied to either (a) **2 solder jumpers on the PCB** (cheapest, lightest — operator solder-bridges at assembly time), OR (b) **a 2-position SMT DIP switch** (alternate — operator-changeable in the field without a soldering iron). 4 codes total; phase-1 uses codes 0 and 1 (one per wing); codes 2 and 3 are reserved for the future-multi-beacon use case. Drop the prior EEPROM-flash-time scheme — the jumper-read-at-boot approach is simpler + visibly inspectable + lets one MCU firmware image serve all pods.
- **31-bit / 480 fps upgrade path** (future, NOT phase 1): if the follow-on noise-cal or multi-beacon spec needs more codes (>4) or stronger error tolerance (5 bits vs the current 1 bit), the upgrade is **31-bit Gold codes at 200 Hz chip rate with a 480 fps camera**. This keeps cold-acquisition latency at ~155 ms (≈ current 15-bit @ 100 Hz) while gaining 31-code family + 5-bit error margin, at the cost of doubled SD bandwidth (37 MB/s @ 8-bit → V60 minimum) and a smaller camera-sensor market. The pod firmware would change the LUT length only; hardware unchanged. **Documented here as a known clean upgrade path** so future specs can reference it without re-deriving.
- **Duty cycle**: ~50% nominal (7-8 ON chips per 15-chip code; exact value falls out of the LFSR-derived Gold-code bit pattern). Per-LED current 300 mA peak when ON (FR-1.4).
- **Phase**: **free-running, self-clocked** from the beacon MCU's internal timer. No sync to camera, no sync between the two pods, no externally-driven clock signal. The matched filter on the FPGA side tolerates the natural offset.
- **Beacon MCU clock-drift tolerance**: see NFR-4 below. ±5% chip-rate drift (the worst case for the ATtiny412 internal RC oscillator across the industrial temperature range) translates to cumulative phase error over the 15-chip code period — the plan-time analysis SHALL determine whether this is workable with a single matched-filter hypothesis or whether multi-hypothesis correlation (or an external crystal on the beacon MCU) is needed.
- **MCU workload**: one timer interrupt at 100 Hz + 15-bit LUT bit lookup + single-GPIO write to the LED driver's DIM pin. **No PWM peripheral used — DIM is driven as a strict on/off digital signal carrying the current LUT bit; LED driver IC translates that to 0 mA or 300 mA on the LED string.** No dim control, no duty-cycle modulation, no analog filtering on the GPIO line. <5 mW MCU power. Code generation is not the load-bearing complexity here; clock-drift tolerance is (NFR-4).

#### FR-1.4 Mass + power budget (revised 2026-05-14 for path C standalone-battery + 300 mA per-LED + 5 LEDs series)

Each pod's mass SHALL not exceed **6 g** (with ~4.5-5 g per the FR-1.2.1 actual estimate, leaving ~1 g margin). The 1S LiPo battery is the dominant mass item (~2 g for a 100 mAh pack). For a research-grade RC airframe in the 1-2 kg class, two pods = ~10 g total, well under 1% of airframe mass.

Each pod is powered by **its own 1S LiPo battery** (100 mAh, 20C). No wires to the airframe.

**Single-mode operation (path C, 2026-05-14)** — no dual-mode, no fail-safe contract:

- **Per-LED**: 0.3 A × 1.9 V = **0.57 W instantaneous** when ON; 50% duty → **0.285 W average** per LED.
- **Per pod, 5 LEDs in series**: total LED-string power = 5 × 0.57 W = **2.85 W instantaneous**, **~1.43 W average** across the 50% duty cycle, plus ~15 mW MCU overhead (no LDO, no supervisor IC — per FR-1.7 #4 revision 2026-05-20). R2 pull-down dissipation when PA3 driven HIGH ≈ 1.4 mW @ 3.7 V — negligible.
- **From 1S LiPo**: boost LED driver steps 3.7 V (nominal) up to 9.5 V output at 300 mA constant current. Input power (at 90% boost efficiency) = output power / 0.9 = 1.43 / 0.9 = **~1.59 W avg from battery**.
- **Battery input current avg**: 1.59 W / 3.7 V = **~430 mA average draw from 1S cell**.
- **Battery runtime**: 100 mAh × ~90 % usable (4.2 V → 3.5 V UVLO cutoff per FR-1.7 revised) ÷ 430 mA ≈ **~12 min per charge** at continuous 50% duty operation. (~1 min less than the original 13 min estimate because the revised 3.5 V cutoff reserves ~10 % capacity for cell-storage safety vs the original 3.3 V cutoff's ~5 %.) Single-flight runtime; insert a fresh battery between flights. **Operator practice**: insert battery just before the airframe lifts off and pull it as soon as on the ground; prevents wasting cell capacity on idle ground time.
- **Battery peak draw**: at chip-ON instants, input current ≈ 9.5/3.7 × 0.3 / 0.9 = 0.86 A — well within the 20C × 0.1 Ah = 2 A continuous capability of the spec'd battery.
- **Thermal envelope inside the half-cube**: 5 LEDs × 0.285 W = 1.43 W average; LED dies dissipate to ambient through the outer cube faces. PCB-side losses ~30 mW + boost-driver losses ~150 mW + driver-inductor losses ~50 mW = **~230 mW PCB-side dissipation**, negligible. Battery self-discharge during operation is negligible at 20C continuous capability. **Bench-validate** thermal soak nonetheless during FR-3.x.
- **Link-budget headroom** (per the 100 m daylight analysis): single visible LED at 300 mA gives ~76 dB post-correlation SNR vs ~13 dB lock threshold = **~63 dB margin**. Even with 100× real-world derating (haze + dirty optics + off-axis + occlusion = −40 dB), retains **~23 dB margin** — comfortable.

The single-mode operation removes the prior dual-mode + fail-safe machinery. The pod is "off" (switch open) or "on at 300 mA" (switch closed); no operating modes between. Eye-safety at this single current level is RG0 at typical distances per FR-1.7.

#### FR-1.5 Orthogonality verification

The two pods together SHALL produce two orthogonal Gold codes whose cross-correlation, when recovered by demodulating a bench-camera clip with both pods illuminated simultaneously, is **≤ −15 dB** (handoff §5.5). Verification path:

- (a) **Scope-level**: capture one pod's LED-string current (via sense-resistor) waveform on a fast scope, confirm the chip rate (**100 Hz, ~10 ms chip period**) + 15-chip code period (150 ms) + correct Gold-code pattern;
- (b) **Photodiode-level**: pin a fast photodiode at 1 m, capture both pods' combined NIR signal, demodulate offline with both correlator templates, confirm A and B peak above each other's noise floor;
- (c) **Camera-level**: record a bench clip via FR-4 with both pods, demodulate the per-pixel intensity time-series at each blob, confirm both codes recoverable with the expected cross-correlation floor.

(a) is mandatory pre-shipment; (b) is a useful sanity check; (c) is what 031-fpga's Python golden model will reproduce against the same recorded clip.

#### FR-1.6 EMI mitigation (revised 2026-05-14 for path C standalone-battery)

The path C pod has **no wires to the airframe** — there is no conducted EMI path. The remaining EMI risk is **radiated emissions from the boost driver's switching node and the LED-string return loop**, coupling into either (a) the on-airframe flight controller / RC receiver at flight-typical distances (~30–60 cm wing-tip-to-FC), or (b) the other pod on the opposite wing (~1 m apart).

Mechanism: the TI LM3410**X** boost driver (FR-1.2.1) switches at **1.6 MHz**; the switch node carries ~1 A swings at sub-µs edges. Without mitigation, the switch-node loop radiates a broadband spectrum extending into the VHF range. The LED-string output current is comparatively benign — constant-current DC at 300 mA, modulated only at the 100 Hz Gold-code chip rate (~10 ms chip period) — easy to filter.

The design SHALL include the following mitigations as defaults, with bench EMC characterization (FR-3.3 below) validating the choices:

1. **Shielded inductor on the boost-driver switch node** (FR-1.2.1 BOM specifies a shielded 22 µH SMD inductor). Unshielded inductors radiate the full switching spectrum from the inductor body; shielded variants reduce this by ~10–20 dB.
2. **Local bulk capacitance at the boost driver input + output**: 22 µF + 100 nF on the V_BAT input rail, 4.7 µF + 100 nF on the V_LED output rail (per FR-1.2.1 BOM). Sources the switching pulse current locally without driving radiated currents back through the battery / LED-string loops.
3. **Compact PCB switch-node + LED-string layout**: place the boost-driver IC, inductor, and Schottky diode in a tight triangle on the PCB; route the 5-LED series-string output with minimal loop area (LED leads soldered direct to PCB pads, kept short). Twisted return-routing where wire lengths exist between PCB and LED indents in the cube faces.
4. **Optional metal shielding inside the half-cube body**: a thin copper-foil or conductive-paint layer on the inside walls of the 3D-printed half-cube, grounded to the pod's V_BAT negative rail, with cutouts only at the 5 LED apertures. Reduces near-field radiated coupling to the FC if bench-EMC shows it is needed.
5. **MCU decoupling**: 1 µF + 100 nF at the ATtiny412 VCC pin (per FR-1.2.1 BOM). Filters boost-switching transients on the shared V_BAT rail — important because the MCU runs directly off V_BAT with no LDO to provide PSRR isolation. *(revised 2026-05-20 per FR-1.7 #4 / R11: supervisor IC removed; no separate supervisor-decoupling cap needed.)*
6. **Chip-rate placement**: the **100 Hz Gold-code chip rate** is well below typical FC gyro low-pass filter cutoffs (≥250 Hz) and well above the audio range. Neither the chip rate itself nor its low-order harmonics should couple into the gyro signal path. The boost-converter 1.6 MHz fundamental + harmonics are the radiated-EMI concern; they are what mitigations 1–4 target.
7. **Pod-to-pod radiated coupling**: the two pods on opposite wings (~1 m apart) run identical boost converters at the same nominal 1.6 MHz fsw, but with **independent MCU clocks and independent battery rails** — no shared timing reference exists for the switching to lock into phase coherence. Cross-pod radiated coupling is therefore incoherent noise that averages out across the 15-chip Gold-code integration. FR-3.2 / FR-3.3 SHALL bench-validate that one pod's switching does not appear as a recoverable pattern in the other pod's recovered code.

The standalone-battery topology eliminates all conducted-EMI paths that the prior tethered draft had to mitigate (shared 3S LiPo, FET-gate slew control, π-input filter on the airframe tail). The remaining mitigations target only the radiated coupling that any boost converter unavoidably produces.

#### FR-1.7 Eye-safety analysis + manual on/off switch (revised 2026-05-14 for path C single-mode 300 mA)

The path C decision (single-mode 300 mA + standalone battery, no RC-PWM dual-mode contract) is justified by the eye-safety analysis below: at 300 mA per LED, the pod is **IEC 62471 Risk Group 0 (eye-safe) at typical bench + flight viewing distances**, eliminating the need for the complex dual-mode + PWM-fail-safe contracts from the earlier draft.

**Eye-safety analysis at 300 mA / 50% duty per LED** (Lumileds Luxeon IR Compact, 850 nm, 130° HPBW):

| Distance | Time-avg irradiance | Source radiance | IEC 62471 thresholds | Classification |
|---|---|---|---|---|
| 200 mm direct viewing | ~3 W/m² | ~5×10⁴ W/m²/sr | Thermal: <100 W/m² (RG0, 1000 s); Retinal extended-source: <1000 W/m²/sr at 8 mrad (RG0, 1000 s) | RG0 for ≤10 s; **RG1 for sustained ≥1 min direct staring** |
| 500 mm | ~0.5 W/m² | (same source radiance) | same retinal threshold scales with subtense; α ≈ 3 mrad at 500 mm → threshold ~2700 W/m²/sr (RG0) | RG0 for ≤100 s; borderline for extended viewing |
| 1 m direct viewing | ~0.12 W/m² | (same source radiance) | α ≈ 1.6 mrad — point-source regime; threshold ~6000 W/m²/sr (RG0 for any exposure) | **RG0 unconditionally** |
| ≥10 m (flight distance) | <10 mW/m² | trivially below any threshold | n/a | **RG0 unconditionally** |

**Practical safety contract**:
- Bench operators handling lit pods at ≥30 cm: unconditionally safe. No goggles required.
- Avoid sustained (>1 min) direct staring at the LED from <30 cm; momentary glances and routine handling are fine.
- At flight distances (≥1 m), pod is unconditionally RG0 for any spectator at any viewing time.
- For comparison: the prior 1 A drive design required ≥1 m for unconditional RG0; the 5× power cut to 300 mA provides ~10× exposure-time safety margin at any distance.

**Battery-as-switch** (path C — no mechanical switch):

- Insertion of the 1S LiPo's JST-PH pigtail into the on-PCB socket powers the pod. Removal disconnects all rails. Operator workflow: insert charged battery → pod boots in <100 ms → emits Gold code until depletion (~14 min) → operator removes spent battery → pod off.
- A **diagnostic visible-light LED** (FR-1.2.1 — mandatory per US6 pre-flight check) blinks at the Gold-code chip rate when the pod is emitting, giving the operator an at-a-glance "the beacon is alive" indicator without needing IR-camera visualization.

**Required pre-flight verifications**:

1. **Power-on behavior on battery insertion**: pod boots into emitting state within ≤100 ms of JST-PH mate; diagnostic LED begins blinking at the chip rate. Bench-verify on first build.
2. **Power-off on battery removal**: all LED emission stops within ≤50 ms of JST-PH demate. Bench-verify (no zombie emission from residual cap discharge — the boost driver's UVLO drops out cleanly).
3. **IEC 62471 classification — Phase 1 deferred** (revised 2026-05-17): formal calibrated-NIR-power-meter measurement at 200 mm is **deferred** to a future spec (031-integration or any milestone with spectator-exposure / regulatory context). Phase 1 relies on the **FR-1.7 link-budget design analysis** (the table above) as the eye-safety basis, supplemented by: (a) **qualitative smartphone-IR-camera check** at 200 mm — pod should appear bright but not pixel-saturated through a typical smartphone camera (most smartphones have IR sensitivity and act as a coarse photodetector); (b) **operator-only 1 m minimum viewing rule** during all bench + flight work — at ≥1 m the FR-1.7 table classifies the design as unconditionally RG0 for any exposure time. Document the deferral + the substitute checks + the operator rule in [`eye-safety-measurements.md`](eye-safety-measurements.md) before US6 flies.
4. **Battery low-voltage cutoff (LiPo protection — safety-critical)** *(revised 2026-05-20 per [research.md R11](research.md#r11--undervoltage-cutoff--led-driver-failsafe-1s-lipo-brown-out-protection))*: the pod SHALL stop driving the LEDs when the 1S cell drops to **3.5 V** (firm UVLO threshold, real V_BAT). This cutoff is implemented as **three defense-in-depth layers**, none requiring an external supervisor IC. The *intent* of MCU-independent UVLO — that no firmware failure mode strands the LED driver running — is preserved by the topological-failsafe layer (the schematic is the failsafe, not a chip). Implementation contract:

   a. **Topological failsafe** (Layer 1, always-on): the LM3410X DIM net is held LOW by a 10 kΩ resistor (R2) to **GND**. The MCU drives DIM HIGH (push-pull active-HIGH) to enable the LED string; any MCU-offline state (POR, BOD reset, WDT reset, software reset, brown-out, or firmware hang in GPIO input mode) leaves PA3 high-impedance, R2 pulls DIM LOW, and the LM3410X enters its ~80 nA shutdown state (datasheet I_Q-shutdown). **This is the primary fail-safe** — the schematic topology itself prevents stranded-LED-on conditions.
   b. **Firmware ADC cutoff** (Layer 2, normal-operation): the MCU samples V_BAT every 100 ms via the internal 1.1 V bandgap reference channel (ratiometric measurement using V_BAT as Vref: V_BAT = 1.1 V × 1024 / ADC_raw — **no external divider or GPIO required**). On 5 consecutive readings below **3.6 V threshold** (firmware-set; 100 mV margin above the 3.5 V spec to absorb Vref ±4 % drift), the MCU drives PA3 LOW and enters POWER_DOWN sleep (~100 nA hold). Wake-up only on battery removal + re-insertion.
   c. **Watchdog timer** (Layer 3, hang-protection): the MCU's internal WDT is enabled with a **≤ 250 ms timeout**, petted in the main loop. If firmware hangs while PA3 is driving HIGH (the only Layer 1 fail-safe gap), WDT resets the MCU within 250 ms; PA3 returns to high-Z; Layer 1 takes over. **Maximum "stranded LEDs on past true UVLO" window: 250 ms ≈ 0.02 mAh ≈ 0.02 % of a 100 mAh pack.**

   - **Threshold**: **3.5 V real V_BAT** (firmware trip set at 3.6 V to absorb Vref drift) with implicit hysteresis (once tripped, MCU stays in POWER_DOWN sleep — no re-engagement until battery removal cycles power).
   - **Reasoning for 3.5 V vs original 3.3 V** (revised 2026-05-20): 3.5 V at rest ≈ ~20 % SOC; under-load sag-corrected trip is ~3.4 V → ~10 % SOC. Preserves cell cycle-life better than the original 3.3 V threshold (which approached 5 % SOC under load) and gives the operator a "land now" warning window before complete cutoff. Runtime cost: ~1 min off the previous ~13 min estimate per 100 mAh pack.
   - **Reasoning for firmware + WDT vs original "hardware supervisor IC, MCU-independent"** (revised 2026-05-20): MCU is now the cutoff path, but the schematic is the failsafe. The only non-self-protecting failure case (MCU hangs while actively driving HIGH) is bounded by the WDT to ≤ 250 ms / ≤ 0.02 % cell impact per incident. Operator-accepted trade-off: simpler BOM (no supervisor IC, no decoupling cap) + tunable threshold (no part-suffix decoding hazard) at the cost of accepting per-hang ≤ 0.02 % cell wear.
   - **NOT a separate supervisor IC**: previous wording mandating MCP1316T / TPS3839 / APX803 is **withdrawn**. Those parts don't reliably exist at the required 3.5 V threshold in the right package + output-type combination (the TPS3839 family caps at 3.08 V then jumps to 4.38 V; the family is also push-pull, not open-drain as the original wired-AND topology required). Going firmware-side avoids the part-survey rabbit-hole entirely.
   - **Boost driver built-in V_IN UVLO is still insufficient as primary protection**: LM3410 trips at ~2.3 V (datasheet §6.5) — too low to prevent LiPo cell damage. The topology-plus-firmware-plus-WDT contract above is what holds the 3.5 V threshold; LM3410's own UVLO is a tertiary catch-all only.
   - **Bench-verify**: (a) on the eval rig (`cad/beacon-eval/`, after R100 cut + R2 reroute), ramp the bench supply down from 4.0 V to 3.0 V; scope the LM3410X DIM pin + V_LED rail to confirm cutoff at 3.5 V real V_BAT with the 500 ms debounce window; (b) physically issue a soft-reset to the MCU mid-emission, confirm DIM goes low within ≤ 1 ms (POR delay) + WDT period ≤ 250 ms total; (c) verify post-cutoff battery quiescent current is < 100 µA (MCU sleep + LM3410X shutdown + R2 leakage with PA3 high-Z).

### FR-2 Camera + lens + filter

**FR-2.1 Image sensor** (revised 2026-05-14)

The camera SHALL be a **global-shutter mono image sensor** meeting all of:

| Requirement | Threshold | Source |
|---|---|---|
| 850 nm QE | ≥40% (≥60% preferred) — and **reasonably flat ±20 nm around 850 nm** (i.e., QE ≥40% at 830 nm and ≥40% at 870 nm) so LED-bin variation per FR-1.1 doesn't shift the link budget | camera_considerations.md link budget + FR-1.1 LED binning |
| Shutter | Global only — rolling-shutter explicitly disqualified | handoff §3.2 (500°/s body rate → 17 px skew on rolling) |
| Resolution mode | 320×240 cropped (ROI windowing preferred over binning for daylight) | FR-2.3 + handoff §6.1 |
| Frame rate | ≥240 fps at 320×240 (firm baseline); **480 fps also required** per FR-2.3 dual-mode pipeline | FR-2.3 |
| Interface | **MIPI CSI-2** (1- or 2-lane D-PHY); raw RGB-Bayer or raw-mono output | FR-2.5 (FPGA recorder ingest) |
| AGC control | **Manual exposure + manual gain via I²C** (the FPGA recorder programs fixed settings per session; auto-AGC drift is explicitly disallowed because it perturbs the recorded photon-flux record) | FR-2.3 + camera_considerations.md §AGC dynamic behavior |
| Bit depth | 8-bit and 10-bit raw modes both selectable | FR-2.3 |

**Primary choice (revised 2026-05-14): OmniVision OG0VA** — purpose-built for our use case:

- 640×480 @ 240 fps native, **320×240 @ 480 fps native** (covers both FR-2.3 modes natively)
- 60% QE @ 850 nm / 40% QE @ 940 nm — best NIR sensitivity in this class
- Global shutter, monochrome variant available
- MIPI CSI-2 output (1 or 2 lanes)
- Manual exposure + gain via I²C; AGC defeatable
- 1/10-inch optical format (industry's smallest at release)
- Available as **OC0VA CameraCubeChip** wafer-level module: 2.69 × 3.04 × 3.04 mm, well under 1 g
- **Sourcing caveat**: OEM channel only — direct OmniVision contact, or via a design house. Lead time + MOQ are the load-bearing constraints; place the inquiry as soon as plan is approved.

**Backup choice (easy bring-up): OmniVision OV9281** via **Arducam B0162** module:

- 1280×800 @ 120 fps native; cropped to 640×480 @ 240 fps; cropped further to 320×240 windowed for higher rates (bench-verify exact achievable fps — datasheet check item per Plan Dependencies A)
- Same 40%/60% QE @ 940/850 nm as OG0VA
- Global shutter, monochrome
- MIPI CSI-2 output
- Manual exposure + gain via I²C
- Arducam B0162 module: $30-50, ships with M12 lens (**verify IR-cut filter is absent or removable** — see camera_considerations.md). 3-8 g typical module mass; OK for bench, heavy for flight.
- **Sourcing**: ready stock at Arducam / Mouser / DigiKey. **Use this as the bench-bring-up path while waiting on the OG0VA OEM channel.**

ST VD55G1 was previously listed as primary; **dropped 2026-05-14** in favor of OG0VA because the latter has stronger NIR-specific design, better sourcing path via OmniVision's drone/AR ecosystem, and explicit 480 fps support at 320×240 native (no need for sensor mode-table hacking).

**FR-2.2 Lens + bandpass filter** (revised 2026-05-14 — explicit prototype + production paths + wavelength-bin match to FR-1.1 LED)

| Requirement | Threshold |
|---|---|
| Mount | M12 (standard, swappable, no custom carrier required) |
| Field of view (horizontal, wide side) | ~120° (per user direction) |
| f/# | F/2.0 or faster (for photon collection on the 850 nm signal) |
| NIR correction | Lens elements corrected for 850 nm focus (no visible-light-only achromatic shift) |
| **850 nm bandpass CWL** | **850 ± 5 nm** — must match the FR-1.1 LED's 850 nm wavelength bin |
| **Bandpass FWHM** | ≤30 nm (10 nm preferred for better daylight rejection) |
| **In-band transmission** | ≥70% (typical for high-quality interference filters) |

**Wavelength matching across the optical chain (firm 2026-05-14)** — all three elements must agree on 850 nm:

| Element | Spec | Tolerance |
|---|---|---|
| LED (FR-1.1) | Lumileds L1IZ at 850 nm bin | ±5 nm |
| Filter (FR-2.2) | Bandpass CWL 850 nm | ±5 nm |
| Sensor (FR-2.1) | QE ≥40% across 830-870 nm | Both OG0VA and OV9281 are reasonably flat in this band; QE peak is near 850 nm (60% typical), falling to ~50% at 820 nm and ~55% at 880 nm — wavelength bin variation has minimal sensor impact |

Worst-case stack-up: an 850-bin LED at the low edge (845 nm) viewed through a filter at the high CWL edge (855 nm) would clip a fraction of the LED's emission. The 10-30 nm filter FWHM is wide enough to accommodate the binned LED emission spectrum even in worst-case mismatch, with ~50-70% transmission of the LED's photon output reaching the sensor. Wider filter (30 nm vs 10 nm) is more forgiving; narrower filter is better at daylight rejection. **Bench-validate** the actual LED-filter combination's transmission at the chosen bin before committing to phase-1 hardware.

**Production path (firm): Commonlands custom M12 lens with integrated 850 nm bandpass filter** (`contact@commonlands.com`). Spec: 120° H FOV, F/2.0, NIR-corrected; integrated bandpass filter with **CWL = 850 ± 5 nm** (matched to the FR-1.1 LED bin) and **FWHM ≤ 30 nm** (10 nm preferred for daylight rejection). Integrated filter eliminates separate alignment step + saves ~0.5 g + minimizes glass count. Lead time: ~4-6 weeks typical for custom config; place the inquiry as a plan-time D (hardware-order) deliverable.

**Prototype path (for bench bring-up before Commonlands lands)**: stack two off-the-shelf parts:

1. **Generic 145° fisheye M12 lens** — e.g., [m12lenses.com PT-02120](https://m12lenses.com/) (~$30, ships from CA in days). Verify no IR-cut filter is present (visual: IR-cut looks blue/cyan in reflection; functional: point an IR remote at the lens, see if the sensor sees the LED).
2. **Discrete 850 nm bandpass filter** placed between lens and sensor:
   - **Edmund Optics #65-679** (10 mm dia, 850 nm CWL, 10 nm FWHM, ~$90) — high-quality 10-nm-narrow filter
   - **Thorlabs FB850-10** (1" dia, 850 ± 2 nm CWL, 10 ± 2 nm FWHM, ~$70) — same class
   - Cheaper alternate: machine-vision-grade 850 nm filters from MidOpt (~$50-100) or eBay/Amazon (~$15 — verify FWHM with spectrometer if possible)

The two prototype options give bench-realistic spectral response within ~10% of the Commonlands integrated filter — clips recorded during early bench sessions remain calibration-valid for the production build (per FR-2.2 decision rule).

**Camera-module-bundled lens caveat (Arducam B0162)**: ships with a generic visible-light M12 lens that **often has an IR-cut filter installed for daylight cameras**. The IR-cut filter would reject our 850 nm signal entirely. **Phase-1 first receiver action**: inspect/measure the bundled lens; if IR-cut present, swap with the prototype-path lens above OR strip the filter with a small screwdriver (some Arducam lenses have removable filter rings).

**FR-2.3 Capture pipeline modes** (revised 2026-05-14 — dual-rate, dual-depth)

The capture pipeline SHALL support **two operating modes, switchable at session start**:

| Mode | Resolution | Frame rate | Bit depth | Data rate | SD class |
|---|---|---|---|---|---|
| **240 fps mode (baseline)** | 320 × 240 mono | 240 fps | 8-bit (baseline) or 10-bit-packed | 18.4 / 23.0 MB/s | V30 / V60 |
| **480 fps mode (high-rate)** | 320 × 240 mono | **480 fps** | 8-bit (baseline) or 10-bit-packed | **36.9 / 46.1 MB/s** | **V60 / V90** |

**Why both modes**:

- **240 fps + 15-bit Gold + 100 Hz chip rate** is the Gold-code-acquisition baseline (FR-1.3 link-budget design point). Adequate for routine US6-class flights.
- **480 fps** doubles the temporal resolution — useful for: (a) the future 31-bit-Gold-code path (FR-1.3 upgrade), (b) close-range motion-blur characterization during fast body rotations, (c) AGC-dynamics studies through sun→shadow transitions where 4 ms time resolution matters. Native on the OG0VA primary sensor; bench-verify achievable rate on OV9281 backup.

**AGC + exposure control** (firm 2026-05-14): the sensor's auto-AGC SHALL be **disabled**. Manual exposure + manual gain are set at session start via the FPGA recorder's I²C config to the sensor. Exposure values bench-tuned per ambient condition (per Q8 + Plan Dependency D). Capturing raw photon-flux data without AGC perturbation is the whole point of the recording — auto-AGC would smear the very dynamics we're trying to characterize offline.

**Bit-depth rationale**: 8-bit matches the FPGA pipeline's `(x_i8, y_i8, cep_u8)` wire format and is the cheapest SD-bandwidth path. 10-bit preserves 2 extra bits of dynamic range — critical for the **camera-dynamics-through-sun/dark-at-high-speed** study (replay raw 10-bit stream through simulated AGC curves offline to find the live-camera settings). Bit depth is recorded in the clip's chunk header (`bit_depth: 8 | 10`) so downstream loaders adapt automatically.

If the recorder host (FR-2.5) cannot sustain the chosen mode's data rate without dropping frames, that's a hardware-revisit, not a mode-compromise.

**FR-2.4** The captured frames SHALL be time-tagged with **monotonically increasing per-frame timestamps at microsecond resolution**. The **authoritative clock source is the FPGA recorder's own free-running monotonic counter** (driven from the board oscillator), derived to microseconds at frame-arrival time. The sensor's own frame-counter (if any) MAY be additionally captured as an auxiliary field but is not the authoritative time source. **Wrap-around**: the counter SHALL be at least 64 bits wide at µs resolution (no wrap for ~580,000 years — effectively wrap-free for any flight session). The **per-chunk header timestamp** (per `data-format.md`) SHALL equal the per-frame timestamp of the first frame in that chunk.

**FR-2.5 Onboard SD-flash recording for flight mode** (revised 2026-05-14)

The flight-mode capture host SHALL record raw frames + per-frame timestamps to an onboard SD-flash card at the FR-2.3 frame rate. Data-rate math (depends on FR-2.3 fps mode + bit-depth):

| Mode | Resolution × fps × bit-depth | Sustained rate | SD class floor |
|---|---|---|---|
| 240 fps, 8-bit | 320 × 240 × 1 B × 240 | **18.4 MB/s** | V30 (V60 preferred) |
| 240 fps, 10-bit packed | 320 × 240 × 1.25 B × 240 | **23.0 MB/s** | **V60** (V90 preferred) |
| **480 fps, 8-bit** | 320 × 240 × 1 B × 480 | **36.9 MB/s** | **V60** minimum |
| **480 fps, 10-bit packed** | 320 × 240 × 1.25 B × 480 | **46.1 MB/s** | **V90** required |

Plus per-frame timestamp + frame-counter overhead (~32 bytes/frame, negligible).

**Firm flight-recorder host: Lattice CrossLink-NX-EVN evaluation board.** Part number: **LIFCL-40-9BG400C** (LIFCL-40 FPGA in 400-ball BGA, on the CrossLink-NX Voyager evaluation board). Key board features:

| Feature | Value | Use |
|---|---|---|
| FPGA | LIFCL-40 (40K logic cells, 96 DSP blocks, 2.5 Mb BRAM, 168 Kb LRAM) | Hosts camera-ingest + SD-write logic for FR-2.5; later hosts the FR-031-fpga five-stage pipeline (background subtract → CCA → track table → correlator → output formatter) |
| MIPI D-PHY hard IP | 2× hard-IP D-PHY blocks, 1.5 Gbps/lane | Ingests OG0VA / OV9281 MIPI CSI-2 output at full bandwidth |
| Onboard SD slot | microSD with 4-bit SDIO routing | The flight-recorder SD slot; supports V60/V90 cards |
| USB-C | USB 2.0 (data), USB 3.0 (some board revs) | Offload path (a): host the SD card as USB Mass Storage when not recording |
| Power input | USB-C (5V) or onboard 3.3V LDO from external supply | Powered from the tracker-craft 3S LiPo via the regulator subassembly below |
| Mass | Eval board PCB + connectors + headers: ~50-80 g (carries headers + debug LEDs + lots of unused I/O; bare-board target on a follow-on production flight build, NOT phase 1) |

**Power tree (tracker craft, 3S LiPo)**:

```
Tracker 3S LiPo (9.0-12.6 V, 1300-2200 mAh typical)
   │
   ├── ESC + motor (existing airframe load)
   ├── BEC (5V) → RX, servos
   │
   └── Lattice CrossLink-NX-EVN board:
         Option A: small 5 V buck regulator → USB-C → eval board (clean, standard)
                   (e.g., Pololu D24V10F5, ~3 g, 9-36 V in, 5 V/1 A out)
         Option B: barrel-jack input on eval board if it accepts 7-15 V directly
                   (verify per board user-guide; some board revs have onboard regulator)
```

The recorder host is **fed from the tracker's 3S LiPo**, NOT from a separate battery (different from the beacon design where pods have their own integrated cell). Mass for the buck + cable: ~5 g; total recorder system mass (eval board + camera + lens + cabling + power): **~70-90 g** target.

**SD card** (firm 2026-05-14): for 240 fps default operating mode, **V30 minimum (V60 preferred)** UHS-I microSD, 64-128 GB. Recommended specific parts:
- **SanDisk Extreme Pro microSD V30 64 GB** (~$20, rated 90 MB/s sustained write — comfortable margin over 23 MB/s)
- **Kingston Canvas Go! Plus V30 64 GB** alternate
- For 480 fps mode: bump to **V60** (e.g., SanDisk Extreme Pro V60) or V90 if 10-bit-packed.

The flight-recorder firmware SHALL:

- Use **4-bit SDIO @ ≥25 MHz** on the SD interface (≥12.5 MB/s theoretical; SDIO drivers in Lattice's Propel reference designs hit ~40 MB/s sustained in practice). **1-bit SPI mode is insufficient** (max ~10 MB/s — below the 240 fps 10-bit requirement).
- Implement a **ring buffer** (≥250 ms / ~6 MB for 480 fps 10-bit) between MIPI ingest and SD write to absorb FAT flush latency. **Note**: the LIFCL-40's on-chip memory (2.5 Mb BRAM ≈ 320 KB + 168 Kb LRAM ≈ 21 KB) is insufficient for the full 6 MB ring buffer; the **EVN board's onboard SDRAM / HyperRAM (verify part + DMA path at plan time — Plan Dep A.8)** SHALL host the bulk ring storage, with BRAM/LRAM serving as the input-side double-buffer staging the MIPI burst into DRAM.
- **Pre-allocate the clip file at session start** (avoids fragmentation-induced slowdown), write directly to pre-allocated sectors — bypassing FS-allocation bookkeeping on the hot path while keeping the recovered clip mountable as a normal file.
- **Auto-start recording on power-up** (no operator action mid-flight); auto-stop on power-down or SD-full.
- Write the FR-4.2 chunked format: independently-parseable ~1-second chunks (240 or 480 frames depending on mode + per-frame timestamp headers + chunk header), plus JSON sidecar.
- **Mid-flight fault handling — reset and continue, not halt** (revised 2026-05-17). The recorder SHALL distinguish recoverable from unrecoverable faults and continue recording across recoverable ones:
  - **Recoverable faults** (transient SDIO write error, ring-buffer overrun, sensor I²C drop-out): write a **fault-sentinel chunk** (zero-frame chunk with a fault-code field, per `contracts/data-format.md`) to mark the discontinuity, reset the affected subsystem (SD re-init / overrun counter reset / sensor I²C re-init), and **continue recording into the same .clip file**. Status LED transitions to YELLOW blinking during the reset (~100 ms) then back to GREEN-solid. Frame loss is bounded to the duration of the reset.
  - **Unrecoverable faults** (SD-full, SD-removed, SDIO re-init repeatedly fails): write a final fault-sentinel chunk, finalize the file, set status LED RED-solid, stop. Operator pulls card.
  - **Brown-out** (supervisor signal from a board-level UVLO): nothing the recorder firmware can do; power is being cut. The last in-flight chunk is lost up to the last SD `CMD25` completion (worst case ~1 second). Surviving chunks remain loadable.
  - **No per-frame fsync** in any path — atomicity is at chunk granularity.

The reset-and-continue path means a typical flight session produces **one .clip file with zero-or-more fault-sentinel chunks embedded** at the discontinuity points. The FR-4.3 loader skips fault-sentinel chunks cleanly and surfaces them in metadata as `fault_events[]` for ground analysis.
- **Raw-LBA fallback** permitted if bench-benchmarking shows FS-mediated writes can't sustain the 480 fps 10-bit-packed rate (46 MB/s); use SD `CMD24`/`CMD25` direct sector writes paired with a host-side LBA-aware offload tool.

Total storage for a typical 10-minute flight session: ~11 GB @ 240 fps 8-bit / ~14 GB @ 240 fps 10-bit / ~22 GB @ 480 fps 8-bit / ~28 GB @ 480 fps 10-bit. A 64 GB or 128 GB card gives multi-flight margin.

**Offload paths** — both produce identical files:
- (a) USB-C download from the eval board (board exposes SD as USB Mass Storage when not actively recording).
- (b) Pull the SD card, read on a desktop PC via a standard card reader.

**FPGA→DSP reprogramming workflow**: per user direction 2026-05-14, the **same Lattice eval board is used for both the recorder (phase 1) and the DSP (phase 2)**. Phase 1 gateware = camera-config + bulk SD recording. Phase 2 gateware (031-fpga) = camera-config + the five-stage DSP pipeline → I²C output of `(x, y, CEP)` triples. The board doesn't change; only the bitstream changes. This is the key reuse the FPGA-eval-board choice was made to capture.

**Carrier-craft caveat**: the FPGA eval-board recorder is **over the weight budget for the eventual production tracker craft**. Phase-1 US6 flights use a **larger carrier craft** (hb1-class or trainer-class airframe with margin to lift the ~80 g recorder system) as the recording platform. Smaller / custom flight hardware (bare-die sensor + custom flex PCB + small FPGA in a single module) is a **031-integration / production-flight follow-on** — not phase 1.

**FR-2.6 SD-recorder status indicator (new 2026-05-12 per clarification)**

Flight-mode recording is fire-and-forget by design (FR-4.1b), so the recorder SHALL surface its operating state via a **visible blinking-LED indicator** the operator checks pre-takeoff. Early flight builds will typically run on dev-kit boards (RPi CM4 / Lattice eval / RP2040 + carrier) that already expose R/G/B GPIO-driven LEDs; this requirement is satisfied by repurposing those onboard LEDs rather than mandating a custom indicator board.

Pattern (following [xiao/src/util.cpp:15-32](../../xiao/src/util.cpp#L15) `heartBeatLED()` + [xiao/include/main.h:14-20](../../xiao/include/main.h#L14) RGB-pin convention used by the NN controller):

| State | LED behavior |
|---|---|
| Power-on / pre-init | RED solid (still bringing up sensor + filesystem) |
| Initializing (sensor enumerated, FS mounted, clip pre-allocating) | GREEN blinking, 250 ms period (matches xiao `BLINK_INTERVAL_MSEC`) |
| **Recording (writing to SD, no errors)** | **GREEN heartbeat blink — short pulse (~100 ms ON) emitted on every successful SDIO chunk-flush (CMD25 completion)**. Visible cadence: ~1 Hz at 240 fps mode (1-sec chunks), ~2 Hz at 480 fps mode (0.5-sec chunks). **Pulses stopping = main-loop hung** — operator's primary liveness signal. The "go for takeoff" signal is the *first observed pulse after init*, not solid GREEN. (Revised 2026-05-17 — see Clarifications.) |
| Mid-flight write stall / frame drop / ring-buffer near-overrun | YELLOW blinking, 100 ms period (rapid alert) — overrides the heartbeat blink during the fault window; returns to heartbeat blink after recovery |
| Mid-flight unrecoverable error (SD full / write fault) | RED solid |
| SD card missing or unreadable at boot | RED blinking, 250 ms period |
| Main-loop hung (soft-CPU fault, no watchdog reset) | LED frozen at last commanded state (typically solid GREEN or solid off mid-pulse) — visible as **absence of heartbeat blink**. No active indicator change; the operator notices the missing blink. |

The indicator SHALL be placed where the operator can see it during pre-flight walk-around without removing the camera mount cover or any wing tape (e.g., LED visible through a small light-pipe slot in the recorder housing, or mounted on an exposed face of the dev-kit board). Operator pre-flight checklist (FR-5.2 / US6 pre-flight safety gate) SHALL include "**confirm GREEN heartbeat blink (~1–2 Hz pulse) on recorder LED**" before throttle-up — a single observed pulse + ongoing cadence is the "alive + recording" signal. (Revised 2026-05-17 from the prior GREEN-solid expectation.)

The blink-code table SHALL be documented alongside this spec (`031-beacon-camera/recorder-status-codes.md` or equivalent quick-reference card) so a future operator unfamiliar with the build can interpret the LED at the flying field.

#### FR-2.7 Camera + recorder system BOM (new 2026-05-14, parallel to FR-1.2.1)

The flight-recorder system mounts to the tracker craft (hb1-class carrier per FR-2.5 caveat). All major parts:

| Block | Part | Sourcing | Mass | Notes |
|---|---|---|---|---|
| **Image sensor** | **OmniVision OG0VA** (primary) / **OmniVision OV9281** (backup) | OG0VA via OEM channel / design house — place inquiry early; OV9281 via **Arducam B0162** breakout module from Mouser/DigiKey/Arducam direct | Bare die: <1 g; Arducam B0162 module: 3-8 g | 320×240 mono, 240/480 fps, MIPI CSI-2, manual AGC, 850 nm QE 60% (OG0VA) / 60% (OV9281). See FR-2.1 |
| **Lens + filter** | **Commonlands custom M12** (primary, 120° H FOV, F/2.0, NIR-corrected, integrated 850 ± 10 nm bandpass) | `contact@commonlands.com` — 4-6 week lead | <1 g | Production path. See FR-2.2 |
| Lens (prototype) | **m12lenses.com PT-02120** 145° M12 fisheye | m12lenses.com, ~$30, days lead | ~3 g | Bench bring-up before Commonlands lands |
| Bandpass filter (prototype) | **Edmund Optics #65-679** (10 mm dia, 850 ± 5 nm, 10 nm FWHM) OR **Thorlabs FB850-10** (1" dia) | Edmund Optics / Thorlabs direct | <0.5 g | Sandwich between prototype lens + sensor; gives bench-realistic spectral response |
| **FPGA recorder board** | **Lattice CrossLink-NX-EVN** (Voyager evaluation board, **LIFCL-40-9BG400C**) | Lattice direct / Mouser / DigiKey | ~50-80 g | Camera-ingest + SD-record in phase 1; reprogrammed for FR-031-fpga DSP pipeline in phase 2 (same board, new bitstream). See FR-2.5 |
| **Power regulator** (tracker craft) | **Pololu D24V10F5** (5V, 1A buck) OR equivalent | Pololu direct / DigiKey | ~3 g | 3S LiPo (9-12.6 V) → 5V → eval board USB-C input. Verify eval board's barrel-jack alternative supports 9-12 V directly per board user-guide; if so, skip the buck |
| **SD card** | **SanDisk Extreme Pro microSD V30 64 GB** (240 fps modes) or **V60 64-128 GB** (480 fps modes) | Amazon / B&H / Adorama | <1 g | Per FR-2.5 data-rate table |
| **MIPI flex cable** | 22-pin or 30-pin MIPI CSI-2 flex (sensor module ↔ FPGA board MIPI connector) | Sized per sensor + FPGA board MIPI connectors; usually shipped with eval kit | <1 g | Plan-time detail: verify connector pinout matches between sensor module and Lattice MIPI hard-IP input |
| **Status LED (FR-2.6)** | Onboard R/G/B LEDs on the eval board (already present) | n/a — built-in | n/a | Repurpose existing eval board LEDs; firmware drives them per FR-2.6 blink-code table |
| **Camera + recorder enclosure** | 3D-printed mount, sized for the carrier craft (hb1 nose / canopy) | Custom STL | ~10-20 g | Holds camera module + lens facing forward, eval board mounted aft of camera with airflow access for thermal |
| **Camera-to-recorder cabling** | MIPI flex + I²C control + power leads | Sized to airframe | <3 g | All routed inside the recorder enclosure |

**Total recorder-system mass (phase 1)**: ~70-100 g for the eval board + Arducam B0162 + lens + filter + buck + SD card + enclosure + cabling. Carrier-craft target: ≥500 g airframe with ≥100 g payload capacity.

**Tracker airframe**: hb1-class or trainer-class fixed-wing with: ESC/motor/servos sized for ~1-1.5 kg AUW, 3S LiPo 1300-2200 mAh capacity, and a forward-facing nose / canopy mount for the recorder system. **Not** a small FPV quad — recorder mass + airframe handling considerations rule out micro-class platforms for phase 1.

### FR-3 Beacon-emission verification

**FR-3.1** Bench-side, the operator SHALL be able to verify that the **5-LED half-cube's** emission pattern matches the design's **270° spherical coverage with unlit cone inboard** (FR-1.1) by rotating a single pod through both azimuth AND elevation in front of the camera at fixed distance, recording received signal vs. the two angles. The blind cone (inboard face pointed toward the camera) SHALL be the only region with recovered signal below the lock threshold. (This is part of the US5 scenario set.)

**FR-3.2** Code orthogonality SHALL be bench-verifiable: with both pods on simultaneously at distinct codes, a recorded clip + Python-side demodulator SHALL show two recoverable signals with the expected cross-correlation floor.

**FR-3.3 Bench EMC sanity check (revised 2026-05-14 for path C — simplified)**

With the standalone-battery design (path C), there is **no conducted EMI path between the pod and the airframe**. Only radiated emissions are possible — substantially easier to characterize and contain. Before any flight session in US6 (Beacon Test Flight 1), the operator SHALL perform a **bench EMC sanity check**:

- Power one beacon pod from its 1S battery (no airframe connection). Mount it ~10 cm from a representative flight controller running the target firmware (INAV).
- Compare against beacon-OFF baseline (per-channel numeric thresholds — fail one, fail the gate):
  - **Gyro noise floor** (gyro spectrogram, blackbox log, or INAV CLI `sensor_info` / gyro RMS): broadband gyro RMS increase ≤ **3 dB** vs beacon-OFF baseline at 10 cm distance (the pod will sit on the wing-tip, ~30-60 cm from a typical FC mount in flight, so 10 cm bench is the conservative case).
  - **RC link RSSI / link quality**: RSSI drop ≤ **2 dB** and link-quality (LQ) drop ≤ **5 %** at the same RX/TX positions.
  - **ESC behavior / throttle response**: no observable throttle twitches and no audible PWM disturbance with the beacon switching across at least one full Gold-code period.

A FAIL SHALL block the test flight and trigger a redesign iteration on FR-1.6 mitigations (bulk-cap sizing, π-input filter, optional foil shielding of the cube interior). Document failures + remediation in the bench log per FR-5.2.

**FR-3.4 Eye-safety verification (revised 2026-05-14 for path C — simplified)**

Before any flight session in US6, the operator SHALL bench-verify (per FR-1.7):

1. **Power-on on battery insertion**: pod boots within ≤100 ms; diagnostic LED blinks at chip rate.
2. **Power-off on battery removal**: all emission stops within ≤50 ms.
3. **IEC 62471 classification — Phase 1 deferred** (revised 2026-05-17 per FR-1.7 #3): formal calibrated-meter measurement deferred. Perform the substitute checks: (a) smartphone-IR-camera qualitative confirmation that the pod is emitting at expected brightness at 200 mm (bright but not pixel-saturated); (b) document the operator-only 1 m minimum viewing rule applied during bench + US6. Record in [`eye-safety-measurements.md`](eye-safety-measurements.md) before US6 flies.
4. **LiPo UVLO at 3.5 V real + WDT-bounded failsafe** *(revised 2026-05-20 per FR-1.7 #4 / R11)* (safety-critical): three sub-checks, all SHALL pass to clear flight gate.

   a. *Firmware ADC cutoff (Layer 2)*: with the pod powered from a programmable supply substituting for the cell, slowly ramp from 4.0 V → 3.0 V at a controlled rate; scope the LM3410X DIM pin + the LED current. Acceptance: clean shutoff at 3.5 V ± 100 mV real V_BAT (allowing for Vref drift), debounce window ≈ 500 ms; no re-emission until power cycles. Confirm battery-side current drops to <100 µA after shutoff.

   b. *Topological failsafe (Layer 1)*: with the pod emitting at nominal V_BAT (~3.7 V), physically issue a soft-reset to the MCU (UPDI reset or programmed reset command). Acceptance: DIM goes LOW within ≤ 1 ms (POR delay); LM3410X enters shutdown; no re-emission until firmware re-engages.

   c. *Watchdog timer (Layer 3)*: load a deliberately-hung firmware variant that drives PA3 HIGH in a tight loop without petting WDT. Acceptance: WDT triggers reset within the configured timeout (≤ 250 ms); after reset, DIM goes LOW (Layer 1 takes over).

   Repeat (a) with the actual LiPo discharging through normal use until cutoff to validate end-to-end behavior.

A FAIL in #1, #2, or #4 (the emission-correctness + LiPo-safety checks) SHALL block the test flight. #3 is informational under the Phase-1 deferral (the operator-only 1 m viewing rule + the FR-1.7 link-budget unconditional-RG0-at-1 m result is the binding eye-safety contract; the smartphone check is corroborative). Failures in the substitute checks should be investigated but do not block US6 unless they indicate the LEDs are not emitting at the expected power level (which is a separate FR-1.5 / FR-3.1 emission-pattern problem, not an eye-safety problem).

### FR-4 Raw-frame recording substrate

(Revised 2026-05-12 — recording now supports two modes producing the same file format: bench-mode for indoor scenario sweeps; flight-mode for Beacon Test Flight 1 per US6. Flight-mode is the new addition; bench-mode is unchanged.)

**FR-4.1 — bench mode (USB-tethered via UVC eval-camera)** (revised 2026-05-17 multi-eval-board strategy). The bench recording system SHALL use an **off-the-shelf UVC-compliant USB camera module** with a NIR-capable global-shutter sensor (default: Arducam B0264 USB-UVC shield + Arducam B0162 OV9281 sensor module, or equivalent), feeding **live frames + per-frame metadata to the host PC over USB-UVC** at 240 fps 320×240 (V4L2 on Linux, AVFoundation on macOS). A **`tools/beacon-viewer/` Python utility** SHALL provide:
- Live-display of the streaming frames (per-pixel inspection, exposure/gain tuning, alignment + EMI debug);
- **Optional record-to-file**: consume UVC frames and write the canonical FR-4.2 `.clip` format + JSON sidecar, exercising the same loader contract as flight mode.

Bench-mode is **completely decoupled from the Lattice flight-mode gateware** — it works as soon as the USB camera is in hand, before any FPGA bring-up. Used for FR-3.1 / FR-3.2 / FR-5.1 indoor + outdoor bench sessions where a host PC is co-located. **Frame-rate caveat**: UVC over USB 2.0 high-speed gives sustainable ~30-60 fps live display at 320×240; recording to disk can hit higher rates limited by the host's USB stack. The 240 fps + 480 fps high-rate modes are flight-mode (FR-4.1b) features — bench mode trades temporal resolution for instant-on usability.

**FR-4.1b — flight mode (onboard SD via Lattice FPGA)** The flight recording system SHALL record raw frames per FR-2.5 to an onboard SD-flash card on the Lattice CrossLink-NX-EVN, auto-starting on power-up and continuing for the full duration the camera is powered (until SD-full or unrecoverable fault). Used for US6 (Beacon Test Flight 1) and any future field session where USB-tethering is not viable. The flight-mode firmware is the LIFCL-40 gateware path described in FR-2.5 + `contracts/fpga-recorder-contract.md`.

**Both modes** SHARE the FR-4.2 file format + JSON sidecar contract — clips recorded via bench-mode UVC and via flight-mode FPGA are bit-identical in format and ingest identically via FR-4.3.

**FR-4.2** Each recorded clip (bench or flight mode) SHALL produce two artifacts: (a) the binary frame stream as specified in the canonical [`data-format.md`](data-format.md) (chunked, versioned `format_version` field, raw 8-bit or 10-bit-packed pixels with per-frame + per-chunk timestamp headers), and (b) a JSON sidecar with per-clip metadata: clip-id, ISO-8601 start time, sensor model + firmware rev, lens + filter spec, capture resolution + fps + bit-depth, sensor exposure + gain settings, ambient-light qualifier (manually logged: indoor / outdoor / cloudy / direct-sun), range-to-target qualifier (manually logged or measured), pose qualifier (stationary / hand-panned / on-aircraft / **in-flight**), recording-mode qualifier (bench-tethered / flight-SD), notes free-text. **In-flight clips** SHALL additionally capture: target craft ID + beacon code IDs A/B + airframe configs + intended flight pattern + observed-from-ground notes.

The [`data-format.md`](data-format.md) doc is the **single source of truth** for the byte layout; it SHALL be authored before the first flight-mode build commits gateware, and the FR-4.3 Python loader SHALL be the executable reference implementation. Format version bumps require updating both `data-format.md` AND the loader in the same change.

**FR-4.3** A Python utility SHALL load a recorded clip (either mode) into a `numpy.ndarray` of shape `(n_frames, height, width)` with `dtype=uint8` plus a `dict` parsed from the JSON sidecar. This is the canonical handoff interface to all downstream tooling (sim noise calibration, FPGA Python golden-model test vectors, hand-rolled blob detector experiments, **the planned follow-on field-data analysis spec**).

### FR-5 Operating-envelope characterization (phase-1 bench observations)

**FR-5.1** The starting set of canonical bench scenarios (operator may amend before the first session, with amendments recorded in the bench log per FR-5.2):

- **S1 — close-range static** (1 m, indoor, dim ambient) — sanity check, expect strong signal on both beacons
- **S2 — mid-range static** (10 m, indoor, mixed ambient) — typical bench distance
- **S3 — far-range static** (50 m, outdoor, varies) — closer to the design 100 m
- **S4 — hand-panned slow** (10 m, hand-rotating camera at ~30°/s) — exercises track-table predictor in follow-on FPGA work
- **S5 — hand-panned fast** (10 m, hand-rotating at >100°/s) — approaches body-rate envelope
- **S6 — direct-sun adversarial** (10 m, outdoor, sun within camera FOV) — exercises filter sun-rejection
- **S7 — dim ambient adversarial** (10 m, dusk / late-evening) — exercises gain headroom
- **S8 — beacon obscured / partial** (10 m, briefly occluding one beacon by hand) — exercises sentinel detection in follow-on
- **S9 — paired-beacon close spatial proximity** (50 m, beacons appear ~1 px apart) — exercises CCA two-blob-merge case per handoff §6.11

Each scenario produces one 30–60 s recorded clip per FR-4. The full set is the phase-1 "ship" artifact — substrate for every follow-on optical experiment.

**FR-5.2** A written bench-log entry per session SHALL capture clip-id, scenario, observed qualitative behavior (beacon visible / blooming-on-sun / blob-mass-vs-range / motion-blur-onset-rate / etc.), and unresolved questions. The bench log is **the primary phase-1 deliverable**; it's the source of the iteration loop for the simulator's noise model and the FPGA pipeline's threshold tuning in follow-on specs.

## Non-Functional Requirements

### NFR-1 Mass + power (revised 2026-05-14 for path C standalone-battery + single-mode 300 mA)

- Per-pod mass: ≤6 g (actual ~4.5-5 g per FR-1.2.1 BOM estimate, including 1S 100 mAh battery ~2 g).
- Per-pod LED power: **~1.43 W average / ~2.85 W peak** at 5 LEDs × 300 mA, 50% duty.
- Per-pod battery draw: **~430 mA average from 1S cell** = **~14 min runtime per 100 mAh charge** (single flight session).
- Two pods: completely electrically isolated from the airframe — no shared power, no shared signal. Each pod is on its own battery.
- Camera + lens mass (sensor module + M12 lens + filter): **2–3 g target** for the bare optical front end (achievable with the OG0VA bare-die path; the Arducam OV9281 backup is heavier).
- Flight-mode recorder system (Lattice CrossLink-NX-EVN board + camera + lens + buck + cabling + enclosure): **~70–100 g**, per FR-2.7 BOM. **Carrier-craft path** — phase-1 explicitly accepts the eval-board weight cost on an hb1-class or trainer-class airframe (≥500 g AUW, ≥100 g payload margin). Production-flight-weight perception hardware (bare-die sensor + small FPGA on flex PCB) is a 031-integration follow-on, not phase 1 (Out-of-Scope section).
- EMI envelope: bench-EMC-validated per FR-3.3 (single-mode, simplified) before any flight; no measurable degradation of FC gyro, RC link, or ESC behavior with beacons powered.
- FPGA path (NOT phase 1): out of scope for this spec.

### NFR-2 Determinism + repeatability

- Recorded clips SHALL be reproducible: same setup + same scenario → same observed envelope. Where day-to-day ambient changes a bench result, the bench log SHALL note it.
- The simulator's downstream consumption of clips (in follow-on calibration work) requires bit-exact replay — the clip-file format SHALL preserve every raw pixel value with no lossy compression.

### NFR-3 Iteration-friendly

- All hardware decisions SHALL be **reversible** at this phase: no PCB spins until the optical chain is proven on dev modules. The Arducam OV9281 module + a CrossLink-NX-EVN / equivalent dev board is the canonical first build; bare-die mounting + flight PCB is a 031-integration concern.
- All component choices SHALL be **traceable** to the handoff doc decision (§3-§5) or to a bench-experiment delta that justified deviating.

### NFR-4 Beacon clock-drift tolerance (new 2026-05-14)

The beacon-channel design SHALL maintain reliable detection in the presence of **±5% absolute chip-rate drift** at the beacon MCU. This bounds the matched filter's tolerance to the worst-case ATtiny412 internal RC oscillator behavior:

- ATtiny412 factory-calibrated internal RC: **±2% accuracy at 25 °C**, **±10% across the -40 to +85 °C industrial temperature range**, **±10% across the 3.0-5.5 V supply range**.
- Bench-realistic worst case (room temperature + cell at 3.7-4.2 V, well within ATtiny412 VCC operating range): **±2-3%**.
- Flight-realistic worst case (wing-tip ambient ~-10 to +50 °C, supply-regulated): **±5%**.
- ±5% drift over the 15-chip 100 Hz code period: chip period of 10 ms drifts by ±0.5 ms; cumulative phase error across 15 chips = ±7.5 ms = ±1.8 camera frame periods.

The plan SHALL include a **simulation analysis or bench measurement** that:

1. Models the matched filter operating against a beacon with chip-rate offset from **-5% to +5% in 1% steps**, using realistic per-frame SNR (~50 dB at 100 m daylight per FR-1.4 link budget).
2. Computes **post-correlation SNR loss** at each offset for both single-hypothesis correlation and multi-hypothesis correlation (where the receiver tries N hypotheses spanning the drift range and picks the best peak).
3. Computes **cold-acquisition probability** at each offset — i.e., P(matched filter peak > lock threshold after 1 code period | drift = X%, SNR = Y dB).
4. Computes **tracking probability** (steady-state lock maintenance) for sustained operation at each drift level.
5. Identifies the implementation choice required to meet the **99% cold-acquisition target at ±5% drift, 30 dB derated SNR**:
   - (a) **External crystal** on the beacon MCU (±20-100 ppm — eliminates the drift problem at the source, but adds 1 part + 2 pins + ~$0.30); OR
   - (b) **Multi-hypothesis matched filter** on the receiver side (no beacon-side changes; receiver computes ~5-10 correlator hypotheses spanning the drift range, picks the best — adds FPGA gate count in 031-fpga but is fixed cost); OR
   - (c) **Internal RC + factory calibration only** (no crystal, no multi-hypothesis) if the simulation shows acquisition probability stays above 99% at the worst-case drift; OR
   - (d) **Both crystal + multi-hypothesis** (defense in depth).
6. Documents the choice in the spec and tasks the 031-fpga follow-on with implementing the multi-hypothesis correlator if (b) or (d) is selected.

The bench-validation path for NFR-4 (executed after the plan-time simulation):

- Build a beacon pod with deliberately-shifted MCU clock (firmware-spoofed chip rate) at -5%, -2%, 0, +2%, +5% offsets.
- Run each through the FR-3.2 code-orthogonality bench test (with both pods on simultaneously).
- Measure recovered-code SNR + lock probability at each offset.
- Compare to the simulation predictions; document any divergence in the bench log per FR-5.2.

NFR-4 is a **plan-time deliverable** — the simulation analysis SHALL be complete before the spec advances to /speckit.implement.

## Out of Scope (Phase 1)

The following are deliberately deferred. Each line names a follow-on 031 spec or BACKLOG entry where it lives.

- **FPGA pipeline (Stages 1-5)**: 031-fpga. IIR background subtract, single-pass CCA, track table, code correlator, output formatter. Phase-1's recorded clips are the test-vector source for the Python golden model that 031-fpga will validate against.
- **I²C wire protocol + register map (handoff §7)**: 031-protocol. Locks once 030's NN-input variants (per handoff §6.11) have been A/B'd in sim.
- **Real-time `(x, y, CEP)` host polling**: 031-integration. Phase 1 just records raw frames; centroid extraction lives in 031-fpga's Python model first.
- **NN-in-the-loop flight + autonomy**: 031-integration. US6 in this spec is a **manual-flight, dual-RC-pilot, both-sides-record-only** experiment with no live link and no tracker autonomy. NN engagement against real beacons remains 031-integration.
- **Sensor noise-model calibration of the simulator**: follow-on **031-noise-cal** (or similar; named pending /clarify). Consumes phase-1's flight-mode recorded clips. Likely extends 030's [src/eval/camera_projection.cc](../../src/eval/camera_projection.cc) with a real-clip-driven noise statistics model. **The Beacon Test Flight 1 footage from US6 is the precondition for this follow-on spec to even start.**
- **Multi-camera (3× cameras at 0°/+60°/-60°)**: handoff §3.5 + §9.1. Architecturally provisioned by the per-camera I²C-address scheme — but phase 1 builds one camera only.
- **Event-camera (Prophesee IMX636-class) alternative**: handoff §3.6 + §9.2. Future variant experiment, not phase-1 critical path.
- **Body-rate compensation feed-forward**: handoff §3.7 + §5.4. Optional; phase 1 records raw without body-rate metadata.
- **Sync (RF sync pulse from tracker to target)**: handoff §5.4. Asynchronous is the canonical design; revisit only for very long codes.

**In scope as of 2026-05-12 revision** (previously deferred to 031-integration):
- **On-aircraft beacon mount, in-flight LiPo power, EMI mitigation, and a single paired-craft test flight** (US6) on a **larger carrier craft** (hb1-class or trainer-class) — phase 1 explicitly accepts the over-weight Lattice eval-board recorder. The reason for promotion: bench-only recording can't characterize the body-rate envelope or the EMI envelope or the daylight/dusk envelope at flight ranges — and those are the assumptions the 030 simulator's noise model rests on. Get real-air data once, then iterate.

**Still out of scope** (clarification 2026-05-12):
- **Production-flight-weight perception hardware** for the eventual target tracker craft (bare-die sensor + custom flex PCB + small FPGA, or bare-module camera with minimal carrier). The Lattice eval-board recorder is the phase-1 + 031-fpga path; production flight hardware is a **031-integration or dedicated small-form-factor follow-on**. Phase-1's job is to surface what the production hardware needs to support, not to build it.

## Decisions Locked at Spec Draft (don't re-litigate without bench evidence)

Each item is traceable to a handoff-doc section. They become "open" only if a phase-1 bench experiment surfaces a contradiction.

| Decision | Source | Rationale |
|---|---|---|
| Wavelength: 850 nm everywhere | handoff §3.3 | Silicon QE ~2× vs 940 nm; visible-glow tolerable for RC test article |
| **Wavelength-bin matching across LED + filter + sensor: all elements at 850 ± 5 nm** | clarification 2026-05-14 | Lumileds LEDs ship in wavelength bins (per part-number suffix); order the 850 ± 5 nm bin specifically. Bandpass filter CWL must match (850 ± 5 nm). Sensor QE is reasonably flat across 830-870 nm so doesn't constrain further. Worst-case LED-edge × filter-edge mismatch still passes ≥50% transmission with the 10-30 nm filter FWHM — but verify bin code at LED order time, NOT after the parts arrive |
| Detection: temporal coding (NOT spectral) | handoff §3.1 | Off-the-shelf single-band filter + Gold-code orthogonality; single sensor; matched-filter processing gain handles two beacons |
| Shutter: global | handoff §3.2 | Rolling-shutter at 500°/s body rate produces ~17 px skew; rejected |
| Resolution: 320×240 (8-bit) | handoff §3.4 + §6.1 | Output is 8-bit signed (x, y); 320×240 ≥ 256-code precision with sub-pixel centroiding |
| Single camera in phase 1 | handoff §3.5 | Architect for multi-cam (I²C address scheme); build one |
| LED: Lumileds Luxeon IR Compact 850 nm | handoff §4.4 | 1.3 W radiant @ 1 A, 2.75 × 2.0 mm pkg, ~30 mg/die, pulsable to 5-10 A peak |
| **LED arrangement: 5 LEDs on a 3D-printed half-cube (2.5×2.5×~1.3 cm); 1 LED on outboard apex face + 4 on vertical side faces; unlit cone INBOARD** | polar-plot analysis 2026-05-14 ([plot_led_configs.py](plot_led_configs.py)) | 270° spherical with only 2.4× min/max variation (vs nulls in the 4-on-pyramid design); fairly even spread per user direction |
| **Pod mount orientation: half-cube base rotated 45° (diamond) on wing-tip** | user direction 2026-05-14 | Wedge frontal profile = roughly half the pressure drag of square-aligned face into airstream |
| **LED drive topology: standalone-battery (path C), TI LM3410**X** (1.6 MHz, SOT-23-5) boost LED driver drives 5 LEDs in series at 300 mA CC from 1S LiPo. ATtiny412 MCU drives the LM3410X **DIM pin** (the only shutdown control — LM3410 has no separate EN) as a strict-on/off digital line at the 100 Hz chip rate (no PWM dim control), via a push-pull active-HIGH GPIO. 10 kΩ R2 pull-down on DIM to GND provides the topological failsafe (MCU-offline = LEDs OFF). 2-bit code-select via solder jumpers or DIP switch. Battery insertion/removal is the power switch (no mechanical on/off).** | operator direction 2026-05-14 / refinement 2026-05-17 ("the part should be warm with strictly on/off from code — when battery is in, blink. That's it.") / **architecture corrected 2026-05-18 per LM3410 datasheet (T007a)**: LM3410**X** has no separate EN pin, so the "warm" intent is satisfied via the IC's 20 µs soft-start (0.2 % of a 10 ms chip — negligible distortion) rather than a held-EN architecture / **revised 2026-05-20 per FR-1.7 #4 / R11**: supervisor IC removed (no TPS3839 / MCP1316 part exists at the 3.5 V threshold + open-drain combination); UVLO is now firmware ADC at 3.6 V + topological pull-down failsafe + WDT ≤ 250 ms; DIM topology inverted from pull-up to pull-down so MCU-offline = LEDs OFF. | M2 effort is transitory; standalone design eliminates all airframe wiring + EMI envelope + dual-mode safety complexity. Battery-as-switch eliminates the on/off mechanism. Single-mode at 300 mA is IEC 62471 RG0 at typical viewing distances. No PWM dimming hardware required at single-mode 300 mA — MCU output is a single push-pull GPIO carrying the LUT bit at chip rate. On firmware-detected UVLO, MCU drives DIM LOW + sleeps → LM3410X enters ~80 nA shutdown. |
| **Per-LED current: 300 mA single-mode (down from 1 A dual-mode)** | link-budget + eye-safety revision 2026-05-14 | 100 m daylight detection retains ~23 dB margin even with 100× real-world derating; RG0 at ≥30 cm viewing distance with momentary exposure — no goggles needed for bench work, no dual-mode safety contract needed |
| **Power source: integrated 1S LiPo 100 mAh, 20C (per pod)** | operator direction 2026-05-14 | ~2 g battery, ~14 min runtime per charge — enough for a single flight session. No wires to airframe, no shared EMI with RC link, no servo-port hijacking. Battery dominates the pod mass (~2 g out of ~5 g total) |
| **Chip rate: 100 Hz** | clarification 2026-05-14 | 15-chip code period = 150 ms = ~1.5 FC samples cold acquisition. 240:100 = 12:5 non-integer ratio avoids the prior 5:1 aliasing edge case. 2.4 fpc oversampling preserves matched-filter margin |
| **Code-select: 2-bit solder jumpers or 2-position DIP switch on the PCB, MCU reads at boot, supports 4 distinct N=15 Gold codes** | clarification 2026-05-14 | Cheap + visibly inspectable + lets one MCU firmware image serve all pods. Phase 1 uses codes 0 and 1 (one per wing); 2 and 3 reserved for future multi-beacon experiments |
| **Beacon MCU clock-drift tolerance: ±5% per NFR-4 → internal RC only (no crystal, no multi-hypothesis)** | NFR-4 simulation 2026-05-18 (`tools/nfr4-clockdrift-sim/sim.py`, see `simulation-results.md`) | Single-hypothesis correlator at the FR-1.4 derated SNR (30 dB) achieves **100% worst-case cold-acquisition probability across −5% to +5% drift in 1% steps × 1000 MC trials**. Meets the 99% target by 1% margin even at worst-case drift. **Decision: (c) internal RC + factory calibration only** — no crystal part, no multi-hypothesis correlator in 031-fpga. Saves ~$0.30 in BOM + significant FPGA gates. Caveat: model is AWGN-only at 30 dB SNR; bench-validate per NFR-4 with deliberately-shifted MCU clocks before production-flight hardware. |
| **LiPo UVLO: 3.5 V real cutoff via firmware ADC + topological failsafe (DIM pull-down to GND) + WDT ≤ 250 ms** *(revised 2026-05-20 per FR-1.7 #4 / R11; supersedes the original "3.3 V hardware supervisor IC, MCU-independent" decision)* | safety clarification 2026-05-14 / architecture corrected 2026-05-18 per LM3410 datasheet (T007a) / **revised 2026-05-20 per R11**: TPS3839 / MCP1316 family survey found no 3.5 V open-drain variant; firmware ADC + pull-down topology + WDT defends-in-depth at zero BOM cost | LiPo over-discharge (below 3.0 V) damages the cell. 3.5 V is the new storage-safe threshold (~20 % SOC rest, ~10 % sag-corrected) — operator can leave spent battery overnight without immediate recharge. Built-in V_IN UVLO of common boost drivers (LM3410 at ~2.3 V per datasheet §6.5) is too low. The MCU-independent intent of the original spec is preserved by the topology: DIM is pulled DOWN by R2 to GND, so any MCU-offline state (reset/boot/hang in input mode) → LEDs OFF. Only failure mode that requires firmware (hang while driving PA3 HIGH) is bounded by WDT to ≤ 250 ms / ≤ 0.02 % cell impact per incident |
| **MCU runs directly off V_BAT (no LDO)** | simplification 2026-05-14 | ATtiny412 is 1.8-5.5 V tolerant — covers the 3.0-4.2 V LiPo swing without regulation. Drops the LDO part + its ~50 µA quiescent current; replaced by 1 µF + 100 nF MCU-decoupling caps to filter boost-switching transients on V_BAT. MCU on the 1S side (not 9.5 V boost output) avoids a bootstrap chicken-and-egg on startup |
| **31-bit Gold codes + 480 fps as future upgrade path** | clarification 2026-05-14 | Clean path forward for future multi-beacon (>4) or stronger error tolerance (5-bit vs 1-bit at N=15). Stay at 15-bit / 240 fps for this transitory M2 effort. Documented in FR-1.3 |
_(prior Decisions-Locked rows about "Pod power: shared with main 3S LiPo", "Pod connector: 3-wire RC servo", "Dual-mode operation" — all superseded by the 2026-05-14 path C standalone-battery design. See git history for the prior contracts.)_
| Lens: Commonlands M12 NIR-corrected ~120° F/2.0 w/integrated bandpass (CWL = 850 ± 5 nm, FWHM ≤ 30 nm, 10 nm preferred) | handoff §4.2 | Integrated filter saves separate alignment + mass; <8 µm focal shift visible→NIR; CWL matched to FR-1.1 LED 850 nm bin |
| **Sensor: OmniVision OG0VA (primary) / OV9281 via Arducam B0162 (backup)** | clarification 2026-05-14 | OG0VA is purpose-built for our use case (480 fps native at 320×240, 60% QE @ 850 nm, MIPI CSI-2, manual AGC). OV9281 is easier-to-source bench-bring-up path with same QE class. ST VD55G1 dropped — less NIR-specific ecosystem |
| **Camera operating modes: 240 fps (baseline, 18.4-23 MB/s) + 480 fps (high-rate, 36.9-46 MB/s); both at 320×240 mono; 8-bit or 10-bit-packed selectable** | clarification 2026-05-14 | 240 fps is the FR-1.3 Gold-code-acquisition baseline; 480 fps supports future 31-bit codes + close-range motion-blur + AGC-dynamics studies. Both natively on OG0VA |
| **Manual AGC: sensor auto-AGC explicitly DISABLED** | clarification 2026-05-14 | Auto-AGC would smear the photon-flux dynamics we're trying to record. FPGA configures fixed exposure + gain at session start via I²C |
| **Flight recorder: Lattice CrossLink-NX-EVN (LIFCL-40-9BG400C)** | clarification 2026-05-14 | Reused across phase 1 (camera-ingest + SD-record) and phase 2 (031-fpga DSP pipeline). Same board, different bitstream. MIPI D-PHY hard IP, onboard SD slot, USB-C for offload |
| **SD card: SanDisk Extreme Pro microSD V30 64GB (240 fps modes) / V60 (480 fps modes)** | clarification 2026-05-14 | Matches the FR-2.5 data-rate table; comfortable margin over 23-46 MB/s sustained |
| **Tracker-craft power source: 3S LiPo + 5V buck (Pololu D24V10F5) → eval board USB-C** | clarification 2026-05-14 | Existing airframe pack powers the recorder; clean isolation from camera/sensor analog rails via the eval board's onboard regulators |
| **Carrier craft for US6: hb1-class or trainer-class fixed-wing (~1-1.5 kg AUW)** | clarification 2026-05-14 | Phase-1 recorder system is ~70-100 g — too heavy for micro/FPV-class; needs ≥500 g airframe with ≥100 g payload margin |
| **Camera fps: 240 (firm)** | user direction 2026-05-12 + camera_considerations.md | Sets the FR-1.3 chip-rate derivation; matches camera_considerations link-budget analysis |
| **Code length: 15-bit Gold** | user direction 2026-05-12 + camera_considerations.md acquisition-error table | 1-bit worst-case acquisition error tolerance (vs 0-bit at N=7); 17-code family for future expansion |
| **Recording substrate: dual mode — bench (USB-tethered to Linux PC) + flight (onboard SD, U3/V30 min, V60 preferred, 18.5 MB/s sustained)** | user direction 2026-05-12 | US6 (Beacon Test Flight 1) requires onboard recording; bench mode unchanged from prior draft |
| **Flight-recorder status indicator: blinking-LED on recorder housing per xiao `heartBeatLED()` pattern; GREEN-solid = recording; checked pre-takeoff** | clarification 2026-05-12 | Fire-and-forget recording without a state indicator wastes flight sessions on silent failures; dev-kit boards already expose R/G/B GPIO LEDs so this is a near-zero-cost requirement |
| **Clip-file durability: pre-allocated file + chunked direct-sector writes (~1 s blocks); each chunk independently parseable; ~1 s trailing-frame loss accepted on power-loss/crash. Raw-LBA fallback permitted if FS bookkeeping bottlenecks 18.5 MB/s** | clarification 2026-05-12 | Standard dashcam-/black-box-recorder pattern; loses last second on hull crash (acceptable per operator); avoids per-frame fsync overhead |
| **US6 acceptance: infrastructure validation (emit → collect → record → ingest → DSP → visualize), NOT beacon-detection-quota** | clarification 2026-05-12 | US6 proves the chain works end-to-end; detection-quantity envelope is the follow-on noise-cal/DSP-tuning spec's job. The recording is the deliverable — fitness-for-downstream-tools is the acceptance test |
| **Recording file format: dedicated `data-format.md` + versioned chunk header (`format_version` uint16); FR-4.3 Python loader is reference implementation** | clarification 2026-05-12 | Single source of truth; loader-as-executable-spec discipline; prevents byte-layout drift across the FPGA recorder, the bench recorder, and downstream sims |
| **Bit-depth: 8-bit baseline OR 10-bit-packed (selectable per clip); chunk-header `bit_depth` field signals to loader** | clarification 2026-05-12 | 8-bit matches FPGA wire format; 10-bit preserves dynamic range for offline AGC-dynamics simulation through sun/shadow transits at body rate |
| **Flight-mode recorder host: Lattice CrossLink-NX-EVN FPGA eval-board + SD, NOT RPi CM4** | clarification 2026-05-12 | FPGA build doubles as 031-fpga's camera-ingest proof; the engineering is reused, not duplicated. RPi CM4 is fallback if FPGA gateware slips |
| **Clip offload: USB-from-board OR direct SD-card-read; both produce identical files** | clarification 2026-05-12 | Field-flexible (no laptop at flying field if pulling the card is enough); same file = same loader |
| Async LED-camera clocks | handoff §5.4 | Matched filter tolerates ±100 ppm offset; sync adds complexity for no first-order benefit |

## Open Questions

After the 2026-05-14 path C refactor, most original questions are resolved (recorded in the Clarifications section above). The remaining open items are operationally-useful but do NOT gate /speckit.plan.

### Resolved (see Clarifications section + Decisions Locked for details)

- Q1 (capture-host platform): **Bench = Linux PC + USB-tethered camera; Flight = Lattice CrossLink-NX-EVN FPGA eval board + onboard SD**.
- Q2 (recording format): **Versioned `data-format.md` + chunked direct-sector writes**.
- Q4 (pod mechanical mount): **5-LED half-cube, 2.5 × 2.5 × ~1.3 cm, 45° diamond orientation, battery-driven form factor** (FR-1.1 + FR-1.2.1).
- Q5 (acceptance bar): **Infrastructure validation — clip loadable + DSP pass-through + visualization pass** (US6 acceptance block).
- Q7 (chip rate): **100 Hz firm; 15-bit Gold; 240 fps camera**.
- Q9 (EMI tuning): **Path C eliminates the airframe-coupling EMI envelope; FR-3.3 becomes a one-step radiated-emissions check**.

### Q3 — Bench scenario list — RESOLVED 2026-05-17

Scenario set S1–S9 promoted into **FR-5.1** as the firm starting set. Operator may amend before first capture session, with any change recorded in the bench log per FR-5.2.

### Q6 — 031 branch creation prerequisites

This spec is being drafted on the 030 branch. The 031 branch and tasks.md begin once:
- (a) 030 v1 is sealed (SMOKE_REPORT.md + outcome.md + closeout commits pushed), AND
- (b) Hardware orders from the Plan Dependencies section (D) are placed, AND
- (c) The /clarify step on THIS spec is complete (this session, 2026-05-12 + 2026-05-14).

Default: gate 031 branch creation on those three prereqs.

### Q8 — Sensor fps + daylight exposure (bench-task, post hardware-arrival)

The camera + lens + filter selection (FR-2.1 / FR-2.2) is firm; the **exposure setpoint** at noon-sun ambient is a bench task per arrival of hardware. Outline:

1. **320×240 mode mechanism on the chosen sensor**: cropping (preferred, lossless per-pixel sensitivity) vs binning (helps low-light SNR but hurts daylight). Confirm from datasheet + bench.
2. **Max exposure without saturation under direct noon-sun in band**: bench-walk 0.1 → 5 ms in 0.5 ms steps, find largest exposure with no pixel saturated.
3. **Arducam B0162 lens IR-cut check**: visually inspect (cyan/blue tint = IR-cut present); functional test with an IR remote control; if IR-cut present, swap the lens or strip the filter before the optical chain works at all.

These are FR-5.1 bench scenarios + sensor-setup tasks. Results inform the final exposure choice + the noise-cal follow-on (Q10).

### Q10 — Follow-on field-data-driven feature scope

Once US6 (Beacon Test Flight 1) lands footage, a follow-on feature consumes it. Default scope (revised 2026-05-12 per operator):

**031-noise-cal + FOV/exposure + acquisition-time + AGC sims**: replays clips through correlator + AGC + FPGA-pipeline sims; produces a quantitative operating-envelope report; informs the next hardware-order decisions (lens FOV adjustment, beacon-power tuning, half-cube-dimension revision).

The phase-1 raw recordings serve as input to:
- **Acquisition-time simulations** — recorded clips through simulated correlators at various chip rates / code lengths / soft-decision thresholds
- **DSP / FPGA-pipeline simulations** — Python golden model validates the five-stage FPGA pipeline against recorded clips
- **AGC-dynamics simulation** — replay 10-bit clips through simulated AGC curves to characterize sun-to-shadow + sun-to-dark transit behavior
- **Noise-model calibration** of the 030 simulator's projection front-end
- **FOV / frame-rate / beacon-intensity validation** — does the design actually deliver what link-budget math predicted?

Risk acknowledgment (operator 2026-05-12): "this early phase is proving the beacon→camera chain — likely non-trivial." Plan for at least one iteration cycle: US6 data → sim insights → hardware/firmware revisions → second flight session.

## Iteration Contract

Phase 1 is **explicitly iterative**. The spec captures the starting decision set; bench experiments WILL change it. Every change SHALL be reflected here as either:
- **A bench-evidence update** to a Decision-Locked row (with the experiment reference + the new decision); OR
- **A new open question** when bench evidence contradicts an assumption but the new direction isn't yet clear; OR
- **A scope split-out** when bench evidence reveals an out-of-phase-1 subsystem (push to 031-fpga or 031-integration).

The /clarify pass is the formal mechanism for promoting open questions to decisions. Bench-log entries are the source-of-truth for evidence cited.

## Reference Architecture

For phase-1 review, the source-of-architecture is [docs/aircraft_tracker_handoff.md](../../docs/aircraft_tracker_handoff.md). All sections 1-5 + 8 are direct phase-1 reading. Sections 6 (FPGA pipeline), 7 (I²C register map), 9 (open questions / phase-2 work), 10 (first-week task list), 11 (reference links), and 12 (glossary) are read-on-demand — 6 and 7 are the source for 031-fpga + 031-protocol.

## Boundary with 030

030 owns:
- the chase NN architecture + training,
- the simulator's synthetic beacon-projection front-end ([src/eval/camera_projection.cc](../../src/eval/camera_projection.cc)),
- the `(x_i8, y_i8, cep_u8)` wire format and its sentinel encoding (also written into the 030 spec FR-007 + FR-017).

031 owns the optical pipeline that, in the limit, replaces the simulator's synthetic projection with real hardware. The 030 NN's training-time interface is the contract 031 must honor on the deploy side.

Phase 1 of 031 (this spec) does NOT touch any 030 code. The deliverable is a recording substrate + bench artifacts + **one paired-craft test flight clip (US6)**. The first place 031 will touch 030 is the **noise-model calibration follow-on (Q10, default scope 031-noise-cal+FOV/exposure)**, which extends the simulator's projection front-end to consume real recorded clips for sensor-noise statistics + FOV/exposure parameters. That follow-on is its own spec, not part of this one — and US6's flight footage is its precondition.

**Shared engineering with 031-fpga (clarification 2026-05-12)**: the flight-mode recorder is an FPGA-eval-board build (Q1(e) default) that does camera-config + bulk-record-to-SD. **This build is the camera→FPGA ingest stage of 031-fpga** — when 031-fpga proper starts, it inherits the working sensor I²C-config + the working camera-frame DMA path, and only needs to add the five pipeline stages (background subtract → CCA → track table → correlator → output formatter) on top. The phase-1 work is therefore **not throwaway**; it's the bottom of 031-fpga's stack.

---

## Plan dependencies (research items for /speckit.plan)

This spec is firm on **what** to build. The following items are concrete inputs the planner SHALL gather before producing a hardware order or a fab schedule. Each item is a focused data-gathering task, not an open design question.

### A. Exact part dimensions (drives the 3D-enclosure CAD)

The pod is built around the battery + cube + LEDs. The recorder system is built around the Lattice eval board + camera + lens. Exact dimensions to verify:

**Beacon pod**:

1. **Battery dimensions**: confirm the exact L × W × H of the chosen 1S 100 mAh 20C battery ([Amazon B083NWXLTK](https://www.amazon.com/dp/B083NWXLTK) or equivalent). Typical Tinywhoop-class packs are ~4 × 22 × 6 mm with a JST-PH 2.0 mm pigtail; verify by datasheet or physical measurement once the part is in hand.
2. **JST-PH socket footprint**: pick a specific JST-PH 2.0 mm 2-pin socket part (THT or SMT), pin pitch + body dimensions + retention-strength rating.
3. **Lumileds L1IZ-0850000000000 LED footprint + wavelength bin**: 2.0 × 1.6 mm SMT typical (Luxeon IR Compact datasheet). Confirm pad layout + thermal-pad requirements. **CRITICAL — wavelength binning**: order the **850 ± 5 nm bin specifically** (trailing digits of the full Lumileds order code encode the sub-bin). Verify the bin code at order time with the distributor / Lumileds direct; do NOT accept "any 0850 bin" — adjacent bins (830, 840, 860, 870 nm) will degrade the optical chain. Same care for the OSRAM SFH 4725S alternate.
4. **LM3410X boost driver + supporting passives**: LM3410 datasheet (TI SNVS541H) — exact pinout (5 pins on SOT-23-5: SW/GND/FB/DIM/VIN — no separate EN), recommended inductor + Schottky + caps with their SMT footprints. Verify the 22 µH inductor's I_sat rating ≥1.5 A on the specific part.
5. **ATtiny412 footprint**: SOIC-8, ~5 × 4 mm — confirm pad layout + recommended UPDI-programming-header placement.
6. ~~Voltage supervisor (MCP1316T-29LE/OT or equivalent)~~ *(removed 2026-05-20 per FR-1.7 #4 / R11: supervisor IC eliminated; UVLO via firmware ADC + topological failsafe.)*
7. **Code-select hardware**: pick between solder jumpers (PCB pads only) and a 2-position SMT DIP switch (CUI DSM-02 or equivalent — exact footprint).

**Recorder system**:

8. **Lattice CrossLink-NX-EVN (LIFCL-40-9BG400C) board dimensions, mounting holes, + onboard memory**: confirm from board user guide. Sizes the recorder enclosure. **Critical**: verify the EVN board's **onboard SDRAM / HyperRAM** (part number, capacity, interface, DMA path from the LIFCL-40 fabric) and confirm it is large + fast enough to host the FR-2.5 ring buffer (6 MB worst-case at 480 fps 10-bit + 46 MB/s sustained throughput). LIFCL-40 on-chip BRAM/LRAM alone (~340 KB) is insufficient.
9. **OmniVision OG0VA / OV9281 module dimensions**: bare-die OG0VA via OEM contact (CameraCubeChip module 2.69 × 3.04 × 3.04 mm), OR Arducam B0162 board dimensions; choose based on sourcing reality at plan time.
10. **Commonlands lens spec**: verify exact thread + housing dimensions, integrated-filter centering tolerance, focal length once specced. **CRITICAL — filter CWL match to LED bin**: confirm Commonlands' integrated bandpass filter CWL is **850 ± 5 nm**, matched to the FR-1.1 LED's 850 nm bin (FR-2.2 wavelength-matching table). If Commonlands offers binned CWL options, pick the one matching the LED bin within ±5 nm. For the prototype-path discrete filters (Edmund Optics #65-679, Thorlabs FB850-10), confirm CWL ±2 nm and FWHM 10 nm spec at order time.
11. **MIPI flex cable**: 22-pin or 30-pin connector verification between the camera module and the FPGA board's MIPI input.
12. **Pololu D24V10F5 buck dimensions**: 15 × 15 × 5 mm typical for placement under the eval board.

### B. Architecture-firming decisions

Once the dimensions in (A) are in hand, the plan SHALL produce:

1. **3D enclosure CAD** (STEP + STL): the 2.5 × 2.5 × ~1.3 cm half-cube with 5 LED indents, internal PCB shelf, battery cavity sized to (A.1), and the inboard-face opening for battery slide-in. Print orientation specified for FDM printer (PLA, ~30% infill, no support).
2. **PCB layout**: 20 × 20 mm 2-layer SMT layout matching the FR-1.2.1 BOM + the cube's interior PCB shelf geometry. Component placement constrained by the battery cavity (under PCB) and the LED indent positions (above PCB).
3. **Battery retention mechanism**: pick from (a) friction-fit ribs molded into the 3D-printed cavity, (b) elastic strap loop around the battery body, (c) magnetic latch. Plan-time decision based on bench iteration once first prints are in hand.
4. **Final Gold-code pair selection**: from the N=15 family (17 codes total), pick 4 codes with the lowest worst-case cross-correlation (per FR-1.3). Offline LFSR exploration; produce a 4-code LUT for the MCU firmware.

### C. NFR-4 clock-drift simulation (plan-time deliverable)

Per NFR-4, the plan SHALL include the simulation analysis that decides whether the beacon MCU needs an external crystal (~$0.30, removes drift problem) OR the FPGA recorder needs a multi-hypothesis matched filter. Inputs: ATtiny412 internal RC oscillator behavior across -40 to +85 °C, simulated correlator response. Output: a documented decision (crystal / multi-hypothesis / both / RC-only) before any PCB design freeze.

### D. Hardware-order shortlist (the plan's first artifact)

From (A) + (B), produce a single bill-of-materials with vendor + part-number + quantity-needed for two pod prototypes (so US6 can fly with one spare pod if needed), one camera + lens + filter set, one FPGA eval board + SD. Include the spare-pod overhead in the BOM-quantity column.

### E. Sub-deliverable docs (auto-generated alongside the BOM)

The plan SHALL note the following as in-scope artifacts to be authored when implementation starts (NOT this spec):

- [`data-format.md`](data-format.md) — versioned chunk-header schema (FR-4.2)
- [`eye-safety-measurements.md`](eye-safety-measurements.md) — IEC 62471 measurements per FR-3.4
- [`recorder-status-codes.md`](recorder-status-codes.md) — LED blink-code table per FR-2.6
- 3D-print STEP/STL files (per B.1)
- PCB Gerber files (per B.2)

---

## Status as of draft

- **Draft created**: 2026-05-10 alongside 030 v1 wrap
- **Revised**: 2026-05-12 — operator firmed beacon (4× 1W, 4-sided pyramid, ~2.5 cm base, unlit cone inboard), code (15-bit Gold), camera (240 fps), recording substrate (dual mode: bench-tethered + onboard SD); added US6 (Beacon Test Flight 1); added FR-1.6 EMI + FR-3.3 EMC bench check; added Q9 (EMI tuning order) + Q10 (follow-on noise-cal scope)
- **Revised**: 2026-05-14 — polar-plot-driven switch from 4-LED pyramid to **5-LED half-cube in 45° diamond mount** (Luxeon Compact L1IZ-0850000000000); chip rate firm at **100 Hz** (15-chip = 150 ms cold-acquisition); driver topology pass through three iterations (discrete buck + FET → Topology A integrated LED driver IC → **path C standalone-battery boost driver**); per-LED current cut **1 A → 300 mA** (link-budget retains 23 dB derated margin, IEC 62471 RG0 at typical distances); **dual-mode safety contract withdrawn** in favor of single-mode + manual on/off switch + IEC 62471 RG0 design; added NFR-4 (beacon clock-drift tolerance simulation, plan-time deliverable); documented 31-bit / 480 fps as future upgrade path
- **/clarify**: run **2026-05-12** (Session 1: eye-safety dual-mode, SD-recorder status LED, clip-file durability, US6 acceptance floor, data-format doc + bit-depth) and **2026-05-14** (Session 2: 5-on-cube vs pyramid, Topology A → path C driver-IC progression, chip-rate 100 Hz firm, beacon clock-drift NFR-4, path C standalone-battery, 15-bit / 240 fps vs 31-bit / 480 fps trade)
- **/analyze**: run 2026-05-17 (this session) — applied stale-text fixes from the post-path-C-refactor consistency pass
- **/plan**: not yet generated; gated on the three prereqs in Q6 (030 v1 seal + hardware orders + /clarify done)
- **Hardware orders**: not yet placed; pending operator decision after Plan Dependency D (hardware-order shortlist) lands
- **Bench experiments**: not started; gated on hardware arrival
- **Beacon Test Flight 1 (US6)**: not started; gated on bench experiments + EMC sanity check (FR-3.3)
