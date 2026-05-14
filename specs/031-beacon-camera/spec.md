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
- Note (operator 2026-05-12, post-Q5): **The phase-1 raw recordings are the input to acquisition-time / DSP / FPGA-pipeline simulations** for actual flight-DSP design — not just visualization or noise-cal. The Q10 follow-on will replay these clips through simulated correlators, AGC curves, and FPGA-pipeline Python models to characterize: acquisition latency under realistic photon flux, correlation SNR vs aspect angle, FOV-coverage utilization, beacon-intensity sufficiency at design range, pyramid-dimension trade-offs (e.g., is the 2.5 cm base too small to dissipate heat / spread the unlit cone correctly?), frame-rate adequacy under body-rate-induced motion blur. The "early phase is proving the beacon→camera chain" is **expected to be non-trivial** — the optical, mechanical, electrical, and timing variables interact in ways the link-budget math can't fully predict, and the recordings are how we surface that interaction.
- Note (operator 2026-05-12, post-Q5): **The Lattice eval-board flight-recorder is fine for early eval on a larger carrier craft** (e.g., the hb1 or a larger trainer-class airframe with margin to lift the eval-board weight). It is **over the weight budget for the eventual target tracker craft**. When the perception loop moves toward production hardware (likely 031-integration or a dedicated small-form-factor follow-on), we will pursue **smaller setups or custom builds** (bare camera + small FPGA on flex PCB, or the bare-die OG-series sensor + custom carrier from camera_considerations.md). Phase 1 explicitly accepts the eval-board weight cost as the price of getting first-flight data fast and reusing the FPGA work for 031-fpga.

## Overview

The 031 phase-1 deliverable is a **bench- and field-runnable beacon-camera setup that records raw video of two coded-IR beacons at flight-relevant ranges and dynamics**. Specifically:

- **A pair of LED beacon pods** sized for wingtip-top mount on an RC aircraft (≤6 g each, see FR-1.4 revision), built from **4× 1 W 850 nm LEDs wired together as one Gold-code-modulated drive group**, arranged on a 3D-printed **4-sided (square-base) pyramid** of **~2.5 cm base × ~2 cm height** mounted with its **apex pointing OUTBOARD** for **270° spherical coverage with the unlit ~90° cone pointed INBOARD toward the craft centerline** — i.e., visible from outboard, above, below, ahead, behind regardless of relative attitude; the only blind direction is the fuselage shadow, which is occluded by craft structure anyway. Each pod carries a small self-clocked timer MCU producing a **15-bit Gold code** modulated at a camera-derived chip rate (see FR-1.3 below). Powered from the **main 3S LiPo flight pack** via an in-pod buck regulator; EMI mitigation is a first-class design concern given the pulsed-current envelope (see FR-1.6 below).
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
**I want to** assemble a 4-LED, 4-sided pyramid pod (~2.5 cm base) on a small driver board with code-modulated output,
**so that** I can verify the optical emission pattern, mass budget, EMI signature, and code orthogonality before committing to a paired-beacon flight build.

Acceptance: a single pod emits the canonical **15-bit** Gold code at the canonical chip rate, draws within the design power budget from a representative **3S LiPo** bus, weighs within the design mass budget, produces a visible-on-scope modulated NIR signal at >1 m distance through a smoke-glass filter (to confirm code is recoverable visually), and passes a bench EMC sanity check (no measurable degradation of a co-located FC's gyro noise floor / RC link RSSI when the beacon is powered).

### US2 — Build the paired-beacon target rig (priority: P1)

**As** the bench operator,
**I want to** integrate two pods on a small target rig (workbench mock-up of wingtips at correct spacing, then onto a real RC airframe for outdoor bench),
**so that** I can collect realistic two-beacon footage for simulator + downstream tooling.

Acceptance: two pods running simultaneously with code A and code B respectively, mounted at hb1-wingtip spacing (~0.9 m), powered from a representative airframe DC bus, surviving moderate vibration. No firmware-level concerns yet — just photons in space.

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

Operational model:
- Target craft (hb1 or equivalent): beacon pods plug into the airframe via standard 3-wire RC servo connectors (V+ / GND / PWM), wired into spare RX channels (FR-1.2 / FR-1.7). On airframe power-up the pods boot to **low-power mode** (ground-safe radiance). Pilot's **arming switch on the TX** is wired to drive the beacon PWM channels to ≥1.5 ms — pods transition to **high-power mode** as part of the same arming gesture that arms the ESC. Disarming returns the pods to low-power mode automatically. No telemetry, no per-pod runtime config beyond arming-tied mode select — fire-and-forget within the safety envelope.
- Tracker craft: camera mounted nose-on or canopy-on with ~120° FOV pointed forward. SD recorder auto-starts at power-up, records continuously, auto-stops at power-down or SD-full. Operator does not interact with the recorder mid-flight.
- Flight pattern: tracker chases target at varying ranges (10 m → 100 m), aspects (head-on, side, trailing, banked), and body rates (gentle pan, hard pull). No automated guidance; pilot judgment only.
- Post-flight: pull SD card, ingest raw clip via the FR-4 Python utility, qualitatively inspect for beacon visibility / blob shape / dropouts / EMI artifacts, log in the bench-log per FR-5.2.

Pre-flight safety gate (new 2026-05-12 per clarification):
- FR-3.3 bench EMC sanity check (both modes) PASS;
- FR-3.4 dual-mode + eye-safety verification (all six FR-1.7 contracts + per-mode IEC 62471 classification) PASS;
- High-power-mode-arming TX-channel mapping documented + tested on the bench BEFORE first power-up at the flying field.

Pre-takeoff per-flight checklist (per FR-2.6 + the safety contracts above):
- Recorder status LED shows **GREEN-solid** (= actively writing to SD) BEFORE throttle-up. Any other state aborts the takeoff and triggers ground troubleshooting.
- Beacons-armed mode-switch on TX confirmed at low-power on initial power-up; transitions to high-power only after the airframe-arming gesture is complete.

Acceptance (revised 2026-05-12 per clarification — infrastructure validation, not detection-performance):

US6 passes when **one** completed paired-craft flight session (≥5 min in air with both craft simultaneously airborne, both auto-recording successfully) produces a **standard-format raw recording that downstream DSP / FPGA-sim / visualization tools can ingest end-to-end**. Specifically:

1. **Clip is loadable** via the FR-4.3 Python utility — surviving chunks per FR-2.5 chunk-write contract are parsed without errors; per-frame timestamps are monotonic; JSON sidecar metadata is well-formed.
2. **Clip plays through at least one downstream DSP** — e.g., a hand-rolled correlator against the canonical Gold-code template, or a blob detector against per-frame imagery. The DSP output must be *sane* (not crash; produce per-frame outputs of the expected type) — even if zero beacons are detected, the *flow* (raw → DSP) is what's validated.
3. **Clip is loadable as input to a visualization tool** — e.g., per-frame imagery display, per-pixel time-series plot, range-over-time plot if range telemetry is available.

**Beacon count, tracking duration, and detection robustness are explicitly NOT US6 acceptance criteria.** Those characterize the operating envelope and belong to the follow-on noise-cal / DSP-tuning spec (Q10). The point of US6 is to make sure the chain works at all — emit, collect, record, retrieve, ingest, process, visualize — so the *follow-on can be planned*. A US6 clip with zero beacon detections is still a pass if it validates the rest of the chain (the failure mode is then "investigate the optics / power / mode-select / arming flow" — different debug paths than "investigate the DSP").

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

Each pod SHALL include **4 LEDs arranged on the 4 sloped facets of a 3D-printed square-base pyramid**, sized **~2.5 cm base × ~2 cm height**, so the combined emission pattern covers **270° spherically with the ~90° unlit cone pointed INBOARD toward the craft centerline**. From any external aspect — outboard (beam side), above, below, ahead, behind — the beacon SHALL be visible regardless of the tracker's relative attitude; only the fuselage-shadow direction (inboard, looking back through the wing/fuselage at the pod) is blind, and that direction is geometrically blocked by the craft itself anyway. This orientation is the right one for a tracker that may come into view from any aspect, since attitude-independent visibility from the chase side is the whole point.

**Mounting**: the pod's pyramid base is **glued/taped to the wing-tip top surface with its apex pointing OUTBOARD** (away from the fuselage, in the +Y direction for a conventional wing). The 4 sloped facets thus point fore-up, aft-up, fore-down, aft-down — each tilted ~45° off the outboard axis, 90° apart in roll-around-outboard-axis. The unlit cone behind the pyramid base (pointing inboard toward fuselage) is the fuselage-shadow blind spot.

The specific LED part is **Lumileds Luxeon IR Compact 850 nm** (primary; OSRAM SFH 4715AS as drop-in mechanical+electrical alternate). Per-die nominal emission is 130° beam-width at ~1 W electrical drive (1.3 W radiant @ 1 A, T_j = 25 °C); **four dies at 90° spacing on the four ~45°-tilted facets** give the 270° spherical coverage with ~30° per-LED overlap at the inter-facet seams, plus on-apex (straight outboard) coverage where the four 130° cones all overlap. Even illumination is the design intent — facet tilt and LED-die collimation pattern SHALL be bench-validated per FR-3.1 (full-rotation azimuth + elevation sweep of recovered signal) before the design freezes.

The 3-facet (triangular) alternate from the prior draft is dropped: 4-facet gives more uniform spherical coverage at the cost of one extra LED + slightly higher pod mass + ~25% higher peak power. The user direction 2026-05-12 accepts that trade.

#### FR-1.2 Driver topology

Each pod SHALL implement the following power + drive chain (revised 2026-05-12 for the 4-LED, 3S-only, **wired-together single-drive-group** topology with **dual-mode PWM safety control** added per the clarification above):

```
main 3S LiPo flight pack (11.1 V nominal, 9.0-12.6 V range)
  │   ── pod connects to airframe via standard 3-wire RC servo connector ──
  │   ── pinout: V_bus (V+) / GND / PWM (signal) — wired off a spare RX channel ──
  │
  ├── V_bus → buck regulator → V_LED (~5-6 V) — headroom over 2S-series LED Vf ~3.4-4.0 V
  │     bulk cap on output + local ceramics at the FET drain
  └── 3.3 V LDO (off V_bus) → MCU
         │
         │ ─── PWM input (50 Hz, 1.0–2.0 ms pulse) ──→ MCU input-capture: low/high power select ─
         │ ─── digital code stream at camera-derived chip rate (FR-1.3) ────────────────────────→
         ▼
     slew-controlled logic-level N-FET (single, drives the 4-LED group) →
        4× LED in 2S2P (two parallel strings of two series LEDs) → sense resistor → GND
                                                                          │
        per-mode current limit set by firmware via per-chip-ON FET on-time ┘
        (low-power: ~0.1–0.2 A peak / ~5% of high-power radiance)
        (high-power: ~2 A peak / full radiance, FR-1.4 nominal)
```

Key changes vs prior draft:

- **The 4 LEDs are wired together as a single electrical load** carrying one Gold code (per user direction 2026-05-12). Series-parallel arrangement (2S2P) is the default: 2 strings × 2 series LEDs × 1 A/string = 2 A total at ~3.8 V across each string — leaves comfortable headroom on a 3S LiPo via the buck. All 4 LEDs blink synchronously; no per-facet independent control. This trades flexibility (we lose the ability to do per-facet PWM) for simplicity (one FET, one sense resistor) + reduced parts count + a deterministic emission pattern that doesn't depend on per-LED current matching at the firmware level.
- **Dual-mode operation via PWM input on a standard 3-wire RC servo connector** (per clarification 2026-05-12). The pod attaches to the airframe like a servo — V+ / GND / PWM signal — wired to a **spare RX channel** on the same airframe whose main 3S LiPo also powers it. The MCU input-captures the PWM pulse width on every 20 ms frame and selects:
  - **Low-power mode** (PWM < 1.5 ms or PWM absent/invalid): pod runs the same Gold code at the same chip rate, but with the LED peak current attenuated to **~5–10% of high-power** (firmware caps the per-chip-ON FET on-time to a fraction of a normal chip period, or PWM-modulates the FET gate within the chip-ON window). Radiant output drops by the same ratio; eye-safety classification per FR-1.6.5 targets Class 1 (eye-safe at zero distance) in this mode.
  - **High-power mode** (PWM ≥ 1.5 ms): pod runs at the FR-1.4 nominal power envelope (~2 A peak per group, ~2 W avg).
  - **Fail-safe**: any PWM pulse outside the 0.9–2.1 ms valid range, or absent for >250 ms (12 RX-frame timeout), defaults to low-power mode. This is the safety-critical contract — verified in bench testing per FR-1.7 below.
  - Mode-switch latency: <50 ms transition between modes (well within RX frame cadence). Mode transitions SHALL preserve Gold-code phase continuity (the matched filter doesn't see a code-restart, just an amplitude step).

Component picks (handoff §5.2, refreshed for the 4-LED group; all rebench-able):

| Block | Part | Notes |
|---|---|---|
| Buck regulator | TPS62933 (primary) / MP2451 (alternate) | 3 A, ~95% η, ~0.3 g; sized for ≥2 A continuous at 50% duty (~4 A peak) |
| Bulk cap | 47-100 µF low-ESR ceramic or polymer at buck output | Reduces ripple at the chip-rate switching edges; **EMI-critical** (FR-1.6) |
| MCU | ATtiny1616 (primary) / RP2040 (alternate) | ATtiny is mass-friendly; RP2040 simpler to flash; needs input-capture / PWM-decode peripheral for FR-1.2 PWM mode-select input |
| Pod connector | Standard 3-wire RC servo connector (V+ / GND / PWM signal), JR or Futaba pinout to match operator's RX | Matches airframe servo harness; pod treats it electrically the same as a servo slot |
| FET | DMN2058U logic-level N-MOSFET, R_DS(on) <50 mΩ | **Single FET driving the whole 4-LED group**; gate resistor sized for controlled slew rate (FR-1.6) |
| Sense resistor | 0.05 Ω 1% 1206 (revised down from 0.1 Ω) | 100 mV @ 2 A total group drive, ADC-friendly |
| LED | Lumileds Luxeon IR Compact 850 nm × 4 | See FR-1.1; 2S2P interconnect on the pyramid PCB |
| Power-input filter | π-LC (small ferrite bead + 10 µF + 100 nF) on the 11.1 V tail | EMI defense against pulsed current radiating back up the LiPo bus to the rest of the airframe (FR-1.6) |

Driver SHALL be **integrated FET drive from MCU GPIO with a gate resistor for slew-rate control** — no separate gate driver IC at the ≤4 A group-drive level. Saves mass + parts (handoff §5.2). The gate resistor (typ. 100-470 Ω) is the **first line of EMI defense** (see FR-1.6).

#### FR-1.3 Gold-code generation (the modulation pattern the FPGA correlates against)

Each pod's MCU SHALL generate a **15-bit Gold code** modulating the LED group. (Revised 2026-05-12 from 7-bit to 15-bit per user direction and the camera-considerations analysis: at the 240 fps operating point we have enough margin to absorb the longer code, and N=15 gives 1-bit worst-case acquisition error tolerance vs N=7's 0-bit. See [camera_considerations.md §Coding & Acquisition Strategy](camera_considerations.md) for the full Gold-code length tradeoff.)

The code generation contract:

- **Chip rate**: **DERIVED from the selected camera frame rate (240 fps)**, NOT independently locked. The design relationship is "camera gets ≥5 frames per chip" (handoff §5.3) to give the matched filter comfortable oversampling and natural robustness to ±100 ppm beacon-MCU↔camera clock drift over a code period (handoff §5.4). Concretely at the 240 fps baseline:
  - **Default: chip rate = 48 Hz (5 frames/chip).** 15-chip code period = 15 / 48 = **312 ms**. Worst-case clock drift over one code period at ±100 ppm = 31 µs = 0.007 frame — negligible. Acquisition time at 240 fps with one full code period = 312 ms; using soft-decision matched filtering with partial-code-window acquisition (handoff §5.4 + camera_considerations.md acquisition table), useful acquisition latency is well under 100 ms.
  - Alternate (tighter, less robust): chip rate = 120 Hz (2 frames/chip), 15-chip period = 125 ms — closer to camera_considerations.md's 63 ms acquisition target but with much less drift headroom + more sensitivity to per-frame motion blur. **Defer to bench** to decide between 48 Hz default and 120 Hz aggressive.
  - Hard floor (Nyquist): chip rate ≤ camera_fps / 2 = 120 Hz. Operate with comfortable margin (5×) to ride out frame drops + clock drift; that's the 48 Hz default.
  - The MCU firmware SHALL accept the chip rate as a build-time constant (or runtime register for the prototype), not hard-code a single value — keeps the bench iteration loop short.
- **Code length**: **15 chips** → one full code period = 15 × chip_period (= 312 ms at the 48 Hz default). Family of 17 N=15 Gold codes available, far more than the 2 the pair needs; cross-correlation bounded by 9/15 ≈ −4.4 dB peak vs −15 dB nominal (verify per pair selection offline). Acquisition error tolerance: 1 bit worst case at N=15 vs 0 bits at N=7 (camera_considerations.md table). This is the headline reason for the bump to 15.
- **Code source**: Gold-code pair generated from a pre-selected LFSR-pair, baked into the MCU as a 15-byte (or 2-byte packed) lookup table. The chosen pair's cross-correlation SHALL be ≤ −15 dB (verified offline at code-design time; pod just plays back the LUT).
- **Code ID assignment**: code A vs code B per pod, configurable via either (a) MCU EEPROM written at flash time, or (b) a wire jumper read at boot (handoff §5.5).
- **Duty cycle**: ~50% nominal (7-8 ON chips per 15-chip code; exact value falls out of the LFSR-derived Gold-code bit pattern). Peak LED current MAY exceed CW rating by 2-3× during ON chips since average is half — keeps photons high without thermal damage (handoff §5.3, §5.6).
- **Phase**: **free-running, self-clocked** from a small dedicated timer crystal or precision oscillator inside the pod. No sync to camera, no sync between the two pods, no externally-driven clock signal (which would require wiring and risk EMI coupling). The matched filter on the FPGA side tolerates the natural offset (handoff §5.4); the long code period at 48 Hz chip rate is well within ±100 ppm drift tolerance.
- **MCU workload**: one timer interrupt + 15-byte LUT + GPIO toggle. ~5 mW MCU power. Code generation is not the load-bearing complexity here; the driver chain is (handoff §5.3 last paragraph).

#### FR-1.4 Mass + power budget

Each pod's mass SHALL not exceed **6 g** (revised up from 4 g for the 4-LED design) including 4 LEDs (~30 mg each = ~120 mg LEDs), MCU + driver electronics, buck regulator + bulk caps, power-input EMI filter, pyramid mount, decoupling, and wiring tails. Total system mass for two pods + camera (FR-2) + onboard SD recorder (FR-4 flight mode) IS a flight-weight constraint from the 2026-05-12 revision onwards because Beacon Test Flight 1 (US6) flies the whole stack — but pods SHALL still be flight-weight from the start, not bench-only stand-ins.

Each pod SHALL be powered from the **main 3S LiPo flight pack** (11.1 V nominal, 9.0-12.6 V actual range) via the buck regulator in FR-1.2 — **not from a separate beacon-only battery**. The user direction 2026-05-12 is explicit: beacons share the main flight pack, with EMI mitigation (FR-1.6) handling the inevitable bus disturbance.

Pod power consumption SHALL be characterized at the canonical 50% duty operating point, **separately for each operating mode** (FR-1.2 / FR-1.7 dual-mode):

**High-power mode (flight-armed):**
- **Per-LED**: ~1 W electrical at 1 A drive (typ. Vf ≈ 1.9 V → 1.9 W instantaneous when ON; 50% duty → ~1 W average).
- **Per pod, 4 LEDs**: ~4 W instantaneous when all four ON (chip-ON intervals), ~2 W average across the duty cycle, plus ~50 mW MCU + buck + sense overhead.
- **From 3S LiPo (11.1 V)**: ~2 W avg / 11.1 V ≈ **180 mA average per pod**, **360 mA per-pod peak** during chip-ON intervals.
- **Two pods**: ~360 mA average, ~720 mA peak instantaneous current draw from the main flight pack — modest compared to ESC draw, but the **switching transients are the real concern** (FR-1.6).
- **Thermal**: 2 W average per pod is non-trivial in a 2.5 cm × 2 cm enclosure. Pyramid mount design SHALL provide a thermal path from the LED dies to the wing-tip aluminum/PCB to keep T_j below the LED's CW derating curve. Bench test: thermal soak at full duty for 5 min, IR thermometer or thermistor on the pyramid surface, confirm steady-state below 60 °C.

**Low-power mode (ground-handling / pre-arm / post-disarm / PWM-loss fail-safe):**
- Per-LED radiant output **5–10% of high-power** (firmware-defined; bench-tunable to balance eye-safety classification against bench-camera detectability).
- **Per pod**: ~100–200 mW average, ~400 mW peak; well below thermal-soak concerns.
- **Two pods, low-power**: ~30–50 mA average from main pack — negligible drain.
- Thermal: no constraint.
- **EMI envelope at low power**: switching di/dt scales with current, so low-power EMI is roughly 5–10% of high-power. The FR-1.6 mitigations dimensioned for high-power are over-spec for low-power; bench EMC at both modes per FR-3.3.

#### FR-1.5 Orthogonality verification

The two pods together SHALL produce two orthogonal Gold codes whose cross-correlation, when recovered by demodulating a bench-camera clip with both pods illuminated simultaneously, is **≤ −15 dB** (handoff §5.5). Verification path:

- (a) **Scope-level**: capture one pod's LED-anode (or sense-resistor) waveform on a fast scope, confirm the chip rate (48 Hz default, ~20 ms chip period) + 15-chip code period + correct Gold-code pattern;
- (b) **Photodiode-level**: pin a fast photodiode at 1 m, capture both pods' combined NIR signal, demodulate offline with both correlator templates, confirm A and B peak above each other's noise floor;
- (c) **Camera-level**: record a bench clip via FR-4 with both pods, demodulate the per-pixel intensity time-series at each blob, confirm both codes recoverable with the expected cross-correlation floor.

(a) is mandatory pre-shipment; (b) is a useful sanity check; (c) is what 031-fpga's Python golden model will reproduce against the same recorded clip.

#### FR-1.6 EMI mitigation (new 2026-05-12)

The pulsed 4-LED group drive presents a **first-class EMI risk** to the rest of the airframe — to the FC's gyros, the ESC, the RC link, and the autopilot's MSP/UART links. The mechanism is the di/dt at FET switching edges: ~2 A swinging in tens of nanoseconds couples into nearby loops as voltage spikes and radiates as broadband noise. With **two pods on the same 3S LiPo flight pack**, the disturbance has a direct conducted path back to every other airframe consumer via the shared battery rails.

The design SHALL include the following mitigations as defaults, with bench EMC characterization (FR-3.3 below) validating the choices:

1. **Slew-rate control at the FET gate**: gate resistor (typ. 100–470 Ω, bench-tune for waveform integrity vs. switching loss) to slow rise/fall edges to **a few hundred nanoseconds** (vs. the 10s of ns "natural" edge). Trades a small efficiency loss for dramatic reduction in conducted+radiated emissions. This is the single highest-leverage mitigation.
2. **Local bulk capacitance** at the buck-regulator output: 47–100 µF low-ESR (ceramic or polymer) close to the FET drain, plus 1 µF + 100 nF ceramic decoupling, to source the chip-ON pulse current without forcing the buck to slew the bus rail.
3. **Power-input π-filter** on the 11.1 V tail entering the pod: small ferrite bead (e.g., 100 Ω @ 100 MHz) in series + 10 µF + 100 nF to ground on both sides. Blocks the conducted return-path EMI from propagating back up the LiPo lead to the rest of the airframe.
4. **Twisted-pair LED return** between buck output and the LED group: minimizes the radiating loop area on the highest-current path.
5. **Pod-to-airframe wiring**: twisted-pair on the 11.1 V tail run; **separated routing** from the servo / ESC / RC-receiver signal harnesses inside the wing; entry into the fuselage on the opposite side of the FC from the RC-link antenna where physically possible.
6. **Pod metal shielding (optional, bench-evaluable)**: a thin copper-foil or conductive-paint layer on the inside of the 3D-printed pyramid, grounded to the pod's negative rail, with cutouts only at the LED apertures. Reduces near-field coupling to nearby surfaces.
7. **Chip-rate placement**: the 48 Hz default chip rate is *intentionally* well below the gyro filter cutoffs and the RC-link channel rate. Harmonics from FET-edge switching extend to MHz, however — those are what the bulk caps + π-filter target, not the chip rate itself.

#### FR-1.7 Dual-mode safety + eye-safety verification (new 2026-05-12 per clarification)

The dual-mode operation introduced in FR-1.2 is **safety-critical** — the contract that ground-handling operators and bench bystanders are exposed only to low-power radiance, while high-power radiance is enabled only when the airframe is airborne / armed. The design SHALL meet the following verifiable requirements:

1. **Power-on default**: on MCU boot, before any PWM input is decoded, the pod SHALL be in **low-power mode**. Verified by: bench measurement of LED radiant power within the first 50 ms of pod power-up with no PWM signal present.
2. **PWM-loss fail-safe**: if PWM pulses are absent for >250 ms (≥12 missed RX frames at 50 Hz) OR pulse width falls outside the 0.9–2.1 ms valid range, the pod SHALL revert to low-power mode within 50 ms. Verified by: scope-capture of LED-anode current envelope during forced PWM-loss event on bench.
3. **High-power mode trigger**: requires sustained valid PWM ≥ 1.5 ms for ≥100 ms (≥5 valid RX frames) before high-power radiance is enabled. Prevents transient signal glitches from accidentally enabling high power. Verified by: scope-capture of mode-switch latency vs. PWM-input edge.
4. **Operator workflow integration**: the pod's PWM input SHALL be wired to a TX channel that is **electrically and procedurally tied to the airframe's arming state** — typically a spare AUX channel that the operator sets via a TX switch as part of the pre-flight arming sequence. The spec does NOT mandate a specific TX channel; it mandates that the operator's pre-flight checklist (FR-5.2 bench-log / operator-notes equivalent for flight ops) ties high-power enable to the same physical action that arms the ESC.
5. **Eye-safety classification per IEC 62471**: bench-measured separately for **both modes** before US6 flight ops:
   - **Low-power mode**: target unconditional **Risk Group 0 (exempt)** or **Class 1 (eye-safe at zero distance)** at 200 mm per IEC 62471. Defines the operator-/bystander-safe envelope during ground handling, pre-arm, and post-disarm.
   - **High-power mode**: bench-measure radiant exitance with a calibrated NIR power meter at fixed distance (e.g., 200 mm and 1 m); compute classification; document **minimum safe-viewing distance** in the bench log. If above Risk Group 1, the bench log SHALL include a flying-field safety procedure (e.g., spectator-perimeter distance, "do not stare at armed-beacon airframe at <X m" placard).
6. **Documentation**: bench-measurement data + IEC 62471 calculation worksheet + final per-mode classification SHALL be persisted in the spec dir alongside this spec (`031-beacon-camera/eye-safety-measurements.md` or equivalent) before US6 flies.

### FR-2 Camera + lens + filter

**FR-2.1** The camera SHALL be a global-shutter mono sensor at VGA-or-better resolution capable of **≥240 fps at 320×240** (revised 2026-05-12 from "≥100 fps"). Primary choice: **ST VD55G1**; backup: **OmniVision OV9281** (Arducam B0162 module as the easy bring-up path). Both advertise 320×240 modes ≥240 fps per [camera_considerations.md](camera_considerations.md) sensor-comparison + handoff §6.1; bench-validate the achievable rate against Q8's exposure constraints in early hardware bring-up. Rolling-shutter sensors are explicitly disqualified (handoff §3.2).

**FR-2.2** The lens SHALL be M12-mount, NIR-corrected, ~120° H FOV, F/2.0-or-faster, with **integrated 850 ± 10 nm bandpass filter** in the housing. Sourcing: Commonlands (`contact@commonlands.com`) for custom config; generic 145° fisheye M12 (e.g., m12lenses.com PT-02120) for early prototyping before the Commonlands order arrives.

**FR-2.3** The capture pipeline SHALL produce **320×240 mono frames at 240 fps**, with bit-depth **selectable between 8-bit (baseline) and 10-bit (extended-dynamic-range option for post-hoc AGC simulation)** (revised 2026-05-12 from "8-bit only, 100–200 fps, bench-derived"). 240 fps is the **firm operating point** for the FR-1.3 chip-rate derivation (48 Hz chip = 5 frames/chip at 240 fps) and for camera_considerations.md's link-budget + acquisition-time analysis. The sensor's higher-fps modes (e.g., 480 fps at 320×240 on OG0VA) remain reserved for follow-on close-in motion-blur experiments per [camera_considerations.md §240 fps vs 480 fps Tradeoff](camera_considerations.md). If the achieved fps under the chosen capture host (Q1) falls below 240, that's a phase-1 blocker that triggers a Q1 revisit (e.g., move from USB to dedicated FPGA streamer) rather than a chip-rate compromise.

**Bit-depth selection rationale (clarification 2026-05-12)**: 8-bit matches the FPGA pipeline's `(x_i8, y_i8, cep_u8)` wire format directly and is the cheapest path through the SD recorder. 10-bit preserves 2 extra bits of dynamic range in the recording — critical for the **camera-dynamics-through-sun/dark-at-high-speed** study target (a tracker craft pitching from sky-toward-sun to sky-toward-shadow at body rate is exactly the AGC-stressor scenario that we want to characterize **offline**, replaying the raw 10-bit stream through simulated AGC curves to find the settings the live camera should use). Bit depth is recorded in the clip's chunk header (`bit_depth: 8 | 10`) so downstream loaders adapt automatically.

**FR-2.4** The captured frames SHALL be time-tagged with monotonically increasing per-frame timestamps at microsecond resolution (typically the sensor or FPGA's own frame-counter + a derived us clock).

**FR-2.5 Onboard SD-flash recording for flight mode (new 2026-05-12)**

The flight-mode capture host SHALL record raw frames + per-frame timestamps to an onboard SD-flash card at the FR-2.3 frame rate. Data-rate math (depends on FR-2.3 bit-depth selection):

| Mode | Pixel size | Sustained rate | SD class floor |
|---|---|---|---|
| **8-bit (baseline)** | 1 B/pixel | 320 × 240 × 1 B × 240 fps = **18.4 MB/s** | V30 minimum, V60 preferred |
| **10-bit packed** | 1.25 B/pixel | 320 × 240 × 1.25 B × 240 fps = **23.0 MB/s** | **V60 minimum**, V90 preferred |
| 10-bit unpacked-16-bit | 2 B/pixel | 320 × 240 × 2 B × 240 fps = **36.9 MB/s** | V90 minimum (use only if 10-bit-packed unpacking on the recorder is impractical) |

Plus per-frame timestamp + frame-counter overhead (~32 bytes/frame × 240 fps = 8 kB/s) and JSON sidecar (one-shot at clip start/end) — both negligible against the pixel-stream rates above.

**Headline write rate: ~18.5 MB/s @ 8-bit / ~23 MB/s @ 10-bit-packed.** The recorder SHALL:

- Use an SD card rated per the bit-depth table above (V30 floor for 8-bit, V60 floor for 10-bit-packed, V90 for 10-bit-unpacked) with at least 1 class of headroom recommended against write-amplification stalls and end-of-card slowdown.
- Use a host SD interface capable of 4-bit SDIO @ ≥25 MHz (≥12.5 MB/s theoretical) — **standard 1-bit SPI mode is NOT sufficient** (max ~10 MB/s in practice, below the 18.5 MB/s requirement).
- Implement a **ring buffer in RAM** (≥250 ms / ~4.5 MB) between the camera capture and the SD write to absorb file-system flush latency without dropping frames.
- **Pre-allocate the clip file at session start** (avoids fragmentation-induced slowdown mid-flight) and then write directly to its pre-allocated sectors — bypassing FS-allocation bookkeeping on the hot path while keeping the recovered clip mountable as a normal file post-flight.
- **Auto-start recording on power-up** (no operator action mid-flight); auto-stop on power-down or SD-full.
- Write to a clip-file format compatible with FR-4.3's Python loader: raw `.bin` with **independently-parseable ~1-second chunks** (each chunk = ~18.5 MB = 240 frames + per-frame timestamp headers + chunk header), plus JSON sidecar.
- **Mid-flight fault handling** (per clarification 2026-05-12): on detected fault (SD-full, write error, brown-out signal from a recorder-host watchdog if available) — **immediate-truncate the current chunk + finalize the file**. Surviving chunks remain loadable by FR-4.3 even if the trailing chunk is lost. **No per-frame fsync** — the 1-second chunk boundary is the durability granularity; trailing-frame loss up to ~1 s is accepted.
- **Raw-LBA writes fallback**: if bench-benchmarking on the chosen Q1 platform shows that FS-mediated pre-allocated writes can't sustain 18.5 MB/s without frame drops, the spec permits a fallback to **raw block writes (SD `CMD24` / `CMD25`)** bypassing the FS entirely — paired with a host-side offload tool that knows the recording's LBA layout to extract clips. Decision deferred to bench.

Total storage for a typical 10-minute paired-craft flight session: 10 × 60 × 18.4 MB ≈ **11 GB @ 8-bit** / ≈ 14 GB @ 10-bit-packed. A 64 GB or 128 GB card gives comfortable margin for several flights between offloads at either bit depth.

**Candidate flight-mode hosts** (revised 2026-05-12 per clarification — the recommended path flipped to FPGA-eval-board because the build doubles as 031-fpga's camera-ingest proof):

- **(default) Lattice CrossLink-NX-EVN dev board + onboard SD socket**: small FPGA program does camera-config (sensor I²C setup) + raw-frame DMA-to-SD. **This build also proves the camera→FPGA path** that 031-fpga depends on, so the engineering is reused. Bit-depth-flexible (8 or 10 native on the sensor; FPGA logic handles packing).
- Raspberry Pi CM4 / Pi 5 + OV9281 module: middle ground, has SDIO + USB; needs careful tuning to sustain 18.5–23 MB/s to SD without dropping frames at 240 fps (CSI-2 ingest + DMA + SD write all on one SBC). Falls back here if the FPGA build slips.
- Custom MCU + FPGA hybrid (e.g., RP2040/STM32H7 frame ingest → external FPGA shim → SD): more engineering, marginal benefit over the eval-board.

Standard Linux PC USB-tethered (the prior Q1 default) is NOT compatible with the flight-mode FR-2.5 requirement — it remains acceptable for the **bench-mode** clips per FR-4 below.

**Offload paths (clarification 2026-05-12)** — both produce identical files:
- (a) USB download from the eval-board (recorder runs a small file-server mode when not recording, e.g., USB Mass Storage exposing the SD card).
- (b) Pull the SD card and read directly on the PC via a standard card reader.

**Carrier-craft caveat (clarification 2026-05-12)**: the FPGA eval-board flight-recorder is **over the weight budget for the eventual target tracker craft**. Phase-1 US6 flights therefore use a **larger carrier craft** (hb1-class or a trainer-class airframe with margin to lift the eval-board + SD socket + wiring) as the recording platform, NOT the production tracker. Smaller / custom flight hardware (bare-die sensor + custom flex PCB + small FPGA, or the bare OG-series module + minimal carrier per camera_considerations.md) is a **031-integration / production-flight follow-on** — not phase 1. This is an accepted weight trade for shorter time-to-first-data + reused FPGA work for 031-fpga.

**FR-2.6 SD-recorder status indicator (new 2026-05-12 per clarification)**

Flight-mode recording is fire-and-forget by design (FR-4.1b), so the recorder SHALL surface its operating state via a **visible blinking-LED indicator** the operator checks pre-takeoff. Early flight builds will typically run on dev-kit boards (RPi CM4 / Lattice eval / RP2040 + carrier) that already expose R/G/B GPIO-driven LEDs; this requirement is satisfied by repurposing those onboard LEDs rather than mandating a custom indicator board.

Pattern (following [xiao/src/util.cpp:15-32](../../xiao/src/util.cpp#L15) `heartBeatLED()` + [xiao/include/main.h:14-20](../../xiao/include/main.h#L14) RGB-pin convention used by the NN controller):

| State | LED behavior |
|---|---|
| Power-on / pre-init | RED solid (still bringing up sensor + filesystem) |
| Initializing (sensor enumerated, FS mounted, clip pre-allocating) | GREEN blinking, 250 ms period (matches xiao `BLINK_INTERVAL_MSEC`) |
| **Recording (writing to SD, no errors)** | **GREEN solid** — the "go for takeoff" signal |
| Mid-flight write stall / frame drop / ring-buffer near-overrun | YELLOW or RED blinking, 100 ms period (rapid alert) |
| Mid-flight unrecoverable error (SD full / write fault) | RED solid |
| SD card missing or unreadable at boot | RED blinking, 250 ms period |

The indicator SHALL be placed where the operator can see it during pre-flight walk-around without removing the camera mount cover or any wing tape (e.g., LED visible through a small light-pipe slot in the recorder housing, or mounted on an exposed face of the dev-kit board). Operator pre-flight checklist (FR-5.2 / US6 pre-flight safety gate) SHALL include "confirm GREEN-solid on recorder LED" before throttle-up.

The blink-code table SHALL be documented alongside this spec (`031-beacon-camera/recorder-status-codes.md` or equivalent quick-reference card) so a future operator unfamiliar with the build can interpret the LED at the flying field.

### FR-3 Beacon-emission verification

**FR-3.1** Bench-side, the operator SHALL be able to verify that the LED pyramid's emission pattern matches the design's **270° spherical coverage with unlit cone inboard** (FR-1.1) by rotating a single pod through both azimuth AND elevation in front of the camera at fixed distance, recording received signal vs. the two angles. The blind cone (apex pointed away from camera) SHALL be the only region with recovered signal below the lock threshold. (This is part of the US5 scenario set.)

**FR-3.2** Code orthogonality SHALL be bench-verifiable: with both pods on simultaneously at distinct codes, a recorded clip + Python-side demodulator SHALL show two recoverable signals with the expected cross-correlation floor.

**FR-3.3 Bench EMC sanity check (new 2026-05-12)**

Before any flight session in US6 (Beacon Test Flight 1), the operator SHALL perform a **bench EMC sanity check at BOTH power modes** (low + high, per FR-1.2 / FR-1.7):

- Power one beacon pod, mount it ~10 cm from a representative flight controller running the target firmware (INAV in our case).
- Drive the PWM input to **low-power mode** (e.g., 1.0 ms pulse), then **high-power mode** (2.0 ms pulse). Compare both against beacon-OFF baseline.
- For each mode, compare:
  - **Gyro noise floor** (gyro spectrogram, blackbox log, or INAV's CLI `sensor_info` / gyro RMS reading) with the beacon OFF vs. ON. Acceptance: no measurable degradation, or degradation below the gyro's natural noise floor at flight conditions.
  - **RC link RSSI / link quality** with the beacon OFF vs. ON at the same RX/TX positions. Acceptance: no measurable degradation.
  - **ESC behavior / throttle response** (if powered up) with the beacon switching. Acceptance: no twitches, no audible signs of PWM disturbance.

A FAIL in **high-power mode** SHALL block the test flight and trigger a redesign iteration on FR-1.6 mitigations (slew rate, shielding, filtering, routing). A FAIL in **low-power mode** is more concerning (current envelope is much smaller) and indicates a fundamental routing or shielding problem that SHALL be addressed before continuing. Document failures + remediation in the bench log per FR-5.2.

**FR-3.4 Dual-mode + eye-safety verification (new 2026-05-12 per clarification)**

Before any flight session in US6, the operator SHALL bench-verify FR-1.7's six safety-critical contracts:

1. Power-on default to low-power mode (no PWM signal): bench scope-measure LED-anode current within first 50 ms of pod power-up.
2. PWM-loss fail-safe → low-power within 50 ms after >250 ms PWM absent: bench scope-measure.
3. High-power mode trigger requires sustained valid PWM for ≥100 ms: bench scope-measure mode-switch latency.
4. Operator-workflow tie to airframe arming state: documented in pre-flight checklist (FR-5.2 equivalent).
5. IEC 62471 measurements at 200 mm + 1 m for both modes; per-mode classification documented.
6. Persistence: bench-measurement data + IEC 62471 worksheet + classification in `031-beacon-camera/eye-safety-measurements.md` (or equivalent).

A FAIL in any of #1-#3 (the safety-critical PWM contracts) SHALL block the test flight and trigger an immediate firmware redesign. A FAIL in #5 (eye safety) does NOT necessarily block flight but DOES dictate the spectator-safety procedures for US6.

### FR-4 Raw-frame recording substrate

(Revised 2026-05-12 — recording now supports two modes producing the same file format: bench-mode for indoor scenario sweeps; flight-mode for Beacon Test Flight 1 per US6. Flight-mode is the new addition; bench-mode is unchanged.)

**FR-4.1 — bench mode (USB-tethered)** The bench recording system SHALL stream raw 320×240 mono frames at 240 fps + per-frame metadata to a host computer (USB / SDIO / Ethernet-tethered acceptable) for clips up to 60 seconds without dropping frames. Used for FR-3.1 / FR-3.2 / FR-5.1 indoor + outdoor bench sessions where a host PC is co-located.

**FR-4.1b — flight mode (onboard SD)** The flight recording system SHALL record raw frames per FR-2.5 to an onboard SD-flash card, auto-starting on power-up and continuing for the full duration the camera is powered (up to SD-full). Used for US6 (Beacon Test Flight 1) and any future field session where tethering is not viable.

**FR-4.2** Each recorded clip (bench or flight mode) SHALL produce two artifacts: (a) the binary frame stream as specified in the canonical [`data-format.md`](data-format.md) (chunked, versioned `format_version` field, raw 8-bit or 10-bit-packed pixels with per-frame + per-chunk timestamp headers), and (b) a JSON sidecar with per-clip metadata: clip-id, ISO-8601 start time, sensor model + firmware rev, lens + filter spec, capture resolution + fps + bit-depth, sensor exposure + gain settings, ambient-light qualifier (manually logged: indoor / outdoor / cloudy / direct-sun), range-to-target qualifier (manually logged or measured), pose qualifier (stationary / hand-panned / on-aircraft / **in-flight**), recording-mode qualifier (bench-tethered / flight-SD), beacon-mode qualifier (low-power / high-power / mixed), notes free-text. **In-flight clips** SHALL additionally capture: target craft ID + beacon code IDs A/B + airframe configs + intended flight pattern + observed-from-ground notes.

The [`data-format.md`](data-format.md) doc is the **single source of truth** for the byte layout; it SHALL be authored before the first flight-mode build commits gateware, and the FR-4.3 Python loader SHALL be the executable reference implementation. Format version bumps require updating both `data-format.md` AND the loader in the same change.

**FR-4.3** A Python utility SHALL load a recorded clip (either mode) into a `numpy.ndarray` of shape `(n_frames, height, width)` with `dtype=uint8` plus a `dict` parsed from the JSON sidecar. This is the canonical handoff interface to all downstream tooling (sim noise calibration, FPGA Python golden-model test vectors, hand-rolled blob detector experiments, **the planned follow-on field-data analysis spec**).

### FR-5 Operating-envelope characterization (phase-1 bench observations)

**FR-5.1** A set of canonical bench scenarios SHALL be defined (close-range static / far-range static / hand-panned / outdoor-direct-sun / outdoor-cloudy / dim-indoor — exhaustive list TBD during execution but pre-agreed before the first session). Each scenario produces a recorded clip per FR-4.

**FR-5.2** A written bench-log entry per session SHALL capture clip-id, scenario, observed qualitative behavior (beacon visible / blooming-on-sun / blob-mass-vs-range / motion-blur-onset-rate / etc.), and unresolved questions. The bench log is **the primary phase-1 deliverable**; it's the source of the iteration loop for the simulator's noise model and the FPGA pipeline's threshold tuning in follow-on specs.

## Non-Functional Requirements

### NFR-1 Mass + power (revised 2026-05-12 for 4-LED pod + flight-mode recorder)

- Per-pod mass: ≤6 g (4-LED design; was ≤4 g for 3-LED).
- Per-pod power: ~2 W average / ~4 W peak from 3S LiPo (4× 1 W LEDs at 50% duty); ~180 mA average / ~360 mA peak instantaneous current per pod.
- Camera + lens mass (for flight, enforced from US6 onwards): 2–3 g target.
- Flight-mode recorder (camera + SD writer + battery if any): TBD when Q1 resolves to a specific platform; target ≤30 g for the tracker craft's installed payload.
- EMI envelope: bench-EMC-validated per FR-3.3 before any flight; no measurable degradation of FC gyro, RC link, or ESC behavior with beacons powered.
- FPGA path (NOT phase 1): out of scope for this spec.

### NFR-2 Determinism + repeatability

- Recorded clips SHALL be reproducible: same setup + same scenario → same observed envelope. Where day-to-day ambient changes a bench result, the bench log SHALL note it.
- The simulator's downstream consumption of clips (in follow-on calibration work) requires bit-exact replay — the clip-file format SHALL preserve every raw pixel value with no lossy compression.

### NFR-3 Iteration-friendly

- All hardware decisions SHALL be **reversible** at this phase: no PCB spins until the optical chain is proven on dev modules. The Arducam OV9281 module + a CrossLink-NX-EVN / equivalent dev board is the canonical first build; bare-die mounting + flight PCB is a 031-integration concern.
- All component choices SHALL be **traceable** to the handoff doc decision (§3-§5) or to a bench-experiment delta that justified deviating.

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
| Detection: temporal coding (NOT spectral) | handoff §3.1 | Off-the-shelf single-band filter + Gold-code orthogonality; single sensor; matched-filter processing gain handles two beacons |
| Shutter: global | handoff §3.2 | Rolling-shutter at 500°/s body rate produces ~17 px skew; rejected |
| Resolution: 320×240 (8-bit) | handoff §3.4 + §6.1 | Output is 8-bit signed (x, y); 320×240 ≥ 256-code precision with sub-pixel centroiding |
| Single camera in phase 1 | handoff §3.5 | Architect for multi-cam (I²C address scheme); build one |
| LED: Lumileds Luxeon IR Compact 850 nm | handoff §4.4 | 1.3 W radiant @ 1 A, 2.75 × 2.0 mm pkg, ~30 mg/die, pulsable to 5-10 A peak |
| **LED arrangement: 4 dies on 3D-printed 4-sided pyramid, apex outboard, unlit cone INBOARD** | user direction 2026-05-12 | 270° spherical coverage from any external aspect; only fuselage-shadow direction is blind (which the craft occludes anyway) |
| **LED drive topology: 4 LEDs wired as one Gold-code-modulated group (2S2P), single FET** | user direction 2026-05-12 | Simplicity + deterministic emission; no per-facet PWM needed; one EMI source to mitigate |
| **Pod power: shared with main 3S LiPo flight pack (no separate beacon battery)** | user direction 2026-05-12 | Mass + integration simplicity; forces EMI mitigation per FR-1.6 |
| **Pod connector: standard 3-wire RC servo connector (V+ / GND / PWM signal)** | clarification 2026-05-12 | Drops into operator's existing servo harness; pod is electrically a "servo slot"; spare RX channel drives PWM mode select |
| **Dual-mode operation: low-power (5-10% radiance, ground-safe) + high-power (full radiance, flight-armed), selected by PWM pulse width on the RC servo input, fail-safe to low-power on PWM loss** | clarification 2026-05-12 | Eye-safety solution at the source: ground handling + bench + pre-flight all operate at intrinsically eye-safe radiance; high-power radiance gates on the same TX switch that arms the airframe |
| Lens: Commonlands M12 NIR-corrected ~120° F/2.0 w/integrated 850 ± 10 nm filter | handoff §4.2 | Integrated filter saves separate alignment + mass; <8 µm focal shift visible→NIR |
| Sensor: ST VD55G1 primary / OV9281 backup | handoff §4.1 | Both global-shutter, mono, NIR-friendly; OV9281 ecosystem (Arducam B0162) is the easy bring-up |
| **Camera fps: 240 (firm)** | user direction 2026-05-12 + camera_considerations.md | Sets the FR-1.3 chip-rate derivation; matches camera_considerations link-budget analysis |
| **Code length: 15-bit Gold** | user direction 2026-05-12 + camera_considerations.md acquisition-error table | 1-bit worst-case acquisition error tolerance (vs 0-bit at N=7); 17-code family for future expansion |
| **Chip rate: 48 Hz default (5 frames/chip at 240 fps); 120 Hz aggressive alternate (2 frames/chip)** | derived from FR-1.3 | 48 Hz default preserves drift margin + motion-blur immunity; 120 Hz alternate sharpens acquisition latency, bench-deferred decision |
| **Recording substrate: dual mode — bench (USB-tethered to Linux PC) + flight (onboard SD, U3/V30 min, V60 preferred, 18.5 MB/s sustained)** | user direction 2026-05-12 | US6 (Beacon Test Flight 1) requires onboard recording; bench mode unchanged from prior draft |
| **Flight-recorder status indicator: blinking-LED on recorder housing per xiao `heartBeatLED()` pattern; GREEN-solid = recording; checked pre-takeoff** | clarification 2026-05-12 | Fire-and-forget recording without a state indicator wastes flight sessions on silent failures; dev-kit boards already expose R/G/B GPIO LEDs so this is a near-zero-cost requirement |
| **Clip-file durability: pre-allocated file + chunked direct-sector writes (~1 s blocks); each chunk independently parseable; ~1 s trailing-frame loss accepted on power-loss/crash. Raw-LBA fallback permitted if FS bookkeeping bottlenecks 18.5 MB/s** | clarification 2026-05-12 | Standard dashcam-/black-box-recorder pattern; loses last second on hull crash (acceptable per operator); avoids per-frame fsync overhead |
| **US6 acceptance: infrastructure validation (emit → collect → record → ingest → DSP → visualize), NOT beacon-detection-quota** | clarification 2026-05-12 | US6 proves the chain works end-to-end; detection-quantity envelope is the follow-on noise-cal/DSP-tuning spec's job. The recording is the deliverable — fitness-for-downstream-tools is the acceptance test |
| **Recording file format: dedicated `data-format.md` + versioned chunk header (`format_version` uint16); FR-4.3 Python loader is reference implementation** | clarification 2026-05-12 | Single source of truth; loader-as-executable-spec discipline; prevents byte-layout drift across the FPGA recorder, the bench recorder, and downstream sims |
| **Bit-depth: 8-bit baseline OR 10-bit-packed (selectable per clip); chunk-header `bit_depth` field signals to loader** | clarification 2026-05-12 | 8-bit matches FPGA wire format; 10-bit preserves dynamic range for offline AGC-dynamics simulation through sun/shadow transits at body rate |
| **Flight-mode recorder host: Lattice CrossLink-NX-EVN FPGA eval-board + SD, NOT RPi CM4** | clarification 2026-05-12 | FPGA build doubles as 031-fpga's camera-ingest proof; the engineering is reused, not duplicated. RPi CM4 is fallback if FPGA gateware slips |
| **Clip offload: USB-from-board OR direct SD-card-read; both produce identical files** | clarification 2026-05-12 | Field-flexible (no laptop at flying field if pulling the card is enough); same file = same loader |
| Async LED-camera clocks | handoff §5.4 | Matched filter tolerates ±100 ppm offset; sync adds complexity for no first-order benefit |

## Open Questions (resolved during phase-1 execution OR escalated to follow-on)

These are explicit unknowns at draft time. The /clarify step is expected to either firm them up before plan, or accept them as bench-discovered.

### Q1 — Phase-1 capture-host platform (revised 2026-05-12 — split bench vs flight)

The 2026-05-12 revision split this question along the recording-mode boundary in FR-4: bench mode and flight mode have different platform constraints.

**Bench mode (FR-4.1)** — tethered to a host PC, no autonomous start, host-side processing OK:
- (a) Standard Linux PC with USB-tethered Arducam B0162 / OV9281 module — proven path, fastest cost-to-first-frame.
- (b) The Lattice CrossLink-NX-EVN dev board acting as a raw-frame USB streamer — preempts 031-fpga work, FPGA gateware is non-trivial.
- (c) A Raspberry Pi CM4 / Pi 5 with the OV9281 module — middle ground.

**Default for bench mode**: (a) — Arducam B0162 + USB on Linux PC. Lowest cost-to-first-frame; phase-1 bench scenarios (FR-5.1 S1-S9) don't need flight-mode capabilities.

**Flight mode (FR-4.1b, FR-2.5)** — onboard, auto-start-on-power-up, no host PC, 18.5 MB/s @ 8-bit / 23 MB/s @ 10-bit sustained write to SD:
- (d) Raspberry Pi CM4 with OV9281 CSI-2 module + onboard SD (4-bit SDIO @ 50 MHz) — known platform. Requires careful real-time tuning (CSI ingest + DMA + SD write all on one SBC). Fallback if (e) slips.
- (e) **Lattice CrossLink-NX-EVN dev board with onboard / daughter SD socket** — FPGA program does camera-config + bulk-record-to-SD. **This build doubles as 031-fpga's camera→FPGA ingest proof**, so the engineering is reused, not duplicated. Bit-depth-flexible (handles both 8-bit and 10-bit-packed in HDL).
- (f) Custom MCU+FPGA hybrid (e.g., STM32H7 ingest + iCE40 shim + onboard SD) — most engineering effort, marginal benefit over (e).

**Default for flight mode (revised 2026-05-12 per clarification)**: **(e) — Lattice CrossLink-NX-EVN + SD**. The "build cost" of an FPGA recorder is recovered immediately by reusing it as the 031-fpga starting point; the alternative (RPi CM4 path) would build single-use code and re-do the camera-ingest later for FPGA. Decision flipped from the prior draft's RPi-CM4 default.

**Implication**: the bench-mode and flight-mode hardware are still different boards (USB-to-Linux-PC for bench; FPGA-eval-board-with-SD for flight), but the flight-mode recorder is now **the first deliverable of 031-fpga in pipe**, not throwaway phase-1 work. The Python loader (FR-4.3) accepts both, anchored on the canonical [`data-format.md`](data-format.md). Operator-accepted trade: longer cost-to-first-flight (FPGA gateware learning curve) for shorter cost-to-031-fpga (camera-ingest already proven on board).

### Q2 — Recording format

Raw `.bin` with documented stride + JSON sidecar, OR `.npy` numpy array, OR `.tiff` per frame, OR multi-frame `.avi`/`.mkv` (mono lossless)?

**Default**: `.npy` files per clip (frames × H × W, uint8) plus matching `.json` sidecar. Reasoning: directly loadable by the Python utility per FR-4.3, no codec dependency, exact pixel preservation. Downsides: larger file size than lossless video containers but phase-1 clips are short and disk is cheap.

### Q3 — Bench scenario list (phase-1 envelope)

Per FR-5.1 the scenarios will be pre-agreed before the first capture session. Initial proposal (operator to amend):

- **S1 — close-range static** (1 m, indoor, dim ambient) — sanity, expect strong signal both beacons
- **S2 — mid-range static** (10 m, indoor, mixed ambient) — typical bench distance
- **S3 — far-range static** (50 m, outdoor, varies) — closer to the design 100 m
- **S4 — hand-panned slow** (10 m, hand-rotating camera at ~30°/s) — exercises track-table predictor in follow-on FPGA work
- **S5 — hand-panned fast** (10 m, hand-rotating at >100°/s) — approaches body-rate envelope
- **S6 — direct-sun adversarial** (10 m, outdoor, sun within camera FOV) — exercises filter sun-rejection
- **S7 — dim ambient adversarial** (10 m, dusk / late-evening) — exercises gain headroom
- **S8 — beacon obscured / partial** (10 m, briefly occluding one beacon by hand) — exercises sentinel detection in follow-on
- **S9 — paired-beacon close spatial proximity** (50 m, beacons appear ~1 px apart) — exercises CCA two-blob-merge case per handoff §6.11

Each scenario produces one 30-60 s recorded clip. The full set is the phase-1 "ship" artifact — it's the substrate for every follow-on optical experiment.

### Q4 — Pod mechanical mount (operator-resolved 2026-05-10, revised 2026-05-12)

**Resolution**: 3D-printed custom mount per pod, sized for the hb1 wing-tip max-chord profile, **with the pyramid apex pointing outboard** (perpendicular to fuselage centerline, away from the craft) so the unlit cone aligns with the fuselage-shadow direction (FR-1.1).

Mount geometry:
- **Base**: matches the flat wing-tip surface at max chord, **~2.5 cm square**. Mounts directly to the wing-tip top surface with double-sided tape or thin adhesive (reversible for early bench, glue-up once design freezes).
- **Pyramid**: **~2 cm tall, 4-facet (square-pyramid)** for the FR-1.1 270° spherical emission pattern (revised 2026-05-12 from prior 3-facet baseline). **Apex points OUTBOARD** (perpendicular to wing chord, away from fuselage).
- **Facet tilt**: each ~45° from the pyramid-axis (outboard direction), 90° apart around the axis. LEDs mounted normal to each facet → four 130°-beam-width emission cones spread around the outboard hemisphere with ~30° per-LED overlap at the seams; the four cones all overlap at the on-axis (straight outboard) direction; the back of the pyramid (inboard face / no facet) is the unlit cone.
- **Cable tail**: small slot/channel for the 11.1 V LiPo tail + the 2-wire MCU programming/jumper pads, running inboard along the wing leading edge to the fuselage.
- **Print orientation**: pyramid apex up so the four LED-mount facets print without overhangs (FET + MCU + buck pads on the underside of the base).
- **EMI considerations** (FR-1.6): if the metal-shielded variant is built, the conductive coating extends across the inner pyramid surface but stops at the LED apertures; ground is at the base PCB.

For bench-only sessions before the airframe integration (FR-5.1 / Q3 scenarios S1-S7, S9):
- Both mounts on a wood / foam jig at **hb1 wing-tip spacing (~0.9 m)** with both pyramid apexes pointed outboard from the simulated fuselage centerline — replicates the eventual flight geometry.
- (S9) Variable spacing for the paired-beacon proximity case (push pods together until the camera sees them as one cluster at 50 m), keeping apex-outboard orientation.

For **US6 (Beacon Test Flight 1)**: glue-up the pods to the hb1 wing-tip top surfaces with the LiPo tail routed cleanly inboard under wing tape. EMI sanity check (FR-3.3) precedes the first flight.

The 3D print files belong in the 031 spec dir alongside this spec when fab starts.

### Q5 — Acceptance bar for "phase 1 done" (revised 2026-05-12 per US6 clarification)

Two reasonable bars, both updated to the infrastructure-validation framing:

- (a) **Minimum**: all 9 bench scenarios captured + **one paired-craft test flight (US6) captured to onboard SD**, all clips loadable in Python per FR-4.3. No DSP / visualization required at this bar.
- (b) **Stretch**: minimum + **at least one DSP pass-through demonstration** (e.g., correlator + blob detector executed against the US6 clip producing sane outputs, regardless of detection count) + **at least one visualization pass** (per-frame imagery viewer + per-pixel time-series). Demonstrates the recording is fit-for-purpose as input to follow-on noise-cal / FPGA-sim / detection-DSP work. No FPGA-pipeline replication required.

**Default**: (b). Bar (a) leaves "is the clip actually usable?" open — pulling the SD card and finding a file you can't ingest is the same as no flight. Bar (b) is essentially the US6 acceptance plus the bench-scenario coverage; the DSP/visualization passes are 1–2 days of Python work after the field session.

The US6 flight clip is the **headline deliverable** of phase 1; it's the substrate the follow-on noise-cal / DSP-tuning spec gates on.

### Q7 — Camera operating mode + derived chip rate (revised 2026-05-12 — 240 fps + 48 Hz are now defaults)

Chip rate is downstream of camera frame rate (FR-1.3 + handoff §5.3 "≥5 frames per chip"). The 2026-05-12 revision firms the camera operating point to **320×240 @ 240 fps** (FR-2.3) and the chip rate to **48 Hz default** (FR-1.3) — both backed by the user direction and camera_considerations.md link-budget analysis.

Residual unknowns the bench resolves:

- **Whether 320×240 @ 240 fps is sustainable in both modes (bench-tethered / flight-SD)** — depends on Q1 platform choice. If either path can't sustain it, Q1 needs to revisit hardware (different sensor module, different capture board), not chip-rate or fps compromise.
- **Cropping vs binning** at 320×240 on the chosen sensor (Q8 captures the detailed sub-questions).
- **Body-rate-induced motion blur** at 240 fps + the chosen exposure — checked qualitatively in S5 (fast pan) + US6 (real flight).
- **48 Hz vs 120 Hz chip rate choice** — defer to bench data: 48 Hz default is robust + has drift margin; 120 Hz tightens acquisition latency but is more sensitive to motion blur. Quantitative sweep is a follow-on, NOT phase-1 acceptance.

**Implication on FR-1.3 / FR-2.3**: both clauses are now firm at 240 fps + 48 Hz (no longer "bench-derived"). The wire-format contract (FR in 031-protocol future spec) is unaffected — chip rate is invisible above the FPGA's correlator interface.

### Q8 — Sensor fps mechanism + daylight exposure budget

The handoff doc claims "320×240 on both VD55G1 and OV9281 can hit 300+ fps" (§6.1 line 305). That number needs to be unpacked against the actual sensor datasheet mode tables and the daylight operating envelope before we order parts.

**Three sub-questions to resolve from datasheets + bench:**

1. **How is 320×240 achieved on each sensor — cropping (ROI windowing) or binning (2×2 / 4×4 pixel summing)?**
   - Cropping: reads only a 320×240 window from the full sensor. Per-pixel sensitivity unchanged. Frame readout time drops linearly with window size → higher max fps. The "free lunch" path.
   - Binning: sums neighbor pixels in hardware, trading spatial resolution for SNR. Useful at low light (shot-noise-limited), less helpful at daylight (sun-saturation-limited).
   - **Both VD55G1 and OV9281 advertise both modes** in their datasheets; the achievable fps in each mode is mode-specific. Bench-side, prefer cropping for daylight, evaluate binning for dim ambient.

2. **What max exposure can we run at noon-sun ambient without saturating pixels?**
   - 850 nm bandpass filter (≈10 nm passband, ~85% transmission) cuts visible sun by ~50× but doesn't help against 850 nm sun.
   - Sun irradiance at sensor through filter ≈ ~10 W/m² (back-of-envelope: 1000 W/m² × 10 nm / 1000 nm × 0.85).
   - LED at 100 m through filter ≈ 25 mW/m² (handoff §5.6 link-budget) — **400× below ambient**.
   - Pixel-saturation budget for an 8-bit sensor with full-well ~7-10 ke⁻ × QE_850 ~30% gives ~1-3 ms before saturation in direct sun (rough estimate; needs sensor-specific QE × full-well numbers).
   - **Implication**: 1 ms exposure (handoff §5.6 assumption) is plausibly the **upper bound** at noon; the actual cap is sensor-specific and bench-measurable.

3. **What's the fps gate at the selected exposure?**
   - `fps_max ≈ 1 / (max(exposure, readout_time_320x240) + overhead)`.
   - Cropped readout time at 320×240 is in the few-hundreds-of-µs range for both sensors (extrapolating from the native 1/48 s = 21 ms for VD55G1 at full res, the cropped readout should scale by area ratio).
   - At 1 ms exposure + 0.5 ms readout + overhead, theoretical max fps ≈ 600. Handoff's "300+" is consistent with this leaving conservative headroom.
   - **Sustained-fps-to-disk** is gated by the capture host (Q1), not the sensor. USB 2.0 streaming at 320×240×8-bit×300 fps = 23 MB/s, which is within USB 2.0 bulk-transfer (~35 MB/s practical) but uncomfortable; USB 3.0 or SDIO removes the bottleneck.

**Datasheet check, before ordering**:
- VD55G1 datasheet mode table: confirm 320×240 mode exists, its max fps, its binning vs cropping mechanism.
- OV9281 datasheet (Arducam B0162 has firmware modes documented separately): confirm 320×240 cropped fps achievable, AGC step behavior, exposure quantization at sub-millisecond.
- Bench experiment after hardware lands: walk exposure from 0.1 ms to 5 ms in 0.5 ms steps under noon sun, capture for each, find the largest exposure that keeps no pixel saturated.

**Arducam B0162 spectral compatibility** (bench dev module ↔ flight hardware parity):
- OV9281 silicon has good 850 nm QE per handoff §4.1.
- The Arducam B0162 module ships with an M12 lens that **may** include an IR-cut filter optimized for visible-light cameras. If present, the IR-cut filter will reject our 850 nm signal before it reaches the sensor — destroying the entire optical path for our application.
- Phase-1 first step on the receiver: **inspect / measure the bundled Arducam lens for IR-cut behavior** (visual: IR-cut filters often look slightly blue/cyan in reflection; functional: point an IR remote control at the lens and see if the sensor sees the LED).
- If IR-cut is present: either remove the filter, swap the lens for a generic 145°-fisheye M12 (per FR-2.2 alternate path, e.g., m12lenses.com PT-02120 ~$30, typically no IR-cut), OR jump straight to the Commonlands order with integrated 850 nm bandpass (handoff §4.2).
- For early bench bring-up before the Commonlands lens arrives: generic M12 fisheye + a discrete 850 nm bandpass filter (Edmund Optics / Thorlabs, ~$30) sandwiched in front of the lens. Validates the spectral chain end-to-end without waiting on Commonlands lead time.
- **Decision rule**: the bench setup's spectral response must match flight hardware (with the Commonlands bandpass lens) within a tight enough tolerance that bench-recorded clips remain calibration-valid for the flight build. Quantify with a known 850 nm source at multiple drive currents before declaring the bench substrate fit for FR-4 deliverables.

**AGC dynamic behavior** (the soft constraint per handoff §6.11):
- Both sensors have AEC/AGC loops that adjust exposure + gain to keep average scene brightness in range.
- For our application, the LED is a small bright point; the average scene brightness is dominated by sky/ground, not the LED.
- AGC will respond to scene brightness changes (cloud / shadow / sun-angle) with some time constant — needs characterization. The FPGA's IIR background subtraction in Stage 1 (handoff §6.3) absorbs slowly-varying AGC adjustments; fast AGC steps could leave temporary artifacts. **Bench-measurable**, defer until clips in hand.

**Implication on the spec (revised 2026-05-12)**: phase 1 locks **fps = 240** (FR-2.3) and **chip rate = 48 Hz default** (FR-1.3); exposure is bench-tuned per Q8 sub-question 2 above. The 240 fps "design point" is now firm — if the Q1-chosen capture host can't sustain it, that's a Q1 revisit, not a chip-rate or fps compromise. Exposure varies session-to-session with ambient conditions and is captured in each clip's JSON sidecar (FR-4.2).

### Q6 — When does 031 get its own branch?

This spec is being drafted on the 030 branch alongside 030 v1 wrap. The 031 branch and tasks.md begin once:
- (a) 030 v1 is sealed (SMOKE_REPORT.md + outcome.md + closeout commits pushed), AND
- (b) Hardware orders from handoff doc §10 are placed (camera + lens + filter, Luxeon LEDs ×8 for two pods, ATtiny1616 ×2, **SD-flash-capable flight-mode capture host per Q1(d)**, Commonlands lens inquiry), AND
- (c) The /clarify step on THIS spec has been completed.

**Default**: yes — gate 031 branch creation on those three prereqs. Don't fork while 030 is mid-wrap.

### Q9 — EMI mitigation strategy detail (new 2026-05-12)

FR-1.6 lists seven mitigation defaults. Which ones are actually needed depends on bench EMC measurements (FR-3.3) — over-engineering all seven adds mass + cost, while skipping the load-bearing one (slew-rate control) risks an unflyable airframe. Order of bench-tuning:

1. Slew-rate control alone: gate resistor 100 Ω → 470 Ω, measure FC gyro noise floor at each step. Stop at the smallest R that passes FR-3.3.
2. If (1) doesn't pass: add π-filter on power input. Re-measure.
3. If (1)+(2) doesn't pass: add bulk caps at the FET drain. Re-measure.
4. If still failing: add metal-shield variant of the pyramid. Re-measure.
5. If still failing: revisit FET choice (slower-switching part) or LED-drive current (lower I → less di/dt). Re-measure.
6. If still failing: this is a redesign trigger — pull power off the main pack onto a small dedicated beacon battery (operator-direction-2026-05-12 says **no separate battery**, but if the EMI envelope is unflyable the rule has to bend).

The bench-log entry per FR-5.2 SHALL document which mitigations were actually built into the flight pod and at what test point each was added/removed.

### Q10 — Follow-on field-data-driven feature scope (revised 2026-05-12 per operator-note)

Once US6 (Beacon Test Flight 1) lands footage, a follow-on feature consumes it. The phase-1 raw recordings serve as the input to:

- **Acquisition-time simulations** — feed recorded clips through simulated correlators at various chip rates / code lengths / soft-decision thresholds to find the operating point under realistic photon flux + body-rate envelope.
- **DSP / FPGA-pipeline simulations** — Python golden model of the five-stage FPGA pipeline (handoff §6) runs against recorded clips, characterizes per-stage SNR + latency, validates threshold + IIR-coefficient choices before HDL is committed.
- **AGC-dynamics simulation** — replay 10-bit clips through simulated AGC curves to characterize sun-to-shadow + sun-to-dark transit behavior at body rate; finds the AGC settings the live camera should use.
- **Noise-model calibration** of the 030 simulator's projection front-end.
- **FOV / frame-rate / beacon-intensity / pyramid-dimension validation** — does the design actually deliver what link-budget math predicted? Does the 120° wide-FOV cover real-flight aspects? Is 240 fps enough at observed body rates? Is 4× 1 W radiance enough at observed ranges? Is the 2.5 cm pyramid base mechanically + thermally + optically right?

Candidate scopes:
- (a) **031-noise-cal narrow**: just extracts noise statistics (per-pixel sigma, blob-shape distribution, false-positive rate from clutter). Smallest scope; insufficient for our use case.
- (b) **031-noise-cal + FOV/exposure + acquisition-time + AGC sims**: replays clips through correlator + AGC + FPGA-pipeline sims; produces a quantitative operating-envelope report; informs the next hardware-order decisions (lens FOV adjustment, beacon-power tuning, pyramid-dimension revision). **This is the default scope post-Q5 expansion.**
- (c) **031-noise-cal + first NN A/B**: in addition to (b), trains a small NN variant against the real-clip-augmented simulator and measures whether the controller stays robust to the real-world noise envelope. Too ambitious for one follow-on.

**Default**: (b). The operator note 2026-05-12 elevates this from "noise-cal only" to "full acquisition-time + DSP + AGC + FOV-validation simulation suite". The flight-DSP design decisions (lens, beacon, pyramid, frame rate) all depend on what these sims surface. Decide downstream actions post-(b).

**Risk acknowledgment (operator 2026-05-12)**: "this early phase is proving the beacon→camera chain — likely non-trivial." The link-budget math (handoff §5.6 + camera_considerations.md) predicts comfortable margin, but the optical / mechanical / electrical / timing variables interact in ways the math can't fully model. The follow-on sims working against US6 recordings are how we surface that interaction. Plan for at least one iteration cycle: US6 data → sim insights → hardware/firmware revisions → second flight session → repeat.

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

## Status as of draft

- **Draft created**: 2026-05-10 alongside 030 v1 wrap
- **Revised**: 2026-05-12 — operator firmed beacon (4× 1W, 4-sided pyramid, ~2.5 cm base, unlit cone inboard), code (15-bit Gold), camera (240 fps), recording substrate (dual mode: bench-tethered + onboard SD); added US6 (Beacon Test Flight 1); added FR-1.6 EMI + FR-3.3 EMC bench check; added Q9 (EMI tuning order) + Q10 (follow-on noise-cal scope)
- **/clarify**: not yet run
- **/plan**: not yet generated; gated on /clarify + the three prereqs in Q6
- **Hardware orders**: not yet placed; pending operator decision after /clarify resolves Q1-Q10
- **Bench experiments**: not started; gated on hardware arrival
- **Beacon Test Flight 1 (US6)**: not started; gated on bench experiments + EMC sanity check (FR-3.3)

Until /clarify is run, treat every Q1-Q10 default as "operator's best guess at draft time" — likely to be revised.
