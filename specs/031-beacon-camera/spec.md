# Feature Specification: 031 Beacon-Camera Optical Perception — Phase 1

**Feature Branch**: TBD (likely `031-beacon-camera` when this is unparked; currently being drafted inside the `030-tracker-mode` working branch alongside the 030 v1 wrap)
**Created**: 2026-05-10
**Status**: DRAFT — spec + clarify only, no plan/tasks yet. Bench experiments will iterate this heavily once hardware lands.
**Input**: [docs/aircraft_tracker_handoff.md](../../docs/aircraft_tracker_handoff.md) (the architectural handoff doc; all decisions there are the source of truth for this phase-1 spec until they're re-litigated by bench evidence). User direction 2026-05-10: focus 031 on **LED + pyramid design**, **camera + filter**, **raw recording for sim analysis** — defer FPGA/DSP/wire-protocol to follow-on phases (031-fpga, 031-protocol).

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

## Overview

The 031 phase-1 deliverable is a **bench-runnable beacon-camera setup that records raw video of two coded-IR beacons at flight-relevant ranges and dynamics**. Specifically:

- **A pair of LED beacon pods** sized for wingtip mount on an RC aircraft (≤4 g each), built from off-the-shelf LEDs arranged on a 3D-printed multi-facet pyramid for ≥270° azimuth coverage, with an integrated MCU driving them with orthogonal Gold codes per the handoff doc §5.
- **A single mounted camera** with a wide-angle (≥120° H FOV) M12 lens, 850 nm bandpass filter integrated in the lens housing, and a global-shutter mono sensor at ~320×240, capturing frames at 100-200 fps.
- **A raw-frame recording capability** — capture raw sensor output (or near-raw, post-MIPI-unpack) to disk over USB or SDIO, time-tagged, with synchronized metadata (LED MCU code phase if accessible, plus ambient/sun conditions logged manually per session). This is the bench substrate that feeds the **simulator's noise model calibration** and the **FPGA pipeline's HDL co-sim test vectors** in follow-on 031 specs.

This phase is **deliberately not** producing a working real-time `(x, y, CEP)` front-end. That's 031-fpga's job. Phase 1 is the optical front + recording substrate.

## Stakeholders

- **Operator (single-developer project)** — designs, builds, benches, iterates. Reads the handoff doc as source-of-architecture; this spec is the working memory of "what was decided and why" once we start.
- **030's evolved controller (downstream consumer)** — already trained against synthetic beacon projections. Treats 031's outputs (eventually) as the same `(x, y, CEP)` wire contract. Phase-1 work doesn't touch 030's controller; phase-2 (FPGA) and phase-3 (integration) do.
- **Future maintainer** — assumes nothing about the project except what's in the repo. Reads this spec to understand why specific LEDs/lens/sensor/filter parts were picked and what bench-result iterations changed those decisions.

## User Stories

Phase-1 has bench-developer-facing user stories only. No end-user-facing UI changes.

### US1 — Build and verify a single beacon pod (priority: P1)

**As** the bench operator,
**I want to** assemble a 3-die LED pyramid on a small driver board with code-modulated output,
**so that** I can verify the optical emission pattern, mass budget, and code orthogonality before committing to a paired-beacon flight build.

Acceptance: a single pod emits the canonical 7-bit Gold code at the canonical chip rate, draws within the design power budget, weighs within the design mass budget, and produces a visible-on-scope modulated NIR signal at >1 m distance through a smoke-glass filter (to confirm code is recoverable visually).

### US2 — Build the paired-beacon target rig (priority: P1)

**As** the bench operator,
**I want to** integrate two pods on a small target rig (workbench mock-up of wingtips at correct spacing, then onto a real RC airframe for outdoor bench),
**so that** I can collect realistic two-beacon footage for simulator + downstream tooling.

Acceptance: two pods running simultaneously with code A and code B respectively, mounted at hb1-wingtip spacing (~0.9 m), powered from a representative airframe DC bus, surviving moderate vibration. No firmware-level concerns yet — just photons in space.

### US3 — Build the receiver: camera + lens + filter, hand-held first (priority: P1)

**As** the bench operator,
**I want to** mount the chosen global-shutter sensor with the chosen wide-angle NIR-bandpass M12 lens on a dev board capable of capturing raw frames,
**so that** I can collect real footage of the paired-beacon rig at varied range, angle, and ambient.

Acceptance: 320×240 (or higher) raw mono frames captured to disk at ≥100 fps for a configurable duration, with frame timestamps. Sun/ambient/range manually logged. Existence of two distinguishable IR blobs in the recorded frames at design ranges confirms the optical chain works end-to-end.

### US4 — Raw-frame recording substrate for downstream (priority: P1)

**As** the bench operator (and future 031-fpga / sim-tuning teams),
**I want to** record N-second raw-frame clips with provenance metadata (sensor settings, exposure, gain, timestamps, env conditions),
**so that** the simulator's noise model can be calibrated against real footage, and the FPGA Python golden model can be validated against the same footage offline.

Acceptance: a recorded clip is loadable by a Python utility that produces `numpy` arrays + a JSON sidecar with the per-clip metadata. The simulator (currently fully synthetic, [src/eval/camera_projection.cc](../../src/eval/camera_projection.cc)) gets a hook to ingest a real clip as a noise-source reference in a follow-on calibration pass (NOT this spec — 031-fpga or a sim-tuning sub-feature).

### US5 — Side-by-side: range, ambient, body-rate scenarios (priority: P2)

**As** the bench operator,
**I want to** run a set of canonical bench-experiment scenarios (close range / far range, indoor / outdoor / direct sun, stationary / hand-induced rotation / fast pan to simulate body rate, dim ambient / bright ambient),
**so that** the operating envelope (acquisition range, sun-rejection performance, dynamics handling) is grounded in real measurements rather than the handoff doc's link-budget estimates.

Acceptance: a written bench-experiments log notes each session's setup + recorded-clip references + qualitative observations. Quantitative analysis (SNR floor estimation, etc.) is a follow-on, **not phase-1 acceptance**.

## Functional Requirements

### FR-1 LED beacon pod

#### FR-1.1 Optical emission pattern

Each pod SHALL include 3-4 LEDs arranged on facets of a 3D-printed pyramid/prism so the combined emission pattern covers ≥270° in azimuth. (Source: handoff §4.4.) The specific LED part is **Lumileds Luxeon IR Compact 850 nm** (primary; OSRAM SFH 4715AS as drop-in mechanical+electrical alternate). Per-die nominal emission is 130° beam-width at 1.3 W radiant @ 1 A drive (T_j = 25 °C); three dies at 120° azimuth spacing give ≥270° coverage with overlap at the seam angles.

#### FR-1.2 Driver topology

Each pod SHALL implement the following power + drive chain (per handoff §5.1):

```
airframe DC bus (7.4 V / 11.1 V)
  ├── buck regulator → V_LED (~3.5-4.5 V) — headroom over LED Vf ~1.7-2.0 V
  └── 3.3 V LDO → MCU
         │
         │ digital code stream @ 1 kHz chip rate
         ▼
     logic-level N-FET (one per facet) → LED + sense resistor → GND
```

Component picks (handoff §5.2, all rebench-able):

| Block | Part | Notes |
|---|---|---|
| Buck regulator | TPS62933 (primary) / MP2451 (alternate) | 3 A, ~95% η, ~0.3 g |
| MCU | ATtiny1616 (primary) / RP2040 (alternate) | ATtiny is mass-friendly; RP2040 simpler to flash |
| FET | DMN2058U logic-level N-MOSFET, R_DS(on) <50 mΩ | One per facet for independent control |
| Sense resistor | 0.1 Ω 1% 1206 | 100 mV @ 1 A drive, ADC-friendly |
| LED | Lumileds Luxeon IR Compact 850 nm | See FR-1.1 |

Driver SHALL be **integrated FET drive from MCU GPIO** — no separate gate driver IC at <5 A drive. Saves mass + parts (handoff §5.2).

#### FR-1.3 Gold-code generation (the modulation pattern the FPGA correlates against)

Each pod's MCU SHALL generate a **7-bit Gold code** modulating the LED string. The code generation contract:

- **Chip rate**: **DERIVED from the selected camera frame rate**, NOT independently locked. The design relationship is "camera gets ≥5 frames per chip" (handoff §5.3) to give the matched filter comfortable oversampling and natural robustness to ±100 ppm LED-MCU↔camera clock drift (handoff §5.4). Concretely:
  - Camera 200 fps → chip rate ≤ 40 Hz (≥5 frames/chip); handoff doc's headline 1 kHz value implicitly assumed sensor running at a much higher mode than the canonical 320×240 binning produces. Verify on bench.
  - Camera 100 fps → chip rate ≤ 20 Hz.
  - Hard floor (Nyquist): chip rate ≤ camera_fps / 2; below that the correlator can't see distinct ON/OFF chips at all. Operate with comfortable margin (5×) to ride out frame drops + clock drift.
  - The MCU firmware SHALL accept the chip rate as a build-time constant (or runtime register for the prototype), not hard-code a single value — pinning it lets us re-tune once Q-CAM (the camera-selection open question, see Q7 below) is resolved.
- **Code length**: 7 chips → one full code period = 7 × chip_period. Short codes preferred over long ones because the track may be lost mid-correlation during high-rate maneuvers, degrading the input ring with zero-fills (handoff §4 dynamics analysis). Lengthening to 11 or 15 chips is a future option if SNR is the bottleneck, but the design first attacks SNR via chip-per-frame oversampling, not code length.
- **Code source**: Gold-code pair generated from a single LFSR-pair pre-computed once per code-ID, baked into the MCU as a 7-byte lookup table. Cross-correlation between code A and code B SHALL be ≤ −15 dB (verified offline at code-design time; pod just plays back the LUT).
- **Code ID assignment**: code A vs code B per pod, configurable via either (a) MCU EEPROM written at flash time, or (b) a wire jumper read at boot (handoff §5.5).
- **Duty cycle**: ~50% nominal (3-4 ON chips per 7-chip code average; exact value falls out of the LFSR-derived Gold-code bit pattern). Peak LED current MAY exceed CW rating by 2-3× during ON chips since average is half — keeps photons high without thermal damage (handoff §5.3, §5.6).
- **Phase**: free-running. No sync to camera, no sync between the two pods. The matched filter on the FPGA side tolerates the natural offset (handoff §5.4).
- **MCU workload**: one timer interrupt + 7-byte LUT + GPIO toggle. ~5 mW MCU power. Code generation is not the load-bearing complexity here; the driver chain is (handoff §5.3 last paragraph).

#### FR-1.4 Mass + power budget

Each pod's mass SHALL not exceed **4 g** including LEDs (3-4 × ~30 mg dies), MCU + driver electronics, pyramid mount, decoupling, and wiring tails. Total system mass for two pods + camera (FR-2) + recorder (FR-4) is NOT a flight-weight constraint in phase 1 — the recorder can be ground-tethered — but pods SHALL be flight-weight from the start so they don't need redesigning when integration moves to flight.

Each pod SHALL be powered from a representative airframe DC bus (7.4 V from a 2S LiPo, or 11.1 V from 3S) via the buck regulator in FR-1.2. Pod power consumption SHALL be characterized at the canonical 50% duty operating point: nominal ≤500 mW average, ≤2 W peak during ON chips (handoff §5.6 closes the link-budget math at this operating point).

#### FR-1.5 Orthogonality verification

The two pods together SHALL produce two orthogonal Gold codes whose cross-correlation, when recovered by demodulating a bench-camera clip with both pods illuminated simultaneously, is **≤ −15 dB** (handoff §5.5). Verification path:

- (a) **Scope-level**: capture one pod's LED-anode waveform on a fast scope, confirm the 1 ms chip rate + 7-chip period + correct Gold-code pattern;
- (b) **Photodiode-level**: pin a fast photodiode at 1 m, capture both pods' combined NIR signal, demodulate offline with both correlator templates, confirm A and B peak above each other's noise floor;
- (c) **Camera-level**: record a bench clip via FR-4 with both pods, demodulate the per-pixel intensity time-series at each blob, confirm both codes recoverable with the expected cross-correlation floor.

(a) is mandatory pre-shipment; (b) is a useful sanity check; (c) is what 031-fpga's Python golden model will reproduce against the same recorded clip.

### FR-2 Camera + lens + filter

**FR-2.1** The camera SHALL be a global-shutter mono sensor at VGA-or-better resolution capable of ≥100 fps at the captured resolution. Primary choice: **ST VD55G1**; backup: **OmniVision OV9281** (Arducam B0162 module as the easy bring-up path). Rolling-shutter sensors are explicitly disqualified (handoff §3.2).

**FR-2.2** The lens SHALL be M12-mount, NIR-corrected, ~120° H FOV, F/2.0-or-faster, with **integrated 850 ± 10 nm bandpass filter** in the housing. Sourcing: Commonlands (`contact@commonlands.com`) for custom config; generic 145° fisheye M12 (e.g., m12lenses.com PT-02120) for early prototyping before the Commonlands order arrives.

**FR-2.3** The capture pipeline SHALL produce 320×240 (or sensor-native cropped equivalent) 8-bit mono frames at the **maximum reliable rate the chosen capture host (Q1) sustains** — handoff §6.1 design point is ≥100 fps, ideally 200+ fps to give the chip-rate-per-frame oversampling per FR-1.3, but the actual achieved fps is one of the phase-1 bench measurements (Q7). Higher resolutions remain a configuration option per handoff §6.1 — not the default.

**FR-2.4** The captured frames SHALL be time-tagged with monotonically increasing per-frame timestamps at microsecond resolution (typically the sensor or FPGA's own frame-counter + a derived us clock).

### FR-3 Beacon-emission verification

**FR-3.1** Bench-side, the operator SHALL be able to verify that the LED pyramid's emission pattern matches the design's ≥270° coverage by rotating a single pod in front of the camera at fixed distance and recording received signal vs. azimuth. (This is part of the US5 scenario set.)

**FR-3.2** Code orthogonality SHALL be bench-verifiable: with both pods on simultaneously at distinct codes, a recorded clip + Python-side demodulator SHALL show two recoverable signals with the expected cross-correlation floor.

### FR-4 Raw-frame recording substrate

**FR-4.1** The recording system SHALL stream raw 320×240 mono frames + per-frame metadata to a host computer (USB / SDIO / Ethernet-tethered acceptable in phase 1 — flight-weight onboard storage is NOT a phase-1 requirement) at the FR-2.3 frame rate without dropping frames for clips up to 60 seconds.

**FR-4.2** Each recorded clip SHALL produce two artifacts: (a) the binary frame stream (raw 8-bit pixels or a vendor-friendly container e.g. raw `.bin` with documented stride, or `.npy` numpy array), and (b) a JSON sidecar with per-clip metadata: clip-id, ISO-8601 start time, sensor model + firmware rev, lens + filter spec, capture resolution + fps, sensor exposure + gain settings, ambient-light qualifier (manually logged: indoor / outdoor / cloudy / direct-sun), range-to-target qualifier (manually logged or measured), pose qualifier (stationary / hand-panned / on-aircraft), notes free-text.

**FR-4.3** A Python utility SHALL load a recorded clip into a `numpy.ndarray` of shape `(n_frames, height, width)` with `dtype=uint8` plus a `dict` parsed from the JSON sidecar. This is the canonical handoff interface to all downstream tooling (sim noise calibration, FPGA Python golden-model test vectors, hand-rolled blob detector experiments).

### FR-5 Operating-envelope characterization (phase-1 bench observations)

**FR-5.1** A set of canonical bench scenarios SHALL be defined (close-range static / far-range static / hand-panned / outdoor-direct-sun / outdoor-cloudy / dim-indoor — exhaustive list TBD during execution but pre-agreed before the first session). Each scenario produces a recorded clip per FR-4.

**FR-5.2** A written bench-log entry per session SHALL capture clip-id, scenario, observed qualitative behavior (beacon visible / blooming-on-sun / blob-mass-vs-range / motion-blur-onset-rate / etc.), and unresolved questions. The bench log is **the primary phase-1 deliverable**; it's the source of the iteration loop for the simulator's noise model and the FPGA pipeline's threshold tuning in follow-on specs.

## Non-Functional Requirements

### NFR-1 Mass + power (carried forward from handoff §2)

- Per-pod mass: ≤4 g.
- Per-pod power: nominal ≤500 mW typical, ≤2 W peak during pulse cadence.
- Camera + lens mass (for eventual flight, not phase-1 enforcement): 2-3 g target.
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
- **Flight integration**: 031-integration. On-aircraft mounts, in-flight power, vibration tolerance, EMI mitigation. Phase 1 stays bench-tethered.
- **Sensor noise-model calibration of the simulator**: orthogonal sim-tuning sub-feature (probably extends 030's [src/eval/camera_projection.cc](../../src/eval/camera_projection.cc)). Consumes phase-1's recorded clips but is itself controller-training adjacent, not optical-front-end work.
- **Multi-camera (3× cameras at 0°/+60°/-60°)**: handoff §3.5 + §9.1. Architecturally provisioned by the per-camera I²C-address scheme — but phase 1 builds one camera only.
- **Event-camera (Prophesee IMX636-class) alternative**: handoff §3.6 + §9.2. Future variant experiment, not phase-1 critical path.
- **Body-rate compensation feed-forward**: handoff §3.7 + §5.4. Optional; phase 1 records raw without body-rate metadata.
- **Sync (RF sync pulse from tracker to target)**: handoff §5.4. Asynchronous is the canonical design; revisit only for very long codes.

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
| LED arrangement: 3-4 dies on 3D-printed pyramid | handoff §4.4 | ≥270° coverage with 130° dies at 120° azimuth spacing + overlap |
| Lens: Commonlands M12 NIR-corrected ~120° F/2.0 w/integrated 850 ± 10 nm filter | handoff §4.2 | Integrated filter saves separate alignment + mass; <8 µm focal shift visible→NIR |
| Sensor: ST VD55G1 primary / OV9281 backup | handoff §4.1 | Both global-shutter, mono, NIR-friendly; OV9281 ecosystem (Arducam B0162) is the easy bring-up |
| Code length: 7-bit Gold | handoff §5.5 | Short code → robust to mid-correlation track loss; cross-corr ≤ −15 dB |
| Chip rate: ~~1 kHz~~ **derived from camera fps, see Q7** | bench-deferred | "≥5 frames per chip" is the design relationship; absolute value falls out of camera mode + binning + frame rate, set on bench |
| Async LED-camera clocks | handoff §5.4 | Matched filter tolerates ±100 ppm offset; sync adds complexity for no first-order benefit |

## Open Questions (resolved during phase-1 execution OR escalated to follow-on)

These are explicit unknowns at draft time. The /clarify step is expected to either firm them up before plan, or accept them as bench-discovered.

### Q1 — Phase-1 capture-host platform

Should the dev capture host be a:
- (a) Standard Linux PC with USB-tethered Arducam B0162 (proven path, fastest bring-up, no FPGA needed in phase 1);
- (b) The Lattice CrossLink-NX-EVN dev board acting as a raw-frame USB streamer (preempts 031-fpga work, but FPGA gateware to stream raw frames is non-trivial);
- (c) A Raspberry Pi CM4 / Pi 5 with the OV9281 module (middle ground, less FPGA learning curve)?

**Default**: (a) — Arducam B0162 + USB on Linux PC. Lowest cost-to-first-frame. Phase 1 is about photons + filter + LED, not FPGA. The dev-board path enters in 031-fpga.

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

### Q4 — Pod mechanical mount (operator-resolved 2026-05-10)

**Resolution**: 3D-printed custom mount per pod, sized for the hb1 wing-tip max-chord profile.

Mount geometry:
- **Base**: matches the flat wing-tip surface at max chord, **~2.5 cm wide**. Mounts directly to the wing-tip with double-sided tape or thin adhesive (reversible for early bench, glue-up once design freezes).
- **Pyramid**: **~2 cm tall**, multi-facet for the FR-1.1 ≥270° emission pattern. 3-facet (triangular pyramid, 3 LED dies at 120° azimuth) is the baseline shape; 4-facet (square pyramid) is an alternate if the LED-die layout benefits from a 90°-spacing pattern.
- **Cable tail**: small slot/channel for the 2-wire bus + 2-wire MCU-data tether running to the airframe DC bus.
- **Print orientation**: facets up so LED-mount surfaces print without overhangs.

For bench-only sessions before the airframe integration:
- (S1-S7 of Q3) — both mounts on a wood / foam jig at **hb1 wing-tip spacing (~0.9 m)** to replicate the eventual flight geometry.
- (S9) — variable spacing for the paired-beacon proximity case (push pods together until the camera sees them as one cluster at 50 m).
- A static airframe mount (Q4-b variant) is deferred to 031-integration; phase 1 stays on the wood jig.

The 3D print files belong in the 031 spec dir alongside this spec when fab starts.

### Q5 — Acceptance bar for "phase 1 done"

Two reasonable bars:
- (a) **Minimum**: all 9 bench scenarios captured and loadable in Python, with one written bench-log entry per session. No quantitative analysis required.
- (b) **Stretch**: minimum + qualitative Python notebook for each scenario showing the two beacon blobs (or sentinel cases), confirming the optical chain works end-to-end. No FPGA-pipeline replication required — just "are the photons reaching the sensor in a recoverable form?"

**Default**: (b). The qualitative notebook is cheap and protects against shipping unusable clips to 031-fpga.

### Q7 — Camera operating mode + derived chip rate (the FPS↔chip-rate loop)

Chip rate is downstream of camera frame rate (FR-1.3 + handoff §5.3 "≥5 frames per chip"). The actual frame rate is a function of:

- **Which sensor**: VD55G1 vs OV9281 have different max fps at any given resolution / bit-depth / lane config.
- **Resolution / binning mode**: handoff §6.1 picks 320×240 to fit per-pixel state in tight BRAM at FPGA time, but the same binning on a bare sensor may run faster or slower than the un-binned mode.
- **Sensor settings**: exposure time, gain mode, AEC/AGC behavior.
- **Capture host capability**: USB streaming bottlenecks (Q1) may cap effective recording fps below the sensor's max.

The handoff doc's headline "1 kHz chip rate @ 200 fps camera = 5 frames/chip" should be read as **the design's reasoning template, not a locked operating point**. The phase-1 bench job is to find the actual feasible (fps, chip_rate, frames_per_chip) tuple that survives the capture pipeline (Q1) and the LED/MCU timing.

**Default**: defer to bench. First bring-up captures at "whatever fps the chosen capture host gets reliably", LED MCU is built with runtime-configurable chip rate, and we pick chip rate to give ≥5 frames/chip with comfortable margin. Quantitative sweeps (chip rate vs SNR vs body-rate-induced loss) are a follow-on experiment — likely the first quantitative phase-1 deliverable.

**Implication on FR-1.3 / FR-2.3**: both clauses now say "phase-1-bench-derived" not "1 kHz / 200 fps". The wire-format contract (FR in 031-protocol future spec) is unaffected — chip rate is invisible above the FPGA's correlator interface.

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

**Implication on the spec**: phase 1 doesn't lock fps or exposure — just records what the bench achieves under each ambient scenario. The 200 fps "design point" stays as the **target operating mode** but actual fps achieved is one of the phase-1 deliverables and likely varies session-to-session.

### Q6 — When does 031 get its own branch?

This spec is being drafted on the 030 branch alongside 030 v1 wrap. The 031 branch and tasks.md begin once:
- (a) 030 v1 is sealed (SMOKE_REPORT.md + outcome.md + closeout commits pushed), AND
- (b) Hardware orders from handoff doc §10 are placed (Lattice board, Arducam OV9281, VD55G1 sample, Commonlands inquiry, Luxeon LEDs, ATtiny1616), AND
- (c) The /clarify step on THIS spec has been completed.

**Default**: yes — gate 031 branch creation on those three prereqs. Don't fork while 030 is mid-wrap.

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

Phase 1 of 031 (this spec) does NOT touch any 030 code. The deliverable is a recording substrate + bench artifacts. The first place 031 will touch 030 is the noise-model calibration sub-feature, which extends the simulator's projection front-end to consume real recorded clips for sensor-noise statistics. That sub-feature is its own spec, not part of this one.

---

## Status as of draft

- **Draft created**: 2026-05-10 alongside 030 v1 wrap
- **/clarify**: not yet run
- **/plan**: not yet generated; gated on /clarify + the three prereqs in Q6
- **Hardware orders**: not yet placed; pending operator decision after /clarify resolves Q1-Q6
- **Bench experiments**: not started; gated on hardware arrival

Until /clarify is run, treat every Q1-Q6 default as "operator's best guess at draft time" — likely to be revised.
