# Optical-link outcome — first light through range & occlusion characterization

**Status**: MEASURED, 2026-07-16/17. **Scope**: the first end-to-end *optical* beacon link — gold-code firmware
(ATtiny416 on a real 1S LiPo) → LM3410X boost → 5× L1IZ-0850 @ 50 mA → free air → BPV10NF → MCP6022 TIA →
MCP3201 → StepFPGA s6 correlator — brought up, debugged, calibrated against the datasheet link budget, and
characterized for range and real-hand occlusion. Follows [s6-closed-loop-outcome.md](s6-closed-loop-outcome.md)
(hardwired closed loop); receiver schematic + revisions in
[cad/beacon-receiver/collector-schematic.md](../../cad/beacon-receiver/collector-schematic.md).

## Bottom line

- **Bare-to-bare range @ 50 mA bench current: ~1 m at 1.1 V swing → ~4 m at 100 mV — 1/r² verified**
  (1 m→2 m dropped 3.7×). 7 m solid; **41 ft (~12.5 m) CONFIRMED locking with good alignment** — that is
  ≈7–10 nA photocurrent (~8 mV swing at 1 MΩ, ~a dozen ADC counts): the correlator's processing gain is
  carrying the link. Measured decoder floor **≤ ~10 nA**, far beyond the 6–8 m projection. Scaled: field
  current (×2.3) → ~29 m bare; + collection optics (×10–25 amplitude) → the 100 m goal has real margin.
- Measured flux constant **I_photo ≈ 1.1–1.6 µA·m²/r²** — 2.5–3.5× better than the datasheet-nominal model
  (hot flux bin + conservative low-current scaling). Field current (306 mA) buys ×2.3 range; the planned
  collection optics buy ×10–25 → the 100 m goal remains an optics story, on track.
- **Real-hand occlusion (87 s, 15 events, 100 % recovery)**: flicks ≤~0.7 s ride through in HOLD (lock never
  drops); 1–2 s passes unlock and warm-relock on clear; a ~3 s hold re-locked *through the hand* — near-IR
  diffuses through tissue (~20 dB attenuation), and the decoder's DC tracker + scale-free quality metric
  acquire on the ~5–20 nA that gets through. At 2 m through-hand is marginal (SEARCH↔ACQ) — brackets the
  decoder floor. **A hand is an attenuator, not a shutter: true-LOS tests need an IR-opaque block.**

## Link budget (datasheet model vs bench)

| element | value | source |
|---|---|---|
| L1IZ-0850 radiant intensity | 286 mW/sr @ 1 A, 150° FWHM; ≈ 18–20 mW/sr @ 50 mA | DS190 Table 1 + curve scaling |
| 5 co-aimed LEDs (series, 50 mA) | ≈ 93 mW/sr predicted on-axis | intensities add |
| BPV10NF | 55 µA/(mW/cm²) @ 870 (≈ flat 850–950; peak 940), ±20° half-angle, ~×0.9 at our 0.45 V bias | 81503 |
| model | I_photo ≈ 0.44/r² µA | — |
| **measured** | **I_photo ≈ 1.1/r² µA (1 m, 1 MΩ, 1.1 V swing); 4 m point suggests up to 1.6** | bench 2026-07-17 |

Clip guidance at R_f = 1 MΩ: output clips ~2 µA → stay **r ≥ ~0.7 m** (measured flux) for linear work; at
1 inch the photocurrent is ~mA-scale — every gain saturates (this masked several bring-up bugs).

## Front-end bring-up traps (each cost real time — check these on every build)

1. **Op-amp power**: an unpowered MCP6022 leaves a *passive* PD network that still decodes (margin 9!) via
   photovoltaic mode — everything "sort of works", gain knobs do nothing. First check: VDD/VSS at the pins,
   then pin 2 = VBIAS (virtual short) dark.
2. **PD orientation**: cathode → −IN is load-bearing. Swapped = junction forward-biases → slow-ramp edges,
   output below VBIAS, no decode. Solar-cell test (DMM mV, lit: + terminal = anode) beats package folklore.
   Anode → GND (TI SBOA268/TIDU535 style) or → VBIAS/+IN (zero-bias) both work; we standardized on the
   TI-style for 1:1 reference comparison.
3. **MCP3201 IN− is spec-limited to ±100 mV of VSS** — the original pseudo-diff (IN− = VBIAS) idea is out of
   spec. As-built: IN− = GND; the 0.45 V pedestal is stripped by the decoder's DC tracker.
4. **Trimpot discipline**: measure the actual resistance (end-stop = known value); verify swing tracks R
   linearly — that linearity IS the feedback-health test. Unmarked/mismarked caps and pot-terminal confusion
   produced two false diagnoses.
5. **TIA stability**: at 1 MΩ without sufficient C_f the loop oscillates (signal *shrinks* as gain rises).
   100 pF works (1.6 kHz BW = 8× chip rate); 5–10 pF is the wideband floor on a breadboard.
6. Emitter **afterglow** (C1 bleeding through the string, mathematically a 1/t decay) dominates the trailing
   edge at 50 mA (~5 ms to 90 %); 6× shorter at field current; the matched filter is indifferent (margin 9
   with fully rounded edges).

## Decoder behavior on the optical link

- Strongest optical lock recorded: **margin 9, corrB 29.4k** (0.55 V swing station), dark level parked
  exactly at VBIAS. rateB nominal with the emitter free-running on battery.
- Occlusion event table (60 s capture, `host/results/hand_occlusion.log`): 9 fast flicks — margin dips to
  1–4, **zero lock drops**; 4 slow passes — 0.3–1.7 s unlock, warm relock; 1 long hold — relock through
  tissue. Matches the emitter-commanded dropout ladder (HOLD ≈ 2 periods, warm relock ≲ 1 period).

## Next steps

1. **Split boards** (done in principle at this checkpoint) → soldered receiver on copper-clad next: kills
   breadboard strays, edge-spike crosstalk (shared-ground di/dt), and the pot/cap fragility.
2. **UVLO is now urgent**: the emitter runs on a real LiPo with NO firmware cutoff (T027–T031) — the boost
   will pull the cell to the 2.7 V hardware floor. Supervised use only until implemented.
3. Range curve with the harness (`recovery_sweep.py` needs the mEDBG command link — battery-standalone TX
   means occlusion tests are physical for now; re-run the commanded ladder when TX is back on USB).
4. Optics stage: collection lens on the RX (×10–25 range) → the 100 m link-budget test (O2-13 filter for
   daylight).
