# Beacon Bench — Order 04 (CAMERA-ERA ANALYSIS, opened 2026-08-08)

**What this is**: the camera-phase un-park — base camera + FPGA for analysis, ordered NOW so lead times
overlap the single-pixel validation endgame (operator 2026-08-08: "start ordering base camera and fpga
for analysis"). Source research: [`verified-bom.md`](../../specs/031-beacon-camera/verified-bom.md) §D
(2026-05-18 audit, header still says do-not-order — THIS doc supersedes it for these lines) + the
D1 requirements hardening (2026-08-04) + the LIFCL sizing datum (s3 N=63 = 3524 LUT/82 % of MachXO2-4000).

## Requirements carried in (non-negotiable, operator 2026-08-04)
- **Global shutter** — rolling shutter turns a ~2 px saturated sun into a full-height stripe across the
  beacon track.
- **Manual/fixed exposure + gain** — auto-exposure re-couples the sun to beacon amplitude (the array's
  AGC-windup equivalent); decoder-driven exposure is the end state (journal 2c).
- IR-sensitive mono preferred (no Bayer tax at 850 nm).

## Lines

- [ ] **O4-1** *(SUPPLIER SWITCHED 2026-08-08)* **InnoMaker CAM-MIPI9281RAW-V2 ×2** (~$40–60 ea) — the
  camera. **Vendor documents the gate number outright: "up to 453 fps external trigger stream mode …
  for Rasp Pi 4B/3B+/3B/3A+/CM3/Pi Zero W, Support Bullseye"** — 453 fps ÷ 2.4 fpc = **189 Hz chips,
  essentially the 200 Hz design point.** Specs (operator-captured 2026-08-08): OV9281 mono global
  shutter, 1280×800, 3 µm px, 1/4″ (image area 3.896×2.453 mm), 2-lane MIPI, 8/10-bit RAW, S/N 38 dB,
  DR 68 dB; **includes a 2.8 mm F/2.2 lens (90° D / 72° H, adjustable focus) and cabling**.
  Why ×2: one lives on the bench Pi permanently, one is spare/flight-cage stock (XNANO lesson — keep
  margin). At-order verifies: (a) **Zero-style 22-pin cable included?** (Zero W is listed as supported —
  else add the ~$6 adapter flex); (b) **is the lens mount M12?** (if yes, the C-14 16 mm IR lens screws
  straight on = narrow-field tracker config + the trusted 850 nm focal plane, no Pi-cam surgery).
  *(Arducam B0162 remains the alt if InnoMaker stock vanishes — but its Pi-stack modes are documented
  slower: 320×240@220 fps binned.)*
  - [ ] received — notes:
- [ ] **O4-2** *(DEMOTED to optional 2026-08-08)* **Arducam USB-UVC shield for OV9281 (B0264) ×1**
  (~$90) — hosts an O4-1 module as a UVC webcam for bench/hand-held work (US3). ⚠ UVC is USB2-capped
  well below 480 fps. **The O4-4 Pi Zero 2 W covers this role** (libcamera stills/preview + our own
  capture code) — buy the shield only if a laptop-native UVC webcam proves independently useful.
  - [ ] received — notes:
- [ ] **O4-4** *(NEW 2026-08-08 — CHEAPEST-FIRST, order ahead of O4-3)* **Raspberry Pi Zero 2 W ×1
  (~$15)** (+ Zero 22-pin adapter flex ~$6 ONLY if O4-1 doesn't include one) — the "$15 correlator"
  experiment: quad
  A53 + NEON ≈ tens of Gops vs ~0.4 Gops needed for even FULL per-pixel correlation at 320×240×480 fps
  (37 Mpx/s; tracker-bank is far cheaper); per-pixel N=31 window state ≈ 11 MB vs 512 MB; CSI ingest
  37 MB/s = trivial for Unicam DMA; Linux jitter is harmless (sensor free-runs = sampling clock;
  drops = erasures the decoder already tolerates). **THE GATE — largely PRE-CLEARED by the O4-1 vendor
  (2026-08-08): InnoMaker documents "up to 453 fps" stream mode on Pi (incl. Zero W, Bullseye).**
  453 ÷ 2.4 fpc = 189 Hz chips ≈ the 200 Hz design point; chip rate is sweepable anyway (Nyquist =
  fps/2; emitter = one timer constant). Platform precedent behind it: raspiraw community runs raw
  CSI→RAM at 660–1007 fps on lesser sensors — the driver mode table, never the Pi, is the limiter.
  Remaining experiment = REPRODUCE: sustain the 453 fps mode on the Zero 2 W, measure drop rate over
  10 s against the sensor frame counter, then load the NEON correlator alongside. O4-3 (Zybo) only
  triggers on a practical dead-end (sustained drops, driver rabbit holes). Flight bonus if it lands:
  11 g / 1–2 W / WiFi — analysis platform and flyable tracker candidate collapse into one $15 board.
  - [ ] received — notes:
- [ ] **O4-3** *(NOW CONDITIONAL — order only if the O4-4 fps gate FAILS)* **Digilent Zybo Z7-20
  (Zynq-7020) ×1** (~$300 street; Digilent/Mouser/Amazon) — the analysis FPGA, **CHANGED FROM
  LIFCL-40-EVN 2026-08-08** (operator: not wedded to Lattice; want out-of-the-box camera + memory +
  SD). Why it wins for FPGA-path ANALYSIS:
  - **Pcam port = Pi-compatible 15-pin MIPI CSI-2 FFC** — the Arducam OV9281's Pi-style flex plugs
    STRAIGHT IN (vs the Lattice EVN's proprietary 30-pin CN1 needing an adapter or header-wiring).
  - **1 GB DDR3L onboard** — the FR-2.5 6 MB ring question evaporates (~22 s of full frames);
    **microSD slot native**; dual Cortex-A9 runs Linux → SD recording/streaming comes from the PS
    side nearly free.
  - **Fabric: XC7Z020 = 53k LUT + 220 DSP + 630 KB BRAM** — tracker-bank estimate (20–35k) fits with
    room; the correlator RTL is plain Verilog and ports.
  - **Digilent's Pcam 5C reference design** = working open MIPI D-PHY/CSI-2 RX for this exact port —
    the bring-up starts from a known-good camera pipeline, not from a blank D-PHY.
  - Bandwidth: 480 fps × 320×240 × 10 bit ≈ 185 Mbps/lane on 2 lanes — far inside the port's envelope.
  - Toolchain: **Vivado** (free tier covers 7020; big install). One more toolchain in the shop, but it
    replaces Radiant which would ALSO have been new.
  - Alt considered: **Kria KV260** (~$250, 4 GB, Ubuntu, RPi CSI connector, huge fabric) — even more
    out-of-the-box but heavier ecosystem (PetaLinux/Vitis) and far from flight-representative. Keep as
    fallback if Zybo availability slips.
  - [ ] received — notes:
- **Lattice postscript (research retained, buy dropped)**: the LIFCL-40-EVN investigation is archived —
  PSG + EVN UG in [`firmware/beacon-receiver/`](../../firmware/beacon-receiver/); findings: no onboard
  DRAM (128 Mbit boot flash only, ~0.35 MB on-chip → no 6 MB ring), no SD/USB-C, proprietary 30-pin
  camera CN1, Radiant toolchain. **CrossLink-NX remains the FLIGHT-article candidate** (small, low-power,
  hard D-PHY) — the analysis vendor and the flight vendor are now deliberately decoupled; the tracker
  RTL stays vendor-neutral Verilog so fabric work ports either way.

- [ ] **O4-5** *(NEW 2026-08-08)* **850 nm-filtered lens for the camera** — the O4-1 included 2.8 mm
  F/2.2 lens has NO IR bandpass (72° H of unfiltered daylight). FOV math on the 3.896 mm-wide sensor:
  2.8 mm → 70° H · 3.6 mm → 57° · 6 mm → 36° · 16 mm → 13.9° H · **120° H (the production spec) needs
  f ≈ 1.1 mm — NOT available with an integrated narrow filter**; true 120° stays the Commonlands
  question (spec.md §lens). Plan: (a) reuse the **C-14 16 mm 850 nm lens** for narrow-field/range
  analysis (if O4-1's mount is M12); (b) order the **2.8 mm SKU of the same AliExpress ELP 850 nm
  IR-lens family** (~$10, operator's "recent source") for filtered wide-ish work — ⚠ check CRA/AoI
  blue-shift at field edges vs the ~40 nm passband before trusting corner response (the daylight doc's
  AoI section; wide fields push chief rays off-normal even with the filter behind the lens).
  - [ ] received — notes:

## Research sources (captured 2026-08-08 — the fps-gate and board-selection evidence)

- InnoMaker CAM-MIPI9281RAW-V2 (the 453 fps claim + module page): <https://www.inno-maker.com/product/cam-mipi9281raw-v2/>
  and the GitHub repo: <https://github.com/INNO-MAKER/CAM-OV9281RAW-V2>
- raspiraw high-fps precedent: Robert Elder's 660 fps guide
  <https://blog.robertelder.org/recording-660-fps-on-raspberry-pi-camera/> · Hermann's fork (750/1007 fps)
  <https://github.com/Hermann-SW/fork-raspiraw> · Hackaday writeup
  <https://hackaday.com/2019/08/10/660-fps-raspberry-pi-video-captures-the-moment-in-extreme-slo-mo/>
- Arducam OV9281 Pi-stack modes (320×240@220 binned, mode table): <https://docs.arducam.com/Raspberry-Pi-Camera/Native-camera/Global-Shutter/1MP-OV9281-OV9282/>
- RPi forum thread, OV9281 for high-speed capture: <https://forums.raspberrypi.com/viewtopic.php?t=321017>
- Pi Zero 2 W product page: <https://www.raspberrypi.com/products/raspberry-pi-zero-2-w/>
- Zybo Z7 (conditional O4-3): product <https://digilent.com/shop/zybo-z7-zynq-7000-arm-fpga-soc-development-board/>
  · reference docs <https://digilent.com/reference/programmable-logic/zybo-z7/start>
- Lattice PSG + EVN UG: archived in-repo at [`firmware/beacon-receiver/`](../../firmware/beacon-receiver/)

## Explicitly NOT in this order
- Thorlabs 10 nm filter (deferred to 040, front-mounted — order-03 C-14 rationale).
- m12lenses PT-02120 fisheye (production-FOV question; analysis runs the 16 mm).
- OG0VA (long-lead upgrade path, inquiry outcome pending).
- SD cards / buck / XT60 (verified-bom D6/D7/D9 — recorder-build era, not analysis).

## Gate context
Single-pixel validation is near its endgame (Option C field-proven; caged copper build + full-chain
characterization next). This order runs in parallel — nothing here blocks or is blocked by the
single-pixel outcome; the UVC camera actually *accelerates* it (lens validation instrument).
