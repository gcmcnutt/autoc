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

- [ ] **O4-1** **Arducam OV9281 1 MP global-shutter MIPI module (B0162) ×2** (~$70 ea) — the FPGA-era
  sensor + spare. Mono, NIR-strong, M12 mount, ROI row-scaling reaches the 480 fps / 320×240 baseline
  over MIPI 2-lane.
  ⚠ Bundled M12 lens usually has an IR-cut — strip/swap; **the C-14 16 mm IR lens screws straight on**
  (bonus: this module IS the trusted focal plane for finishing the lens validation — no Pi-cam surgery).
  - [ ] received — notes:
- [ ] **O4-2** *(DEMOTED to optional 2026-08-08)* **Arducam USB-UVC shield for OV9281 (B0264) ×1**
  (~$90) — hosts an O4-1 module as a UVC webcam for bench/hand-held work (US3). ⚠ UVC is USB2-capped
  well below 480 fps. **The O4-4 Pi Zero 2 W covers this role** (libcamera stills/preview + our own
  capture code) — buy the shield only if a laptop-native UVC webcam proves independently useful.
  - [ ] received — notes:
- [ ] **O4-4** *(NEW 2026-08-08 — CHEAPEST-FIRST, order ahead of O4-3)* **Raspberry Pi Zero 2 W ×1
  (~$15) + Zero 22-pin→15-pin camera adapter flex (~$6)** — the "$15 correlator" experiment: quad
  A53 + NEON ≈ tens of Gops vs ~0.4 Gops needed for even FULL per-pixel correlation at 320×240×480 fps
  (37 Mpx/s; tracker-bank is far cheaper); per-pixel N=31 window state ≈ 11 MB vs 512 MB; CSI ingest
  37 MB/s = trivial for Unicam DMA; Linux jitter is harmless (sensor free-runs = sampling clock;
  drops = erasures the decoder already tolerates). **THE GATE: OV9281 ≥480 fps mode on the Pi stack**
  — mainline driver tops out ~640×400@210 fps; 480 needs Arducam's driver or custom register modes via
  raw capture. 210 fps ceiling = 1.05 samples/chip @ 200 Hz = Nyquist fail → mode-unlock or bust.
  **Run this gate experiment BEFORE buying O4-3.** Flight bonus if it clears: 11 g / 1–2 W / WiFi —
  analysis platform and flyable tracker candidate collapse into one $15 board.
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

## Explicitly NOT in this order
- Thorlabs 10 nm filter (deferred to 040, front-mounted — order-03 C-14 rationale).
- m12lenses PT-02120 fisheye (production-FOV question; analysis runs the 16 mm).
- OG0VA (long-lead upgrade path, inquiry outcome pending).
- SD cards / buck / XT60 (verified-bom D6/D7/D9 — recorder-build era, not analysis).

## Gate context
Single-pixel validation is near its endgame (Option C field-proven; caged copper build + full-chain
characterization next). This order runs in parallel — nothing here blocks or is blocked by the
single-pixel outcome; the UVC camera actually *accelerates* it (lens validation instrument).
