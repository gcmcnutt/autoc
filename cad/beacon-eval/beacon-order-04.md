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
- [ ] **O4-2** **Arducam USB-UVC shield for OV9281 (B0264) ×1** (~$90) — hosts an O4-1 module as a UVC
  webcam for bench/hand-held work (US3). ⚠ UVC path is USB2-capped well below 480 fps — geometry /
  exposure / lens analysis only; rate validation is the FPGA path. Check MIPI flex included (else +1
  flex, verified-bom D8).
  - [ ] received — notes:
- [ ] **O4-3** **Lattice CrossLink-NX Evaluation Board `LIFCL-40-EVN` ×1** (~$300–400, Lattice/Mouser) —
  the analysis FPGA. Sizing: tracker-bank architecture ≈ 20–35k LUT vs 39k cells (s3 correlator datum
  3.5k LUT/unit on MXO2, cheaper here with DSP slices), hardened 4-lane MIPI D-PHY vs a 0.37 Gbps need.
  **T011 RESOLVED 2026-08-08** (Lattice PSG Dec-2020, p.26 — archived at
  [`firmware/beacon-receiver/PSG_2020_I0211K_Rev28.pdf`](../../firmware/beacon-receiver/PSG_2020_I0211K_Rev28.pdf)):
  the EVN board is LIFCL-40-**9BG400C**, expansion = FMC + Raspberry Pi + Pmod + MIPI CSI-2/D-PHY
  headers, PCIe 5G SERDES, USB-B programming — and its only memory is **128 Mbit SPI BOOT FLASH: NO
  onboard DRAM/HyperRAM**. On-chip total is ~0.35 MB (1512k EBR + 1024k LRAM + 240k distributed), so
  the FR-2.5 **6 MB frame ring does NOT fit this board natively**. Consequences:
  (a) the ANALYSIS scope (MIPI bring-up, tracker bank, live detection stats out over USB/UART) needs no
  frame ring — EVN is sufficient as-is; (b) full-frame recording later = **HyperRAM Pmod / FMC memory
  card** on the expansion headers (LIFCL-40 caBGA400 supports LPDDR3/DDR3L-1066 if a custom board ever
  wants it); (c) PSG is Dec-2020 — glance at the current user guide at order for board-rev deltas.
  ⚠ **Toolchain: CrossLink-NX = RADIANT, not Diamond** — new install alongside the MXO2 Diamond flow
  (free license covers LIFCL); plan a hello-LED bring-up before the MIPI work.
  - [ ] received — notes:

## Explicitly NOT in this order
- Thorlabs 10 nm filter (deferred to 040, front-mounted — order-03 C-14 rationale).
- m12lenses PT-02120 fisheye (production-FOV question; analysis runs the 16 mm).
- OG0VA (long-lead upgrade path, inquiry outcome pending).
- SD cards / buck / XT60 (verified-bom D6/D7/D9 — recorder-build era, not analysis).

## Gate context
Single-pixel validation is near its endgame (Option C field-proven; caged copper build + full-chain
characterization next). This order runs in parallel — nothing here blocks or is blocked by the
single-pixel outcome; the UVC camera actually *accelerates* it (lens validation instrument).
