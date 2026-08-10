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

- [ ] **O4-1** *(SUPPLIER SWITCHED 2026-08-08)* **InnoMaker CAM-MIPI9281RAW-V2 ×2** (**~$39 ea**,
  vendor-page price per operator 2026-08-10) — the
  camera. **Vendor documents the gate number outright: "up to 453 fps external trigger stream mode …
  for Rasp Pi 4B/3B+/3B/3A+/CM3/Pi Zero W, Support Bullseye"** — 453 fps ÷ 2.4 fpc = **189 Hz chips,
  essentially the 200 Hz design point.** Specs (operator-captured 2026-08-08): OV9281 mono global
  shutter, 1280×800, 3 µm px, 1/4″ (image area 3.896×2.453 mm), 2-lane MIPI, 8/10-bit RAW, S/N 38 dB,
  DR 68 dB; **includes a 2.8 mm F/2.2 lens (90° D / 72° H, adjustable focus) and cabling**.
  Why ×2: one lives on the bench Pi permanently, one is spare/flight-cage stock (XNANO lesson — keep
  margin). At-order verifies: (a) **Zero-style 22-pin cable included?** (Zero W is listed as supported —
  else add the ~$6 adapter flex); (b) **is the lens mount M12?** (if yes, the C-14 16 mm IR lens screws
  straight on = narrow-field tracker config + the trusted 850 nm focal plane, no Pi-cam surgery);
  (c) **453 fps mode geometry — CROP or SKIP/BIN?** FOV at speed hinges on it: the 2.8 mm lens gives
  ~**72° H × 48° V** full-field (pinhole math 69.7°/47.3°/78.8° D; vendor 90° D/72° H with the −17 %
  distortion), 0.056°/px at full res — the beacon is sub-pixel at 100 m at any binning — but a center
  **crop** to 320×240 collapses the field to ~20° H × 15° V at speed, while skip/bin keeps the full
  ~72° at 0.23°/px. **And skip ≠ bin for a SUB-PIXEL source** (sharpened 2026-08-09): **bin** sums the
  groups — full field, gap-free, all beacon photons land in some super-pixel (background ×N, SNR −√N,
  correlator absorbs it); **skip** reads 1-of-N photosites and DISCARDS the rest — a sub-pixel beacon
  falling between sampled sites VANISHES, and would blink in/out as the target drifts across the
  sampling grid (code corruption), unless the PSF is deliberately defocused to span the skip pitch.
  Ranking for the beacon: **bin > crop > skip**. Real modes are often chains (bin-2 + skip-2) — read
  the actual register config of the 453 fps mode on arrival, not just its output resolution.
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

- [ ] **O4-5** *(NEW 2026-08-08; REVISED after reading the InnoMaker lens drawing)* **850 nm filtering
  for the camera**. The included 2.8 mm F/2.2 lens per the vendor drawing
  (`github.com/INNO-MAKER/CAM-OV9281RAW-V2` → Lens drawing/Lens.png, mirrored in scratch): design
  format **1/2.7″** with D=148°/H=118° ON THAT FORMAT — **the OV9281's 1/4″ sensor crops this to
  ~72° H** (the product page's number; sensor sets FOV, not the glass). Drawing confirms: **M12
  thread** ✓ (the C-14 16 mm 850 nm lens interchanges directly = narrow/range config), **"4G+IR",
  λ 400–1100 nm** = IR-corrected NOT IR-cut (works at 850, just unfiltered), and **CRA = 10°** — the
  quiet win: a **12.5 mm 850 nm disc dropped BEHIND this lens** sees near-normal incidence over the
  whole field → 40 nm-passband AoI shift negligible. Plan: (a) 16 mm 850 lens swap-in for
  narrow-field/range; (b) ~~Quanmin disc behind the included 2.8 mm~~ — **CLARIFIED 2026-08-10: NO
  filter discs on hand** (the "existing IR filter" = the 16 mm's integrated one). The filtered-wide
  config comes from the **AliExpress 2.8 mm 850 nm-integrated SKU** being ordered; the Quanmin disc
  pair (~$12) is now an OPTIONAL flexibility buy (filters any future bare lens incl. the InnoMaker
  stock 2.8 mm). **True 120° H on
  OV9281 still needs f ≈ 1.1 mm = the Commonlands custom question (spec.md), unchanged.**
  **DECIDED 2026-08-09 (operator)**: go Pi + included 2.8 mm + existing IR filter stack; ALSO order a
  **2.8 mm-or-slightly-longer 850 nm-integrated SKU** from the ELP family (~$10) as the dedicated
  filtered-wide lens. **FWHM answer (asked 2026-08-10): it will be 30–60 nm class — integrated-10 nm
  M12 lenses do not exist** (10 nm = laser/VCSEL disc market), and per
  [`camera-era-knobs.md`](../../specs/031-beacon-camera/camera-era-knobs.md) §5 that is the RIGHT width:
  30–40 nm is matched to the LED's 30 nm line; 10 nm would discard ~70 % of our own signal and
  blue-shifts off-band at wide-field CRA anyway. Record the vendor's admitted FWHM on arrival.
  Flight architecture note: **two of these birded outward** (~120–125° H combined, overlap parallax) is
  the current flight-optics shape — FOV soft, co-designed with training (knobs §6). Tracked as tasks A8. NB the physics tether (see 041 note below): at fixed F/#, FOV and pupil trade —
  2.8 mm F/2.2 = 1.27 mm pupil ≈ **40× less point-source light than the 16 mm F/2** → the wide lens is
  inherently a **short-range (~15 m-class) acquisition/bench device**; the 16 mm is the range camera.
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

## THE CART — final, ready to place (2026-08-10; tasks A8-1)

| Vendor | Items | ~$ |
|---|---|---|
| **DigiKey** | ATTINY412 SOIC-8 ×10 (order-03 C-26) · *optional*: ATTINY416-XNANO spare ×1 | 6–18 |
| **InnoMaker** (inno-maker.com / their Amazon store) | **CAM-MIPI9281RAW-V2 ×2 @ ~$39** (O4-1) | ~78 |
| **AliExpress** (M12 commodity market — lenses only; cameras stay InnoMaker for the DRIVER) | **Lens kit**: 2.8 mm 850 nm-integrated ×1–2 (O4-5; record vendor FWHM claim, expect 30–60 nm) + **1.7 mm fisheye ×1** (~$10–15 — the single-camera ~110–120° topology experiment: 0.85 mm pupil = only ×2.2 signal penalty vs a bird-pair camera; unfiltered — characterize first, disc-filter later if it earns it). **Candidates ranked 2026-08-10**: (1) **Arducam M12 1.7 mm 1/2.5″ ultra-wide** (~$10–15, Amazon/Pi Hut, ships with holder) — BUY FIRST; note "IR-corrected" is OPTIONAL for monochromatic-850 + corrB-focus use, only a built-in IR-cut disqualifies; (2) Vision Datum VT-LEM01720FE-i (spec'd, ~$20); (3) FOCtek M12-1.7IR 8MP via RMA (~$35, IR-corrected) / Commonlands CIL290 2 mm 190° (~$50, IP67) as the upgrade path if the fisheye topology wins the training fork | 20–35 |
| **Pi reseller** (PiShop/Adafruit/CanaKit) | **Pi 3A+ ×1** (O4-4; SWITCHED 2026-08-10 — Zero 2 W scalped everywhere incl. AliExpress. 3A+ = same BCM2710 family at 1.4 GHz (faster), explicitly on InnoMaker's supported list, STANDARD 15-pin CSI = cable question dissolves. Zero 2 W stays the flight-mass option when supply normalizes; Pi 5 dual-CSI = the bird-pair host candidate. **CONFIRMED 2026-08-10: 3A+ is the ground-test unit**; storage = its own microSD for boot/capture, plus the on-hand microSD breakouts ×2 (order-01) if extra logging paths are wanted) | ~25 |
| *Optional, Amazon* | Quanmin 12.5 mm 850 nm disc pair (filters any bare lens; NOT on hand; 10.4 mm variant exists — **measure the holder bore first**) | ~12 |

**Barrel-label "IR" decoder (2026-08-10)** — three meanings, two-second test: (1) **IR-corrected**
(most common): day/night confocal design, NO filter, transmits 400–1100 nm — 850 AND 940 pass fine
(glass goes far beyond; 1100 is the SENSOR's limit, not the lens's) — look through it: room looks
normal; (2) **850-bandpass integrated** (our 16 mm): opaque to the eye; (3) **"IR CUT"**: built-in
650 nm cut for color cams — the only poison type, usually says CUT. Confocality band (546→850/940)
irrelevant for monochromatic corrB-focus use.

**Filter-disc mounting (2026-08-10)**: discs drop INSIDE the M12 holder onto its internal step (where
factory IR-cuts sit) — retain MECHANICALLY (O-ring / foam ring / cut plastic ring), zero adhesive. If
bonding is unavoidable: 3 tiny edge dots of canopy glue (PVA, no fog) or slow epoxy. **NEVER
cyanoacrylate near optics** — CA vapor frosts glass while curing (blooming). 1 mm glass shifts focus
~0.3 mm (thread absorbs it). Fisheye caveat: rear chief rays hit the disc at 25–35° at field edges →
passband blue-shift on top of the 52 % corner illumination — characterize unfiltered first.

**Total ≈ $110–145.** Post-order: the 22-pin Zero flex (+$6) only if InnoMaker's included cabling
doesn't cover the Zero (O4-1 verify (a)). NOT bought: Zybo (conditional), 120° single lens (physics),
Thorlabs 10 nm (doesn't exist integrated / anti-optimal), second bird-pair set (waits on static range +
training topology).

## Gate context
Single-pixel validation is near its endgame (Option C field-proven; caged copper build + full-chain
characterization next). This order runs in parallel — nothing here blocks or is blocked by the
single-pixel outcome; arrival work is scripted as tasks **A8-2…A8-8**.
