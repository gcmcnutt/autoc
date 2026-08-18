# High-fps plan — can the OV9281 + Pi 3A+ sample at 400–480 fps? (031 research question)

**Scope (operator 2026-08-16)**: this is a *can-the-hardware-get-there* question for 031 (sampling
budget, DMA/CSI throughput). The moving-scene high-speed correlator that USES it is a **future feature**.
031's deliverable = a yes/no with numbers + the recipe.

## Finding 1 — there is NO InnoMaker "special driver" (2026-08-16)
The InnoMaker GitHub README installs `dtoverlay=ov9281` — the MAINLINE kernel driver we already run —
and its documented mode table is IDENTICAL to ours (RAW8 640×400 @ 309.79 fps max; RAW10 @ 247.83).
The product page's "453 fps" has no shipped mode behind it. Their extra is an **external-trigger
script** (`ov9281_trigger`, GPIO23 test-pulse) — useful later for phase-locking capture, not for fps.
→ Reflashing to Bullseye for a vendor driver would buy NOTHING. Stay on Bookworm.

## Finding 2 — the ceiling is a NUMBER IN A TABLE, and it's ours to edit
The mainline `ov9281.c` mode struct hard-codes per-mode timing (`vts_def`, `hts`, PLL, link rate). Our
measured 260–280 sustained vs the listed 309.79 is Pi-3 CSI/Unicam headroom, not the sensor. **To go
faster you add a mode with fewer rows** — the sensor's row time is fixed by HTS/pixel-clock, so fps ≈
1/(VTS × t_row): halve the rows, ~double the fps. Community precedent: editing `vts_def` for 640×400
took people from 210 → 260 fps ([RPi forum](https://forums.raspberrypi.com/viewtopic.php?t=340880),
[t=321017](https://forums.raspberrypi.com/viewtopic.php?t=321017)); the raspiraw crowd hit 660–1007 fps
on other sensors by exactly this row-cropping trick.

## The steps (est. 1–2 evenings; all on a SECOND SD card — reload is cheap)
1. **Clone the card** (or fresh Bookworm + `pi/INSTALL.md`). Keep the working card pristine.
2. **Get the driver source**: `apt source`/git the RPi kernel tree matching `uname -r`
   (`raspberrypi/linux` branch `rpi-6.12.y`), file `drivers/media/i2c/ov9281.c`.
3. **Add a mode**: copy the 640×400 (bin-2) mode struct → new **640×200** entry: window/output height
   200, `vts_def` ≈ half (keep ≥ height + blanking min ~ 20–30 rows), same HTS/PLL/link freq. This is a
   full-WIDTH bin mode with a VERTICAL CROP → field = 95° H × ~30° V on the fisheye (full H field
   preserved — the axis that matters for the birded pair). Alt entry: 640×256 (~380 fps, 39° V) as the
   softer step.
4. **Build just the module** (`make M=drivers/media/i2c` against the matching headers — `raspberrypi-
   kernel-headers`), install `ov9281.ko`, `depmod`, reboot. (Also expose the mode to libcamera: the
   IPA/tuning uses the driver's enumerated sizes — a new size should appear in `--list-cameras`.)
5. **Measure with `pi/fps_probe.py --modes 640:200:8 --fps 400,453,480,520`** — the honest counter.
   Success = ≥453 sustained, errs 0. If the frontend times out below the sensor's rate, the Pi 3 CSI is
   the limit → the answer for THIS Pi is "no", and the recipe moves to Pi 4/5 (faster Unicam/PiSP) or
   the FPGA route — still a valid 031 conclusion.
6. **Re-run the code capture at 453 fps** (chip-rate energy detector, `--shutter ≤2000 --gain 1`) —
   confirm 2.4 samples/chip at ~189 Hz reads ≥26/31 like the 250 fps runs. That's the sampling-budget
   YES for the record.

## What 031 hands forward
- Emitter solution: cube @306 mA (A7), thermally proven; C-11 stars for robust mounting.
- Receiver/optics: Option-C 1-pixel front end (field-proven in sun); OV9281 + fisheye array receiver
  with measured f·θ 95×61 model; 850 filter still needed outdoors.
- Sampling budget: mainline 260–280 fps today; ≥453 via a driver mode entry (this plan) — result TBD.
- **Future feature**: high-speed correlator for moving beacons across the scene (per-ROI/tracker bank
  on the Pi/NEON, or FPGA), built on whichever fps the recipe delivers.

## RESULT (2026-08-16, second SD card, Trixie 6.18.34 arm64, patched `ov9282.c` — the answer)

**Custom modes WORK; the Pi 3 does NOT get faster. Ceiling ≈ 280 fps regardless of frame size.**

What was learned building it (mechanism, not folklore):
- **The driver is `ov9282.c`** (the `ov9281` dtoverlay is a name only). Mode table is fully table-driven —
  add an entry and it enumerates (`--list-cameras` showed `640x200 [588.93 fps]`, `640x392`, etc.).
- **How to shrink a binned mode on the OV9282: crop at the ISP OUTPUT (`0x380a/0x380b`), NOT the readout
  window (`0x3806/0x3807`).** Every readout-window crop of the bin-2 mode (400, 408, 600, 608, 784 rows)
  produced ZERO frames — the sensor never asserts frame-valid. Full 800-row window + output height 200
  streams cleanly. (Four register-crop variants died before this was isolated.)
- **The `media-ctl` + raw-V4L2 harness was BROKEN** (control 640×400 also gave 0 frames on a fresh boot);
  it invalidated a whole round of "zero-frame" verdicts. `rpicam-raw` to tmpfs is the honest harness.
  Lesson: run the CONTROL first, every time.
- **The number**: `640×200 @ ≤300 req → 280 fps sustained; ≥350 → frontend timeout.` Same as 640×400
  (260–280). Half the bytes per frame, identical fps ⇒ **the Pi 3 limit is per-FRAME (Unicam/pipeline
  overhead), not bandwidth and not sensor row time.**

**031 verdict on the sampling-budget question**: on Pi 3-class hardware the OV9281 path tops out at
~280 fps ⇒ chip rates ≤ ~115 Hz at 2.4 samples/chip (emitter is one timer constant). **≥453 fps needs a
faster host** — Pi 4/5 (faster Unicam / PiSP; Pi 5 also brings the dual-CSI the birded pair wants) or
the FPGA route (Zybo, still conditional). The recipe (patched driver + `fps_probe.py`) transfers as-is
to a Pi 4/5 for the next measurement. Files: `pi/ov9282-experimental.c`, `pi/ov9282-640x200.patch`.
Card state: the experimental module is installed on the Trixie card (`ov9282.ko`; stock kept as
`ov9282.ko.xz.stock` — revert = rename back + `depmod -a`).

## Addendum (2026-08-16): 320×200 attempt + HOW reduced modes compute pixel values (matters for the correlator)

- **320×200 in-sensor (h-skip-4 via `0x3814=0x71`)**: enumerates (744 fps advertised) but streams ZERO
  frames — that skip value isn't accepted by the readout. Parked: the per-frame ceiling means it could
  not have raised fps on the Pi 3 anyway.
- **What a reduced mode actually does to the photons — three different knobs on the OV9281:**
  - **vertical BIN (`0x3815=0x22`)** — analog row summing, ALL photons kept (used for V in 640×400/640×200).
  - **horizontal SKIP (`0x3814=0x31`)** — 1 column read, 2 discarded: photons LOST, sub-pixel beacon can
    fall in a skipped column. **The stock 640×400 mode is bin-V × skip-H** — half good, half the "vanishing
    beacon" trap from order-04 verify (c).
  - **digital output crop (`0x380a/0b`)** — rows discarded, no summing: shrinks FIELD, not sampling.
  Analog binning tops out at 2×2; deeper "320×240" in-sensor is skip or ISP averaging — NOT 16-photon sums.
- **Design consequence (the correlator's 320×200)**: take **640×400 @ ~280 fps** from the sensor and do the
  final 2×2 as a **software SUM** on the Pi (NEON, ~65 Mpx/s — trivial). Keeps every delivered photon,
  gap-free, and the reduction math (sum / max / centroid per ROI) becomes a controllable knob instead of
  whatever the readout did. In-sensor reduction below 640×400 is not worth its photon cost.

## What a 640×400 / 640×200 pixel IS (from the mode registers, 2026-08-16)

`0x3814=0x31` (h odd/even increment 3/1 → read 2 adjacent columns, skip the next 2), `0x3815=0x22` (v bin-2),
`0x3820=0x60` (bit5: v-bin enable), `0x3821=0x01` (bit0: h-bin enable). ⇒ **each output pixel = an analog
2×2 BIN of 4 photosites, on a 4-column × 2-row footprint; the other 2 columns × 2 rows are SKIPPED — never
sampled.** Half the light in each pixel's footprint is captured; the horizontal axis is a *sampled* grid.
Consequences: (a) a point-source beacon's response MODULATES as it drifts across the column grid (the
"angular variation" the operator flagged); (b) mitigation = PSF ≥ ~2 output pixels wide (slight defocus,
both lenses focusable) — smooth field response + centroiding, at the knobs-doc √k SNR derate; (c) 641×400
vs 640×200 are identical in this respect — the 200 just drops rows at the output. Feed 041's sensor
model: sampling = 2×2 bin on a 4×2 stride (H stride 4 photosites = 12 µm, V stride 2 = 6 µm) → effective
IFOV per output pixel 0.076° H (fisheye) with a 50 % horizontal fill factor.

## Beacon signal level on the camera (2026-08-16, exposure ladder; feeds the daylight budget)

Bench emitter (5-die string, 51 mA), **~90° off-axis, ~2 m**, 1.8 mm fisheye, 640×400 8-bit, gain 1:
lit-chip peak = **255 (rail) at 2000 and 500 µs; 170 at 200 µs; ~85 at 100 µs** (linear) →
**~850 counts per ms of exposure ≈ 8–10× over full scale at the 2 ms capture exposure.** This is the
WORST emitter aspect through the SMALLEST pupil at bench current; face-on cube @306 mA is ~×30 more.
Consequences: (a) signal headroom is not the daylight problem — sky background per pixel is (empirical
#1 still the number to get); (b) **exposure must track range / be per-ROI** — at 2 m the correlator
wants ~200 µs, not 2 ms (the "manual exposure is load-bearing" requirement, now with a number);
(c) 8-bit is the tight axis — 10-bit mode buys 4× headroom for ~20 % fps. Emitter bench 'H' mode
(~115 Hz, camera-measured 121 Hz) is live; run the camera at its 280 ceiling for 2.3 samples/chip.

## Will a Raspberry Pi 5 reach 453 fps? — research verdict (2026-08-17)

**Confirmed**: the OV9281 works on Pi 5 with the same `dtoverlay=ov9281`, same mode table (309.79 max @
640×400 stock) — RPi forum thread + vendor "all Pi models incl. Pi 5" claims. **Pi 5 pipeline is different
in kind**: RP1 CSI-2 front-end + PiSP ISP; raw V4L2 (`/dev/video0`) is NOT a drop-in path (an RPi engineer:
"the Pi 5 pipeline is more complex … you need to configure it via Media Controller"); rpicam/libcamera is
the sanctioned route (that suits us — our tools are already rpicam-raw). Pi 5's CSI-2 receiver is faster
(4-lane-capable, higher link rate) and the SoC has ~3× the per-core CPU.

**NOT confirmed anywhere: a measured OV9281 rate above the stock 309.79 on Pi 5.** Nobody has published
it — the same "custom mode" gap as on Pi 3; the Pi 5 threads stop at "it works, ~90 fps met my need". So
the honest expectation, not a promise: **the Pi 3's ~280 ceiling was a per-FRAME cost (identical at 640×400
and 640×200) → on the Pi 5's much faster CSI front-end + CPU that ceiling should rise substantially, and
our patched 640×200 mode (sensor advertises 589 fps) is the tool that would show it.** 453 fps at 640×200
on Pi 5 = plausible-to-likely; ~600 possible; the risk is a new per-frame limit inside PiSP/libcamera at
those rates. **This is a ~$80 experiment with a ready-made recipe** (patched ov9282.c + fps_probe.py +
the same rpicam-raw harness — all transfer as-is).

**Recommendation**: yes, buy a **Pi 5 (4 GB is plenty)** — for 453 fps *and* two other reasons that make
it the right host regardless of the fps result: (1) **dual CSI ports = the birded pair on one board**;
(2) ~3× per-core CPU + a real NEON budget for the tracker-bank without threading heroics. Order with the
official 27 W PSU and an active cooler (sustained rpicam-raw pegs a core). Keep the 3A+ as the bench/
flight-mass reference. First act on arrival: `dtoverlay=ov9281`, rebuild the patched module against the
Pi 5 kernel (`rpi-6.18.y`/`v8-16k`), `fps_probe.py --modes 640:200:8 --fps 400,453,520,600`.
