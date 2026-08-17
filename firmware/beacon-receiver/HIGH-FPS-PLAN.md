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
