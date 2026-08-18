# firmware/beacon-receiver — the camera-era receiver toolchain (Pi + OV9281)

**Direction (2026-08-16)**: the array-receiver analysis platform is a **Raspberry Pi 3A+ + InnoMaker
OV9281 global-shutter module** (order-04), running Raspberry Pi OS **Bookworm** with the *mainline*
`ov9281` driver. The FPGA route (Lattice CrossLink-NX / Zybo) is parked — research retained here as
PDFs; CrossLink-NX stays the eventual flight-article candidate. The decoder prototype for this toolchain
is tasks **A8-6** (031) and runs in parallel with 041 training. Camera model measured for 041:
f·θ projection, 95° H × 61° V (`specs/031-beacon-camera/camera-era-knobs.md`).

## What's here
| file | what |
|---|---|
| `pi/fps_probe.py` | frame-rate ladder per sensor mode via `rpicam-raw` to tmpfs (honest frame counting, detects mode remapping) |
| `pi/INSTALL.md` | **everything installed on the Pi**, in order, with the config.txt lines — reproducible rebuild |
| `PSG_2020_I0211K_Rev28.pdf`, `FPGA-EB-02028-1-4-*.pdf` | Lattice research (parked FPGA route) |
| `innomaker-cam-mipi9281raw-v2-lens-drawing.png` | the module's stock 2.8 mm lens drawing (1/2.7″ format, CRA 10°, M12) |

## Measured facts (this rig)
- **Sustained fps, mainline driver, 640×400 8-bit raw: ~260–280.** 300 falls short; ≥320 = frontend
  timeout. Same on the 3B and 3A+ (same VideoCore IV / CSI-2 RX).
- **The kernel driver defines ONLY 1280×800 / 1280×720 / 640×400** (`v4l2-ctl --list-subdev-framesizes`).
  A 320×200 request maps to 640×400 → no faster. **400+ fps needs a driver mode-table entry** = the
  InnoMaker Bullseye vendor driver (its "453 fps stream mode"). That's a second-SD-card experiment.
- 640×400 modes are `(0,0)/1280x800 crop` = **full sensor field at speed** (binned/skipped, not center-cropped).
- At 250 fps the beacon's N=31 Gold code was read from raw frames (whole-frame photometry, then
  per-pixel chip-rate energy with a twilight room in frame): **26/31 vs CODE0, chip rate 209.5 Hz**
  (emitter RC +4.7 %) — 2026-08-13 (3B) and 2026-08-16 (3A+).
- Auto-exposure/auto-gain WILL sabotage code capture (settling ramps read as "signal"): always
  `--shutter <us> --gain 1` for measurements.

## Workflow (from the DGX, over Tailscale)
- Pi = `pi@100.110.13.80` (key auth installed). Console preview: `rpicam-hello -t 0` on the Pi's HDMI.
- Raw burst: `rpicam-raw -n -t 2000 --mode 640:400:8 --framerate 250 --shutter 3000 --gain 1 -o /dev/shm/b.raw`
  (tmpfs; SD can't take 64 MB/s). Analysis scripts run on the DGX after `scp`.
- Bench instruments (PSU `psu.py`, emitter cmd link) live in `firmware/beacon-decoder-stepfpga/host/`.

## Live tracker (2026-08-17) — `pi/beacon_track.py` (Pi) + `pi/beacon_display.py` (DGX), `pi/live.sh`
- Camera **250 fps** 640×400 8-bit; emitter bench **'H' mode = 115 Hz nominal (measured ~121–123 with the
  RC skew — pass `--chip 121`; the DPLL-lite tracks the rest)**. 2×2 SUM to 320×200; one-word raw ring;
  matched filter both Gold codes × 31 phases; ACQUIRE full-field (2.5 s in Python) → TRACK ±16 px ROI
  (21 ms); HOLD rides ≤6 low-q reports; q scale-free (noise floor ~0.5, lock ≥0.62); centroid + CEP;
  bearing via measured f·θ; JSON lines every 0.5 s (frame-counted). **30 s run: 49/49 locked, 8 held,
  chip tracked 121→123.4.**
- Display: M2 grid 320×200 @ 0.304°/px, centre (0,0), +x right / +y down (041 `camera_projection.h`);
  **code A = PORT (red), code B = STARBOARD (green)** — aviation nav-light convention.
- Known limit: Python. Per-frame 1.5 ms of a 4 ms budget; full-field acquire 2.5 s starves the pipe
  (fps sags then recovers). **Next (A8-6): C/NEON `beacon_trackd`, same pipe in / same JSON out —
  ~0.05 ms/frame + ~30 ms full-field acquire.**
