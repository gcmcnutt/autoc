# firmware/beacon-receiver — the camera-era receiver toolchain (Pi + OV9281)

**Direction (2026-08-16)**: the array-receiver analysis platform is a **Raspberry Pi 3A+ + InnoMaker
OV9281 global-shutter module** (order-04), running Raspberry Pi OS **Bookworm** with the *mainline*
`ov9281` driver. The FPGA route (Lattice CrossLink-NX / Zybo) is parked — research retained here as
PDFs; CrossLink-NX stays the eventual flight-article candidate. The decoder prototype for this toolchain
was tasks **A8-6** (031); **superseded by 042** — the C11 receiver in `src/`, `tools/` and `tests/` here
(see §Build below). It runs in parallel with 041 training. Camera model measured for 041:
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
- Pi = `pi@100.87.61.53` (tailscale `beaconpi`, the 3A+; key auth installed). The older `100.110.13.80`
  (`raspberrypi`, the 3B) is a different machine and is usually offline — check `tailscale status` before
  blaming the network. **The 3A+ has no `cmake`**, so quickstart.md §2's native build needs it installed
  first, or build the dependency-free parts with `gcc` directly. Console preview: `rpicam-hello -t 0` on the Pi's HDMI.
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

## 25 ft "intermittent" — diagnosis (2026-08-17): the emitter had reset to 200 Hz, not gain/exposure
Symptom: tracker intermittent/no-lock with the 5-LED @50 mA string at 25 ft. Suspects in order tested:
exposure (a 200→3900 µs sweep showed the beacon at 168 counts modulation @2 ms — signal fine, and gain
buys nothing: lit and floor rise together), the correlator (scored the KNOWN blinking pixel at q≈0.27
= noise while its series showed a perfect 8:1 code — so the DECODE was failing on a good signal),
and finally the chip-rate: run-length histogram at the beacon = 1–2 frames/chip → **209.5 Hz — the
emitter had rebooted to 200-nominal when it lost USB during the move ('H' mode is volatile PERBUF;
mEDBG dropped, trap #2)** while the tracker templated 121 Hz. `--chip 209` locked immediately (q→0.975).
Fixes: (1) **acquire is now rate-agnostic** — tries {--chip, ×1.05, 200/210, 115/121} and adopts the
winner (ring sized for the slowest); (2) 2 ms is the right shutter at 25 ft (`--agc` doubles/halves
shutter to keep the lit level in band, restarting capture); (3) lesson for the C port: multi-rate
acquire is ~10 s in Python (6 full-field passes) — fine once, but the reason the port matters.
Reminder: 250 fps / 209 Hz = 1.19 samples/chip (sub-Nyquist) — locks, but 'H' (115) is the honest mode.


## Build — the three paths (042 T004)

`core/`, `tools/` and `tests/` are **C11 with zero dependencies**, so they build natively on every host
with nothing installed. That is not tidiness: it is why the identical code runs live on the Pi, in replay
on the dev box, and cross-compiled from WSL2, and it is what makes replay parity checkable at all. `io/`
and `app/` are the only things that need libcamera, and they are kept thin for exactly that reason.

### 1. Dev box (aarch64) — where you live day to day
```bash
cmake -S . -B build -DBEACON_RECEIVER=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build --target beacon_core beacon_tools beacon_tests -j
ctest --test-dir build -R beacon --output-on-failure
```
No camera, no libcamera, no cross-compile. **Use a RelWithDebInfo build for `beacon_bench`** — an
unoptimised bench under-reports by ~5x and would falsely condemn R5; the binary warns you if you forget.

### 2. Receiver-only configure (the Pi, and the cross path)
```bash
cmake -S . -B build -DBEACON_RECEIVER=ON -DBUILD_AUTOC=OFF
```
`BUILD_AUTOC=OFF` stops the top-level `CMakeLists.txt` before its `find_package(VTK|Eigen3|AWSSDK REQUIRED)`
block — none of which exists on a Pi, and all of which are fatal at *configure* time. One build system,
still (Constitution IV); the receiver simply does not drag the sim toolchain onto the bench host.

### 3. WSL2 (x86_64) → aarch64 — the field-update path
```bash
sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
cmake -S . -B build-cross -DBEACON_RECEIVER=ON -DBUILD_AUTOC=OFF \
      -DCMAKE_TOOLCHAIN_FILE=firmware/beacon-receiver/cmake/aarch64-linux-gnu.cmake
cmake --build build-cross --target beacon_core beacon_tests -j
```
`core/`, `tools/` and `tests/` cross with **no sysroot at all**; only `io/`+`app/` need one
(`-DCMAKE_SYSROOT=$HOME/pi-sysroot`, rsync'd from the Pi). At the field, prefer not to compile — tuning
knobs are runtime config, so edit the `.ini` and restart.

### The one-command gate
```bash
cmake --build build --target all-targets
```
Builds every target in the tree and **propagates failure** (T002b). Tier 0 is the default; `-DWITH_EMBEDDED=ON`
adds the tier-1 cross/embedded targets, which need no hardware attached and are the tier that catches
interface drift. Tier 2 (Diamond/stepfpga, anything needing hardware) is never automatic.

### Contracts have golden vectors, not shared headers
`tests/golden/record_vectors/` holds canonical encoded records with known field values. Any second
implementation — xiao, analysis tooling — verifies against those bytes with **its own codec**, so drift
fails a test on whichever side is stale with zero shared source and zero `#ifdef`. Regenerate them
(`beacon_gen_record_vectors`) **only** on a `format_version` bump; regenerating to make a failing test
pass destroys the mechanism.
