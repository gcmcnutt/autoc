# 042 — continue here (handoff, written 2026-08-21 on the DGX)

**This trip's host**: `msi` (Windows + WSL2). It is the only box with **all three** of the toolchains
this feature still needs: the WSL2→aarch64 cross path, Lattice Diamond (gateware), and avr-gcc for the
beacon pod. Three jobs below, independent — do them in any order.

---

## Where everything is

| host | address | role now |
|---|---|---|
| `beaconpi5` | `pi@100.97.242.96` | **the 042 rig**: Pi 5 8 GB, OV9281 on CAM0, NVMe `/data`, tracker runs live here |
| `beaconpi` | `pi@100.87.61.53` | 3A+, **spare** (camera removed); still has the original patched driver + build dir |
| `msi` | `100.92.184.6` | Windows + WSL2: Diamond, avr-gcc, the cross path — **this trip** |
| DGX | `promaxgb10-4331` | dev box; **the bench PSU + emitter USB live here** (`/dev/ttyACM0` = mEDBG), venv `~/.venvs/avr` |

**Physical**: emitter (code B) + PSU at the DGX bench, camera on the Pi 5, **StepFPGA currently
disconnected** (operator pulled it) — convenient, since it may travel to msi for the Diamond flash.

**State**: Stage 1 exit met single-code (ASCII scope at 10 Hz on live beacons). Tracker on the Pi 5 runs
at 640×400 / 288.5 fps with 0.000 % deadline misses. 10/10 test suites green on DGX, Pi 5, and 3A+.

---

## Job A — WSL2 toolchain check (**T069**, the actual reason the plan kept a cross path)

Tier 0, native x86_64 — everything with logic in it must build and pass with nothing installed:
```bash
cmake -S . -B build -DBEACON_RECEIVER=ON -DBUILD_AUTOC=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build --target all-targets -j
ctest --test-dir build --output-on-failure       # expect 10/10
```
Tier 1, cross to aarch64 — **no sysroot needed** for core/tools/tests:
```bash
sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
cmake -S . -B build-cross -DBEACON_RECEIVER=ON -DBUILD_AUTOC=OFF \
      -DCMAKE_TOOLCHAIN_FILE=firmware/beacon-receiver/cmake/aarch64-linux-gnu.cmake
cmake --build build-cross --target beacon_core beacon_tools beacon_tests -j
```
The cross-built tests **cannot run on WSL** — that is expected, and the end-to-end proof is to scp them to
`beaconpi5` and run them there (same binaries, foreign builder). `app/` (beacon_record/trackd) needs
libcamera, i.e. a sysroot: `rsync -a pi@100.97.242.96:/usr/include pi@100.97.242.96:/usr/lib ~/pi-sysroot/`
then add `-DCMAKE_SYSROOT=$HOME/pi-sysroot`. Record the procedure that worked in
`firmware/beacon-receiver/README.md` §Build (T069's deliverable) and tick T069 in tasks.md.

---

## Job B — replatform to 288 fps / **120 Hz chips** (the 2.4 samples-per-chip design point)

**The arithmetic**: the OV9281 delivers a measured **287.9–288.5 fps** at 640×400 (silicon ceiling ~326;
see the bench journal's speed-lab entries — the 453/480 claims were fiction). 288 ÷ 2.4 = **120.0 Hz**.

**Honest framing**: 115 Hz was never broken — at 288 fps it is already 2.50 samples/chip, *above* the
design point. Moving to 120 buys three modest things: the documented ratio exactly, a 4 % shorter word
(269.6 → 258.3 ms, i.e. slew tolerance per §2.5), and a **deterministic 12-frames-per-5-chips sampling
pattern** instead of a drifting one. Do it because the platform should sit on its design point, not
because anything is failing.

### The three artifacts that must move together

| artifact | file | from | to |
|---|---|---|---|
| **emitter** | `firmware/beacon-pod/src/config.h` | `TCA_TOP_HALF = 5435` (114.995 Hz) | **`5208`** → 625000/5208 = **120.0077 Hz** |
| **receiver** | `firmware/beacon-receiver/beacon-bench.ini` | `chip_hz_nominal = 115.0` | **`120.0`**; candidates → `115.0,120.0,121.0` |
| **decoder** | `firmware/beacon-decoder-stepfpga/experiments/s7.v` | staged `FDIV = 43480` (276 Hz / 115 Hz) | **`41667`** → 12 MHz/41667 = 287.998 Hz sample = **120.0 Hz chip** |
| ” | same file | staged `EDIV_NOM = 462629` | **`443305`** → 53.2 MHz/443305 = 120.0075 Hz (fits the 19-bit `ediva_eff`, max 524287) |
| ” | `firmware/beacon-decoder-stepfpga/host/beacon_telemetry/frame.py` | `SAMPLE_HZ` default `480.0` | **`288.0`** when the new bitstream is flashed (or `BEACON_SAMPLE_HZ=288`) |

Notes:
- `TCA_TOP_HALF`'s name becomes a misnomer (120 is not half of 200) — rename to `TCA_TOP_BENCH` while
  you are in there; `BOOT_HALF_RATE=1` and the `'H'` / `'R'` command semantics stay as they are.
- **The transition is graceful by construction**: `chip_hz_candidates` carrying both 115 and 120 means
  acquisition rotates onto whichever the emitter is actually running (see `acquire_next_rate_q8`), so
  the emitter and receiver do not have to change in the same minute. Drop 115 once the pod is reflashed.
- **`fps = 300` and `mode = 640x400` do NOT change.** 300 requested delivers ~288; ≥305 wedges the sensor.
- **The 320×200 NN contract is already satisfied** — the record grid is *always* the 320×200 M2 grid
  (`m2_div` in `track.c`: 640-wide native → M2 = native/2). The sensor's 320×200 mode was only ever
  about speed, and it does not deliver speed. Nothing to do here.

### Flashing the emitter from this trip
avr-gcc is WSL-native on msi, but the pod's USB is **at the DGX bench**. Two options:
1. Build the `.hex` on msi, copy it over, flash from the DGX with `~/.venvs/avr/bin/pymcuprog` (already
   installed there, and `pymcuprog reset -d attiny416` is the documented un-latch). **Preferred.**
2. Or install avr-gcc on the DGX and retire this cross-host dance entirely — worth 20 minutes if the
   pod is going to change more than once.

⚠️ **Policy (CLAUDE.md): run `firmware/beacon-decoder-stepfpga/host/regression.py` after ANY emitter
firmware or decoder gateware change.** That needs the StepFPGA reconnected and, post-Job-C, agreeing on
120 Hz with the pod.

---

## Job C — build + flash the staged gateware (Diamond, Windows side)

`s7.v` already carries the retune, staged and **never built** (commit `5d2532c`). Apply Job B's 120 Hz
constants first, then:
- Build: `firmware/beacon-decoder-stepfpga/experiments/deploy_s7.sh` is the WSL-interop recipe
  (`pnmainc.exe` against `C:\lscc\diamond\3.14`).
- Flash: copy the `.jed` to the STEPLink mass-storage volume. **Note (found 2026-08-19): STEPLink mounts
  natively on Linux** (`/media/gmcnutt/STEPLink` on the DGX) — so only the *build* actually needs Windows;
  `docs/toolchains.md` lines 41–43 still claim otherwise and should be corrected once confirmed on msi.
- `+define+CHIP200` restores the 200 Hz flight-nominal set; the default build is now the bench rate.

---

## Carry-over traps (each cost real time)

1. **Verify the target actually rebuilt.** `cmake --build --target X` prints `Built target X` even when
   it compiled nothing. Two debug rounds were lost to a stale binary on the Pi. `touch` the edited file
   and look for a `Building C object` line before trusting any behavioural change.
2. **`recovery_sweep.Emitter.close()` now parks the pod at `'H'`** (bench rate), deliberately — no
   harness exit may silently leave it at 200 Hz. FPGA sessions that want 200 must select `'R'`
   explicitly and put `'H'` back.
3. **The bench is 115-family only** (now 115/120) — a 200 Hz lock is a false comfort on a derated rig,
   so 200/210 are out of the candidate list. Production rates return with the flight article, not before.
4. **DPLL rate adjustment is PARKED** (phase adoption only) — two live attempts destabilised it. It comes
   back as sim-first work against golden clips, never live tuning. See `track.c`.
5. **lyu-guest has client isolation**: bench hosts can only reach each other over tailscale. The DGX has
   an `nmcli` shared-mode connection `pi5-share` on `enP7s7` for cabled bring-ups.

---

## Where the work stands (57/74 tasks)

Done: Setup + Foundational, US1 recorder, US2 tracker core, US3-lite acquisition, US4 scope + Stage 1
exit (single-code). Open, roughly in value order:

- **T044–T047** — oracle, inject, score, and the first envelope CSV from a pan/tilt slew clip. *This is
  the feature's actual deliverable* (a measured envelope, not an anecdote) and it is all dev-box work.
- **Evening A/B re-run** — the Pi 5 gate scored 80 % lock duty in daylight vs the 3A+'s 100 % in evening
  calm. Owed: a like-for-like evening number, and a replay-autopsy of the remaining churn.
- **T050–T053** — RANSAC proto-tracks, decode-along-track, acquisition off-thread.
- **T057/T058** — two-code (gated on the code-A flight cube) and the false-alarm run.
- **T066–T070** — golden vectors from live, retire `beacon_track.py`, tracker regression entry point,
  and T069 above.
- **Backlog**: thermal-event sensing/logging before any outdoor session (`specs/BACKLOG.md`, 042 section).

**The 480 fps question is not closed, it is re-framed** — the OV9281 cannot reach it at stock PLL. The
three surviving paths (PLL overclock / dual-camera FSIN interleave / accept ~288) are written up at the
end of the bench journal's speed-lab entry. The operator leans toward the interleave since both cameras
are already in hand and the birded pair was always the flight architecture.
