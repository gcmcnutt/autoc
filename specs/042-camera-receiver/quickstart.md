# 042 — quickstart

Bring-up on all three build paths, then the first thing worth running. Assumes the 031 rig
(`firmware/beacon-receiver/pi/INSTALL.md`) is already good.

## 1. Dev box (aarch64) — where you live day to day

```bash
cmake -S . -B build -DBEACON_RECEIVER=ON
cmake --build build --target beacon_core beacon_tools beacon_tests -j
./build/firmware/beacon-receiver/beacon_tests
```
No camera, no libcamera, no cross-compile: `core/`, `tools/` and `tests/` build native. **Same
architecture, same NEON, bit-identical results as the Pi** — so correctness transfers and only *timing*
has to be measured on target.

## 2. Pi (native) — the always-works path

```bash
rsync -a --exclude build/ ~/autoc-beacon/ pi@100.110.13.80:~/autoc-beacon/
ssh pi@100.110.13.80 'cmake -S autoc-beacon -B autoc-beacon/build -DBEACON_RECEIVER=ON \
    && cmake --build autoc-beacon/build --target beacon_record beacon_trackd -j2'
```
`-j2`, not `-j4`: the 3A+ has 512 MB and libcamera headers are heavy. Add swap if it OOMs.

## 3. WSL2 (x86_64) → aarch64 — the field-update path

```bash
sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
rsync -a pi@<pi>:/usr/include pi@<pi>:/usr/lib ~/pi-sysroot/     # once, then on libcamera upgrades
cmake -S . -B build-cross -DBEACON_RECEIVER=ON \
      -DCMAKE_TOOLCHAIN_FILE=firmware/beacon-receiver/cmake/aarch64-linux-gnu.cmake \
      -DCMAKE_SYSROOT=$HOME/pi-sysroot
cmake --build build-cross --target beacon_trackd -j
```
`core/`, `tools/` and `tests/` cross with **no sysroot at all** — only `io/`+`app/` need it. That is the
whole point of the dependency boundary.

**At the field, prefer not to compile.** Tuning knobs are runtime config; edit the `.ini` and restart.
Cross-compilation is for real code changes (spec §16.2).

## 4. First thing to run: the kernel bench

```bash
./build/firmware/beacon-receiver/beacon_bench --kernels   # dev box: correctness + relative cost
ssh pi@<pi> './autoc-beacon/build/.../beacon_bench --kernels'   # Pi: the number that matters
```
Confirms R5's **1–2 GMAC/s per A53 core** assumption before any design leans on it. If the Pi comes back
materially under, the §10 compute budget and the acquisition threading model both need revisiting — better
to know in week one.

## 5. First capture (no tracker involved)

```bash
ssh pi@<pi> 'beacon_record --config beacon.ini --mode burst \
   --burst-frames 64 --burst-every 500 --duration 60 --out /dev/shm/clip.bcnr'
scp pi@<pi>:/dev/shm/clip.bcnr .
```
Bursts of 64 contiguous frames (one full word at 121 Hz) every 500 — ~8 MB/s, SD-safe, and unlike uniform
decimation **every burst is replayable**. Proves libcamera, the container, timestamps and metadata with no
algorithm attached.

## 6. Replay parity check — the gate everything else rests on

```bash
beacon_trackd --config beacon.ini --source replay:clip.bcnr --emit binary:a.bcn1
beacon_trackd --config beacon.ini --source replay:clip.bcnr --emit binary:b.bcn1
cmp a.bcn1 b.bcn1        # MUST be identical
```
Then the real one: a live run recording raw, replayed afterwards, must reproduce its own record stream
byte-for-byte. If that fails, something reads a wall-clock or floats in `core/` (R2, R3) — fix it before
building anything on top.

## 7. Scoring a run

```bash
oracle --in clip.bcnr --out truth.bin
score  --records a.bcn1 --truth truth.bin --out cell.csv
```
One CSV row per envelope cell, carrying all of §3.1's two rates, §3.2's invariant plus both °/s columns,
§11.1's deadline-miss rate, false-acquire rate and relock times.

## Traps carried forward from 031

- **Run the control first, every time.** A broken harness invalidated a whole round of "zero-frame"
  verdicts during the high-fps work.
- **Never auto-exposure/auto-gain for measurements** — settling ramps read as signal.
- The bench emitter's `'H'` mode is **volatile**: any reset returns it to 200 Hz nominal. Acquisition is
  rate-agnostic by design, but if something "locks at the wrong rate", check the emitter before the code.
- 3A+ throttles at 80 °C under sustained capture. Heatsink it.
