# 031 bench journal — living state (portable across machines)

> **What this is**: the running state of the beacon bench — instruments, quirks, open items, and pointers —
> kept **in-repo** (operator decision 2026-07-18) so any machine/session can pick up the work. Update this
> file as the bench evolves; deep results live in the outcome docs it links. Machine-local assistant memory
> should hold only a pointer here.

## Camera-era analysis order OPENED (2026-08-08): [beacon-order-04.md](../../cad/beacon-eval/beacon-order-04.md)

OV9281 MIPI ×2 (B0162) + UVC shield (B0264) + **LIFCL-40-EVN** (CrossLink-NX — sized from the s3 datum:
3.5k LUT/correlator on MXO2 → 20–35k LUT tracker-bank architecture vs 39k cells, hard MIPI D-PHY).
At-order verifies: EVN onboard RAM ≥6 MB (T011), Radiant toolchain (new install, not Diamond). The UVC
OV9281 doubles as the trusted focal plane to finish the C-14 lens validation (M12 mount, global shutter,
no IR-cut after lens swap).

## Pi 5 Phase 3: camera ported to pisp, live tracking at 80 % daylight duty (2026-08-20 afternoon)

**Camera**: on beaconpi5 CAM0 (`dtoverlay=ov9281,cam0`), refocused. Mode table tops at 640×400 R8
309.79 fps (the 640×200 patched mode needs the §12 driver port to this kernel).

**The pisp port cost a debugging chain worth recording — every step is also commented in
`src_libcamera.cc`:**
1. **R8-in-memory does not exist on pisp** — the CFE writes 16-bit containers even for Y8 sensor modes
   (sample in the HIGH byte). And the raw/CFE path starved into Idle after its 2 internal image buffers
   in every configuration tried. **The ISP path is the answer**: YUV420, plane 0 = the u8 Y image,
   hardware depth conversion, zero CPU. (ISP tuning curve now sits between sensor and samples — noted
   as a caveat for §5 photometry work.)
2. **The 8-bit SENSOR mode must be forced via `SensorConfiguration`** (bitDepth 8) — accepting a 16-bit
   stream format otherwise selects the Y10 mode and caps at 247.8 fps.
3. **`FrameDurationLimits` below ~3.3 ms WEDGES the sensor after one frame** (the mode advertises
   3228 µs — optimistic). Delivery ladder: request 300 → 288.5 fps delivered; ≥305 → wedge. fps=300 is
   the pinned Pi 5 bench rate; 309.79-and-beyond belongs to the §12 driver-timing evening.
4. Error-path segfault fixed (camera shared_ptrs must not outlive the CameraManager).

**Daylight lessons (it is afternoon; the 3A+ baseline was evening):**
- Beacon contrast is exposure-inverse under daylight: 200 µs → 1.24×, 30 µs → 2.06×. Bench ini floor is
  now 60 µs/gain 2 with the ROI-driven AGC walking it up as the scene darkens.
- LED-PWM flicker out-blinks the beacon at short exposure: acquisition widened to 3 seeds/pass, 4
  candidates ("the code kills false candidates" is the designed discriminator — let it see candidates).
- **THE structural fix, found by replay-debugging a captured daylight clip offline (the parity
  architecture earning its keep): the scale ladder may only move on MEASURED ticks.** A fresh
  candidate's empty window reports q=0, and "weak → widen" demoted every seed to coarse on its first
  tick, where flicker owns the correlation surface — nothing ever confirmed. Absence of evidence is not
  evidence of weakness. One condition: 0 % → 80 % lock duty.

**Gate status**: 80 % duty / 8.2 s runs / q 0.89 / 0.000 % deadline misses at 288.5 fps in DAYLIGHT —
vs the 3A+'s 100 % in evening calm. Re-run the gate this evening for the like-for-like number; the
daylight churn (death + reacquire every ~8 s) is the next tuning target, same replay-debug method.

**Next**: §12 evening for 453/480 fps (port `ov9282-640x200.patch`, fix the pisp frame-duration floor,
add faster 640×400 timings). 480 fps = exactly 200 Hz chips at 2.4 samples/chip → emitter `'R'` and the
FPGA's `CHIP200` become correct again — that is the production-rate flip.

## Pi 5 bring-up Phases 0-2 COMPLETE (2026-08-20): beaconpi5 on the tailnet, NVMe at 454 MB/s, all tests green

**Host**: Pi 5 Model B Rev 1.1, 8 GB, active cooler, 52pi EP-0241 M.2/PoE+ HAT, WD SN7100S 1 TB (2230,
Gen4 stick on the Gen2 x1 link). `beaconpi5` = tailscale `100.97.242.96`, wifi `192.168.1.197`
(lyu-guest), user `pi`, passwordless sudo, fallback password `beacon5`.

**Measured**:
- NVMe `/data` (ext4, fstab by PARTUUID): **454 MB/s sustained write, 475 MB/s read** (8 GB, fsync) —
  3.9x the 453 fps flight recording rate. Spec §16.4's "continuous full raw — comfortable" is now a
  measurement, not a claim.
- A76 kernels (vs the A53): `mac_i16` scalar **7.6 GMAC/s** (6.2x), `reduce2` NEON **0.06 ns/px**
  (17.5 G/s), `reduce4` NEON 27 G/s -> full-frame reduce4 = 10 us. All 10 test suites pass
  (libcamera 0.7.2).

**The headless bring-up cost five card-shuttles; the traps, so nobody pays twice:**
1. **THE wifi blocker on a manually-seeded (non-Imager) card is NetworkManager's software radio
   switch** — `nmcli radio all` -> "WIFI disabled". NM re-asserts the rfkill soft-block every start, which
   is why fixing the persisted rfkill state file, the modprobe `default_state`, AND the cmdline regdom
   all failed in turn (each was real, none was the actor). Fix: `nmcli radio wifi on` +
   `raspi-config nonint do_wifi_country US`. The Imager GUI does this invisibly; manual seeds must.
2. **The guest wifi (lyu-guest) has client isolation** — LAN debugging of a headless Pi is impossible on
   it, and tailscale is the only path between bench hosts (relay "sfo"). The escape hatch that ended the
   loop: **direct cat5 to the DGX's free port (`enP7s7`) with an NM `ipv4.method shared` connection** —
   DHCP + NAT internet over the cable, the pre-baked tailscale join fired on its own, and interactive
   nmcli found the real blocker in one look. Keep `pi5-share` configured on the DGX for future bring-ups.
3. Black-box pattern that worked: a boot service appending rfkill/nmcli/ip/https-reachability to the FAT
   boot partition every 20 s — one boot cycle = full evidence, readable on any machine.
4. tailscale is the OFFICIAL apt package now (1.102.3), state under /var/lib/tailscale; the static-binary
   bootstrap + join units + black box were all removed after use.
5. Pi 5 MAC OUI on this unit: 88:A2:9E (newer Raspberry Pi allocation).

**NEXT = Phase 3, needs hands**: move the OV9281 (22-pin cable) to beaconpi5, `dtoverlay=ov9281` is
already seeded in its config.txt. Then: mode table on the pisp pipeline, the WC-read-wall measurement
(does continuous capture hold sensor rate where the 3A+ capped at 215 fps?), and the A/B gate — trackd
live at the same 115 Hz operating point must match the 3A+'s 100 %-lock baseline before the 3A+ is
touched further. Phases 4-6 (NVMe recorder milestone, the §12 high-fps evening, role swap) follow.

## Bench rate policy: 115-FAMILY ONLY (operator 2026-08-19, evening)

**200/210 Hz removed from the camera receiver's acquire candidates** (`beacon-bench.ini`). Rationale: the
whole rig is derated to ~115 Hz; a 200 Hz lock is a false comfort that measures the wrong operating
point. Consequences, all deliberate:
- An emitter reset to 200 Hz (volatile-'H', trap #2) now presents as **NO LOCK** — go check the emitter.
- `recovery_sweep.Emitter.close()` now sends `'R'` (clears knobs) **then `'H'`** — no harness exit leaves
  the emitter at 200 silently (that silent restore cost a live debugging session).
- **The stepfpga rejoins at 115 after its staged retune is built** (s7.v commit `5d2532c`, one-constant
  sample-clock change, `+define+CHIP200` restores flight rates): operator will move the FPGA to the
  Windows box for the Diamond build when convenient. After flashing: `regression.py` re-baseline, and
  flip `BEACON_SAMPLE_HZ` default in `host/beacon_telemetry/frame.py` from 480 to 276.
- Production rates (200 Hz / 453 fps) return **with the Pi 5**, as a config + `CHIP200` flip.

Verified live after the change: 118/118 scope paints LOCKED, q=1.00, **lock_health 0.99** (the
peak-pixel fix at work), chip pinned at 115.00.

## 042 STAGE 1 EXIT MET single-code (2026-08-19): live ASCII scope at 10 Hz, cold acquisition, 0.000 % deadline misses

`beacon_trackd` (C11 engine: corr/track/bank/agc/sched) running LIVE on the 3A+ against the code-B bench
emitter ('H' 115 Hz), streaming 20 Hz records over `tcp:4242` to `ascii_scope.py` on the dev box:
**355/356 scope paints LOCKED** (mean q 0.98) at M2 (7.8, 16.2) — native (336, 232), the independently
verified beacon pixel — chip 115.00 Hz, fine scale, K=31, `MEASURED_FIX` set. **Cold acquisition** (blink
detect through the sched budget, no manual seed) locks in a few hundred ms. **Deadline (§11.1): 0/600
misses, margin min/median +18/+35.7 ms** — under the <0.1 % bar. Replay parity is a golden test (two runs
byte-identical over a tracked clip) and the same binary cold-acquired code B from a REAL recorded burst
clip at q=0.99.

**The live-tuning war stories are recorded in-source (track.c/engine.c comments); the ones that will bite
again:**
1. **Verify the Pi actually rebuilt.** Two debugging rounds (~45 min) were spent "fixing" a binary that
   was never rebuilt — `cmake --build --target beacon_trackd` printed "Built target" while running stale
   code. `touch` the edited file or check for "Building C object" lines. The final fixes all worked on
   the first properly-built run: **98.3 % lock duty, chip solid at 115.0**.
2. **DPLL rate adjustment is PARKED (phase adoption only).** Two live attempts destabilised it (naive:
   walked 115→109 Hz because a rate change rebases `corr_chip_at` across the whole epoch; epoch-re-anchor
   repair: sprayed 112→129 Hz). Nominal-rate tracking holds q=1.00 — +0.7 % emitter offset accrues 0.2
   chip over a K=31 window. Rate tracking returns as SIM-FIRST designed work against golden clips.
3. **Innovation gating is load-bearing indoors**: LED-lamp PWM makes the scene breathe in-band (trap #4)
   and an aperture-edge pixel occasionally out-peaks the beacon; ungated, one teleport fix kicked v and
   HOLD extrapolated off the field. Gate = 4·cep floored at 2 M2 px; a wild fix is a coasted tick.
4. **lock_health watches the last measured PEAK pixel** — the aperture-centre version dipped 0.4→0.2 on
   sub-pixel straddle with q=1.00, sent healthy tracks to HOLD, and the HOLD-widen bin reset then ate the
   150 ms re-affirmation window. lock_health is reported but does NOT yet demote (q-only HOLD gate) —
   spec §2.6 wants it driving the decision; it earns that vote back with the estimator work.
5. cep must use the same weights as the centroid (flat-top mixed weights saturated cep→256 px and the
   HOLD cep bound killed tracks in one tick); acquisition episodes must reset on EMPTY passes too, or
   the first pass (which only primes the diff plane) burns the budget forever.

**Open items for the tail of Stage 1**: T044-T047 (oracle/inject/score + the slew-clip envelope run),
T050-T053 (RANSAC proto-track, decode-along-track, acquisition off-thread), T057/T058 (two-code — gated
on the flight cube; false-alarm run), lock_health estimator maturation, DPLL rate loop (sim-first),
T066-T070 polish (golden vectors from live, retire beacon_track.py, regression entry point).

## 042 US1 recorder bench-verified (2026-08-19): burst-to-SD at sensor rate, code B decodes from the clip

`beacon_record` (C11 recorder, libcamera Request-based source) verified end-to-end on the 3A+
(`pi@100.87.61.53`): **20 s burst capture straight to SD** (80 frames / 500, 640×400 @ 276.5 fps, 200 µs /
gain 4) → 889 frames, 12 bursts, **intra-burst dt 3.642 ms mean = 274.6 fps ≈ sensor rate**, zero drops
inside bursts. Clip pulled to the dev box: two replay runs byte-identical; an **independent Python codec
(zero shared source)** decoded every frame — the container contract holds at arm's length — and **code B
correlated at +1.000 within a single recorded burst** (chip 116.4 Hz). T025 CLOSED.

Findings that cost time, so they are traps now:
1. **The dmabuf mmap is write-combine: one uncached 256 KB read ≈ 4 ms on the A53** — slightly over the
   3.6 ms frame period. Consequences: (a) continuous full-raw on the 3A+ tops out ~215 fps — a HOST limit,
   not a bug (continuous is the Pi 5 flight mode; the bench records bursts); (b) bufferCount=16 is
   REQUIRED — 8 buffers (29 ms slack) lose frames at burst tails, 16 (58 ms) absorb a full 80-frame
   burst's copy deficit. For the tracker this is also the front-end budget reality: reduce-in-register
   (read once, write 1/16th) fits; anything that memcpys full frames does not.
2. **`metadata().sequence` on this pipeline counts DELIVERED frames, not sensor frames** — "0 seq gaps"
   proves nothing about drops. Timestamps are the only honest witness (`bcnr_info` dt stats).
3. **SCHED_FIFO on the capture loop made it WORSE** (204 fps vs 216) — it starves libcamera's own
   delivery thread. RT hygiene stays best-effort/warn; do not "fix" it by escalating priority.
4. The recorder is two-threaded (capture → 48 MiB queue → writer): the first single-threaded version lost
   22 % of frames even to tmpfs (the 4 MiB flush pwrite stalls capture ~40 ms = 11 frames). Queue-full
   drops are COUNTED (`frames_dropped`), never silent.
5. Pi toolchain: cmake 3.31.6 + libcamera-dev 0.7.1 installed 2026-08-19 (`sudo apt install cmake
   libcamera-dev`). Build: `cmake -S . -B build -DBEACON_RECEIVER=ON -DBUILD_AUTOC=OFF`.

## 4.7 µH flight-inductor bring-up (2026-08-09): inductor GOOD, bench-supply wiring was the problem

Swapped the 22 µH bench brick for the SPM4020T 4.7 µH (C-9 flight part) in the LM3410X boost. Findings:
- **On the 1S LiPo: starts and runs clean.** On the SPD1168X bench supply: NO START — MCU in fast-flash
  "quiver" (BOD brownout boot-loop). Raising the CC limit did NOT fix it. Scope showed **volts-class
  ringing on the input bus** (−1.5/+3 V vs rail, at only 50 mA drive; measured with a ground-clip probe —
  re-check tip-and-barrel before trusting the amplitude).
- **Discriminator: LiPo clipped onto the SAME bus → ring GONE entirely.** Diagnosis: classic input-filter
  instability — the boost is a negative-incremental-resistance load; PSU lead inductance (~µH) + small
  input C = underdamped tank the converter pumps. 4.7 µH raises ripple ~5× over 22 µH and crosses the
  threshold; battery milliohms damp it dead. NB a lone extra low-ESR cap HELPS NOT AT ALL (raises tank Q —
  observed); the fix shape is a DAMPED leg (ganged 10 µF electrolytics — their ESR is the damper) +
  ceramics at VIN + short twisted leads.
- **Policy**: bench PSU-fed work runs the **22 µH** (or battery-parallel float, supervised); the
  **4.7 µH is VALIDATED on its actual flight source**. Cube-tile build spec picks up: 4.7 µF ceramic +
  100 nF at LM3410 VIN, damped bulk leg, short battery leads. `psu.py` MAX_CURR deliberately raised
  0.5 → 1.3 A (dated comment; start-with-headroom-then-re-arm procedure) for the coming 306 mA era.

## Regression on the DGX-native bench (2026-08-08, post circuit clean-up): 17/19 PASS

**The bench now runs NATIVELY on the DGX Spark** (no WSL interop): STEPLink CDC = `/dev/ttyACM1`
(monitor.sh grew a native branch — `COM3` maps to `$BCN_PORT`, default ACM1), mEDBG = `/dev/ttyACM0`,
PSU via pyvisa-py (udev rule `99-beacon-bench.rules`: f4ec + 03eb + hidraw 0666), venv `~/.venvs/avr`
(pyserial/pyvisa-py/pyusb/pymcuprog), `regression.py` preflight skips usbipd when absent.

**Result 17/19**: P0 cold start ✓ (4.200 V / 91 mA — matches the parked baseline), P1 lock 100 % margin 8
corr 16.2k, P2 dropout ladder ✓ (envelopes identical), **P3 step settle 0.15 s ✓ — 2× better than the
0.30 s bar** (AC-coupled front end pins the decoder pedestal at VBIAS), P4 injection/skew ✓, P5 I-V ✓,
P6 UVLO trip + UPDI recovery ✓, P7 restore ✓.
**FAILs = the two deep-attenuation rungs only**: P3 P=32 (lock 51 %, margin 4) and P=16 (61 %, margin 4)
vs the WSL-era 100 %-at-3 %-duty envelope. Corr scales linearly with duty (16.2k×32/128 ≈ 3.6k measured ✓)
— the decoder is fine; the weak-signal MARGIN is ~one notch lower than the old rig. Two suspects, both
receiver-side and expected: (a) geometry — corr baseline 16k vs the historical 29k best; (b) **the bench
PD is now the unfiltered BPW34** — indoor LED-lamp PWM lands in-band and inflates the AGC energy
denominator exactly where signal is weakest (the old BPV10NF's 780–1050 package filter hid it).
**Operator context (2026-08-08): the run geometry was IMPROVISED** — emitter pointing right, lens/filter
loose over a face-UP sensor, lots of light leakage → the coupling was indirect/bounce with ambient pouring
in around the optics. Margin 8 + clean recovery through THAT is a strong showing; the two deep-attenuation
rungs are not diagnostic of the real chain. **Deep-attenuation characterization + the P-ladder
discriminator are DEFERRED to the field unit** (copper-clad cage, light-sealed, boresighted) — re-run the
full ladder as the first act on that build and expect the WSL-era envelope back.

## Current state (2026-07-18)

- **Optical link WORKS end-to-end and is characterized**: LiPo-capable emitter (ATtiny416 XNANO eval @
  10 MHz, UVLO 3.48 V + WDT + BOD 2.6 V) → LM3410X boost → 5× L1IZ-0850 @ 51 mA bench → air → BPV10NF →
  MCP6022 TIA (breadboard) → MCP3201 (IN−=GND) → StepFPGA **s7** correlator. Best optical: margin 9,
  corrB 29.4k; **41 ft (~12.5 m) locks bare-to-bare** (decoder floor ≤10 nA). Full story:
  [optical-link-outcome.md](optical-link-outcome.md); hardwired-era: [s6-closed-loop-outcome.md](s6-closed-loop-outcome.md).
- **Full bench regression is automated and POLICY**: `firmware/beacon-decoder-stepfpga/host/regression.py`
  — 19/19 PASS maiden run (2026-07-18). **Run after ANY emitter-firmware or decoder-gateware change.**
  Covers PSU cold start → baseline → dropout ladder → AGC ladder/step → injection/skew → I-V profile →
  deliberate UVLO trip → UPDI recovery → restore.
- Bench parked: locked margin 9, supply 4.200 V / ~91 mA, output ON.
- **Regression re-baselined 2026-07-26 on the Option-C rig ("current values" breadboard): 19/19 PASS**
  ([results/regression-2026-07-26.log](../../firmware/beacon-decoder-stepfpga/host/results/regression-2026-07-26.log)).
  Baseline lock 100 % margin 8 corr ~10.5 k; dropout ladder clean (1 s → 18–24 unlocked frames, 4 s
  re-acquires); AGC step settle **0.30 s** (vs 0.62 s at s7 bring-up); locks at P=16; +2.6 % skew holds;
  I-V 4.2→3.7 V no LOS; UVLO trips + UPDI recovery clean. First attempt died at P3 on a truncated
  telemetry row — `regression.py capture()` now skips malformed rows (try/except) instead of crashing;
  also note `tee` masks the suite's exit code — check the `19/19` line, or run with `PIPESTATUS`.

## Instruments (all scriptable from `firmware/beacon-decoder-stepfpga/host/`)

| instrument | driver | notes |
|---|---|---|
| Siglent SPD1168X supply | `psu.py` (pyvisa-py USBTMC) | **guardrails MAX_VOLT 4.5 / MAX_CURR 0.5** (REFUSES; raise deliberately for 306 mA field ≈1.3 A). Quirks: ~0.5 s cmd pacing; abandoned query wedges TMC → `usbipd detach/attach`; pyvisa addr uses DECIMAL VID/PID (`USB0::62700::5136::…`). udev: `99-siglent.rules` (f4ec 0666). |
| Emitter cmd link | `recovery_sweep.Emitter` (pyserial, **DTR required**) | 38400 8N1 on the mEDBG CDC (`firmware/beacon-pod/attach-medbg.sh` → `/dev/ttyACM0`; auto-reattach built in). Cmds: `R` nominal, `F` rate (⚠ 0.16 %/step @10 MHz — halved vs 20 MHz era), `C` corrupt, `D` dropout (0x1F = full stop), `P` pulse width (**BENCH-ONLY** — production waveform contract = FULL-DUTY chips, camera exposure-phase immunity). Boot banner `'B'+RSTFR hex` per reset (WDRF=08, BORF=02, UPDIRF=20). |
| Decoder telemetry | `monitor.sh COM3 <s>` (Windows-side read) | **one COM3 reader at a time** — a second silently OPEN-FAILs and logs nothing. BCN frame: seq(40 Hz),adc,corrA,lockA,marginA,corrB,lockB,marginB,rateA,rateB,recA,recB. |
| FPGA build/flash | `experiments/deploy_s7.sh` (s7 current) | Diamond via WSL interop; flash = copy `.jed` → STEPLink `D:`. HDL sim: `sim/run.sh` (iverilog, `ifdef SIM` ÷100 clocks). |
| Firmware build/flash | PIO `xnano416` env + pymcuprog | `pymcuprog reset -d attiny416` = **the un-latch** (see traps). BOD fuse recipe in `firmware/beacon-pod/SETUP.md`. |

## Traps (each cost real time — check before debugging "mysteries")

1. **mEDBG trickle-latch**: with the XNANO USB attached, supply-only power cycles CANNOT clear a UVLO-latched
   chip (bridge-pin leakage preserves the sleep). Un-latch = `pymcuprog reset` or unplug USB. (SETUP.md)
2. **mEDBG drops off USB randomly** — retry `attach-medbg.sh` immediately; harnesses auto-reattach.
3. Receiver front-end traps (unpowered-amp passive decode, PD orientation, MCP3201 IN− ±100 mV spec, TIA
   oscillation without C_f, trimpot/cap discipline): the table in
   [optical-link-outcome.md](optical-link-outcome.md) §front-end bring-up traps.
4. **A hand is a ~20 dB attenuator, not a shutter** (locks through tissue at 1 m); **indoors is an
   integrating sphere** (ceiling-bounce locks) — true-dark tests need emitter-off or a capped sensor.
5. usbipd bind targets a BUSID — devices swap busids across replugs; verify against `usbipd list` before any
   `bind --force` (a mis-bind once ate COM3).
6. **NIR viewers are not equal (2026-08-10)**: the Andonstar microscope camera shows 850 nm as BRIGHT
   purple (weak IR-cut) — it is the bench's NIR eyeball; the **iPhone attenuates 850 heavily** (good
   IR-cut) — a dim phone image does NOT mean weak emission (this dimmed the card-projection tests and
   understated the 16 mm lens all along). Emission checks → microscope camera, not phone.

## Decoder knowledge (s7 gateware)

- AGC = energy-normalized quality (excellent: 100 % lock at 3 % duty) + **gear-shifted DC tracker (s7)**:
  α=1/256 locked (τ=533 ms) / **1/32 unlocked (τ=67 ms)** — HW step settle **0.62 s** (s6 was 1.23 s; s6-era
  curves in `host/results/agc_step.log`). Min-energy lock gate (beste ≥ 64) kills dark-input false locks.
- Occlusion: ≤1 word rides through in HOLD (300–325 ms absorption); warm relock ≲1 period; no ratchet
  (5 geometries of `recovery_sweep` CSVs in `host/results/`).
- **s7 SHIPPED 2026-07-18** (`experiments/s7.v`, sim `sim/tb_s7.v`): A4d-8 gear-shifted α + A4d-7 min-energy
  gate — sim-first (0.81 ms settle, zero dark flashes), regression 19/19 with the settle bar ratcheted <1.0 s.

## Daylight receiver: OPTION C SELECTED (2026-07-24, bench-verified)

**Reverse-biased PD + load R + single SBOA224-style gain stage** — the doc
[cad/beacon-receiver/daylight/collector-schematic-daylight.md](../../cad/beacon-receiver/daylight/collector-schematic-daylight.md)
was **restructured 2026-07-25 to be Option-C canonical**: final values with time constants sized to the code
(f_hp 10.8 Hz ≪ chip band 30–200 Hz; gain-leg 16 Hz; anti-alias 1.9 kHz ≈ 10× chip; step recovery
~65–80 ms ≪ 155 ms word), ambient-pedestal ceilings (70 µA @ 47 k → pair R_load with the C-14 filter: 10 nm
→ 47 k, 40 nm → 22 k), clean netlist + KiCad sync checklist (two-stage TIA → Appendix B fallback). KiCad
sheet (`daylight/beacon-receiver-daylight.kicad_sch`) still carries pre-wiring [OPT-C] value tags — manual
rewire per the netlist is the next CAD step. Bench: strong close-range drive + wall-bounce decode with
substitute values — topology-tolerant. **NEXT FIELD TEST RUNS OPTION C** — bring the rig (COM3 `adc` =
pedestal meter; note `adc` is AC-stripped now — scope N_PD for the raw pedestal). Gain leg needs R5 = R6/100
(×2 symptom = equal resistors). Orders: **02+03 COMBINED into the single active
`cad/beacon-eval/beacon-order-03.md`** (C-1..C-25; order-02 deleted; filter FWHM 40-vs-10 nm = open decision
C-14, now coupled to R_load). **Reconciled 2026-07-26 vs the two DK packlists + operator inventory answers — cart is FINAL**: dropped
C-3/C-5/C-7/C-13/C-17/C-18/C-25 (survivor counts OK, no 412s owned, wire + 1N4148 + 3.74 Ω on hand);
**C-12 FIRM ×10 — the 5-LED bench string stays bench permanently, the flight cube gets its own LEDs**;
confirmed shorts = SOT23→DIP adapters (C-6 ×10) + 1 broken XNANO (C-4 ×2).

## Field test #4 (2026-08-02, outdoors, ~6× emitter current): AMBIENT COMPRESSION IS THE WALL — current doesn't buy it back

Rig: **6 parallel LEDs @ 50 mA each (~300 mA total)** "pointing in the general direction" (not boresighted
per-LED), bare PD, no optical filter — i.e. the field-current multiplier from the test-#3 roadmap, applied
ahead of the filter/optics stages. Operator result:

- **PD shaded (hand/body shadow) but in an otherwise sunlit outdoor scene → locks to ~20 ft (~6 m).**
- **PD exposed to direct sun → does NOT work at any distance tested.**
- **Good reflection sensitivity** — bounce/off-surface returns decode readily, so raw sensitivity is intact.
- **AGC feels a little slow** in the field (subjective — see open item below).

**Read**: this is the compression model in the daylight doc doing exactly what it predicts. Ambient forward-biases
the PD, its dynamic resistance collapses, and beacon current gets **shunted at the sensor** — a loss that sits
*upstream* of every downstream multiplier, so **×6 emitter current cannot buy back a direct-sun failure**. The
shaded-vs-exposed split is the cleanest evidence yet that the limiter is ambient at the PD, not link budget:
same emitter, same distance, shadow alone flips it. **C-14's bandpass filter is therefore not an
optimization — it is the gate on every remaining range multiplier.** Optics (C-14 lens) multiply signal but do
nothing about compression; the filter is what restores headroom, and only then does the ×6 current show up as range.

⚠ **Confounds to resolve on the next run — do NOT treat this as a regression vs test #3 yet** (which logged
15–20 ft with the sensor *in* direct sun at 51 mA):
1. **Was ~20 ft the signal limit or the yard limit?** If the operator stopped walking, the number is a floor,
   not a measurement. Pace out past loss-of-lock next time.
2. **Effective on-axis current is NOT 6×.** With each LED aimed "generally" rather than boresighted, the
   received flux is the sum of six off-axis contributions — plausibly ~1–3× of a single aimed LED, not 6×.
   Boresight the cube (or measure one LED aimed vs six splayed at fixed range) before crediting/faulting ×6.
3. **Ambient was likely higher** (2026-08-02 conditions vs 07-26) and the PD/values may differ from test #3's
   as-measured build (R_load 47 k trimmer — confirm it still reads 47 k; BPV10NF vs BPW34).
Until 1–3 are pinned, the defensible claim is the **shaded/exposed split**, which stands on its own.

## Field test #2 (2026-07-26, outdoors, Option C with substitute values): compresses in sunlight — VALUES, not topology

As-built rig (on-hand parts): **R_load 1 MΩ** (not 47 k), C6 0.1 µF, **R4 10 kΩ** (not 100 k), **R6 1 MΩ ∥
4700 pF** (not 100 k ∥ 820 pF), R5 1 k + C8 10 µF. Result: sensor in direct sunlight (sun ~90° off
boresight) = barely locks; marginal-but-real lock on strong signal. Diagnosis (all three deltas point the
same way, math in the daylight doc):
1. **1 MΩ load → ambient ceiling 3.3 µA** (vs 70 µA @ 47 k): outdoor ambient forward-biases the PD, its
   dynamic resistance collapses to ~10²–10³ Ω and SHUNTS the beacon current ~40×+ — compression, not gain,
   is the failure. Indoors (≤1 µA) sits just under the ceiling — hence the clean bench curve.
2. **R4 10 k AC-loads the PD through C6** → in-band transimpedance ≈10 k regardless of the 1 M (which only
   sets the DC point). This build also exposed a doc error: canonical in-band I→V is **R1∥R4 ≈ 32 k**, not
   47 k (budget table corrected 2026-07-26).
3. **1 M ∥ 4.7 nF = 34 Hz LP** — chips smeared to τ ≈ 1 chip; net equivalent ≈1.6 MΩ @ 200 Hz ≈ v1 gain
   (familiar curve, correlator processing gain carried it). Plus τ_hp ≈ 100 ms (attitude blindness).
   Verdict: **topology field-validated; go canonical values** (47 k / 100 k / 100 k ∥ 820 pF) for
   sunlit-sensor operation; **C-14 filter still required for sun near the FOV** (bare-PD 30–300 µA → ÷7–25
   brings it inside the 70 µA ceiling).

**Field test #3 (same day, canonical values fitted)**: R_load = **47 k MEASURED post-test** (1 M 3296W
trimmer, holding value), R4 100 k, R6 100 k ∥ 800 pF, R5 2.2 k (×46, two-emitter-safe). **DIRECT-SUNLIGHT LOCK:
~15–20 ft (4.5–6 m) outdoors, bare round PD (BPV10NF), NO optical filter, bench current 51 mA.**
Hand-shadowing the PD from direct sun improves signal → residual ambient compression is the remaining
limiter, i.e. exactly the C-14 filter's job. Matches the corrected budget (~5 m ≈ 100-count class at ×46
before daylight shot noise). **AC-coupled Option C is FIELD-PROVEN**; remaining range multipliers to 100 m:
field current ×6 (range ×~2.4) → filter (kills compression + √7 shot noise) → collection optics (×10–25
signal → range ×3–5). Scope note: BK 2120B + 10× probes (screen ×10; 1× probing would load the 100 k
bias node — don't).

## Field finding (2026-07-17, outdoors, sunny): NO LOCK even at inches — TIA RAILED by ambient

Expected physics, now measured: full sun ≈ 15–20 mW/cm² in the PD's wide 780–1050 band → **0.3–1 mA ambient
photocurrent** vs a 1 MΩ DC-coupled TIA → output rails at 3.3 V, ADC pegged, no decode at any distance.
AC-coupling alone can't fix a railed amp — the day-one hypothesis was: **FB850-10 optical filter (÷~25
ambient, order-03 O3-11) + lower first-stage R_f (~47–100k for headroom) + AC couple (C2/R4) + U1B post-gain
(×20–50, the MCP6022's unused half)** *(superseded 2026-07-24 by Option C — see the daylight section
above)*; ambient shot noise then sets the daylight floor (to be measured). No field
instruments that day — NOTE: the rig is portable (laptop + STEPLink + battery TX); the telemetry `adc` column
is a pedestal meter — bring it next time.

## Planned experiments (2026-07-19)

- **Set A (gain vs cap, explains the 16/19 paper-bounce run)**: A1 = 675k no-cap P-ladder; A2 = 1 MΩ +
  100 pF P-ladder. Prediction: A2 recovers margin 6/100 % (cap = anti-aliasing at the 480 Hz point sampler);
  A1 barely moves (margin is gain-normalized).
- **Set B (DC-bias / synthetic sunlight) — reframed 2026-07-25 to run on Option C**: B1 incandescent-lamp
  pedestal ladder (LED lamps emit no 850 nm!) — scope N_PD = pedestal meter (`adc` is AC-stripped in
  Option C), find the margin knee + soft-compression point vs the 70 µA @ 47 k ceiling; B2 shot-noise margin
  curve below compression; B3 clamp-diode A/B (with/without D2/D3 under pedestal steps); B4 indoor-flicker
  station (100/120 Hz + LED-PWM margin cost). Feeds the A3-b soldered freeze + the C-14 filter/R_load choice.
  (The old B3 AC-tap / B4 two-stage line items are superseded — Option C selected.)
- **Set C (SATURATE-vs-DARK at chip granularity) — planned 2026-08-04, feeds open item 2a/2b.** The
  question the lens analysis raised and nothing has measured: **is warm relock from a SATURATED start the
  same as from a DARK one?** Everything characterised so far (occlusion ≤1 word in HOLD, warm relock ≲1
  period, the 5 `recovery_sweep` geometries) entered from *dark*. A sun transit enters from *saturated*,
  and the mechanism below predicts they are not the same.

  **HYPOTHESIS (this is the point of the set — it is falsifiable, and cheap to falsify).**
  - *Dark*: the min-energy gate (`beste ≥ 64`) fires → lock drops → gear shifts **fast** (α=1/32,
    τ=67 ms) → recovery is period-dominated, ≲150 ms. This is the measured baseline.
  - *Saturated*: energy is **high**, so the min-energy gate does **not** fire and the decoder **retains
    (false) lock** — leaving the DC tracker in the **slow** gear (α=1/256, τ=533 ms) at exactly the moment
    it needs the fast one. Recovery becomes τ-dominated, not period-dominated.
  - **Predicted asymmetry: saturated recovery 3–8× slower than dark at the same N.** If the arms come out
    equal, no max-energy gate is needed and item 2a closes for free. If they diverge as predicted, the
    gate is justified by measurement rather than by argument, and C5 sizes it.

  **Chip-granular gating must be FIRMWARE-side.** The Siglent's ~0.5 s command pacing is **50 chips** at
  the 100 Hz chip rate — the PSU cannot produce the transient. Its job here is to set the steady
  saturation *depth*; the emitter MCU produces the event, chip-synchronous by construction.

  - **C1 — dark baseline (mostly free).** Re-run `recovery_sweep` over the N ladder below and confirm it
    reproduces the existing `host/results/` CSVs. Establishes today's decoder as the comparison point.
  - **C2 — saturate arm.** Add an `S` command to the pod firmware: hold the LED **unmodulated ON for N
    chips**, the exact mirror of `D` dropout (same code path, same chip-clock alignment). Emitter at
    inches so the mean current is mA-scale and compresses — per the daylight doc, "at inches it is
    mA-scale and compresses everything". ⚠️ Firmware change ⇒ **`regression.py` before and after**.
  - **N ladder (chips, 10 ms each)**: 1, 2, 4, 8, 16, 31, 62, 100, 150. Brackets both the 300–325 ms HOLD
    absorption (~31 chips) and the two field-relevant sun transits — **16 chips ≈ 163 ms @ 60 °/s** and
    **98 chips ≈ 978 ms @ 10 °/s**. ≥10 reps per cell; report medians, not means (relock is bimodal once
    a word boundary is crossed).
  - **C3 — depth sweep (S only).** Saturation depth as a multiple of the 70 µA @ 47 k compression ceiling:
    ~1×, 3×, 10×, 30×, set by PSU current + standoff. Establishes whether recovery time depends on how
    *hard* it saturated or only on how *long* — the first is a soft-recovery story, the second a
    state-machine one, and they want different fixes.
  - **C4 — instrument the MECHANISM, not just the outcome.** Log `lockA/marginA/adc` **through** the event,
    not merely after it. The hypothesis stands or falls on whether `lockA` stays asserted during
    saturation; that single bit is the whole argument. If lock drops on its own, the min-energy gate is
    already covering this case and 2a is moot.
  - **C5 — max-energy gate A/B (only if C2/C4 confirm).** Add the gate, re-run C2 identically, and report
    the recovery-time delta. Sizing question it must answer: what energy threshold fires on the sun but
    never on a legitimate close-range pass, where the beacon's own mean current is *also* mA-scale (the
    daylight doc's "back off to ≥1 m for linear measurements" caveat is exactly this collision).
  - **Faithful variant, no new hardware (do after C2).** C2 saturates with the code *absent*, which models
    "sun swamps everything". The truer sun case is code *present but buried*. The decoder already tracks
    two codes (`corrA/corrB`) — drive pod A with the code under test and hold pod B unmodulated as the
    aggressor, then watch `lockA/marginA` survive or not. Same instruments, no aggressor LED to build.

### C1 RESULT — dark baseline measured 2026-08-04 (Ubuntu host, synthetic channel A)

Runner: [`host/set_c.py`](../../firmware/beacon-decoder-stepfpga/host/set_c.py) · data:
`host/results/set_c_dark_chA.csv` (18 trials, 3 reps). Stimulus = one-shot `enA` clear on the decoder's
**synthetic** channel A (operator direction: code B's optical amplitude is not repeatable enough).

| span | duration | outcome | unlock span | relock AFTER restore | min marginA |
|---:|---:|---|---:|---:|---:|
| 16 chips | 80 ms | rode through | — | — | 2–4 |
| 31 chips | 155 ms | rode through | — | — | 3 |
| 62 chips | 310 ms | **rode through** | — | — | 1–3 |
| 124 chips | 620 ms | unlocked | 300–325 ms | −10 … +15 ms | 1 |
| 196 chips | 980 ms | unlocked | 625–775 ms | −45 … +105 ms | 0–2 |
| 400 chips | 2000 ms | unlocked | 1675–1850 ms | −15 … +160 ms | 0 |

**Two numbers worth keeping.**
1. **HOLD absorption is bracketed to 310 ms < x ≤ 620 ms** — 62 chips rides through, 124 does not.
   Independently reproduces the previously logged 300–325 ms from a different stimulus path.
2. **Warm relock from DARK is essentially immediate: 0–2 frames after signal returns.** Every rung's
   unlock span is just `duration − ~310 ms of HOLD`, with nothing left over. This is *tighter* than the
   journal's earlier "≲1 period" (155 ms) claim and it is the number C2 has to beat. **If the saturated
   arm shows any non-zero recovery after restoration, that delta is the AGC windup, isolated.**

`min marginA` degrades monotonically 4 → 3 → 1 → 0 with span, confirming the stimulus is graded and real
rather than a step function.

### C2 IS BLOCKED, and the fix is one bit of gateware

Muting code A drives `injA` to **MID** (mid-scale) — a clean signal removal with **zero DC step**. Real
saturation is a large DC step the tracker must chase. So no existing knob produces the C2 stimulus:
amplitude `'A'` moves HI/LO separation *symmetrically about MID* (no DC shift), and `'K'` blanks to MID
like everything else. **The dark and saturated arms are not distinguishable with today's bitstream.**

The change is minimal and reuses the existing chip-exact burst machinery. `s7.v` mask bits **2 and 6 are
unallocated** ([0]enA [1]enB [3]inj1 [4]inj2 [5]weak). Take bit 6 as `blank-to-rail`:

```verilog
wire erail = remote & cmd_reg[6];              // NEW: blank to rail instead of mid
wire [11:0] injA = (enA & ~blankA_rx) ? (codeA_rx ? hiA : loA)
                                      : (erail ? 12'hFFF : MID);   // was: MID
```

That makes C1 and C2 **the same chip-exact stimulus differing by one bit**, which is exactly the
controlled comparison Set C wants — better than the pod-firmware `'S'` command originally planned, since
it needs no emitter change and inherits `'K'`'s chip-exact timing. The `'S'` route stays valid for
end-to-end optical work later.

⚠️ **Neither can be built on the Ubuntu host.** Diamond (gateware) and the pod toolchain both live on the
Windows box per [`docs/toolchains.md`](../../docs/toolchains.md); Ubuntu has the hardware but not the
builders. C2 is gated on a Windows-side rebuild + flash, then `regression.py` per policy.

### Host-portability findings (cost real time — folded into Traps by reference)

- **Every existing harness entry point is Windows-bound**: `monitor.sh` and `cmd_read.sh` shell to
  `powershell.exe` against COM3 via `/mnt/c`; `regression.py` calls `usbipd.exe`. None exist on Ubuntu, so
  the *measurement* side of `recovery_sweep`/`regression` is unreachable there. `set_c.py` reads the CDC
  directly and is the pattern for porting the rest.
- **Ports are inverted on Ubuntu.** `/dev/ttyACM0` = ARM DAPLink = **decoder** (was COM3);
  `/dev/ttyACM1` = ATMEL mEDBG = **emitter**. `recovery_sweep.py --port` defaults to `/dev/ttyACM0`,
  which on Ubuntu sends emitter commands to the decoder.
- **`'+'` alone disables both correlators.** `cmd_reg` powers up at 0, so REMOTE with no mask sets
  enA=enB=0. Send `0x80|mask` **first**, then `'+'`; restore with `'-'`.
- **Channel A does not respond to the pod.** corrA ≈ 66000 is the synthetic injected channel; corrB ≈ 1200
  is the real optical one. `recovery_sweep` measures `lockB` for that reason. A first attempt here keyed on
  `lockA` against a pod blanking command and reported "rode through" on every rung — a null result that was
  purely an instrumentation error.
- **A hand-rolled emitter write is not safe.** `recovery_sweep.Emitter.cmd()` verifies the command echo and
  re-attaches the flaky mEDBG; a plain `write()` silently "succeeds" when nothing landed.

## Open items / next steps

1. **Order-03** ([cad/beacon-eval/beacon-order-03.md](../../cad/beacon-eval/beacon-order-03.md)) — **mostly
   landed**: AMZ-1 (2026-07-30: adapters, R/C kits, FR4, M12 holder, breadboards) + DK-3 (invoice 129837577,
   2026-08-02: BZX55C15 ×10, 1 k, XNANO ×1, **4.7 µH SPM4020T ×9 of 10 — 1 backordered, separate shipment**,
   Molex UMX ×10, L1IZ-0850 ×10). **Sections A (OVP) and C (flight cube) are fully sourced — buildable now.**
   **C-14 lens ARRIVED 2026-08-05** — barrel: `16mm 5MP 1/2.5" IR`. Day-one findings: (a) **integrated
   filter confirmed** — opaque to visible AND passes 850 (emitters viewed through lens + microscope glow
   the classic Bayer purple/magenta = strong NIR transmission → **lens-arm checklist item 3 CLOSED
   functionally**; FWHM still unknown, assume 40–60 nm camera-class); (b) 16 mm + BPW34 2.65 mm die →
   **9.5° field — matches the 9.8° assumed by `lensed_pd_range.py`**, so the 2b sun-transit table stands;
   (c) 1/2.5" image circle (~7.2 mm) comfortably covers the 3.75 mm die diagonal — no vignetting;
   (d) visible-blindness confirms corrB-peak focusing is the only focusing method (checklist item 4).
   **NEXT = checklist item 1: functional aperture-gain measurement** (bare vs lensed photocurrent, same
   emitter/distance). Still out: separate 850 nm bandpass (if still wanted given the integrated filter),
   C-24 flight pack, gated F-section parts.
2. **AGC speed — quantify (deferred, operator 2026-08-02)**: field impression is "a little slow" while the
   bench measures **0.30 s step settle** (regression 2026-07-26, vs 0.62 s at s7 bring-up). Suspect the
   field case is not the bench case: outdoor pedestal steps (sun/shadow/attitude) are far larger than the
   `agc_step` ladder, and near compression the loop chases a *shunted* signal. Plan: log the AGC state +
   pedestal together during the next outdoor run, then extend the ladder to field-sized steps before
   touching s7's gear-shift α.
   - **2a. MAX-energy gate — new work item (2026-08-04, from the C-14 lens analysis).** Once the lens is
     fitted, a **sun transit through the field is unsurvivable by pedestal design** — 209–952 µA
     depending on filter FWHM, over every R_load ceiling (daylight doc pedestal table, lens-era note).
     The transit itself is not the problem; **AGC windup during it is.** The gear-shifted DC tracker
     (α=1/256 locked, τ=533 ms) would chase a ~10⁵× pedestal and then need ~533 ms to unwind — an order
     of magnitude longer than the event that caused it. **Fix shape: a max-energy gate that FREEZES the
     AGC and asserts HOLD**, the exact mirror of s7's existing min-energy lock gate (`beste ≥ 64`) that
     kills dark-input false locks. Then a sun transit is handled by the same machinery as a propeller
     occlusion, which is already characterised (≤1 word in HOLD, warm relock ≲1 period).
   - **2b. Transit duration is FOV-linear, and the C-14 field is not narrow enough at low body rates.**
     At the BPW34 focal-plane field of **9.8°** ([`lensed_pd_range.py`](lensed_pd_range.py) §8):

     | body rate | blanked | vs the 300–325 ms HOLD absorption |
     |---:|---:|---|
     | 10 °/s | 978 ms | **EXCEEDS — lock lost** |
     | 20 °/s | 489 ms | **EXCEEDS — lock lost** |
     | 60 °/s | 163 ms | rides through |
     | 120 °/s | 82 ms | rides through |

     So slow, sun-ward passes are the worst case, not aggressive ones. Either extend the HOLD budget to
     ~1 s for the max-energy case specifically, or accept a reacquire and measure how long warm relock
     actually takes from a saturated start (it is **not** the same entry condition as a dark occlusion —
     worth its own bench arm). A narrower field shortens this linearly; the BPV10NF's 3.2° would cut it
     ~3×, but that PD is the finder, not the focal-plane sensor.
   - **2c. Down the line: the decoder should CONTROL the camera** (operator 2026-08-04). Same loop, one
     level up — once the array is in, exposure/gain become decoder-driven rather than free-running, so
     the max-energy gate and the exposure command are one control problem, not two. Camera-side
     prerequisites (global shutter, manual exposure) are now **requirements** on the D1 line in
     [`verified-bom.md`](verified-bom.md).
3. **A1-ovp**: zener + 1 k clamp parts are IN HAND (DK-3) — fit the 15 V zener + 1 kΩ clamp; pull-LED-header
   live test. No longer blocked.
4. **A3-b**: soldered **Option-C** receiver on copper-clad (**FR4 stock in hand — AMZ-1 C-8/C-20**; netlist
   in the daylight doc; R_load per the C-14 filter choice; star ground) → re-run regression.
   - **Receiver-head cage (operator plan 2026-08-04, pre-lens)**: BPW34 in a **light-tight enclosure,
     black except the lens aperture** — copper-clad as the mount **tied to GND** (shields the 47 k PD
     node; bare BPW34 is unfiltered 430–1100 nm and demonstrably picks up LED-lamp PWM on the bench),
     **matte-black interior** (the decoder locks on nA of internal bounce), aperture cappable (= the
     true-dark test fixture the traps list wants). Filter sits BEHIND the lens (F/2 cone ±14° vs full
     field angle — AoI blue-shift math in `lensed_pd_range.py`).
   - **Bench note 2026-08-04 (canonical-values rebuild, BPW34 bare)**: works indoors; LED-lamp PWM now
     visible (no package filter — BPV10NF's built-in 780–1050 nm hid it, but that 270 nm window is also
     why bare sun was fatal in the v1 era); strong-drive waveform = bottom-rail clip first (1.25 V down
     vs 2.05 V up headroom) + AC-coupler ride-up — looks alarming, is correct polarity overdriven.
     PD-orientation check that settles it in 30 s: PD_NODE ≈ 0 V dark rising with light (backwards =
     pinned ~2.7 V, near-zero signal). Full tune deferred until the C-14 lens+filter land.
5. **A2-uvlo-2**: scripted slow-creep + dwell micro-dropout sweep (psu.py ramp + banner listener) — the
   original 3 blips didn't reproduce in the fast static-hold pass.
6. Flight cube build @306 mA (raise psu MAX_CURR deliberately) — **parts in hand; boresight the 5 tiles,
   don't splay them (field test #4 confound 2)** → optics stage → **100 m field test**.
7. **Next-field-run checklist** (from field test #4): pace out past loss-of-lock instead of stopping at the
   yard edge; confirm the R_load trimmer still measures 47 k; record which PD is fitted (BPV10NF vs BPW34);
   scope N_PD for the raw pedestal in both shaded and sun-exposed states; log AGC state alongside.
   - **Added 2026-08-04 (C-14 lens arm)** — the first lensed run should close these, in this order:
     1. **Measure the aperture gain FUNCTIONALLY — do not derive it.** Same emitter, same distance,
        same PD: record photocurrent bare, then lensed. The ratio folds F/#, lens transmission, filter
        transmission and focus quality into ONE number, and it drops straight into `SignalOpticsGain`
        (040 ships it at 1.0, an untried `A` row; the calibration rehearsal pencils in 4.0). This is
        the single largest uncertainty left — range scales **linearly** with pupil diameter, so the
        F/1.6-vs-F/2.4 spread alone is 115 m vs 77 m for one die at 306 mA.
     2. **The front element is convex** — its diameter is NOT the entrance pupil, so do not caliper it
        and infer F/#. (Item 1 makes this moot, which is why it is item 1.)
     3. **Confirm the ELP integrated filter passes 850** and record whatever FWHM the vendor will
        admit to. Camera-market "narrow pass" is typically 40–60 nm, not 10 nm — which is the RIGHT
        width behind a fast lens (verified-bom D5 note), but it sets the sun pedestal.
     4. **Focus on the beacon itself, not by eye.** The integrated filter blocks visible, so there is
        no visible image to focus on — which also retires the NIR focus-shift trap. `corrB`-peak
        focusing per C-14 is the method.
     5. **Focus, then back off to ~30–50% of the die.** Total photocurrent is identical focused or
        defocused (the PD integrates) and ambient is focus-independent, so defocus buys nothing on
        signal — but a slight blur is cheap insurance against die non-uniformity and thermal drift.
        Do NOT defocus to fill the die: that softens and shrinks the effective field.
     6. **Provoke a sun transit deliberately** and log AGC state + pedestal through it — this is the
        entry condition for item 2a/2b above, and it is the one measurement neither the bench nor any
        prior field test has.
8. Camera-era (040) notes seeded in optical-link-outcome.md (multipath, tracker-bank, full-duty contract).

## History (chronological commits of record)

s5 real-ADC decode `1a35c30` → s6 closed loop + RAM windows + sim harness `afc6f69` → receiver bring-up +
harness auto-reattach `60c58c9` → 10 MHz `ca1fe78` → bench findings + OVP + order-03 `2d61e42` → optical
checkpoint (41 ft) `d012eaf` → UVLO implemented `1dbe44e` → verified+banner `07d195f` → closed/calibrated
`f58dcab` → order-03 flight section `49bb4c8` → 'P' attenuator + AGC measured `afcec11` → full-duty contract
`81f9c99` → PSU driver `241c540` + profile `6adad83` + guardrails `1e022f5` → **regression suite `f4b8590`** → s7 gear-shift AGC + energy gate `32a2596`.
