# 031 bench journal — living state (portable across machines)

> **What this is**: the running state of the beacon bench — instruments, quirks, open items, and pointers —
> kept **in-repo** (operator decision 2026-07-18) so any machine/session can pick up the work. Update this
> file as the bench evolves; deep results live in the outcome docs it links. Machine-local assistant memory
> should hold only a pointer here.

## Current state (2026-07-18)

- **Optical link WORKS end-to-end and is characterized**: LiPo-capable emitter (ATtiny416 XNANO eval @
  10 MHz, UVLO 3.48 V + WDT + BOD 2.6 V) → LM3410X boost → 5× L1IZ-0850 @ 51 mA bench → air → BPV10NF →
  MCP6022 TIA (breadboard) → MCP3201 (IN−=GND) → StepFPGA s6 correlator. Best optical: margin 9,
  corrB 29.4k; **41 ft (~12.5 m) locks bare-to-bare** (decoder floor ≤10 nA). Full story:
  [optical-link-outcome.md](optical-link-outcome.md); hardwired-era: [s6-closed-loop-outcome.md](s6-closed-loop-outcome.md).
- **Full bench regression is automated and POLICY**: `firmware/beacon-decoder-stepfpga/host/regression.py`
  — 19/19 PASS maiden run (2026-07-18). **Run after ANY emitter-firmware or decoder-gateware change.**
  Covers PSU cold start → baseline → dropout ladder → AGC ladder/step → injection/skew → I-V profile →
  deliberate UVLO trip → UPDI recovery → restore.
- Bench parked: locked margin 9, supply 4.200 V / ~91 mA, output ON.

## Instruments (all scriptable from `firmware/beacon-decoder-stepfpga/host/`)

| instrument | driver | notes |
|---|---|---|
| Siglent SPD1168X supply | `psu.py` (pyvisa-py USBTMC) | **guardrails MAX_VOLT 4.5 / MAX_CURR 0.5** (REFUSES; raise deliberately for 306 mA field ≈1.3 A). Quirks: ~0.5 s cmd pacing; abandoned query wedges TMC → `usbipd detach/attach`; pyvisa addr uses DECIMAL VID/PID (`USB0::62700::5136::…`). udev: `99-siglent.rules` (f4ec 0666). |
| Emitter cmd link | `recovery_sweep.Emitter` (pyserial, **DTR required**) | 38400 8N1 on the mEDBG CDC (`firmware/beacon-pod/attach-medbg.sh` → `/dev/ttyACM0`; auto-reattach built in). Cmds: `R` nominal, `F` rate (⚠ 0.16 %/step @10 MHz — halved vs 20 MHz era), `C` corrupt, `D` dropout (0x1F = full stop), `P` pulse width (**BENCH-ONLY** — production waveform contract = FULL-DUTY chips, camera exposure-phase immunity). Boot banner `'B'+RSTFR hex` per reset (WDRF=08, BORF=02, UPDIRF=20). |
| Decoder telemetry | `monitor.sh COM3 <s>` (Windows-side read) | **one COM3 reader at a time** — a second silently OPEN-FAILs and logs nothing. BCN frame: seq(40 Hz),adc,corrA,lockA,marginA,corrB,lockB,marginB,rateA,rateB,recA,recB. |
| FPGA build/flash | `experiments/deploy_s6.sh` | Diamond via WSL interop; flash = copy `.jed` → STEPLink `D:`. HDL sim: `sim/run.sh` (iverilog, `ifdef SIM` ÷100 clocks). |
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

## Decoder knowledge (s6 gateware)

- AGC = energy-normalized quality (excellent: 100 % lock at 3 % duty) + **DC tracker α=1/256 @480 Hz
  (τ=533 ms) — the dominant settle constant** (10× down-step: margin craters ~1 s, settles 2.5–3 s; up-step
  0.5–1 s; hand-relock ~1 s = pedestal step). Data: `host/results/agc_step.log`.
- Occlusion: ≤1 word rides through in HOLD (300–325 ms absorption); warm relock ≲1 period; no ratchet
  (5 geometries of `recovery_sweep` CSVs in `host/results/`).
- s7 gateware queue: **A4d-8** gear-shifted DC α (1/32 unlocked → ~4× faster pedestal recovery) + **A4d-7**
  min-energy lock gate (dark-input false-lock flashes). Both sim-first via `sim/`.

## Open items / next steps

1. **Order-03** ([cad/beacon-eval/beacon-order-03.md](../../cad/beacon-eval/beacon-order-03.md)): OVP parts,
   flight-cube tiles, filter+lens, 4.7 µH flight inductor, PicoBlade battery connectors (flight pack =
   Spektrum SPMX1501S50, std-LiPo 4.20 V charge, "PH 1.25 Ultra Micro" = PicoBlade-compatible).
2. **A1-ovp**: fit the 15 V zener + 1 kΩ clamp when parts land; pull-LED-header live test.
3. **A3-b**: soldered receiver on copper-clad (fixed R_f + real C_f, star ground) → re-run regression.
4. **A2-uvlo-2**: scripted slow-creep + dwell micro-dropout sweep (psu.py ramp + banner listener) — the
   original 3 blips didn't reproduce in the fast static-hold pass.
5. Flight cube build @306 mA (raise psu MAX_CURR deliberately) → optics stage → **100 m field test**.
6. Camera-era (040) notes seeded in optical-link-outcome.md (multipath, tracker-bank, full-duty contract).

## History (chronological commits of record)

s5 real-ADC decode `1a35c30` → s6 closed loop + RAM windows + sim harness `afc6f69` → receiver bring-up +
harness auto-reattach `60c58c9` → 10 MHz `ca1fe78` → bench findings + OVP + order-03 `2d61e42` → optical
checkpoint (41 ft) `d012eaf` → UVLO implemented `1dbe44e` → verified+banner `07d195f` → closed/calibrated
`f58dcab` → order-03 flight section `49bb4c8` → 'P' attenuator + AGC measured `afcec11` → full-duty contract
`81f9c99` → PSU driver `241c540` + profile `6adad83` + guardrails `1e022f5` → **regression suite `f4b8590`**.
