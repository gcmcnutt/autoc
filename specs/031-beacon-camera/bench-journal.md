# 031 bench journal — living state (portable across machines)

> **What this is**: the running state of the beacon bench — instruments, quirks, open items, and pointers —
> kept **in-repo** (operator decision 2026-07-18) so any machine/session can pick up the work. Update this
> file as the bench evolves; deep results live in the outcome docs it links. Machine-local assistant memory
> should hold only a pointer here.

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

## Open items / next steps

1. **Order-03** ([cad/beacon-eval/beacon-order-03.md](../../cad/beacon-eval/beacon-order-03.md)): OVP parts,
   flight-cube tiles, filter+lens, 4.7 µH flight inductor, PicoBlade battery connectors (flight pack =
   Spektrum SPMX1501S50, std-LiPo 4.20 V charge, "PH 1.25 Ultra Micro" = PicoBlade-compatible).
2. **A1-ovp**: fit the 15 V zener + 1 kΩ clamp when parts land; pull-LED-header live test.
3. **A3-b**: soldered **Option-C** receiver on copper-clad (netlist in the daylight doc; R_load per the
   C-14 filter choice; star ground) → re-run regression.
4. **A2-uvlo-2**: scripted slow-creep + dwell micro-dropout sweep (psu.py ramp + banner listener) — the
   original 3 blips didn't reproduce in the fast static-hold pass.
5. Flight cube build @306 mA (raise psu MAX_CURR deliberately) → optics stage → **100 m field test**.
6. Camera-era (040) notes seeded in optical-link-outcome.md (multipath, tracker-bank, full-duty contract).

## History (chronological commits of record)

s5 real-ADC decode `1a35c30` → s6 closed loop + RAM windows + sim harness `afc6f69` → receiver bring-up +
harness auto-reattach `60c58c9` → 10 MHz `ca1fe78` → bench findings + OVP + order-03 `2d61e42` → optical
checkpoint (41 ft) `d012eaf` → UVLO implemented `1dbe44e` → verified+banner `07d195f` → closed/calibrated
`f58dcab` → order-03 flight section `49bb4c8` → 'P' attenuator + AGC measured `afcec11` → full-duty contract
`81f9c99` → PSU driver `241c540` + profile `6adad83` + guardrails `1e022f5` → **regression suite `f4b8590`** → s7 gear-shift AGC + energy gate `32a2596`.
