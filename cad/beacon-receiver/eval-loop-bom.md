# First Emitter↔Receiver Loop — Eval BOM

**Status**: DRAFT — order-ready buy list, 2026-06-17
**Scope**: 031 re-scoped to a **1-bit (single-IR-sensor) acquisition-research phase**. This BOM is the minimal hardware to **close one coded-IR loop on the bench**: an MCU-modulated 850 nm emitter → free space → a single photodiode receiver → an FPGA correlator that declares lock. The full 320×240 MIPI camera pipeline (`verified-bom.md`, FR-2.x) is **deferred** until the temporal-acquisition dynamics are characterized on this rig.

**Why single-sensor first**: a single photodiode + fast ADC **decouples the receiver sample rate from camera frame rate**, so we can study oversampling, acquisition time vs SNR, early/partial-code acquisition, and single-chip dropout tolerance with massive oversampling *before* committing to the camera. CEP / 2-D apparent-motion studies belong to the later camera phase (see "Deferred constraints" at end).

**Buy in stages** — each stage has a milestone that de-risks the next. Don't buy Stage 3/4 until Stage 1/2 closes. Nothing here has been purchased yet; **Stage 1 (MCU eval board) is the recommended first order.**

Prices are 1-piece street/catalog (±10%); confirm in-cart. Distis: DK = DigiKey, MOU = Mouser, AMZ = Amazon, TL = Thorlabs.

---

## Stage 1 — Emitter: build the existing beacon-eval rig  (~$30 new parts + on-hand bench gear)

**The emitter is already designed and schematic-captured** — don't re-spec it here. [../beacon-eval/verified-bom-eval.md](../beacon-eval/verified-bom-eval.md) is the printable order checklist for the real target circuit: **ATtiny416-XNANO + LM3410X boost driver + 5× L1IZ-0850 in series @ 306 mA CC**, with the R11 firmware-ADC UVLO + topological DIM pull-down failsafe. It carries cart groupings (§EV-A/B/C/D), the one-time XNANO R100 cut, and a 9-step bring-up sequence. Order against that document.

Headline parts (see the eval BOM for the full per-line checklist + spares):

| Block | Part # | Disti | Note |
|---|---|---|---|
| **MCU eval board (buy first)** | Microchip **ATTINY416-XNANO** (`ATTINY416-XNANO-ND`, ~$11) | DK | Onboard mEDBG UPDI programmer + USB power. Firmware is drop-in to the production ATtiny412. |
| Boost LED driver | TI **LM3410XMF-NOPB** (SOT-23-5) + DR0810-223ML 22 µH + MBR130 + 0.62 Ω sense + caps | DK | The validated CC boost string — keep it (not a simplified FET drive). |
| Emitter LEDs | Lumileds **L1IZ-0850000000000** ×5 (+spares) | DK | 850 ± 5 nm bin. |
| DIM pull-down | 10 kΩ 0603 to GND (R2) | DK | Topological failsafe per R11. |
| Bench gear | adj. 5 V/3 A supply + ≥20 MHz scope + breadboard + SOT-23→DIP adapters | — | Mostly on-hand (EV-D). |

**Toolchain (free)**: PlatformIO `env:beacon-eval` (megaTinyCore, `xplainedmini_updi` over the XNANO mEDBG), or `pymcuprog` / `avr-gcc`+avrdude. Firmware lives in `firmware/beacon-pod/` per `mcu-firmware-contract.md`.
**Stage-1 milestone**: per the eval bring-up §8 — scope at U1 DIM shows the 100 Hz Gold code; the sense-resistor current envelope tracks it; the IR string glows on a phone camera. (FR-1.5(a) scope-level check, full boost driver.)

---

## Stage 2 — Single-IR-sensor receiver front-end  (~$30)

Goal: convert received 850 nm photons → a voltage carrying the code, and digitize it. **The ADC is the primary path** — the MachXO2 has no analog input (Stage 3), and amplitude is exactly what the research needs: AGC dynamics (sun→shadow gain tracking), soft tentative/confirmed thresholds, and erasure-aware correlation (saturation/partial-blob marking) are all *amplitude* decisions a comparator has already discarded. The comparator stays as an **optional** cheap hard-1-bit baseline (one-pin, MHz-capable), but the ADC subsumes it — you can threshold in the FPGA.

**AGC note**: AC-coupling after the TIA kills the solar DC pedestal (good for the data link) but also hides the ambient level AGC needs. For AGC experiments, use a **low AC-coupling corner** (so the ADC still sees the slow envelope) or tap the pre-AC-couple node — keep both nodes reachable on the breadboard.

| Block | Part # | Qty | ~$ ea | Disti | Why |
|---|---|---|---|---|---|
| **Photodiode (wide-FOV / all-attitude)** | Vishay **BPW34** (plain, *not* FA) | 5 | $1.00 | DK 751-1007-ND | 7.5 mm² — ~10× the BPV10NF's light grasp → makes the **40° all-attitude receiver** viable (étendue). Use the **plain** part: the external MidOpt bandpass does daylight rejection, so the PD's built-in filter would just add loss. Low signal BW (~100–200 Hz) means its large capacitance doesn't matter. (BPW34FA filtered version was OOS 2026-06-18.) |
| **Photodiode (narrow-aimed 100 m)** | Vishay **BPV10NF** | 2 | $1.20 | DK | Through-hole, daylight-blocking PIN. Small (~0.8 mm²) = pairs with a collection lens for the narrow-aimed max-range shot. |
| Photodiode (SMD alt) | Vishay **VEMD10940F** / **TEMD5510FX01** | 2 | $0.80 | DK | Filtered SMD fallbacks if both leaded parts are out. |
| **TIA op-amp (breadboard — no adapter)** | Microchip **MCP6022-I/P** (DIP-8) | 1 | $1.10 | DK | Dual 10 MHz RRO in **DIP-8** — breadboards directly. Bring the loop up with this. |
| TIA op-amp (precision, later) | TI **OPA381** (OPA381AIDGKR, MSOP-8) | 2 | $2.24 | DK | Lower Ib/offset; **needs an MSOP-8→DIP breakout**. Swap in only if the MCP6022 noise floor limits range. |
| **R_f — TIA gain (start)** | 1 MΩ 0603 1% — Yageo **RC0603FR-071ML** | 10 | — | DK | Vout = Iph·Rf; 1 MΩ covers ~0.1–2 µA. Also the AC-couple bias R. |
| R_f — range spares | 100 kΩ **RC0603FR-07100KL** + 10 MΩ **RC0603FR-0710ML** (0603 1%) | 10 ea | — | DK | Sweep gain for the link-budget / saturation study. |
| C_f — TIA compensation | 2 pF **0603 C0G** — Samsung **CL10C2R0CB8NNNC** (alt KEMET C0603C209C5GACTU / Yageo CC0603CRNPO9BN2R0) | 10 | — | DK | Across R_f; tames TIA ringing. **Cut-tape** (the Murata GRM18 P/N was reel-only/10k MOQ). Value non-critical: any 1–3 pF C0G works, or omit and add only if the TIA rings. |
| **AC-coupling cap** | 1 µF 0805 X7R — Samsung **CL21B105KBFNNNE** (or WIMA MKS2 film, THT) | 10 | — | DK | Strip solar DC. ~1 MΩ bias → ~0.16 Hz corner (keeps the AGC envelope). |
| Decoupling | 100 nF 0603 X7R — Samsung **CL10B104KB8NNNC** | 10 | — | DK | Op-amp + ADC supply decoupling (same as emitter EV-A10). |
| **ADC (primary — soft-decision + AGC)** | Microchip **MCP3201** (MCP3201-CI/SN; DIP-8 = -CI/P) | 2 | $3.00 | DK | 12-bit, 100 kS/s, 3-wire SPI → MachXO2 hard SPI. 100 kS/s ÷ 100 Hz chip = **1000× oversampling** for soft-decision / AGC / dropout studies. |
| ADC (alt, headroom) | TI **ADS7042** 12-bit 1 MS/s | 1 | $3.22 | DK | Faster oversampling sweeps if needed. |
| Comparator (optional 1-bit baseline) | TI **LM393P** (DIP-8) or TLV3201 (SOT-23-5) | 2 | $0.40 | DK | Hard-decision reference only; LM393P is DIP (no adapter). Skip for first loop — ADC subsumes it. |
| Proto passives | bias R (1 MΩ), 100 nF decoupling, breadboard | asst | ~$5 | AMZ/DK | Front-end glue. |

**Order note**: these receiver parts are all DigiKey — **batch them with the emitter §EV-A DigiKey cart** ([../beacon-eval/verified-bom-eval.md](../beacon-eval/verified-bom-eval.md)) in a single checkout, since the research runs in parallel.
**Breakout note**: picking DIP parts above (MCP6022, MCP3201-CI/P, LM393P) leaves the **only SMD-on-adapter part in the whole loop = the emitter's LM3410X** — already covered by the emitter BOM's **EV-D3** SOT-23-5→DIP adapter (SchmalzTech `ST-SOT23-5`, or Chip Quik `PA0089`/`PCB3007-1`). No receiver breakouts needed.
**Stage-2 milestone**: at ~1 m (ND-attenuated), the TIA output shows the code on a scope; the MCP3201 (on a dev board / logic-analyzer capture) recovers the Gold-code pattern and resolves amplitude/envelope. This closes the **optical** loop before the FPGA correlator exists.

**Bench support (optional DigiKey adds):**
- **USB-UART adapter** (FTDI `TTL-232R-3V3` or a 3.3 V CP2102/CH340 module) — stream StepFPGA correlator output / raw ADC to a laptop for bench + ground tests.
- **microSD socket breakout + card** (SparkFun `BOB-00544` / Adafruit `254`, plain 3.3 V — no level shifter) — airborne-logging step (low rate ~0.2 MB/s; see Stage 3 notes).
- **1 MΩ multi-turn trimpot** (Bourns `3296`) — continuous TIA-gain sweep for link-budget / saturation studies vs swapping fixed R_f.
- **ADC voltage reference** (MCP1525 2.5 V / REF3025) — stable Vref for the MCP3201 beats noisy VDD.

---

## Stage 3 — FPGA correlator  (use the on-hand STEP-MXO2 — $0)

Goal: FPGA samples the comparator/ADC at high rate, runs a sliding matched filter against the code LUT, and raises tentative/confirmed lock — the heart of the acquisition-dynamics study (oversampling, partial-code early lock, dropout tolerance).

**Board: Lattice MachXO2 STEP-MXO2 (already owned — no purchase).** It fits the 1-bit phase well:
- **Onboard USB-JTAG programmer** — single USB cable, no external programmer.
- **Hard-core SPI block** → directly clocks the MCP3201 ADC (Stage 2) for soft-decision capture.
- **30 GPIOs on 100-mil through-holes** → breadboard-friendly; comparator (1-bit) wires to one input pin.
- **Onboard LEDs / 7-seg displays / buttons** → "tentative/confirmed lock" indicators and acquisition-counter readout with zero extra parts.
- MachXO2-4000 LUTs are ample for a 15-chip sliding matched filter + control FSM.

**Toolchain (free)**: **Lattice Diamond** (free node-locked license) — synthesis + place-and-route + the bundled Diamond Programmer over the onboard USB-JTAG. (No mature open-source flow exists for MachXO2; Diamond is the path. Diamond is also a different tool from the Radiant flow the eventual CrossLink-NX camera phase will use — that's fine, the MachXO2 work stands alone.)

**Note**: MachXO2 has **no onboard ADC** — the external MCP3201/ADS7042 (Stage 2) is required for soft-decision capture. (The $249 CrossLink-NX-EVN from `verified-bom.md` is **not** needed in the 1-bit phase; it returns only when MIPI camera ingest does.)
**Stage-3 milestone**: STEP-MXO2 ingests the live receiver (comparator on a GPIO + MCP3201 over the hard SPI), correlates, and lights an onboard "lock" LED as the emitter code is acquired; stream acquisition-time + peak-SNR telemetry to PC over UART.

---

## Stage 4 — Daylight rejection + 100 m field  (buy when the bench loop closes)

**FWHM choice — go WIDER (~30–40 nm), not narrow, for this wide-FOV single-PD receiver.** Two reasons converge: (1) the **L1IZ-0850 LED is unbinned** (~840–860 nm) — a 10 nm filter risks clipping it; a 30 nm passband covers the spread. (2) Interference filters **blue-shift their CWL as angle-of-incidence rises**, so in a wide (~120°-emulating) acceptance cone a narrow 10 nm filter walks off the LED for off-axis rays. A 30–40 nm filter tolerates both. (Reserve the narrow 10 nm Thorlabs for a future *narrow-FOV / collimated* variant.) Not DigiKey — separate Thorlabs/Edmund/Amazon order.

| Block | Part # | ~$ | Disti | Why |
|---|---|---|---|---|
| **850 nm bandpass (recommended)** | **MidOpt BP850** (~40 nm FWHM, MV-grade, threaded M25.5/M30.5 mount) | $50–80 | MidOpt | Known specs + screws into a printed cone mount; right FWHM for wide-FOV + unbinned LED. |
| Bandpass (cheap start) | AMZ glass bandpass **B09MD4FCBL** (850 ± 5 CWL, ~30 nm FWHM, Ø25 mm) | $10 | AMZ | Validates the chain. **Avoid Wratten 87/87C gel** — long-pass, passes NIR solar, no daylight rejection. |
| Bandpass (narrow — narrow-FOV only) | Thorlabs **FBH850-10** (850/10 nm, hard-coated, Ø25 mm) | $140 | TL | Max solar rejection, but only for near-collimated / narrow-FOV; will clip off-axis + unbinned LED in a wide cone. |
| **Collection lens (the 100 m range-maker)** | Ø25–50 mm, f ≈ 50–100 mm — plano-convex **Thorlabs LA1131** (~$30, BK7 fine at 850 nm) or NIR achromat **AC254-075-B** (~$100); lazy alt: a $20 **CCTV/C-mount lens** or monocular objective | $20–100 | TL/AMZ | Bare 0.8 mm² PD at 100 m collects ~pW; a 25–50 mm lens gathers ~100–1000× more (~nW). **PD sits at the focal point; filter in the barrel.** This — not the filter — is what makes 100 m work. |
| Cone / barrel | Thorlabs SM1 lens tube or 3D-printed barrel | ~$15 | TL | Holds lens + filter + PD coaxially; baffles stray sky. |

**Range vs FOV (étendue) — for the 100 m shot, AIM it.** One small detector can't have wide FOV *and* large collection aperture at once (conserved étendue). To see 100 m you use a lensed, **narrow-FOV (~couple°) receiver aimed at the craft** on a tripod — that's how you get signal. The wide ~120° regime is a separate, harder case characterized on its own. Filter FWHM follows the geometry: **wide-FOV bare/short receiver → 30–40 nm** (angle-shift + unbinned LED); **narrow-FOV aimed-lens receiver → 10 nm is viable and better** (collimated, on-axis, max solar rejection — verify LED ≈850 nm on the bench first).

**Stage-4 milestone**: emitter at 100 m in daylight, receiver still acquires through the bandpass filter — validates the link-budget margin (camera_considerations.md predicts ~40 dB at 100 m, ~10 dB after 30 dB real-world derating) on real hardware.

---

## Toolchain bring-up checklist

**MCU (UPDI)** — Stage 1:
1. Install `pymcuprog` (`pip install pymcuprog`) *or* megaTinyCore in Arduino IDE.
2. Plug in ATTINY416-XNANO; `pymcuprog ping` over the onboard programmer.
3. Flash a blink → confirm toolchain.
4. Flash the Gold-code firmware (`firmware/beacon-pod/` per `mcu-firmware-contract.md`); scope the GPIO.

**FPGA** — Stage 3 (STEP-MXO2 / MachXO2, already owned):
1. Install **Lattice Diamond** + request the free node-locked license.
2. Build a stock blink for the STEP-MXO2; program over the onboard USB-JTAG with **Diamond Programmer** → confirm flow + cable.
3. Synthesize the correlator; wire comparator → a GPIO and MCP3201 → the hard SPI block; light an onboard lock LED + stream telemetry over UART.

---

## Rough first-order budget

- **Stage 1 (order first):** ~$30 new parts (per `verified-bom-eval.md`) + on-hand bench supply/scope/breadboard.
- **Stage 2:** ~$30 — PD + TIA + ADC + passives (comparator optional, +$3).
- **Stage 3:** **$0** — STEP-MXO2 already on hand.
- **Stage 4:** ~$40 (cheap filter + optics) … ~$170 (Thorlabs hard-coated).
- **Minimal bench loop (Stages 1–3): ~$60 to buy** (everything but the field optics).

---

## Deferred constraints (camera phase — for the eventual re-scope of 031/031-fpga)

Captured here so they aren't lost while 031 narrows to the 1-bit phase:

- **20 Hz control loop (037)**: cold-acquisition latency framed in *control ticks*, not 10 Hz FC samples — 150 ms code period = **3 ticks @ 20 Hz** (was "1.5 FC samples @ 10 Hz" in FR-1.3). Early/partial-code acquisition (declare at ~70% of the code ≈ 105 ms ≈ 2 ticks) is the lever that keeps re-acquisition inside the loop budget — a primary study target on this bench.
- **480 fps camera baseline**: shifts frames-per-chip and the Nyquist chip-rate ceiling (≤ fps/2 = 240 Hz). Independent of this single-sensor bench (which oversamples far beyond 480 Hz), but the chip-rate/code-length choice this bench validates must stay representable at the eventual 480 fps spatial sampling.
- **CEP strategy**: matters once the beacon's apparent motion crosses pixels during the ~150 ms decode window (blob-crossing-rate term, the real CEP driver per `specs/BACKLOG.md` "[DEFERRED — 031-fed] CEP realism" + `038-accurate-m2` sensor grounding). Not exercisable with a single fixed sensor; deferred to the camera phase.
- **Soft-decision / erasure-aware correlation** (camera_considerations.md §Coding & Acquisition): flips cost 2, erasures cost 1; tentative-vs-confirmed thresholds. The ADC path in Stage 2 is what lets this bench characterize those soft metrics on real signals.
