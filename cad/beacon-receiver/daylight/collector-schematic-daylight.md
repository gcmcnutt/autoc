# Collector v2 — DAYLIGHT receiver (two-stage, AC-coupled) — wire-level

**Status**: DRAFT design, 2026-07-19 — response to the field finding (bench-journal / A3-c): full sun puts
**0.3–1 mA** into the wide-band PD and rails the v1 single-stage 1 MΩ DC-coupled TIA at any distance.
**Chain**: `PD → optical BP filter → LOW-R TIA (U1A) → AC-couple → ×100 post-gain (U1B, mid-rail bias) → ADC`.
v1 (indoor/bench, DC-coupled): [../collector-schematic.md](../collector-schematic.md). Validate stage-by-stage via
the Set-B bench experiments (lamp pedestal ladder) before freezing into the soldered build (A3-b).

## Why two stages (the daylight math)

| condition | ambient I_pd (no filter) | with FB850-10 (÷~25) |
|---|---|---|
| direct sun on PD | 0.8–1.1 mA | ~25–40 µA |
| sunny outdoors, PD not sun-facing | 30–300 µA | 1–12 µA |
| indoor bench | ≤ ~1 µA | ~0 |

A single-stage TIA cannot span this: R_f big enough for the beacon (µA×1 MΩ) rails on ambient; small enough
for ambient (≤2 kΩ bare-sun) buries the beacon. Split the problem: **stage 1 = R_f1 10 kΩ** (linear up to
~200 µA pedestal ≈ filtered-sun worst case with margin; the pedestal is DC so it just sits there), **AC-couple
away the pedestal**, then **stage 2 = ×100 voltage gain in the code band only** → equivalent 1 MΩ
transimpedance for the code, mA-class DC tolerance. The optical filter is REQUIRED outdoors (O3-11): without
it, bare direct sun (1 mA × 10 k = 10 V) still rails stage 1 — by design; don't point a bare PD at the sun.

## Reference designators / BOM deltas vs v1

| Refdes | Description | Value / part | New vs v1? |
|---|---|---|---|
| D1 | Photodiode | BPV10NF (unchanged; cathode → U1A −in, anode → GND per HW-verified orientation) | — |
| FLT1 | 850 nm bandpass in front of D1 | **see Open Q1**: FB850-10 (10 nm) vs FBH850-40 (40 nm) | **NEW (mechanical)** |
| R1 | Stage-1 TIA feedback | **10 kΩ 1 %** (fixed — no trimpot in v2) | was 1 MΩ pot |
| C1 | Stage-1 comp/limit | **1 nF** across R1 (f≈16 kHz: stability + first band-limit) | was 2 pF |
| C2 | AC-couple | **1 µF film/X7R** | existed as option, now REQUIRED |
| R4 | HP re-bias to VBIAS2 | **100 kΩ** (f_hp = 1/(2π·R4·C2) ≈ **1.6 Hz** — passes the 6.45 Hz word envelope) | was 1 MΩ to VBIAS |
| U1B | Post-gain, non-inverting | MCP6022 unit B — **finally earns its keep** | was parked |
| R5 | U1B gain bottom (−in → **VBIAS2**, see note) | **1 kΩ** | NEW |
| R6 | U1B feedback | **100 kΩ** → G = 1 + R6/R5 = **×101** | NEW |
| C5 | Across R6 — **the anti-alias pole** (Set-A lesson: band-limit BEFORE the 480 Hz point sampler) | **820 pF** → f ≈ 1.9 kHz | NEW |
| R7/R8 | **VBIAS2 = 1.25 V** divider off VREF2V5 (mid-ADC bias for the now-BIPOLAR AC signal) | 2× **10 kΩ** + **10 µF** bypass | NEW |
| R2/R3 (+10 µF) | VBIAS1 ≈ 0.45 V for U1A +in (as-built 4.7k/1k — keep) | unchanged | — |
| U2 | MCP3201, **IN+ = U1B out, IN− = GND** (single-ended per the v1 retraction) | unchanged | — |

## ASCII schematic

```
                     FLT1 (850 BP)             R1 10k ∥ C1 1nF
                        ┊                    ┌───/\/\/\──┬─────────┐
   photons ─────────────┊──► D1 ─┐           │           │         │
                                 │ cathode   │        (feedback)   │
                                 ├───────────┴──► U1A −in          │
                anode ── GND     │                U1A +in ◄── VBIAS1 (0.45 V, R2/R3 + 10 µF)
                                                  U1A out ─────────┴──● TIA_OUT
                                                                      │   (rest = 0.45 V + I_amb·10k;
                                                                      │    code = ±I_sig·10k, mV-scale)
                                                        C2 1 µF ──────┤
                                                                      ● AC_NODE
                                              R4 100k ── VBIAS2 ──────┤   (pedestal GONE; rest = 1.25 V)
                                                                      │
                                                       U1B +in ◄──────┘
                                       R5 1k ── VBIAS2 ── U1B −in
                                              R6 100k ∥ C5 820pF: −in ↔ out
                                                       U1B out ────────► U2 IN+ (MCP3201)   U2 IN− ── GND
                                                       (1.25 V ± 100·v_sig, band 1.6 Hz – 1.9 kHz)
```

## NETLIST (draw from this — a component is between exactly the nets listed; ∥ = same two nets)

| Net | Connections |
|---|---|
| **GND** | D1.anode · R3.2 · R8.2 · CB1.2 · C7.2 · U1.VSS(4) · U2.VSS(4) · **U2.IN−(3)** · U3.GND · J1.GND |
| **3V3** | U1.VDD(8) · U2.VDD(8) · U3.VIN · decoupling caps |
| **VREF2V5** | U3.OUT · U2.VREF(1) · R2.1 · R7.1 |
| **VBIAS1** (≈0.45 V) | R2.2 · R3.1 · CB1.1(10 µF) · **U1A.+in(3)** — nothing else |
| **N_SUM** (virtual VBIAS1) | D1.cathode · **U1A.−in(2)** · R1.1 · C1.1 |
| **N_TIA** | **U1A.out(1)** · R1.2 · C1.2 · C6.1 — so **R1 ∥ C1** both bridge N_SUM↔N_TIA (feedback pair) |
| **N_AC** | C6.2 · R4.1 · **U1B.+in(5)** — C6 is SERIES (the only path from N_TIA); R4 is the only DC path |
| **VBIAS2** (1.25 V) | R4.2 · R5.2 · R7.2 · R8.1 · C7.1(10 µF) — note **R5 returns here, NOT to GND** (DC gain 1 about VBIAS2) |
| **N_FB2** | **U1B.−in(6)** · R5.1 · R6.1 · C5.1 |
| **N_OUT** | **U1B.out(7)** · R6.2 · C5.2 · **U2.IN+(2)** — so **R6 ∥ C5** bridge N_FB2↔N_OUT |

MCP6022 pins: 1=OUTA 2=−INA 3=+INA 4=VSS 5=+INB 6=−INB 7=OUTB 8=VDD.

## Pin-by-pin (build/verify list — each pin connects to EXACTLY what is listed)

- **U1A +in (3)**: R2.B, R3.A, CB1.+ (the 0.45 V VBIAS1 node; nothing else)
- **U1A −in (2)**: D1.cathode, R1.A, C1.A (3 things exactly)
- **U1A out (1)**: R1.B, C1.B, C6.A
- **U1B +in (5)**: C6.B, R4.A (+ optional clamps D2/D3.A)
- **U1B −in (6)**: R5.A, R6.A, C5.A (3 things exactly)
- **U1B out (7)**: R6.B, C5.B, U2.IN+(2)
- **D1**: cathode→U1A−in, anode→GND · **R1 10k & C1 1nF**: both U1A−in↔U1A out (parallel pair)
- **C6 100nF**: U1A out↔U1B+in (series, only bridge) · **R4 100k** (+D2/D3 anti-parallel): U1B+in↔VBIAS2
- **R5 1k**: U1B−in↔**VBIAS2 (not GND)** · **R6 100k & C5 820pF**: both U1B−in↔U1B out (parallel pair)
- **VBIAS1**: R2 4.7k from VREF2V5, R3 1k to GND, CB1 10µF to GND
- **VBIAS2**: R7 10k from VREF2V5, R8 10k to GND, C7 10µF to GND
- **U2**: IN+(2)=U1B out, IN−(3)=GND, VREF(1)=VREF2V5, VDD=3V3, VSS=GND, SPI→J1 as v1
- **U1 power**: pin 8=3V3, pin 4=GND (check FIRST — the unpowered-amp trap)

## DC-transient response (all-attitude concern — the ambient pedestal moves FAST)

A rolling craft sweeps sky↔ground↔sun through the FOV: multi-volt pedestal steps at N_TIA in tens of ms.
Stage 1 follows instantly (DC-coupled, τ≈R1·C1=10 µs) — fine while in linear range. **The problem is the AC
coupler**: a pedestal step ΔV couples straight through C6 onto N_AC, slams U1B (×101) to the rail, and decode
is blind until the step decays through R4: **t_blind ≈ τ_hp · ln(ΔV / (headroom/G)) ≈ τ_hp · ln(ΔV/10 mV)**.

| C6 / R4 | τ_hp | f_hp | 2 V step blindness | code cost |
|---|---|---|---|---|
| 1 µF / 100 k (as drafted) | 100 ms | 1.6 Hz | **~530 ms** — unacceptable in a roll | none |
| 100 nF / 100 k | 10 ms | 16 Hz | ~53 ms | ~1 dB (little code energy < 20 Hz) |
| 100 nF / 100 k **+ anti-parallel clamp diodes across R4** (D2/D3, 1N4148) | 10 ms | 16 Hz | **~20–40 ms** (diodes fast-charge C6 to within ±0.6 V, linear decay only for the last bit) | none extra |

Recommendation pending the talk: **C6 = 100 nF + clamp diodes**, keeping R4 = 100 k. Bonus: with AC coupling,
the DECODER's pedestal is pinned at VBIAS2 — its DC tracker barely moves, so total system recovery ≈ analog
HP recovery + 1–2 code periods (the s7 gear-shift covers the rest). The 040 camera-era per-pixel version of
this problem is different (each pixel's ambient is scene-local) — this design only has to solve the 1-pixel case.

## Signal budget (bench flux constant I_sig ≈ 1.1 µA·m²/r², beacon @ 50 mA)

| r | I_sig | stage-1 v_sig | U1B out swing (×101) | ADC counts p-p |
|---|---|---|---|---|
| 1 m | 1.1 µA | 11 mV | ~1.1 V p-p (near max — see headroom) | ~1800 |
| 3 m | 122 nA | 1.2 mV | 123 mV | ~200 |
| 10 m | 11 nA | 110 µV | 11 mV | ~18 |
| 30 m (field current ×2.3 flux) | ~3 nA | — | ~6 mV | ~10 — decoder floor territory |

Headroom: VBIAS2 = 1.25 V centers the bipolar AC signal in the 0–2.5 V ADC window → **±1.05 V usable** ≈
2.1 V p-p ≈ 20 mV p-p at stage 1 ≈ **2 µA beacon** before clipping (≈0.75 m at bench flux — walk away or
accept AGC-tolerated clipping). Filter transmission derates I_sig (see Open Q1).

## Open questions (resolve via Set-B bench experiments before A3-b freeze)

1. **Filter FWHM vs the LED's 30 nm spectrum (DS190)**: FB850-10 (10 nm) passes only ~25–35 % of the beacon
   but kills ambient ÷25; **FBH850-40 (40 nm, T>90 %) passes ~85 % of the beacon at ambient ÷~7**. Net
   pedestal-to-signal: 10 nm wins ×3 on headroom, 40 nm wins ×2.5 on absolute signal. Lean **FBH850-40** for
   range + the 10 nm as the direct-sun-pointing insurance; measure both if budget allows.
2. **Artificial-light flicker**: 100/120 Hz (and LED PWM kHz) passes the 1.6 Hz HP and gets ×101. The Gold
   code decorrelates it but it inflates the energy denominator (margin loss) — B1 should include an indoor
   LED/fluorescent station. Mitigation lever: raise f_hp toward ~20 Hz (C2 → 100 nF) at slight word-envelope
   droop cost.
3. **R_f1 value**: 10 k chosen for filtered-sun margin; if B1 shows the knee far away, 22–47 k buys SNR.
4. **DC-servo alternative** (integrator nulling the pedestal at the summing node, keeps single-stage 1 MΩ):
   more parts, single-supply awkwardness — rejected for v2 unless B-series falsifies the two-stage.
5. **Stage-2 reference = TI SBOA224** (AC-coupled HPF non-inverting, cookbook): topology cross-checked
   2026-07-24 — signal into +in via the coupler, bias divider at +in, feedback out→−in, gain 1+R6/R5: all
   match. TI's one extra: a **cap in series with the gain-set leg** (their C1 15 µF) → DC gain = 1 (Vos not
   amplified; also can't amplify settling-pedestal tails). Our equivalent: **~10 µF in series with R5 = 1 k**
   (16 Hz leg corner). Optional but recommended for the soldered build. Bench note 2026-07-24: 1 nF coupling
   = 160 Hz corner = edge spikes (wrong); 0.1 µF/1 MΩ = 1.6 Hz (right, but τ=100 ms — see DC-transient
   table); gain ×2 observed = R5≈R6 ratio error, want R5 = R6/100.
6. AGC interplay: the decoder's DC tracker + scale-free margin already handle residual wander; nothing in
   the gateware needs to change for v2 (the s7 gear-shift helps the AC-coupled recovery transients too).

## ★ Option C — SELECTED (2026-07-24, bench-verified): reverse-biased PD + load R + single gain stage

The classic IR-receiver front end; ONE amp instead of two: **PD cathode → 3V3; PD anode → R_load
(22–47 k) → GND; anode → C 0.1 µF → the SBOA224 gain stage (×50–100 @ VBIAS2) → ADC.**
- ~3 V reverse bias (lowest C_pd of any option, 11 pF → ~4–5 pF), PD is a pure photon→electron current
  source; R_load does the I→V.
- Daylight grace: ambient slides the DC point down R_load; extreme sun COMPRESSES (PD runs out of reverse
  bias) instead of railing an op amp — softer failure than the TIA.
- **Bench result 2026-07-24**: strong drive close-range AND wall-bounce decode restored, with substitute
  values — the topology is tolerant; exact C/R values are optimization, not function. **Next field test runs
  Option C.** The two-stage (above) remains the linear-pedestal fallback.
- **Option C pin list (canonical)**: PD cathode→3V3; PD anode→R_load 47k→GND and →C6 100nF→U1A +in;
  U1A +in also →R4 100k→VBIAS2 (+ optional D2/D3 1N4148 anti-parallel across R4, attitude clamps);
  U1A −in →R6 100k + C5 820pF→U1A out (parallel pair) and →R5 1k→**C8 10 µF→GND** (series cap: DC gain 1,
  out rests at VBIAS2); U1A out→U2 IN+; U2 IN−→GND; VBIAS2 = R7 10k/R8 10k/C7 10 µF (unchanged).
  Gain 1+R6/R5 = ×101 (start ×51 = R5 2k if clipping close-in). NO VBIAS1, no TIA — R2/R3/CB1/C1 deleted.
- Trades: compressive (not linear) at pedestal extremes; PD node voltage moves (C modulation — negligible
  at 200 Hz). Fewer parts than the two-stage. Head-to-head vs the two-stage in Set-B before A3-b freezes.

## Bring-up sequence (mirrors v1's, stage-gated)

1. Stage 1 alone (scope TIA_OUT): dark rest 0.45 V; lamp pedestal moves it (this IS experiment B1); beacon
   adds mV-scale code. No rail below ~200 µA ambient.
2. Add C2/R4/VBIAS2: AC_NODE rests at 1.25 V, pedestal gone, code passes.
3. U1B ×101: out rests 1.25 V, code at ×101, band-limited (C5). Feed ADC → decoder margin.
4. Re-run `regression.py` (policy) + the P-ladder → new baseline; then the lamp ladder for the daylight
   margin curve; then outdoors again — WITH the telemetry rig this time (`adc` = pedestal meter).
```
