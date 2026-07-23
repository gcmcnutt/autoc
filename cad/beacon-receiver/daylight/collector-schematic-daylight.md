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
5. AGC interplay: the decoder's DC tracker + scale-free margin already handle residual wander; nothing in
   the gateware needs to change for v2 (the s7 gear-shift helps the AC-coupled recovery transients too).

## Bring-up sequence (mirrors v1's, stage-gated)

1. Stage 1 alone (scope TIA_OUT): dark rest 0.45 V; lamp pedestal moves it (this IS experiment B1); beacon
   adds mV-scale code. No rail below ~200 µA ambient.
2. Add C2/R4/VBIAS2: AC_NODE rests at 1.25 V, pedestal gone, code passes.
3. U1B ×101: out rests 1.25 V, code at ×101, band-limited (C5). Feed ADC → decoder margin.
4. Re-run `regression.py` (policy) + the P-ladder → new baseline; then the lamp ladder for the daylight
   margin curve; then outdoors again — WITH the telemetry rig this time (`adc` = pedestal meter).
```
