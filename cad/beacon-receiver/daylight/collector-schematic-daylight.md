# Collector v2 — DAYLIGHT receiver, Option C (SELECTED) — wire-level

**Status**: **OPTION C SELECTED 2026-07-24, bench-verified — and FIELD-PROVEN 2026-07-26 with canonical
values: lock in DIRECT SUNLIGHT at ~15–20 ft, bare PD, no optical filter, bench current** (bench-journal
field tests #2/#3: substitute values compress in daylight; canonical values do not). This doc is now the **canonical Option-C reference**:
final values with time-constant rationale, a clean netlist for the manual KiCad update
(`beacon-receiver-daylight.kicad_sch`), and the bring-up list. Cleaned 2026-07-25 in prep for
[parts order 03](../../beacon-eval/beacon-order-03.md).
**Chain**: `PD (reverse-biased) → R_load → AC-couple → single ×101 gain stage (SBOA224-style) → ADC`.
**History**: v1 single-stage 1 MΩ DC-coupled TIA ([../collector-schematic.md](../collector-schematic.md)) is
INDOOR-ONLY — full sun rails it (bench-journal / A3-c field finding 2026-07-17). The two-stage TIA design
(Option B, drafted 2026-07-19) is demoted to the **linear-pedestal fallback** — kept as Appendix B.

## Design constants (what the time constants are sized against)

| Parameter | Value | Source |
|---|---|---|
| Chip rate | **200 Hz** (5 ms chips, FULL-DUTY square — production waveform contract) | tasks.md E1 / optical-link-outcome §4 |
| Code | **N=31 Gold (CODE0)**, 16/15 balanced → word period **155 ms**, word rate **6.45 Hz** | tasks.md E1b |
| Code spectrum | Lines at k·6.45 Hz under a sinc² envelope, first null at 200 Hz — bulk of energy ~30–200 Hz, little below 20 Hz (near-balanced code) | — |
| ADC sampling | **480 Hz point sampler** (MCP3201, no aperture integration) → band-limit BEFORE it (Set-A lesson) | A4b |
| Decoder DC tracker | α=1/256 locked (τ=533 ms) / 1/32 unlocked (τ=67 ms), s7 gear-shift | A4d-8 |
| Ambient IR (the enemy) | full sun 0.8–1.1 mA into the bare wide-band PD; see the pedestal table below | field 2026-07-17 |
| Bench flux constant | I_sig ≈ 1.1 µA·m²/r² @ 51 mA drive; field 306 mA = ×2.3 | optical-link-outcome |

## Why Option C

- **One amp instead of two** (MCP6022 unit A only; unit B parked). Fewer parts than Option B, and the
  classic IR-receiver front end.
- **~3 V reverse bias** → lowest C_pd of any option (11 pF → ~4–5 pF); the PD is a pure photon→current
  source; R_load does the I→V.
- **Graceful large-DC-bias behavior**: ambient IR slides the PD node DC point up R_load; extreme sun
  **COMPRESSES** (PD runs out of reverse bias) instead of railing an op amp — a soft failure that recovers
  as the sun leaves the FOV. A TIA rails hard and is blind until the pedestal drops.
- **More gain than v1, not less**: in the code band R4 AC-loads the PD node through C6, so the effective
  I→V is **R1∥R4 ≈ 32 k**, and equivalent transimpedance = (R1∥R4) × G ≈ 32 k × 101 ≈ **3.2 MΩ**
  (v1 was 1 MΩ). At R1 22 k → ≈1.8 MΩ. (Corrected 2026-07-26 — earlier revision quoted I·R1 = 47 mV/µA,
  which ignored the R4 loading; field build #1 exposed it.)

## The circuit — values + time constants (each sized, not guessed)

| Refdes | Value | Function | Time constant / corner | Why this value |
|---|---|---|---|---|
| D1 | BPV10NF | PD, **cathode→3V3, anode→N_PD** (reverse-biased) | C_pd ~4–5 pF · R1 → ~700 kHz pole | irrelevant to the 200 Hz code — free bandwidth |
| FLT1 | 850 nm BP (order C-14) | optical ambient rejection | — | REQUIRED outdoors; FWHM choice ties to R1 (see pedestal table) |
| R1 | **47 kΩ** (R_load) | I→V | signal ≈32 mV/µA in-band (**R1∥R4** — R4 loads through C6); **DC pedestal ceiling 3.3 V/47 k = 70 µA** (DC sees R1 alone) | pairs with the 10 nm filter; drop to **22 k** if the 40 nm filter wins C-14. NOT 1 M: ceiling would be 3.3 µA — indoor-only (field-proven 2026-07-26) |
| C6 | **100 nF** | AC-couple (strips the pedestal) | with R1+R4 = 147 k: **f_hp = 10.8 Hz, τ = 14.7 ms** | passes the chip band (bulk ≥30 Hz, ≤1 dB cost); τ ≪ word (155 ms) so attitude steps recover within a word. NOT 1 nF (160 Hz corner = edge spikes, bench 2026-07-24); NOT 1 µF (τ 100 ms = half-second blindness in a roll) |
| R4 | **100 kΩ** | DC return of U1A +in to VBIAS2 | (sets f_hp with C6 above) | only DC path for +in bias current |
| D2/D3 | 1N4148 anti-parallel across R4 | attitude clamps | bound the +in excursion to VBIAS2 ±0.6 V | optional in Option C (see transient section) — keep for the soldered build |
| R6 ∥ C5 | **100 kΩ ∥ 820 pF** | feedback | **f_lp = 1.94 kHz ≈ 10× chip rate** | the anti-alias pole before the 480 Hz sampler (Set-A: 1.6 kHz class verified); 1.5–2.2 nF (~0.9–1.3 kHz) is a legal harder-anti-alias swap |
| R5 + C8 | **1 kΩ + 10 µF series to GND** | gain-set leg | corner **15.9 Hz**, matches f_hp | G = 1+R6/R5 = **×101** above ~16 Hz, **×1 at DC** (SBOA224 trick: Vos and pedestal tails NOT amplified; out rests exactly at VBIAS2). Start **×51 (R5 = 2 k)** if close-range clipping annoys |
| R7/R8 + C7 | 10 k / 10 k + 10 µF | **VBIAS2 = 1.25 V** off VREF2V5 | bypass τ = 50 ms | mid-ADC bias — bipolar AC signal centered in the 0–2.5 V window (±1.05 V usable) |
| U1 | MCP6022 | unit A = the gain stage; **unit B PARKED** (+in→VBIAS2, −in→out) | GBW 10 MHz → ×101 BW ≈ 100 kHz | plenty above 1.9 kHz |
| U2 | MCP3201 | **IN+ = U1A out, IN− = GND** (single-ended, per the v1 retraction — IN− spec ±100 mV of VSS) | 480 Hz sample | unchanged from v1 |
| U3 | MCP1525 | VREF2V5 → U2.VREF + the VBIAS2 divider | — | unchanged from v1 |
| ~~C1,C2,R2,R3~~ | — | **DELETED** (TIA comp, old coupler, VBIAS1 divider) | — | no TIA, no VBIAS1 in Option C |

The gear ratio to remember: **f_hp (10.8 Hz) ≈ gain-leg corner (15.9 Hz) ≪ chip band (30–200 Hz) ≪
anti-alias (1.9 kHz) ≪ sampler-driven analysis in the decoder**; and τ_hp (15 ms) ≪ word (155 ms) ≪
tracker-unlocked τ (67 ms) — the analog HP recovers from pedestal steps faster than the decoder even notices.

## ASCII schematic

```
                FLT1 (850 BP)                          3V3
                     ┊                                  │ cathode
  photons ───────────┊────► D1 BPV10NF ─────────────────┘
                                │ anode
                                ●─ N_PD ──── C6 100n ────● N_AC ───────► U1A +in (3)
                                │                        │
                         R1 47k │                 R4 100k│ ∥ D2/D3 (1N4148 anti-∥)
                                │                        │
                               GND                    VBIAS2 (1.25 V = R7 10k / R8 10k / C7 10µ off VREF2V5)

   U1A −in (2) ──┬── R5 1k ── C8 10µ ── GND          (leg: ×101 above 16 Hz, ×1 at DC)
                 └── R6 100k ∥ C5 820p ── U1A out (1) ───► U2 IN+ (2)      U2 IN− (3) ── GND
                     (out rests at 1.25 V; code = ±101·v_sig, band 11 Hz – 1.9 kHz)
   U1B parked:  +in (5) ── VBIAS2,  −in (6) ── out (7)
   U2: VREF(1)=VREF2V5, VDD(8)=3V3, VSS(4)=GND, CS(5)/CLK(7)/DOUT(6) ──► J1 (SPI, as v1)
```

## NETLIST (draw from this — a component sits between exactly the nets listed; ∥ = same two nets)

| Net | Connections |
|---|---|
| **GND** | R1.2 · C8.2 · R8.2 · C7.2 · C3.2 · C4.2 · U1.VSS(4) · U2.VSS(4) · **U2.IN−(3)** · U3.GND · J1.GND |
| **3V3** | **D1.cathode** · U1.VDD(8) · U2.VDD(8) · U3.VIN · C3.1 · C4.1 · J1.3V3 |
| **VREF2V5** | U3.OUT · U2.VREF(1) · R7.1 |
| **VBIAS2** (1.25 V) | R7.2 · R8.1 · C7.1(10 µF) · R4.2 · D2.cathode · D3.anode · **U1B.+in(5)** (parked unit) |
| **N_PD** | D1.anode · R1.1 · C6.1 — the load node; DC = I_amb·R1, code = I_sig·R1 |
| **N_AC** | C6.2 · R4.1 · D2.anode · D3.cathode · **U1A.+in(3)** — C6 is SERIES (the only path from N_PD) |
| **N_FB** | **U1A.−in(2)** · R6.1 · C5.1 · R5.1 |
| **N_LEG** | R5.2 · C8.1 — nothing else (C8 is SERIES under R5) |
| **N_OUT** | **U1A.out(1)** · R6.2 · C5.2 · **U2.IN+(2)** — so **R6 ∥ C5** both bridge N_FB↔N_OUT |
| **N_PARK** | **U1B.−in(6)** · **U1B.out(7)** — unity follower of VBIAS2 |
| **SPI** | U2.CS(5) / U2.CLK(7) / U2.DOUT(6) → J1 (same pins as v1) |

MCP6022 pins: 1=OUTA 2=−INA 3=+INA 4=VSS 5=+INB 6=−INB 7=OUTB 8=VDD.
MCP3201 pins: 1=VREF 2=IN+ 3=IN− 4=VSS 5=CS 6=DOUT 7=CLK 8=VDD.

### KiCad sync checklist (manual edit of `beacon-receiver-daylight.kicad_sch`)

1. **Delete** C1, C2, R2, R3 (already value-tagged DELETE except C2 — delete it too; it was the Option-B coupler).
2. D1 flips vs v1: **cathode→3V3** (was cathode→U1A −in), anode→N_PD.
3. R1 becomes the 47 k load N_PD→GND (value already tagged [OPT-C]).
4. Wire per the netlist above; add D2/D3 (1N4148) across R4.
5. Park U1B: +in(5)→VBIAS2, −in(6)→out(7), no other connections.
6. C3/C4 stay as IC decoupling (3V3→GND at U1, U2). ERC should drop from the 22 intentional pre-wiring errors to ~0.

## Large DC bias from ambient IR (the pedestal — why R1 = 47 k and when to change it)

The PD node DC voltage is V_ped = I_amb·R1; the PD compresses (softly) as V_ped approaches 3.3 V
(reverse bias exhausted). Ceiling: **70 µA @ 47 k · 150 µA @ 22 k · 330 µA @ 10 k**.

| Condition | I_amb bare | ÷7 (FBH850-40) | ÷25 (FB850-10) | @47 k verdict (filtered) |
|---|---|---|---|---|
| Direct sun in FOV | 0.8–1.1 mA | 115–160 µA | 32–44 µA | 40 nm: **compresses** (soft, transient) · 10 nm: 1.5–2.1 V — OK |
| Sunny, PD not sun-facing | 30–300 µA | 4–43 µA | 1.2–12 µA | OK either filter |
| Indoor bench | ≤ ~1 µA | ~0 | ~0 | 47 mV — invisible |

- **Pairing rule (order C-14)**: FB850-10 → keep R1 = 47 k (max signal). FBH850-40 → drop R1 to **22 k**
  so filtered direct sun stays linear (signal cost ×0.47, still 2.2 MΩ equivalent).

> ⚠️ **THE TABLE ABOVE IS THE BARE-PD CASE AND PREDATES THE C-14 LENS (noted 2026-08-04).** Fitting the
> lens splits ambient into two cases that behave *oppositely*, and only one of them improves:
>
> - **Extended ambient** (sky, sunlit ground — the sun NOT in the field) scales as `T_amb/(4·F#²)`,
>   i.e. it **drops ~113×** at F/2.0 with a 50 nm filter. "Sunny, not sun-facing" 30–300 µA bare
>   becomes **0.27–2.7 µA** lensed. Comfortably linear at either R1. This is the good case.
> - **The sun DISC inside the field** is collected by the whole entrance pupil, so it scales with
>   **pupil area ÷ die area = 6.7×** at F/2.0 — the lens makes it *worse*, and the filter only partly
>   offsets that. Against the ceilings ([`lensed_pd_range.py`](../../../specs/031-beacon-camera/lensed_pd_range.py) §8):
>
> | filter | sun pedestal | vs 47 k (70 µA) | vs 22 k (150 µA) | vs 10 k (330 µA) |
> |---|---:|---:|---:|---:|
> | 10 nm | 209 µA | 3× over | 1× over | OK |
> | 30 nm | 628 µA | 9× over | 4× over | 2× over |
> | 50 nm | 952 µA | 14× over | 6× over | 3× over |
>
> **No load resistor survives a direct hit.** Dropping R1 to 22 k buys linearity for *diffuse* sun,
> which is what the pairing rule was for — it does not and cannot buy it for the disc. **Direct sun
> must be RIDDEN THROUGH, not designed around**; see the decoder AGC max-energy gate in the bench
> journal. Do not chase this with a smaller R1: 10 k would still be 3× over at 50 nm while costing
> 4.7× signal everywhere else.
>
> Not a damage risk, for the record: the focused solar image is 148 µm at ~101 kW/m², but total power
> is only ~1.7 mW and silicon spreads it to a **~39 mK** junction rise. It is purely a saturation and
> recovery problem.
- The pedestal itself is DC — C6 strips it; only pedestal **motion** matters (next section).
- The beacon's own mean current is also a pedestal (full-duty chips, 16/31 duty): at inches it is mA-scale
  and compresses everything — bench 2026-07-24 decoded anyway (AGC is scale-free); back off to ≥1 m for
  linear measurements.
- **Noise floor in sun**: at a 40 µA filtered pedestal, in-band shot noise ≈ 0.16 nA rms (B ≈ 2 kHz) —
  an order below the measured ≤10 nA decoder floor; R1 Johnson (~27 pA) and MCP6022 voltage noise (~8 pA
  current-referred) are negligible. Ambient shot noise, not the electronics, sets the daylight floor.

## DC-transient response (all-attitude — the pedestal MOVES fast in a roll)

A rolling craft sweeps sky↔ground↔sun through the FOV → V_ped steps of volts in tens of ms. The step couples
through C6 onto N_AC and the ×101 stage rails until it decays. With C6 = 100 nF:

| Config | 2 V step blind time | 3.3 V (full-scale) step |
|---|---|---|
| R4 100 k, no clamps | τ(14.7 ms)·ln(ΔV/10 mV) ≈ **78 ms** | ~85 ms |
| + D2/D3 clamps | clamp phase (τ = C6·R1 = 4.7 ms) to ±0.6 V, then linear tail ≈ **~65 ms** | ~65 ms |

(10 mV = output headroom 1.05 V ÷ G 101.) Note the clamps buy **less** here than in Option B — the source
impedance is R1 = 47 k, not an op-amp output — their real job is bounding the U1A input excursion on
multi-volt steps. Either way recovery ≪ 1 code word (155 ms), and the decoder's pedestal is pinned at VBIAS2
(its DC tracker barely moves) → total system recovery ≈ analog HP recovery + ~1 word warm relock (s7
gear-shift). Lever if field rolls still hurt: R4 → 47 k (f_hp 17 Hz, tail ×0.64).

## Signal budget (bench flux 1.1 µA·m²/r² @ 51 mA; effective I→V = R1∥R4 ≈ 32 k, G = 101; LSB = 0.61 mV)

*(Corrected 2026-07-26: in-band the PD current splits between R1 and the C6→R4 branch, so use R1∥R4, not
R1 — counts are ×0.68 vs the earlier revision.)*

| r | I_sig | v_sig (in-band) | U1A out p-p | ADC counts p-p |
|---|---|---|---|---|
| 1 m | 1.1 µA | 35 mV | **clips** (limit 2.1 V) | AGC-tolerated clipping (bench-verified decode) |
| 1.3 m | 0.64 µA | 21 mV | 2.1 V | ~3400 — clip edge |
| 3 m | 122 nA | 3.9 mV | 0.40 V | ~650 |
| 10 m | 11 nA | 0.35 mV | 36 mV | ~58 |
| 12.5 m (41 ft, the v1 record) | 7 nA | 0.22 mV | 23 mV | ~37 |
| 30 m, field current ×2.3 | 2.8 nA | 90 µV | 9 mV | ~15 |
| 100 m, field + optics ×10–25 | 2.5–6.3 nA | — | 8–20 mV | 13–33 |

Derate everything by filter transmission (×0.85 @ 40 nm, ×0.25–0.35 @ 10 nm) and add the collection-lens
gain (×10–25) for the 100 m story. Clip onset = 2.1 V/101/(R1∥R4) ≈ **0.64 µA (r ≈ 1.3 m bench)**; ×51
gain moves it to ~0.9 m and halves all counts.

## Two-emitter (CDMA) considerations — two async, independently-drifting codes on one PD

The production link is TWO wingtip emitters (Gold pair A/B), free-running on separate ±5 % RC oscillators
(up to ~10 % relative chip-rate drift, no shared phase). Checked against every time constant above:

**Passband corners: NO changes.** The front end is linear, so the summed signal is just the union of two
line spectra — no intermodulation, no new frequencies. With ±5 % drift the chip rates span ~190–210 Hz:
the code-energy floor moves ~30 → 28.5 Hz (still 2.6× above f_hp, <1 dB), the top stays 9× below the
1.9 kHz anti-alias pole and inside the 480 Hz sampler's 240 Hz Nyquist. Gain difference between the two
codes across the band: fractions of a dB, and the decoder normalizes per-beacon anyway.

**The beat is sub-Hz and the HP eats it — an argument for NOT lowering f_hp.** The two word envelopes slip
past each other at the word-rate difference, ≤ ~0.65 Hz (10 % of 6.45 Hz); the composite short-term mean
wobbles at that rate as the codes slide through relative phase. 0.65 Hz ≪ f_hp 10.8 Hz → stripped in the
analog domain before the decoder's DC tracker sees it. Dropping f_hp toward the old Option-B 1.6 Hz draft
would let this baseline wander through — keep C6/R4 where they are.

**Minor**: the DC pedestal doubles (two 16/31-duty means) — µA-scale at range, invisible next to the 70 µA
ambient budget. If the codes end up time-slotted (TDM words) instead of superposed, each burst start is a
~0.52·v_sig pedestal step through C6 settling in ~3τ ≈ 45 ms (29 % of a word) — fine, and one more reason
not to push f_hp much above ~20 Hz.

**The real adjustment: clipping policy (gain step).** With ONE code, hard clipping is benign — the clipped
waveform is still the code sign, AGC tolerates it (bench-verified). With TWO summed codes, clipping is a
nonlinearity applied to the sum: it intermodulates A×B and lets the stronger code crush the weaker one's
modulation — a near–far problem the correlator cannot undo (A4d-3 already shows two equal-power skewed
codes are an energy-share stress corner even WITHOUT clipping). Equal-power emitters double the composite
swing, so clip onset halves per emitter: 0.64 → **0.32 µA each (bench r ≈ 1.9 m @ ×101)**. Rule:
**single-emitter work — ×101, clipping tolerated; any two-emitter work (Stage 1 A6 bench, wingtip era) —
×51 (R5 = 2 k) or verify the operating range keeps the SUM out of clip.** At field ranges (≥10 m, tens of
mV out) ×101 clears either way.

## Open questions (close via Set-B + the first Option-C field test)

1. **Filter FWHM (order C-14) ↔ R1 pairing** — see the pedestal table: 40 nm+22 k (range) vs 10 nm+47 k
   (direct-sun insurance). Measure both if budget allows.
2. **Artificial-light flicker**: 100/120 Hz (+ LED-lamp PWM) is in-band and gets ×101. The Gold code
   decorrelates it but it inflates the AGC energy denominator (margin loss). Include an indoor
   LED/fluorescent station in Set-B. Lever: C6 → 47 nF (f_hp 23 Hz) at small code-energy cost.
3. **Clamp diodes**: modest benefit in Option C (numbers above) — measure with/without before the soldered
   build commits.
4. **Anti-alias depth**: 820 pF (1.9 kHz) is Set-A-verified; 1.5–2.2 nF (~1 kHz) buys more rejection at the
   480 Hz sampler if field noise shows aliasing. Don't go below ~600 Hz (3× chip) — chip-edge smear.
5. **Gain step**: ×101 vs ×51 (R5 1 k vs 2 k). For ONE emitter this is range policy — the decoder's
   scale-free margin tolerates clipping, so ×101 is the default. For TWO emitters it is a CORRECTNESS
   question — clipping the sum intermodulates the codes and feeds the near–far capture (see the two-emitter
   section): ×51 (or a verified no-clip range) whenever both emitters run. Make R5 easy to swap on the
   soldered build.

## Bring-up sequence (stage-gated, mirrors v1 discipline)

1. **Power first** (the unpowered-amp trap): U1 pin 8 = 3.3 V, pin 4 = GND; U3 out = 2.5 V; VBIAS2 = 1.25 V.
2. **Dark statics**: N_PD ≈ 0 V (dark current is nA); U1A out rests at 1.25 V ±(Vos, unamplified thanks to C8).
3. **Pedestal (incandescent lamp — LED lamps emit no 850 nm)**: N_PD DC climbs with lamp distance
   (telemetry `adc` won't see it — it's AC-stripped; scope N_PD directly); verify soft compression as
   N_PD → 3.3 V, and recovery. This is experiment B1 on the real topology.
4. **Beacon**: code visible at N_PD (mV), at N_AC (pedestal gone, rest 1.25 V), at out (×101, band-limited).
5. **Decoder**: margin baseline via `regression.py` (policy) + P-ladder; lamp-ladder margin curve (Set-B);
   then outdoors — WITH the telemetry rig (`adc` column + a scope on N_PD = pedestal meters).

---

## Appendix B — Option B, two-stage TIA (linear-pedestal FALLBACK — build only if C falsifies in the field)

`PD → 10 k TIA (U1A, VBIAS1 0.45 V) → AC-couple → ×101 non-inverting (U1B @ VBIAS2) → ADC`. Rationale: splits
mA-class ambient (stage 1 linear to ~200 µA pedestal) from µV-class code (stage 2 gain in-band only) —
stays **linear** through pedestals that compress Option C, at the cost of a second stage and VBIAS1.
Values as drafted 2026-07-19: R_f1 10 k ∥ C1 1 nF (16 kHz), C2/C6 100 nF + R4 100 k → VBIAS2 (16 Hz HP,
clamps D2/D3), U1B: R5 1 k (→VBIAS2), R6 100 k ∥ C5 820 pF, VBIAS1 = 0.45 V (R2 4.7 k / R3 1 k + 10 µF).
Netlist and the stage-by-stage analysis: git history of this file (pre-2026-07-25 revision, commit 50adad8
and earlier). SBOA224 is the stage-2 reference topology for both options.
