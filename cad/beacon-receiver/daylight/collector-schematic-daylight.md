# Collector v2 — DAYLIGHT receiver, Option C (SELECTED) — wire-level

**Status**: **OPTION C SELECTED 2026-07-24, bench-verified — and FIELD-PROVEN 2026-07-26 with canonical
values: lock in DIRECT SUNLIGHT at ~15–20 ft, bare PD, no optical filter, bench current** (bench-journal
field tests #2/#3: substitute values compress in daylight; canonical values do not).
**KiCad sheet REBUILT + NET-VERIFIED 2026-08-04** (`beacon-receiver-daylight.kicad_sch`) — refdes in this
doc now MATCH THE SHEET (renumbered vs pre-08-04 revisions of this doc; the wire-level walk verified every
net, see §NETLIST).
**Chain**: `PD (reverse-biased) → R_load → AC-couple → single gain stage (SBOA224-style) → ADC`.
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
- **More gain than v1, not less**: in the code band the bias-return R5 AC-loads the PD node through C6, so
  the effective I→V is **R6∥R5 ≈ 32 k**, and equivalent transimpedance = (R6∥R5) × G ≈ 32 k × 101 ≈
  **3.2 MΩ** at the ×101 gain option (v1 was 1 MΩ); ≈1.5 MΩ at the fitted ×46.

## The circuit — values + time constants (each sized, not guessed; refdes = the KiCad sheet)

| Refdes | Value | Function | Time constant / corner | Why this value |
|---|---|---|---|---|
| D1 | **BPW34** (article #2, this sheet) | PD, **cathode→3V3, anode→PD_NODE** (reverse-biased) | C_pd (BPW34 ~7 pF @3 V) · R6 → ~500 kHz pole | irrelevant to the 200 Hz code. **Article decision 2026-08-04**: this sheet = the SECOND article — the lens+filter build, where the planar 7.5 mm² BPW34 at the lens focus wins (area + alignment; FOV comes from the optics). The bare-PD field rig (article #1, field tests #2/#3, flux constant measurements) is the round **BPV10NF** |
| FLT1 | 850 nm BP (order C-14) | optical ambient rejection (mechanical, not in sheet) | — | REQUIRED outdoors; FWHM choice ties to R6 (see pedestal table) |
| **R6** | **47 kΩ 1 %** (R_load) | I→V | signal ≈32 mV/µA in-band (**R6∥R5** — R5 loads through C6); **DC pedestal ceiling 3.3 V/47 k = 70 µA** (DC sees R6 alone) | pairs with the 10 nm filter; drop to **22 k** if the 40 nm filter wins C-14. NOT 1 M: ceiling 3.3 µA — indoor-only (field-proven 2026-07-26) |
| **C6** | **100 nF** | AC-couple (strips the pedestal) | with R6+R5 = 147 k: **f_hp = 10.8 Hz, τ = 14.7 ms** | passes the chip band (bulk ≥30 Hz, ≤1 dB cost); τ ≪ word (155 ms) so attitude steps recover within a word. NOT 1 nF (160 Hz corner = edge spikes); NOT 1 µF (τ 100 ms = half-second blindness in a roll) |
| **R5** | **100 kΩ 1 %** | DC return of U1A +in to VBIAS | (sets f_hp with C6 above) | only DC path for +in bias current |
| D2/D3 | 1N4148 anti-parallel across R5 | attitude clamps | bound the +in excursion to VBIAS ±0.6 V | **DNP — not fitted** (breadboard runs without them; Set-B B3 measures their worth; modest benefit in Option C, see transient section) |
| **R1 ∥ C7** | **100 kΩ 1 % ∥ 820 pF** | feedback | **f_lp = 1.94 kHz ≈ 10× chip rate** | the anti-alias pole before the 480 Hz sampler (Set-A: 1.6 kHz class verified); 1.5–2.2 nF (~0.9–1.3 kHz) is a legal harder-anti-alias swap |
| **R4 + C5** | **2.2 kΩ + 10 µF series to GND** | gain-set leg | leg corner ≈ **7 Hz** (below f_hp — passband unchanged) | G = 1+R1/R4 = **×46 as fitted** (field-proven 2026-07-26, two-emitter-safe); **1 k → ×101** is the max-range option; ×46 counts ≈ 0.45× the ×101 budget table. C5 makes DC gain **×1** (SBOA224 trick: Vos and pedestal tails NOT amplified; out rests exactly at VBIAS) |
| **R2/R3 + C2** | 10 k / 10 k + 10 µF | **VBIAS = 1.25 V** off VREF2V5 (the two-stage draft called it VBIAS2) | bypass τ = 50 ms | mid-ADC bias — bipolar AC signal centered in the 0–2.5 V window (±1.05 V usable). 10 µF matters: pedestal-step recovery dumps ~20–30 µA through R5 into this node (1 µF → 200 mV bump, 10 µF → 20 mV) |
| U1 | MCP6022 | unit A = the gain stage; **unit B PARKED** (+in(5)→VBIAS, −in(6)→out(7)) | GBW 10 MHz → ×101 BW ≈ 100 kHz | plenty above 1.9 kHz |
| U2 | MCP3201 | **IN+ = U1A out (TIA_OUT), IN− = GND** (single-ended per the v1 retraction — IN− spec ±100 mV of VSS) | 480 Hz sample | unchanged from v1 |
| U3 | MCP1525 | VREF2V5 → U2.VREF + the VBIAS divider | — | unchanged from v1 |
| C3/C4 | 100 nF | IC decoupling (U1, U2) | — | — |

The gear ratio to remember: **gain-leg corner (7 Hz) < f_hp (10.8 Hz) ≪ chip band (30–200 Hz) ≪
anti-alias (1.9 kHz)**; and τ_hp (15 ms) ≪ word (155 ms) — the analog HP recovers from pedestal steps
faster than the decoder even notices.

## ASCII schematic (sheet refdes)

```
                FLT1 (850 BP)                          3V3
                     ┊                                  │ cathode
  photons ───────────┊────► D1 BPW34 ────────────────────┘
                                │ anode
                                ●─ PD_NODE ── C6 100n ────● N_AC ───────► U1A +in (3)
                                │                         │
                         R6 47k │                  R5 100k│ (∥ D2/D3 1N4148 anti-∥, DNP)
                                │                         │
                               GND                     VBIAS (1.25 V = R2 10k / R3 10k / C2 10µ off VREF2V5)

   U1A −in (2) ──┬── R4 2.2k ── C5 10µ ── GND         (leg: ×46 above ~10 Hz, ×1 at DC; 1 k → ×101)
                 └── R1 100k ∥ C7 820p ── U1A out (1) ───● TIA_OUT ──► U2 IN+ (2)    U2 IN− (3) ── GND
                     (out rests at 1.25 V; code = ±46·v_sig, band ~11 Hz – 1.9 kHz)
   U1B parked:  +in (5) ── VBIAS,  −in (6) ── out (7)
   U2: VREF(1)=VREF2V5, VDD(8)=3V3, VSS(4)=GND, CS(5)/DOUT(6)/CLK(7) ──► J1 (SPI, as v1)
```

## NETLIST — WIRE-LEVEL VERIFIED against the sheet 2026-08-04 (∥ = same two nets)

| Net | Connections | Verified |
|---|---|---|
| **3V3** | D1.cathode · U1.VDD(8) · U2.VDD(8) · U3.VIN(1) · C3.1 · C4.1 · J1.20 | ✓ |
| **GND** | R6.2 · C5.2 · R3.2 · C2.2 · C3.2 · C4.2 · U1.VSS(4) · U2.VSS(4) · **U2.IN−(3)** · U3.GND(3) · J1.1 | ✓ |
| **VREF2V5** | U3.OUT(2) · U2.VREF(1) · R2.1 | ✓ |
| **VBIAS** (1.25 V) | R2.2 · R3.1 · C2.1(10 µF) · R5.2 · **U1B.+in(5)** (parked) | ✓ |
| **PD_NODE** | D1.anode · R6.1 · C6.1 — ⚠ sheet label currently sits on the FEEDBACK net; move it here | ✓ (wiring) |
| **N_AC** | C6.2 · R5.1 · **U1A.+in(3)** — exactly 3 members, nothing to GND | ✓ |
| **N_FB** | **U1A.−in(2)** · R1.2 · C7.1 · R4.1 | ✓ |
| **N_LEG** | R4.2 · C5.1 — nothing else | ✓ |
| **TIA_OUT** | **U1A.out(1)** · R1.1 · C7.2 · **U2.IN+(2)** — so **R1 ∥ C7** bridge N_FB↔TIA_OUT | ✓ |
| **N_PARK** | **U1B.−in(6)** · **U1B.out(7)** — unity follower of VBIAS | ✓ |
| **SPI** | U2.CS(5)→J1.5 · U2.DOUT(6)→J1.3 · U2.CLK(7)→J1.4 | ✓ |

MCP6022 pins: 1=OUTA 2=−INA 3=+INA 4=VSS 5=+INB 6=−INB 7=OUTB 8=VDD.
MCP3201 pins: 1=VREF 2=IN+ 3=IN− 4=VSS 5=CS 6=DOUT 7=CLK 8=VDD.

**Sheet FINAL 2026-08-04**: `PD_NODE` label relocated to the D1/R6/C6 node ✓ · **ERC = 0** (was 22
pre-wiring) ✓ · D1 = BPW34 by decision (lens/filter article — see D1 row). Note the bench flux constant
(1.1 µA·m²/r²) was measured with the BPV10NF bare rig; re-measure behind the optics for the article-2
budget.

Pin-count audit (the fast bench/schematic cross-check): +in(3) touches exactly {C6, R5};
−in(2) exactly {R1, C7, R4}; PD_NODE exactly {D1, R6, C6}; R4/C5 junction exactly those two.

## Large DC bias from ambient IR (the pedestal — why R6 = 47 k and when to change it)

The PD node DC voltage is V_ped = I_amb·R6; the PD compresses (softly) as V_ped approaches 3.3 V
(reverse bias exhausted). Ceiling: **70 µA @ 47 k · 150 µA @ 22 k · 330 µA @ 10 k**.

| Condition | I_amb bare | ÷7 (FBH850-40) | ÷25 (FB850-10) | @47 k verdict (filtered) |
|---|---|---|---|---|
| Direct sun in FOV | 0.8–1.1 mA | 115–160 µA | 32–44 µA | 40 nm: **compresses** (soft, transient) · 10 nm: 1.5–2.1 V — OK |
| Sunny, PD not sun-facing | 30–300 µA | 4–43 µA | 1.2–12 µA | OK either filter |
| Indoor bench | ≤ ~1 µA | ~0 | ~0 | 47 mV — invisible |

- **Pairing rule (order C-14)**: FB850-10 → keep R6 = 47 k (max signal). FBH850-40 → drop R6 to **22 k**
  so filtered direct sun stays linear (signal cost ×0.47, still ~1 MΩ-class equivalent).
- The pedestal itself is DC — C6 strips it; only pedestal **motion** matters (next section).
- The beacon's own mean current is also a pedestal (full-duty chips, 16/31 duty): at inches it is mA-scale
  and compresses everything — bench 2026-07-24 decoded anyway (AGC is scale-free); back off to ≥1 m for
  linear measurements.
- **Noise floor in sun**: at a 40 µA filtered pedestal, in-band shot noise ≈ 0.16 nA rms (B ≈ 2 kHz) —
  an order below the measured ≤10 nA decoder floor; R6 Johnson (~27 pA) and MCP6022 voltage noise (~8 pA
  current-referred) are negligible. Ambient shot noise, not the electronics, sets the daylight floor.
- **Field data 2026-07-26 (test #3)**: bare PD, direct sun, canonical values → locks to ~15–20 ft;
  hand-shadowing the PD improves it = residual compression, i.e. the filter's job.

## DC-transient response (all-attitude — the pedestal MOVES fast in a roll)

A rolling craft sweeps sky↔ground↔sun through the FOV → V_ped steps of volts in tens of ms. The step couples
through C6 onto N_AC and the ×46/×101 stage rails until it decays. With C6 = 100 nF:

| Config | 2 V step blind time | 3.3 V (full-scale) step |
|---|---|---|
| R5 100 k, no clamps (as built) | τ(14.7 ms)·ln(ΔV/22 mV) ≈ **66 ms** @×46 | ~74 ms |
| + D2/D3 clamps | clamp phase (τ = C6·R6 = 4.7 ms) to ±0.6 V, then linear tail ≈ **~55 ms** | ~55 ms |

(22 mV = output headroom 1.05 V ÷ G 46; at ×101 add ~10 ms to the tails.) The clamps buy **less** here than
in Option B — the source impedance is R6 = 47 k, not an op-amp output — their real job is bounding the U1A
input excursion on multi-volt steps. Either way recovery ≪ 1 code word (155 ms), and the decoder's pedestal
is pinned at VBIAS (its DC tracker barely moves) → total system recovery ≈ analog HP recovery + ~1 word
warm relock (s7 gear-shift). Lever if field rolls still hurt: R5 → 47 k (f_hp 17 Hz, tail ×0.64).

## Signal budget (bench flux 1.1 µA·m²/r² @ 51 mA; effective I→V = R6∥R5 ≈ 32 k; LSB = 0.61 mV)

Table at the **×101 option** (R4 = 1 k); **as-fitted ×46 (R4 = 2.2 k): multiply counts by 0.45**, clip
onset 1.4 µA ≈ 0.9 m.

| r | I_sig | v_sig (in-band) | U1A out p-p @×101 | ADC counts p-p @×101 |
|---|---|---|---|---|
| 1 m | 1.1 µA | 35 mV | **clips** (limit 2.1 V) | AGC-tolerated clipping (bench-verified decode) |
| 1.3 m | 0.64 µA | 21 mV | 2.1 V | ~3400 — clip edge |
| 3 m | 122 nA | 3.9 mV | 0.40 V | ~650 |
| 10 m | 11 nA | 0.35 mV | 36 mV | ~58 |
| 12.5 m (41 ft, the v1 record) | 7 nA | 0.22 mV | 23 mV | ~37 |
| 30 m, field current ×2.3 | 2.8 nA | 90 µV | 9 mV | ~15 |
| 100 m, field + optics ×10–25 | 2.5–6.3 nA | — | 8–20 mV | 13–33 |

Derate everything by filter transmission (×0.85 @ 40 nm, ×0.25–0.35 @ 10 nm) and add the collection-lens
gain (×10–25) for the 100 m story. Clip onset @×101 = 2.1 V/101/(R6∥R5) ≈ **0.64 µA (r ≈ 1.3 m bench)**.

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
would let this baseline wander through — keep C6/R5 where they are.

**Minor**: the DC pedestal doubles (two 16/31-duty means) — µA-scale at range, invisible next to the 70 µA
ambient budget. If the codes end up time-slotted (TDM words) instead of superposed, each burst start is a
~0.52·v_sig pedestal step through C6 settling in ~3τ ≈ 45 ms (29 % of a word) — fine, and one more reason
not to push f_hp much above ~20 Hz.

**The real adjustment: clipping policy (gain step).** With ONE code, hard clipping is benign — the clipped
waveform is still the code sign, AGC tolerates it (bench-verified). With TWO summed codes, clipping is a
nonlinearity applied to the sum: it intermodulates A×B and lets the stronger code crush the weaker one's
modulation — a near–far problem the correlator cannot undo (A4d-3 already shows two equal-power skewed
codes are an energy-share stress corner even WITHOUT clipping). Equal-power emitters double the composite
swing, so clip onset halves per emitter: 0.64 → **0.32 µA each @×101 (bench r ≈ 1.9 m)**. Rule:
**single-emitter work — ×101 tolerable with clipping; any two-emitter work (Stage 1 A6 bench, wingtip era)
— the fitted ×46 (R4 = 2.2 k) or a verified no-clip range.** At field ranges (≥10 m, tens of mV out)
either gain clears.

## Open questions (close via Set-B + the filtered field test)

1. **Filter FWHM (order C-14) ↔ R6 pairing** — see the pedestal table: 40 nm+22 k (range) vs 10 nm+47 k
   (direct-sun insurance). Measure both if budget allows.
2. **Artificial-light flicker**: 100/120 Hz (+ LED-lamp PWM) is in-band and gets the full gain. The Gold
   code decorrelates it but it inflates the AGC energy denominator (margin loss). Include an indoor
   LED/fluorescent station in Set-B. Lever: C6 → 47 nF (f_hp 23 Hz) at small code-energy cost.
3. **Clamp diodes D2/D3**: modest benefit in Option C (numbers above) — measure with/without (Set-B B3)
   before the soldered build commits. Currently DNP.
4. **Anti-alias depth**: C7 820 pF (1.9 kHz) is Set-A-verified; 1.5–2.2 nF (~1 kHz) buys more rejection at
   the 480 Hz sampler if field noise shows aliasing. Don't go below ~600 Hz (3× chip) — chip-edge smear.
5. **Gain step (R4)**: 2.2 k = ×46 as fitted (field-proven, two-emitter-safe) vs 1 k = ×101 max-range.
   For ONE emitter this is range policy; for TWO it is a CORRECTNESS question (see two-emitter section).
   Keep R4 easy to swap on the soldered build.

## Bring-up sequence (stage-gated, mirrors v1 discipline)

1. **Power first** (the unpowered-amp trap): U1 pin 8 = 3.3 V, pin 4 = GND; U3 out = 2.5 V; VBIAS = 1.25 V.
2. **Dark statics**: PD_NODE ≈ 0 V (dark current is nA); U1A out rests at 1.25 V ±(Vos, unamplified thanks
   to C5).
3. **Pedestal (incandescent lamp — LED lamps emit no 850 nm)**: PD_NODE DC climbs with lamp distance
   (telemetry `adc` won't see it — it's AC-stripped; scope PD_NODE directly, 10× probe); verify soft
   compression as PD_NODE → 3.3 V, and recovery. This is experiment B1 on the real topology (R6 = trimmer
   → sweep the knee).
4. **Beacon**: code visible at PD_NODE (mV), at N_AC (pedestal gone, rest 1.25 V), at TIA_OUT (×46,
   band-limited, droop per the AC-coupling — expected shape, edges carry the information).
5. **Decoder**: margin baseline via `regression.py` (policy) + P-ladder; lamp-ladder margin curve (Set-B);
   then outdoors — WITH the telemetry rig (`adc` column + a scope on PD_NODE = pedestal meters).

---

## Appendix B — Option B, two-stage TIA (linear-pedestal FALLBACK — build only if C falsifies in the field)

`PD → 10 k TIA (U1A, VBIAS1 0.45 V) → AC-couple → ×101 non-inverting (U1B @ VBIAS2) → ADC`. Rationale: splits
mA-class ambient (stage 1 linear to ~200 µA pedestal) from µV-class code (stage 2 gain in-band only) —
stays **linear** through pedestals that compress Option C, at the cost of a second stage and VBIAS1.
Values as drafted 2026-07-19 (pre-08-04 refdes): R_f1 10 k ∥ 1 nF (16 kHz), 100 nF + 100 k → VBIAS2 (16 Hz
HP, clamps), stage 2: 1 k leg (→VBIAS2), 100 k ∥ 820 pF, VBIAS1 = 0.45 V (4.7 k/1 k + 10 µF).
Netlist and the stage-by-stage analysis: git history of this file (pre-2026-07-25 revision, commit 50adad8
and earlier). SBOA224 is the stage-2 reference topology for both options.
