# 040 camera-redo — input data checklist

---

## SCOPE (agreed with operator 2026-07-28) — read this first

**040 is a research + design-refinement feature, not a build feature.** Outcome: *a refined camera scheme
grounded in what hardware can actually deliver*, plus a sim that reflects it — while 031 proves field
hardware in parallel and **M3 (optical + time-of-flight) is the real destination**. Investment should
therefore favour what **generalises to M3** (camera geometry, occlusion, photon budgeting, acquisition
latency) over what is beacon-CDMA-specific. Note ToF gives *direct* range, so the span→range inference
channel is itself transitional — the `(x, y, CEP)` interface survives (CEP becomes range-quality, range
becomes a direct input), but do not over-engineer span-based ranging.

**Mission modes matter**: patrol wants detection at ~100 m over a wide search volume (but can sweep —
nothing is manoeuvring yet); engage wants a wide *instantaneous* field at short range (~3 m terminal).
Different optics, different modes. That is the eventual dual-FOV case — **deferred out of 040**.

### In scope

1. **320×240 pixel-grid geometry cleanup** — isotropic angles (radians), int8 retired, FOV derived from
   `deg_per_px`, beacon separation corrected 0.9 → **0.772 m**
2. **FOV refinement** — model 90° alongside 120°; the **effective (clipped) field** is the honest output
3. **Occlusion** — airframe slab + pod nose + **prop disc**, from the measured station stack
4. **Incrementally better CEP** — acquisition / signal-quality grounded in the 031 curves
5. **Camera variations** — PRNG slot 5: mount 6-DOF, wing thickness, ambient pedestal
6. **Sim-vs-real airframe check** — early and *gating* (see below)
7. **M2 retrain** → a more robust M2, which is the feature's outcome

### Deferred (later features)

Photon budget at 480 fps (needs **article 1** + raw uncompressed high-bandwidth capture to characterise the
chain) · second camera / front+rear · multi-range & dual-FOV optics · FPGA pipeline · multipath, glint, sun.

### Parallel

Feed-forward to 031 field measurement, and to M3.

### The airframe question — expected verdict: DEFER the M1 rebake

`hb1_streamer.xml` is a **stability-derivative model, not a geometric one** (tuned across 021/023; the
vertical fin is already represented via `Cn_b=0.07`). So the sketch geometry feeds **occlusion only** and
does not touch the FDM. Checking real data against the sim:

| | FDM | real | verdict |
|---|---|---|---|
| mass | 0.515 kg | 520 g | ✅ |
| span | 0.762 m | 30″ | ✅ |
| wing area | 0.136 m² (7″×30″) | eCalc 0.1742 m² | ⚠️ **FDM matches the sketch; eCalc is the outlier** |
| prop | D 0.127 / H 0.114 (5.0×4.5) | **5.5×4** | ❌ genuine error |
| CG | `CG_arm=0.28` | unknown | ⚠️ |

- **eCalc's wing area is likely a data-entry error** (0.1742 m² implies a 9″ mean chord vs a measured 7″).
  It does not invalidate the RPM data but *does* contaminate the **speed** axis of the dynamic table ⇒
  **fit throttle→RPM, not airspeed→RPM** (throttle is what the NN commands anyway). Reinforces G2.
- **The prop error is real but expensive**: changing D/H changes thrust ⇒ changes the plant ⇒ invalidates
  every M1 elite ⇒ forces a rebake — and the M2 camera work trains *off* an M1 source dmp.
- **Recommendation: do NOT rebake M1 in 040.** (a) The 039 wrap carries a standing *"NO sim recalibration
  from this n=1 airframe"* decision pending new articles; (b) perception is chase-side at M2 train time, so
  no camera work needs a rebake — 040 stays parallel only if we don't touch the plant; (c) **sequencing
  trap**: if we ever do rebake, it must happen *before* the M2 work, so this check must run early and gate
  everything downstream. File the prop discrepancy for the flight-test feature.

### FOV comparison (for deliverable 2)

| | 120° | 90° |
|---|---|---|
| deg/px (320 px) | 0.375° | 0.281° |
| vertical FOV | 90° | 67.5° |
| SNR vs 120° | — | **+2.5 dB** |
| span at 100 m | 1.18 px | 1.57 px |
| focal length (1/4″ sensor) | ~1.7 mm | ~2.3 mm — comfortably a real M12 lens |

Prop shadow sits at fixed angles (−8.9° to −40.2°), so at ±33.75° vertical it eats ~37% of the vertical
field — about the same *fraction* as at 120°, just a smaller absolute field.

---

**Purpose**: the measured/confirmed values the 040 perception-fidelity spec needs before (or during)
implementation, plus the re-runs and bench experiments that produce them. Working document — tick items
off in place.

**Created** 2026-07-28 during the 040 scoping conversation. Sources cited inline so every assumption is
traceable.

**Legend**: ⛔ blocking (no defensible default) · ⚠️ assumable (I'll proceed on the stated assumption and
you correct it) · ✅ closed

---

## A. Chase airframe geometry — occlusion model

The canonical design is the [HB1 E1000 combat plane](https://hbaircraft.com/hb1-e1000-combat-plane/):
modular 3D-printed pod + foam wing, tail on two 5/16″ (7.9 mm) wooden dowels, cylinder motor mount.
**Because the camera looks forward, the boom / dowels / vertical fin cannot occlude anything** — the
occluder set reduces to **prop disc + wing slab + pod nose**. Only the prop is time-varying.

| # | Value | Unit | Feeds | Status | Current assumption + source |
|---|---|---|---|---|---|
| A1 | Motor configuration: **tractor or pusher** | — | Whether the prop disc intersects the FOV at all | ✅ | **TRACTOR** (operator 2026-07-28) — the prop is the streamer-cutting weapon. Prop occlusion is **live**; see §C geometry |
| A1b | Motor / thrust-line position in body frame | m | Disc plane position ⇒ §C crossing geometry | ⛔ | unknown |
| A2 | Camera mount point (x fwd, y right, z down) + boresight tilt | m / deg | Occlusion ray origin; FOV pointing; **radial offset from the thrust line is a design lever — see §C** | ⛔ | `(0, 0, −0.05)`, identity quat — `camera_config.h`. **Known broken**: z = −0.05 sits exactly on `box_min_z`, so enabling occlusion today occludes everything |
| A3 | Wing planform: span, chord, sweep, the asymmetry | in / deg | Wing occlusion slab extent | ⚠️ | **span 30″ / 762 mm**, **chord 7″**, elevon ≈1.25″ behind, no sweep (sketch 2026-07-28; span agrees with eCalc + FDM `ref span`). Asymmetry still to describe |
| A4 | Wing max thickness | in | Slab thickness; **downward view limit — see §C** | ✅ | **1″ confirmed** (operator 2026-07-28) = 14% of the 7″ chord. Flat-bottom folded foam board, **highly variable** ⇒ treat as a variation axis (F5) |
| A4c | Camera **standoff above the wing surface** | in | Downward view **and** prop clearance | ⚠️ | Design decision — **~1″ recommended** (45° down view + 8.9° prop clearance in one bracket, §C) |
| A5 | Station stack / sketch legend | in | Nose occlusion primitive; disc geometry | ✅ | **Resolved 2026-07-28** — prop 0″, wing LE 6″, camera 8″, wing TE 13″, elevon TE 14″. Tail: 3″ boom-to-fin, 4″ boom length. See the station-stack table in §C |
| A5b | Vertical tail | in | None (aft of camera) — recorded for completeness | ✅ | ≈6.5″ tall × 4″ base (sketch) |
| A6 | Anything else forward of the camera — antenna, motor-mount cylinder OD, wiring | m | Additional occluders | ⚠️ | assume none |
| A7 | Boom / tail / vertical fin | — | — | ✅ | All **aft** of a forward camera ⇒ no occlusion contribution (hbaircraft.com) |
| A8 | All-up weight as flown | g | Cross-check only | ⚠️ | 520 g (eCalc) / 515 g (FDM `Mass`) |

**Open reconciliation**: eCalc lists span 762 mm and wing area 17.42 dm² (270 in²) ⇒ ~9″ mean chord, but
the site caps the pod at a **7″ chord**. Either the area entry is approximate or the planform is wider
than the pod attachment. Resolve when A3 lands.

---

## B. Camera / optics

| # | Value | Unit | Feeds | Status | Current assumption + source |
|---|---|---|---|---|---|
| B1 | **Entrance pupil diameter** | mm | Blade transit time ⇒ prop occlusion duty | ⛔ | 8 mm (f/2 @ 16 mm) — `camera_considerations.md`, **pre-031, parked, and see the B4 conflict below** |
| B2 | **Exposure duty within the frame** | µs or % of 2.08 ms | Whether blade occlusion is uniform (benign) or alternating (structured) — pivotal for §C | ⚠️ | **Unknown for the camera (not yet selected); parameterize with a knob.** Default 1.04 ms (50% duty, the 480 fps figure). Note the PD-vs-camera distinction is a *locked contract*: full-duty chips, because a camera integrates and the 480 Hz point-sampling PD does not |
| B3 | Sensor: confirm 320×240; pixel pitch | px / µm | Quantization grid | ⚠️ | 320×240, 3 µm — `camera_considerations.md` |
| B4 | Lens: focal length, f/#, quoted FOV, projection type | mm / — / deg | `deg_per_px`; residual distortion | ⛔ | **CONFLICT — see below** |

### B4 — the 16 mm lens is the 031 RECEIVER collection optic, not the 040 flight lens

*(Raised as a conflict 2026-07-28, resolved same day by operator: the 16 mm is for the 1-pixel test.)*

It is correctly specified for that job: 8 mm aperture over a BPV10NF's 0.78 mm² active area is a **~64×
geometric collection gain** (the docs' conservative ×10–25 nets out transmission + spot-capture losses),
at the cost of a **~3.2° acceptance cone** — a *pointed* instrument for the 100 m range test.

**The 040 flight lens is still unselected.** It needs to be wide (~1.0–1.7 mm focal length for ~120° on a
quarter-inch sensor), which is a fundamentally different optic.

### ⚠️ The camera link budget EXISTS — but it is for the 3.4° camera, not a wide one

*(Corrects a claim made earlier the same day that it had never been computed.)*

**It was computed, and it is what drove the 300 mA decision** — `spec.md` FR-1.4 + the Q&A at
`spec.md:39`:

> "drop to **300 mA per LED** per the link-budget analysis (100 m daylight retains ~23 dB margin even at
> 100× real-world derating)"
> "**single visible LED** at 300 mA gives ~76 dB post-correlation SNR vs ~13 dB lock threshold = **~63 dB
> margin**"

Note it assumes **one visible LED** — so the E-RECHECK half-cube concern below does *not* apply to it. That
finding narrows to the bench→field extrapolation in `optical-link-outcome.md`, which scaled a **5-LED
co-aimed** 12.5 m measurement by current alone. The design-time budget is sound.

**The problem is its optics assumption.** `camera_considerations.md` states it plainly: *"Photon budget
with **8 mm f/2 lens**, 30 nm bandpass, OG-series sensor, daytime sky background."* An 8 mm aperture at
f/2 **is** the 16 mm lens — the **3.4° FOV** row of its own table. A 120° lens loses on both terms at once:

| | narrow (16 mm, 3.4°) | wide (~1.6 mm, 120°) |
|---|---|---|
| aperture @ f/2 | 8 mm ⇒ 50 mm² | 0.8 mm ⇒ 0.5 mm² |
| per-pixel IFOV | 0.0107° | 0.107° |
| per-pixel Ω | 3.5e-8 sr | 3.5e-6 sr |
| `√(A/Ω)` (background-limited SNR) | 3.8e4 | 3.8e2 |

**≈40 dB apart.** Against 63 dB clean / 23 dB derated, a 120° camera sits at ~23 dB margin clean at 100 m
but **~17 dB in the hole once 100× real-world derating applies** — lock threshold around **10–15 m** in
realistic daylight. (Close to the bench's measured 12.5 m bare-PD figure, which suggests the scaling is not
wildly off. Exact dB indicative only — QE, read noise, well depth, integration time unmodelled.)

### ⇒ THE central architectural decision of 040: **you cannot have 120° FOV and 100 m range on 320×240**

The FOV-vs-resolution tension flagged earlier is *also* a photon problem, and it is ~40 dB wide.

- **Narrow (~3.4°)** — 100 m works; nothing keeps a maneuvering target inside a 3.4° window without a gimbal
- **Wide (~120°)** — target stays in frame; range collapses to ~10–15 m realistic
- **Middle (~30–40°)** — worth costing out if the real engagement band is 5–50 m

**This promotes the backlog's dual-FOV / two-camera item from nice-to-have to the natural answer** — wide to
acquire and keep-in-frame, narrow for range and track. Previously motivated by "resolution is wasted at the
edges"; now motivated by 40 dB. Pairs with the stereo/multipath item (§D5 note).

**⇒ Still open — the operational-range requirement**, which decides all of the above: mission terminates at
~3 m, M2 sits at ~17 m median error, arena is R = 80 m. If real engagement lives in 5–50 m, the 100 m
detection goal is a legacy requirement that should be **explicitly retired** rather than left as an unmet
target in the docs.
| B5 | Optical filter FWHM + peak transmission | nm / % | Ambient pedestal magnitude | ⚠️ | **Open decision C-14** (10 nm vs 40 nm) — 031 bench journal |
| B6 | Collection-optics gain vs bare PD | × | Link budget range scaling | ⚠️ | ×10–25 — `optical-link-outcome.md` |
| B7 | Confirm frame rate | fps | Aliasing analysis in §C | ⚠️ | 480 fps (2.4 frames/chip at 200 Hz) |

---

## C. Propulsion + prop occlusion — **MOSTLY BACKLOGGED 2026-07-28**

> **Operator 2026-07-28**: *"never mind [the propCalc re-run] — keep this as backlog if we wind up putting
> the camera behind the prop later on."*
>
> The LE / 8″-outboard baseline puts the prop **41–61° off boresight**, so blade-phase fidelity buys
> nothing. **040 keeps only a STATIC angular prop shadow** with a fixed representative attenuation
> (spec FR-009) — which needs **no RPM data at all**, since engine speed was only ever required for blade
> *phase* and the resonance question.
>
> **Backlogged** (trigger = a mount that puts the camera behind the disc): G1 propCalc re-run · G2
> throttle→RPM fit · C5/C6/C7 propulsion confirmations · blade-phase and periodic attenuation · the
> resonance analysis below (kept for its arithmetic, not to be resolved here).
>
> ⚠️ **eCalc CSV export is a dead end regardless**: `xiao/eCalc Results.csv` came back an empty template —
> the `Download .csv (0)` button exports the *comparison list*, which needs `add to >>` first. But even
> populated it is a **single-operating-point summary** with no partial-load tables, so the throttle/rpm/
> speed triples exist **only in the printed PDF**. Print, do not export, if this is ever revisited.

| # | Value | Unit | Feeds | Status | Current assumption + source |
|---|---|---|---|---|---|
| C1 | **Actual prop in use**: diameter, pitch, blade count, make | in / mm | Disc geometry; RPM curve | ✅ | **Windsor Propeller / Master Airscrew 5.5×4, 2-blade** (photo 2026-07-28) ⇒ D = 139.7 mm, **r_tip = 2.75″ / 69.85 mm**. Matches the eCalc sheet ⇒ C5 table stands, G1 re-run optional |
| C2 | **Prop axial standoff**: camera → disc plane | m | Occlusion duty; crossing radius | ✅ | **≈ 8″ / 203 mm** (sketch 2026-07-28) — confirm the reading |
| C3 | **Prop radial offset**: camera vs prop axis | m | Which FOV region the disc covers | ⚠️ | Prop is **tractor, ahead of camera**. Camera **2.5–3″ above** the thrust line; lateral offset currently 0 — **see the clearance analysis above, this is a design decision** |
| C4 | Blade chord at mid-radius; hub/spinner diameter | mm | Transit time; fully-opaque inner region | ⚠️ | **~0.5″ / 12.7 mm chord** measured off the cutting mat (photo) — matches the prior assumption. Hub ≈ 0.75″ |
| C5 | RPM vs airspeed vs throttle | rpm / m/s / % | Blade-pass frequency; throttle→RPM map | ✅ *(pending re-run — G1)* | eCalc dynamic table, EMAX EcoII 2207-2400 + MAS GF 5.5×4 @ 3S 1000 mAh |
| C6 | Confirm motor KV | rpm/V | RPM curve | ⚠️ | 2207-**2400** (eCalc); stock alt is 2207-1700 |
| C7 | Confirm battery as flown | S / mAh | RPM curve | ⚠️ | 3S 1000 mAh |

### C5 as it currently stands (level flight, eCalc dynamic table)

`f_bp = 2 × RPM/60 = RPM/30` for a 2-blade prop.

| speed | RPM | throttle | f_bp |
|---|---:|---:|---:|
| 6.9 m/s (stall) | 7 833 | 31% | 261 Hz |
| 11.1 m/s | 12 532 | 52% | 418 Hz |
| **13.0 m/s (cruise)** | **~14 660** | **~62%** | **~489 Hz** |
| 15.3 m/s | 17 232 | 75% | 574 Hz |
| 18.1 m/s | 20 365 | 92% | 679 Hz |

Range **261–679 Hz** in level flight; down to ~187 Hz static at low throttle.

### Commensurabilities inside the envelope

- `f_bp` = 2× chip rate (400 Hz) → 12 000 rpm ≈ 10.6 m/s
- `f_bp` = 3× chip rate (600 Hz) → 18 000 rpm ≈ 15.9 m/s
- `f_bp` = **frame rate (480 Hz)** → 14 400 rpm ≈ **12.8 m/s** — sits on autoc cruise

**Exact lock is benign** (fixed phase every frame ⇒ uniform attenuation ⇒ eaten by the energy-normalized
AGC, which 031 shows locks at 3% duty). **The near-miss is the hazard**: when the beat period approaches
the 75 ms code word, the occlusion phase walks a full cycle *within* one code, producing a systematic
attenuation ramp across the 15 chips — structured error, and flips cost 2× erasures. At cruise the beat
is |489 − 480| ≈ 9 Hz (114 ms), so phase advances ~66% of a cycle per code word. Predicted danger band:
`f_bp` = 480 ± 13 Hz ⇒ **14 010–14 790 rpm ⇒ 12.4–13.2 m/s**.

Duty estimate (assumption-laden, pending B1/C2/C4): ~13 mm chord + 8 mm pupil at r ≈ 35 mm, 14 400 rpm
⇒ 0.40 ms transit vs 2.08 ms blade-pass period ⇒ **~19% duty**.

**Note**: throttle is an NN output, so the controller can select its own position relative to these bands.

### Disc-crossing geometry (tractor confirmed, 2026-07-28)

Camera behind the disc by axial distance `d`, offset `h` from the thrust line. A ray at angle θ crosses
the disc plane at `h + d·tan θ`, and is occluded when that lands inside `|r| < r_tip` (69.85 mm for the
5.5″; 101.6 mm for the 8″).

**Coaxial mount (`h = 0`) is the bad case** — θ = 0 crosses at r = 0, i.e. **the target dead ahead sits
behind the hub/spinner**, which is precisely the tail-chase geometry. The clear region is an annulus at
the FOV edges:

| axial standoff `d` | FOV inside the disc |
|---:|---|
| 50 mm | θ < 54° (nearly the whole ±60° FOV) |
| 150 mm | θ < 25° |
| 300 mm | θ < 13° |

**Radial offset resolves it independently of `d`**: dead-ahead clears whenever **`h > r_tip`** — 70 mm for
the 5.5″ prop, 102 mm for the 8″. A wing-top or pod-top mount gets that for free. At `h = 80 mm`,
`d = 100 mm` the residual disc band is ≈ **6°–56° below boresight**, which is the cheap part of the FOV to
lose in a tail-chase.

**⇒ Treat camera radial offset as a design decision, not only a measurement.** If the mount is not already
≥ `r_tip` off the thrust line, moving it there is likely the highest-value zero-cost change available.

### Station stack (RESOLVED 2026-07-28 — prop = station 0, increasing aft)

| station | feature |
|---:|---|
| 0″ | prop disc (5.5×4, r_tip 2.75″) |
| 6″ | wing LE |
| **8″** | **camera** — 28.6% chord, i.e. genuinely the max-thickness apex |
| 13″ | wing TE (7″ chord) |
| 14″ | elevon TE (1″ elevon) |

Wing 30″ span × 7″ chord × **1″ max thickness**. Tail: boom 3″ to the front of the vertical stab, 4″ boom
length, fin ≈6.5″ × 4″ — **all aft of the camera ⇒ zero occlusion contribution**, recorded for completeness.

Camera sits **2″ aft of and 1″ above the LE**, which is what drives both calculations below.

### MEASURED 2026-07-28 (operator photo + sketch) — the camera does NOT reliably clear the disc

Measured: **r_tip = 2.75″** (5.5×4 prop), **axial standoff d ≈ 8″**, **camera height h = 2.5–3″** above the
thrust line. That straddles the clearance threshold:

| camera height | dead-ahead | prop-blocked band |
|---|---|---|
| **2.5″** | **BLOCKED** | +1.8° above → 33.3° below boresight |
| **3.0″** | clear by **1.8°** | 1.8° → 35.7° below boresight |

In a tail-chase the target sits at boresight and wanders several degrees, so **either height leaves the
prop disc overlapping the target's image region much of the time**. ⇒ Prop occlusion is a *primary*
perception effect for this airframe, not second-order. The §C resonance analysis is fully live.

**Cheapest remedy is lateral offset, not height** — radial distance from the thrust axis is what matters,
so the two combine in quadrature:

| mount | radial r | clear cone about boresight |
|---|---:|---:|
| 3″ up | 3.00″ | 1.8° |
| **3″ up + 3″ outboard** | **4.24″** | **~10.6°** |
| 4.2″ up alone | 4.20″ | ~10° (tall pylon) |

3″ outboard on a 15″ semi-span is structurally trivial and buys ~6× the clear cone; parallax cost stays
~1.3° at 3 m terminal range.

### Flush mounting gives ~zero downward view (corrects an earlier note)

With the camera sitting *at* the apex of the convex upper surface, the surface tangent there is horizontal,
so any downward ray intersects the wing immediately ahead of the camera. The binding constraint is the
standoff above the surface, not the LE position. Parabolic fit (1″ thickness over the ~2.1″ apex→LE run):

| standoff above wing | downward view |
|---|---|
| flush | ~0° |
| 0.25″ | ~25° |
| 0.5″ | ~34° |
| 1.0″ | ~44° (tangent reaches the LE) |

⇒ The camera needs a standoff for downward vision **and** radial offset for prop clearance — one piece of
hardware satisfies both.

### RESOLVED with the real station stack: the standoff alone is enough

Recomputed against camera-at-station-8 (LE exactly 2″ forward, 1″ below; parabolic fit k = 0.25/in). A
standoff raises the camera above the **thrust line** as well as above the wing, so it buys both at once:

| standoff | total height above thrust line | downward view (wing) | clear cone (prop) |
|---|---:|---:|---:|
| flush | 3.0″ | ~0° | 1.8° |
| 0.5″ | 3.5″ | 35.3° | 5.4° |
| **1.0″** | **4.0″** | **45°** | **8.9°** |
| 1.0″ + 3″ lateral | 5.0″ radial | 45° | 15.7° |

**⇒ The ~1″ standoff required for downward vision independently lifts prop clearance from 1.8° to ~9°.
Lateral offset becomes optional, not necessary.** This supersedes the lateral-offset recommendation above.

### Mount option: LEADING-EDGE camera (operator floated 2026-07-28) — the numbers strongly favour it

Motivation: *"putting a camera in the leading edge as an option if the prop disc is going to be annoying."*
Moving outboard raises the radial distance from the thrust axis far above `r_tip = 2.75″`, so the prop
shadow swings off to the inboard side; and at the very front of the wing **nothing is ahead of the camera**.

| | wing-top (4″ high, d = 8″) | **LE at 6″ outboard** (d = 6″) |
|---|---:|---:|
| radial distance from thrust axis | 4.0″ | ~6.3″ |
| **clear cone at boresight** | 8.9° | **~31°** |
| prop shadow location | 8.9–40.2° **below** boresight | 31–57° **inboard** |
| wing occlusion | everything below ≈45° | **none** |

⇒ **~3.5× the clear cone, and the lower field comes back entirely.** The shadow also moves away from where
a tail-chased target actually sits.

**Drag concern quantified — likely a non-issue.** A 1 cm² frontal bump at 6″ outboard is ≈ **1 gram-force**
at cruise (Cd≈1, q≈104 Pa at 13 m/s) ⇒ yaw moment ≈ 0.0016 N·m. Trivial against a 30″ span with a fin,
static and trimmable, and negligible beside a 25 ft towed streamer. **The real costs are structural**: foam
leading edge, a wiring run out the wing, and **crash exposure** — the LE is what hits things on a combat
plane, whereas wing-top is sheltered.

⇒ **Mount position becomes a model PARAMETER** (spec FR-011a) so the occlusion model can price this option
before foam is cut.

#### BASELINE ADOPTED 2026-07-28: leading edge, **~8″ outboard**, boresight ∥ thrust line

Prop-shadow sweep with the camera in the LE (station 6″ ⇒ d = 6″), `r_tip` = 2.75″:

| outboard offset | shadow spans | inside a ±60° frame? |
|---:|---:|---|
| 4″ | 11.8° – 48.4° | yes, badly |
| 6″ | 28.4° – 55.6° | yes |
| **8″ ← baseline** | **41.2° – 60.8°** | only beyond 41° inboard |
| 10″ | 50.4° – 64.8° | only beyond 50° |
| 13″ | 59.7° – 69.2° | effectively gone |

Full elimination at 120° FOV needs **~13″ on a 15″ semi-span** — a wingtip mount, with the crash-exposure
and foam-structure penalty that implies on a combat aircraft, for a 41°→57° threshold gain. **8″ is the
knee.** Wing occlusion is eliminated at *any* LE offset. The pod nose shadows the same inboard region and
merges into the same patch.

**⚠️ Occlusion is NOT fully "designed away", because of the variation envelope.** At 8″ the shadow starts
41° off boresight in *body* frame — but a camera glued **20° inboard** (the clipped worst case, F1) brings
it to **≈21° from its own boresight**. Design intent and build reality must be checked together ⇒ the
acceptance test is at the **clipped extremes**, not nominal (spec FR-007a).

⇒ US3 rescoped: occlusion is now a **design-validation tool**, not a fidelity model of an obstruction we
are stuck with. Effort shrinks (no wing slab, prop off-axis) but the machinery stays parameterized so a
later mount change can be priced.

**Second camera (deferred)**: symmetric at ∓8″ ⇒ **16″ / 0.41 m baseline** ≈ half the target's 0.772 m
beacon separation ⇒ stereo ranging ≈ **1.9× coarser** than span-based. Its value is *not* accuracy but
(a) range from a **single** visible beacon and (b) geometric false-blob rejection. Also cancels asymmetric
drag.

### Effective FOV to model (NOT 120×90 centred)

At `h = 4″` the prop shadow is an **ellipse ≈38° tall × 38° wide centred ~27° below boresight** (blocking
−8.9° to −40.2°); the wing cuts everything below ≈−45°. Net usable vertical field ≈ **+45° up to −9°
down** — about 54° of the nominal 90°, strongly asymmetric. Horizontal remains full ±60° at and above the
horizon. **Model the clipped shape, not the nominal rectangle.**

**Planned mount (operator 2026-07-28): above the thrust axis, at the peak of the wing chord.** Two checks
this creates:

- **Does it actually clear?** `h` ≈ (pod half-height) + (wing thickness). With a 17 mm battery in the pod
  and a foam combat wing that plausibly lands ≈ 35–45 mm — **short of the 70 mm needed**, leaving
  dead-ahead still crossing the disc. Measure before assuming the peak mount solves it. Remedies: a small
  standoff, or a lateral offset (the condition is `|radial| > r_tip` in *any* direction; 70 mm outboard
  works as well as 70 mm up, at a parallax cost of ~1.3° at 3 m terminal range — deterministic, calibrated
  out or simply learned).
- **The wing's own LE clips the lower FOV.** At the max-thickness point (~30% chord) the leading edge is
  ahead of and below the camera. For a ~230 mm chord at ~12% thickness the LE sits ~69 mm forward and
  ~14 mm down ⇒ **everything below ≈11° down is blocked by our own wing.** Stacked on the prop-disc band
  this means the effective FOV is **not 120×90 centered** but asymmetrically clipped below — model the
  clipped shape, not the nominal number. Needs real chord + thickness (A3/A4).

---

## D. Beacon / target craft

| # | Value | Unit | Feeds | Status | Current assumption + source |
|---|---|---|---|---|---|
| D1 | **Tip-to-tip beacon separation as mounted** | m | `range ≈ wingspan / span_angle` — the entire range-inference channel | ✅ | **0.772 m** = 30″ span (0.762) + 5 mm per side for a 1 cm cube centred on each tip. Sim currently uses ±0.45 m = **0.9 m** ⇒ **~17% too wide, fix it** |
| D2 | Beacon mount + outward-axis orientation on the wingtip | m / deg | Emission geometry; self-occlusion by the target's own wing (backlogged) | ✅ | **1 cm cube, one face against the wingtip**; centre 5 mm outboard; outward axis = ±y |
| D3 | Flight-cube LED count + geometry | — | Emission pattern `G(aspect)` | ✅ | **5 LEDs, one per exposed face** (outboard, fore, aft, up, down) — *exactly* `beam_axes_cube_minus_base()` in `plot_led_configs.py`, so sim and hardware agree by construction. Beam exponent pending E7 |
| D4 | Target craft — same HB1 article, or different? | — | Wingspan, occlusion, aspect behavior | ⚠️ | assume same class |
| D5 | **Towed streamer**: length, attachment point, material, width | m / mm | Perception: an occasional thin LOS obstruction. Fitness: see note below | ⚠️ | **~25 ft (7.6 m) tied to the back of the tail, draping — NOT a rigid projection aft, and it shortens as it is chopped** (operator 2026-07-28) |

**Streamer note (corrects an earlier assumption).** `TrailDistance = 3.048 m` (10 ft) in
`autoc-tracker.ini` is **not** the streamer's position — a full streamer is ~2.5× that long. The rabbit
is a *chosen aim point* roughly 40% down an uncut streamer, which is defensible (mid-streamer is the
fattest part of the target) but should be named as a choice rather than a physical landmark.

- **Perception (in 040, low priority)**: the streamer can cross the LOS to a wingtip beacon when the chase
  is close and directly astern. A ~40 mm ribbon at 5 m subtends ≈8 mrad ≈ 1.2 px ⇒ a brief partial-pupil
  obstruction, an **erasure source, not a gate** — same character as the prop. Otherwise invisible to an
  850 nm bandpass camera. Note it; do not model in v1.
- **Fitness (NOT 040 — backlog candidate)**: the real target is a **distributed, deformable, shrinking**
  object whose shape depends on airspeed / maneuver / gravity and whose length changes as it is cut. The
  fitness models it as a rigid point at a fixed offset. That is a target-modeling question, not a
  perception one, and belongs in its own backlog item per [feedback_clear_objectives_not_tuning] — does
  the reward point at the physical thing?

---

## E. Optical link budget — 031-fed, mostly confirmations

| # | Value | Unit | Status | Source |
|---|---|---|---|---|
| E1 | Field emitter current | mA | ✅ | **306 mA field / 51 mA bench** — but see the re-check below, the current gain does not deliver what the doc credits |
| E2 | Flux-constant anchor | µA·m²/r² | ⚠️ | measured 1.1–1.6; pick one |
| E3 | Decode floor | nA | ⚠️ | ≤10 nA measured — confirm as the sim lock threshold |
| E4 | Ambient photocurrent, overcast vs direct sun, **post-filter** | µA | ⚠️ | bare-PD 30–300 µA; 70 µA @ 47 kΩ compression ceiling; ÷7–25 with filter |
| E5 | Code parameters | — | ✅ | N=15 Gold, 200 Hz chips, 75 ms word, **full-duty chips** (locked 2026-07-18) |
| E6 | Decoder behavior | — | ✅ | CDMA ≈ 1 SNR tier; flip ≈ 2× erasure; AGC locks at 3% duty; HOLD ≈ 2 periods; warm relock ≲ 1 period |
| E7 | Emitter beam width + pattern shape | deg | ✅ | **RESOLVED from the datasheet 2026-07-28** — Lumileds DS190 Table 1, L1IZ-0850: **150° typical FWHM**, 286 mW/sr radiant intensity, 1250 mW radiometric power, 30 nm spectrum FWHM @ 1000 mA. `optical-link-outcome.md` was right; **`plot_led_configs.py`'s `HPBW_DEG = 130.0` is a DEFECT** — it attributes 130° to this datasheet. Fix at source. **Shape (operator)**: relative intensity is **flat to ≈±45°**, then rolls through 50% at ±75° ⇒ **flat top with shoulders, NOT `cos^m`** (a cosine fit under-reads the flat region ~16% and over-reads the skirt) |

### E-RECHECK 2026-07-28 — the ×2.3 field-current range gain does NOT survive the geometry change

Operator asked for a link-budget double-check. **Finding: the bench and flight emitters are not the same
optical source, and the doc's scaling implicitly holds the bench geometry.**

- **Bench**: 5× L1IZ-0850 in series, **co-aimed** ⇒ all five add on-axis = `5 × I₁(51 mA)`. The 12.5 m lock
  was measured "with good alignment", i.e. on-axis.
- **Flight cube**: one LED per exposed face (outboard, fore, aft, up, down). At any viewing direction only
  ~1–2 contribute; the rest sit at 90° where `cos^m(90°) = 0`.
- **Critically, the chase sits ASTERN**, so it views a wingtip beacon at ~90° off the cube's outboard axis
   — through the **aft-facing** LED.

**REVISED 2026-07-28 with the correct emission model (E7).** The first pass used `cos^0.805` and was too
pessimistic. With the datasheet's 150° FWHM and a **flat top to ±45°**, adjacent dies' flat regions *tile*
the outboard hemisphere, so the cube is both more uniform and brighter than a cosine fit predicts:

| | first pass (`cos^0.805`) | corrected (flat-top ±45°, 150° FWHM) |
|---|---|---|
| half-cube relative output | 1.0 – 1.9 | **2.0 – 2.6** |
| field vs bench intensity | 1.06 – 2.0× | **2.1 – 2.8×** |
| range multiplier (√) | ×1.03 – 1.42 | **×1.45 – 1.66** |
| **field bare range** | 13 – 16 m | **18 – 21 m** |

⇒ **`optical-link-outcome.md`'s 29 m is still optimistic, but by ~1.4× rather than ~2×.** Carried
downstream with collection optics (×10–25 amplitude ⇒ ×3.2–5 range): **58–105 m** against the 100 m goal —
reachable at the optimistic end. Daylight remains the binding case (field test #3 measured 4.5–6 m
bare/unfiltered).

**The finding stands, softened**: the ×2.3 field-current gain still double-counts, because the bench's five
co-aimed dies do not carry over to a cube that aims them in five directions. The magnitude is smaller than
first estimated.

**This is not a design error.** The half-cube is doing its job — trading on-axis concentration for the
hemispherical coverage a banking target requires. It means only that the current increase buys far less
than credited, and 100 m in daylight is at the optimistic edge. Levers if it matters: more LEDs per face,
bias the **aft** face (where the chase actually lives), or lean harder on collection optics.

---

## F. Variation sigmas — PRNG class slot 5 (`camera`)

Slot 5 of `deriveClassSubSeeds` is reserved and unused; `ScenarioMetadata` has the documented
`cameraSeed` append point. Mount alignment is the headline axis per the backlog (precedent:
[project_board_alignment] — a 170°-vs-180° error put ~10° of pitch bias into flight data).

**SPECIFIED 2026-07-28 (operator)**: *"mostly orientation, very subtle for position — assume we put it in a
1 cm box across a few articles, but each one may be off by gaussian 10 deg worst case 20 deg in the
pointing direction and a little rotation error ±10 up to 20. It just gets glued on the top of the wing
somewhere."*

| # | Value | Unit | Status | Value |
|---|---|---|---|---|
| F1 | Camera **boresight** (pointing) error | deg | ✅ | **σ = 10°, clip 20°** — 2 DOF (azimuth + elevation) |
| F1b | Camera **roll** about the optical axis | deg | ✅ | **σ = 10°, clip 20°** |
| F2 | Camera **translation** error | mm | ✅ | within a **1 cm box** (≈ ±5 mm per axis) |
| F3 | Beacon mount position + angular error on the wingtip | mm / deg | ⚠️ | not yet specified |
| F4 | Ambient level distribution (overcast → direct sun) | µA | ⚠️ | see §E |
| F5 | **Wing thickness** (folded foam board, "highly variable") | in | ⚠️ | nominal 1″; σ TBD |
| F6 | Is prop diameter a variation axis? (5.5″ vs 8″ = 45% disc swing) | — | ⚠️ | open |

**RESOLVED**: **hard clip at 20°** (operator) — *"don't want to train on a 45 deg offset camera."* Sample
σ = 10°, clip, do not sample the tail.

### Related gap — attitude-reference (IMU) misalignment → BACKLOG (operator 2026-07-28)

The flight controller carries a board-alignment setting (pitch/roll/yaw offsets) and position updates come
from its estimator. Real precedent: [project_board_alignment] — a 170°-vs-180° roll setting put a **~10°
pitch bias** into recorded flight data. **Not modelled anywhere today.**

**It is a distinct axis, not more of the same**: camera misalignment biases where the target *appears*;
attitude misalignment biases where the craft *believes it is pointing*. The NN receives both independently
(beacon bearings vs the attitude quaternion), so the errors neither cancel nor imply one another — a
controller robust to camera error is not thereby robust to attitude error. Natural follow-on to the camera
variation work here. **Trigger**: after camera variation is exercised, or any flight where attitude bias is
suspected.

### Two consequences of the F1/F2 split

1. **Position variation is negligible for BEARING but material for OCCLUSION.** ±5 mm subtends 0.03° at
   10 m and 0.003° at 100 m — nothing beside 10° of pointing error. But the same ±5 mm swings the prop
   clear cone by ~15%, because `clear = atan((h − r_tip)/d)` has a small numerator:

   | camera height | clear cone |
   |---|---:|
   | 96.6 mm (−5 mm) | 7.5° |
   | 101.6 mm (nominal 4″) | 8.9° |
   | 106.6 mm (+5 mm) | 10.3° |

   ⇒ Feed translation into the **occlusion** path; it may be ignored in the bearing path.

2. **Roll error corrupts `target_tilt` one-for-one.** Boresight error under the angular representation is
   ≈ a constant additive NDC offset — clean and learnable, exactly as the backlog predicted for angular
   NDC. But roll about the optical axis *rotates the image plane*, biasing the port→starboard tilt angle
   directly. At σ = 10° (clip 20°) the tilt cue — a roll-command input — is wrong by that much. **This is
   the variation most likely to change controller behaviour**, and the best robustness target of the set.

---

## G. Iterations + re-runs to perform

| # | Action | Produces | Depends on |
|---|---|---|---|
| **G1** | **Re-run eCalc propCalc** — field-by-field deltas in the table below. Use the **Download .csv** button so both partial-load tables come through as data, not a PDF render | C5 — the authoritative RPM / throttle / airspeed table | C1, C6, C7 |
| **G2** | Fit the **throttle→RPM map** from the G1 table; drive prop phase from it | Deterministic prop phase for both the crrcsim and minisim paths, with no new coupling to the FDM integrator | G1 |
| **G3** | Decide whether to reconcile the FDM prop (`<propeller D H>` in `hb1_streamer.xml`, currently the stock 5×4.5) with the flown prop | A thrust/speed-model question, **separate from occlusion** — flag only, don't fix inside 040 | C1 |
| **G4** | Reconcile eCalc wing area (270 in²) against the 7″ pod chord limit | Wing slab dimensions + beacon separation | A3 |
| **G5** | Pin the post-filter ambient pedestal on the 031 bench; resolve the **C-14** filter FWHM decision | E4, B5 | 031 order-03 parts |
| **G6** | **Bench-measure blade occlusion directly** — put the existing single-PD rig behind a spinning prop, sweep RPM, log `corrB` / `margin` ripple vs the predicted commensurabilities | A *direct empirical test* of the §C resonance prediction, using an instrument that already exists and is regression-covered. Would either retire the concern or size it before any modeling effort | prop + PSU + `monitor.sh` |

**G6 is the high-value one.** The 031 receiver is already a calibrated instrument with scripted telemetry;
pointing it through a spinning prop at the emitter tests the entire resonance hypothesis for the cost of a
bench afternoon, and it distinguishes "uniform attenuation the AGC eats" from "structured per-chip ramp"
directly rather than by inference.

### G1 field deltas vs the prior propCalc run

| field | prior | change to | why |
|---|---|---|---|
| **Wing Area** | 17.42 dm² / **270 in²** | **13.55 dm² / 210 in²** | ❌ **The error.** 270 in² ⇒ 9″ mean chord; measured chord is 7″ and 7″×30″ = 210 in². FDM agrees with 210. Inflated lift ⇒ optimistic stall + level speeds ⇒ contaminates the **airspeed axis** |
| Drag coefficient | 0.18 Cd | judgment | 0.18 is clean-airframe; a ~25 ft streamer is a large drag item. Affects the airspeed axis only |
| Model Weight | 520 g incl. drive | confirm | Current flying weight; also whether the camera-carrying article is heavier |
| Motor | EcoII 2207-**2400** | confirm | Airframe spec also lists 2207-1700kv; KV scales the entire RPM curve |
| Battery | custom 3S 1000 mAh, 0.0113 Ω, 65C/100C | confirm | Internal resistance + C-rate set sag under load ⇒ **RPM at high throttle**, the part we care most about |
| Controller | 20 A cont/max | confirm | Only binds if it limits before the motor |
| Propeller | Master Airscrew GF 5.5×4, 2-blade | ✅ confirmed (photo) | Windsor Propeller = Master Airscrew; just verify the dropdown is the **GF** series |
| Field elevation / temp | 0 m, 25 °C | optional | Set to typical flying conditions; small effect |
| Flight Speed | 0 km/h | optional ~47 km/h | Puts the main gauges at cruise; tables emit regardless |

**Why wing area matters less than it looks**: the map of record is **throttle→RPM**, and the *static* table
is airspeed-independent by construction, so it is already trustworthy. Fixing wing area makes the *dynamic*
table's speed axis usable too, enabling a proper airspeed correction. From the prior tables that correction
is modest — ≈ **50 rpm per m/s at low throttle, ≈13 rpm per m/s at high throttle** — second-order against
resonance bands ~±390 rpm wide, but not nothing.

---

## Blocking summary

Nothing in §D–§F blocks the spec. The blocking set is:

**Closed 2026-07-28**: A1 (tractor) · A3 (30″ span / 7″ chord) · A4 (1″ thickness) · A5 (station stack) ·
A5b (tail) · C1 (5.5×4) · C2 (8″) · C4 (~0.5″ chord) · partial A2 (2.5–3″ height at the wing surface).

**Also closed 2026-07-28**: A2b (boresight = **parallel to the thrust line, no tilt** ⇒ the +45°/−9°
asymmetry stands; up-tilt remains an available lever) · A4b (**flat-bottom folded foam board, highly
variable** ⇒ thickness is a **variation axis**, not a constant — suits PRNG slot 5, and the wing-occlusion
tangent math should be a distribution) · B2 (parameterize with a knob) · D1 / D2 / D3.

**⛔ Blocking: NONE — both cleared 2026-07-28.**

- **LENS / FOV → DECIDED: keep 120°, design CEP to ~100 m detection** (operator). Rationale: *plumbing
  first* — get the structure and the knobs in place while field hardware is being proven; numeric
  calibration lands later with real→sim measurement. The ~40 dB wide-field shortfall and the
  sensor-format problem are **recorded as calibration targets, not blockers** (spec FR-035, US7).
  Consequence now in the spec as FR-033: **bearing to ~100 m, separation-derived range only inside
  ~25 m** — two envelopes, modelled separately.
- **E7 → RESOLVED from the datasheet**: 150° FWHM, flat top to ±45°. See the E7 row.

**Note**: the camera link budget does *exist* (`spec.md` FR-1.4 — it drove 300 mA) but covers a narrow
optic only. Re-deriving it for a wide lens is a **calibration** task, deferred with the photon budget to
article 1.

**Still ⚠️ (carryable, values not structure)**: A1b thrust-line position · A2 camera mount + A2c standoff ·
A4b airfoil camber · B1 pupil diameter · B2 exposure duty · §E filter FWHM, flux anchor, ambient pedestal ·
§F variation sigmas.

**⚠️ Carryable as stated assumptions:**

- **A2c** camera **standoff height above the wing** — a design decision; ~1″ recommended (buys 45° down
  view + 8.9° prop clearance in one bracket)
- **A1b** CG station, to place camera + prop in the sim body frame (FDM has `CG_arm="0.28"`)
- §E remaining — filter FWHM (open C-14), flux-constant anchor, ambient pedestal
- §F variation sigmas (now including **wing thickness** per A4b)

Everything else can be specified against the stated assumption and corrected later without structural
change — these values feed slots, not architecture.
