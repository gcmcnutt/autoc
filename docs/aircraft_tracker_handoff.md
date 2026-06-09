# Aircraft Beacon Tracker — Design Handoff

**Status:** architecture locked, prototype hardware selection complete, firmware & DSP design ready to start.
**Audience:** Claude Code (or any engineer) picking up implementation in the software/firmware repo.

---

## 1. System overview

An RC plane (the *tracker*) carries a forward-looking camera that locates another RC plane (the *target*) by detecting two coded IR beacons mounted on the target's wingtips. The frontend processor (FPGA) consumes raw camera frames and emits, on demand, the normalized image-plane coordinates and confidence of each wingtip beacon. The autopilot polls this state at 10–20 Hz and runs its own tracking filter.

### 1.1 Top-level data flow

```
[Target aircraft]
  ├── Left wingtip:  IR LED pod, modulated with code A
  └── Right wingtip: IR LED pod, modulated with code B
                          │
                          │  850 nm photons
                          ▼
[Tracker aircraft]
  ┌─────────────────────────────────────────────┐
  │ Camera module (sensor + lens + NIR filter)  │
  │   MIPI CSI-2 → FPGA                         │
  └─────────────────────────────────────────────┘
                          │
                          ▼
  ┌─────────────────────────────────────────────┐
  │ FPGA frontend                                │
  │   - MIPI RX → streaming pixel pipeline       │
  │   - IIR background subtract → 1-bit mask     │
  │   - Single-pass CCA → blob descriptors       │
  │   - Multi-frame track table (≤8 tracks)      │
  │   - Bipolar code correlation per track       │
  │   - Lens-distortion LUT → int8 x, y, u8 CEP  │
  │   - I²C slave register file (double-buffered)│
  └─────────────────────────────────────────────┘
                          │
                          │  I²C @ 400 kHz–1 MHz, polled at 10–20 Hz
                          ▼
                     [Autopilot MCU]
```

### 1.2 Output abstraction

The autopilot never sees pixels. The FPGA reports, per beacon, just three bytes:
- `x_i8` — signed int8, normalized horizontal image coordinate, range maps to `[-1, +1]`
- `y_i8` — signed int8, normalized vertical image coordinate
- `cep_u8` — uint8, hyperbolic mapping to a CEP-like quality value; **value 255 (0xFF) means "no detection"**

Six bytes total for both beacons. A few additional aux bytes (frame counter, timestamp, gain, SNR proxy, status flags) are available in adjacent registers for hosts that want them. See §7 for the full register map.

This is the canonical interface; everything else is an implementation detail.

---

## 2. Mission constraints (the spec that drove every decision)

| Constraint | Value | Implication |
|---|---|---|
| Camera horizontal FOV | ≥120° | Wide-angle M12 lens, ~1.1–1.7 mm focal length depending on sensor |
| Frame rate | ≥30 fps, prefer 100–200 fps | High frame rate → more code samples per acquisition window |
| Shutter | Global preferred | Avoids IMU-rate-feed-forward into FPGA at 500°/s body rates |
| Camera mass | 2–3 g target (flight) | Drives bare-die or minimal-flex sensor mounting |
| Detection range | ~100 m | Drives LED radiant intensity and processing gain |
| Beacon coverage | ~270° per wingtip | Multi-die LED pyramid, not single emitter |
| Beacon mass | 3–4 g per wingtip | Constrains LED count and driver size |
| Body rate (tracker) | up to 500 °/s pitch/roll | Settles the global-shutter argument |
| Autopilot poll rate | 10–20 Hz | I²C @ 400 kHz is more than sufficient |
| Acquisition time | ≤50 ms (half the autopilot sample interval) | 10 frames @ 200 fps = one full code period |

---

## 3. Architectural decisions (and why)

The architecture went through several iterations. These are the decisions and the reasoning, recorded so they don't get re-litigated.

### 3.1 Detection scheme: temporal coding, not spectral filtering

**Decision:** Both beacons emit at the same wavelength (850 nm), distinguished by orthogonal modulation codes.

**Rejected alternative:** Two wavelengths (e.g., 850 + 940 nm) with a custom dual-narrowband filter and per-band detection.

**Why the temporal route wins:**
- Custom dual-narrowband filters are expensive and have to be specified per-vendor (Iridian, Alluxa, etc.)
- A single off-the-shelf 850 nm bandpass filter (~$30–50, ~0.3 g) does the optical clutter rejection
- Code orthogonality + matched-filter processing gain (>15 dB for a 7-bit code) handles the rest
- Same LED part number on both wings → simpler BOM
- Single sensor, single optical path

The optical filter still earns its keep: it prevents the sensor from saturating on broadband sun and it knocks daylight ambient down by ~50× before the temporal correlator sees it.

### 3.2 Shutter: global, not rolling-with-correction

**Decision:** Global shutter sensor.

**Why:** At 500°/s body rate, a 5 ms rolling-shutter readout window produces ~17 pixels of skew across an 800-pixel-wide frame. Correctable in the FPGA via per-row warp driven by IMU samples synchronized to ~100 µs — but this couples the vision frontend to the autopilot's IMU and adds a calibration burden. Global shutter eliminates this entire coupling.

### 3.3 Wavelength: 850 nm

**Decision:** 850 nm for everything (LEDs, filter, sensor optimization).

**Why:** Silicon QE at 850 nm is ~2× what it is at 940 nm. With temporal coding handling disambiguation, there's no reason to pay the QE penalty for 940 nm. The faint visible red glow at the LED is acceptable for an RC test article.

### 3.4 Resolution: low is fine

**Decision:** VGA-class resolution (e.g., 800×704 on VD55G1, 640×480 cropped on OV9281) is sufficient.

**Why:** The output is two centroids, not an image. At 120° H FOV and 800 H pixels, each pixel subtends 0.15° ≈ 26 cm at 100 m. Sub-pixel centroiding to 0.1 px gives 2.6 cm CEP at 100 m — well below any meaningful tracking error.

### 3.5 Single camera, with multi-cam as a future experiment

**Decision:** Start with one forward-facing camera. Architect the I²C interface to support multi-cam by address.

**Why:** Three cameras at 0°/+60°/-60° give ~270° forward coverage matching the beacon coverage, but that's a phase-2 problem. Same firmware, same register map, different I²C address jumpered at boot.

### 3.6 Event cameras as future option

Prophesee IMX636-class event sensors map naturally onto pulsed-beacon detection (look for spatially clustered events at the modulation frequency). Heavier today (~5–10 g for off-the-shelf modules) and outside the canonical start, but worth a phase-2 evaluation.

### 3.7 The dynamics problem

Both aircraft are moving, the tracker is rotating, and the correlation window spans many frames. This means **the beacon does not stay at one pixel location during the correlation window**. The previous architecture's "per-track ring buffer" had to be sharpened to handle this honestly.

**Motion budget at the worst case:**

| Source | Apparent rate | Per-frame @ 200 fps | Over 7-frame window |
|---|---|---|---|
| Target lateral motion at 30 m, 30 m/s | ~150 px/s | 0.75 px | 5 px |
| Tracker body rate at 500°/s | 1330 px/s | 6.7 px | **47 px** |
| Combined worst case | — | ~7 px | ~50 px |

At 320×240, a 50-pixel excursion crosses ~15% of the frame during one correlation window. The correlator must operate on intensities of a *moving* blob, not on pixels at a fixed image location.

**Architectural consequences:**

1. **Stage 3 (track table) becomes load-bearing**, not an afterthought. It must run a per-track motion predictor and gate the cross-frame data association predictively.
2. **Stage 4 (code correlator) operates on intensities of associated blobs**, not on pixels at fixed coordinates. The math is unchanged; the input is the intensity time-series from the track, not from a pixel.
3. **CEP must grow during fast maneuvers** — the position uncertainty into the next frame depends on the body rate and the track's miss history. CEP composes contributions from centroid precision, motion uncertainty, and code-lock confidence.
4. **Output is the *latest frame's* measurement**, not an average over the correlation window. A stale-but-correlated position is less useful than a fresh, marginally-validated one.
5. **Code length sweet spot is short** (7 chips). Longer codes give more processing gain but the track may be lost mid-correlation during high-rate maneuvers, degrading the input ring with zero-fills. Use chip-per-frame oversampling for SNR rather than longer codes.
6. **Body-rate compensation is optional but valuable.** If the host pushes its current angular rates over I²C, the FPGA can predict pure-rotation motion exactly, leaving only target-translation motion for the per-track velocity estimator to handle. Without body-rate input, search gates must be wider and tracks more likely to break during fast maneuvers. We support both modes, default to off (no IMU coupling).

---

## 4. Hardware selection

### 4.1 Sensor

**Primary choice: ST VD55G1**
- 800×704 mono, global shutter, MIPI CSI-2 (1 lane) + I3C
- Die: 2.73 × 2.16 mm — very small
- Built-in AEC/AGC, programmable contexts
- Designed for AR/VR NIR-illumination eye tracking — matches our use case
- Auto wake-up mode (potential power optimization)
- ~35 mW @ 48 fps

**Backup choice: OmniVision OV9281**
- 1280×800 mono, global shutter, MIPI CSI-2 + DVP parallel
- 1/4" optical format, larger but mature ecosystem
- Class-leading 940 nm QE; still excellent at 850 nm
- Open-source schematics from Antmicro and Luxonis (OV9282 = electrically identical)
- Arducam B0162 module is the easiest dev path

**Rejected:**
- Himax HM01B0/HM0360 — rolling shutter, disqualified
- OV6211 — older, only 400×400, marginal
- OV7251 — workable VGA fallback if VD55G1/OV9281 sourcing fails

### 4.2 Lens

**Vendor: Commonlands (San Diego, CA)**

Why:
- US-based, direct optical-engineering contact
- Same-day shipping from stock
- Offers M12 lenses with **integrated 850 nm bandpass filter in the housing** — saves separate filter alignment and mass
- IR-corrected coatings hold focal-shift <8 µm from visible to 850 nm — no defocus when switching from visible alignment to operational NIR
- Custom configs available on focal length, F/#, CRA, and bandpass center

Spec to request:
- M12 mount, NIR-corrected
- ~120° H FOV on the chosen sensor format
- F/2.0 or faster
- 850 ± 10 nm bandpass filter integrated (ask for `M12ANIR` style mounting with the filter pre-installed; no IR-cut)
- Sensor-specific CRA optimization (provide VD55G1 or OV9281 datasheet)

For VD55G1 (1.73 × 1.73 mm pixel array, 1/10" format): focal length ~1.5–1.7 mm.
For OV9281 (3.84 × 2.4 mm sensor, 1/4" format): focal length ~1.1–1.3 mm.

For prototype-only experimentation, generic 145°-fisheye M12 lenses (e.g., m12lenses.com PT-02120, ~$30) work fine; swap to Commonlands for flight units.

### 4.3 FPGA — prototype dev board

**Primary: Lattice CrossLink-NX Evaluation Board (CrossLink-NX-EVN)**

Why:
- **Hardened MIPI D-PHY** built into silicon — no soft-PHY timing closure pain
- Lattice Radiant has reference designs for OV9281-class CSI-2 RX with I²C config
- Open-source toolchain support via Project Oxide / nextpnr-nexus (also free Radiant)
- 28 nm low-power process, ~150 mW typical for vision workloads
- Antmicro has open-hardware CrossLink-NX boards as a custom-board reference
- GitHub `circuitvalley/USB3_MIPI_CSI2_RX_V2_Crosslink_NX` — tested Verilog MIPI CSI-2 RX (IMX219/IMX477) — adapt by changing the I²C init script

**Backup: Digilent Arty S7-50 (Spartan-7 + Vivado)**
- Familiar tooling, more community resources, ~$150
- **No hardened MIPI** — must use OV9281 in DVP parallel mode (incompatible with VD55G1)
- Acceptable Plan B if team has zero Lattice experience

**Flight-weight path:** Same FPGA (Lattice LIFCL-17 in 6×6 mm BGA) on a 30×40 mm 4-layer custom carrier — total flight PCB 3–5 g.

### 4.4 LED beacons (one pod per wingtip)

**Primary LED:** Lumileds LUXEON IR Compact, 850 nm
- 1.3 W radiant @ 1 A drive, T_j=25°C
- 2.75 × 2.0 mm package, ~30 mg per die
- Standard 130° emission pattern
- Pulsable to 5–10 A peak at low duty (well within thermal envelope when modulated)

**Alternative:** OSRAM SFH 4715AS, Kingbright high-efficiency 940 nm (only if covertness requires) — interchangeable in the pyramid mechanical design.

**Pod construction:** 3–4 dies arranged on the facets of a small 3D-printed pyramid (or hexagonal prism) facing outward at the wingtip. Three 130° dies at 120° azimuth spacing give >270° coverage with overlap. Total 3–4 g per pod including driver, FET, decoupling, and pyramid.

LED driver detail in §5.

---

## 5. LED driver / beacon design

The beacon side is straightforward analog/power-electronics, but a few details matter for SNR and EMI.

### 5.1 Power architecture

Each wingtip pod is fully self-contained: input is the airframe DC bus (typically 7.4 V from a 2S LiPo or 11.1 V from a 3S), output is modulated NIR.

```
Bus Vin (7.4 V or 11.1 V)
    │
    ├──► Buck converter or LDO → V_LED rail (~3.5–4.5 V)
    │     (provides headroom over LED Vf ~1.7–2.0 V for current sense)
    │
    └──► Microcontroller / code generator (3.3 V LDO from Vin)
              │
              │ digital code stream @ ~1–2 kHz chip rate
              ▼
         Gate driver  →  N-MOSFET  →  LED string with sense resistor
                                          │
                                          ▼
                                       (3–4 LEDs, parallel facets,
                                        each with own current path)
```

### 5.2 Component picks

| Block | Recommended part | Notes |
|---|---|---|
| Buck regulator | TPS62933 or MP2451 | 3 A, small, ~95% eff, fits 0.3 g |
| MCU / code generator | ATtiny1616 or RP2040 | ATtiny is lighter, RP2040 is overkill but trivial to program |
| Gate driver | Integrated FET drive in MCU pin (low side, with logic-level FET) — no separate driver needed at <5 A | Saves mass and parts |
| Current-sense FET | DMN2058U or similar SOT-23 logic-level N-FET, R_DS(on) <50 mΩ | One per facet for independent control |
| Current-sense resistor | 0.1 Ω 1% 1206, in series with each LED string | Sets current; 0.1 Ω × 1 A = 100 mV across, easy ADC monitor |
| LED | Lumileds Luxeon IR Compact 850 nm | See §4.4 |

### 5.3 Modulation timing

- **Chip rate:** 1 kHz (1 ms per chip) — comfortably above mechanical motion blur but slow enough that the camera at 200 fps gets 5 frames per chip
- **Code length:** 7-bit Gold code or m-sequence — 7 ms per code period
- **Window:** repeat the code continuously; camera demodulates over a 7-period sliding window for confidence
- **Duty:** 50% nominal; pulse current can be 2–3× CW rating during ON chips since average is half

The MCU needs only one timer interrupt, a code lookup table, and a GPIO toggle — no real DSP load. Power consumption is negligible (~5 mW MCU, ~2 W peak LED).

### 5.4 Synchronization (or not)

The cleanest design is **asynchronous**: the LEDs run on their own clock, the camera runs on its own clock, and the FPGA matched filter handles the small frequency offset (parts-per-million between the LED MCU's crystal and the camera's clock). The matched filter naturally tolerates ±100 ppm offset over a 50 ms window with negligible loss.

If tighter sync is ever needed (e.g., for very long codes), the tracker can broadcast a sync pulse on a separate RF channel — but **don't add this complexity to the canonical design**.

### 5.5 Code orthogonality

For two beacons:
- Generate a pair of 7-bit Gold codes (from a single LFSR pair) with cross-correlation ≤ −15 dB
- Assign code A to left wingtip, code B to right
- Burn the code ID into MCU EEPROM at flash time (or read a wire jumper at boot)
- Future-proof for >2 beacons (e.g., multiple aircraft) by reserving code-pair slots in the FPGA register map (`code_select`)

### 5.6 Power budget reality check

At 100 m with a 130° emitter at ~1.3 W radiant, on-axis irradiance at the camera ≈ 1.3 W / (2π × (1−cos 65°) × (100 m)²) ≈ 3 µW/cm² before the bandpass filter. With ~85% filter transmission, ~2.5 µW/cm² hits the sensor. With OV9281's 850 nm sensitivity (~10 V/(lux·s) equivalent at NIR), this gives a strong signal at 1 ms exposure even before correlation gain. With 7-bit code processing gain, link margin against full daylight is comfortable.

---

## 6. FPGA DSP architecture

The frontend's job is to take a high-frame-rate stream of frames and produce two centroids with confidence at 10–20 Hz polling. The previous draft of this section described a "per-pixel matched filter," which was misleading shorthand. Per-pixel correlators are infeasible at any meaningful image size. The real architecture is **hierarchical**: a cheap whole-frame screening pass identifies a small bounded number of candidate regions, then full code correlators run only on those regions.

This is the same pattern used in GPS acquisition (cheap FFT-based search, then full correlators on a few hypotheses), in star trackers (centroid first, identify second), and in published single-pass FPGA blob-analysis literature (Bailey-Johnston single-pass CCL with on-the-fly moment accumulation, used at >200 fps in older/smaller FPGAs than ours).

### 6.1 Working at low resolution is a feature, not a compromise

The output is 8-bit signed x and y (256 distinct positions per axis). At 8-bit precision, **a 320×240 capture mode is more than enough**. Specifically:

- 320 px ÷ 256 output codes = 1.25 px per output step → trivially achievable with sub-pixel centroiding (centroids easily reach 0.1 px precision)
- At 120° H FOV / 320 px ≈ 0.375°/px ≈ 65 cm/px at 100 m → 0.1 px ≈ 6.5 cm precision — way better than CEP ever needs to be
- Frame rate at 320×240 on both VD55G1 and OV9281 can hit 300+ fps, giving more code samples per acquisition window
- Memory budget for any per-pixel state drops by ~7× vs full resolution (320×240 = 77 k px vs 800×704 = 563 k px)

**Decision:** target sensor binning/cropping to ~320×240 for the canonical first build. Higher modes are a configuration option, not the default.

### 6.2 Pipeline overview

```
                  MIPI CSI-2 (1 lane)
                          │
                          ▼
           ┌──────────────────────────────┐
           │  MIPI D-PHY hard IP          │
           │  (Lattice CrossLink-NX HW)   │
           └──────────────────────────────┘
                          │ pixel stream + line/frame sync
                          ▼
           ┌──────────────────────────────┐
           │  Sensor unpack & frame ID    │
           │  - 8b output                 │
           │  - frame counter, timestamp  │
           └──────────────────────────────┘
                          │
                          ▼
           ┌──────────────────────────────┐
           │ STAGE 1: Per-pixel screen    │
           │  - IIR background tracker    │
           │  - Δ = max(0, px - bg)       │
           │  - threshold → 1-bit mask    │
           │  - 1 bit/pixel, streaming    │
           └──────────────────────────────┘
                          │
                          ▼
           ┌──────────────────────────────┐
           │ STAGE 2: Single-pass CCL     │
           │  - Bailey-Johnston style     │
           │  - emits blob descriptors:   │
           │    {id, x̄, ȳ, area, peak,    │
           │     bbox, σ_xx, σ_yy, σ_xy}  │
           │  - bounded blob count (16)   │
           └──────────────────────────────┘
                          │
                          ▼
           ┌──────────────────────────────┐
           │ STAGE 3: Blob track table    │
           │  - data-association across   │
           │    frames (gating by bbox)   │
           │  - per-track ring buffer of  │
           │    intensity samples         │
           │  - bounded track count (8)   │
           └──────────────────────────────┘
                          │
                          ▼
           ┌──────────────────────────────┐
           │ STAGE 4: Code correlator     │
           │  - 2 codes × 8 tracks max    │
           │  - 7-tap, bipolar codes      │
           │  - shift-add only, no mult   │
           │  - identifies which track is │
           │    code A vs code B          │
           └──────────────────────────────┘
                          │
                          ▼
           ┌──────────────────────────────┐
           │ STAGE 5: Output formatter    │
           │  - distortion LUT correction │
           │  - normalize to int8 x, y    │
           │  - hyperbolic CEP map → u8   │
           │  - double-buffered regs      │
           └──────────────────────────────┘
                          │
                          ▼
                   I²C slave  →  Autopilot
```

The key property: **stages 1 and 2 process the full pixel stream once, in raster order, with no frame buffer**. Stages 3, 4, 5 work on a small bounded list of blob descriptors, not pixels. This is what makes it fit in a small FPGA at high frame rates.

### 6.3 Stage 1 — Per-pixel screen

For each incoming pixel `px[i,j]` at frame `t`:

```
bg[i,j,t]  = bg[i,j,t-1] + ((px[i,j,t] - bg[i,j,t-1]) >> 6)   // α = 1/64 IIR
Δ[i,j,t]   = max(0, px[i,j,t] - bg[i,j,t])
mask[i,j]  = (Δ > thresh)                                      // 1 bit
```

Background storage: 8 bits/pixel × 320×240 = 77 kbit. Trivial.
Threshold is auto-tuned from the global noise estimate (running variance of `Δ` over background-only pixels), updated once per frame. Range typically 4–16 LSBs of an 8-bit pixel.

This stage outputs a 1-bit binary mask synchronized to the pixel stream. It also forwards the raw `Δ` value alongside, because stage 2 needs intensity weights for sub-pixel centroiding.

### 6.4 Stage 2 — Single-pass connected-component analysis (CCA)

This is the standard Bailey-Johnston single-pass architecture, well-documented in FPGA vision literature. Key properties:

- Operates on the streaming binary mask in raster order
- Maintains a row buffer of provisional labels for the last row (320 entries × 8-bit label = 2.5 kbit)
- Maintains a small label-equivalence table for online merging when a U-shape is detected (typically 32 entries)
- For each labeled blob, accumulates feature moments **as pixels arrive** — no separate pass:

```
Per blob k, accumulate over all pixels (i,j) in blob:
  N_k    = Σ Δ[i,j]              // intensity-weighted area
  Sx_k   = Σ i · Δ[i,j]
  Sy_k   = Σ j · Δ[i,j]
  Sxx_k  = Σ i² · Δ[i,j]
  Syy_k  = Σ j² · Δ[i,j]
  Sxy_k  = Σ i·j · Δ[i,j]
  peak_k = max(Δ[i,j])
  bbox_k = (i_min, i_max, j_min, j_max)
```

When a blob is closed (no further mask pixels can join it — known when the row past the blob's last row is processed), it's emitted as a finished descriptor. Blob descriptor record (~24 bytes):

```
struct blob {
  uint16  id;           // assigned by CCL
  int16   x_hat_q8;     // Sx/N in Q8 sub-pixel
  int16   y_hat_q8;
  uint16  N;            // intensity-weighted area
  uint8   peak;
  uint16  bbox_packed;  // i_min<<8 | j_min (or similar)
  uint16  bbox_size;    // width<<8 | height
  uint16  sigma_xx_q4;  // for CEP later
  uint16  sigma_yy_q4;
  int16   sigma_xy_q4;
  uint16  frame_id;
};
```

**Maximum simultaneous blobs:** capped at 16. If more candidates exist, only the 16 largest by `N` are kept (a small priority queue managed online). 16 is generous — in a real scene with the IR bandpass filter rejecting clutter, there will rarely be more than 2–4 active blobs.

Resource cost (from the FPGA-CCL literature for similar resolutions): ~2 k LUTs, ~4 kbit BRAM for label state, no multipliers required for the labeling itself; the moment accumulators need ~6 small multipliers (i·Δ, j·Δ, etc., where i and j are the running raster coordinates).

### 6.5 Stage 3 — Blob track table with motion predictor

Stage 2 produces a fresh list of blob descriptors per frame, with arbitrary IDs. Stage 3 maintains a small persistent **track table** (max 8 tracks) and associates each frame's blobs to existing tracks **using a predictive gate**, not naive nearest-neighbor.

**Why predictive:** at 500°/s body rate, a beacon may move 6–7 pixels per frame and 47 pixels across a correlation window. Static gates would lose the track or mis-associate it with clutter.

**Reacquisition is the dominant operating mode.** Beacons go in and out of view constantly — tracker rotation, target maneuvers, occlusion, glare. The architecture is designed around fast reacquisition, not steady-state tracking.

**Three track states:**

```
INACTIVE   — slot is free, will be assigned to a new blob if any are unassigned
ACTIVE     — track is being measured every frame, emits output (when code-locked)
CANDIDATE  — track was active but lost lock or association. State preserved
             (last position, velocity, intensity ring, partial code history),
             but no output emitted (CEP=sentinel). On re-association, can
             transition back to ACTIVE *without* full code re-acquisition —
             the partial intensity ring is reused.
```

The candidate state is the key to fast reacquisition. When a beacon disappears and reappears in roughly the predicted location within a reasonable time, the FPGA recovers it in a few frames using the preserved code-correlation state, rather than starting over with full 7-chip cold acquisition.

**Per-track state:**

```
struct track {
  enum    state;             // INACTIVE | ACTIVE | CANDIDATE
  int16   x_q8, y_q8;        // last *measured* centroid (sub-pixel)
  int16   vx_q8, vy_q8;      // per-frame velocity estimate
  uint16  sigma_pos_q8;      // 1σ position uncertainty, grows on miss
  uint16  candidate_age_ms;  // time spent in CANDIDATE state (drops at re-lock)
  uint8   miss_count;
  uint8   last_meas_frame_offset;   // frames since last direct measurement
  uint8   intensity_ring[16]; // circular buffer of associated-blob intensities
  uint8   ring_head;
  uint8   ring_valid_mask;    // bit set per slot if frame had a real measurement
  uint16  code_score_A;
  uint16  code_score_B;
  uint8   code_lock;          // 0=unknown, 1=A, 2=B
};
```

**Per-frame association loop:**

```
For each existing track T (ACTIVE or CANDIDATE):
  predicted_x = T.x + T.vx                    // one-frame extrapolation
  predicted_y = T.y + T.vy
  
  // Optional body-rate compensation
  if body_rate_comp_enable and body_rate_age_ms < 50:
    predicted_x += body_rate_to_pixel_dx(ω_yaw)
    predicted_y += body_rate_to_pixel_dy(ω_pitch)
  
  // Search gate radius: 3σ position uncertainty
  // CANDIDATE tracks have larger sigma_pos, so wider search gate naturally
  gate_r² = 9 * (T.sigma_pos² + motion_noise²)
  
  best_blob = nearest blob in B_N to (predicted_x, predicted_y) within gate_r
  
  if best_blob found:
    T.vx = α * (best_blob.x - T.x) + (1-α) * T.vx     // α-β filter
    T.vy = α * (best_blob.y - T.y) + (1-α) * T.vy
    T.x  = best_blob.x
    T.y  = best_blob.y
    T.sigma_pos = update_kalman_post(T.sigma_pos)     // shrinks
    T.intensity_ring[T.ring_head] = best_blob.peak
    T.ring_valid_mask |= (1 << T.ring_head)
    T.last_meas_frame_offset = 0
    T.miss_count = 0
    T.state = ACTIVE                                  // re-promote if was candidate
    T.candidate_age_ms = 0
    blob is consumed; remove from B_N
  else:
    T.x = predicted_x
    T.y = predicted_y
    T.sigma_pos = update_kalman_predict(T.sigma_pos)  // grows
    T.intensity_ring[T.ring_head] = 0
    T.ring_valid_mask &= ~(1 << T.ring_head)
    T.last_meas_frame_offset++
    T.miss_count++
    if T.miss_count > active_to_candidate_threshold:
      T.state = CANDIDATE
    if T.candidate_age_ms > candidate_timeout:
      T.state = INACTIVE                              // give up, free the slot
  
  T.ring_head = (T.ring_head + 1) % 16
  if T.state == CANDIDATE: T.candidate_age_ms += frame_period_ms

For each unconsumed blob in B_N:
  if INACTIVE slot available: spawn new candidate track
                              (large sigma_pos, zero velocity, empty intensity ring)
```

**Tuning parameters (all FPGA registers):**

- `active_to_candidate_threshold` — frames missed before ACTIVE → CANDIDATE (default 3)
- `candidate_timeout` — ms in CANDIDATE before INACTIVE (default 500 ms; longer = better reacquisition memory but more spurious-track noise during clutter)

**Reacquisition latency floor:**

From beacon-visible-again to FPGA-reports-position, the worst case is governed by code length:

- Cold acquisition (no candidate state): full 7-chip × frame-rate window ≈ 35 ms + blob discovery (~5 ms) + admission stabilization (~10 ms) ≈ **50 ms floor**
- Warm reacquisition (candidate state preserved): typically 2–3 frames to re-confirm code lock ≈ **10–15 ms**

At 500°/s body rate and 50 ms cold-acquisition floor, the beacon rotates 25° in the FPGA's blind period. As long as the maneuver doesn't move the beacon off-frame again within those 7 frames, acquisition succeeds. This is another reason to keep code length at 7 chips, not longer.

**Parallel speculative correlation:**

The per-blob correlator cost is tiny (7 add/sub ops per blob per code per frame). The architecture runs correlation on **all 16 blobs per frame**, not just the 8 tracked ones. Unassigned blobs that happen to be the beacon coming back will accumulate correlation history that gets handed to the new track when admitted. This costs ~16 × 2 × 7 = 224 ops/frame, still negligible, and it cuts reacquisition latency by avoiding the "wait for new track to start collecting samples" delay.

**Track scoring for output selection:**

When more than two tracks are code-locked (e.g., reflections, multipath, or a third aircraft), the output picks the two strongest by combined score: `score = code_score - α*sigma_pos - β*miss_count`. Prevents transient clutter from displacing real beacons.

### 6.6 Stage 4 — Code correlator (with miss handling)

For each active track, every frame, the correlator updates running correlations against codes A and B using the track's intensity ring buffer.

```
For each track t in active tracks:
  // Correlation over the ring, weighted by validity
  // Bipolar codes (±1); ring slots with valid_mask=0 contribute nothing.
  
  corr_A[t] = 0
  corr_B[t] = 0
  valid_count = 0
  
  for k in 0 .. L-1:
    slot = (head - k) mod 16
    if ring_valid_mask & (1 << slot):
      corr_A[t] += ring[slot] * code_A[k]    // code is ±1, so just add or sub
      corr_B[t] += ring[slot] * code_B[k]
      valid_count++
  
  // Penalize tracks with too few valid samples
  if valid_count < L * min_valid_fraction:
    code_lock = 0  // no decision
  else:
    // Normalize by valid sample count, not by L
    corr_A_norm = corr_A * L / valid_count
    corr_B_norm = corr_B * L / valid_count
    
    if corr_A_norm > corr_thresh AND corr_A_norm > corr_B_norm:
      code_lock = 1
    elif corr_B_norm > corr_thresh AND corr_B_norm > corr_A_norm:
      code_lock = 2
    else:
      code_lock = 0
```

**Why renormalize:** if the track missed 3 of 7 frames, the raw correlation sum is naturally lower even if the 4 valid samples agreed perfectly with the code. Scaling by L/valid_count restores apples-to-apples comparison against `corr_thresh`. Below `min_valid_fraction` (say 4/7), don't make a decision at all.

**Cost:** 8 tracks × 2 codes × 7 taps × (add or sub) = 112 conditional ops per frame. At 200 fps this is ~22 k ops/sec — single-LUT-cluster work.

This stage holds the architectural payoff intact: **the correlator count is bounded by max-tracked-blobs (8), not by image size.** Body motion does not change this.

### 6.7 Stage 5 — Output formatting

For the two beacons (the tracks with `code_lock == 1` and `code_lock == 2`):

1. **Position source.** Use the track's *latest* `(x, y)` — which is either the most recent measured blob centroid or a one-frame extrapolation if the latest frame was a miss. Never report an average over the correlation window. Stale-but-correlated positions are less useful than fresh, marginally-validated ones.

2. **Lens distortion correction.** Apply a 2D LUT (32×32 grid with bilinear interp) to map raw `(x_hat, y_hat)` in pixel coordinates to corrected angular coordinates. LUT loaded at boot from calibration data.

3. **Normalization to int8 x, y:**
   ```
   x_i8 = clip(round(x_corr · 127 / max_half_width), -127, +127)
   y_i8 = clip(round(y_corr · 127 / max_half_height), -127, +127)
   ```

4. **Composed CEP** — this is the critical change for high dynamics:

   ```
   CEP² = CEP_centroid²              // single-frame blob covariance Σ
                                      // approx 0.5908*(σ_x + σ_y)
        + CEP_track_uncertainty²      // the track's sigma_pos (grew during misses)
        + CEP_motion²                 // expected drift since last measurement:
                                      //   sqrt(vx² + vy²) * last_meas_frame_offset
        + CEP_codelock²               // if code_score is weak: large penalty
                                      //   if strong and unambiguous: ~0
   ```
   
   Then map through the hyperbolic encoder to uint8. The exact functional form is TBD with the host team but the wire contract is fixed: 0=perfect, 255=no detection, monotonic.

   **Why this matters:** during a hard maneuver, the FPGA may report `(x, y)` that came from a 2-frame-old extrapolation while waiting for re-acquisition. The host needs to know — and will see it as a CEP that's 2-3× larger than steady state, even though the *centroid quality* of the last good measurement was excellent. This is exactly the on-the-fly CEP adjustment the host's filter needs to weight measurements correctly.

5. **Last-measurement age** is exposed separately as `a_age_ms` / `b_age_ms` in the register map, so the host can independently judge freshness without unpacking CEP.

6. **Double-buffered register file** — atomic snapshot on I²C transaction start.

If a track has no `code_lock` after the correlation window has elapsed (or if `valid_count < L * min_valid_fraction` for too long), report `cep_u8 = 0xFF` (sentinel: no detection) for that beacon.

### 6.8 Latency budget

| Stage | Time |
|---|---|
| Sensor exposure | 1–2 ms |
| Frame readout @ 200 fps, 320×240 | 5 ms |
| Stage 1 (screen) | streaming, 0 added latency |
| Stage 2 (CCA, moments) | streaming + ~1 line of latency (~30 µs) |
| Stage 3 (track update) | <100 µs (8 tracks × few updates) |
| Stage 4 (correlation) | <50 µs per frame |
| Stages 1–4 per-frame total | within one frame period (5 ms) |
| Code acquisition window | 7 chips × 5 frames/chip × 5 ms/frame = 175 ms |
| **First-lock latency** | **~180 ms** |
| Steady-state output cadence | 5 ms (one per frame) |
| I²C transaction | 0.3–1 ms when polled |

**Acquisition latency note:** the original target was ≤50 ms, which assumed 1 ms-per-chip and a 7-chip code. The actual numbers depend on the chip rate vs. frame rate ratio, which is a design parameter. To hit 50 ms acquisition with 7-chip code at 200 fps, set chip rate = frame rate (1 chip per frame) and acquisition takes 7 frames = 35 ms. If chip rate is slower than frame rate (oversampling, more robust), acquisition takes longer. **This trade-off is worth tuning during bringup**; the architecture supports both.

> **Rate dependency on the autopilot control loop (feature 037).** The "≤ half the autopilot sample interval" acquisition budget is set by the *control loop rate*, which is moving **10 → 20/50 Hz** under `specs/037-20hz-control-loop/`. That tightens the budget: 100 → 50 → 20 ms interval ⇒ **≤ 50 → 25 → 10 ms acquisition**. So **the video frame rate and the gold-code chirp/chip rate must correspond to the chosen 037 rate.**
>
> **Considered standard: 480 fps** (operator, 2026-06-08) — chosen for robust gold-code **detection + sync** (high frame-oversampling per chip → strong SNR and clean chip-edge sync). At 480 fps a 1-chip-per-frame design gives a 480 Hz chip and a 7-bit code period ≈ 14.6 ms (fits the 20 Hz ≤25 ms budget; for the 50 Hz ≤10 ms budget, shorten the code or raise chips/frame). The fps↑ helps detection/sync; the **chip rate + code length are then co-designed against the 037 acquisition budget**. **When 037 Phase A picks the control rate, re-derive chip rate / code length against 480 fps here.** (031 is post-037 in the queue; the rate is decided in 037.)
>
> **±5% clock tolerance (internal RC, unsynced).** The beacon chip rate — and the camera, IMU, servo, TX clocks — each run off their own built-in RC oscillator at ~±5%, and we do **not** sync them. So harmonic rate choices are *nominal-only*; the rates slip ±5% relative to each other. This is exactly the regime the gold-code/correlator is for: the **acquisition correlator must lock to the *actual* chip rate** (search a ±5% chip-rate window, like GPS handles ppm clock error), not assume nominal. **480 fps oversampling supplies the phase-lock margin** to track the slip. Net: don't try to sync clocks — pick harmonic nominals to bound the beat, and let the correlator + oversampling absorb the ±5%.
>
> **The two beacons are MUTUALLY independent ±5% — decode each independently (037 input, operator 2026-06-08).** The ±5% note above is easy to read as "one chip rate ±5%"; it is not. The **two wingtip beacons run off two separate RC oscillators**, so beacon-A and beacon-B chip rates can differ by up to **~10%** and drift independently of each other (and of the camera). The decoder must therefore run a **separate, independently-self-syncing chip-rate/phase loop per beacon** — each correlator searches and locks to *its own* actual chip rate, with its own DPLL and its own locked-rate register. **Do not** assume the two beacons share a clock, a phase, or an integer ratio (this revises the "single LFSR pair" framing of §5 — the *codes* may be generated as a Gold pair, but at the **receiver** the two are decoded as independent timing domains). The harmonic-family discipline protects the intra-airframe clocks (control/camera/IMU); it says **nothing** about the inter-beacon relationship, which is unconstrained. The 480 fps oversampling is what makes per-beacon independent lock affordable (margin per correlator). Extend the chip-rate auto-sync (§"Chip rate auto-sync") and the register map to be **per-beacon** (two locked-rate registers), not a single shared rate.
>
> **Achievable rates are gated by 037's clock/frequency-tolerance survey + the 115 kbaud servo-send limiter.** The chip-rate / code-length / fps numbers here are nominal until 037 finishes its part-tolerance survey (camera 480 fps part, Xiao time-base, Lattice FPGA candidate clock sources) and quantifies the **115 kbaud command-send budget** that caps the control rate (~9–12 ms/send today ≈ half a 50 Hz tick). 031's chip-rate/code-length co-design must consume 037's surveyed tolerances and link-bounded control rate as inputs — see `specs/037-20hz-control-loop/spec.md` "Required research — clock/frequency-tolerance survey."

### 6.9 Estimated FPGA resource use (revised)

For Lattice LIFCL-33 (CrossLink-NX-33), 320×240 input, 16 max blobs, 8 max tracks:

| Resource | Estimated use | Available | Notes |
|---|---|---|---|
| LUTs | ~6 k | 33 k | Stage 2 CCL dominates |
| Registers | ~5 k | 33 k | |
| BRAM | ~150 kbit | 2.5 Mbit | 80 kbit bg + 2.5 kbit row label + ~70 kbit blob/track records & buffers |
| Multipliers (DSP) | ~12 | 64 | Mostly moment accumulators; correlator uses none |
| MIPI D-PHY | 1 | 4 | |

The smaller LIFCL-17 (in 6×6 mm BGA for flight units) is likely sufficient — has ~17 k LUT, ~1 Mbit BRAM, and the design fits with margin.

### 6.10 Open implementation questions

These need decisions during firmware development, not architecture:

1. **Chip-rate vs. frame-rate ratio.** 1 chip/frame is fastest acquisition; N>1 chips/frame oversamples and improves SNR. With high-dynamics motion, prefer oversampling over longer codes. Tunable at runtime via `chip_per_frame` register.

2. **Code length.** 7-bit Gold codes are the canonical start. **Don't go longer than 7 chips for high-dynamics flight** — track loss probability grows faster than the SNR improvement, and reacquisition latency grows linearly with code length.

3. **Track-blob association metric.** Currently specced as Mahalanobis-style gated nearest-neighbor on predicted position with sigma_pos as the gate radius. Adequate for canonical config. Hungarian / global-optimal assignment is overkill.

4. **α-β filter coefficients.** Tune from simulation/flight data. Good starting points: α = 0.6 (position trust), β = 0.3 (velocity smoothing). Faster α tracks better but is noisier under sensor noise.

5. **min_valid_fraction.** 4/7 ≈ 0.57 (or Q4 value 9) is reasonable starting point. Higher = more conservative code-lock. Tune via `min_valid_fraction_q4` register.

6. **ACTIVE → CANDIDATE → INACTIVE thresholds.** `active_to_candidate_threshold` (default 3 missed frames) and `candidate_timeout` (default 500 ms) trade off reacquisition memory vs. spurious-track noise. Tune from simulation against realistic clutter and dropout scenarios. Larger candidate_timeout = better reacquisition for long occlusions but more risk of resurrecting a track on unrelated clutter.

7. **Speculative correlation budget.** Running correlators on all 16 blobs per frame is essentially free (~224 ops/frame), but the bookkeeping (which blob is paired with which speculative correlator across frames) requires care. Worst case is the correlator on a clutter blob, which never code-locks — the architecture rejects it naturally. Simulate against realistic clutter densities.

8. **Sub-pixel centroid bias correction.** Intensity-weighted centroids on a small blob have known sub-pixel bias; literature has standard corrections. Apply during the lens-distortion LUT pass.

9. **Multi-aircraft case.** What if there are 4 beacons (two opposing aircraft)? The architecture supports it — 8 tracks, 4 codes — but the register map needs extra slots. Defer to phase 2.

10. **Saturation handling.** When a beacon is very close, it may saturate sensor pixels. The correlator needs to detect saturation and either reduce exposure or report "too close, low CEP from saturation rather than noise." Handled in firmware AGC loop.

11. **Body-rate comp coordinate transform.** The map from `body_rate_xyz_dps` to per-frame `(Δpx, Δpy)` for a track at image position `(x, y)` involves the camera's intrinsic matrix (focal length, principal point) and the camera-to-body rotation extrinsic. Spec these as boot-loaded constants in flash. Likely the host's calibration tool emits both the distortion LUT and the body-to-camera matrix in one step.

### 6.11 Designing for an evolved-NN consumer (sim-to-real)

This is a sim-to-real experiment. The downstream consumer is an evolved neural network. Today it ingests current+5 prior samples of `(x, y, CEP)` per beacon (~250–500 ms of history at 10–20 Hz polling) and is already extracting useful signal. The NN also has direct access to rate gyros. The network is small but training plus characterization is expensive — variants are evaluated deliberately, not in rapid-fire iteration.

**Methodology:**

1. **Best-guess initial input set** based on engineering judgment (the canonical set: `x_i8`, `y_i8`, `cep_u8`, `age_ms`, `snr`, plus existing gyros). Don't try to optimize the input vector before there's a working pipeline.
2. **Develop the optical pipeline to spec in parallel** with continued NN work. The FPGA simulator emits a rich record of internal observables; the NN polls whichever subset is current.
3. **Simulate against varied scenarios** — full mission envelope including high body rate, frequent beacon loss/reacquire, varying clutter, varying light, sun glare, occlusion.
4. **Compare variants of the NN against simulation.** When adding an input is being considered (e.g., FPGA-supplied velocity, disaggregated CEP components, gain), train a variant alongside the baseline and measure whether acquisition latency and accuracy improve. Treat each new input as needing to earn its place.
5. **Ablation is available as a tool** if a variant looks ambiguous — remove the candidate input and retrain to confirm the delta — but it's not the primary methodology because each retrain is costly.

The FPGA's register map exposes the rich set so input variants can be evaluated without firmware changes. The wire format the NN actually polls is a runtime configuration; firmware doesn't change when the NN's input vector changes.

**On specific input choices:**

The current "rich set" candidates, with comments on each:

| Field | Best-guess assessment | Notes |
|---|---|---|
| `x_i8`, `y_i8` | Definitely use | Already in use |
| `cep_u8` (composed) | Definitely use | Already in use; carries acquisition-state signal |
| `age_ms` | Likely useful | Honest freshness indicator; lets NN discount stale extrapolations |
| `snr` | Likely useful | Per-beacon signal strength, less environment-coupled than gain |
| `vx`, `vy` | Try a variant | Potentially redundant with position history + gyros, but with higher Nyquist and sub-pixel resolution; compare a NN variant with and without |
| `analog_gain`, `digital_gain` | Defer / cautious | Strong environment signal but device-dependent and time-of-day-dependent; may hurt generalization across cameras and conditions |
| `noise_floor` | Maybe useful | More device-independent than gain; reflects sensor health |
| CEP components separately (`cep_centroid`, `cep_track`, `cep_motion`, `cep_codelock`) | Probably not first round | Disaggregating CEP costs input dimensions for marginal benefit; revisit only if composed CEP shows limitations |
| `chip_phase_lock_quality` | Probably not first round | Optical channel health; only useful if NN learns to discount marginal lock |
| `n_blobs`, `n_tracks_locked` | Probably not first round | Scene complexity / clutter signal; speculative |

The CEP-vs-gain question is a good example of why best-guess + comparative simulation is the right approach. Gain is closer to a physical observable but device-dependent. CEP is more device-independent but aggregates information. Rather than guess which is better, the NN team can run a comparison: baseline NN vs. baseline + gain vs. baseline with CEP replaced by disaggregated components, measure acquisition latency and accuracy on the same simulated scenarios, pick the winner.

**Sim-to-real specific considerations:**

The simulator must reproduce, faithfully:

- **Sensor noise characteristics** — shot noise, read noise, dark current, fixed-pattern noise. Synthetic Poisson + Gaussian noise is a good start; calibrate from bench measurements of the actual sensor before flight.
- **AGC behavior** — the sensor's auto-exposure/auto-gain loop has dynamics; the simulator must include them, not assume instantaneous gain adjustment.
- **Optical chain effects** — vignetting from the wide-angle lens, distortion (which the FPGA partially corrects), filter bandwidth (which affects sun rejection).
- **Beacon LED behavior** — non-instantaneous LED turn-on/off (microseconds), beam pattern roll-off with angle, output droop at high pulse current.
- **Body-rate-driven motion blur** — even with global shutter, fast rotation during the exposure window causes blur. At 500°/s and 1 ms exposure, that's 0.5° of blur, ~1.3 px at our angular scale.
- **Multi-beacon spatial proximity** — when the target is far away, the two wingtip beacons might be in the same pixel cluster. The simulator must handle this and the FPGA must too (Stage 2's CCA will see them as one blob; Stage 3/4 must disambiguate by code).

**Simulator fidelity is staged:**

- **Phase 1 — High-level Python model.** Fast iteration, validates pipeline logic and supports early NN evolution rounds. Sufficient for input-vector comparisons.
- **Phase 2 — Cycle-accurate HDL co-simulation.** Validates the FPGA implementation matches the Python model. One-time validation before flight, not used for routine NN training.
- **Phase 3 — Real-camera bench data.** Real sensor frames with synthetic-motion-overlay or real flight footage replayed through the FPGA. Final calibration of noise models and confirmation of sim-to-real transfer.

**Conservatism principles that remain in force regardless of which inputs win:**

- **Permissive about candidates, conservative about claiming lock.** Always report whatever the FPGA has, with honest uncertainty. The NN learns "low SNR + large CEP means hint not fix." It cannot recover from measurements that were filtered out in firmware.
- **Consistency over precision.** As long as the FPGA's output mapping is consistent and monotonic across operating conditions, the NN learns the residuals. Avoid hidden mode changes.
- **Latency consistency over latency minimum.** Deterministic output cadence beats sometimes-fast / sometimes-slow.
- **Never silently extrapolate.** `age_ms` always reflects truth. Extrapolated positions are flagged via `age_ms > 0` and growing CEP.

**Auto clock sync — the FPGA must self-adapt.** The LED MCU and FPGA clocks will drift by tens of ppm (negligible) but the chip rate may be off by *percent* between hardware revisions or firmware variations of the LED driver. The correlator must:

1. **Discover chip rate during initial acquisition** within a tolerance band (±5%), not require it pre-configured. Implementation: try a small grid of chip-rate hypotheses around the nominal value during cold acquisition.
2. **Track chip phase continuously after lock** via a digital phase-locked loop on the correlator output. Once locked, frequency drift is tracked silently.
3. **Expose the locked chip rate** in `locked_chip_rate_hz` register so simulation and the NN (if it cares) can see the operating point.

None of this is visible at the NN's wire interface during steady operation — it shows up as "the FPGA continues to deliver locked detections through long flights without the user setting anything."

---

## 7. I²C register map

Slave address: configurable 0x40–0x4F via three input strap pins (default 0x42 for camera 1).

Frequency: tested up to 1 MHz (Fast-mode-Plus). Default 400 kHz.

The output is intentionally tiny — the autopilot is not a vision system, it's a controller, and it wants a few bytes of guidance state per poll. The core per-beacon record is **3 bytes**: signed-int8 x, signed-int8 y, uint8 CEP.

### 7.1 Wire format for x, y, CEP

**`x_i8`, `y_i8`** — signed int8, range `[-127, +127]`, mapping to normalized image-plane coordinates `[-1.0, +1.0]`. Value `-128` reserved (treat as invalid/sentinel).

**`cep_u8`** — uint8, hyperbolic mapping to a CEP value in roughly `[0, 1.5]`. The exact functional form is TBD with the host team, but the wire contract is:
- `cep_u8 = 0` → CEP = 0 (perfect lock, ideal)
- `cep_u8 = 170` (≈ 0xAA) → CEP = 1.0 (full image extent — borderline useful)
- `cep_u8 = 255` (0xFF) → CEP = 1.5 (sentinel: **no detection**)
- Monotonic, hyperbolic-ish curve in between (gives high resolution near zero, compresses at high CEP)

Sentinel-CEP is the canonical "no detection" indicator. Hosts read three bytes per beacon and check `cep_u8 == 0xFF` to know whether `(x_i8, y_i8)` are meaningful.

### 7.2 Read-only state (host reads these)

| Offset | Type | Name | Description |
|---|---|---|---|
| **Per-frame meta** | | | |
| 0x00 | uint16 LE | `frame_counter` | Increments every camera frame |
| 0x02 | uint32 LE | `timestamp_us` | FPGA local microsecond counter, free-running, wraps every ~71 min |
| **Beacon A (3 bytes)** | | | |
| 0x06 | int8 | `a_x` | Beacon A normalized X |
| 0x07 | int8 | `a_y` | Beacon A normalized Y |
| 0x08 | uint8 | `a_cep` | Beacon A CEP (255 = no detection) |
| **Beacon B (3 bytes)** | | | |
| 0x09 | int8 | `b_x` | Beacon B normalized X |
| 0x0A | int8 | `b_y` | Beacon B normalized Y |
| 0x0B | uint8 | `b_cep` | Beacon B CEP (255 = no detection) |
| **Per-beacon aux (kept compact)** | | | |
| 0x0C | uint8 | `a_age_ms` | Milliseconds since beacon A's position was directly measured (vs. extrapolated). 0=current frame; saturates at 255. |
| 0x0D | uint8 | `b_age_ms` | Same for B |
| 0x0E | uint8 | `a_snr` | Beacon A peak correlation / noise floor (rough SNR proxy) |
| 0x0F | uint8 | `b_snr` | Beacon B peak correlation / noise floor |
| **Per-beacon velocity (experimental — see §6.11)** | | | |
| 0x1C | int8 | `a_vx` | Beacon A velocity X, int8-units per 100 ms |
| 0x1D | int8 | `a_vy` | Beacon A velocity Y |
| 0x1E | int8 | `b_vx` | Beacon B velocity X |
| 0x1F | int8 | `b_vy` | Beacon B velocity Y |
| **Sensor / AGC state** | | | |
| 0x10 | uint16 LE | `exposure_us` | Current sensor exposure in µs |
| 0x12 | uint8 | `analog_gain` | Sensor analog gain register value |
| 0x13 | uint8 | `digital_gain` | Sensor digital gain register value |
| 0x14 | uint8 | `ambient_level` | Mean background brightness (0–255), AGC visibility |
| 0x15 | uint8 | `noise_floor` | Estimated noise σ in 8-bit pixel LSBs |
| **Track-table observability** | | | |
| 0x16 | uint8 | `n_blobs` | Blobs detected in last frame (0–16) |
| 0x17 | uint8 | `n_tracks` | Active tracks (0–8) |
| 0x18 | uint8 | `n_locked` | Tracks currently code-locked (0–8) |
| **Clock sync state** | | | |
| 0x20 | uint16 LE | `locked_chip_rate_hz` | Currently tracked chip rate in Hz (0 = not locked) |
| 0x22 | uint8 | `chip_phase_lock_quality` | DPLL lock quality, 0=unlocked, 255=tight lock |
| 0x23 | uint8 | `reserved` | |
| **Status / health** | | | |
| 0x19 | uint8 | `status_flags` | bit0=sensor_ok, bit1=overexposed, bit2=temp_warn, bit3=mipi_err, bit4=fifo_overflow |
| 0x1A | uint8 | `temp_c` | Estimated FPGA junction temperature (signed, °C) — best-effort |
| 0x1B | uint8 | `fw_version` | Firmware revision |

A burst-read of 0x00–0x1B (28 bytes) yields a fully consistent snapshot due to internal double-buffering. The "core" minimum-poll read is 0x06–0x0B (6 bytes — both beacons' x/y/CEP), or 0x00–0x0B (12 bytes — beacons + frame counter + timestamp), depending on whether the host wants the freshness metadata.

### 7.3 Write-side configuration

| Offset | Type | Name | Description |
|---|---|---|---|
| 0x40 | uint8 | `mode` | bit0: 0=run,1=acq-only; bit1: test-pattern; bit2: sleep; bit3: body_rate_comp_enable |
| 0x41 | uint8 | `code_select` | Which code-pair to look for (0=default A/B) |
| 0x42 | uint8 | `detection_thresh` | Stage-1 mask threshold override (0=auto) |
| 0x43 | uint8 | `corr_thresh` | Stage-4 correlation threshold override (0=auto) |
| 0x44 | uint16 LE | `exposure_override_us` | 0=auto-exposure |
| 0x46 | uint8 | `agc_max_analog_gain` | Cap on analog gain |
| 0x47 | uint8 | `agc_max_digital_gain` | Cap on digital gain |
| 0x48 | uint8 | `chip_per_frame` | Code chips per camera frame (1=fastest acquisition, N>1=oversampling) |
| 0x49 | uint8 | `code_length` | Code length in chips (7, 15, 31) |
| 0x4A | uint8 | `track_timeout` | Frames-without-match before track retire |
| 0x4B | uint8 | `min_valid_fraction_q4` | Min fraction of code-window slots that must be valid for code-lock decision (Q4: 0..16) |
| **Body-rate feed-forward (optional, only used if `body_rate_comp_enable` set)** | | | |
| 0x4C | int16 LE | `body_rate_x_dps` | Tracker pitch rate in degrees/sec, host-supplied |
| 0x4E | int16 LE | `body_rate_y_dps` | Tracker yaw rate in degrees/sec |
| 0x50 | int16 LE | `body_rate_z_dps` | Tracker roll rate in degrees/sec |
| 0x52 | uint16 LE | `body_rate_age_ms` | Read-back: ms since host last wrote rates (FPGA falls back to no comp if >50 ms) |
| **Commands** | | | |
| 0x54 | uint32 LE | `cmd` | Write-only command word; see §7.4 |

### 7.4 Commands

Writing to offset 0x54 triggers an action:
- 0x00000001 — soft reset
- 0x00000002 — re-zero background
- 0x00000003 — recalibrate AGC from scratch
- 0x00000004 — clear track table
- 0x00000005 — reload distortion LUT from flash
- 0xDEADBEEF — enter bootloader (firmware update path)

### 7.5 Total per-poll wire cost

At 20 Hz polling with the core 6-byte read (beacons only):
- Wire bytes: 6 data + 1 address + 2 START/STOP overhead ≈ 9 bytes per poll
- At 400 kHz: ~225 µs per poll, 4.5 ms/sec total bus time → 0.45% bus utilization
- Leaves ~99.5% of the bus for other devices

At 20 Hz with the 28-byte snapshot read: still ~700 µs per poll, 1.4% bus utilization. Either fits comfortably with other I²C peripherals (IMU, baro, etc.) on the same bus.

---

## 8. What's already decided (don't relitigate)

- **Sensor:** VD55G1 (primary), OV9281 (backup)
- **Capture mode:** ~320×240, target 200+ fps. Higher res is a runtime config option, not the default.
- **Wavelength:** 850 nm
- **Filter:** single-band 850 ± 10 nm bandpass, integrated into Commonlands lens housing
- **Shutter:** global
- **Detection scheme:** temporal coding (Gold codes), not wavelength multiplexing
- **DSP architecture:** hierarchical — IIR bg-subtract → 1-bit mask → single-pass CCA → bounded track table → per-track bipolar correlator. **No per-pixel correlator.**
- **Motion handling:** Stage 3 maintains an α-β motion predictor per track; cross-frame data association uses 3σ predictive gating, not nearest-neighbor; correlator handles missed-frame slots via validity mask. Latest-frame measurement is what gets reported, not a window average.
- **Reacquisition is the dominant mode.** Tracks live in three states (ACTIVE, CANDIDATE, INACTIVE). Lost tracks become CANDIDATE for ~500 ms, preserving position/velocity/intensity-ring state for fast re-lock without full cold acquisition. Speculative correlation runs on all 16 blobs per frame, not just confirmed tracks. Warm reacquisition latency: 10–15 ms. Cold reacquisition floor: ~50 ms.
- **Sim-to-real methodology.** Best-guess initial input set, optical pipeline developed to spec in parallel, simulate across varied scenarios, NN variants compared deliberately (not exhaustively ablated). The register map exposes a rich set of internal observables; what the host polls is a runtime configuration. Wire format stabilizes during simulation phase before flight test.
- **Body-rate compensation:** optional, off by default. Host can opt in by writing pitch/yaw/roll rates to config registers; FPGA uses these to predict pure-rotation image motion. Without this, the FPGA still works with wider gates and lower track persistence at high body rates.
- **CEP composition:** sqrt-sum-of-squares of centroid uncertainty + track position uncertainty + extrapolation drift + code-lock confidence. Grows automatically during fast maneuvers.
- **Code length cap:** 7 chips for canonical config. Longer codes degrade under high body rates because tracks may be lost mid-window.
- **Max simultaneous blobs:** 16. Max active tracks: 8. Both bounded — keeps FPGA resource use predictable.
- **FPGA family:** Lattice CrossLink-NX (CrossLink-NX-EVN dev board for prototype, LIFCL-17 6×6 mm BGA target for flight)
- **Host interface:** I²C, register-file model, 10–20 Hz poll, slave addresses 0x40–0x4F strap-selectable
- **Output wire format:** int8 x, int8 y, uint8 CEP per beacon (3 bytes). CEP=255 sentinel for "no detection". Hyperbolic CEP encoding TBD with host team but wire contract is fixed.
- **LED:** Lumileds Luxeon IR Compact 850 nm, 3–4 dies per wingtip on a pyramid
- **Code:** Gold codes, length & chip-rate-vs-frame-rate ratio runtime-configurable
- **Coordinate normalization:** lens distortion corrected in FPGA via LUT, output in normalized image coords
- **LED/camera sync:** asynchronous; matched filter tolerates ±100 ppm clock drift over acquisition window
- **Chip rate auto-sync:** FPGA discovers chip rate during cold acquisition (search ±5% band) and tracks it with a DPLL during steady operation. Locked chip rate is exposed in a read-only register for observability.
- **Consumer design philosophy:** the wire output serves an evolved NN. Consistency and monotonicity matter more than calibration precision. The FPGA reports honest measurements with honest uncertainty — never silently extrapolates without flagging via age_ms, never suppresses marginal detections in firmware. Latency consistency matters more than minimum latency.
- **Velocity output:** exposed as 4 optional bytes in the register map (experimental, log-first-then-feed-NN approach).

---

## 9. Open questions / phase-2 work

These are explicitly out of scope for the canonical first build but worth tracking:

1. **Multi-cam fusion** — three cameras at 0°/+60°/-60° for ~270° coverage. Same firmware, different I²C addresses. Host fuses extrinsics.
2. **Event-camera variant** — IMX636 / Prophesee Gen4 evaluation. Modulated beacons map naturally onto event streams.
3. **Sync between camera and LED clock** — currently asynchronous; revisit if very long codes (>15-bit) are wanted for higher processing gain.
4. **Dual-band variant** — if covertness ever matters, swap to 940 nm or add a second wavelength for redundancy.
5. **In-flight LED brightness control** — close a loop where the tracker tells the target (over the existing RC link) to dim its LEDs at close range. Avoids over-saturation. Requires bidirectional comms to the target.
6. **Body-rate compensation** — even with global shutter, if frames are integrated/averaged the body rate matters. Currently ignored; revisit if integration over multiple frames is added.
7. **Range estimation** — the two beacon centroids' angular separation, combined with known wingspan, gives range. Trivial post-processing in the host but worth noting as a "free" capability.

---

## 10. First-week task list (concrete starts)

**Hardware procurement:**
- [ ] Order Lattice CrossLink-NX-EVN board (~$300, Lattice direct or Mouser/DigiKey)
- [ ] Order Arducam B0162 (OV9281) module (~$60) for initial bringup before VD55G1 sample arrives
- [ ] Sample-request VD55G1 from STMicro (mention drone/RC vision application)
- [ ] Email Commonlands engineering (contact@commonlands.com) with VD55G1 + OV9281 datasheets, request NIR-corrected M12, ~120° H FOV, F/2.0, 850 ± 10 nm integrated filter
- [ ] Order 10× Lumileds Luxeon IR Compact 850 nm (DigiKey)
- [ ] Order ATtiny1616 dev boards for LED MCU prototyping

**Simulation / algorithm validation (critical path — start here):**
- [ ] Build a Python testbench that synthesizes 320×240 frames at 200 fps with two modulated point sources, configurable noise, sun glare, and clutter
- [ ] Add motion to the synthesis: target lateral velocity (10–30 m/s), tracker body rate up to 500°/s. Beacon image-plane positions evolve frame-to-frame accordingly. Include frequent beacon loss/reacquire events (off-frame transitions, occlusion, glare).
- [ ] Calibrate the sensor noise model from bench measurements (Poisson shot + Gaussian read + dark current + fixed-pattern) before final NN training rounds
- [ ] Include sensor AGC dynamics, lens vignetting, LED beam-pattern roll-off, and motion blur at the exposure window
- [ ] Implement the full pipeline (§6.3–6.7) in numpy/scipy: IIR bg, threshold mask, single-pass CCA, track table with ACTIVE/CANDIDATE/INACTIVE state machine + α-β predictor, bipolar correlator with validity-masked normalization, speculative correlation across all blobs, distortion LUT, output formatting with composed CEP
- [ ] Emit the rich record per beacon (every internal observable) so input variants can be evaluated without firmware changes
- [ ] Run sweeps: SNR vs. acquisition time, code-length vs. false-lock rate, blob-cap=16 saturation under heavy clutter, body-rate vs. track-persistence (with and without body-rate compensation), warm vs. cold reacquisition latency across body rate / clutter / occlusion-duration
- [ ] Develop the optical pipeline to spec in parallel with continued NN evolution
- [ ] Compare NN variants against simulation (baseline vs. variants with added inputs); measure acquisition latency and accuracy
- [ ] Validate the hyperbolic CEP encoding curve produces useful host-side resolution across the full dynamic range
- [ ] Generate test vectors (input frames + expected outputs) for HDL verification, including high-body-rate scenarios, beacon loss/reacquire, and dense-clutter cases
- [ ] Stabilize the wire format before HDL implementation — once flight testing starts, changing it costs an NN retrain

**Firmware/HDL:**
- [ ] Set up Lattice Radiant project for CrossLink-NX-EVN
- [ ] Clone `circuitvalley/USB3_MIPI_CSI2_RX_V2_Crosslink_NX` and adapt I²C init for OV9281 at 320×240
- [ ] Implement Stage 1 (IIR bg + mask) in Verilog; verify against Python golden vectors
- [ ] Implement Stage 2 (single-pass CCA with on-the-fly moments) in Verilog. Reference: Bailey-Johnston papers — there are open-source implementations to study
- [ ] Implement Stage 3–5 (track table, correlator, output formatter) — these are not streaming-pixel pipelines, they're small state machines, easier than Stage 2
- [ ] Sketch I²C slave register file in Verilog with the §7.2 layout exactly
- [ ] Implement Gold-code generator (LFSR + product) on ATtiny1616, drive a single LED, verify on a scope

**Software (host side):**
- [ ] Define C struct matching §7.2 layout exactly (`__attribute__((packed))`, little-endian for multi-byte fields)
- [ ] Write a small command-line tool that polls the camera over I²C (Linux i2c-dev or autopilot HAL) and prints `{x_i8, y_i8, cep_u8}` for both beacons, plus optionally the aux fields
- [ ] Implement the host-side hyperbolic CEP decode (inverse of the FPGA's encode) so logs/UIs can show real-units CEP
- [ ] Add a "calibration" mode that captures the centroid stream from a stationary beacon at known angles and emits a lens distortion LUT for the FPGA to ingest

---

## 11. Reference links

- ST VD55G1: https://www.st.com/en/imaging-and-photonics-solutions/vd55g1.html
- VD55G1 datasheet: https://www.st.com/resource/en/datasheet/vd55g1.pdf
- OmniVision OV9281: https://www.ovt.com/products/ov9281/
- Lattice CrossLink-NX: https://www.latticesemi.com/Products/FPGAandCPLD/CrossLink-NX
- CrossLink-NX-EVN board: https://www.latticesemi.com/en/Products/DevelopmentBoardsAndKits/CrossLink-NXEvaluationBoard
- Antmicro OV9281 dual camera open-hardware: https://openhardware.antmicro.com/boards/ov9281-dual-camera-board/
- Circuitvalley MIPI CSI-2 RX Verilog (CrossLink-NX): https://github.com/circuitvalley/USB3_MIPI_CSI2_RX_V2_Crosslink_NX
- Commonlands M12 lenses: https://commonlands.com/collections/m12-lenses
- Lumileds IR LEDs: https://lumileds.com/products/infrared-emitters/
- Bailey-Johnston single-pass CCL (open-access survey): https://www.ncbi.nlm.nih.gov/pmc/articles/PMC8320945/
- FPGA single-pass blob analysis with RLE: https://www.researchgate.net/publication/228912595

---

## 12. Glossary

- **AEC/AGC** — Auto-exposure control / auto-gain control. Built into VD55G1 and OV9281.
- **Ablation** — Systematically removing input dimensions and retraining to determine which inputs are essential vs redundant. Available as a tool for ambiguous cases; not the primary methodology for input selection in this project (best-guess + comparative simulation is).
- **α-β filter** — Two-state recursive estimator for position and velocity. Simpler than full Kalman, well-tuned for constant-velocity targets, fits FPGA in a few dozen LUTs.
- **Bailey-Johnston single-pass CCL** — A streaming connected-component-labeling architecture that extracts blob features (area, moments, bounding box) on-the-fly during the first raster pass, eliminating the second pass and the labeled-image buffer. Standard reference for FPGA blob analysis.
- **Candidate track** — A track that has lost lock but whose state (position, velocity, intensity ring, partial code correlation) is preserved for fast reacquisition. Distinct from ACTIVE (emits output) and INACTIVE (free slot).
- **CCA / CCL** — Connected-component analysis / connected-component labeling. The process of finding spatially-contiguous groups of "on" pixels in a binary image and assigning each a unique label.
- **CEP** — Circular error probable. The radius within which 50% of measurements fall. Used here as a confidence radius around the centroid, encoded as uint8 with hyperbolic mapping.
- **CRA** — Chief ray angle. Lens characteristic; must match sensor's microlens design.
- **CSI-2** — Camera Serial Interface, version 2. MIPI's standard sensor-to-host protocol.
- **DVP** — Digital Video Port. Older parallel video interface (HSYNC/VSYNC/PCLK/D[7:0]).
- **D-PHY** — MIPI's physical layer for CSI-2.
- **Gold code** — A pseudorandom binary sequence with low cross-correlation; used in GPS and CDMA. Generated from two LFSRs.
- **IIR** — Infinite impulse response. Used here for the slow background tracker.
- **LFSR** — Linear feedback shift register. Generates pseudorandom sequences.
- **NIR** — Near-infrared, ~750–1400 nm.
- **Q15 / Q8** — Fixed-point formats. Q15: 15 fractional bits in a 16-bit signed word, range [-1, +1). Q8: 8 fractional bits, used internally in the FPGA for sub-pixel centroid arithmetic.
- **QE** — Quantum efficiency. Photons-to-electrons conversion ratio of an image sensor.

---

*End of handoff document.*
