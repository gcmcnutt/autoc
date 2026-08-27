# Indoor range ladder — 2.5 to 16.7 m, and what it says for the field test (2026-08-26)

First range measurement on the **flight optical configuration**: 1.26 mm lens with the 850 nm bandpass,
five co-aligned L1IZ-0850 emitters at 50 mA transmitting the Gold code, Pi 5 on a tripod looking down a
hallway. One `range_station.py` invocation per station — photometry and tracking together, appended to
`beaconpi5:/data/range.csv`.

| range | note | peak | area | sum | bg | **MEASURED** | present | q50 | cep50 | first lock |
|---|---|---|---|---|---|---|---|---|---|---|
| 2.54 m | 8′4″ | 255 sat | 19 | — | 0.0 | 91% | 94% | 1.00 | 0.29 | 0.60 s |
| 4.55 m | 14′11″ | 190 | 14 | 1681 | 2.0 | **97%** | 97% | 0.99 | 0.21 | 0.45 s |
| 4.83 m | 15′10″ | 230 | 18 | 2096 | 0.0 | **13%** | 66% | 0.46 | 0.48 | 0.30 s |
| 8.15 m | 26′9″ | 46 | 16 | 318 | 0.0 | 71% | 94% | 1.00 | 0.21 | 0.65 s |
| 11.58 m | 38′ | 130 | 8 | 477 | 0.0 | 80% | 91% | 1.00 | 0.18 | 0.75 s |
| 16.69 m | 54′9″ | 96 | 8 | 469 | 2.0 | **96%** | 97% | 1.00 | 0.27 | 0.55 s |

## The headline: range was never the limit — the hallway was

**Decode sits at 71–97% across the whole ladder with q pinned at 1.00**, and there is no downward trend
with distance. The best station is the *longest* one. First lock stays under 0.75 s everywhere and beats
the §3 400 ms bar at three of six.

So **16.7 m is the end of the corridor, not the end of the link.** With five emitters and the bandpass
this configuration has margin far past anything indoors can test, and the far-field question the
[AC-coupling note](../ac-coupling-and-exposure.md) raises — trouble at 60–100 m — is untouched by this
data. It needs outdoor range.

## The 4.83 m null is multipath, and it is the most useful thing here

One station broke the pattern: **13% decode at peak 230**, the second-brightest reading on the ladder.
Re-shot at 4.55 m — 30 cm away, comparable brightness — it returned **97%**. The anomaly does not
reproduce, so it is positional, not range-driven.

Operator identified the mechanism live: *"we can get specular highlights or additional bits from house
lights."* Confirmed by measurement — `beacon_spot.py` found **five modulating sources** spread across
±104 M2 px, including a pair 6 px apart (the classic mirror-pair signature):

```
amp 114  area 665  M2 (+45.2, -25.6)
amp  88  area   7  M2 (+65.1, -11.8)
amp  85  area  76  M2 (-104.3, -25.8)   <- pair
amp  76  area  73  M2 ( -98.5, -24.3)   <- pair
amp  69  area   6  M2 (+74.3, -10.5)
```

**A specular reflection is a fundamentally different adversary from everything else chased this session.**
The window and the overhead lamps were *uncoded* clutter and the bandpass annihilated them (212×
background reduction, [see the base check](stage1-pendulum.md)). A reflection carries **the beacon's own
Gold code, in band, at full modulation depth** — so no optical filter removes it, and no energy or
modulation-depth gate rejects it, because by every amplitude statistic it *is* a beacon. That is exactly
what spec §9's mirror-pair rule exists for, and it is the one thing T085's `min_mod_depth` could never
have caught.

The failure mode is specific: a reflection landing *inside the correlation aperture* sums two copies of
the code at **different phases**, which partially cancel. Strong signal, collapsed q — precisely the
230-peak / 0.46-q reading. **84 points of decode rate lost at a spot 30 cm from a good one.**

## The photometric ladder is NOT usable, and the reason matters

Peak reads 46 → 130 → 96 as range *increases*. That is not noise and not 1/r²: with multipath present,
the brightest blob in frame is not reliably the direct beacon, so "peak" measures whichever specular
return happens to dominate at that station. **A hallway with parallel hard walls cannot produce a clean
photometric range curve.** The *tracking* ladder is unaffected and is what this exercise actually
delivers.

Two tool bugs found and fixed on the way, both of the silent kind:
- A global `0.15 × peak` threshold selected **all 256000 pixels** at 16.69 m once house lights raised the
  floor above the max-projection noise — reporting the entire frame as the emitter. Now flood-fills from
  the peak pixel instead, which cannot fail that way.
- On the optical bench there was **no operating point where the drive current was measurable AND the
  image unsaturated**: 50 mA at 0.28 m stayed clipped at 53 µs even with current cut 60×, and below
  ~1 mA the supply lost current regulation entirely (meter 0.0000 A, Vf pinned at the LED knee, and the
  measured flux went *non-monotonic*). Photometric calibration needs range, not attenuation.

---

# Notes for the field test

**What indoor testing has established, and can stop re-testing:**

- The optical chain works end to end at 16.7 m with large margin. Decode 71–97%, q 1.00, cep 0.18–0.29
  M2 px.
- The bandpass solves *uncoded* clutter completely. Window open and lamps on gives frame mean 0.02 ADU
  and exactly one modulating source.
- Acquisition is fast and consistent: first lock 0.30–0.75 s across every station.

**What only the field can answer:**

1. **The far-field cells.** 60–100 m is where the SNR arithmetic puts trouble, and the 1.26 mm lens costs
   **70% of the photons** relative to the 2.31 mm lens all earlier tracking data was taken on. Predicted
   coherent SNR at 100 m is **0.76 photon-limited, 0.42 read-noise-limited** — i.e. below unity. Expect
   60 m to be the practical ceiling on the current exposure policy, and treat a 100 m result as a
   genuine surprise either way.
2. **Whether multipath is an indoor artefact.** Open ground has no parallel walls, and the sky has no
   specular returns at all — so the 4.83 m null may simply not exist outdoors. **This is worth checking
   deliberately rather than assuming**, because if it persists near the ground (wet tarmac, vehicle
   glass, water) it is a flight-relevant failure with no software fix in place.
3. **Sky background.** Every indoor scene here read 0.0–2.0 ADU. Sky is the case where the background is
   *large and structureless*, which is a different regime from anything measured, and the case where
   ego-motion registration (T071) has no texture to work with.

**Procedure notes carried forward:**

- **Photometry needs a clean site.** Pick a station with no reflective surface within the beam, or the
  1/r² anchor will be contaminated the way it was here. One good unsaturated photometric point at known
  range and current is worth more than a dozen contaminated ones.
- **Re-shoot any station that looks anomalous before believing it.** The 4.83 m null would have entered
  the record as a range effect if it had not been repeated 30 cm away.
- **Log which stations were dark vs lit.** Background rose 0.0 → 2.0 ADU partway through this ladder as
  house lights came on; that is a second variable moving alongside range and it was nearly missed.
- **`range_station.py` is the instrument** — one command per station, photometry and tracking, appended
  to CSV. It flags saturated photometry explicitly rather than letting a clipped sum masquerade as a
  measurement.
- **The angular scale for this lens is 0.548 °/M2 px**, not the 0.304 of every earlier clip. Pass
  `--deg-per-px 0.548` to the pendulum tools or every rate comes out 81% low, silently.
