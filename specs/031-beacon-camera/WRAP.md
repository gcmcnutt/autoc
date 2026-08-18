# 031 — wrap plan (drafted 2026-08-17)

## Where 031 landed (the deliverables, as built)
| Area | Outcome | Evidence |
|---|---|---|
| **Emitter** | LM3410X boost + ATtiny416 (bench XNANO) Gold-code emitter, N=31, 200 Hz nominal / 115 Hz bench boot; UVLO 3.48 V + WDT + BOD; 4.7 µH flight inductor validated on battery; 1″ FR4 tile passes thermal at 306 mA (5–8 °C); regression suite 19/19 (17/19 native) | tasks A1–A2, A7-1, `firmware/beacon-pod`, bench-journal |
| **1-pixel receiver** | Option-C AC-coupled front end **field-proven in direct sunlight** (15–20 ft bare, more with the 16 mm); s7 correlator + gear-shifted AGC + energy gate | daylight doc, bench-journal field tests #2–#4, `experiments/s7.v` |
| **CDMA/correlator** | Two-code separation, DPLL, dropout ladders, N-length study — done on the StepFPGA; the same math ported to Python on the Pi and **locking on the camera** | A4c/A4d, `beacon_track.py` |
| **Camera toolchain** | Pi 3A+ + OV9281 (mainline driver, Trixie): 250–280 fps sustained; custom sensor modes work (per-frame ceiling found); lens measured (f·θ, 95×61); code read optically; real-time tracker + M2-grid display; port/starboard convention | `firmware/beacon-receiver/`, camera-era-knobs, handoff-041 |

## What must finish IN 031 before wrap
1. **Emitter schematic: add the OVP clamp** (D3 15 V zener cathode→V_OUT/anode→FB, R4 1 k FB→sense) to
   `cad/beacon-eval/beacon-eval.kicad_sch` — the BOM (EV-A13/A14) and bench (A1-ovp fitted) have it; the
   sheet doesn't. Regenerate `beacon-eval-schematic.pdf`. **(operator flagged)**
2. **A7 cube build (two cubes, codes A and B)** — the flight emitter story is 031's central deliverable:
   A7-2 driver board, A7-3 OVP fitted, A7-4 bring-up @306 mA, A7-5/6 tiles+assembly (C-11 stars — the
   Luxeonstar order if going that route), A7-7 runtime, A7-8 regression re-baseline. **Blocked on the bare
   MCU buy**: order-03 **C-26 = ATtiny412 SOIC-8 ×10** (hand-solderable with the on-hand SOIC-8→DIP
   adapters; serial-UPDI via TTL-232R + 4.7 k). Firmware: 416→412 pin remap; `-DBOOT_HALF_RATE` per cube.
   *(One 200 Hz question to decide at build time: flight boot = 200 Hz nominal or the Pi-3-era 115.)*
3. **Field test the cube(s) with the 1-pixel receiver at range** (A7-9 / the 100 m attempt) — closes the
   emitter+optics story with a number.
4. **Docs**: bench-journal "current state" refreshed to today; tasks.md reconciled (below); this WRAP.

## What LEAVES 031 (goes to 042 or the backlog)
- **042 — camera receiver hardening** (new feature): native C/NEON tracker-bank on the Pi (sliding-window,
  motion-compensated trackers, 20 Hz fixes, threaded acquire) — A8-6's "next"; **faster host** (Pi 4/5 for
  ≥453 fps + dual CSI; the FPGA patterns as the alternative); lens/filter (850 disc mounting, filtered wide
  lens, sky-background measurement — empirical #1 STILL OPEN); exposure/gain AGC as a controller; the
  arrival-checklist leftovers (A8-4 sky, A8-5 static range on the camera, A8-7 defocus). Camera work in 031
  reached "toolchain verified + prototype locks"; 042 makes it a receiver.
- **Deferred/superseded task families** (mark in tasks.md, no work): T012–T014 FreeCAD/enclosure (cube is
  hand-built FR4); T044–T052 US3 UVC viewer (superseded by the Pi/OV9281 path); T053–T065 US4 Lattice
  recorder (FPGA route parked; recorder is not an analysis need); A4a/A4b/A4c/A4d-x remaining sub-items
  (correlator research complete for 031's purposes); A5/A6 Stage-0/1 (done in substance via s6/s7 + camera
  CDMA); A7a/A7b (subsumed by A7-9); A2-osc/A2-uvlo-2 (RC committed via the 412 decision; micro-dropout
  hunt → backlog); A3-b/A3-c soldered Option-C build (**do only if the 100 m test wants it** — the breadboard
  proved the design; else → 042 as "field receiver head").
- **Backlog notes**: BOOT_HALF_RATE flight default decision; 041 L/R by code identity; sun-transit
  max-energy gate (journal 2a) — camera-era items already filed there.

## Wrap criteria (checklist)
- [ ] Emitter schematic has D3/R4 + regenerated PDF
- [ ] Two cubes built (A/B), regression re-baselined, one field range number recorded
- [ ] tasks.md reconciled (superseded / migrated / done), bench-journal current-state refreshed
- [ ] 042 spec seeded (`specs/042-camera-receiver/`) from the "leaves 031" list + camera-era-knobs
- [ ] Final merge into 041/main; branch closes
