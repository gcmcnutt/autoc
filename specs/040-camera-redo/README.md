# 040 — Camera Redo

**Status**: 🟢 **ACTIVE — specified 2026-07-28.** See [`spec.md`](spec.md).

**Created**: 2026-06-22 as a paperwork stub alongside the 031 re-scope. **Unparked 2026-07-28.**

## Two eras live in this directory — do not confuse them

| | what it is | where |
|---|---|---|
| **040 as specified (ACTIVE)** | **Perception-fidelity refinement for M2** — make the *simulated* camera resemble buildable hardware, then retrain M2 against it. Sim-side only. | [`spec.md`](spec.md), [`input-data-checklist.md`](input-data-checklist.md), [`checklists/`](checklists/) |
| **Camera hardware phase (PARKED)** | The original beacon→camera→FPGA→SD **recording** chain: gateware, clip format, Python loader, bench build guides. **No part of it is in 040 as specified.** | [`camera-hardware-phase/`](camera-hardware-phase/) |

The hardware-phase material was relocated into its own subdirectory on 2026-07-28 for two reasons: four
of its filenames (`plan.md`, `data-model.md`, `quickstart.md`, `contracts/`) collide with what
`/speckit.plan` generates and would have been overwritten; and keeping the two eras adjacent was already
causing confusion across sessions. It is **not** obsolete — several of its documents are actively
referenced by the running 031 emitter build.

## What 040 is (as specified)

A **research + design-refinement** feature, not a build feature. Outcome: a refined camera scheme grounded
in what hardware can actually deliver, a simulator that reflects it, and a more robust M2 — while 031
proves field hardware in parallel and **M3 (optical + time-of-flight)** remains the real destination.

Seven deliverables, priority-ordered in the spec: airframe-fidelity verdict (gating) · honest camera
geometry · obstruction as design validation · signal-quality CEP · retrained M2 · camera variations ·
optics record. Governing principle is **plumbing first, calibration later** — right structure and right
knobs now, numeric calibration when real-to-simulation measurement exists.

**Explicitly out of scope**: photon budget at frame rate (needs article 1 + raw capture), second camera,
dual-FOV optics, the detection-pipeline hardware, multipath/glint/sun, engine-speed propeller modelling.
Each carries a recorded trigger in the spec.

## Still current at this level

- [`camera_considerations.md`](camera_considerations.md) — sensor selection + link budget + code
  acquisition maths. **Reference material the active spec cites** (notably that its 100 m link budget
  assumes an 8 mm aperture / 3.4° camera, which is what makes the wide-field shortfall visible). Carries a
  stale-rate banner: it predates the 20 Hz / 480 fps / 200 Hz / 75 ms baseline.
- [`input-data-checklist.md`](input-data-checklist.md) — **the input of record for the spec**: every
  measured hardware value, its source, and the open items. Consume this rather than re-deriving.

## Parked in `camera-hardware-phase/`

`plan.md` · `data-model.md` · `quickstart.md` · `recorder-status-codes.md` · `contracts/` ·
`beacon-viewer/` · `beacon-loader/`

**Unpark trigger**: a camera bench exists (article 1 + raw uncompressed capture). Indexed from
[`../BACKLOG.md`](../BACKLOG.md). ⚠️ These predate the 031 1-bit phase and the current rate baseline —
re-validate before building from them.

## Sibling tracks

- **031** — active: 1-bit single-sensor beacon acquisition, bench + field. Produces the calibration 040
  consumes. Several of its live build documents link into `camera-hardware-phase/`.
- **M3** — optical + time-of-flight. Supplies range directly, which is why 040 corrects rather than
  extends separation-derived ranging.
