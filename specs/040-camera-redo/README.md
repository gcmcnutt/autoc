# 040 — Camera Redo (PARKED placeholder)

**Status**: 🅿️ **PARKED — not yet specified.** This directory is a placeholder so the many
cross-references to "feature 040" resolve to a real home. No `spec.md`/`plan.md`/`tasks.md` exists
yet; run `/speckit.specify` here when 040 is unparked.

**Created**: 2026-06-22 (paperwork stub alongside the 031 re-scope to the 1-bit acquisition phase).

## What 040 is

040 is the **camera-redo phase**: the full beacon→camera→FPGA→recording optical pipeline that was
originally drafted under 031, then split out when 031 was re-scoped (2026-06-20) to the **1-bit
single-IR-sensor acquisition-research phase**. The relationship:

- **Emitters are shared with 031** — same coded-IR beacon pods (5-LED half-cube, 1S-battery, Gold-code
  modulated). 040 does not re-do the emitter; it consumes it.
- **The single-sensor analog front end (031) is replaced by a camera + bigger FPGA** — a global-shutter
  mono sensor (320×240 @ 480 fps), ~120° NIR-bandpass M12 lens, MIPI ingest, FPGA five-stage detection
  pipeline, raw-frame recording, and the eventual real-time `(x, y, CEP)` wire contract that 030's
  evolved controller consumes.
- 031 proves the **temporal/code channel** (acquire + decode + two-code CDMA separation) cheaply on one
  photodiode; 040 adds the **spatial channel** (localization / bearing) that a single detector cannot
  provide. See [`../031-beacon-camera/acquisition-research-plan.md` §2](../031-beacon-camera/acquisition-research-plan.md).

## Reference material (lives in 031, is 040's source-of-design)

The original camera-phase design docs were drafted in `031-beacon-camera/` and remain there as 040's
reference. **Do not build the camera chain from them under 031** — they are 040's to inherit:

- [`../031-beacon-camera/spec.md`](../031-beacon-camera/spec.md) — full camera-pipeline spec (camera + lens + filter + recorder + paired-craft flight).
- [`../031-beacon-camera/plan.md`](../031-beacon-camera/plan.md), [`data-model.md`](../031-beacon-camera/data-model.md), [`tasks.md`](../031-beacon-camera/tasks.md) — camera-phase planning artifacts.
- [`../031-beacon-camera/camera_considerations.md`](../031-beacon-camera/camera_considerations.md) — sensor selection + link budget + Gold-code acquisition/tracking math.
- [`../031-beacon-camera/verified-bom.md`](../031-beacon-camera/verified-bom.md) — full-camera BOM (bannered stale under 031; 040's starting BOM).
- [`../031-beacon-camera/contracts/`](../031-beacon-camera/contracts/), [`recorder-status-codes.md`](../031-beacon-camera/recorder-status-codes.md), [`schematic.md`](../031-beacon-camera/schematic.md), [`quickstart.md`](../031-beacon-camera/quickstart.md).

## Baseline correction to apply on unpark

The 031 reference docs carry a **2026-06-22 rate-baseline correction** to the current program baseline:
**20 Hz control loop (037), 480 fps camera, 200 Hz Gold-code chip rate, 75 ms (15-chip) code period,
15-bit codes** (200 Hz × 2.4 frames/chip = 480 fps; 15 chips @ 200 Hz = 75 ms = 1.5 ticks @ 20 Hz). The
correction was applied to `031-beacon-camera/spec.md`'s operative text, but the **parked camera sub-docs
above (`plan.md`, `data-model.md`, `tasks.md`, `quickstart.md`, `schematic.md`, `verified-bom.md`,
`camera_considerations.md`, `recorder-status-codes.md`) still carry the legacy 240 fps / 100 Hz / 150 ms
numbers.** When 040 is specified, sweep these to the corrected baseline first. 240 fps / 100 Hz is the
legacy low-rate point, not the acquisition baseline.

## Toolchain (deferred)

Per [`../../docs/toolchains.md`](../../docs/toolchains.md): 040 targets **Lattice Radiant + Propel
(CrossLink-NX / LIFCL-40)** on the Windows host — **NOT installed**; install only when 040 restarts.
(Distinct from 031's Lattice **Diamond** / MachXO2 STEP-MXO2 acquisition-correlator flow — don't conflate
the two.)

## Sibling tracks (for orientation)

- **031** — active: 1-bit single-sensor beacon-acquisition research, **bench + ground only** (proves the
  code + FPGA architecture; gate = Stage 2 ground-field).
- **"pre-camera flight" follow-on** (unnumbered, end-of-roadmap) — flying the 1-bit receiver (Stage 3):
  small-form-factor receiver + onboard record-to-SD. Sits between 031 and 040; sequencing condition-
  dependent (e.g. emitters-flying-against-a-ground-receiver may come first). Number it when specced.
- **038** — active: M2 RNN/tracker robustness (hull-crash penalty + camera variations); consumes 031's
  eventual field-calibrated CEP model but is otherwise orthogonal to 040.
- **040** — this: camera pipeline + localization, parked until 031 field-proves the acquisition channel.
</content>
</invoke>
