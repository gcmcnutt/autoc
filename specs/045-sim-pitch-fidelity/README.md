# 045 — sim pitch fidelity (seed)

**Created 2026-10-13 at the close of 043.** Not started. Precedes **044 M2 tracking** because M2 inherits this plant.

## Why

Five ACRO flights on three genomes showed the same 2 Hz, ±200 °/s, ±7 g pitch cycle, and the same genomes do not cycle in
the sim ([043 outcome § 2](../043-acro-dual-loop/outcome.md)). The closed inner loop's frequency response agrees between sim
and aircraft to 1 Hz in gain and 2.5 Hz in phase; the aircraft then resonates at 2.5 Hz (|H| 1.6) where the sim keeps rising
to 6 Hz. The model file's short period is 6.9 Hz / ζ 0.81; the aircraft's is ~3 Hz / ζ 0.2–0.3; `I_yy = 0.0013` is the
first-order cause. Until the sim exhibits the cycle, no objective can price it — three bakes of objective work proved that.

Proposal of record: [sim-pitch-fidelity-proposal.md](../043-acro-dual-loop/sim-pitch-fidelity-proposal.md).
Evidence: [tempered-control-first-principles.md § 11](../043-acro-dual-loop/tempered-control-first-principles.md),
[flight-analysis-20261011.md](../043-acro-dual-loop/flight-analysis-20261011.md) / [b](../043-acro-dual-loop/flight-analysis-20261011b.md).
Tools: [frf_pitch_compare.py](../043-acro-dual-loop/frf_pitch_compare.py), [sim_pitch_spectrum.py](../043-acro-dual-loop/sim_pitch_spectrum.py).

## Work carried from 043 (tasks.md Phase 10c/10d numbering kept for traceability)

1. **T116** — measure `I_yy` on article 1 and article 2 (bifilar pendulum). A measurement, not a fit; do it first.
2. **T095** — MANUAL held pitch doublets on both articles, 250–500 Hz blackbox.
3. **T096** — fit `Cm_a` / `Cm_q` (and `CL_a` only if ζ demands it), keep `Cm_de/Cm_a`, re-centre `CraftCmQSigma`.
4. **T093** — per-cell step-response tool; validation ladder: step test → closed-loop FRF matches the flight column →
   **gen 488 shows the 2 Hz cycle in the corrected sim**.
5. **T112** — apply the NN command at the 5 ms physics sub-step (today: 50 ms frame top); latency sweep 10/50/100/150 ms with
   the three-way decision tree.
6. **T113** — open-loop replay of the flight command stream through the sim plant.
7. **T114** — article 2: rate-parity gate, gen 488 flown, FR-056 review at n = 2 (T047a).
8. **T092** — trainability gate on the new plant before any bake. ⚠️ Every pinned eval number moves (determinism-affecting);
   record new baselines, as 037 T017 did.
9. **T046** servo bench step; **T047** static-margin / pitch-damping constant review (subsumed by T096).

## Gates

- Clean `scripts/rebuild-perf.sh` (operator) after the crrcsim changes; new eval baselines recorded.
- No new objective work until the corrected sim reproduces the cycle with gen 488 — that is the exit criterion for this feature.
- The flight baseline of record is 043-t5 gen 488 (`1731d937e3af9c1c`); it stays in the aircraft until something beats it in the air.
