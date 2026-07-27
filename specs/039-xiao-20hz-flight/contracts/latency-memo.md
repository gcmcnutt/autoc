# Contract: Latency Decision Memo (FR-004 / FR-005 / FR-006)

**Scope**: US2's deliverable — the numbers + a recommendation for the operator review. No pre-set
retrain trigger (clarification 2026-07-10): the memo recommends; the operator decides.

## The memo MUST contain

1. **Measured, 10 Hz era** (already gathered — research.md R1): fetch/eval/send mean/95%/max from
   `flight-20260503` + `flight-20260517`, tick cadence spread, servo response numbers.
2. **Measured, 20 Hz bench** (fresh, on the regenerated firmware): the same components re-measured
   via the per-span MSP pipeline stats + DWT eval cycles, at BOTH the current 115200 baud and one
   raised-baud candidate (e.g. 460800) — the baud decision rides on this. MUST include the tail
   (max over several 3–4 min spans), not just means — the 50 ms tick is judged against the tail.
3. **Modeled**: crrcsim `COMPUTE_LATENCY_MSEC` (30 ms, command-delay mechanism) + servo v2 (PWM
   latch 0–20 ms phase, slew ~24.2 u/s) — values AND mechanism, so the comparison is
   apples-to-apples (what is delayed relative to what).
4. **Gap table + recommendation**: side-by-side; a recommendation among (a) model stands,
   (b) amend `COMPUTE_LATENCY` (+ value) and retrain M1, (c) amend without retrain (flight rides
   the existing elite; retrain queued). Rationale must address: mean gap, tail-vs-tick risk, and
   whether the 038 craft-variation envelope (servo slew/thrust-lag sigmas) already covers the delta.
5. **Local-IMU verdict (FR-006)**: explicit "stays deferred / needed", justified by the measured
   state-fetch freshness at 20 Hz.

## Acceptance

- Memo exists in the feature dir (`latency-memo.md` results section or outcome.md), operator
  decision recorded with date.
- If (b): the retrain launches via `scripts/train.sh` after a clean `rebuild-perf.sh` gate
  (Constitution IX), its elite is pinned + recorded, and FR-002 bench verification repeats on the
  new weights before flash.
