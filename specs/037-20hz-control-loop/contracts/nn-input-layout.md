# Contract: NN Input Layout & History Time-Basis

**Scope**: Phase A (sim layout) + Phase B (firmware layout in lockstep). Principle-V surface.

## Current layout (baseline)

- **M1**: `NN_INPUT_COUNT = 33`, topology `"33,32,16r,3"`, 1923 weights. History: `target_{x,y,z}[6]`,
  `dist[6]`, uniform 100 ms past-only (`HIST_PAST[]={5,4,3,2,1,0}`).
- **M2**: 54 inputs, topology `"54,32,16r,3"`, 2595 weights. History: `left/right_{x,y,cep}[6]`,
  `span[6]`, `span_rate` (one-tick diff), tilt sin/cos.

## Change (R5): tick-indexed → time-based / log-spaced

- History lags chosen on a **time** basis (e.g. log/Fibonacci-spaced: t−1,−2,−3,−5,−8,−13…), so the
  window length in seconds is ~rate-invariant and much longer than 6 ticks.
- `NN_INPUT_COUNT` (M1) and the tracker input count (M2) change with the new slot count. Topology strings
  and weight counts update accordingly.
- `span_rate` / closing_rate / derivatives computed from the **actual lag dt**, not one tick.

## Requirements (MUST)

1. **Sim and firmware layouts are identical and assembled by one shared definition** (the input enum /
   struct in `nn_inputs.h`) — no divergence between `evaluator.cc` gather and the xiao gather.
2. **Fail-loud on mismatch** (Principle V): a reader/loader encountering a dmp whose `NN_INPUT_COUNT` /
   layout ≠ the build's MUST error with both layout identities — never truncate or zero-fill. No cereal
   version bump (`feedback_no_cereal_versioning`); the loud read-fail is the safety net.
3. **Honest recording** (`feedback_honest_dmp_recording`): the new layout records ALL NN inputs + outputs
   in the dmp (audit at this schema boundary).
4. **fp32 throughout** for sim↔real parity.
5. Old t6 dmps are NOT replayed through the new layout; rate comparison uses recorded metrics (Q3).

## Tests

- static_assert on the new input-count vs struct size (mirror the existing `nn_inputs.h` assert).
- Round-trip: write a dmp with the new layout, read it back, byte-identical inputs/outputs.
- Loader fail-loud: feed an old-layout dmp → expect a clear version/layout error, not a crash or silent
  parse.
