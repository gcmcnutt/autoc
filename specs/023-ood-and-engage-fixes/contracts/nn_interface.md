# Contract: NN Interface

**Feature**: 023 | **Phase**: 1 | **Scope**: compile-time API between sensor gathering and NN forward pass

## Interface boundary

The `NNInputs` struct (defined in `include/autoc/nn/nn_inputs.h`, documented in
`data-model.md` §1) is the **sole contract** between four producers and one
consumer:

### Producers (sensor gathering — populate `NNInputs`)

| Producer | Location | When it runs |
|---|---|---|
| CRRCSim bridge | `crrcsim/src/mod_inputdev/inputdev_autoc/inputdev_autoc.cpp` | Each CRRCSim physics tick during training |
| Minisim | `tools/minisim.cc` | Each simulated tick during minisim smoke test / training |
| Xiao flight firmware | `xiao/src/msplink.cpp` | Each real-flight NN cycle (nominally 10 Hz) |
| Evaluator (unit tests) | `tests/nn_evaluator_tests.cc`, `tests/contract_evaluator_tests.cc` | Test fixtures |

### Consumer (matrix multiply — reads `NNInputs`)

| Consumer | Location | Access pattern |
|---|---|---|
| NN forward pass | `src/nn/evaluator.cc` → `nn_forward()` | `reinterpret_cast<const float*>(&inputs)` for inner loop MAC |

## Contract rules

### P1. Producers MUST populate every field

Every producer must use **designated-initializer construction** or field-by-field
assignment that exhaustively sets every member of `NNInputs`. Uninitialized
fields are a bug.

```cpp
// CORRECT — designated initializer flags missing fields at compile time
NNInputs inputs {
    .target_x = {...},
    .target_y = {...},
    .target_z = {...},
    .dist = {...},
    .closing_rate = ...,
    .quat_w = ...,
    // ... etc
};

// INCORRECT — may leave fields uninitialized
NNInputs inputs;
inputs.target_x[0] = ...;  // Missing fields uninitialized
```

Test coverage: `tests/nn_inputs_tests.cc` includes a "poison-value" test that
constructs an `NNInputs` via `memset(&inputs, 0xAB, sizeof(inputs))`, runs each
producer's population routine, then asserts every field is NOT 0xABABABAB.

### P2. Producers MUST maintain the unit-vector invariant

The `target_x/y/z[i]` triple at each time sample must satisfy
`x² + y² + z² ≈ 1.0` (within FP rounding). This is enforced by having all
producers call the shared helper `computeTargetDir()` (documented in
`contracts/history_reset.md` §2) rather than normalizing inline.

### P3. Producers MUST maintain the time-sample ordering

Index 0 = oldest history sample, index 3 = now, indices 4-5 = lookahead.
Producers are responsible for shifting history on each tick (index 0 discarded,
index 1 → 0, etc.) and populating index 3 from current state + indices 4-5
from path projection.

History buffer shift logic lives in a shared helper
`AircraftState::advanceHistoryBuffer(NNInputs&, const NNInputs& latest)`
so all producers use identical semantics.

### P4. Consumer MUST NOT assume field order for its inner loop

The matrix multiply in `nn_forward()` uses `reinterpret_cast<const float*>(&inputs)`
to get a contiguous `float[NN_INPUT_COUNT]` view. This is safe because of the
`static_assert` on `sizeof(NNInputs) == NN_INPUT_COUNT * sizeof(float)` and
`alignof(NNInputs) == alignof(float)`.

The weight matrix order (which row corresponds to which input) is implicitly
defined by the declaration order in `NNInputs`. Changing declaration order
invalidates all saved weight files. This is the constitutional note in the
header — field order IS the serialization contract.

### P5. Logging MUST cover every field

Every `NNInputs` field MUST be logged to `data.dat` and to xiao flash logs,
one column per field, every tick. The column header comes from the field name
(e.g., `target_x_0`, `gyro_p`). Implementation: a single `emitNNInputsColumns()`
helper used by both the `data.dat` writer and the xiao log writer.

If a producer forgets to populate a field, it still gets logged — the log is
the audit trail. P1's poison-value test catches this at test time.

### P6. Parsers MUST read by column name

`sim_response.py` and any other `data.dat` parser MUST look up columns by
name, not by positional index. Silent column reordering (which should never
happen per P4, but belt-and-suspenders) then becomes a `KeyError` at parse
time instead of a misread plot.

```python
# CORRECT
target_x_now = row['target_x_3']

# INCORRECT
target_x_now = row[7]  # Will silently read a different column if order changes
```

## Compile-time checks (enforced via `static_assert`)

1. `sizeof(NNInputs) == NN_INPUT_COUNT * sizeof(float)` — no padding
2. `alignof(NNInputs) == alignof(float)` — safe for reinterpret_cast to `float*`
3. `NN_TOPOLOGY[0] == NN_INPUT_COUNT` — topology.h matches nn_inputs.h
4. `NN_WEIGHT_COUNT == 1667` — weight arithmetic matches declared topology

All four assertions live in `include/autoc/nn/topology.h` which includes
`nn_inputs.h`. Any one of them failing is a hard compile error, not a warning.

## Runtime checks (enforced in producers)

1. After each population, optional debug-build assertion that
   `target_x[i]² + target_y[i]² + target_z[i]² ∈ (0.99, 1.01)` for all `i`.
2. After each population, optional debug-build assertion that
   `quat_w² + quat_x² + quat_y² + quat_z² ∈ (0.99, 1.01)`.

Both assertions disabled in release builds for performance. Not load-bearing —
just early warning during development.

## Verification test matrix

| Test | File | Purpose |
|---|---|---|
| Sizeof contract | `tests/nn_inputs_tests.cc` | `static_assert` fires if padding introduced |
| Alignment contract | `tests/nn_inputs_tests.cc` | `static_assert` fires if struct misaligned |
| Topology match | `tests/nn_inputs_tests.cc` | Asserts `NN_TOPOLOGY[0] == sizeof(NNInputs)/sizeof(float)` |
| Weight count | `tests/contract_evaluator_tests.cc` | Asserts `NN_WEIGHT_COUNT == 1667` |
| Poison-value completeness | `tests/nn_inputs_tests.cc` | Every field populated after producer call |
| Unit-vector invariant | `tests/nn_inputs_tests.cc` | `computeTargetDir` output is unit vector |
| Field-name round trip | `tests/nn_inputs_tests.cc` | Write `NNInputs` to `data.dat`, parse via `sim_response.py`, compare |
| Deliberate-break (Phase 0a.3) | Manual | Add throwaway field, confirm compile error, revert |

## Migration order (Phase 0a.1 → 0a.2 → 0a.3)

### Phase 0a.1 — Refactor at 27 inputs, no behavior change

1. Create `include/autoc/nn/nn_inputs.h` with a 27-field `NNInputs` struct
   matching the current 022 layout (no target_x/y/z yet — keep the old
   `dPhi[6]` + `dTheta[6]` fields for this step).
2. Update `AircraftState::nnInputs_` to store `NNInputs` instead of
   `std::array<float, 27>`.
3. Update `nn_gather_inputs()` in `src/nn/evaluator.cc` to populate via
   designated initializers.
4. Update CRRCSim, minisim, xiao, and all tests to populate via field names.
5. Update `data.dat` writer to emit field-name column headers.
6. Update `sim_response.py` to parse by column name.
7. Update `nn2cpp` codegen to emit field-name access.
8. **Run all tests.** Build both autoc and xiao. Zero behavior change expected.
9. **Pre/post diff test:** run a fixed scenario pre-refactor and post-refactor,
   compare `data.dat` output. Column count matches (27+3+meta), per-tick values
   match bitwise.

### Phase 0a.2 — Grow to 33 inputs

1. Edit `nn_inputs.h`: replace `dPhi[6]` + `dTheta[6]` with
   `target_x[6]` + `target_y[6]` + `target_z[6]`. Update `static_assert`
   to expect 33 floats.
2. Update hidden layer sizes in `topology.h`: `{32, 16}`.
3. Compile. **Expect compile errors** at every site that still references
   `inputs.dPhi[...]` or `inputs.dTheta[...]` — fix each one to use
   `computeTargetDir()` and populate the new fields.
4. Update weight deserialization: existing NN01 files at 611 weights cannot
   be loaded by a 1667-weight network. Add a version check that rejects old
   files with a clear error message. No compat shim.
5. Train a short minisim run to verify plumbing works (Phase 2 smoke test).

### Phase 0a.3 — Deliberate-break verification

1. In a scratch branch: add a throwaway field `float DELIBERATE_BREAK;` to
   `NNInputs`.
2. Build — `static_assert(sizeof(NNInputs) == 33 * sizeof(float))` must fire.
3. Capture the compile error output, paste into the commit message as
   "deliberate-break verification confirmed at <file:line>".
4. Revert the change.

If the compile error does NOT fire, the struct layout safety is broken —
investigate immediately (probable cause: `NN_INPUT_COUNT` is being defined
as a magic number somewhere instead of being derived from `sizeof`).
