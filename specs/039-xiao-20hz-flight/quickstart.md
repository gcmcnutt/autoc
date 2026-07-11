# 039 Quickstart — regen → bench → fly → decode → compare

The end-to-end operator path once implementation lands. Referenced files/contracts in this dir.

## 1. Regenerate firmware from the pinned 038 elite

```bash
# extract the pinned t5 elite (autoc-m1 bucket, gen 800)
build/nnextractor -k autoc-9223370253844606963-2026-07-02T16:36:08.844Z -o nn_weights.dat -i autoc.ini
# emit UNROLLED 37-in/2051-w code with the arena TEMPLATE (placement happens at engage, D5)
tools/nn2cpp -i nn_weights.dat -u -a 80,5,100 -o xiao/src/generated/nn_program_generated.cpp
cd xiao && pio run -e xiaoblesense_arduinocore_mbed   # Constitution II gate
```

Note: `-u` requires the 039 unrolled-recurrent support (D6) — before that lands it silently falls
back to table-driven (check the emitted header comment).

## 2. Bench (contracts/bench-validation.md)

```text
flash → ground ERASE:ALL (initializes log) → engage a span on the stationary bench →
BLE download → decode → review per FR-002 checklist (arena floor MUST read −25 clamp on bench)
→ then the FR-011 soak: several consecutive 3–4 min spans at divisor=1 (20 Hz)
```

## 3. Latency memo + operator review (contracts/latency-memo.md)

Bench pipeline stats (both bauds) + DWT eval cycles → memo → operator decides: model stands /
amend + retrain (train.sh, pinned, re-bench) / amend without retrain.

## 4. Fly

Pre-flight per `project_preflight_checklist` + FR-013 (failsafe unchanged). Two flights of 3–4 min
fit one flash fill. Field retrieval over BLE (~7 s for a full log).

## 5. Decode + acceptance report

```bash
python3 src/analytics/flightlog_decode.py flight_log_<ts>.bin -o flight_ticks.csv   # loud-fails on version/CRC
python3 src/analytics/flight_vs_sim_axes.py flight_ticks.csv --sim <t5 sim per-axis baseline> \
    -o specs/039-xiao-20hz-flight/autoc-039-t<N>-flight_vs_sim.png
```

PASS = each axis within ±25% of sim on dCtrl ⟨|Δu|⟩ AND amplitude ⟨|out|⟩ (SC-005), plus the
qualitative bang-bang read vs the 2026-05-17 flight. Verdict + residual gaps → outcome.md (SC-006).
