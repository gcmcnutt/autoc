# Quickstart — 024 Sim/Real Fidelity

How to reproduce the WI1 full-flight sensor self-consistency audit on the
flight-20260417 data. This runs once the WI1 analysis tool
(`sensor_self_check.py`) is implemented; for now it documents the intended
workflow so implementers have a concrete target.

## Prerequisites

- Repo checked out on `024-sim-real-fidelity` branch.
- Python 3.11 with `numpy`, `matplotlib`, `scipy` installed.
- Reference data in place (already committed via 023):
  - `flight-results/flight-20260417/blackbox_log_2026-04-17_173039.01.csv`
  - `flight-results/flight-20260417/flight_log_2026-04-18T00-36-37.txt`
- Sim data slice:
  - `test4-data.dat` in repo root (or generate via `scripts/rebuild.sh` +
    training run).
  - Gen 400, path 0 & 2, variation 0 slice at `/tmp/gen400_p0_p2.dat`
    (create with:
    `head -1 test4-data.dat > /tmp/gen400_p0_p2.dat && grep -E '^1400400 000000 00[02]/00:' test4-data.dat >> /tmp/gen400_p0_p2.dat`)

## WI1 sensor self-consistency audit (flight)

```bash
# From repo root
python3 flight-results/flight-20260417/sensor_self_check.py \
    flight-results/flight-20260417/blackbox_log_2026-04-17_173039.01.csv
```

Expected output files:

- `flight-results/flight-20260417/sensor_self_check.png` — 6-panel summary
  with scatter plots for each cross-check
- `flight-results/flight-20260417/sensor_self_check_report.md` — per-check
  pass/fail (sign-inversion only) + Pearson r values

Expected console summary (directional pass/fail):

```
Sensor self-consistency audit — flight-20260417
================================================
Samples: N, span: T ms, cadence: mean ± jitter ms

[1/6] Position ↔ velocity integration      PASS  slope=+1.00 r=+0.999
[2/6] Quat-delta ↔ gyro                    FAIL  pitch=−0.22 (sign inversion)   ← hypothesis confirmed
                                                  roll =+0.17 r=+0.21
                                                  yaw  =+0.93 r=+0.33
[3/6] Euler from quat ↔ attitude[]         WARN  pitch sign diff (expected — INAV convention)
[4/6] Accel body-Z ↔ g·sec(bank)           PASS  (in coordinated turn)
[5/6] Quat heading ↔ ground track          PASS  offset = +X° (wind?)
[6/6] Mag direction ↔ quat heading         PASS  (modulo declination)
```

## WI2 sim conformance audit

```bash
python3 flight-results/flight-20260417/sensor_self_check.py /tmp/gen400_p0_p2.dat
# ^ same script, pointed at sim data — script auto-detects source format
```

Expected: all 6 checks PASS. Sim is the identity reference.

## WI3 root-cause fix cycle (after each fix)

1. Apply the fix (e.g., msplink pitch-axis correction).
2. Rebuild xiao (`cd xiao && pio run -e xiaoblesense_arduinocore_mbed`).
3. Re-run WI2 on fresh sim data.
4. Re-run WI1 on existing flight data — confirm the previously FAIL check
   now PASSES.
5. Commit the fix + updated PNG/report.

Expected progression:
- After msplink pitch fix: WI1 check [2/6] pitch → PASS.
- After minisim q_EB fix: WI2 check [2/6] minisim → PASS (was FAIL).

## WI4 cadence verification

```bash
# After applying accumulator fix to inputdev_autoc.cpp:341 and rebuilding:
scripts/rebuild.sh
# Run a short eval to produce a new data.dat
# ... (training / eval invocation per your standard workflow)

# Verify cadence:
awk 'NR>1 {print $4}' <new_data.dat> | awk 'NR>1 {print $1-prev} {prev=$1}' | sort | uniq -c
```

Expected: a single count line `N   100` — every row exactly 100 ms apart.

## WI5 bench verification

Bench procedure (hand-held flight FC):

1. Hold each compound attitude B1–B5 (see spec WI5 table). Record INAV MSP
   quat via xiao serial output OR blackbox dump.
2. Run `specs/024-sim-real-fidelity/bench_attitude_check.py <log>` (WI5
   deliverable) — compares each hold against expected Euler.
3. Pass criterion: each hold Euler within ±5° of expected.

Document results in `docs/COORDINATE_CONVENTIONS.md` appendix.

## WI6 post-retrain flight test

After WI3/WI4 fixes land and WI5 passes:

1. Retrain on fixed-cadence sim. Eval suite must pass (tier0–3).
2. Regenerate `nn_program_generated.cpp` via `tools/nn2cpp`.
3. Rebuild xiao with new weights.
4. Preflight checklist.
5. Fly — pilot-only warmup + short xiao engage span(s) + pilot-flown manoeuvres.
6. Run `sensor_self_check.py` on new blackbox — all checks PASS including
   the gyro↔quat-delta check on all three axes.

## Troubleshooting

| Symptom | Likely cause | Refer to |
|---|---|---|
| Reader bails on "missing column `quaternion[0]`" | Log is from mainline INAV, not custom fork | INAV_BLACKBOX.md, custom fork note |
| Flight quat↔gyro pitch sign inverted | Pre-WI3 msplink convention bug | spec WI3, research.md §1 |
| Sim data.dat cadence 117 ms | Pre-WI4 fix | research.md §3 |
| Flight `xiao_ms` ≠ `inav_ms` by ~30 ms | Expected — two independent clocks | xiao_log_contract.md |
| gyro_vs_quat.py slope ~0.5 on one axis (sim) | Possibly central-difference smoothing over aggressive maneuver | tolerable; sim identity is overall |
