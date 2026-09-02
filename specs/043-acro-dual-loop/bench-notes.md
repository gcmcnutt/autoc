# 043 — bench verification (2026-09-01)

Rig: `MAMBAF722_2022A`, INAV 8.0.0 (63cffaf4), stock firmware (no fork build — see below).
Logs: `eval-results/bench-20260901/` (INAV blackbox + xiao flash log, clock-joined).
Genome: 043-t2 bake **gen 554**, fitness −87,763.77, `weight_id=610e0eba3506b149`.

## Results

| task | result |
|---|---|
| **T053** xiao selects ACRO | ✅ `ARM\|MSPRCOVERRIDE` × 1014 samples with **no MANUAL flag**. INAV also announced "ACRO" to the TX. (041-t7 was the inverse: 4962 MANUAL+override vs 184 ACRO.) |
| **T051a** `pitch_rate 24` on the FC | ✅ blackbox header `H rates:36,24,3`; `axisRate[1]` reaches **238 °/s**. |
| **T051b** sim↔FC rate parity | ✅ FC 36/24 == model XML maxRate 360/240. |
| **T056** arm-C signature | ✅ `axisF[1]` reaches **537** (was capped at 270 with pitch_rate 12) and saturates the fixed ±500 `pidSumLimit`; `servo[1]` spans **1091–2000 µs**, hitting the stop. Full elevator travel is now reachable. |
| **T056** no yaw surface | ✅ `axisRate[2] = 0.0`, `axisF[2] = 0.0` — nothing commands yaw (FR-018). |
| **T055** polarity | ✅ end-to-end, see below. |
| **T054** release | ✅ path-complete → "pilot has control" → servo reset → re-arm allowed. |
| **T057** engage latency | ✅ measured **783 ms**, see below. |
| **T058** telemetry rate | ✅ **~60 Hz** (1673 samples / 28.05 s) at `blackbox_rate_denom` as configured — matches the 041 R7 finding. |
| **SC-006** loop health | ✅ 341 ticks, **0 overruns / 0 resyncs / 0 drops**, `maxLate 17 ms`, `avgLate 0.55 ms`, pipeline `total 12.7/20.5/41.3 ms` of a 50 ms budget. |

## Clock join (xiao ↔ INAV)

Four `INAV_CLOCK` anchor events in the xiao log give `inav_ms = a·xiao_ms + b` with
**a = 1.001348 (+1348 ppm)**, b = 114.6 ms, max anchor residual **17.3 ms**. All 341 xiao ticks
matched a blackbox sample within 30 ms (median skew 4.0 ms).

## T055 — polarity, end to end

⛔ Verified to the SURFACE, not through the gyro: the airframe was static (`gyro ≈ 0`), so the final
surface → achieved-body-rate link is still untested and only closes in the air.

* xiao NN output → PWM sent (xiao-internal, same rows): pitch **−500.0** (the documented inversion),
  roll **+500.0**, throttle **+499.9**, all |corr| = 1.0000.
* xiao → INAV: `rcData → rcCommand` +0.997 / +0.971; `rcCommand → axisRate` **+1.0000** both axes.
* NN output → commanded rate: pitch **−221.7 °/s per unit** (nominal −240), roll **+189.3** (nominal
  +360). ⚠️ Magnitudes are attenuated by errors-in-variables from the 20 Hz-vs-60 Hz time join, not by
  a scaling fault — the *signs* are the result, and they are correct throughout.
* Elevon decomposition (flying wing — each servo carries both axes): pitch PID → elevon **common**
  mode **+0.9984**, roll PID → **differential** **+1.0000**. Raw per-servo correlations (0.64/0.77)
  are just the mixing, not a polarity problem.

## T057 — engage latency is 783 ms, and it is LOAD-BEARING

| event | after xiao ENGAGE |
|---|---:|
| MSPRCOVERRIDE flag (the switch) | +21 ms |
| `rcData[1]` departs 1500 → MSP data goes live | **+783 ms** |
| MANUAL clears → pure ACRO | +783 ms (same sample) |

783 ms ≈ `PERIOD_RXDATA_RECOVERY` (200) + `failsafe_recovery_delay` 5 × 100 = 700 ms, plus 60 Hz
sampling and clock residual. Both events coincide because until MSP data goes live `rcData[5]` is not
1500 either, so the mode still comes from the RX.

⭐ **Deliberately NOT removed** (operator 2026-09-01): this window is when the xiao is computing and
sending while the pilot still has control, so the NN's 0.8 s history buffers are primed at takeover.
An earlier attempt to zero it (INAV fork commit 52374cb37) was **reverted** (c412bfd76) — it would
have handed over on an unprimed buffer, and the flying behaviour depends on the delay.
⇒ **No custom INAV build is needed at all**; the FC runs stock 8.0.0 with a CLI config change only.

## ⛔ Bug found and fixed: xiao log decoder had roll/pitch swapped

`src/analytics/flightlog_decode.py` declared `OUTPUT_NAMES = ["out_roll", "out_pitch", ...]`, but the
NN output vector is **pitch-first** — `tools/dmp_dump.cc` names the same slots `out_pt,out_rl,out_th`,
and the xiao maps slot0 through the inverted pitch transform. Caught here because `out_roll`
correlated **−1.0000** with `rc_pitch`. The **firmware was correct**; only the desktop decoder's
labels were wrong. ⚠️ Any earlier analysis reading `out_roll`/`out_pitch` from a xiao log had the two
axes transposed. Same stale-label class as the 041 catalogue.
