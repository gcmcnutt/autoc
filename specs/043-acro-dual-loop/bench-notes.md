# 043 — bench verification (2026-09-01)

Rig: `MAMBAF722_2022A`, INAV 8.0.0 (63cffaf4), built **Aug 22 2026 12:11:07** from the fork `~/inav` branch `autoc` — ⛔ **NOT stock** (corrected 2026-09-04). The fork carries MSP override, the quaternion MSP reply, and the timestamp export the clock-join depends on. What 043 established is that **no NEW build is needed**, not that the firmware is stock.
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
⇒ **No NEW INAV build is needed**; the FCs keep the existing `~/inav@autoc` build (rev `63cffaf4`, Aug 22 2026) that also flew 041-t7, and 043 needs a **CLI config change only**. ⛔ Corrected 2026-09-04 — this line previously read *"the FC runs stock 8.0.0"*, which is wrong: the firmware is a custom fork build and always has been. Fork verified unchanged: HEAD `67ba0919e` (2025-11-13), clean tree, reflog shows no movement since, and no 2026 commits on any branch. ⚠️ The hashes this file cites for the T057 fork change and its revert (`52374cb37` / `c412bfd76`) do **not** exist in `~/inav` — that change never landed there.

## ⛔ Bug found and fixed: xiao log decoder had roll/pitch swapped

`src/analytics/flightlog_decode.py` declared `OUTPUT_NAMES = ["out_roll", "out_pitch", ...]`, but the
NN output vector is **pitch-first** — `tools/dmp_dump.cc` names the same slots `out_pt,out_rl,out_th`,
and the xiao maps slot0 through the inverted pitch transform. Caught here because `out_roll`
correlated **−1.0000** with `rc_pitch`. The **firmware was correct**; only the desktop decoder's
labels were wrong. ⚠️ Any earlier analysis reading `out_roll`/`out_pitch` from a xiao log had the two
axes transposed. Same stale-label class as the 041 catalogue.

## ⛔ What is NOT covered here — the flight article

Every result above is the **bench** board, `MAMBAF722_2022A`. Both Sept-1 CLI captures in `xiao/` are that
same board, five minutes apart, showing the rate load: `INAV_8.0.0_cli_20260901_184527.txt` has roll 18 /
pitch 9 (before) and `…185027.txt` has **36 / 24** (after). The bench rig was given the flight rates so it
could show the arm-C signature, which is why T056 reads as bench-verified despite being scoped to the
flight article.

⛔ **There is no Sept-1 dump of the flight article (`MATEKF722MINI`).** `xiao/inav-hb1.cfg` is an **Apr 2
2026** dump with the `pitch_rate 24` line hand-edited in (commit `f4a21a5` changed that one line and left
the version header stale), so it is the config **of record**, not evidence of what is loaded on the
aircraft. ⇒ Flight setup must CLI `pitch_rate = 24` into control_profile 1 on the MATEKF722MINI and pull a
**fresh dump** to prove it (T069 rate-parity gate: `rates 36,24,3` against the model's `maxRate` 360/240).
The 043-t2 bake trains against pitch 240 °/s; flying an FC still at 120 °/s diverges precisely in the axis
this feature exists to fix.

---

# 043 — bench verification #2 (2026-09-04), FLIGHT ARTICLE

⭐ First bench cycle on the **flight article** (`MATEKF722MINI`, fork build `38ff0d29e`, Aug 22 2026
16:25:46) and on the **flown genome** (gen 800). Supersedes nothing in the Sept-1 record above — that was
the bench rig on gen 554 — but it is the run that clears T069.

Logs: `eval-results/bench-20260904/`. Genome: 043-t2 **gen 800**, fitness −88,013.840878,
`weight_id=3af8e3ab787b75a5` (Sept-1 was gen 554 = `610e0eba3506b149`).

## Gates

| task | result |
|---|---|
| **flash identity** | ✅ `weight_id=3af8e3ab787b75a5`, `program=…/gen9200.dmp.zst`, topology `45→32→16r→3`. The flown genome is on the aircraft. |
| **T051b** sim↔FC rate parity ⭐ | ✅ **on the flight article** — blackbox header `H rates:36,24,4` against model `maxRate` 360/240. Roll 36, pitch 24. (Yaw 4 vs the bench rig's 3; immaterial, see below.) |
| **T053** ACRO, not MANUAL | ✅ **1044** samples `ARM\|MSPRCOVERRIDE` with no MANUAL (Sept-1: 1014). The 138 `ARM\|MANUAL\|MSPRCOVERRIDE` are the 783 ms handover window and the release. |
| **T056** arm-C signature | ✅ `axisF[1]` mean **+504**, max **+541** — saturating the fixed ±500 `pidSumLimit`, the thing `pitch_rate 12` could not reach (capped 270). Servos `servo[0]` 1294–2000, `servo[1]` 1000–1593. |
| **T056** no yaw surface | ✅ `axisRate[2]` and `axisF[2]` **exactly 0.0** across all 1044 engaged samples (FR-018). |
| **T054** release | ✅ path complete 17.0 s → "pilot has control" → servo reset → re-arm allowed. |
| **recordings** | ✅ flash logger armed → archived flight #1 → finalized. v4 log, 341 ticks, 0 gaps, 276 breadcrumbs. Blackbox 1832 I-frames @ **59 Hz** (T058 unchanged). |
| **SC-006** loop health | ✅ 341 ticks, **0 overruns / 0 resyncs / 0 drops**, maxLate 20 ms, avgLate 0.68 ms. MSP cadence avg **50.0 ms** = 20 Hz. |

⚠️ One number to watch, not act on: pipeline max **41.3 → 47.4 ms** of a 50 ms budget (5% margin);
MSP interval max 69.7 ms. Zero overruns and zero drops, so healthy — but it is the tightest figure here.
NN eval itself is only 3.16 ms (202,223 cycles @ 64 MHz).

## ⛔ The policy railed nose-down for the entire engagement — and it is the KNOWN OOD mode

`axisRate[1]` (pitch setpoint) held **+223.1 °/s mean, sd 16, range +130…+240, ZERO sign flips** across
1044 samples. INAV's pitch convention is nose-down = + (`xiao/include/MSP.h`), so this is a sustained
**93%-of-maximum nose-down rate command** for 17 s.

⭐ **This is the OOD-saturation mechanism already documented in `specs/BACKLOG.md`, reproduced on the
bench**, not a new fault:

| | `out_pt` |
|---|---|
| BACKLOG, `random` floor-dying scenarios | median **−0.927**, sustained nose-down |
| this bench run | median **−0.942**, **99%** one-directional |

The trigger is the same class of unseen initial condition. On a bench the aircraft is **static** — the
policy is fed `airspeed ≈ 0.2 m/s` against a trained cruise of 13 m/s, and every other channel is frozen
(`specific_energy` sd 0.0011, `dist_to_boundary` sd 0.0039, `score_grad` ≈ 0). Only `dist_*` and
`closing_rate` move at all, and those only because the rabbit runs away from a craft that cannot follow.

⛔ **Therefore no control-character conclusion transfers from this run to flight, in either direction.**
The bench verifies plumbing. It cannot exercise the policy, and this run is a demonstration of why.

⚠️ gen 800 is **more** railed than gen 554 here, not less: pitch sign-flips **3 → 0**, `axisRate[1]`
sd **94 → 16**. Read as OOD behaviour hardening with training, which is what the BACKLOG entry describes.

## ⭐ The throttle decision — the channels are COORDINATED, which is the encouraging part

Throttle sat at rail-low for 15 of 17 s (`rcData[3]` mean 1137, min 1000) with one full-power block at
t = 15.3–16.1 s. Structure is **identical** to gen 554 — both genomes make exactly **4 rail transitions =
0.24 Hz**. This is not throttle bang-bang; it is a slow, decisive schedule.

⭐ The throttle is not saturating independently of pitch — it moves *with* it:

| | `out_pitch` mean | `\|pitch\|` railed |
|---|---|---|
| throttle RAIL-HIGH (n=39) | **−0.831** | **10%** |
| throttle RAIL-LOW (n=250) | **−0.942** | **41%** |

`corr(throttle, pitch) = +0.244`. Throttle comes up exactly as pitch backs off the nose-down rail.
⭐ **Nose-down + throttle idle is a coherent dive; pitch easing + throttle full is a coherent pull-out.**
The policy is executing one coordinated manoeuvre, not two independently pegged channels — and it is the
textbook response to the low-energy state it believes it is in. That is the sense in which the throttle
decision is *good*: the channel is live and correlated, not stuck at a rail.

⚠️ Do not over-read it. `corr = +0.244` is modest, the throttle-high block is 39 samples, and the only
genuinely varying inputs are time-confounded (`dist_*` grows monotonically as the rabbit departs), so
input→output attribution is not available from this run. What is established is that the throttle channel
**responds**, and responds in phase with pitch.

## Clock join (xiao ↔ INAV) — T071 method rehearsed on the bench

Four `INAV_CLOCK` anchors give `inav_ms = A·xiao_ms + B` with **A = 1.000508090 (+508 ppm)**,
B = +62889.6 ms, max anchor residual **11.3 ms**. **341/341** xiao ticks matched a blackbox sample within
30 ms (median skew **4.2 ms**, max 8.5 ms).

⚠️ The ppm is per-pairing, not a constant: Sept-1 was **+1348 ppm** (bench rig), 041-t7 in flight was
**−970 ppm**. Different FC ⇒ different relative drift. The fit must be redone for every log.

## ⭐ ACRO confirmed *during* autoc — by time-join, not by inference

Mode flag at every one of the 341 engaged ticks, after joining:

| flags | ticks | |
|---|---:|---|
| `ARM\|MSPRCOVERRIDE` (pure ACRO) | **325** | **95.3%** |
| `ARM\|MANUAL\|MSPRCOVERRIDE` | 15 | 4.4% |
| `ARM\|MANUAL` | 1 | 0.3% |

⭐ The 16 non-pure ticks are **contiguous from tick 0**, spanning 0.00–0.73 s = **800 ms** — the T057
engage-latency window (measured 783 ms), where the xiao is computing and sending while the pilot still
holds the aircraft and the NN history buffers prime. **From tick 16 to the end, every single tick is pure
ACRO.** The operator's "I heard ACRO during autoc" is confirmed against the flag, not just the announcement.

## End-to-end authority, pure-ACRO ticks only (n = 325)

| link | r | slope | nominal |
|---|---:|---:|---:|
| `out_pitch` → `axisRate[1]` | **−0.890** | −298 °/s/unit | −240 |
| `out_roll` → `axisRate[0]` | **+0.895** | +304 °/s/unit | +360 |
| `out_throttle` → `rcData[3]` | **+0.980** | +490 µs/unit | ±500 band |

Signs correct on every axis; the NN is steering the FC's rate loop end to end.

⛔ **Read the magnitudes with care, and the pitch one not at all.** Over the ACRO window `out_pitch` has
sd **0.048** and range **0.189** — the policy pinned it (see the OOD section above), so its slope is a
narrow-range extrapolation, not a gain measurement. `out_roll` (sd 0.465, range 1.78) and `out_throttle`
(sd 0.654, range 2.0) *are* well-conditioned: roll's 84% of nominal is the errors-in-variables attenuation
from the 20 Hz ↔ 60 Hz join already described for Sept-1, and throttle's +490 µs/unit against a ±500 band
is essentially exact.

---

# 043 — bench verification #3 (2026-09-05 evening): pre-flight gate for the re-fly

Flight article `MATEKF722MINI`, INAV `38ff0d29e`, genome gen 800
(`weight_id=3af8e3ab787b75a5`). Logs: `eval-results/bench-20260905/` — ⚠️ the blackbox card holds **two**
logs; the bench is **log 2** (29.3 s), log 1 is the 2026-09-05 flight.

Verifies the three changes going into the re-fly: the engage-prefill fix, `rc_expo = 0`, and the gyro
Kalman OFF — plus the v5 log format.

## Gates

| check | result |
|---|---|
| ⭐ **engage prefill (the flight's root cause)** | ✅ **FIXED.** `dist[0..4] = 0.00 m` at tick 0. The bug would have seeded them with ‖origin‖ = **6.13 m** here (162–210 m in flight). All six slots now hold the correct engage geometry. |
| ⭐ **`rc_expo = 0`** | ✅ **TOOK.** Fitting `rcData → rcCommand` over 1042 pure-ACRO samples: best fit **expo 0**, residual **0.98** roll / **1.24** pitch, against expo20 at 25.88 / 13.68. Yesterday's flight fit expo=20. **The action space is linear again — it now matches the sim.** |
| **rate parity (T051b)** | ✅ `H rates:36,24,4` vs model `maxRate` 360/240. |
| **ACRO** | ✅ 1042 samples `ARM\|MSPRCOVERRIDE`; 209 `ARM\|MANUAL\|MSPRCOVERRIDE` = the T057 priming window. |
| **v5 log format** | ✅ end to end — firmware writes it, `flightlog_decode.py` and the **BLE web downloader** both read it, cone constants shown in the header (`behind=7 ahead=2 angle=45 streak>=0.5 ramp=5s multMax=5`), `step_score` populated. |
| **loop health** | ✅ 341 ticks, **0** overruns / resyncs / drops, maxLate **11 ms** (was 20), avgLate 0.49 ms. |

⚠️ Verifying expo by measurement was necessary, not pedantic: expo only bends **mid**-stick, so the
arm-C full-deflection check looks identical either way and would have told us nothing.

## ⭐ Unexpected: the gyro Kalman was costing ~10 ms of loop latency

Same board, same INAV build, same xiao cadence. The only changes were `rc_expo` (a lookup — no CPU cost)
and `setpoint_kalman_enabled = OFF`:

| xiao MSP pipeline (ms) | fetch min/avg/max | eval | send | **TOTAL** | maxLate |
|---|---|---|---|---|---|
| bench 2026-09-04 — Kalman **ON** | 7.3 / **13.0** / 35.4 | 1.1/1.6/10.2 | 3.9/5.7/9.0 | 12.7 / **20.5** / 47.4 | 20 ms |
| bench 2026-09-05 — Kalman **OFF** | 2.1 / **2.9** / 26.7 | 1.1/1.6/7.4 | 3.9/5.4/9.1 | 7.3 / **9.9** / 37.7 | 11 ms |

⭐ **Sensor→command total: 20.5 → 9.9 ms (−10.6 ms)**, essentially all of it in `fetch` — the MSP
round-trip to the FC. Mechanism: `gyroKalmanUpdate()` runs per axis at gyro rate on an F722; removing it
frees enough CPU that the MSP task services the request sooner. ⇒ **~13% of the 81.6 ms phase budget, from
a change made purely for modelling fidelity.**

⚠️ Strongly suggested, not proven — one bench run with the Kalman toggled back ON would confirm it
cleanly. Nothing else in the diff plausibly explains a 10 ms MSP round-trip change.

⛔ **Consequence for the sim**: `COMPUTE_LATENCY_MSEC_DEFAULT = 30` (`inputdev_autoc.h:57`, "bench-measured
fetch 12 + eval 5 + send 12") is now **3× the measured 9.9 ms**. Combined with §10's finding that the sim's
rate loop is also laggier than real (~150 ms vs 84 ms), the picture is consistent: **the sim is
systematically slower than the aircraft**, and a policy trained on a sluggish plant flying a snappier one
is under-damped — which is the 2–5 Hz problem 043 exists to fix. ⇒ Do not bake before this is re-pinned.

## v5 earns its 2 bytes — measured on this log

Logged `step_score` vs reconstructing it from positions (what a v4 log forces):

```
|error|  median 0.00008   mean 0.00396   p95 0.0113   max 0.637
in-streak classification disagreements: 2 / 341 = 0.59%
```

⭐ The tail is exactly as predicted from the sim study (max 0.79, ~1% disagreement). The logged value is
now ground truth; reconstruction is a lookalike. ⓘ Bench span occupancy 4.4% in-streak — meaningless on a
static rig (the craft cannot follow), reported only to show the channel is live.
