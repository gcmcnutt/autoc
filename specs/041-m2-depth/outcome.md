# 041 — outcome

**Scope (TASK RESET):** *"a fresh full M1 toolchain, flown."*
**Verdict: achieved.** Best M1 in the project's history by 47%, flown, with the controller *and* the
airframe loading both better than the run it replaced. Closed 2026-08-23.

---

## 1. Result

| | 038-t5 (prior baseline) | **041-t7** | |
|---|---:|---:|---|
| best fitness | −45,254 @ g800 | **−81,413 @ g800** | **+80%** |
| vs all-time record (034-origm1 −55,270 @ g590) | — | **+47%** | |
| `pctInStreak` | — | 38.3% | |
| throttle pegged (>0.99) | 37.7% | **21.3%** | de-pegged |
| load p50 / p95 | 1.92 / 5.21 g | **1.45 / 4.26 g** | |
| ticks ≥ 8 g | 0.118% | **0.020%** | **5.9× lower** |
| crash rate | — | 2.7% (8/294) | |

⭐ **Score, airframe load and throttle saturation all improved together.** These normally trade; that they
did not is the substantive result, not the fitness number.

⭐ **And it finished on an improvement** — −78,996 at g790 → −81,413 at g799, at full variation scale 1.0.
It had not converged.

**Flown 2026-08-23** (genome gen 633): 4 spans, 1,682 ticks, **0 overruns, 0 resyncs, 0 gaps**.
Loop 20.4 ms mean of a 50 ms budget.

## 2. What actually caused it — and the three things that did not

⛔ **The feature spent three runs on the wrong causes before measuring the inputs.** All three are recorded
because the wrong turns are the transferable part.

| hypothesis | verdict | evidence |
|---|---|---|
| **The objective can't observe energy** → add `Es`, charge Es-destroyed | ⛔ **REFUTED as the cause** | t4 ran it to gen 511: throttle pegged **100.0%** of 129,732 ticks, `pctInStreak` **3.2%**. Es-destroyed charges for the RESULT not the EXPENDITURE — full throttle RAISES Es, so pinning the stick was *rewarded*. [objective-amendment.md](objective-amendment.md) |
| **"Every prior energy objective muted the regiment"** (spec.md reason 2) | ⛔ **PREMISE WAS FALSE** | 035's own outcome: *"ENERGY WORKS … throttle 0.93 → 0.72."* What muted was **scalar aggregation** (033, 034), not energy. Corrected in place in spec.md. |
| **Restoring the 035 throttle-power axis is the fix** | ⚠️ **worth ~8%** | t5 (correct axis) −15,985 vs t4 −14,855, and **froze at the same generation**. Not the constraint. |
| **The variation ramp being off is the constraint** | ⛔ **REFUTED** | t6 with ramp ON still pegged throttle **96.4%**. The 2/2-vs-2/2 ramp correlation was real but not causal for pegging. |
| ⭐ **The inputs are unreachable — 200:1 spread, so quiet slots are never selected on** | ✅ **CONFIRMED** | t5: every input's weight investment flat at ~1.0 for **475 generations**, `eff_rank` 11.2 → 10.9 — no differentiation *at all*. P2-8 rescaled four raw-units inputs; t7 beat the all-time record by gen 289. |

⭐ **The mechanism was not what I predicted, either.** I expected the *quiet new* inputs to come alive.
Measured, the winners are `TARGET_Z_NOW`, `DIST_NOW`, `DIST_TM2`, `CLOSING_RATE` — **the loud old channels
I rescaled**. They were not quiet, they were *saturating* the first layer; scaling restored their gradient.
The eight 041-added inputs were de-weighted (new-slot mean 0.903). ⚠️ **T068's ablation question is
therefore still open and now has a likely answer.**

## 3. Sim-to-real: mostly close, one thing is not

Measured engaged-only (xiao ticks exist only inside spans) and **distance-standardized** — the operator's
correction, and it changed the answer:

| | verdict |
|---|---|
| throttle | ✅ raw gap +0.153 → **+0.068** standardized. **56% was WHERE it flew** (sim 81.7% of time inside 10 m, real 46.4%) |
| roll rate amplitude | ✅ 88.4 vs **87.3 °/s** |
| `specific_energy`, `dist_to_boundary` | ✅ 0.11σ, 0.04σ |
| vs the July flight, engaged-only | ✅ throttle **0.952 → 0.766**, %>0.95 **87.7% → 65.7%** — the de-pegging transferred |
| ⛔ **2–5 Hz pitch/roll oscillation** | ⛔ roll power 3–5 Hz **2.39×**, 5–10 Hz **3.22×**; roll *acceleration* **1.66×** at matching rate |

⛔ **The oscillation is architectural, not tuning.** Phase budget: **81.6 ms** total loop delay →
crossover **6.1 Hz**, against an observed 2–5 Hz. The 20 Hz ZOH (25 ms) and the actuator (30 ms) are
**67%**; the gyro filter is **8%**. Every filter/baud/loop-rate fix combined buys 32 ms and lands at 10 Hz
— still inside the band. ⭐ **Muting it and dulling the controller are the same knob at 20 Hz.**
→ [043 ACRO dual-loop](../043-acro-dual-loop/README.md).

## 4. Six stale labels, one class

⭐ A tool changed, a caller did not, nothing checked. Worth naming as a pattern because the expensive one
cost three runs:

* **input scales** (P2-2 un-normalized what 030 normalized) — three runs
* `NN_TOPOLOGY_STRING` read `"42"` for a 45-wide net — misled this investigation
* energy chart titled *"P2-5 Ps objective"* after P2-7 withdrew it
* renderer read post-P2-8 NN units as physical
* log decoder still 37-based → junk telemetry
* `input_investment` called `nn2cpp` with the pre-refactor CLI — panel froze at 551 gens for a day and
  shipped **two truncated reports**; the tool failed loud and the caller **swallowed the message**

⭐ Guards added: `static_assert` on both topology strings, `nn_input_scaling_tests`,
`shared_input_block_tests`, P2-9's scale signature in `nn2cpp`, and the swallowing `except` now prints.

## 5. Verified, not assumed

* **Toolchain**: gen 633 regenerated through a toolchain changed three ways since the bake —
  `generatedNNWeightId` and the `nn_weights[2307]` array **byte-identical**. Only P2-9's guard block differs.
* **Filters**: `gyro_lpf_hz:25`, `acc_lpf_hz:15`, `acc_notch_hz:0` — read from the **flight's own blackbox
  header**, not the `.cfg`. Measured roll-off is steeper than PT1 alone (dynamic notch, Q 250).
* **Clock join**: −970 ppm fit, cross-validated to **0.5%** against `ARM|MANUAL|MSPRCOVERRIDE`.
* **Retention**: 800/800 objects `retain=keep`, [MANIFEST](artifacts/t7-baseline/MANIFEST.md) with the
  scale constants — ⛔ without them the genome loads clean and flies wrong.

## 6. Carried forward

| item | to |
|---|---|
| 2–5 Hz oscillation, dual-loop | **043** |
| camera params | **042** |
| M2 tracking + its two gates | **044** |
| formal measured normalization (replaces hand-derived constants) | BACKLOG |
| **P2-10** variation-sweep "knee" | open in 041 |
| **P3-1/P3-2** datum + renderer `'a'` acceptance | open in 041 |
| T068 ablation — do the 8 added inputs earn their keep? | open, now with a likely answer |
