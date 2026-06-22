# t12 retrospective — hull-crash penalty: theories, top levers, and what's left in 037

**Date**: 2026-06-19. **Run**: t12 (`autoc-037-t12-m2-hullpen`, gen 1→774), vs t11 baseline (gen 1→604).
**Companion**: T001 outcome in [tasks.md](tasks.md); plots in `specs/037-20hz-control-loop/autoc-037-t12-m2-hullpen_*.png`.

> Bottom line: **the penalty objective worked** (hull-strikes → 0, sustained). It is **too heavy-handed**
> in a specific, mechanical way (it amplifies fitness variance), and **tuning the weights trades the
> failure mode around the encounter-distance axis without solving the underlying brittleness**. The
> brittleness is a perception/gradient problem on hard courses — *not* recurrent capacity (confirmed:
> W_hh eff-rank ~10/16, not saturated) and *not* something penalty weights can fix.

## 1. Theories on the ragged results

### 1a. Why ragged *as the variations ramp* — and worse than other runs
The variation ramp does **not** redraw scenarios each gen. `applyVariationScale()` takes each scenario's
**fixed-direction** perturbation (its seeded gust / entry-pose / position offset) and **scales the
magnitude** by a ramp factor `[0→1]` that climbs over generations (craft variations are unramped — full
from gen 0). So as the ramp climbs, each scenario's *same* perturbation grows until it **crosses a
failure threshold** (leaves the arena → `Eval`/OOB; pushes the target out of FOV → lost track;
makes the M1 trail-path infeasible in the chase's air mass). Scenarios fall off these cliffs **one at a
time** as the scale rises → the crash count moves in **discrete steps**, not smoothly.

**Empirically confirmed**: t12's best-fitness gen-to-gen step is **3.5× t11's** (mean |Δbest| **105 vs
30** over the last 200 gens), and it's **plateau-hopping** between distinct levels (−14634 → −12532 →
−13728 → −12122 → −13836) — the elite sits on one level for several gens, then jumps — not high-frequency
jitter. So the raggedness is real and t12-specific.

**Corrected mechanism (the first draft of this section was wrong).** I initially blamed the `0.5^K`
multiplier amplifying the elite's own fitness — but **the t12 elite held `hullStrike=0` the whole run, so
the multiplier never fired on the reported-best member** (`0.5^0 = 1`). And `LexicaseEpsilonMode=mad` was
**already on** in both t11 and t12 (since 035), so this isn't a constant-ε tie-breaking artifact either.
The real cause is **indirect — the penalty reshapes the selection landscape**:

1. **The harsh penalty *sparsifies the top* of the landscape.** `0.5^K_hull` is steep enough (K≈12 →
   ×0.0002) that it **annihilates** every hull-striking member. t11 (penalty off) could select from a
   *smooth gradient* of "slightly-crashy but excellent tracker" members; t12 restricts the contention to
   **hull=0 members only** — a sparser, more multi-modal set at the top.
2. **The ramp reshuffles which clean member is best.** Among that sparse hull=0 set, the tracking-vs-OOB
   tradeoff is contested, and as the ramp pushes scenarios over OOB cliffs (§1a) the ranking flips → the
   elite **hops between distinct clean members** at different fitness levels → the plateaus-and-jumps.
3. **MAD-ε can't smooth a sparse top.** It breaks ties on real per-scenario spread, but it can't
   manufacture a gradient where the penalty removed one. So the fix must **restore the gradient** (soften
   the penalty), not sharpen the comparison.

Corollaries, all consistent with "harsh penalty sparsifies the top + ramp reshuffles":
- **population avg worsening** → the penalty crushes the crasher tail of the population (and the ramp
  hardens the environment), dragging the mean down → the run converges/ends quick.
- **sigma flattening** → with the smooth-tracker gradient removed from the top, selection can't keep
  contracting the spread on tracking quality.
- **avg-max-streak worsening** → the surviving clean members favor the safe standoff tactic, which trades
  streak depth for safety (see 1c).

### 1b. The encounter-distance histogram
Min-distance per scenario shifted **outward** wholesale: median 2.17 → **3.63 m**, frac < 2 m 48% → **17%**,
frac < 1 m (hull) 16% → **3%**. Read as two populations:
- **A healthy tracking peak near the 3 m trail distance**, throttle-modulated — this is *good* behavior
  (hold standoff and track, exactly the operator's "track, not intercept-and-wave-off"). The penalty
  pushed this peak from ~2 m (collision-courting) to ~3.6 m (safe trail). **Desirable.**
- **A brittle far tail** — courses where the chase **loses the target** and distance blows out to the
  arena edge → OOB. This tail is **not** "chose to flee the hull"; it's "**failed** on a hard course."

This nuance revises the pure-displacement story: most of t12's OOB is **brittleness on hard courses**,
not a deliberate bail. That matters for the prescription — see §2.

### 1c. Throttle modulation is correct, not a pathology
The throttle-cut-when-close policy (tactics Panel B) is the controller doing the right thing for a
*tracker*: modulate energy to hold the trail point rather than overshoot-and-wave-off. The standoff peak
is the *success* mode. We should preserve it, not tune it away.

## 2. What this means: tuning is trading, not solving

- The penalty **achieved its objective** — collisions are gone and tracking depth held
  (`rabbitComplete` 272–273/294). That's a real safety win on the courses it flies well.
- **Penalty-weight tuning moves failures along the encounter-distance axis** (collision ↔ standoff ↔
  flyaway). It cannot manufacture skill on the brittle courses; it only chooses *which* way they fail.
- **The brittleness root is perception/gradient, not capacity.** "Good for some courses but brittle" is
  the FOV-limited / complex-maneuver courses where the chase can't predict through the blind gaps. W_hh
  eff-rank ~10/16 on *both* runs says the 16-dim memory is **not saturated** — a wider RNN would add idle
  modes, not skill. So the lever is **better signal** (gradient / perception), not more recurrent width.

## 3. Top levers (ranked) — for the next run(s)

### #1 — Restore the selection gradient: soften the hull penalty + give OOB a smooth cost
The raggedness is the harsh `0.5^K_hull` *annihilating* crashers and sparsifying the landscape top (§1a) —
**not** a constant-ε artifact (MAD-ε is already on and didn't smooth it). So #1 **restores a smooth
gradient**: relax the hull factor `0.5 → 0.75` (penalize, don't annihilate), and give OOB a smooth cost
instead of being free. MAD-ε stays on, unchanged. **Does not fix brittleness** — but stops the penalty
from *manufacturing* selection noise on top of it.

**t13 (first attempt, 2026-06-19) — STOPPED at gen 7, instructive failure.** Used a *linear-clamped*
OOB term `mult ×= max(0, 1 − w·K_oob/N)`, `w=2.0`, **un-ramped**. It **collapsed the early population**:
the term hits 0 at 50% OOB and clamps everything above it to exactly 0, and in early gens *most* random
members crash >50% OOB → gen-1 **avg fitness 0** vs t11/t12's ~−3000, worst pinned at 0. The clamp didn't
trim the dead tail — it **flattened the bulk to an identical zero**, destroying the early selection
gradient (only the elite + a sliver stayed non-zero). Confirmed the "don't kill the fresh population
before it learns" failure with hard numbers.

**t14 (the fix, 2026-06-19) — curriculum-ramped, smooth, never-clamped.** Both terms scaled by
`scale = computeVariationScale()` ∈ [0,1]:
- **Hull**: `factor^(K_hull · scale)`, `factor=0.75`. Early (scale≈0) → 1 (forgive the rare/noise early
  strikes); late → full. Matches "hull rare early → skill-correlated late."
- **OOB**: `exp(−w · scale · K_oob/N)`, `w=2.0`. Early → 1 (forgive the common random fly-out → **no
  population collapse**); late → full. **Smooth exponential, never reaches 0**, monotonic in the OOB
  rate → per-scenario ordering (selection signal) preserved at *every* crash rate. Matches "OOB common
  early → should-be-gone late."
- A single ramp direction (low→high) serves both because both want **forgive-early / enforce-late**.
  Eval-mode reuses the stored `variation_scale` → bitwise-reproducible.
- **Implemented**: `src/autoc.cc` (ramped two-shape penalty, `<cmath>`); tracker inis hull=0.75 / w=2.0;
  M1 inis w=0.0 (M1 bit-identical — penalty gated off; note the hull term is moot for M1 anyway since
  `HullStrike` is tracker-only). Build clean, contract tests pass. Ready for the operator to launch as t14.

### #2 — Reproduce M1's on-course wind (parity) — cheap falsification of the infeasibility tail
Today the chase flies the t10 source paths in a **different** air mass than M1 recorded them in. On the
**complex t10 paths**, a trail-the-rabbit task can be **physically infeasible** in a mismatched wind
(you can't hold 3 m behind a path flown in a tailwind while you're in a headwind) — and that infeasibility
**grows as the ramp scales `WindDirectionSigma`**, feeding both the brittle far-tail (§1b) and the
ragged-as-ramp behavior (§1a-1). Replaying the source's wind removes that axis.
- **Honesty caveat**: `wind_study.py` found wind-delta **uncorrelated** with per-scenario score (corr
  +0.03) at the current ramp point — evidence this may be **low-value**. But it's **cheap and
  falsifiable**: run it, see if the OOB far-tail and the raggedness shrink. If they don't, we've cleanly
  ruled wind out and learned the brittleness is purely perception/control.

### The honest meta — the real solve is reward-shaping / perception (next cycle, not a t13 knob)
Neither lever above *solves* brittleness; they de-noise (#1) and de-risk-one-hypothesis (#2). The
durable fix for "brittle on hard courses" is **gradient/perception**, and we already have the candidates:
- **Negative reward when the chase gets *ahead* of the target** (backlogged 2026-06-19) — gives the
  controller a gradient to *follow back* into the trail position on a hard course, instead of a cliff to
  flee from. A reward-shaping sibling to the penalty; may shape safety through the gradient and shrink OOB
  without any crash multiplier.
- **Reshape the tracking cone / ramp pacing** for the harder t10 paths (steeper `FitDistScaleAhead`,
  slower ramp so the controller consolidates before the environment hardens).
- **Perception-through-blindness** (the 43%-blind-before-strike problem) — the 038 camera/CEP and the
  US4 deeper-history levers; this is where capacity *would* matter, but as input richness, not W_hh width.

**Recommendation**: t13 = **#1 (penalty shape + MAD-ε)**, because it directly addresses the clearest
observed problem (raggedness) and is the smallest change; run **#2 (wind parity)** as a parallel cheap
falsification. Treat both as *de-risking*, and route the **negative-ahead reward-shaping** into the 038
US1 design as the actual brittleness solve — not a t13 weight tweak.

## 4. What else is left in 037

037's **M1-controller** half is closed (t10 GO, pinned). The M2 retrain half (US1b) is effectively done
(t11 = clean 20 Hz M2; t12 = this penalty probe). What remains is genuinely **two buckets**:

**A. The xiao M1-flight track (the deliberately-parallel "037 closeout" — substantive, not M2 work):**
- **US2 (xiao at 20 Hz)** — T031–T040: local-IMU (LSM6DS3) fusion + auto-cal, 021 convention
  cross-check, raise link baud, INAV-side 20 Hz override, `MSP_NN_EVAL_DIVISOR=1`, packed dual-stream log.
- **US3 (50 Hz)** — T041–T045: **fully unroll + fast-tanh the NN forward pass**, RT slot scheduling,
  tail-bounding (async QSPI, NVIC priorities). Gated on US1 clearing at 20 Hz.
- **R-tasks** T009/T010 (camera-fps / transport-ceiling) — largely answered by the 20 Hz GO; confirm.

**B. Housekeeping / hygiene (quick wins, can fold into the wrap):**
- **P-O11** — time-denominate the rate-dependent reports (raw tick-denominated streak metrics read 2× at
  20 Hz; convert to seconds or pctInStreak).
- **P-O8** — `svTau` cleanup (draw-order vestige).
- **P-O4** — basic-m1 smoke ×2 (standing rule after any loop-plumbing change).
- **P-O5** — 10 Hz confirm bake (only if we want the rate-isolation anchor; optional now that 20 Hz is GO).
- **T028** — US1 milestone-close type-domain grep audit on touched `src/eval/ src/nn/`.

Nothing in 037 blocks the M2/038 direction; bucket A is the real remaining spend and it's the flight-prep
track running in parallel.
