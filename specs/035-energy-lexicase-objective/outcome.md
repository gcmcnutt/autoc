# 035 Outcome — Energy as a Lexicase Secondary Objective

**Status**: M1 COMPLETE (2026-06-08). M2 COMPLETE — qualitative (2026-06-08, t7; off-nominal
scenario set, see M2 caveats). Clean M2 milestone bake deferred to the skip-fix (037 prework).

## M1 verdict: **ENERGY WORKS** (energy-lexicase, no tracking collapse)

Run **t6** = `autoc-9223370256079660488-2026-06-06T19:45:15.319Z` (autoc-m1), `autoc.ini`
pop 5000 / wind 49, `LexicaseEpsilonMode=mad`, energy axis on, seed 1780775115. 800 gens,
~36.6 h, 1.18 B sim-runs.

The three classifications from the spec were energy-works / tracking-collapses / energy-unmoved.
**M1 = energy-works**, on the evidence below.

### 1. No tracking collapse — clean climb, full envelope
Best fitness climbed monotonically to **-37689.57** at gen 800 (gen 1 -2250 → 100 -6923 → 400
-25878 → 800 -37690), **0% crash, 294/294 scenarios completing**. The energy axis did not collapse
tracking (the failure mode 027/028 feared and 033's scalar smoothness hit). It also did **not stall
in the basin lottery** at this config — a clean single-bake climber (T036's 2–3 budget unused).

### 2. The energy win is real and specific — throttle came off Vmax-pinning
The clearest energy signal is throttle amplitude falling **over the run**: `mag_throttle` 0.93
(gen 100) → **0.72** (gen 800). The controller learned to *spend less throttle*. Contrast the 034
energy-*scalar* run, which **froze throttle pinned at ~0.997** (a dead axis). Pitch also de-saturated
(`mag_pitch` 0.83 in 034 → 0.69 here). End-state amplitudes are **balanced** — pitch 0.69 / roll
0.70 / throttle 0.73 — vs 034's degenerate saturated-pitch / pinned-throttle corner.

### 3. Lexicase beats same-config scalar by *specializing* the per-axis effort
Same config (pop 5000 / wind 49), scalar→lexicase, per-axis `dctrl ⟨|Δ|⟩`:

| axis | 034 energy-scalar (gen 590) | t6 energy-lexicase (gen 800) | Δ |
|---|---|---|---|
| pitch | 0.524 | 0.506 | smoother |
| throttle | 0.381 | 0.305 | **−20% smoother** |
| roll | 0.527 | 0.952 | concentrated here |

Scalar spread aggressiveness evenly across axes (no specialization). **Lexicase smooths the
energy-expensive axes (pitch, throttle) and concentrates change-rate on roll** — the axis whose
aggressiveness is energetically cheapest (aileron drag ≪ AoA/induced-drag and throttle power; see
`tuning-backlog.md`). That is the energy-rational allocation, and it is *caused by* the selection-rule
change — the headline US1 result.

### 4. The one thing energy did NOT smooth — roll — is correctly out of scope
Roll stays bang-bang (`dctrl_roll` ~0.95–1.25 throughout). Per-tick + sim↔real analysis attributes
this largely to the **10 Hz control-loop rate** (the roll command dithers at the ~5 Hz control
Nyquist), NOT to the objective — it is a control-rate artifact, addressed by feature **037**
(faster loop), not by any fitness change. So incomplete roll smoothing is **not** an 035 failure;
the objective smoothed every axis it has leverage over.

## Total-energy (PE/KE) go/no-go (FR-007)
The v1 **convex throttle proxy** (`Σ((out_th+1)/2)²`) produced real, directional movement (throttle
off the ceiling, balanced amplitudes) **without** a measured power curve. **Recommendation: GO** on a
follow-on total-energy (altitude+airspeed) objective — the proxy worked well enough to justify the
richer metric, and the ESC current-monitor calibration remains the gating prerequisite for the
measured curve (per Clarifications). Separate feature; not 035.

## Caveats
- **One M1 bake.** Clean climber, but n=1 at this config (the lottery just didn't bite here).
- **Not the smoothest controller ever** — 029 (no-energy, old arch) had lower absolute dctrl on all
  axes, but on an easier objective; the relevant comparison is same-config scalar (above), which
  lexicase wins on the energy axes.
- **pitch dctrl non-monotonic** — dipped to 0.37 (gen 500) then rose to 0.50 (gen 800); the deep
  smoothing mid-run wasn't fully held. Energy pressure vs late variation-ramp interplay.

## Milestone pinning (Principle VIII) — TODO
Pin the t6 M1 run so it survives the 30-day expiry:
```
# operator: autoc-pin equivalent
aws s3api put-object-tagging --bucket autoc-m1 \
  --key autoc-9223370256079660488-2026-06-06T19:45:15.319Z/<gen>.dmp.zst \
  --tagging 'TagSet=[{Key=retain,Value=keep}]'   # for each pinned gen (or the final + a few checkpoints)
```
**Pinned prefix:** `autoc-m1/autoc-9223370256079660488-2026-06-06T19:45:15.319Z/` (record here once tagged).

## M2 verdict: **ENERGY WORKS** (energy-lexicase generalizes to tracker mode — qualitative)

Run **t7** = `s3://autoc-m2/` (tracker), `autoc-tracker.ini`, source `TrackerSourceRun=t6
gen9200.dmp.zst` (the pinned M1 above), `LexicaseEpsilonMode=mad`, energy axis on, same seed family.
Stopped early at **gen 207** ("close enough to proceed" — the qualitative verdict was clear; the run
was not driven to convergence the way M1's 800-gen bake was).

**Classification: M2 = energy-works.** Energy-as-lexicase did **not** collapse tracking in the
lottery-prone tracker mode, and it produced the same energy-rational per-axis allocation as M1.

### 1. No tracking collapse — energy generalizes to the harder mode
Best climbed monotonically -4430 (gen 1) → -9258 (50) → -12143 (100) → **-14760** (207), with
**268/294 scenarios completing** (12 hullStrike, 14 eval) and `avgMaxStreak` 9.0 / `pctInStreak` 8.5
at the stop. Live recurrence held (`whh_xh_ratio` 0.66). This is the key US1 generalization result:
the failure mode 027/028/033 feared (energy/smoothness pressure killing tracking) **did not occur**
in tracker mode either.

### 2. The energy signature in M2 is *smoothing*, not amplitude-shedding — and that's task-correct
M1's energy win was throttle amplitude coming off the ceiling (`mag_throttle` 0.93 → 0.72). **M2 is
different and instructive**: throttle climbed *to* the ceiling and stayed there (`mag_throttle` 0.20
→ 0.94) but became **extremely smooth** — `dctrl_throttle` rose then fell hard to **0.088** (gen
207), i.e. a near-constant high throttle, no pumping. The tracker task demands sustained power to
chase a maneuvering target, so amplitude *can't* drop the way M1's could; instead energy-lexicase
saved energy on the axis it still could — **change-rate** (no wasteful throttle pumping). Same
objective, opposite-looking lever, both energy-rational for their task.

### 3. Same per-axis allocation as M1 — roll carries the change-rate
Per-axis at gen 207: `dctrl` pitch **0.61** / roll **1.38** / throttle **0.088**; `mag` pitch 0.86 /
roll 0.83 / throttle 0.94. Identical pattern to M1: **lexicase smooths the energy-expensive axes
(throttle especially, pitch moderately) and concentrates change-rate on roll** — the cheapest axis
(aileron drag ≪ throttle power / induced drag) *and* the one whose bang-bang is the 10 Hz
control-rate artifact (037 territory, not an objective failure). The allocation is selection-rule-
caused and consistent across both modes — the headline result reproduces.

### 4. Tracking quality vs baseline — comparable, NOT a clean win (and not claimed as one)
The 034-m2 tracking baseline (gen 251, `autoc-034-m2-tracker-origm1`) reached `avgMaxStreak` **11.5**
/ **278/294** / 9 hullStrike — *slightly better tracking* than t7's 9.0 / 268 / 12. But t7 ran
**fewer gens (207 vs 251)** AND on the **off-nominal scenario set** (skip bug below), so this is not
a fair head-to-head. The honest read is **"energy did not regress tracking out of the comparable
band,"** not "energy improved tracking." A clean, converged M2 bake is needed for a real comparison.

## M2 caveats (why this is qualitative, not a milestone)
- **Off-nominal scenario set.** t7 ran with the short-source skip bug active (`c95887e`): the source
  list was trimmed to 292 while the eval still iterated 294 slots, so `srcIdx = pathSelector %
  sourceList.size()` reshuffled ~200 (variation, source) pairings and duplicated 2 sources. Tracking
  quality numbers (268/294, hullStrike 12) are therefore **not clean or reproducible**. The
  energy/per-axis *allocation* verdict is robust to this (it's a controller-behavior signature, not a
  scenario-accounting artifact); the *tracking-quality* numbers are not.
- **Stopped early (gen 207, n=1).** Not driven to convergence; M1 ran 800. Qualitative verdict only.
- **Clean M2 milestone bake is deferred** to after the short-source skip fix (keep 294 1:1,
  neutralize in place) — tracked as **037 prework** (`specs/037-20hz-control-loop/spec.md`,
  "fix the short-source skip"). That rerun, not t7, is the pinnable M2 milestone.

## Combined verdict
**Energy-as-lexicase works in BOTH modes** — M1 (clean, converged, energy off the throttle ceiling)
and M2 (qualitative, energy as throttle-smoothing under a power-hungry tracking task). The per-axis
energy-rational allocation (smooth throttle+pitch, change-rate on cheap roll) reproduces across modes
and is caused by the lexicase selection rule. US1 is satisfied. The remaining roll bang-bang is a
control-rate artifact owned by 037, not an objective failure.
