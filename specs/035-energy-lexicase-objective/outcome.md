# 035 Outcome — Energy as a Lexicase Secondary Objective

**Status**: M1 COMPLETE (2026-06-08). M2 pending (T037 bake from this build).

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

## Next: M2 (T037)
M2 energy bake (`autoc-tracker.ini`) from this same build — the lottery-prone mode and the real test
of whether energy-as-lexicase generalizes. Compare vs the pinned M2 tracking-only baseline
(032-phase1). Append the M2 verdict here.
