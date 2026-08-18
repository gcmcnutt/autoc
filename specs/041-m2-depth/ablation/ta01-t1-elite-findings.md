# TA01 + TA03 — findings on the t1 gen-608 elite

**2026-08-17.** Both zero-bake research tasks, run against the pinned t1 elite
(`autoc-m1/autoc-9223370249927095135-2026-08-17T00:48:00.672Z/` gen 608, seed 1786927680).

**Instrument validated first**: baseline eval reproduces the stored fitness **exactly** —
`-23651.291732` vs `-23651.291732` across 294 scenarios (SC-004). Config
`ablation/autoc-eval-m1-t1.ini` is derived from `autoc.ini` **by override**, so every
scenario/variation/fitness knob is what t1 trained under by construction, not by inspection.

## Path focus

Operator 2026-08-17: *"focus the analysis on path 6 — this is really the random all attitude paths which
will be actual field work."* That is `aeroStandard` index **5 = `SeededRandomB`** (random control points in
the cylinder, cubic-interpolated), scenarios **245–293**. Reported separately below throughout.

## TA01 — the ablation matrix

More negative fitness is better; Δ% is the loss when a slot is zeroed.

| ablation | all paths | **path 5** | note |
|---|---:|---:|---|
| `DIST_NOW` | −90.2% | **−98.9%** | the known-critical end of the ruler |
| `GYRO_P/Q/R` | −51.2% | −35.0% | |
| **`DIST_TO_BOUNDARY`** | **−40.7%** | **−25.0%** | ⭐ third most important input |
| `INWARD_BODY_*` | −8.3% | −3.9% | |
| `ACCEL_Y` | −2.7% | **−4.5%** | worse than all three accel axes together |
| `ACCEL_X/Y/Z` | −3.0% | −3.5% | |
| `IN_ENVELOPE`+`ENVELOPE_SECS` | −1.7% | −0.7% | |
| `ENVELOPE_SECS` | −2.4% | −0.2% | |
| `IN_ENVELOPE` | −1.9% | **+0.3%** | removing it is *better* on path 5 |

### H1a verdict: **FAIL on the field-relevant path**

On path 5, zeroing `IN_ENVELOPE` **improves** score by 0.3% and `ENVELOPE_SECS` costs 0.2% — both inside
noise. Pooled across all paths the pair costs 1.7%, the mildest arm in the matrix. Per FR-014b/T070 that is
a **fail**: no fitness drop, no behavioural shift. The envelope inputs did not earn their place in M1.

⚠️ A fail is a successful outcome (SC-012). It cost one eval, not a bake, and it is *decisive* precisely
because the run was stopped — the elite was already pinned.

### ⚠️ THE CONTRIBUTION SCREEN WAS WRONG — THREE TIMES

Contribution (relative first-layer weight × input std) ranked `IN_ENVELOPE` **2nd of all measurable
inputs**, `DIST_TO_BOUNDARY` near the **bottom** (0.050), and `ACCEL_Y` **last** (0.014). The ablation says
`IN_ENVELOPE` is worthless on path 5, `DIST_TO_BOUNDARY` is the **third most important input in the
vector**, and `ACCEL_Y` matters *more* on path 5 than the whole accel group.

**Do not use weight-based screens to decide what to drop.** They mis-rank in both directions:
- *understate* limit-class inputs whose value is concentrated in rare states (`DIST_TO_BOUNDARY` is >0.95
  for 92.8% of ticks — inert until it is decisive);
- *overstate* inputs the net has weighted but does not act on (`IN_ENVELOPE`).

The operator's instinct to retain the boundary input was right, against my analysis. FR-033 already
retained it; this is the evidence.

### ⚠️ Ablation is non-monotonic — single-slot arms can mislead

`IN_ENVELOPE` alone costs 1.9%, `ENVELOPE_SECS` alone 2.4%, but **both together only 1.7%**. And `ACCEL_Y`
alone (−4.5% on path 5) costs *more* than `ACCEL_X/Y/Z` together (−3.5%). The network compensates through
correlated inputs, so removing more can hurt less. Any future trim must ablate the **candidate set it
intends to remove**, not sum single-slot results.

## TA03 — energy, per path

`Es = h + v²/2g`, `Ps = dEs/dt`. ⚠️ Note `Ps` needs **no FDM internals** — it is the time-derivative of a
state quantity, so it is computable on every dmp ever recorded.

| path | nz med | nz max | Ps med | losing energy | Ps penalty at high-g |
|---|---:|---:|---:|---:|---:|
| all | 3.18 | 11.13 | 1.61 | 43.6% | 4.23 m/s |
| **5 (SeededRandomB)** | **3.48** | **11.13** | 1.89 | 43.6% | **5.19 m/s** |
| 1 (gentlest) | 2.85 | 9.64 | 2.57 | 38.8% | 2.61 m/s |

**Path 5 is both the most aggressive and the most energy-expensive** — highest sustained load and the
largest specific-excess-power penalty when manoeuvring. The field-relevant path is the worst case on both
axes, which is the right place to aim an energy objective.

### SC-014: `Ps` discriminates, decisively

- `corr(Ps, closure rate) = −0.048` — **essentially zero**. `Ps` carries information the current objective
  is entirely blind to.
- At **every** matched-closure bin, `Ps` still spans **24–32 m/s** p10–p90.

The same task progress is bought at wildly different energy cost, and nothing scores it. This also explains
why absolute-throttle penalties failed: throttle is pinned at 0.99 with almost no variance to select on,
while `Ps` has 25+ m/s of spread *at matched progress*.

## ⭐ Why every previous energy objective "muted the entire regiment"

**The network cannot observe its own energy state.** The 42 pathgen inputs include `AIRSPEED` but **no
altitude, height, or AGL term of any kind**. `Es = h + v²/2g`: the policy has the `v²` half and has never
had the `h` half.

035 added an energy *objective* without an energy *observation*. Asked to reduce a quantity it cannot see,
the only lever a policy has is **reduce output everywhere** — which is exactly the observed muting. This is
not a tuning failure; it is an observability failure, and no amount of reweighting would have fixed it.

## What this implies for the fitness vector vs the inputs

Operator: *"how does this fit into fitness vector for the evolutionary side? and or should some of this be
RNN feedback signal?"* — **both, and neither alone works:**

- **Input only**: the policy could see energy but nothing rewards managing it. `Ps` correlates ~0 with the
  current objective, so the objective will never select for it. Inert.
- **Axis only**: this is what 035 did. Optimising an unobservable → muting.
- **Both**: observation makes it actionable, the axis makes it selected-for.

**For the axis**: `Ps` being *orthogonal* to progress (r = −0.048) is the ideal case for **lexicase** —
independent axes give complementary selection pressure rather than the Pareto-corner collapse that
[project_scalar_multiobjective_collapse](../../../.claude/projects/-home-gmcnutt-autoc/memory/project_scalar_multiobjective_collapse.md)
recorded for *scalar-aggregated* smoothness. Add it as an axis, never as a scalar penalty term.

**For the input**: give **`Es` (the state), not only `Ps` (the rate)**. `Es` is the integral of `Ps`;
handing the recurrent layer the rate and asking it to accumulate wastes capacity it is already not filling
(effective rank 11.3 of 16). Provide altitude (or `Es` directly) and the network can compute the rest.

## Consequences for Phase A′

- **TA11** (input trim): `IN_ENVELOPE` and `ENVELOPE_SECS` are now candidates for **removal** on evidence,
  not suspicion — pending confirmation that the reshape (TA09, score gradient × multiplier) is a better use
  of those slots than deletion. **`ACCEL_Y` is RETAINED** — the proposal to drop it was wrong; it is the
  most impactful accel axis on the field path.
- **New**: add an **altitude / `Es` input**. This is the missing observation, and it belongs in the same
  format break as TA04/TA05.
- **TA10** (step cost vs `Ps_max`) is now evidenced by TA03 rather than assumed.
