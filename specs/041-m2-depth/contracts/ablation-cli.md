# Contract — genome ablation tool

**New target**: `tools/nn_ablate.cc`. **Scope held deliberately narrow**: input-column masking only.

**Why this exists**: 041 has **no control bake** — with PRNG, model and fitness all moving between runs, a
cross-run delta measures everything at once. Ablation is therefore the *only rigorous instrument in the
feature*: same build, same weights, same scenarios, same seeds, one variable.

**What it can and cannot answer** — stated plainly, because the limit is load-bearing:

- ✅ Does the **trained policy depend** on this input?
- ❌ What would training have produced **without** it? *(That needs a control bake, deliberately not bought.)*

---

## Interface

```text
nn_ablate -i <ini> --genome <dmp-key|weights-file> [--zero-input NAME[,NAME...]] [--out <csv>]
```

| flag | meaning |
|---|---|
| `-i <ini>` | config file. `-i` is the project convention; `--config` is the back-compat alias ([[feedback_cli_config_flag_convention]]) |
| `--genome` | an S3 dmp key or a local `nn_weights*.dat` |
| `--zero-input` | comma-separated **slot names** from the metadata tables (e.g. `IN_ENVELOPE,ENVELOPE_SECS`). Empty/omitted = unablated baseline |
| `--out` | optional per-scenario CSV |

Configuration is via CLI flags, never env vars ([[feedback_cli_over_env_vars]]).

**Slot names come from the existing metadata tables** — `kPathgenInputMeta` / `kTrackerInputMeta` already
carry `{"GYRO_P","gyrP",7}`-style rows, `static_assert`-ed against `COUNT`. No new naming infrastructure is
needed; the Type-Safe-Sensor-Interface backlog item is **not** a dependency (research.md R12).

## Masking semantics

- A masked column is forced to **0.0 at every tick, after gathering, before the forward pass** — so the
  mask sees exactly what the net would have seen.
- Masking is applied identically in every scenario and every tick; no per-scenario variation.
- An unrecognised slot name is a **hard error** naming the valid set — never a silent no-op (Constitution
  VII).
- Evaluation uses the **identical scenario set and seeds** as the source run's eval path, so the comparison
  is within-build and deterministic.

## Report

| field | note |
|---|---|
| baseline fitness, ablated fitness, Δ | aggregate |
| per-axis Δ`dCtrl`, Δ`⟨\|u\|⟩` | pitch / roll / throttle — the aggressiveness read |
| Δ`pctInStreak`, Δ`avgMaxStreak` | the tracking read |
| Δ peak load, Δ mean normal load | the load read |
| per-scenario Δ distribution | so a change concentrated in a few scenarios is distinguishable from a broad one |
| **regime breakdown** | Δ per `{tracking, intercept, patrol}` — **required, not optional** (FR-011a). The hypothesis predicts a signal in *one* regime; pooling would hide exactly the expected shape |

## Verdict rule for H1a — pass / partial / fail

A **pass** requires **both**:

1. a measurable fitness drop, **and**
2. a behavioural signature — `pctInStreak` down or per-axis `dCtrl` up.

One without the other is a **partial** (a weaker, differently-meaning result — record it as such, do not
round it up). Neither is a **fail**, which closes the hypothesis just as usefully.

**Calibrate against control ablations, never an absolute threshold** (FR-011b). Run a spectrum of similar
slot count so "meaningful" has a within-run reference:

```bash
nn_ablate … --zero-input DIST_NOW                              # known-critical end
nn_ablate … --zero-input GYRO_P,GYRO_Q,GYRO_R                  # ego state
nn_ablate … --zero-input INWARD_BODY_X,INWARD_BODY_Y,INWARD_BODY_Z   # arena, plausibly marginal
nn_ablate … --zero-input IN_ENVELOPE,ENVELOPE_SECS              # the question
```

The envelope verdict is stated as a position on that spectrum.

## ⚠️ The prior M1 cannot be ablated — profile it from recorded data instead

The prior M1 genome has **37 inputs**; a 041 binary expects 42. It therefore **cannot be loaded or
evaluated at all** — the "baseline weights expire when the schema moves" trap, in a new form and fully
predictable this time.

So the prior-M1 side of any comparison comes from **Study A on its recorded dmp**
([offline-study.md](offline-study.md)), never from re-evaluation. That is sufficient for the per-regime
behavioural comparison (FR-011c), which is a profile comparison rather than a controlled delta.

## Required tests

| test | asserts |
|---|---|
| **empty-mask identity (SC-004)** | with no mask, reproduces the source run's fitness **exactly** — validates the instrument before it is trusted. A tool that quietly perturbs the eval path would make every finding worthless |
| unknown slot name | hard error listing valid names |
| known-load-bearing input | masking an input the policy must use (e.g. a target-bearing channel) degrades fitness measurably — proves the mask is actually applied |
| determinism | two identical invocations produce bit-identical output |

## Out of scope for this feature

`--zero-whh` (recurrence ablation), `--zero-layer`, weight perturbation, partial zeroing, targeted
permutation. All are recorded in `specs/BACKLOG.md`; add them when a research question needs them, not
speculatively.

## Primary use in 041

```bash
# H1a — does the learned policy USE the envelope state?
nn_ablate -i autoc-eval.ini --genome <new-M1-elite> --zero-input IN_ENVELOPE,ENVELOPE_SECS

# and the load observation
nn_ablate -i autoc-eval.ini --genome <new-M1-elite> --zero-input ACCEL_X,ACCEL_Y,ACCEL_Z
```

A decisive answer either way closes the hypothesis: degradation means the state is load-bearing; no change
means the net ignored it and H1 is dead for the cost of an eval rather than a bake.
