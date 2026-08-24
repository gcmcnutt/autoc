# Ablation instrument — validation run (T049/T052)

**2026-08-13.** Validating the instrument BEFORE trusting it, per the contract: *"a tool that quietly
perturbs the eval path would make every finding worthless."*

⚠️ **These numbers are NOT the H1a verdict.** The genome here is the **041 M1 smoke** elite (gen 320 of a
pop-3000 / single-longSequential diagnostic), not the production M1 bake. T070's verdict must come from
T063. This page validates the *instrument*; the spectrum below is a shakedown of it.

## Required tests — all pass

| test | result |
|---|---|
| **empty-mask identity (SC-004)** | eval with the mask code present but unset reproduces the source fitness **exactly**: `-1045.136851` vs stored `-1045.136851` |
| unknown slot name | hard error listing valid names (`GYROP` → rejected, message names `GYRO_P`) |
| known-load-bearing input degrades | `DIST_NOW` → `-413.06`, a 60% loss |
| determinism | two identical invocations → identical to all printed digits |
| wrong-length mask | hard error at the setter, both short and long |
| off-by-one in name→index | slot-by-slot assertion that ONLY the named slots are zeroed |

## Calibration spectrum (FR-011b)

Baseline `-1045.137`. **More negative is better**, so a smaller magnitude is a worse controller.

| ablation | fitness | Δ | retained |
|---|---:|---:|---:|
| *(baseline)* | −1045.14 | — | 100% |
| `GYRO_P,GYRO_Q,GYRO_R` | −106.74 | −938.4 | **10.2%** |
| `DIST_NOW` | −413.06 | −632.1 | 39.5% |
| `INWARD_BODY_X/Y/Z` | −851.90 | −193.2 | 81.5% |
| `ACCEL_X/Y/Z` | −863.50 | −181.6 | 82.6% |
| `IN_ENVELOPE,ENVELOPE_SECS` | −972.33 | −72.8 | **93.0%** |

**Reading it.** The spectrum spans an order of magnitude, which is exactly what makes it a usable ruler:
gyro is catastrophic (the controller is essentially destroyed without body rates), `DIST_NOW` severe, arena
and accel moderate, and the envelope pair mildest. On *this* genome the envelope sits at the **marginal
end** — but it is **not zero** (−72.8, ~7%), so the policy is not ignoring the channels outright.

⚠️ Do not read that as H1a failing. This genome saw the envelope inputs for 320 generations on a small
diagnostic config; the production bake is 8000/49-wind for 800. The instrument is what is being certified
here, and it is: it reproduces the baseline exactly, degrades measurably when it should, and repeats.

## Reproduce

```bash
scripts/train.sh autoc-basic-m1-eval.ini logs/ablate-baseline.log
scripts/train.sh autoc-basic-m1-eval.ini logs/ablate-envelope.log --zero-input IN_ENVELOPE,ENVELOPE_SECS
```

`scripts/train.sh` forwards any argument after `<logfile>` to `autoc`, so an ablation launches through the
same detached path (Constitution IX) as every other run rather than being invoked by hand beside it.
