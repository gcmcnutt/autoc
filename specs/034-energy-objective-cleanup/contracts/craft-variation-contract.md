# Contract: Craft Variation (US4)

**Surface**: per-scenario airframe perturbation applied to the CRRCSim-simulated NN-controlled craft (the "chase") in BOTH pathgen and tracker modes.

## Inputs (ini → AutocConfig)

| Key (`[Craft]`) | Type | Meaning | Default |
|---|---|---|---|
| `CraftCGSigma` | fractional σ | Gaussian σ for CG perturbation | `0.0` (no-op) |
| `CraftDragSigma` | fractional σ | σ for profile-drag perturbation | `0.0` |
| `CraftTrimSigma` | fractional σ | σ for pitch-trim perturbation | `0.0` |
| `CraftThrustSigma` | fractional σ | σ for max-thrust scale | `0.0` |
| `CraftPitchEffSigma` | fractional σ | σ for pitch control effectiveness (`Cm_de`/`CL_de`) | `0.0` |
| `CraftRollEffSigma` | fractional σ | σ for roll control effectiveness (independent → asymmetry) | `0.0` |

*(Servo-lag time-constant deferred per D1 — not in 034.)*

- Magnitude = **fractional Gaussian σ** (multiplicative on each param's nominal). σ=0 → exactly nominal.
- **Ramping**: drawn deltas stored at full magnitude in `ScenarioMetadata`; **per-eval `applyVariationScale()` scales them** (additive deltas toward 0, `craftThrustScale` toward 1.0 — same pattern as `entryHeadingOffset` / `entrySpeedFactor`). Training: scale = `computeVariationScale(gen)`. Eval: scale = `genome.variation_scale` saved in weight file (replays training-time scale exactly via `gEvalVariationScaleOverride`).

## Sampling (autoc-side, `craft_variation.{h,cc}`)

- Draw from `deriveClassSubSeeds(scenarioSeed).craft` → deterministic per scenario.
- Each param: `delta_i = gaussian(0, sigma_i)` → realized as additive (CG, trim, pitch-eff, roll-eff, drag) or multiplicative (thrust) at the FDM site.
- Output: populate `ScenarioMetadata.{craftCGDelta, craftDragDelta, craftTrimDelta, craftThrustScale, craftPitchEffDelta, craftRollEffDelta, craftSeed}` at full magnitude (scale=1.0). Per-eval scaling happens later via `applyVariationScale()`.
- All `gp_scalar` (Constitution VI).

## Scaling (worker-side, extends `applyVariationScale()`)

Same call as for entry/wind — workers call `applyVariationScale(meta, evalData.variationScale)` once before each eval. The function scales each craft delta:

```cpp
meta.craftCGDelta *= scale;
meta.craftDragDelta *= scale;
meta.craftTrimDelta *= scale;
meta.craftPitchEffDelta *= scale;
meta.craftRollEffDelta *= scale;
meta.craftThrustScale = 1.0 + scale * (meta.craftThrustScale - 1.0);  // interpolate toward 1.0
```

## Application (crrcsim-side)

Per-scenario reset hook (`inputdev_autoc.cpp:494-520` → `Global::craft*` → `initAirplaneState()` / `engine()`):

| Param | FDM target | Operation |
|---|---|---|
| CG | `CG_arm` (`fdm_larcsim.h:193`) | `CG_arm = nominal + craftCGDelta` |
| Drag | `CD_prof` (`fdm_larcsim.h:164`) | `CD_prof = nominal * (1 + craftDragDelta)` |
| Trim | `Cm_0` (`fdm_larcsim.h:160`) | `Cm_0 = nominal + craftTrimDelta` |
| Thrust | `engine()` (`fdm_larcsim.cpp:220`) | `maxThrust *= craftThrustScale` (new multiplier) |
| Pitch eff | `Cm_de`/`CL_de` (`fdm_larcsim.cpp:309,317`) | `Cm_de *= (1 + craftPitchEffDelta)` (settable scalar at init) |
| Roll eff | aileron→roll coeff (verify name in `fdm_larcsim`) | `*= (1 + craftRollEffDelta)`; independent of pitch |
| ~~Servo lag~~ | — | DEFERRED (D1) — new dynamical element, follow-on |

## Invariants (test assertions)

1. **No-op**: all σ=0 ⇒ per-scenario fitness byte-identical to nominal (FR-021, SC-004).
2. **Determinism (draw)**: same `scenarioSeed` ⇒ identical *drawn* deltas in `ScenarioMetadata` bit-for-bit (FR-018, SC-005).
3. **Replay gate**: bit-exact M1→M1 replay (validated in 033) survives craft-seed cascade addition (SC-005).
4. **Ramp (training)**: applied delta at gen 1 ≈ `0 * drawn`; applied delta at late gen ≈ `1 * drawn`. The drawn delta itself is identical across gens for a fixed seed.
5. **Ramp (eval)**: eval mode replays `genome.variation_scale` exactly (via `gEvalVariationScaleOverride`); applied craft delta matches the gen-of-save scale bit-for-bit, NOT recomputed and NOT 100%.
6. **Reproducible draw**: `craftSeed` round-trips through dmp `ScenarioMetadata` (FR-020).
7. **Camera-seed forward-compat**: appending a future `cameraSeed` requires no other schema change.
