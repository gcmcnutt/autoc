# Phase 0 Research — 035 Energy Lexicase Objective

Resolves the unknowns from plan Technical Context. Each item: **Decision / Rationale /
Alternatives considered.** Clarification-session decisions (spec §Clarifications) are treated as
fixed inputs and only their *implementation* is researched here.

---

## R1 — Convex energy metric `f(out_th)` form (FR-001b)

**Decision:** v1 `energy_score` per scenario = `Σ_t f(thr_t)` where `thr_t = (out_th + 1)/2`
(map NN tanh output `out_th ∈ [−1,1]` to throttle fraction `∈ [0,1]`) and
**`f(thr) = thr²`** (quadratic, the simplest convex super-linear form). Accumulated in
`gp_fitness`. Lower = better. No per-tick normalization beyond the existing tick count (keep
parity with how `score` accumulates so axis magnitudes are comparable for MAD).

**Rationale:** The clarify decision was "convex super-linear function of throttle command,
modeled (no ESC current monitor yet)." Quadratic is the minimal honest convex shape: it charges
disproportionately for high throttle (super-linear motor draw) while a cruise throttle costs
little, and it has no free tuning parameter to over-fit. Induced-drag cost from aggressive
maneuvering is charged *implicitly* by the FDM (FR-009), so `f` only needs to model the
direct-throttle term. The `(out_th+1)/2` remap fixes a latent defect in the old
`Σ(out_th−1)/2` proxy, which was **negative** (range [−1,0]) and rewarded high throttle — a sign
that the placeholder was never a real "minimize energy" term.

**Alternatives considered:**
- Keep linear `Σ(out_th−1)/2` — rejected by clarify (placeholder); also sign-wrong.
- Cubic / `thr^k` with tunable k — deferred; introduces a knob with no measured curve to fit.
  Revisit when the ESC current-monitor calibration follow-on lands (then fit k to the real
  power curve).
- FDM shaft/electrical power integral — checked: LaRCSim does not cleanly expose instantaneous
  electrical power at the autoc boundary without new plumbing; deferred to the calibration
  follow-on. Quadratic-of-command is the pragmatic stand-in.
- `Δ(PE+KE)+work_in` mechanical accounting — explicitly the FR-007 richer-energy go/no-go, not v1.

**Open for the bake:** whether quadratic is "convex enough" is itself part of the verdict; if
energy is unmoved with quadratic, the go/no-go (FR-007) considers a steeper `f` before declaring
energy-as-lexicase a failure.

---

## R2 — MAD-relative lexicase epsilon (FR-003)

**Decision:** Replace the constant `epsilon_floor = 0.5` with a **per-axis, per-selection-round
MAD-relative epsilon**: for each test case (scenario × field), ε = `MAD_pop(field_values over
current candidates)` where MAD = median(|x_i − median(x)|). Use the MAD computed over the
candidate set still alive at that test case (standard ε-lexicase). Retain the constant-0.5 path
behind a new ini field **`LexicaseEpsilonMode = mad | constant`** (default `mad`; `constant`
reproduces prior runs bit-for-bit).

**Rationale:** [project_lexicase_mad_epsilon] — constant ε=0.5 doesn't scale: path-5 long-track
spread ≈0.3%, early-gen crash spread ≈100%, and the energy axis + craft variations widen the
spread further, so a fixed absolute floor either passes everyone (tiny-spread axis) or no one
(huge-spread axis). MAD is the textbook ε-lexicase scale (Spector et al.), robust to outliers
(unlike std-dev), and makes each axis self-calibrating. Per-axis is required because tracking
`score`, `energy_score` magnitudes differ by orders of magnitude.

**Alternatives considered:**
- Std-dev-relative — rejected: non-robust to the crash-outlier tail that dominates early gens.
- Global single ε across axes — rejected: defeats the purpose when axis magnitudes differ.
- MAD over the *whole population* (not surviving candidates) — simpler/cheaper but deviates from
  canonical ε-lexicase; chose surviving-candidate MAD for correctness, revisit if it costs too
  much per-gen.

**Determinism:** MAD is computed from already-deterministic per-scenario scores; median/MAD are
order-independent (FP-stable given the same input set). The `constant` switch guarantees
SC-003 bit-exact reproduction of historical runs.

---

## R3 — rebuild-perf replay gate basis (FR-P04)

**Decision:** The bit-replay regression gate compares the **per-scenario `ScenarioScore` vector
byte-for-byte** (the `computeScenarioScores(evalResults)` output), NOT whole-dmp bytes.

**Rationale:** `stampEvalResultsProvenance()` (`src/autoc.cc:1435`) writes provenance timestamps
into the dmp, and the `.zst` container adds compression framing — both make whole-dmp bytes
non-deterministic across runs. The load-bearing invariant is *fitness reproducibility*
([reference_perf_build_reproducibility]: eval-vs-training exact match), which lives in the
`ScenarioScore` numbers, not the serialized envelope. Comparing the score vector preserves the
non-negotiable bit-replay property while tolerating the (irrelevant) provenance/compression
noise. The operator drives this gate ([feedback_operator_runs_regression_gate]).

**Alternatives considered:**
- Whole-dmp byte equality — impossible post-provenance-stamp + zstd.
- Decompress-then-compare-dmp-bytes — still fails on the provenance timestamps; would require
  zeroing them, more fragile than comparing the scores directly.
- Keep data.dat byte equality — rejected: data.dat is being retired (FR-P05), the whole point.

---

## R4 — zstd level + container layout (FR-P09)

**Decision:** Compress the cereal binary blob with **zstd level 19** (one-shot
`ZSTD_compress`), wrapped so the S3 object is a self-contained `.zst` stream. Store as
`gen<N>.dmp.zst`. Read path: if key ends `.zst`, `ZSTD_decompress` into the cereal input buffer;
else (legacy `.dmp`) feed bytes directly. Confirm round-trip byte-equality of the
*decompressed* blob against the pre-compression blob in a unit test before trusting it.

**Rationale:** LETTER §4 measured gzip 2.07× / xz 3.29× on a real 40 MB dump; zstd-19 lands near
xz ratio at far higher throughput. Training writes one elite dmp per gen (not per-eval), so even
level-19 compress latency (~tens of ms on 40 MB) is negligible against a multi-minute gen. The
spike is folded into the implementation pass (clarify decision) rather than standalone: the unit
test *is* the spike (it reports ratio + round-trip), and the level is a single
`#define`/config constant easily retuned if the measured ratio disappoints.

**Alternatives considered:**
- Streaming zstd (ZSTD_CCtx) — unnecessary; the blob is already fully in memory as an
  `ostringstream`.
- Level 10 (faster) — keep as the fallback if level-19 compress ever shows on a gen profile;
  exposed as a constant.
- Compress at the AWS-SDK transfer layer — rejected: opaque, not portable to local-file dmps or
  the `dmp-dump` read path.

---

## R5 — `dmp-dump` output format (FR-P02)

**Decision:** `dmp-dump` emits a single stream with two segregated blocks:
1. **YAML metadata header** (document 1): run provenance (gp hash, mode, scenario count, seeds
   from `scenarioMetaList`), `crashReasonList`, per-scenario aggregates, arena/camera config.
2. **CSV per-tick body** (after a `--- ` separator / second YAML doc boundary): one row per
   `(scenario, tick)`, columns = `AircraftState` fields (position, quat, velocity,
   pitchCommand/rollCommand/throttleCommand, NN outputs) **plus recomputed derived columns**
   (`dhome, dist, along, stpPt, mult, rampSc`, and `hull` in tracker mode).

The derived columns are **recomputed in the dumper** from dmp content (FR-P01 default), matching
the inline math in the retired `logEvalResults*` writers (`src/autoc.cc` ~954–1052 pathgen,
~815–879 tracker). Input: S3 URI (primary) or local path; auto-inflates `.zst`.

**Rationale:** CSV-for-timeseries + YAML-for-metadata matches how the Python plot scripts already
think (pandas `read_csv` for per-tick, dict for run meta) and keeps the two cleanly parseable.
Recompute (vs storing the derived columns in the dmp) avoids bloating every elite dmp with
columns trivially derivable from position + path — the FR-P01 default. The recompute math is
self-contained geometry (offset norms, dot with tangent) + `FitnessComputer` calls, all
reconstructable from `aircraftStateList` + `pathList` + config.

**Alternatives considered:**
- Add derived columns to the dmp schema — rejected per FR-P01 default (recompute unless coupling
  forces storage); these are pure functions of stored state.
- Pure-CSV (metadata as comment rows) — rejected: metadata is nested (per-scenario lists);
  YAML carries it cleanly.
- Python-native cereal reader — explicitly out of scope (spec); the CLI text bridge suffices.

**Coupling check:** `stpPt`/`mult` call `FitnessComputer::computeStepScore` + streak multiplier
— the dumper links `autoc_common` (where FitnessComputer lives), so the math is shared, not
re-implemented (no drift risk). `rampSc` = `computeVariationScale()` needs the gen number, which
is recoverable from the dmp key (`10000−N`) — pass it in or read from `scenarioMetaList`.

---

## R6 — Bucket migration sequencing (FR-P07/P08, admin split)

**Decision:** Code lands selector + uniform naming + tag-on-upload + zstd **first**, all still
pointing at existing buckets via ini. Bucket cutover is a separate, admin-gated step:
1. (Admin) create `autoc-m1` / `autoc-m2` / `autoc-eval`; grant `s3:PutObjectTagging` +
   `s3:GetObjectTagging` to `autoc-generator`; apply `lifecycle-policy.json` to each.
2. (Code/ini) flip `S3Bucket` per ini to the new names; retire the `tracker-` prefix.
3. Legacy `autoc-storage` keeps its lifecycle until it drains; pinned milestones re-tagged
   `retain=keep` first (LETTER §1 table).

**Rationale:** `autoc-generator` lacks `s3:CreateBucket`/bucket-policy perms (LETTER §2), so
bucket+lifecycle+IAM are unavoidably an admin role. Decoupling code from cutover lets the
verification gate run on *either* bucket set — the selector is bucket-relative, so the gate is
valid before the rename and the rename becomes a config-only change. Avoids a code/infra
deadlock.

**Alternatives considered:**
- Do everything atomically — rejected: blocks all code progress on an admin action.
- Have autoc auto-create buckets — rejected: IAM scope + Principle (least privilege); admin owns
  bucket lifecycle.

---

## R7 — FR-002 physical axis grouping with stability OFF

**Decision:** For v1 (stability off, energy + tracking only), FR-002's "bank = pitch+roll
together, throttle/energy separate" grouping is **satisfied trivially**: `energy_score` is a
throttle-domain axis already distinct from the tracking `score`, and there is no pitch/roll
control-amplitude axis present to group. Implement energy as its own pool entry alongside
`score`; record that the bank-grouping provision is **latent** — it activates only if/when a
stability/smoothness axis is re-enabled (the FR-008 follow-on).

**Rationale:** Axis grouping ([project_smoothness_axis_grouping]) matters when multiple
control-rate axes (pitch, roll) would otherwise each get an independent lexicase vote and
fragment the bank decision. With stability off there are no such axes, so grouping is a no-op
for v1. Documenting it prevents a future reader from thinking grouping was forgotten.

**Alternatives considered:** Pre-build the grouping machinery now — rejected as speculative
(YAGNI until stability returns); the pool structure (`selection.cc`) makes adding a grouped axis
later straightforward.

---

## R8 — Energy axis from gen 0 (timing, clarify-decided) — implementation note

**Decision:** Uncomment `pool.push_back({s, &ScenarioScore::energy_score, …})` unconditionally
(no gen guard). The MAD epsilon (R2) replaces the literal `0.5`.

**Rationale:** Clarify decision: gen-0, no ramp ("ramp = tuning"). Simplest implementation; no
schedule plumbing. If gen-0 energy suppresses exploration, that is a recorded negative result
(spec), not a knob.

---

## Summary of resolved decisions

| Unknown | Decision |
|---|---|
| Convex `f()` | `thr²` with `thr=(out_th+1)/2`, `gp_fitness` |
| MAD epsilon | per-axis MAD over surviving candidates; `LexicaseEpsilonMode` ini switch (mad default) |
| rebuild-perf gate | per-scenario `ScenarioScore` byte equality (dmp non-deterministic) |
| zstd | level 19 one-shot, `.dmp.zst`, legacy `.dmp` read-accepted, unit-test round-trip |
| dmp-dump | YAML meta + CSV per-tick; derived columns recomputed; S3-primary, `.zst`-aware |
| bucket migration | code first (bucket-relative), admin creates buckets+IAM+lifecycle, then ini flip |
| FR-002 grouping | trivially satisfied (energy throttle-separate); bank-grouping latent |
| energy timing | gen-0, unconditional pool entry |
