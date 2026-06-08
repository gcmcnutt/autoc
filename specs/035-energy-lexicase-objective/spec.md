# Feature Specification: Energy as a Lexicase Secondary Objective

**Feature Branch**: `035-energy-lexicase-objective` (to be created when this feature is picked up)
**Created**: 2026-05-29
**Status**: Draft (split out of 034 on 2026-05-29 — operator routing)
**Input**: Split from `034` US4. 034 ("M1 cleanup + craft variations") delivers a clean, honest M1/M2 codebase with airframe-parameter diversity, up to and including a flight test. 035 then takes up the substantive multi-objective question 034 deliberately left alone: **does energy work as a real lexicase secondary objective?**

## Context

033 wrapped with the finding that a scalar-aggregated multi-objective collapses the controller into one Pareto corner (`project_scalar_multiobjective_collapse`). The intended secondary objective was always **energy minimization** (lower throttle / better energy management), and energy is already plumbed: `ScenarioScore::energy_score` is computed + emitted, and its lexicase test-case pool entry is one commented line at `src/eval/selection.cc:69` (with the sibling `stability_score` axis at `:68`).

034 was originally going to carry this, but the operator split it out so 034 can ship variations + cleanup + a flight test without entangling them with the open multi-objective investigation. 035 is **genuine investigation, not a one-line uncomment** — energy-as-lexicase-axis underperformed in 027/028, so the deliverable is a *determination* (it works, with evidence / it collapses tracking, with the failure mode / it's unmoved), not a guaranteed win.

035 also inherits the **MAD-relative lexicase epsilon** change (`project_lexicase_mad_epsilon`): constant ε=0.5 doesn't scale with per-scenario magnitude, and both the energy axis and 034's craft-variation dimensions widen that magnitude spread, so epsilon semantics are part of making any lexicase multi-axis result honest.

**Dependency:** 035 builds on the 034-delivered baseline (minisim gone, smoothness gone, craft variations in, tech-debt fold-ins done). The energy bake is compared against the 033/034 tracking-only baseline.

## Clarifications

### Session 2026-06-04

- Q: v1 `energy_score` metric form — linear throttle proxy vs non-linear/consumed-energy integral? → A: **B** — convex (non-linear) function of throttle command integrated over time, `Σ_t f(out_th)` with `f` super-linear; induced drag captured implicitly by the FDM. Aspiration: calibrate `f()` against a **measured power curve off the actual craft**, but the current ESC has **no current monitor**, so v1 ships a modeled convex shape and the measured-curve calibration is a follow-on once instrumented.
- Q: hull-crash-cost — fold into 035 now, or defer? → A: **Defer to a separate future feature** (out of 035 scope; see BACKLOG "Hull-crash-cost as a lexicase fitness dimension"). 035 stays focused on energy, which must be solved (or at least help smoothness) for M1/M2/beyond. Design constraints recorded for the future spec: the crash penalty must NOT merely zero/stop the score; it can't be a flat scalar (an early crash is worse than a late one — any crash is bad, but timing must register); and the core hard problem is **credit assignment** — a single individual that crashes in only ~1 of ~300 scenarios barely differentiates from clean siblings under lexicase, so a naive per-scenario `crash_cost` gets drowned out.
- Q: also re-enable the `stability_score` lexicase axis alongside energy? → A: **No — OFF for v1.** `stability_score` is a control-amplitude/bang-bang penalty (the control-rate term FR-009 prohibits), and enabling it would confound the energy verdict. Re-enable only as a follow-on if energy alone leaves the controller too aggressive.
- Q: ramp energy pressure in late, or apply from gen 0? → A: **From gen 0, no ramp.** A late ramp is tuning; the first-principles "stay on track with minimum energy" approach should work on its own, and a ramp would add a timing confound. If gen-0 energy suppresses early exploration, that's a documented negative result, not something to pre-empt with a knob.
- Q: energy investigation scope — M1 only, or M1 + M2? → A: **Both M1 and M2** (FR-005 + FR-005b). The objective is identical in both modes — on-point tracking with minimum energy — and energy is a characteristic of the controller, so an M1 energy win is implicitly needed for M2; a bang-bang M2 doesn't help. M2 also produces the hull-strike telemetry for the future crash feature.

## Prerequisites (pre-035, before US1)

### Retire data.dat — S3 dmps become the sole per-run trace

**Rationale:** Today every training run writes a multi-GB `data.dat` (text per-tick trace of the elite) alongside an S3 dmp per gen. The dmp is a cereal-serialized `EvalResults` containing essentially the same elite trajectory (`aircraftStateList` per-tick, `pathList`, `scenarioMetaList`, `crashReasonList`, tracker-mode camera/target lists). S3 dmps are already unique per run and persist longer than the workspace `data.dat`. Maintaining the text duplicate adds I/O cost, disk pressure, and a divergence surface (every schema bump must update both writers).

The 034-era trade ("keep data.dat for Python analysis tools, it streams cheaply") expires now that 035 introduces new per-scenario axes (energy_score, MAD-relative epsilon stats) that would otherwise need to be added to *both* writers. Single source of truth wins.

**Approach:** drop the data.dat writer entirely; add any data.dat-only columns to the dmp schema (greenfield, no cereal version bump per project policy); build a `dmp-dump` CLI that reads a `.dmp` (from local file or S3 stdin) and emits the same text-column format the Python plot scripts already consume. Python tools invoke the CLI (`dmp-dump < x.dmp | python3 plot_…py`) instead of streaming data.dat directly.

**Functional Requirements (pre-035):**

- **FR-P01**: `EvalResults` MUST carry every data.dat field that isn't trivially recomputable from `aircraftStateList[scenario][tick]` + `pathList`. Audit columns: NN inputs/outputs/position/quat/body-velocity/PID-internals/rabbit-speed are already in `AircraftState`; the **derived group** (dhome, dist, along, stepPoints, mult, rampSc) is NOT — decide per-column whether to (a) add to AircraftState/ScenarioMetadata as stored state or (b) recompute in the dumper from existing dmp content. Default: recompute unless the path-following math is too coupled to autoc internals.
- **FR-P02**: A new CLI tool MUST fetch a `.dmp` directly from S3 (given an S3 URI or bucket+key) and stream a semi-human-readable, easily-parseable representation to stdout. Format SHOULD be CSV for per-tick time-series rows (one row per scenario × tick, columns named for `aircraftStateList` fields + derived dhome/dist/along/etc.) and YAML for the per-run/per-scenario metadata block (`scenarioMetaList`, `crashReasonList`, gp hash, provenance), with a header that segregates the two. Local-file input MAY be supported as a developer convenience but S3 is the primary mode (since S3 is the authoritative per-run trace post-data.dat retirement).
- **FR-P03**: The Python analysis scripts in `specs/03[2-5]*/*.py` **and the per-axis aggressiveness script `specs/029-no-future-arch/plot_per_axis_time_series.py`** (the verification-gate consumer and the FR-005 energy comparator) MUST be updated to consume the dumper's CSV/YAML output (subprocess invocation or piped stdin) — no direct data.dat dependency remaining, and no byte-compat with the legacy `data.dat` format is required (the scripts get rewritten to use the new column names directly).
- **FR-P04**: The `rebuild-perf.sh` M1→M1 replay regression gate MUST swap from `data.dat`-byte equality to **per-scenario `ScenarioScore` byte equality** (RESOLVED 2026-06-04, research R3: dmps ARE non-deterministic — `stampEvalResultsProvenance` writes timestamps and zstd adds framing — so whole-dmp bytes cannot match; the replay invariant lives in the per-scenario score vector). The byte-exact replay property itself is non-negotiable.
- **FR-P05**: All `data.dat` plumbing MUST be removed from `src/autoc.cc` (the `fout` ofstream, `logEvalResults` writer, `strOutFile` open, pathgen + tracker header emission). Constitution III: clean cut, no dual-write.
- **FR-P06**: The `.gitignore` `*.dat` rule, `include/autoc/eval/eval_logger.h` comment, and any spec/doc reference to `data.dat` as a live artifact MUST be updated.

**Acceptance:** a short bake produces zero `data.dat` files; the Python chart tools now generate their plots from the dmp files (via `dmp-dump`), not `data.dat`; `rebuild-perf.sh` gate passes via **per-scenario-score byte equality** (not whole-dmp — dmps are non-deterministic, FR-P04).

**Out of scope (deferred to a later iteration):**
- Self-describing dmp format / cross-version compatibility — current cereal is positional; OK for now, document the limitation.
- A Python-native cereal reader — the CLI dumper as text bridge is sufficient; native Python deserialization is nice-to-have, not gating.

### S3 storage contract — per-mode buckets + normalized selector + lifecycle

**Decision (2026-06-04):** M1/M2 dmps are effectively *polymorphic* (pathgen vs
tracker `EvalResults` carry different fields), and sharing one bucket with a
run-id prefix (`autoc-`/`tracker-`, T033) broke the auto-selectors. New contract:
**one bucket per mode**, mirroring the eval split (eval already uses
`autoc-eval-arm` vs training `autoc-storage`). M1 train, M2 train, (future) M3
train each get their own bucket; **run-id naming + the selector stay as before**
(`autoc-` prefix, `SetPrefix("autoc-")`) — the *bucket* is the discriminator, so
the T033 `tracker-` prefix is retired (cosmetic at most). M3 follows the same
pattern. (Per-run config already carries `S3Bucket`, so this is a config change,
not a schema change.) Interim bucket names are ad-hoc — M1 `autoc-storage`, M2
`autoc-storage-tracker`, eval `autoc-eval-arm`. **Naming convention decided
(2026-06-04): `autoc-m1` (M1 train), `autoc-m2` (M2 train), `autoc-eval`
(catch-all eval); future M3 → `autoc-m3`.** The mode token is the discriminator;
train-vs-eval is implied (`m#` vs `eval`). Migrate to these as part of FR-P07/P08.

**Pre-035 prerequisites (do before US1):**
- **FR-P07 — normalize the S3 run-selector into one common function.** The
  "latest run / latest gen" logic is currently duplicated and hardcoded to
  `autoc-` in `tools/nnextractor.cc` (≈48,105) and `tools/renderer.cc`
  (≈1779,1898) (and any future tool). Extract a single shared helper
  (`extractGenNumber` + `findLatestRun(bucket)`) — bucket-relative, prefix-
  agnostic — and route all tools through it. (This is why the in-flight M2 needs
  a manual artifact rename — see below — to avoid a code rebuild now.) The shared
  `extractGenNumber` MUST fix the `renderer.cc` bug where it fails to invert the
  `10000−N` filename encoding (`nnextractor.cc` inverts; renderer does not), and
  MUST discover `.zst` keys (FR-P09).
- **FR-P07b — uniform run-id naming across all buckets (retire the mode prefix).**
  Per the bucket-as-discriminator contract, M1 and M2 dumps MUST use a **byte-identical**
  naming scheme — `<run-id>/gen<10000−N>.dmp.zst` — in their respective buckets; only the
  `S3Bucket` ini key differs. The filename *suffix* is already mode-independent, but
  034's `autoc::runIdPrefixForMode()` (`include/autoc/util/run_id.h:15`, called at
  `src/autoc.cc:2121`) still stamps M2 run-ids with `tracker-` and M1 with `autoc-` (a
  shared-bucket workaround, T033). Collapse it to a single uniform prefix for every mode
  (`autoc-`), retiring the `tracker-` branch; update `tests/run_id_prefix_tests.cc` and the
  run_id.h header comment accordingly. After this, the selector's `SetPrefix("autoc-")`
  matches every mode's runs in every bucket.
- **FR-P08 — S3 object lifecycle that "just works," set on ALL buckets.** A
  retention/expiration policy (training + eval + per-mode buckets) so old dmps
  auto-clean without manual pruning. Apply uniformly to `autoc-m1`, `autoc-m2`,
  `autoc-eval`, the legacy `autoc-storage` (until it drains), and future
  `autoc-m3`. Concrete policy mechanics in FR-P11.
- **Manual S3 admin setup (pre-035, done by an admin role — NOT the
  `autoc-generator` IAM user, which lacks `s3:CreateBucket`,
  `s3:GetBucketLocation`, `s3:GetObjectTagging`):**
  1. **Create the per-mode buckets** `autoc-m1` / `autoc-m2` / `autoc-eval`
     (+ future `autoc-m3`), manually.
  2. **Set the lifecycle policy** (FR-P08/P11) on every bucket.
  3. **Grant object tagging to `autoc-generator`** (`s3:PutObjectTagging` +
     `s3:GetObjectTagging`, LETTER §2) so retention tags can be set on upload and
     managed multipart copies work normally (no `--copy-props none` workaround).
  4. **Update the ini files** (`S3Bucket` per mode) to point at the new buckets,
     going forward.
  Until that's done, M2 stays on `autoc-storage` and its `tracker-` run-ids are
  copy-renamed in place to `autoc-<id>` via the 034 rename script (with
  `--copy-props none`) so the unchanged selector finds them.

**Out of scope for the rename:** the in-flight M2 run (run-id
`tracker-9223370256301596645-2026-06-04T06:06:19.162Z` in `autoc-storage`) is a
one-off; its artifacts get copy-renamed to the M2 bucket with `autoc-` naming via
`specs/034-energy-objective-cleanup/rename_m2_artifacts_to_bucket.sh` so the
current (unchanged) selector finds them — no code rebuild.

### Compression, object tagging, retention & provenance (from LETTER-s3-retention.md, 2026-06-02)

The admin-session letter (`LETTER-s3-retention.md` at repo root) installed a tag-driven
30-day retention scheme on `autoc-storage` (which had grown to ~868 GB / ~84% of the AWS
bill) and asks the training-box side to wire the code + repo half. Folded into 035 pre-work:

- **FR-P09 — zstd compression on the dmp serialization boundary.** Compress at the
  cereal-write boundary (both `src/autoc.cc` PutObject sites ≈1448 eval / ≈1640 per-gen) and
  inflate at every read boundary (`src/eval/source_dmp_loader.cc` ≈71, plus the renderer and
  nnextractor fetch paths). Store `genN.dmp.zst`; the read path MUST still accept legacy plain
  `.dmp`. Add libzstd to CMake (already present on the box: `/usr/bin/zstd`, `libzstd.so`).
  Round-trip byte-equality is the gate. **Bundled with the data.dat retirement pass
  (FR-P01–P06)** — operator decision 2026-06-04 — since both touch the same cereal write/read
  serialization boundary; no standalone spike. Level chosen empirically against a real 40 MB
  dmp (LETTER §4 baseline: gzip ~2.07× / 52%, xz ~3.29× / 70%; target zstd level ~10–19 for
  near-xz ratio at much higher throughput). Compounds with the 30-day retention on the bill.

- **FR-P10 — tag new uploads `retain=expire` at PutObject time.** Set the object `Tagging`
  field to `retain=expire` on every dump upload (neither autoc.cc PutObject site sets Tagging
  today). New runs auto-expire 30 days after creation unless promoted to `retain=keep`.
  Requires `s3:PutObjectTagging` (+ `s3:GetObjectTagging`) on the `autoc-generator` IAM user
  (LETTER §2 — admin-applied, out of code scope; tracked in the manual-admin checklist above).

- **FR-P11 — lifecycle policy checked into the repo, applied to ALL buckets.** A single rule
  per bucket: filter `Tag retain=expire`, `Expiration: 30 days`; `retain=keep` → preserved;
  **no tag → never matched (fail-safe by design)**. Commit the JSON (the letter references
  `autoc-storage-lifecycle.json`; not yet in repo) and apply uniformly to `autoc-m1`,
  `autoc-m2`, `autoc-eval`, and legacy `autoc-storage` until it drains. (Concrete mechanics for
  FR-P08.)

- **FR-P12 — pinning is a documented manual one-liner (no helper scripts).** Promoting a
  milestone run to `retain=keep` uses the `aws s3api put-object-tagging` loop from LETTER §3,
  documented in a repo note / the spec outcome doc. (Operator declined the `autoc-pin` /
  `autoc-unpin` helpers, 2026-06-04 — manual one-liner suffices.) Pinned prefixes MUST be
  recorded in the relevant spec's outcome report, as the 029 reports already do (provenance
  lives in the repo, not the bucket).

- **FR-P13 — fix the dangling TrackerSourceRun pointer.** `autoc-eval-tracker.ini:42` points at
  `autoc-9223370258388840205-2026-05-11…/gen9999.dmp`, a prefix with **0 objects** (LETTER §1).
  Repoint at a real run (the live `autoc-tracker.ini` source, in the M2 bucket post-migration).
  The dump loader MUST fail loud on a missing key, never silently substitute (Constitution VII —
  this dangling pointer is exactly the failure that principle guards).

- **FR-P14 — eval-mode correctness: VERIFY (already fixed in 034).** Plan-phase code audit
  (2026-06-04) found both eval bugs already resolved by 034's `buildEvalData` refactor, so this
  reduces from "fix" to "verify they hold and are covered by the gate":
  - **Bug 3 (rabbit-speed):** FIXED — `buildEvalData` always sets
    `evalData.rabbitSpeedConfig = gRabbitSpeedConfig` (× `computeVariationScale()`) for both
    training and eval (`src/autoc.cc:1248-1252`). Verify the basic-eval gate exercises it.
  - **Bug 2 (stale S3 fitness):** FIXED — eval overwrites `genome.fitness` with the
    eval-computed value before re-serializing into `evalResults.gp` (`src/autoc.cc:1370`,
    `1425`, `1432-1434`). Verify the renderer shows eval fitness, not training-time fitness.
  - **Determinism note for FR-P04:** `stampEvalResultsProvenance()` (`src/autoc.cc:1435`) writes
    provenance timestamps into the dmp, so **dmp bytes are non-deterministic** — the rebuild-perf
    replay gate MUST compare **per-scenario-score byte equality** (the `ScenarioScore` vector),
    NOT whole-dmp bytes.

- **FR-P15 — constitution principle "Training-Artifact Lifecycle & Retention."** Add the
  principle (LETTER §5): (1) **ephemeral by default** — every dump tagged `retain=expire`,
  auto-deleted at 30 d; training output is disposable unless promoted; (2) **explicit pinning** —
  milestones (flown controllers, M2 source libraries, documented baselines) MUST be pinned
  `retain=keep` with the prefix recorded in the spec outcome report; (3) **provenance lives in
  the repo, not the bucket**; (4) **compression on upload** once the loader supports it (FR-P09);
  (5) **fail-loud, no silent fallback** (existing Principle VII) — the loader errors on a missing
  `TrackerSourceRun` key rather than substituting.

**Note (selector — FR-P07 dependency):** the shared `findLatestRun(bucket)` /
`extractGenNumber` extraction must also **fix the `renderer.cc::extractGenNumber` bug** — it
does not invert the `10000-gen` filename encoding the way `nnextractor.cc` does
([reference_dmp_filename_gen_encoding](../../.claude/projects/-home-gmcnutt-autoc/memory/reference_dmp_filename_gen_encoding.md)),
so renderer currently mis-reads gen numbers. And the selector must discover `.zst` keys (FR-P09).

## User Scenarios & Testing *(mandatory)*

### User Story 1 — Determine whether energy works as a lexicase secondary objective (Priority: P1)

The project wants controllers that minimize energy in addition to tracking. This story re-enables energy as a lexicase test-case axis (not a scalar penalty), runs bakes, and determines whether the GA produces a controller that is materially more energy-efficient *without* collapsing tracking — and if it doesn't, documents why.

**Why this priority:** This is the actual goal the 033 smoothness detour was a wrong turn toward. Side benefit: an energy axis directly penalizes the throttle-pegged dead-neuron stuck basin, potentially lowering the intrinsic basin-lottery fail rate (the cheap first attack from the basin-landscape backlog entry).

**Independent Test:** Run an energy-lexicase bake to convergence and compare against the tracking-only baseline (pop=5000/wind=49 — the 034-validated config; `autoc.ini` — plus 034's craft variations) on two axes: (a) tracking quality (per-scenario score, avgMaxStreak) must not materially regress; (b) energy_score must materially improve. A clear yes (both hold) or a clear no (with documented failure mode) both satisfy the story.

**Acceptance Scenarios:**

1. **Given** the energy lexicase axis re-enabled and axes grouped physically (bank = pitch+roll, throttle/energy separate), **When** a bake runs to convergence, **Then** the outcome is classified as energy-improved-without-tracking-collapse, tracking-collapsed, or energy-unmoved — with per-scenario evidence in an outcome doc.
2. **Given** the 4 previously-`DISABLED_` Selection027 multi-objective tests, **When** the energy axis is active, **Then** those tests are re-enabled and pass.
3. **Given** lexicase active with craft variations widening the per-scenario magnitude spread, **When** epsilon is applied, **Then** it is MAD-relative (not constant 0.5), with the constant-epsilon path retained behind an ini switch for historical reproducibility.
4. **Given** the investigation outcome, **When** energy-as-throttle-proxy is found insufficient, **Then** the open question on richer total-energy (altitude+airspeed) is answered with a go/no-go for a follow-on, rather than silently dropped.

### Edge Cases

- **Energy axis re-collapses tracking** the way scalar smoothness did. The physical-axis grouping + MAD-relative epsilon are the hypothesized mitigations; if it still collapses, that's a documented outcome, not a silent failure.
- **Energy improves but only by flying slower/lower** (degenerate "minimize energy by minimizing flying"). Tracking-non-regression is the guardrail.
- **MAD-epsilon change perturbs determinism / historical reproducibility** — the constant-epsilon path must remain selectable so prior runs can be reproduced bit-for-bit.
- **Basin lottery confounds the comparison** — a stuck (throttle-pegged) bake tells us nothing about the energy axis; per the basin protocol, budget 2–3 bakes to clear the ~1:3 lottery before drawing conclusions.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Energy MUST be re-enabled as a lexicase test-case axis (per-scenario `energy_score`), NOT a scalar penalty composed into the tracking score (uncomment `src/eval/selection.cc:69`).
- **FR-001b**: The v1 `energy_score` metric MUST be a **convex (super-linear) function of throttle command integrated over time** — `Σ_t f(out_th)`, `f` super-linear — replacing the existing linear `Σ(out_th−1)/2` placeholder (Clarification 2026-06-04). Induced drag is charged implicitly by the FDM (FR-009), so no separate maneuver penalty is added. The convex shape is **modeled** for v1; calibrating `f()` against a **measured power curve from the real craft** is a follow-on, blocked today by the absence of an ESC current monitor. PE/KE mechanical-energy terms remain the FR-007 richer-energy go/no-go, not v1.
- **FR-002**: Lexicase axes MUST be grouped physically — bank (pitch+roll) together, throttle/energy separate — per `project_smoothness_axis_grouping`. **For v1 this is satisfied trivially (research R7): with stability OFF (FR-008) there is no pitch/roll control-amplitude axis to group, and energy is already throttle-separate from the tracking `score`; the bank-grouping provision is latent and activates only if a stability/smoothness axis returns.**
- **FR-003**: Lexicase epsilon MUST be MAD-relative (scales with per-scenario magnitude) rather than constant 0.5, with the constant-epsilon path retained behind an ini switch for historical reproducibility (`project_lexicase_mad_epsilon`).
- **FR-004**: The 4 `DISABLED_` Selection027 multi-objective tests MUST be re-enabled and pass with the energy axis active.
- **FR-005**: An **M1** energy-lexicase bake MUST be run and its outcome classified against the M1 tracking-only baseline (**pop=5000/wind=49** + 034 craft variations — the 034-validated `autoc.ini` config that tracks pastonly3; **NOT** the 8000/wind=36 shape, which 034 found throttle-pegs and would confound the energy verdict) on both tracking-quality and energy-efficiency, with per-scenario evidence captured in an outcome doc. **Primary comparator = the per-axis aggressiveness distributions** (the existing `per_axis_aggressiveness` PNGs / per-axis `dCtrl`·⟨|out|⟩ time-series, `project_late_run_fitness_interpretation`), which are variation- and config-stable — so they isolate the energy effect even though MAD-epsilon (FR-003) lands at the same time as the energy axis (absolute fitness vs the constant-ε baseline alone would confound the two changes). The energy verdict requires the distributions to move toward goal on **all three control axes (pitch, roll, throttle)**, not throttle alone.
- **FR-005b**: An **M2 (tracker)** energy-lexicase bake MUST also be run in 035 (decided 2026-06-04, Clarifications) and classified against the M2 tracking-only baseline. **Baseline = 032-phase1** (the pinned M2 tracker milestone, S3 `autoc-9223370257807536859-2026-05-17…`) or the most-recent confirmed-climber tracking-only tracker run if a fresher one exists. The M2 baseline is itself subject to the basin lottery (infrequent stalls / throttle-peg dead neurons seen in M2 training), so the comparison MUST be against a **confirmed non-stuck climber**, and 2–3 bakes are budgeted for the energy run to clear the lottery. The **objective is identical in both modes** — *on point (tracking) with minimum energy* — using the same energy axis, metric (FR-001b), and MAD-epsilon (FR-003); energy is a **characteristic of the controller**, so an M1 energy win is implicitly required of M2 too, and a bang-bang M2 is not acceptable. Use the same per-axis aggressiveness comparator as FR-005. M2 also yields the `#GenCrash hullStrike=N` telemetry that seeds the future hull-crash feature (FR-008b).
- **FR-006**: The investigation MUST guard against degenerate energy wins (energy improved only by abandoning tracking) — tracking-quality non-regression is a required condition for an "energy works" verdict.
- **FR-007**: The open question (throttle-proxy energy vs richer total-energy = altitude+airspeed) MUST be resolved with a go/no-go recommendation for a follow-on, informed by the bake outcome.
- **FR-008**: The `stability_score` axis (`selection.cc:68`) stays **OFF** for v1 (decided 2026-06-04, Clarifications). Rationale: `stability_score = Σ_t (|out_pt|−1)+(|out_rl|−1)` is a control-amplitude / bang-bang penalty — exactly the kind of control-rate term FR-009 prohibits (smoothness must *emerge* from the energy axis via induced drag, not be added as its own whip) — and enabling it would confound the energy verdict (a tracking change could not be attributed to energy vs stability). Re-enable only as a follow-on if energy alone leaves the controller too aggressive.
- **FR-008b**: Candidate lexicase axes for this multi-objective framework are **energy** (primary, this feature) and **stability** (FR-008). **M2 hull-crash-cost is split out of 035 into its own future feature** (decided 2026-06-04, Clarifications) — see BACKLOG "Hull-crash-cost as a lexicase fitness dimension (M1/M2)". It remains a *fitness dimension* (NOT 036/island-selection work) and is load-bearing (hull-strikes grow monotonically with tracking skill — 030: ~1→11/gen; 032: ~3× faster — gating real-flight deployment), but its penalty design is hard enough to warrant a dedicated spec: it must not merely stop the score, must not be a flat scalar (early crash worse than late), and must solve the credit-assignment problem of a rare-but-fatal crash (1 of ~300 scenarios). 035's `#GenCrash hullStrike=N` telemetry from the M2 baseline run feeds that future spec.
- **FR-009**: 035 MUST optimize the **energy dimension, NOT smoothness**. Do not add a control-rate / bang-bang penalty. Smoothness is an emergent *consequence* of minimizing energy (aggressive maneuvering → induced drag → energy cost), not a target in its own right. The 034 origm1-5000×49 run already self-smoothed (roll-rate halved in the back half) under a pure tracking objective with no smoothness term — evidence that the search drifts toward smooth on its own, so the lever to add is energy, not a smoothness whip. (Prior scalar smoothness/penalty attempts *dulled* the system — see discussion below.)

### Design discussion — measuring total energy input (operator, 2026-06-03)

The energy metric is the central design question of 035 (supersedes the throttle-proxy placeholder in FR-007 / Key Entities). Direction:

- **Quantity to minimize = total energy *input*, summed over time** — `Σ_t energy_consumed(t)`. Energy *consumed*, not a control-amplitude proxy. Start with the consumed-energy integral as the v1 metric.
- **Energy input is probably a non-linear function of throttle**, not the linear `Σ(out_th − 1)` placeholder. Motor/prop power draw rises super-linearly with throttle; the metric should reflect that (e.g. a convex function of throttle command, or the FDM's actual electrical/shaft power if exposed).
- **Induced drag is part of the cost.** Aggressive maneuvering (tight turns, high-AoA) burns energy via induced drag even at constant throttle — and the LaRCSim FDM already simulates it. So a *measured* energy term naturally charges for bang-bang on every axis, which is exactly why no separate smoothness penalty is needed (FR-009).
- **Optionally fold in mechanical-energy state** — potential energy (altitude) and kinetic (airspeed). A craft that trades altitude/speed for tracking is spending stored energy; a full accounting is `Δ(PE+KE) + work_in`. **But start simple**: consumed-energy-over-time first; add the PE/KE state terms only if the throttle-integral metric proves insufficient (this is the richer-total-energy go/no-go of FR-007).
- **Why lexicase, and the "dulling" risk.** Several prior approaches that made "more energy = worse" *dulled the whole system* — the scalar 033 smoothness-penalty floor collapsed the controller into a Pareto corner ([project_scalar_multiobjective_collapse](../../.claude/projects/-home-gmcnutt-autoc/memory/project_scalar_multiobjective_collapse.md)), and energy-as-penalty underperformed in 027/028. The bet for 035 is that **lexicase keeps energy as a separate selection dimension** (some test cases select on energy, others on tracking) rather than a scalar discount on tracking — so it can pull energy down without dulling the tracking drive. If lexicase *also* dulls it, that's a key negative result.
- **Timing (DECIDED 2026-06-04 — apply from gen 0, no ramp):** the energy lexicase axis is in the selection pool from gen 0 throughout the run — no late ramp. Rationale (operator): a late ramp is *tuning*; the first-principles approach (stay on track with minimum energy) should work on its own, and adding a ramp would introduce a timing confound into the energy verdict. The earlier consideration that bang-bang aids early basin-finding (so energy might ramp in late like the variation ramp) is explicitly **rejected** as premature tuning — if gen-0 energy pressure turns out to suppress exploration, that is itself a documented negative result, not a knob to pre-empt it with.
- **M2 motivation:** tracking mode is intrinsically high-energy (a chase craft maneuvers continuously to follow an erratic target), and real-flight battery is a hard constraint — so the energy dimension matters *more* for M2/deployment than for smooth pathgen courses. This is the deployment reason 035 is load-bearing.

### Key Entities

- **Secondary objective**: a per-scenario score axis (energy_score, optionally stability_score) used as a lexicase test case alongside the tracking score — distinct from a scalar penalty folded into one number.
- **energy_score**: the per-scenario energy metric. v1 (decided 2026-06-04, FR-001b) is a **convex throttle-command integral** `Σ_t f(out_th)`, `f` super-linear, replacing the existing linear `Σ(out_th − 1)/2` placeholder; induced drag is captured implicitly by the FDM. The convex shape is modeled (real-craft power-curve calibration is a follow-on, blocked by no ESC current monitor); PE/KE state terms are the FR-007 later option. Lower = better (less energy).
- **MAD-relative epsilon**: lexicase pass/fail threshold scaled by the median-absolute-deviation of per-scenario scores, replacing the constant 0.5.
- **Baseline**: the tracking-only run(s) that energy bakes are compared against — the M1 baseline (034-delivered, pop=5000/wind=49 + craft variations, the validated `autoc.ini` config) for FR-005, and the corresponding M2 tracking-only tracker baseline for FR-005b. Same objective in both modes (on-point tracking + minimum energy).

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: The energy investigation produces a clear verdict (energy-works / tracking-collapses / energy-unmoved) for **both M1 and M2**, each backed by a baked comparison against its respective baseline, captured in an outcome doc — no ambiguous "maybe."
- **SC-002**: If energy works, the resulting controller shows materially improved energy_score AND per-axis aggressiveness distributions that move toward goal on **all three controls (pitch/roll/throttle)**, with tracking quality (per-scenario score + avgMaxStreak) within noise of the baseline; if it doesn't, the failure mode is documented well enough to inform the total-energy go/no-go.
- **SC-003**: The MAD-relative epsilon change preserves bit-exact reproducibility of prior runs when the constant-epsilon ini switch is set.
- **SC-004**: The Selection027 multi-objective tests are re-enabled and green.

## Assumptions

- 034 is delivered first: minisim removed, smoothness removed, craft variations landed, tech-debt fold-ins done, and a craft-variation-trained controller flight-tested. 035 starts from that baseline.
- The investigation may conclude energy-as-lexicase doesn't work; that is a valid, valuable outcome (it closes a question the project has circled since 027).
- "Materially improved/regressed" thresholds are set against the baseline's per-scenario metrics at investigation time (per-scenario currency).
- The basin lottery (~1:3 stuck) applies; budget 2–3 bakes to land a non-stuck climber worth analyzing.

## Out of Scope

- Total-energy (altitude+airspeed) implementation — 035 only produces the go/no-go recommendation for it (FR-007).
- Demetic / island-model GA for basin escape — separate BACKLOG research entry (the energy axis is the cheap first attack on the same stuck basin).
- Craft/camera variation work — owned by 034 (craft) and a later iteration (camera).
- Changing NN topology or the tracking fitness surface (conical scoring).
- **`dmp-dump` re-fetch avoidance (analytics perf).** `--run-summary` re-pulls every gen's dmp
  from S3 on each call (~1.6 s/gen), painful when re-plotting a live, growing run.
  - **DONE (2026-06-07, interim):** `dmp-dump --run-summary --since-gen N` skips gens < N (header
    suppressed) so you fetch only new gens and append to a cached CSV — t6 regen at gen 670 with
    1–565 cached fetched only ~104 new gens (**2.5 min vs ~15 min**). Workflow in `docs/REPORTS.md`.
  - **Still deferred:** a transparent per-dmp `/tmp/<run-id>/` cache that makes re-fetch avoidance
    automatic (no `--since-gen` bookkeeping). Tracked in agent memory
    `project_dmp_driven_analytics_backlog`.
