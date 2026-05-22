# AutoC Constitution

## Core Principles

### I. Testing-First

All significant changes MUST include tests that validate the intended behavior. The test-driven
development workflow is:

1. Write tests that specify expected behavior before implementation
2. Verify tests fail (confirming they test something meaningful)
3. Implement the feature or fix
4. Verify tests pass
5. Refactor if needed while maintaining passing tests

**Rationale**: The NN evolution system involves complex mathematical operations and control logic
where subtle bugs can produce invalid evolution results. Tests provide confidence that changes
do not regress existing functionality.

**Exemptions**: Exploratory prototyping and research spikes may skip tests, but any code
promoted to mainline MUST have corresponding tests.

### II. Build Stability

All commits to the main branch MUST compile successfully and pass the test suite. The build
MUST NOT be left in a broken state.

**Build verification requirements**:

- autoc + crrcsim: `bash scripts/rebuild.sh` MUST compile without errors and all tests pass
- xiao: `cd xiao && pio run -e xiaoblesense_arduinocore_mbed` MUST compile without errors

**Recovery**: If a commit breaks the build, the fix is highest priority.

### III. No Compatibility Shims

Clean-cut all changes. Update all callers directly rather than adding backwards-compatibility
wrappers, re-exports, or unused variable renames. Every file must justify its existence.

### IV. Unified Build

The top-level CMakeLists.txt is the single source of truth. Shared dependencies (cereal,
GoogleTest, inih) are declared once via FetchContent and inherited by subdirectories.
crrcsim builds as `add_subdirectory(crrcsim)`. No duplicate dependency declarations.

### V. Versioned Persistence Artifacts

Persistence-format files that flow between training runs, embedded targets, and downstream
tooling MUST carry an explicit format version starting from the commit that introduces
this principle. Currently load-bearing artifacts:

- **NN dump (`.dmp`) files** — the cereal-serialized `EvalResults` written to S3 by autoc
  workers and consumed by the renderer, eval pipeline, xiao firmware, and tracker-mode
  library construction. These are also the first kind of file expected to feed downstream
  features (030 tracker mode consumes prior-run `.dmp` as target trajectories), so a
  schema mismatch silently producing wrong-but-parseable results is the failure mode this
  principle is designed against.
- Other on-disk artifacts that bridge processes / tools (e.g., `data.dat`, `data.stc`,
  evaluation result files) SHOULD carry the same versioning treatment when the scope of a
  feature touches them.

**Read-side contract**: a reader consuming a versioned artifact MUST attempt
backward-compatible loading where possible (older versions read with documented field
defaults / migrations). When backward-compatible loading is not possible, the reader MUST
**fail loudly with a clear error message** identifying both the artifact version and the
reader version — never silently truncate, default-init, or otherwise pretend success. The
error message is the prompt for an explicit operator decision (re-run training to produce
an upgraded artifact, port the migration path forward, or otherwise resolve).

**Write-side contract**: writers MUST embed the version field at a stable, parseable
offset that does not itself depend on later schema content. Cereal's `CEREAL_CLASS_VERSION`
is the standard mechanism, but per the project's no-cereal-versioning practice for
in-tree greenfield changes, version transitions to which the project is fully committed
should bump the version field directly (not maintain compatibility shims) — fail-loud on
read is the safety net for older files.

**Rationale**: dmp files are the input contract for at least three downstream consumers
(renderer, xiao deploy, tracker-mode library). A future schema change made without a
version bump risks silent semantic drift across those consumers; the asymmetry between a
hard error (operator fixes immediately) and a silent semantic mismatch (subtle wrong
flight behavior weeks later) makes the loud-fail rule strongly preferred.

**Exemption**: this principle does not apply to ephemeral / per-run logs (e.g., autoc
gen-log lines, xiao flash log text). Those are human-readable, short-lived, and tolerate
parser drift via grep-pattern liberality.

### VI. Type-Domain Discipline

Eval-pipeline scalars are `gp_scalar`. Geometric quantities are `gp_vec3` / `gp_quat`.
Fitness accumulation is `gp_fitness`. Raw `float` / `double` is reserved for cases where
on-disk byte format, hardware protocol, or library-imposed signature outranks alias
documentation, and each such case MUST be annotated `// raw-ok: <reason>` at the
declaration site.

**Domain definitions:**

| Alias | Underlying | Domain |
|---|---|---|
| `gp_scalar` | `float` | Eval-pipeline scalars: anything flowing into NN forward-pass, `gp_vec3` / `gp_quat` arithmetic, or down to flight hardware (xiao). |
| `gp_vec3` / `gp_quat` | `Eigen::Matrix<float,3,1>` / `Eigen::Quaternion<float>` | Any 3D geometric quantity — position, velocity, axis, attitude. |
| `gp_fitness` | `double` | Fitness accumulation across ticks/scenarios; per-axis aggregates summing many small contributions; ranking-decisive numerics in `FitnessComputer` and selection. |

**Whitelist** (raw `float` / `double` permitted, `// raw-ok: <reason>` required at site):

- NN byte-format buffers (`float[N]` where N is layout-locked by `NNInputs` / cereal serialization)
- Hardware-protocol fields (MSP, INAV blackbox, cereal byte-format struct members) where on-disk byte layout outranks alias documentation
- Host-only metadata that never crosses into eval or fitness math (logging timestamps, plot axis labels, file paths, debug print formatting)
- Library-imposed signatures (`std::chrono` durations, `time_t`, etc.)

**Verification — the question this principle is designed to answer:**

> *"Did we drift or miss a use of `gp_scalar` / `gp_fitness` / `gp_vec3` / `gp_quat`?"*

A reviewer (or assistant at `/speckit.implement` time) MUST be able to answer **yes** or
**no** in finite time by running, from repo root:

```bash
grep -nE '\b(float|double)\b' src/eval/ src/nn/ include/autoc/eval/ include/autoc/nn/ \
  | grep -v -E '// raw-ok:'
```

Each remaining hit MUST either (a) be justified with a `// raw-ok: <reason>` annotation
at the declaration site or (b) be converted to the appropriate alias. The grep IS the
audit; the comment IS the justification record. An unannotated raw `float` / `double` in
eval / nn / fitness code is the violation, regardless of whether it produces an
observable bug.

**Verification cadence:**

- **Per-milestone (mandatory)**: every `/speckit.implement` closing report runs the grep
  on the touched paths and either annotates or converts before marking the milestone
  complete. No milestone is "done" with unannotated raw-type hits in its diff.
- **Codebase-wide backfill (one-time)**: a separate audit-pass spec backfills existing
  violations across the tree. Until that lands, the principle is enforced incrementally
  on touched code.
- **Periodic sanity (optional)**: a CI job or pre-commit hook running the grep + denylist
  closes the loop without operator intervention.

**Rationale**: A `double` holding fitness adjacent to raw-`float` accumulator code
produces silent precision drift in selection rankings that compounds across generations
into wrong-direction evolution. A raw `float` for a beacon mount that should have been
`gp_scalar` doesn't trip Eigen template type-mismatch — it silently implicit-converts at
the call site, doing slightly different rounding than the canonical pipeline path.
Neither failure mode produces a compile error, a test failure, or even an obviously-wrong
number; they produce a *different* training trajectory that nobody can attribute to the
missed alias months later. The aliases exist precisely because that class of bug is
undetectable after the fact — the only defense is a pre-merge type-domain audit, and the
audit is only tractable if the convention is universal in eval / nn / fitness code.

### VII. No Silent Fallback Defaults

Member variables that receive values from constructor parameters MUST NOT carry in-class
default initializers. The constructor's initializer list is the single assignment site; if
a new constructor omits a member, the compiler must flag it rather than silently falling
back to a stale default.

**Permitted exceptions** (must be annotated `// default-ok: <reason>` at the declaration):

- Counters and accumulators that genuinely reset each use (e.g., `int count_ = 0;`)
- Sentinel / flag states that have a universally correct initial value
  (e.g., `bool prev_out_valid_ = false;`)

**Rationale**: The 032 `cepGateThreshold` bug demonstrated this failure mode exactly.
`evaluator.cc` and `tracker_stepper.cc` fell back to a hardcoded `1.25` default instead of
reading the value from `WorkerInit`, producing correct-looking but semantically wrong
results on any configuration where the operator had set a different threshold. The fallback
was invisible — no compile error, no test failure, no runtime warning. Removing in-class
defaults for constructor-supplied values ensures the compiler catches the omission.

**Scope**: applies to all classes in the eval / nn / fitness / stepper pipeline where
values flow from `WorkerInit`, `EvalData`, `ScenarioMetadata`, or `.ini` config. Does NOT
apply to plain-old-data structs used purely as wire-format containers (e.g.,
`TrackerHistoryWindow`), where zero-initialization via `{}` is the intended contract.

## Architecture

- **C++17**, CMake, Eigen, cereal (serialization), GoogleTest
- **Desktop** (train): autoc evolution engine + minisim or crrcsim FDM
- **Embedded** (deploy): xiao — Seeed XIAO BLE Sense via PlatformIO
- **Three components**: autoc (evolution), crrcsim (flight dynamics), xiao (embedded target)
- **NN-only**: GP tree evolution has been removed. NN01 binary format is the sole controller format.

## Governance

Constitution supersedes all other practices. Amendments require documentation and rationale.

**Version**: 1.3.0 | **Ratified**: 2026-03-16 | **Last Amended**: 2026-05-22 (Principle VII added — No Silent Fallback Defaults)
