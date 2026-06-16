<!--
SYNC IMPACT REPORT
Version change: 1.6.0 → 1.7.0 (MINOR — added Principle X: Single Ordered Backlog)
Modified principles: none
Added sections:
  - X. Single Ordered Backlog — project backlog lives in ONE ordered file (specs/BACKLOG.md);
    new items are appended in order, never split into separate per-item .md files (which lose
    their place in the ordering and get lost). Agent-memory backlog entries are non-authoritative
    one-line pointers at most. Surfaced 2026-06-16 when the 038 reporting-standardization item was
    first written as a standalone memory .md before being consolidated into BACKLOG.md.
Removed sections: none
Templates / dependent artifacts:
  - .specify/templates/plan-template.md, tasks-template.md, spec-template.md — ✅ no change
    needed (generic Constitution Check; no hardcoded backlog steps).
  - specs/BACKLOG.md — remains the single backlog of record; 038 reporting item added there.
Follow-up TODOs: none

PRIOR (1.5.0 → 1.6.0, MINOR — added Principle IX: Detached Training Launch):
  - IX. Detached Training Launch — training MUST be started via scripts/train.sh (detached
    session via setsid, reparented to systemd --user, nohup, line-buffered, cores enabled,
    unique logfile). Assistants MUST NOT launch via the Bash-tool run_in_background mechanism,
    which the harness reaps at agent-session end (silently killed t4 gen 278, t5 gen 152 on
    2026-06-06). Codifies what was scattered in agent memory (reference_autoc_launch_command,
    now retired in favor of this principle + the script).
-->
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

**Build discipline**: a change to `CMakeLists.txt` (a new target, dependency, link, or test
registration) MUST be built with a clean `scripts/rebuild-perf.sh` (PERFORMANCE_BUILD,
single-threaded for FP determinism) — NOT an incremental `cmake -S . -B build` reconfigure — so
the optimized build, link graph, and `run_autoc_tests` registration regenerate coherently.
Source-only edits use incremental `cmake --build build --target <t>`. The operator drives the
clean rebuild.

**Rationale**: an incremental reconfigure after a CMakeLists.txt edit can leave stale link state
and miss test registration; `rebuild-perf.sh` is also the FP-deterministic basis for the
bit-replay regression gate. This is the build-*coherence* rule and is distinct from the
Principle II `rebuild.sh` compile-and-test correctness gate. The rule is binding here because
agent memory (`feedback_incremental_build_default`) is advisory, not authoritative — the
constitution is the single source.

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

### VIII. Training-Artifact Lifecycle & Retention

S3-resident training artifacts (per-gen `.dmp` dumps) are storage that costs money and grows
without bound. They MUST be ephemeral by default and preserved only by deliberate act.

1. **Ephemeral by default.** Every uploaded run dump MUST be tagged `retain=expire` at
   `PutObject` time and is auto-deleted by the bucket lifecycle policy 30 days after creation.
   Training output is disposable unless explicitly promoted.
2. **Explicit pinning.** A run is preserved only by tagging its objects `retain=keep`.
   Milestones — flown controllers, M2/M3 source libraries, documented baselines — MUST be
   pinned, and the pin recorded (with its S3 prefix) in the relevant spec's outcome report.
3. **Provenance lives in the repo, not the bucket.** Flight reports and outcome docs cite the
   exact S3 prefix; the bucket is not a system of record. A bucket that loses an unpinned run
   to expiry MUST never lose the *knowledge* of a run that mattered.
4. **Uniform naming, bucket-as-discriminator.** Per-mode buckets (`autoc-m1`, `autoc-m2`,
   `autoc-eval`, future `autoc-m3`) hold artifacts under an **identical** run-id + filename
   convention (`<run-id>/gen<N>.dmp[.zst]`); the bucket — not a name prefix — distinguishes
   mode. Tools resolve "latest run / latest gen" through one shared, bucket-relative,
   prefix-agnostic selector.
5. **Compression on upload.** Dumps MUST be zstd-compressed at the serialization boundary once
   the loader supports transparent inflation; the read path accepts both `.dmp.zst` and legacy
   `.dmp`.
6. **Fail-loud loader (reinforces VII).** The dump loader MUST error on a missing
   `TrackerSourceRun` key rather than silently substituting a fallback. A dangling source
   pointer is a hard stop, not a default.

**Rationale**: `autoc-storage` reached ~868 GB / ~84% of the AWS bill before a tag-driven
30-day retention scheme was installed (LETTER-s3-retention.md, 2026-06-02). The
no-tag-never-deleted lifecycle is fail-safe by design — unmarked objects are never matched —
so the risk is a milestone silently expiring, which (2) and (3) guard against by making pinning
deliberate and provenance repo-resident. Compression (5) compounds with retention: ~3× on
float-weight dumps on top of the 30-day cap.

### IX. Detached Training Launch

Training runs MUST be started via `scripts/train.sh <ini-file> <logfile>`. The script is the
single authoritative launch path; it starts autoc:

- **Detached** — `setsid` into its own session with no controlling tty, reparenting to
  `systemd --user`, plus `nohup`. The run survives terminal/SSH teardown **and** agent-session
  teardown.
- **Line-buffered** — `nohup stdbuf -oL -eL <binary>` (stdbuf innermost), so `tail -f` sees
  per-gen lines as they happen.
- **Core-enabled** — `ulimit -c unlimited` for autoc and its inherited crrcsim workers.
- **Non-clobbering** — refuses to overwrite an existing logfile; one unique log per run.
  Multiple concurrent runs against the same ini are permitted (reduce per-run worker count as
  needed).

**Assistants/agents MUST NOT launch training via a session-bound mechanism** — specifically not
the Bash-tool `run_in_background` task, nor a foreground shell job. Harness-tracked background
tasks are owned by the agent session and are signalled dead (group SIGTERM/SIGKILL) when that
session ends, clears, or is superseded.

**Rationale**: runs t4 (died gen 278) and t5 (died gen 152) on 2026-06-06 were killed mid-run
with **no error in the log, no core, and flat memory** — not a code bug. The log is the
process's stderr, so an uncaught C++ throw would have printed `terminate called … what(): …`
into it; it did not, and the log ended on a clean line. The harness background-task wrapper
output was 0 bytes (vs the `… Aborted …` a genuine internal crash leaves), proving an external
group signal: the harness reaping its own agent-owned background tasks. These were launched by
the assistant via `run_in_background`; the operator's own `nohup … &` terminal launches never
had the problem because they are owned by a long-lived login. `nohup` alone is insufficient
(it only ignores SIGHUP, not the harness's SIGTERM/SIGKILL); the fix is full detachment via a
non-tracked `setsid` launch — exactly what `scripts/train.sh` encapsulates. Core dumps are also
re-enabled per-run because the system `ulimit -c` default was silently reset (driver/software
update), leaving recent crashes with no core for diagnosis.

### X. Single Ordered Backlog

The project backlog lives in **one ordered file** — `specs/BACKLOG.md`. New backlog items MUST be
appended to it, in order, under the appropriate section. Backlog items MUST NOT be written as
separate per-item `.md` files.

**Rationale**: a backlog split across many standalone files loses the one thing a backlog needs —
a single, scannable ordering. Separate files have no inherent sequence, drift out of any index that
references them, and get lost; a reader can no longer see "what's next" or "what's related" at a
glance. One file keeps priority, grouping, and cross-references visible and reviewable in a single
read.

**Agent memory is not a backlog store.** Per the in-repo-authoritative-memory practice, a
`~/.claude` memory entry for a backlog item is at most a **one-line pointer** at `specs/BACKLOG.md`
(or a specific section) — never the authoritative item text, and never a parallel set of
per-item files. When a backlog thought first lands in memory, it MUST be promoted into
`specs/BACKLOG.md` and the memory reduced to a pointer.

**Scope**: applies to forward-looking work items (deferred features, infra cleanups, research
threads). Does NOT apply to per-feature task lists (`specs/<feature>/tasks.md`), which are the
speckit work-breakdown for an active feature, nor to outcome/findings docs.

## Architecture

- **C++17**, CMake, Eigen, cereal (serialization), GoogleTest
- **Desktop** (train): autoc evolution engine + crrcsim FDM (sole worker since 034; minisim retired)
- **Embedded** (deploy): xiao — Seeed XIAO BLE Sense via PlatformIO
- **Three components**: autoc (evolution), crrcsim (flight dynamics), xiao (embedded target)
- **NN-only**: GP tree evolution has been removed. NN01 binary format is the sole controller format.

## Governance

Constitution supersedes all other practices. Amendments require documentation and rationale.

**Version**: 1.7.0 | **Ratified**: 2026-03-16 | **Last Amended**: 2026-06-16 (Principle X — Single Ordered Backlog: project backlog lives in one ordered file, specs/BACKLOG.md; never scatter into per-item .md files)
