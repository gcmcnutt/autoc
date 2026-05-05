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

## Architecture

- **C++17**, CMake, Eigen, cereal (serialization), GoogleTest
- **Desktop** (train): autoc evolution engine + minisim or crrcsim FDM
- **Embedded** (deploy): xiao — Seeed XIAO BLE Sense via PlatformIO
- **Three components**: autoc (evolution), crrcsim (flight dynamics), xiao (embedded target)
- **NN-only**: GP tree evolution has been removed. NN01 binary format is the sole controller format.

## Governance

Constitution supersedes all other practices. Amendments require documentation and rationale.

**Version**: 1.1.0 | **Ratified**: 2026-03-16 | **Last Amended**: 2026-05-04 (Principle V added)
