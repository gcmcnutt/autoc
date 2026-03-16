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

## Architecture

- **C++17**, CMake, Eigen, cereal (serialization), GoogleTest
- **Desktop** (train): autoc evolution engine + minisim or crrcsim FDM
- **Embedded** (deploy): xiao — Seeed XIAO BLE Sense via PlatformIO
- **Three components**: autoc (evolution), crrcsim (flight dynamics), xiao (embedded target)
- **NN-only**: GP tree evolution has been removed. NN01 binary format is the sole controller format.

## Governance

Constitution supersedes all other practices. Amendments require documentation and rationale.

**Version**: 1.0.0 | **Ratified**: 2026-03-16
