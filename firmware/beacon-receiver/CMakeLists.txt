# 042 camera receiver (T002, T013).
#
# TIER 0 HERE BUILDS WITH NO DEPENDENCIES AT ALL. That is the property the whole architecture rests on
# (plan.md §Structure Decision): it is why core/ runs live on the Pi, in replay on the dev box, and
# cross-compiles from WSL2 with nothing but a cross-gcc. Do not add a find_package() for core/, tools/ or
# tests/ — if something here needs a library, it belongs in io/ or app/.

# ---- beacon_core: C11, zero dependencies, no allocation after init -----------------------------------
add_library(beacon_core STATIC
  src/core/config.c
  src/core/record.c
  src/core/container.c
)
set_target_properties(beacon_core PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF)
target_include_directories(beacon_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/core)
target_compile_options(beacon_core PRIVATE -Wall -Wextra -Wconversion -Wsign-conversion)

# Stamp the build so every record and every recording traces back to the source that produced it
# (spec §16.2). Falls back to a timestamp hash inside config.c when git is unavailable.
find_package(Git QUIET)
if(GIT_FOUND)
  execute_process(COMMAND ${GIT_EXECUTABLE} rev-parse --short=16 HEAD
                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                  OUTPUT_VARIABLE BR_GIT_SHA OUTPUT_STRIP_TRAILING_WHITESPACE
                  ERROR_QUIET RESULT_VARIABLE BR_GIT_RC)
  if(BR_GIT_RC EQUAL 0 AND BR_GIT_SHA)
    target_compile_definitions(beacon_core PRIVATE BCN_BUILD_ID=0x${BR_GIT_SHA}ULL)
  endif()
endif()

# ---- beacon_tools: dev-box analysis binaries, also dependency-free ------------------------------------
add_executable(beacon_bench tools/bench.c)
set_target_properties(beacon_bench PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON)
target_link_libraries(beacon_bench PRIVATE beacon_core)

# T006b: the golden-vector generator. Run ONLY on a format_version bump — regenerating vectors to make a
# failing test pass would defeat the mechanism the vectors exist to provide.
add_executable(beacon_gen_record_vectors tools/gen_record_vectors.c)
set_target_properties(beacon_gen_record_vectors PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON)
target_link_libraries(beacon_gen_record_vectors PRIVATE beacon_core)

add_custom_target(beacon_tools COMMENT "042 receiver dev-box tools")
add_dependencies(beacon_tools beacon_bench beacon_gen_record_vectors)

# ---- tests: registered with CTest (T013) -------------------------------------------------------------
# See tests/bcn_test.h for why these are plain C rather than GoogleTest — short version: tasks.md names
# every test file .c, and requiring C++ plus a network FetchContent to test core/ would put a dependency
# in front of exactly the Pi and cross paths the core/shell split exists to keep dependency-free.
add_custom_target(beacon_tests COMMENT "042 receiver tests (run with: ctest -R beacon)")

set(BR_TESTS
  unit/test_config
  unit/test_versions
  golden/test_record_vectors
)

foreach(t IN LISTS BR_TESTS)
  string(REPLACE "/" "_" tname "beacon_${t}")
  add_executable(${tname} tests/${t}.c)
  set_target_properties(${tname} PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON)
  target_link_libraries(${tname} PRIVATE beacon_core)
  target_include_directories(${tname} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
  target_compile_options(${tname} PRIVATE -Wall -Wextra)
  # Golden-vector tests need the committed vectors; give them the source path rather than copying, so a
  # stale copy in the build tree can never be what actually passed.
  add_test(NAME ${tname} COMMAND ${tname} ${CMAKE_CURRENT_SOURCE_DIR}/tests/golden/record_vectors)
  add_dependencies(beacon_tests ${tname})
endforeach()

# T002b: these are tier-0 targets, so they join the one-command gate.
add_dependencies(all-targets beacon_core beacon_tools beacon_tests)

# ---- tier 1: cross / embedded targets that need no hardware (T002a) ----------------------------------
if(WITH_EMBEDDED)
  # app/ + io/ need libcamera and therefore a sysroot; they are declared here as the tasks land (T021+).
  message(STATUS "042 receiver: WITH_EMBEDDED=ON — tier-1 targets will be declared as io/app land (T021+)")
endif()
