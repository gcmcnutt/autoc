cmake_minimum_required(VERSION 3.11)
project(autoc)

# Build crrcsim FDM alongside autoc (requires SDL, OpenGL, etc.)
option(BUILD_CRRCSIM "Build CRRCSim flight dynamics model" ON)

find_package(VTK REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(AWSSDK REQUIRED COMPONENTS core s3)
# 035 FR-P09 — zstd for dmp compression on the serialization boundary.
find_library(ZSTD_LIBRARY NAMES zstd REQUIRED)

include(FetchContent)

# inih config parser
FetchContent_Declare(
    inih
    GIT_REPOSITORY https://github.com/benhoyt/inih.git
    GIT_TAG r58
)
FetchContent_Populate(inih)
add_library(inih STATIC
    ${inih_SOURCE_DIR}/ini.c
    ${inih_SOURCE_DIR}/cpp/INIReader.cpp
)
target_include_directories(inih PUBLIC ${inih_SOURCE_DIR} ${inih_SOURCE_DIR}/cpp)

# cereal serialization (header-only, replaces Boost.Serialization)
FetchContent_Declare(
    cereal
    GIT_REPOSITORY https://github.com/USCiLab/cereal.git
    GIT_TAG v1.3.2
    GIT_SHALLOW TRUE
)
set(SKIP_PERFORMANCE_COMPARISON ON CACHE BOOL "" FORCE)
set(BUILD_SANDBOX OFF CACHE BOOL "" FORCE)
set(BUILD_DOC OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(cereal)

option(ENABLE_AUTOC_TESTS "Build autoc unit tests" ON)
if(ENABLE_AUTOC_TESTS)
  include(GoogleTest)
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_Declare(
      googletest
      URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
  )
  FetchContent_MakeAvailable(googletest)
endif()

option(USE_SANITIZER "Build with Address Sanitizer" OFF)
option(ENABLE_PROFILING "Build with frame pointers and debug info for perf/FlameGraph" OFF)
option(PERFORMANCE_BUILD "Build with high performance flags (keeps debug symbols for core dumps)" OFF)

set(CMAKE_EXE_LINKER_FLAGS "-no-pie")

if(USE_SANITIZER)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address")
endif()

if(ENABLE_PROFILING)
    message(STATUS "Enabling profiling-friendly compiler flags for autoc")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -g -fno-omit-frame-pointer -fno-optimize-sibling-calls")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 -g -fno-omit-frame-pointer -fno-optimize-sibling-calls")
endif()

if(PERFORMANCE_BUILD)
    message(STATUS "Enabling performance-optimized build for autoc (with debug symbols)")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -march=native -mtune=native -funroll-loops -flto=auto")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -g -march=native -mtune=native -funroll-loops -flto=auto")
endif()

include(${VTK_USE_FILE})
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${VTK_INCLUDE_DIRS}
    ${AWSSDK_INCLUDE_DIRS}
)
add_compile_options(-Wno-write-strings)

# Shared library for common sources (compiled once, linked by all executables)
add_library(autoc_common STATIC
    src/util/config.cc
    src/util/logger.cc
    src/util/scenario_prng.cc
    src/util/s3_run_selector.cc

    src/eval/pathgen.cc
    src/eval/sensor_math.cc
    src/eval/fitness_computer.cc
    src/eval/fitness_decomposition.cc
    src/eval/selection.cc
    src/eval/eval_logger.cc
    src/eval/source_dmp_loader.cc
    src/eval/camera_projection.cc
    src/eval/arena.cc
    src/eval/trail_rabbit.cc
    src/eval/crash_hull.cc
    src/eval/tracker_stepper.cc
    src/nn/evaluator.cc
    src/nn/serialization.cc
    src/nn/population.cc
    src/nn/telemetry.cc
    src/nn/mode.cc
)
target_link_libraries(autoc_common inih cereal::cereal Eigen3::Eigen ${ZSTD_LIBRARY})

add_executable(autoc src/autoc.cc)
add_executable(renderer tools/renderer.cc)
add_executable(nnextractor tools/nnextractor.cc)
add_executable(nn2cpp tools/nn2cpp.cc)
add_executable(source_dmp_inspect tools/source_dmp_inspect.cc)
add_executable(tracker_dmp_inspect tools/tracker_dmp_inspect.cc)
add_executable(dmp-dump tools/dmp_dump.cc)  # 035 FR-P02/P06

target_link_libraries(autoc autoc_common ${AWSSDK_LIBRARIES})
target_link_libraries(renderer autoc_common ${VTK_LIBRARIES} ${AWSSDK_LIBRARIES})
# 039 T032: -x mode reads the binary flight log (shared format header)
target_include_directories(renderer PRIVATE xiao/include)
target_link_libraries(nnextractor autoc_common ${AWSSDK_LIBRARIES})
target_link_libraries(dmp-dump autoc_common ${AWSSDK_LIBRARIES})
target_link_libraries(nn2cpp autoc_common)
target_link_libraries(source_dmp_inspect autoc_common ${AWSSDK_LIBRARIES})
target_link_libraries(tracker_dmp_inspect autoc_common ${AWSSDK_LIBRARIES})

if(ENABLE_AUTOC_TESTS)
  add_executable(nn_evaluator_tests tests/nn_evaluator_tests.cc)
  target_link_libraries(nn_evaluator_tests GTest::gtest GTest::gtest_main autoc_common)

  add_executable(nn_serialization_tests tests/nn_serialization_tests.cc)
  target_link_libraries(nn_serialization_tests GTest::gtest GTest::gtest_main autoc_common)

  add_executable(nn_population_tests tests/nn_population_tests.cc)
  target_link_libraries(nn_population_tests GTest::gtest GTest::gtest_main autoc_common)

  add_executable(fitness_computer_tests tests/fitness_computer_tests.cc)
  target_link_libraries(fitness_computer_tests GTest::gtest GTest::gtest_main autoc_common)

  add_executable(fitness_decomposition_tests tests/fitness_decomposition_tests.cc)
  target_link_libraries(fitness_decomposition_tests GTest::gtest GTest::gtest_main autoc_common ${AWSSDK_LIBRARIES})

  add_executable(selection_tests tests/selection_tests.cc)
  target_link_libraries(selection_tests GTest::gtest GTest::gtest_main autoc_common)

  add_executable(contract_evaluator_tests tests/contract_evaluator_tests.cc)
  target_link_libraries(contract_evaluator_tests GTest::gtest GTest::gtest_main autoc_common)

  add_executable(contract_config_tests tests/contract_config_tests.cc)
  target_link_libraries(contract_config_tests GTest::gtest GTest::gtest_main autoc_common)

  add_executable(contract_evolution_tests tests/contract_evolution_tests.cc)
  target_link_libraries(contract_evolution_tests GTest::gtest GTest::gtest_main autoc_common)

  add_executable(rpc_transport_tests tests/rpc_transport_tests.cc)
  target_link_libraries(rpc_transport_tests GTest::gtest GTest::gtest_main)

  add_executable(msplink_quat_convention_tests tests/msplink_quat_convention_tests.cc)
  target_link_libraries(msplink_quat_convention_tests GTest::gtest GTest::gtest_main autoc_common)

  add_executable(nn_telemetry_tests tests/nn_telemetry_tests.cc)
  target_link_libraries(nn_telemetry_tests GTest::gtest GTest::gtest_main autoc_common)

  add_executable(cereal_version_anchor_tests tests/cereal_version_anchor_tests.cc)
  target_link_libraries(cereal_version_anchor_tests GTest::gtest GTest::gtest_main autoc_common)

  add_executable(nn_sensor_interface_tests tests/nn_sensor_interface_tests.cc)
  target_link_libraries(nn_sensor_interface_tests GTest::gtest GTest::gtest_main autoc_common)

  add_executable(source_dmp_loading_tests tests/source_dmp_loading_tests.cc)
  target_link_libraries(source_dmp_loading_tests GTest::gtest GTest::gtest_main autoc_common ${AWSSDK_LIBRARIES})

  add_executable(source_dmp_s3_integration_tests tests/source_dmp_s3_integration_tests.cc)
  target_link_libraries(source_dmp_s3_integration_tests GTest::gtest GTest::gtest_main autoc_common ${AWSSDK_LIBRARIES})

  add_executable(beacon_projection_tests tests/beacon_projection_tests.cc)
  target_link_libraries(beacon_projection_tests GTest::gtest GTest::gtest_main autoc_common)

  add_executable(contract_tracker_config_tests tests/contract_tracker_config_tests.cc)
  target_link_libraries(contract_tracker_config_tests GTest::gtest GTest::gtest_main autoc_common)
  target_compile_definitions(contract_tracker_config_tests PRIVATE
    AUTOC_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

  add_executable(gather_tracker_inputs_tests tests/gather_tracker_inputs_tests.cc)
  target_link_libraries(gather_tracker_inputs_tests GTest::gtest GTest::gtest_main autoc_common)

  add_executable(situational_inputs_tests tests/situational_inputs_tests.cc)
  target_link_libraries(situational_inputs_tests GTest::gtest GTest::gtest_main autoc_common)

  add_executable(predictor_head_tests tests/predictor_head_tests.cc)
  target_link_libraries(predictor_head_tests GTest::gtest GTest::gtest_main autoc_common)

  add_executable(tracker_dmp_roundtrip_tests tests/tracker_dmp_roundtrip_tests.cc)
  target_link_libraries(tracker_dmp_roundtrip_tests GTest::gtest GTest::gtest_main autoc_common)

  add_executable(tracker_stepper_init_tests tests/tracker_stepper_init_tests.cc)
  target_link_libraries(tracker_stepper_init_tests GTest::gtest GTest::gtest_main autoc_common)

  add_executable(arena_tests tests/arena_tests.cc)
  target_link_libraries(arena_tests GTest::gtest GTest::gtest_main autoc_common)

  add_executable(trail_rabbit_tests tests/trail_rabbit_tests.cc)
  target_link_libraries(trail_rabbit_tests GTest::gtest GTest::gtest_main autoc_common)

  add_executable(crash_hull_tests tests/crash_hull_tests.cc)
  target_link_libraries(crash_hull_tests GTest::gtest GTest::gtest_main autoc_common)

  add_executable(derived_features_tests tests/derived_features_tests.cc)
  target_link_libraries(derived_features_tests GTest::gtest GTest::gtest_main autoc_common)

  add_executable(scenario_prng_tests tests/scenario_prng_tests.cc)
  target_link_libraries(scenario_prng_tests GTest::gtest GTest::gtest_main autoc_common)

  # 034 US4 — craft variation contract tests (T043).
  add_executable(craft_variation_tests tests/craft_variation_tests.cc)
  target_link_libraries(craft_variation_tests GTest::gtest GTest::gtest_main autoc_common)

  # 034 US3 — run-id prefix routing test (T034).
  add_executable(run_id_prefix_tests tests/run_id_prefix_tests.cc)
  target_link_libraries(run_id_prefix_tests GTest::gtest GTest::gtest_main autoc_common)

  # 035 FR-P07 — shared S3 run-selector tests (pulls s3_run_selector.o → needs AWSSDK).
  add_executable(s3_run_selector_tests tests/s3_run_selector_tests.cc)
  target_link_libraries(s3_run_selector_tests GTest::gtest GTest::gtest_main autoc_common ${AWSSDK_LIBRARIES})

  # 035 FR-P09 — zstd round-trip + ratio spike for the dmp boundary.
  add_executable(dmp_dump_tests tests/dmp_dump_tests.cc)
  target_link_libraries(dmp_dump_tests GTest::gtest GTest::gtest_main autoc_common ${AWSSDK_LIBRARIES})

  # 035 FR-001b/R1 (US1) — convex throttle-energy metric.
  add_executable(energy_metric_tests tests/energy_metric_tests.cc)
  target_link_libraries(energy_metric_tests GTest::gtest GTest::gtest_main autoc_common)

  # 037 T003 -- cadence-invariance + tick-rescale scaffolding (bodies: T014/T015).
  add_executable(cadence_invariance_tests tests/cadence_invariance_tests.cc)
  target_link_libraries(cadence_invariance_tests GTest::gtest GTest::gtest_main autoc_common)

  add_executable(tick_rescale_tests tests/tick_rescale_tests.cc)
  target_link_libraries(tick_rescale_tests GTest::gtest GTest::gtest_main autoc_common)

  # 037 T016 -- NN-layout round-trip + history-layout fail-loud.
  add_executable(nn_layout_tests tests/nn_layout_tests.cc)
  target_link_libraries(nn_layout_tests GTest::gtest GTest::gtest_main autoc_common)

  # 039 T006/T007 -- engage-scoped arena re-centering (FR-001, D5).
  add_executable(arena_recenter_tests tests/arena_recenter_tests.cc)
  target_link_libraries(arena_recenter_tests GTest::gtest GTest::gtest_main autoc_common)

  # 039 T011/T012 -- versioned binary flight log round-trip (US3, FR-007/008/009).
  # Compiles the SAME xiao/include/flight_log_format.h the firmware uses
  # (single shared format definition; no autoc_common needed — self-contained).
  add_executable(flightlog_roundtrip_tests tests/flightlog_roundtrip_tests.cc)
  target_link_libraries(flightlog_roundtrip_tests GTest::gtest GTest::gtest_main)
  target_include_directories(flightlog_roundtrip_tests PRIVATE xiao/include)

  # 039 T003/T004 -- nn2cpp unrolled-recurrent emission equivalence (D6).
  # Drives the real nn2cpp binary + g++ harness compiles; paths injected here.
  add_executable(nn2cpp_unroll_tests tests/nn2cpp_unroll_tests.cc)
  target_link_libraries(nn2cpp_unroll_tests GTest::gtest GTest::gtest_main autoc_common)
  add_dependencies(nn2cpp_unroll_tests nn2cpp)
  target_compile_definitions(nn2cpp_unroll_tests PRIVATE
    NN2CPP_BIN="$<TARGET_FILE:nn2cpp>"
    AUTOC_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
    AUTOC_EIGEN_INC="${EIGEN3_INCLUDE_DIR}"
    AUTOC_CEREAL_INC="${cereal_SOURCE_DIR}/include")

  enable_testing()
  add_test(NAME nn_evaluator_tests COMMAND nn_evaluator_tests)
  add_test(NAME nn_serialization_tests COMMAND nn_serialization_tests)
  add_test(NAME nn_population_tests COMMAND nn_population_tests)
  add_test(NAME fitness_computer_tests COMMAND fitness_computer_tests)
  add_test(NAME fitness_decomposition_tests COMMAND fitness_decomposition_tests)
  add_test(NAME selection_tests COMMAND selection_tests)
  add_test(NAME contract_evaluator_tests COMMAND contract_evaluator_tests)
  add_test(NAME contract_config_tests COMMAND contract_config_tests)
  add_test(NAME contract_evolution_tests COMMAND contract_evolution_tests)
  add_test(NAME rpc_transport_tests COMMAND rpc_transport_tests)
  add_test(NAME msplink_quat_convention_tests COMMAND msplink_quat_convention_tests)
  add_test(NAME nn_telemetry_tests COMMAND nn_telemetry_tests)
  add_test(NAME cereal_version_anchor_tests COMMAND cereal_version_anchor_tests)
  add_test(NAME nn_sensor_interface_tests COMMAND nn_sensor_interface_tests)
  add_test(NAME source_dmp_loading_tests COMMAND source_dmp_loading_tests)
  add_test(NAME source_dmp_s3_integration_tests COMMAND source_dmp_s3_integration_tests)
  add_test(NAME beacon_projection_tests COMMAND beacon_projection_tests)
  add_test(NAME contract_tracker_config_tests COMMAND contract_tracker_config_tests)
  add_test(NAME gather_tracker_inputs_tests COMMAND gather_tracker_inputs_tests)
  add_test(NAME situational_inputs_tests COMMAND situational_inputs_tests)
  add_test(NAME predictor_head_tests COMMAND predictor_head_tests)
  add_test(NAME tracker_dmp_roundtrip_tests COMMAND tracker_dmp_roundtrip_tests)
  add_test(NAME tracker_stepper_init_tests COMMAND tracker_stepper_init_tests)
  add_test(NAME arena_tests COMMAND arena_tests)
  add_test(NAME trail_rabbit_tests COMMAND trail_rabbit_tests)
  add_test(NAME crash_hull_tests COMMAND crash_hull_tests)
  add_test(NAME derived_features_tests COMMAND derived_features_tests)
  add_test(NAME scenario_prng_tests COMMAND scenario_prng_tests)
  add_test(NAME craft_variation_tests COMMAND craft_variation_tests)
  add_test(NAME run_id_prefix_tests COMMAND run_id_prefix_tests)
  add_test(NAME s3_run_selector_tests COMMAND s3_run_selector_tests)
  add_test(NAME dmp_dump_tests COMMAND dmp_dump_tests)
  add_test(NAME energy_metric_tests COMMAND energy_metric_tests)
  add_test(NAME cadence_invariance_tests COMMAND cadence_invariance_tests)
  add_test(NAME tick_rescale_tests COMMAND tick_rescale_tests)
  add_test(NAME nn_layout_tests COMMAND nn_layout_tests)
  add_test(NAME nn2cpp_unroll_tests COMMAND nn2cpp_unroll_tests)
  add_test(NAME arena_recenter_tests COMMAND arena_recenter_tests)
  add_test(NAME flightlog_roundtrip_tests COMMAND flightlog_roundtrip_tests)

  # Run all tests after build
  add_custom_target(run_autoc_tests ALL
    DEPENDS nn_evaluator_tests nn_serialization_tests nn_population_tests
            fitness_computer_tests fitness_decomposition_tests selection_tests
            contract_evaluator_tests contract_config_tests
            contract_evolution_tests rpc_transport_tests
            msplink_quat_convention_tests nn_telemetry_tests
            cereal_version_anchor_tests nn_sensor_interface_tests
            source_dmp_loading_tests source_dmp_s3_integration_tests
            beacon_projection_tests
            contract_tracker_config_tests
            gather_tracker_inputs_tests
            situational_inputs_tests
            predictor_head_tests
            tracker_dmp_roundtrip_tests
            tracker_stepper_init_tests
            arena_tests
            trail_rabbit_tests
            crash_hull_tests
            derived_features_tests
            scenario_prng_tests
            craft_variation_tests
            run_id_prefix_tests
            s3_run_selector_tests
            dmp_dump_tests
            energy_metric_tests
            cadence_invariance_tests
            tick_rescale_tests
            nn_layout_tests
            arena_recenter_tests
            nn2cpp_unroll_tests
            flightlog_roundtrip_tests
    COMMAND nn_evaluator_tests --gtest_brief=1
    COMMAND nn_serialization_tests --gtest_brief=1
    COMMAND nn_population_tests --gtest_brief=1
    COMMAND fitness_computer_tests --gtest_brief=1
    COMMAND fitness_decomposition_tests --gtest_brief=1
    COMMAND cadence_invariance_tests --gtest_brief=1
    COMMAND tick_rescale_tests --gtest_brief=1
    COMMAND nn_layout_tests --gtest_brief=1
    COMMAND arena_recenter_tests --gtest_brief=1
    COMMAND nn2cpp_unroll_tests --gtest_brief=1
    COMMAND flightlog_roundtrip_tests --gtest_brief=1
    COMMAND selection_tests --gtest_brief=1
    COMMAND contract_evaluator_tests --gtest_brief=1
    COMMAND contract_config_tests --gtest_brief=1
    COMMAND contract_evolution_tests --gtest_brief=1
    COMMAND rpc_transport_tests --gtest_brief=1
    COMMAND msplink_quat_convention_tests --gtest_brief=1
    COMMAND cereal_version_anchor_tests --gtest_brief=1
    COMMAND nn_sensor_interface_tests --gtest_brief=1
    COMMAND source_dmp_loading_tests --gtest_brief=1
    COMMAND source_dmp_s3_integration_tests --gtest_brief=1
    COMMAND beacon_projection_tests --gtest_brief=1
    COMMAND contract_tracker_config_tests --gtest_brief=1
    COMMAND gather_tracker_inputs_tests --gtest_brief=1
    COMMAND situational_inputs_tests --gtest_brief=1
    COMMAND predictor_head_tests --gtest_brief=1
    COMMAND tracker_dmp_roundtrip_tests --gtest_brief=1
    COMMAND tracker_stepper_init_tests --gtest_brief=1
    COMMAND arena_tests --gtest_brief=1
    COMMAND trail_rabbit_tests --gtest_brief=1
    COMMAND crash_hull_tests --gtest_brief=1
    COMMAND derived_features_tests --gtest_brief=1
    COMMAND scenario_prng_tests --gtest_brief=1
    COMMAND craft_variation_tests --gtest_brief=1
    COMMAND run_id_prefix_tests --gtest_brief=1
    COMMAND s3_run_selector_tests --gtest_brief=1
    COMMAND dmp_dump_tests --gtest_brief=1
    COMMAND energy_metric_tests --gtest_brief=1
  )
endif()

# CRRCSim flight dynamics model (optional, needs SDL/OpenGL/etc.)
if(BUILD_CRRCSIM)
  add_subdirectory(crrcsim)
endif()
