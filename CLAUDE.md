# autoc Development Guidelines

Auto-generated from all feature plans. Last updated: 2026-04-30

## Active Technologies
- C++17 (renderer, shared code), Python 3.11 (analysis scripts) + Eigen (math), VTK (renderer), cereal (data.dat parsing), blackbox-tools (INAV decode) (018-flight-analysis)
- File-based — flight logs, blackbox CSVs, eval-results/ directories, S3 for training artifacts (018-flight-analysis)
- C++17 (autoc, crrcsim), C (INAV autoc branch), C++ (xiao/PlatformIO), Python 3.11 (analysis scripts) + Eigen (math), cereal (serialization), CRRCSim LaRCSim FDM, INAV MSP protocol (019-improved-crrcsim)
- File-based — hb1_streamer.xml (model), data.dat (training output), blackbox CSV (flight data) (019-improved-crrcsim)
- C++17 (autoc, crrcsim), C (INAV autoc branch), C++ (xiao/PlatformIO), Python 3.11 (analysis) + Eigen (math), cereal (serialization), CRRCSim LaRCSim FDM, INAV MSP protocol, LSM6DS3 IMU (021-xiao-ahrs-crosscheck)
- File-based — blackbox CSV, xiao flash logs, S3 for training artifacts (021-xiao-ahrs-crosscheck)
- C++17 + Eigen (vec3/dot), inih (config parsing), cereal (serialization), GoogleTest (022-tracking-cone-fitness)
- File-based (autoc.ini, data.dat, data.stc) (022-tracking-cone-fitness)
- C++17 (autoc, crrcsim, xiao), Python 3.11 (analysis scripts, data.dat parsers) + Eigen (vec3/dot), cereal (serialization), inih (config), GoogleTest, CRRCSim LaRCSim FDM, INAV MSP protocol, PlatformIO (xiao target) (023-ood-and-engage-fixes)
- File-based — `autoc.ini`, `data.dat`, `data.stc`, xiao flash logs, S3 for training artifacts (023-ood-and-engage-fixes)
- C++17 (autoc, crrcsim), Python 3.11 (analysis) + Eigen (math), cereal (serialization), GoogleTest (023-ood-and-engage-fixes)
- File-based — data.dat, data.stc, NN01 weights (023-ood-and-engage-fixes)
- C++17 (autoc, crrcsim, xiao), Python 3.11 (analysis scripts) + Eigen (vec3/quat math), cereal, inih, GoogleTest, (024-sim-real-fidelity)
- File-based — `data.dat` (sim), blackbox CSV (flight), xiao flash logs (024-sim-real-fidelity)
- C++17 (autoc, crrcsim, xiao), Python 3.11 (analysis & plotting) + Eigen (vec3/dot, mat-vec), cereal (NN serialization), inih (autoc.ini), (028-deeper-rnn)
- File-based — `data.dat` (training), evolution log lines (per-gen telemetry), (028-deeper-rnn)
- C++17 (autoc, crrcsim, xiao), Python 3.11 (analysis scripts, conversion tools) + Eigen (vec3 / quat / matrix-vector math, projection geometry), cereal (NN serialization, EvalResults schema, new tracker-mode dump format), inih (autoc.ini parsing), GoogleTest (unit/contract tests), CRRCSim LaRCSim FDM (sim physics) (029-tracker-mode)
- file-based — `data.dat` (training output, schema extends per FR-015), evolution log lines (per-gen telemetry), `data.stc`, S3 `.dmp` (cereal-serialized `EvalResults` — schema bump for tracker-mode dumps), playback library files (crrcsim binary playback format for v1, per spec assumption pending R1 research finding) (029-tracker-mode)

- C++17 + Eigen, cereal (serialization), inih (config), GoogleTest (015-nn-training-improvements)

## Project Structure

```text
src/
tests/
```

## Commands

# Add commands for C++17

## Code Style

C++17: Follow standard conventions

## Recent Changes
- 029-tracker-mode: Added C++17 (autoc, crrcsim, xiao), Python 3.11 (analysis scripts, conversion tools) + Eigen (vec3 / quat / matrix-vector math, projection geometry), cereal (NN serialization, EvalResults schema, new tracker-mode dump format), inih (autoc.ini parsing), GoogleTest (unit/contract tests), CRRCSim LaRCSim FDM (sim physics)
- 028-deeper-rnn: Added C++17 (autoc, crrcsim, xiao), Python 3.11 (analysis & plotting) + Eigen (vec3/dot, mat-vec), cereal (NN serialization), inih (autoc.ini),
- 024-sim-real-fidelity: Added C++17 (autoc, crrcsim, xiao), Python 3.11 (analysis scripts) + Eigen (vec3/quat math), cereal, inih, GoogleTest,


<!-- MANUAL ADDITIONS START -->
<!-- MANUAL ADDITIONS END -->
