# autoc Development Guidelines

Auto-generated from all feature plans. Last updated: 2026-04-08

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
- 023-ood-and-engage-fixes: Added C++17 (autoc, crrcsim, xiao), Python 3.11 (analysis scripts, data.dat parsers) + Eigen (vec3/dot), cereal (serialization), inih (config), GoogleTest, CRRCSim LaRCSim FDM, INAV MSP protocol, PlatformIO (xiao target)
- 022-tracking-cone-fitness: Added C++17 + Eigen (vec3/dot), inih (config parsing), cereal (serialization), GoogleTest
- 021-xiao-ahrs-crosscheck: Added C++17 (autoc, crrcsim), C (INAV autoc branch), C++ (xiao/PlatformIO), Python 3.11 (analysis) + Eigen (math), cereal (serialization), CRRCSim LaRCSim FDM, INAV MSP protocol, LSM6DS3 IMU


<!-- MANUAL ADDITIONS START -->
<!-- MANUAL ADDITIONS END -->
