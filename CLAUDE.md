# autoc Development Guidelines

Auto-generated from all feature plans. Last updated: 2026-03-20

## Active Technologies
- C++17 (renderer, shared code), Python 3.11 (analysis scripts) + Eigen (math), VTK (renderer), cereal (data.dat parsing), blackbox-tools (INAV decode) (018-flight-analysis)
- File-based — flight logs, blackbox CSVs, eval-results/ directories, S3 for training artifacts (018-flight-analysis)

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
- 018-flight-analysis: Added C++17 (renderer, shared code), Python 3.11 (analysis scripts) + Eigen (math), VTK (renderer), cereal (data.dat parsing), blackbox-tools (INAV decode)

- 015-nn-training-improvements: Added C++17 + Eigen, cereal (serialization), inih (config), GoogleTest

<!-- MANUAL ADDITIONS START -->
<!-- MANUAL ADDITIONS END -->
