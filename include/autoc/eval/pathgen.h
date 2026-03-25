// Path generation for GP training
//
// USAGE PATTERN FOR autoc.cc:
// TODO: Update autoc.cc to pass RandomPathSeedB from config manager as baseSeed parameter
//       Example usage:
//         unsigned int seed = extraConfig.randomPathSeedB;  // Read from .ini (default: 67890)
//         auto paths = generateSmoothPaths("aeroStandard", 6, radius, height, seed);
//
//       For 'aeroStandard': All 6 paths use the SAME seed (paths are deterministic based on pathIndex)
//       For 'random': Each path gets a DIFFERENT seed (generated from baseSeed using mt19937)
//
#ifndef PATHGEN_H
#define PATHGEN_H

#include <vector>
#include <random>

#include "autoc/rpc/protocol.h"
#include "autoc/types.h"

#define NUM_SEGMENTS_PER_PATH 16

// Maximum path length in meters (geometry-only paths have no timing)
constexpr gp_scalar MAX_PATH_LENGTH_M = 2000.0f;

gp_vec3 randomPointInCylinder(gp_scalar radius, gp_scalar height, gp_scalar base = 0.0f);
gp_vec3 cubicInterpolate(const gp_vec3& p0, const gp_vec3& p1, const gp_vec3& p2, const gp_vec3& p3, gp_scalar t);
// Generate paths with controllable random seed
// For 'aeroStandard' and deterministic methods: use ONE seed for all numPaths
// For 'random' method: generate a NEW seed for each path
std::vector<std::vector<Path>> generateSmoothPaths(const std::string& method, int numPaths, gp_scalar radius, gp_scalar height, unsigned int baseSeed);

class GeneratorMethod {
public:
  virtual ~GeneratorMethod() = default;
  // pathIndex: which path variant to generate (0-5 for aeroStandard, etc)
  // seed: random seed for reproducible generation (creates private PRNG instance)
  // NOTE: Callers should use ONE seed for all paths in aeroStandard/computedPaths,
  //       but generate NEW seeds per-path for 'random' method
  virtual std::vector<Path> method(int pathIndex, gp_scalar radius, gp_scalar height, gp_scalar base, unsigned int seed) = 0;

protected:
  // Generate random point in cylinder using local mt19937 PRNG (does not affect global rand state)
  static gp_vec3 localRandomPointInCylinder(std::mt19937& rng, gp_scalar radius, gp_scalar height, gp_scalar base = 0.0f) {
    std::uniform_real_distribution<gp_scalar> dist(static_cast<gp_scalar>(0.0), static_cast<gp_scalar>(1.0));
    gp_scalar r = radius * std::cbrt(dist(rng));
    gp_scalar theta = dist(rng) * static_cast<gp_scalar>(M_PI * 2.0);
    gp_scalar z = base - dist(rng) * height;
    gp_scalar x = r * std::cos(theta);
    gp_scalar y = r * std::sin(theta);
    return gp_vec3(x, y, z);
  }
};

class GenerateRandom : public GeneratorMethod {
  std::vector<Path> method(int pathIndex, gp_scalar radius, gp_scalar height, gp_scalar base, unsigned int seed) override {
    std::vector<gp_vec3> controlPoints;
    std::vector<Path> path;
    int numPoints = NUM_SEGMENTS_PER_PATH;

    // Initialize local mt19937 PRNG (does not affect global rand())
    std::mt19937 rng(seed);

    // Generate random control points at canonical origin
    for (size_t i = 0; i < numPoints; ++i) {
      controlPoints.push_back(localRandomPointInCylinder(rng, radius, height, 0.0f));
    }

    // Ensure the path is continuous by looping through control points
    gp_scalar odometer = 0;
    gp_scalar turnmeter = 0;
    gp_vec3 lastPoint;
    gp_vec3 lastDirection;
    bool first = true;

    for (size_t i = 1; i < controlPoints.size() - 3; ++i) {
      for (gp_scalar t = 0; t <= 1; t += static_cast<gp_scalar>(0.02f)) {
        gp_vec3 interpolatedPoint = cubicInterpolate(controlPoints[i - 1], controlPoints[i], controlPoints[i + 1], controlPoints[i + 2], t);
        gp_vec3 newDirection;

        if (!first) {
          // Compute the new distance
          gp_scalar newDistance = (interpolatedPoint - lastPoint).norm();

          // Compute new new direction
          newDirection = (interpolatedPoint - lastPoint).normalized();

          // difference to prior direction
          gp_scalar dVector = lastDirection.dot(newDirection);
          gp_scalar dAngle = std::acos(std::clamp(dVector / (lastDirection.norm() * newDirection.norm()), -1.0f, 1.0f));

          // add next segment
          Path pathSegment = Path(interpolatedPoint, gp_vec3::UnitX(), odometer, turnmeter);
          path.push_back(pathSegment);

          odometer += newDistance;
          turnmeter += dAngle;
        }
        else {
          first = false;
        }

        lastPoint = interpolatedPoint;
        lastDirection = newDirection;

        // stop at max path length
        if (odometer > MAX_PATH_LENGTH_M) {
          goto exitLoop;
        }
      }
    }
  exitLoop:

    return path;
  }
};

class GenerateClassic : public GeneratorMethod {
  std::vector<Path> method(int pathIndex, gp_scalar radius, gp_scalar height, gp_scalar base, unsigned int seed) override {
    // Classic path is deterministic, seed is unused
    std::vector<gp_vec3> controlPoints;
    std::vector<Path> path;
    int numPoints = NUM_SEGMENTS_PER_PATH;

    controlPoints.push_back({ 0, 0, 0 });

    // Sin
    gp_scalar x, y, z;
    for (size_t i = 0; i < numPoints; ++i) {
      x = -(std::cos(static_cast<gp_scalar>(2 * M_PI) * static_cast<gp_scalar>(i) / static_cast<gp_scalar>(numPoints)) * SIM_PATH_BOUNDS / static_cast<gp_scalar>(2.0f) - SIM_PATH_BOUNDS / static_cast<gp_scalar>(2.0f));
      y = std::sin(static_cast<gp_scalar>(2 * M_PI) * static_cast<gp_scalar>(i) / static_cast<gp_scalar>(numPoints)) * SIM_PATH_BOUNDS / static_cast<gp_scalar>(2.0f);
      z = -static_cast<gp_scalar>(i);
      controlPoints.push_back(gp_vec3(x, y, z));
    }

    gp_scalar lastX = x;
    gp_scalar lastY = y;
    gp_scalar lastZ = z;
    for (size_t i = 0; i < numPoints; ++i) {
      y = lastX + std::sin(static_cast<gp_scalar>(2 * M_PI) * static_cast<gp_scalar>(i) / static_cast<gp_scalar>(numPoints)) * SIM_PATH_BOUNDS / static_cast<gp_scalar>(3.0f);
      z = lastZ - SIM_PATH_BOUNDS / static_cast<gp_scalar>(2.0f) + std::cos(static_cast<gp_scalar>(2 * M_PI) * static_cast<gp_scalar>(i) / static_cast<gp_scalar>(numPoints)) * SIM_PATH_BOUNDS / static_cast<gp_scalar>(2.0f);
      x = lastY - static_cast<gp_scalar>(i);
      controlPoints.push_back(gp_vec3(x, y, z));
    }

    // Ensure the path is continuous by looping through control points
    gp_scalar odometer = 0;
    gp_scalar turnmeter = 0;
    gp_vec3 lastPoint;
    gp_vec3 lastDirection;
    bool first = true;

    for (size_t i = 1; i < controlPoints.size() - 3; ++i) {
      for (gp_scalar t = 0; t <= 1; t += static_cast<gp_scalar>(0.02f)) {
        gp_vec3 interpolatedPoint = cubicInterpolate(controlPoints[i - 1], controlPoints[i], controlPoints[i + 1], controlPoints[i + 2], t);
        gp_vec3 newDirection;

        if (!first) {
          // Compute the new distance
          gp_scalar newDistance = (interpolatedPoint - lastPoint).norm();

          // Compute new new direction
          newDirection = (interpolatedPoint - lastPoint).normalized();

          // difference to prior direction
          gp_scalar dVector = lastDirection.dot(newDirection);
          gp_scalar dAngle = std::acos(std::clamp(dVector / (lastDirection.norm() * newDirection.norm()), -1.0f, 1.0f));

          // add next segment
          Path pathSegment = Path(interpolatedPoint, gp_vec3::UnitX(), odometer, turnmeter);
          path.push_back(pathSegment);

          odometer += newDistance;
          turnmeter += dAngle;
        }
        else {
          first = false;
        }

        lastPoint = interpolatedPoint;
        lastDirection = newDirection;

        // stop at max path length
        if (odometer > MAX_PATH_LENGTH_M) {
          goto exitLoop;
        }
      }
    }
  exitLoop:

    return path;
  }
};

enum PathType {
  HorizontalCircle = 0,
  VerticalCircle,
  FigureEight,
  HorizontalSquare,
  VerticalSquare,
  FortyFiveLoop,
  END_MARKER
};

class GenerateComputedPaths : public GeneratorMethod {
  std::vector<Path> method(int pathIndex, gp_scalar radius, gp_scalar height, gp_scalar base, unsigned int seed) override {
    // Computed paths are deterministic, seed is unused
    std::vector<Path> path;

    PathType pathType = static_cast<PathType>(pathIndex % static_cast<int>(PathType::END_MARKER));

    switch(pathType) {
      case PathType::HorizontalCircle: {
        gp_scalar cSize = static_cast<gp_scalar>(0.8f) * radius;
        for (gp_scalar turn = 0; turn < static_cast<gp_scalar>(M_PI * 2.0); turn += static_cast<gp_scalar>(0.05f)) {
          gp_vec3 interpolatedPoint = { cSize * std::cos(turn), cSize * std::sin(turn), -SIM_INITIAL_LOCATION_DITHER };
          path.push_back(Path(interpolatedPoint, gp_vec3::UnitX(), turn * radius, 0.0f));
        }
        break;
      }
      case PathType::VerticalCircle: {
        gp_scalar cSize = static_cast<gp_scalar>(0.8f) * radius;
        for (gp_scalar turn = 0; turn < static_cast<gp_scalar>(M_PI * 2.0); turn += static_cast<gp_scalar>(0.05f)) {
          gp_vec3 interpolatedPoint = { cSize * std::cos(turn), 0.0f, -SIM_INITIAL_LOCATION_DITHER - cSize * std::sin(turn)};
          path.push_back(Path(interpolatedPoint, gp_vec3::UnitX(), turn * cSize, 0.0f));
        }
        break;
      }
      case PathType::FigureEight: {
        gp_scalar cSize = static_cast<gp_scalar>(0.4f) * radius;
        for (gp_scalar turn = 0; turn < static_cast<gp_scalar>(M_PI * 2.0); turn += static_cast<gp_scalar>(0.05f)) {
          gp_vec3 interpolatedPoint = { (cSize * std::cos(turn)) - cSize, 0.0f, -SIM_INITIAL_LOCATION_DITHER - (cSize * std::sin(turn))};
          path.push_back(Path(interpolatedPoint, gp_vec3::UnitX(), turn * cSize, 0.0f));
        }
        for (gp_scalar turn = 0; turn < static_cast<gp_scalar>(M_PI * 2.0); turn += static_cast<gp_scalar>(0.05f)) {
          gp_vec3 interpolatedPoint = { (-cSize * std::cos(turn)) + cSize, 0.0f, -SIM_INITIAL_LOCATION_DITHER - (cSize * std::sin(turn))};
          path.push_back(Path(interpolatedPoint, gp_vec3::UnitX(), (turn * cSize) + (static_cast<gp_scalar>(M_PI * 2.0) * cSize), 0.0f));
        }
        break;
      }
      case PathType::HorizontalSquare: {
        gp_scalar sSize = static_cast<gp_scalar>(0.7f) * radius;
        for (gp_scalar span = -sSize; span < sSize; span += static_cast<gp_scalar>(0.1f)) {
          gp_vec3 interpolatedPoint = { span, -sSize, -SIM_INITIAL_LOCATION_DITHER };
          path.push_back(Path(interpolatedPoint, gp_vec3::UnitX(), span, 0.0f));
        }
        for (gp_scalar span = -sSize; span < sSize; span += static_cast<gp_scalar>(0.1f)) {
          gp_vec3 interpolatedPoint = { sSize, span, -SIM_INITIAL_LOCATION_DITHER };
          path.push_back(Path(interpolatedPoint, gp_vec3::UnitX(), static_cast<gp_scalar>(2.0f) * sSize + span, 0.0f));
        }
        for (gp_scalar span = -sSize; span < sSize; span += static_cast<gp_scalar>(0.1f)) {
          gp_vec3 interpolatedPoint = { -span, sSize, -SIM_INITIAL_LOCATION_DITHER };
          path.push_back(Path(interpolatedPoint, gp_vec3::UnitX(), static_cast<gp_scalar>(4.0f) * sSize + span, 0.0f));
        }
        for (gp_scalar span = -sSize; span < sSize; span += static_cast<gp_scalar>(0.1f)) {
          gp_vec3 interpolatedPoint = { -sSize, -span, -SIM_INITIAL_LOCATION_DITHER };
          path.push_back(Path(interpolatedPoint, gp_vec3::UnitX(), static_cast<gp_scalar>(6.0f) * sSize + span, 0.0f));
        }
        break;
      }
      case PathType::VerticalSquare: {
        gp_scalar sSize = static_cast<gp_scalar>(0.7f) * radius;
        for (gp_scalar span = -sSize; span < sSize; span += static_cast<gp_scalar>(0.1f)) {
          gp_vec3 interpolatedPoint = { span, 0.0f, -SIM_INITIAL_LOCATION_DITHER - sSize };
          path.push_back(Path(interpolatedPoint, gp_vec3::UnitX(), span, 0.0f));
        }
        for (gp_scalar span = -sSize; span < sSize; span += static_cast<gp_scalar>(0.1f)) {
          gp_vec3 interpolatedPoint = { sSize, 0.0f, -SIM_INITIAL_LOCATION_DITHER + span };
          path.push_back(Path(interpolatedPoint, gp_vec3::UnitX(), static_cast<gp_scalar>(2.0f) * sSize + span, 0.0f));
        }
        for (gp_scalar span = -sSize; span < sSize; span += static_cast<gp_scalar>(0.1f)) {
          gp_vec3 interpolatedPoint = { -span, 0.0f, -SIM_INITIAL_LOCATION_DITHER + sSize };
          path.push_back(Path(interpolatedPoint, gp_vec3::UnitX(), static_cast<gp_scalar>(4.0f) * sSize + span, 0.0f));
        }
        for (gp_scalar span = -sSize; span < sSize; span += static_cast<gp_scalar>(0.1f)) {
          gp_vec3 interpolatedPoint = { -sSize, 0.0f, -SIM_INITIAL_LOCATION_DITHER - span };
          path.push_back(Path(interpolatedPoint, gp_vec3::UnitX(), static_cast<gp_scalar>(6.0f) * sSize + span, 0.0f));
        }
        break;
      }
      case PathType::FortyFiveLoop: {
        gp_scalar cSize = static_cast<gp_scalar>(0.8f) * radius;
        for (gp_scalar turn = 0; turn < static_cast<gp_scalar>(M_PI * 2.0); turn += static_cast<gp_scalar>(0.05f)) {
          gp_vec3 interpolatedPoint = {
            cSize * std::cos(turn),
            cSize * std::sin(turn),
            -SIM_INITIAL_LOCATION_DITHER - cSize * std::sin(turn)
          };
          path.push_back(Path(interpolatedPoint, gp_vec3::UnitX(), turn * cSize, 0.0f));
        }
        break;
      }
      default: {
        std::cerr << "Unknown path type: " << pathType << std::endl;
        assert(false);
      }
    }

    return path;
  }
};

class GenerateLine : public GeneratorMethod {
  std::vector<Path> method(int pathIndex, gp_scalar radius, gp_scalar height, gp_scalar base, unsigned int seed) override {
    // Line path is deterministic, seed is unused

    std::vector<Path> path;
    gp_scalar distance = 0.0f;
    for (gp_scalar i = -SIM_PATH_RADIUS_LIMIT; i <= SIM_PATH_RADIUS_LIMIT; i += static_cast<gp_scalar>(5.0f)) {
      gp_vec3 interpolatedPoint = { static_cast<gp_scalar>(-30.0f), i, 0.0f };
      path.push_back(Path(interpolatedPoint, gp_vec3::UnitX(), distance, 0.0f));
      distance += static_cast<gp_scalar>(5.0f);
    }
    return path;
  }
};

enum AeroStandardPathType {
  StraightAndLevel = 0,
  SpiralClimb,
  HorizontalFigureEight,
  FortyFiveDegreeAngledLoop,
  HighPerchSplitS,
  SeededRandomB,
  AERO_END_MARKER  // Total: 6 paths
};

class GenerateAeroStandard : public GeneratorMethod {
public:
  std::vector<Path> method(int pathIndex, gp_scalar radius, gp_scalar height, gp_scalar base, unsigned int seed) override {
    std::vector<Path> path;
    gp_scalar totalDistance = 0.0f;

    AeroStandardPathType pathType = static_cast<AeroStandardPathType>(pathIndex % static_cast<int>(AeroStandardPathType::AERO_END_MARKER));

    // Entry point: canonical origin at (0,0,0) heading south
    // NOTE: base parameter is now deprecated - coordinate offset applied at output stage
    gp_vec3 entryPoint(0.0f, 0.0f, 0.0f);

    switch(pathType) {
      case AeroStandardPathType::StraightAndLevel: {
        addStraightSegment(path, entryPoint, gp_vec3(-1.0f, 0.0f, 0.0f), 20.0f, totalDistance);
        gp_vec3 turn1Start = path.back().start;
        addHorizontalTurn(path, turn1Start, 20.0f, M_PI, true, totalDistance);
        gp_vec3 northStart = path.back().start;
        addStraightSegment(path, northStart, gp_vec3(1.0f, 0.0f, 0.0f), 40.0f, totalDistance);
        gp_vec3 turn2Start = path.back().start;
        addHorizontalTurn(path, turn2Start, 20.0f, M_PI, true, totalDistance);
        gp_vec3 returnStart = path.back().start;
        addStraightSegment(path, returnStart, gp_vec3(-1.0f, 0.0f, 0.0f), 20.0f, totalDistance);
        break;
      }

      case AeroStandardPathType::SpiralClimb: {
        addStraightSegment(path, entryPoint, gp_vec3(-1.0f, 0.0f, 0.0f), 20.0f, totalDistance);
        gp_vec3 spiralStart = path.back().start;
        gp_scalar climbAmount = -50.0f;
        addSpiralTurn(path, spiralStart, 20.0f, 540.0f * M_PI / 180.0f, true, climbAmount, totalDistance);
        gp_vec3 northStart = path.back().start;
        addStraightSegment(path, northStart, gp_vec3(1.0f, 0.0f, 0.0f), 40.0f, totalDistance);
        break;
      }

      case AeroStandardPathType::HorizontalFigureEight: {
        gp_scalar loopRadius = 20.0f;
        // Lead-in: fixed 13m straight south (was rabbitSpeed * 1s)
        const gp_scalar leadDistance = 13.0f;
        addStraightSegment(path, entryPoint, gp_vec3(-1.0f, 0.0f, 0.0f), leadDistance, totalDistance);
        gp_vec3 loopOrigin = path.back().start;
        addHorizontalLoop(path, loopOrigin, loopRadius, false, totalDistance);
        addHorizontalLoop(path, loopOrigin, loopRadius, true, totalDistance);
        break;
      }

      case AeroStandardPathType::FortyFiveDegreeAngledLoop: {
        const gp_scalar cos45 = std::sqrt(2.0f) / 2.0f;
        const gp_scalar sin45 = std::sqrt(2.0f) / 2.0f;
        gp_scalar loopRadius = 15.0f;
        gp_scalar centerAlt = -loopRadius * cos45;
        gp_scalar yOffset = loopRadius * sin45;

        for (gp_scalar turn = 0; turn < static_cast<gp_scalar>(M_PI * 2.0); turn += 0.5f) {
          gp_scalar angle = turn - static_cast<gp_scalar>(M_PI / 2.0);
          gp_scalar x_plane = -loopRadius * std::cos(angle);
          gp_scalar z_plane = loopRadius * std::sin(angle);
          gp_vec3 interpolatedPoint = {
            x_plane,
            z_plane * sin45 + yOffset,
            centerAlt - z_plane * cos45
          };
          if (!path.empty()) {
            gp_scalar distance = (interpolatedPoint - path.back().start).norm();
            totalDistance += distance;
          }
          path.push_back(Path(interpolatedPoint, gp_vec3::UnitX(), totalDistance, 0.0f));
        }
        break;
      }

      case AeroStandardPathType::HighPerchSplitS: {
        addStraightSegment(path, entryPoint, gp_vec3(-1.0f, 0.0f, 0.0f), 20.0f, totalDistance);
        gp_vec3 seg2Start = path.back().start;
        gp_scalar climbAmount = -20.0f;
        addSpiralTurn(path, seg2Start, 20.0f, M_PI, true, climbAmount, totalDistance);
        gp_vec3 seg3Start = path.back().start;
        gp_vec3 headingNorth(1.0f, 0.0f, 0.0f);
        gp_scalar horizontalDist = 40.0f;
        gp_scalar verticalClimb = -20.0f;
        gp_vec3 climbVector = headingNorth * horizontalDist + gp_vec3(0.0f, 0.0f, verticalClimb);
        gp_scalar climbDistance = climbVector.norm();
        addStraightSegment(path, seg3Start, climbVector.normalized(), climbDistance, totalDistance);
        gp_vec3 seg4Start = path.back().start;
        addHorizontalTurn(path, seg4Start, 5.0f, 150.0f * M_PI / 180.0f, true, totalDistance);
        gp_scalar headingAngle = 150.0f * M_PI / 180.0f;
        gp_vec3 headingSW(std::cos(headingAngle), std::sin(headingAngle), 0.0f);
        gp_vec3 seg5Start = path.back().start;
        addStraightSegment(path, seg5Start, headingSW, 30.0f, totalDistance);
        gp_vec3 seg6Start = path.back().start;
        if (std::abs(headingSW[0]) > 0.001f) {
          gp_scalar distToX20 = (-20.0f - seg6Start[0]) / headingSW[0];
          if (distToX20 > 0) {
            addStraightSegment(path, seg6Start, headingSW, distToX20, totalDistance);
          }
        }
        gp_vec3 seg7Start = path.back().start;
        addPitchDownLoop(path, seg7Start, headingSW, 15.0f, totalDistance);
        gp_vec3 seg8Start = path.back().start;
        gp_vec3 headingNE = -headingSW;
        if (std::abs(headingNE[0]) > 0.001f) {
          gp_scalar distToX40 = (40.0f - seg8Start[0]) / headingNE[0];
          addStraightSegment(path, seg8Start, headingNE.normalized(), std::abs(distToX40), totalDistance);
        }
        break;
      }

      case AeroStandardPathType::SeededRandomB: {
        std::mt19937 rng(seed);
        std::vector<gp_vec3> controlPoints;
        int numPoints = NUM_SEGMENTS_PER_PATH;
        for (int i = 0; i < numPoints; ++i) {
          controlPoints.push_back(localRandomPointInCylinder(rng, radius, height, 0.0f));
        }
        gp_scalar odometer = 0;
        gp_scalar turnmeter = 0;
        gp_vec3 lastPoint;
        gp_vec3 lastDirection;
        bool first = true;
        for (size_t i = 1; i < controlPoints.size() - 3; ++i) {
          for (gp_scalar t = 0; t <= 1; t += static_cast<gp_scalar>(0.02f)) {
            gp_vec3 interpolatedPoint = cubicInterpolate(controlPoints[i - 1], controlPoints[i],
                                                         controlPoints[i + 1], controlPoints[i + 2], t);
            if (!first) {
              gp_scalar newDistance = (interpolatedPoint - lastPoint).norm();
              gp_vec3 newDirection = (interpolatedPoint - lastPoint).normalized();
              gp_scalar dVector = lastDirection.dot(newDirection);
              gp_scalar dAngle = std::acos(std::clamp(dVector / (lastDirection.norm() * newDirection.norm()), -1.0f, 1.0f));
              path.push_back(Path(interpolatedPoint, gp_vec3::UnitX(), odometer, turnmeter));
              odometer += newDistance;
              turnmeter += dAngle;
              lastDirection = newDirection;
            } else {
              first = false;
              lastDirection = (interpolatedPoint - entryPoint).normalized();
            }
            lastPoint = interpolatedPoint;
          }
        }
        break;
      }

      default: {
        std::cerr << "Unknown aero standard path type: " << pathType << std::endl;
        assert(false);
      }
    }

    return path;
  }

private:
  void addStraightSegment(std::vector<Path>& path, const gp_vec3& start, const gp_vec3& direction,
                          gp_scalar distance, gp_scalar& totalDistance) {
    const gp_scalar step = 0.4f;  // 0.4m spacing (ensures ≥20Hz at min rabbit speed 8m/s)
    gp_vec3 dir = direction.normalized();

    // Start from step if path is not empty to avoid duplicating the last point
    gp_scalar startD = path.empty() ? 0.0f : step;

    for (gp_scalar d = startD; d <= distance; d += step) {
      gp_vec3 point = start + dir * d;
      if (!path.empty()) {
        gp_scalar segmentDist = (point - path.back().start).norm();
        totalDistance += segmentDist;
      }
      path.push_back(Path(point, gp_vec3::UnitX(), totalDistance, 0.0f));
    }
  }

  void addHorizontalTurn(std::vector<Path>& path, const gp_vec3& start, gp_scalar radius,
                         gp_scalar angleRadians, bool clockwise, gp_scalar& totalDistance) {
    const gp_scalar step = 0.02f;  // 0.02 rad (~1°) spacing for finer temporal resolution

    // Determine initial heading from last two points if possible
    gp_vec3 heading(-1.0f, 0.0f, 0.0f); // default south
    if (path.size() >= 2) {
      heading = (path.back().start - path[path.size()-2].start).normalized();
    }

    // Project heading onto XY plane (ignore Z component for horizontal turn)
    gp_vec3 headingXY(heading[0], heading[1], 0.0f);
    headingXY = headingXY.normalized();

    // Right perpendicular in xy plane: rotate heading 90° RIGHT (clockwise when viewed from above)
    // For heading=(-1,0,0) (south), right perpendicular is (0,-1,0) (west)
    // Formula: rotate (x,y) by -90° (clockwise) = (y, -x)
    gp_vec3 rightPerpendicular(-headingXY[1], headingXY[0], 0.0f);

    // Place center to the right for clockwise turn, left for counterclockwise
    gp_vec3 center = start + rightPerpendicular * (clockwise ? 1.0f : -1.0f) * radius;

    // Starting angle: we're at the start position relative to center
    gp_scalar startAngle = std::atan2(start[1] - center[1], start[0] - center[0]);

    // When looking down from above (down +z axis), clockwise motion means INCREASING angle
    gp_scalar angleSign = clockwise ? 1.0f : -1.0f;

    for (gp_scalar angle = step; angle <= angleRadians; angle += step) {
      gp_scalar totalAngle = startAngle + angleSign * angle;
      gp_vec3 point(center[0] + radius * std::cos(totalAngle), center[1] + radius * std::sin(totalAngle), start[2]);

      gp_scalar segmentDist = (point - path.back().start).norm();
      totalDistance += segmentDist;

      path.push_back(Path(point, gp_vec3::UnitX(), totalDistance, 0.0f));
    }
  }

  void addSpiralTurn(std::vector<Path>& path, const gp_vec3& start, gp_scalar radius,
                     gp_scalar angleRadians, bool clockwise, gp_scalar totalClimb, gp_scalar& totalDistance) {
    const gp_scalar step = 0.02f;  // 0.02 rad (~1°) spacing for finer temporal resolution

    gp_vec3 heading(-1.0f, 0.0f, 0.0f);
    if (path.size() >= 2) {
      heading = (path.back().start - path[path.size()-2].start).normalized();
    }

    // Project heading onto XY plane (ignore Z component for turn calculation)
    gp_vec3 headingXY(heading[0], heading[1], 0.0f);
    headingXY = headingXY.normalized();

    // Right perpendicular in xy plane: rotate (x,y) by -90° = (y, -x)
    gp_vec3 rightPerpendicular(-headingXY[1], headingXY[0], 0.0f);

    // Place center based on turn direction
    gp_vec3 center = start + rightPerpendicular * (clockwise ? 1.0f : -1.0f) * radius;

    // Starting angle from center to start position
    gp_scalar startAngle = std::atan2(start[1] - center[1], start[0] - center[0]);

    // When looking down from above, clockwise means increasing angle
    gp_scalar angleSign = clockwise ? 1.0f : -1.0f;

    for (gp_scalar angle = step; angle <= angleRadians; angle += step) {
      gp_scalar totalAngle = startAngle + angleSign * angle;
      gp_scalar zOffset = (angle / angleRadians) * totalClimb;
      gp_vec3 point(center[0] + radius * std::cos(totalAngle), center[1] + radius * std::sin(totalAngle), start[2] + zOffset);

      gp_scalar segmentDist = (point - path.back().start).norm();
      totalDistance += segmentDist;

      path.push_back(Path(point, gp_vec3::UnitX(), totalDistance, 0.0f));
    }
  }

  void addHorizontalLoop(std::vector<Path>& path, const gp_vec3& loopOrigin, gp_scalar loopRadius,
                         bool clockwise, gp_scalar& totalDistance) {
    const gp_scalar step = 0.02f;  // 0.02 rad (~1°) spacing for finer temporal resolution
    gp_scalar sign = clockwise ? -1.0f : 1.0f; // inverted for consistency

    for (gp_scalar turn = 0; turn < static_cast<gp_scalar>(M_PI * 2.0); turn += step) {
      gp_vec3 circleCenter = loopOrigin + gp_vec3(0.0f, sign * loopRadius, 0.0f);
      gp_vec3 point = circleCenter + gp_vec3(-loopRadius * std::sin(turn), -sign * loopRadius * std::cos(turn), 0.0f);

      if (!path.empty()) {
        gp_scalar distance = (point - path.back().start).norm();
        totalDistance += distance;
      }
      path.push_back(Path(point, gp_vec3::UnitX(), totalDistance, 0.0f));
    }
  }

  void addPitchDownLoop(std::vector<Path>& path, const gp_vec3& start, const gp_vec3& heading,
                        gp_scalar loopRadius, gp_scalar& totalDistance) {
    // Pitch-down loop: 180° loop in the vertical plane perpendicular to current heading
    // This is a Split-S maneuver - half loop downward (toward +z) that reverses course
    const gp_scalar step = 0.02f;  // 0.02 rad (~1°) spacing for finer temporal resolution

    // Heading should be normalized (only xy components matter for horizontal heading)
    gp_vec3 headingNorm = heading.normalized();

    // Find the center of the loop: radius distance DOWN (+z direction) from start position
    // In NED, +z is down, so we move the center downward
    gp_vec3 center = start + gp_vec3(0.0f, 0.0f, loopRadius);

    // Loop from 0 to π (180°), starting from top of loop heading down
    for (gp_scalar angle = step; angle <= static_cast<gp_scalar>(M_PI); angle += step) {
      // In the vertical plane aligned with heading:
      // - At angle=0: at top of loop (start position)
      // - At angle=π/2: halfway, heading straight down
      // - At angle=π: at bottom, reversed heading

      // Position along the loop arc
      // x,y offset: moves forward along heading direction as we loop
      gp_scalar horizontalOffset = loopRadius * std::sin(angle);  // 0 at top, r at middle, 0 at bottom
      // z offset: loops downward (toward +z)
      gp_scalar verticalOffset = -loopRadius * std::cos(angle);  // -r at top (angle=0), +r at bottom (angle=π)

      // Combine: move along heading (xy) and vertically (z)
      gp_vec3 point = center + headingNorm * horizontalOffset + gp_vec3(0.0f, 0.0f, verticalOffset);

      gp_scalar segmentDist = (point - path.back().start).norm();
      totalDistance += segmentDist;

      path.push_back(Path(point, gp_vec3::UnitX(), totalDistance, 0.0f));
    }
  }
};

// Progressive Distance Path - single long path with increasing difficulty
// Sequence: Racetrack -> Figure-8 -> 45° Loop -> Spiral Climb -> Split-S
// All segments return to origin, no lead-ins needed after first segment
class GenerateProgressiveDistance : public GeneratorMethod {
public:
  std::vector<Path> method(int pathIndex, gp_scalar radius, gp_scalar height, gp_scalar base, unsigned int seed) override {
    std::vector<Path> path;
    gp_scalar totalDistance = 0.0f;

    gp_vec3 raceStart(0.0f, 0.0f, 0.0f);
    addStraightSegment(path, raceStart, gp_vec3(-1.0f, 0.0f, 0.0f), 20.0f, totalDistance);
    addHorizontalTurn(path, path.back().start, 20.0f, M_PI, true, totalDistance);
    addStraightSegment(path, path.back().start, gp_vec3(1.0f, 0.0f, 0.0f), 40.0f, totalDistance);
    addHorizontalTurn(path, path.back().start, 20.0f, M_PI, true, totalDistance);
    addStraightSegment(path, path.back().start, gp_vec3(-1.0f, 0.0f, 0.0f), 20.0f, totalDistance);

    gp_scalar loopRadius = 20.0f;
    gp_vec3 loopOrigin = path.back().start;
    addHorizontalLoop(path, loopOrigin, loopRadius, false, totalDistance);
    addHorizontalLoop(path, loopOrigin, loopRadius, true, totalDistance);

    gp_vec3 phase4Start = path.back().start;
    const gp_scalar cos45 = std::sqrt(2.0f) / 2.0f;
    const gp_scalar sin45 = cos45;
    gp_scalar angledLoopRadius = 15.0f;
    gp_scalar centerAlt = -angledLoopRadius * cos45;
    gp_scalar yOffset = angledLoopRadius * sin45;

    for (gp_scalar turn = 0; turn < static_cast<gp_scalar>(M_PI * 2.0); turn += 0.05f) {
      gp_scalar angle = turn - static_cast<gp_scalar>(M_PI / 2.0);
      gp_scalar x_plane = -angledLoopRadius * std::cos(angle);
      gp_scalar z_plane = angledLoopRadius * std::sin(angle);
      gp_vec3 loopPoint(
        phase4Start[0] + x_plane,
        phase4Start[1] + z_plane * sin45 + yOffset,
        phase4Start[2] + centerAlt - z_plane * cos45
      );
      if (!path.empty()) {
        gp_scalar distance = (loopPoint - path.back().start).norm();
        totalDistance += distance;
      }
      path.push_back(Path(loopPoint, gp_vec3::UnitX(), totalDistance, 0.0f));
    }

    gp_vec3 spiralStart = path.back().start;
    gp_scalar climbAmount = -30.0f;
    addSpiralTurn(path, spiralStart, 20.0f, 540.0f * M_PI / 180.0f, true, climbAmount, totalDistance);

    gp_vec3 spiralEnd = path.back().start;
    addStraightSegment(path, spiralEnd, gp_vec3(1.0f, 0.0f, 0.0f), 20.0f, totalDistance);
    addHorizontalTurn(path, path.back().start, 15.0f, M_PI, true, totalDistance);

    gp_vec3 preSplit = path.back().start;
    gp_scalar distToSplitEntry = std::abs(preSplit[0]) - 15.0f;
    if (distToSplitEntry > 0) {
      addStraightSegment(path, preSplit, gp_vec3(-1.0f, 0.0f, 0.0f), distToSplitEntry, totalDistance);
    }

    addPitchDownLoop(path, path.back().start, gp_vec3(-1.0f, 0.0f, 0.0f), 15.0f, totalDistance);

    gp_vec3 splitSEnd = path.back().start;
    gp_scalar distToOrigin = std::sqrt(splitSEnd[0]*splitSEnd[0] + splitSEnd[1]*splitSEnd[1]);
    if (distToOrigin > 1.0f) {
      gp_vec3 toOrigin = -splitSEnd;
      toOrigin[2] = 0.0f;
      addStraightSegment(path, splitSEnd, toOrigin.normalized(), distToOrigin, totalDistance);
    }

    return path;
  }

private:
  void addStraightSegment(std::vector<Path>& path, const gp_vec3& start, const gp_vec3& direction,
                          gp_scalar distance, gp_scalar& totalDistance) {
    const gp_scalar step = 0.4f;  // 0.4m spacing (ensures ≥20Hz at min rabbit speed 8m/s)
    gp_vec3 dir = direction.normalized();
    gp_scalar startD = path.empty() ? 0.0f : step;

    for (gp_scalar d = startD; d <= distance; d += step) {
      gp_vec3 point = start + dir * d;
      if (!path.empty()) {
        gp_scalar segmentDist = (point - path.back().start).norm();
        totalDistance += segmentDist;
      }
      path.push_back(Path(point, gp_vec3::UnitX(), totalDistance, 0.0f));
    }
  }

  void addHorizontalTurn(std::vector<Path>& path, const gp_vec3& start, gp_scalar radius,
                         gp_scalar angleRadians, bool clockwise, gp_scalar& totalDistance) {
    const gp_scalar step = 0.02f;  // 0.02 rad (~1°) spacing for finer temporal resolution

    gp_vec3 heading(-1.0f, 0.0f, 0.0f);
    if (path.size() >= 2) {
      heading = (path.back().start - path[path.size()-2].start).normalized();
    }

    gp_vec3 headingXY(heading[0], heading[1], 0.0f);
    headingXY = headingXY.normalized();
    gp_vec3 rightPerpendicular(-headingXY[1], headingXY[0], 0.0f);
    gp_vec3 center = start + rightPerpendicular * (clockwise ? 1.0f : -1.0f) * radius;
    gp_scalar startAngle = std::atan2(start[1] - center[1], start[0] - center[0]);
    gp_scalar angleSign = clockwise ? 1.0f : -1.0f;

    for (gp_scalar angle = step; angle <= angleRadians; angle += step) {
      gp_scalar totalAngle = startAngle + angleSign * angle;
      gp_vec3 point(center[0] + radius * std::cos(totalAngle), center[1] + radius * std::sin(totalAngle), start[2]);

      gp_scalar segmentDist = (point - path.back().start).norm();
      totalDistance += segmentDist;

      path.push_back(Path(point, gp_vec3::UnitX(), totalDistance, 0.0f));
    }
  }

  void addSpiralTurn(std::vector<Path>& path, const gp_vec3& start, gp_scalar radius,
                     gp_scalar angleRadians, bool clockwise, gp_scalar totalClimb, gp_scalar& totalDistance) {
    const gp_scalar step = 0.02f;  // 0.02 rad (~1°) spacing for finer temporal resolution

    gp_vec3 heading(-1.0f, 0.0f, 0.0f);
    if (path.size() >= 2) {
      heading = (path.back().start - path[path.size()-2].start).normalized();
    }

    gp_vec3 headingXY(heading[0], heading[1], 0.0f);
    headingXY = headingXY.normalized();
    gp_vec3 rightPerpendicular(-headingXY[1], headingXY[0], 0.0f);
    gp_vec3 center = start + rightPerpendicular * (clockwise ? 1.0f : -1.0f) * radius;
    gp_scalar startAngle = std::atan2(start[1] - center[1], start[0] - center[0]);
    gp_scalar angleSign = clockwise ? 1.0f : -1.0f;

    for (gp_scalar angle = step; angle <= angleRadians; angle += step) {
      gp_scalar totalAngle = startAngle + angleSign * angle;
      gp_scalar zOffset = (angle / angleRadians) * totalClimb;
      gp_vec3 point(center[0] + radius * std::cos(totalAngle), center[1] + radius * std::sin(totalAngle), start[2] + zOffset);

      gp_scalar segmentDist = (point - path.back().start).norm();
      totalDistance += segmentDist;

      path.push_back(Path(point, gp_vec3::UnitX(), totalDistance, 0.0f));
    }
  }

  void addHorizontalLoop(std::vector<Path>& path, const gp_vec3& loopOrigin, gp_scalar loopRadius,
                         bool clockwise, gp_scalar& totalDistance) {
    const gp_scalar step = 0.02f;  // 0.02 rad (~1°) spacing for finer temporal resolution
    gp_scalar sign = clockwise ? -1.0f : 1.0f;

    for (gp_scalar turn = 0; turn < static_cast<gp_scalar>(M_PI * 2.0); turn += step) {
      gp_vec3 circleCenter = loopOrigin + gp_vec3(0.0f, sign * loopRadius, 0.0f);
      gp_vec3 point = circleCenter + gp_vec3(-loopRadius * std::sin(turn), -sign * loopRadius * std::cos(turn), 0.0f);

      if (!path.empty()) {
        gp_scalar distance = (point - path.back().start).norm();
        totalDistance += distance;
      }
      path.push_back(Path(point, gp_vec3::UnitX(), totalDistance, 0.0f));
    }
  }

  void addPitchDownLoop(std::vector<Path>& path, const gp_vec3& start, const gp_vec3& heading,
                        gp_scalar loopRadius, gp_scalar& totalDistance) {
    const gp_scalar step = 0.02f;  // 0.02 rad (~1°) spacing for finer temporal resolution
    gp_vec3 headingNorm = heading.normalized();
    gp_vec3 center = start + gp_vec3(0.0f, 0.0f, loopRadius);

    for (gp_scalar angle = step; angle <= static_cast<gp_scalar>(M_PI); angle += step) {
      gp_scalar horizontalOffset = loopRadius * std::sin(angle);
      gp_scalar verticalOffset = -loopRadius * std::cos(angle);

      gp_vec3 point = center + headingNorm * horizontalOffset + gp_vec3(0.0f, 0.0f, verticalOffset);

      gp_scalar segmentDist = (point - path.back().start).norm();
      totalDistance += segmentDist;

      path.push_back(Path(point, gp_vec3::UnitX(), totalDistance, 0.0f));
    }
  }

  void addCubicTransition(std::vector<Path>& path, const gp_vec3& from, const gp_vec3& to,
                          const gp_vec3& endHeading, gp_scalar& totalDistance) {
    // Create smooth cubic transition between two points
    // Use 4 control points: from, from+tangent, to-tangent, to
    gp_scalar transitionDist = (to - from).norm();
    gp_scalar tangentScale = transitionDist * 0.4f;

    gp_vec3 fromTangent = (path.size() >= 2) ?
      (path.back().start - path[path.size()-2].start).normalized() * tangentScale :
      gp_vec3(-1.0f, 0.0f, 0.0f) * tangentScale;

    gp_vec3 p0 = from - fromTangent;  // Control point before start
    gp_vec3 p1 = from;
    gp_vec3 p2 = to;
    gp_vec3 p3 = to + endHeading.normalized() * tangentScale;  // Control point after end

    const gp_scalar step = 0.02f;  // 0.02 rad (~1°) spacing for finer temporal resolution
    for (gp_scalar t = step; t <= 1.0f; t += step) {
      gp_vec3 point = cubicInterpolate(p0, p1, p2, p3, t);

      if (!path.empty()) {
        gp_scalar segmentDist = (point - path.back().start).norm();
        totalDistance += segmentDist;
      }
      path.push_back(Path(point, gp_vec3::UnitX(), totalDistance, 0.0f));
    }
  }
};

class GenerateLongSequential : public GeneratorMethod {
public:
  std::vector<Path> method(int pathIndex, gp_scalar radius, gp_scalar height, gp_scalar base, unsigned int seed) override {
    // LongSequential path is deterministic, seed is unused
    std::vector<Path> longPath;
    gp_scalar totalDistance = 0.0f;

    // Origin point at canonical (0,0,0)
    gp_vec3 origin(0.0f, 0.0f, 0.0f);
    gp_scalar loopRadius = static_cast<gp_scalar>(20.0f);

    // 0. Lead-in: fixed 13m straight-and-level southbound
    const gp_scalar leadDistance = 13.0f;
    const int leadSteps = 20;
    for (int i = 0; i <= leadSteps; ++i) {
      gp_scalar frac = static_cast<gp_scalar>(i) / static_cast<gp_scalar>(leadSteps);
      gp_vec3 point = origin + gp_vec3(-leadDistance * frac, 0.0f, 0.0f);
      gp_scalar distance = (longPath.empty() ? 0.0f : (point - longPath.back().start).norm());
      totalDistance += distance;
      longPath.push_back(Path(point, gp_vec3::UnitX(), totalDistance, 0.0f));
    }

    // Use the end of the lead-in as the loop origin so we start the horizontal 8 from the current spot
    gp_vec3 loopOrigin = longPath.empty() ? origin : longPath.back().start;

    // 2. LEFT HORIZONTAL LOOP - Start at origin heading south, turn left (counter-clockwise)
    // Circle center to the west of origin so we turn left around it  
    for (gp_scalar turn = 0; turn < static_cast<gp_scalar>(M_PI * 2.0); turn += static_cast<gp_scalar>(0.05f)) {
      gp_vec3 circleCenter = loopOrigin + gp_vec3(0.0f, -loopRadius, 0.0f);
      // Start at origin (turn=0): point at center + (-radius, 0, 0), heading south
      gp_vec3 point = circleCenter + gp_vec3(-loopRadius * std::sin(turn), loopRadius * std::cos(turn), 0.0f);
      gp_scalar distance = (longPath.empty() ? 0.0f : (point - longPath.back().start).norm());
      totalDistance += distance;
      longPath.push_back(Path(point, gp_vec3::UnitX(), totalDistance, 0.0f));
    }

    
    // 3. RIGHT HORIZONTAL LOOP - Start at origin heading south, turn right (clockwise)  
    // Circle center to the east of origin so we turn right around it
    for (gp_scalar turn = 0; turn < static_cast<gp_scalar>(M_PI * 2.0); turn += static_cast<gp_scalar>(0.05f)) {
      gp_vec3 circleCenter = loopOrigin + gp_vec3(0.0f, loopRadius, 0.0f);
      // Start at origin (turn=π): point at center + (-radius, 0, 0), heading south
      gp_vec3 point = circleCenter + gp_vec3(-loopRadius * std::sin(turn), -loopRadius * std::cos(turn), 0.0f);
      gp_scalar distance = (point - longPath.back().start).norm();
      totalDistance += distance;
      longPath.push_back(Path(point, gp_vec3::UnitX(), totalDistance, 0.0f));
    }

    return longPath;
  }
};


#endif
