#ifndef AUTOC_H
#define AUTOC_H

#include <vector>

#include "autoc/types.h"
#include "autoc/nn/topology.h"

// Forward declarations
struct WorkerContext;

// ========================================================================
// FITNESS (022): Point-accumulation with ellipsoidal scoring surface
// See specs/022-tracking-cone-fitness/spec.md
// Config params: FitBehindScale, FitAheadScale, FitCrossScale, etc. in autoc.ini
// ========================================================================

// Entry position safe bounds (~15m margin from crash boundaries)
#define ENTRY_SAFE_RADIUS static_cast<gp_scalar>(55.0f)       // SIM_PATH_RADIUS_LIMIT - 15m
// Entry altitude bounds are origin-relative (offset from start altitude, NED)
#define ENTRY_SAFE_ALT_MIN static_cast<gp_scalar>(3.0f)       // shallowest: 3m below start (NED +)
#define ENTRY_SAFE_ALT_MAX static_cast<gp_scalar>(-80.0f)     // deepest: 80m above start (NED -)

struct WindScenarioConfig {
  unsigned int windSeed = 0;
  int windVariantIndex = 0;
};

struct ScenarioDescriptor {
  std::vector<std::vector<Path>> pathList;
  std::vector<WindScenarioConfig> windScenarios;
  unsigned int windSeed = 0;
  int pathVariantIndex = -1;   // -1 = unset/aggregated
  int windVariantIndex = -1;   // -1 = unset/aggregated
};

// Global aircraft state used by minisim evaluation loop
extern AircraftState aircraftState;

#endif
