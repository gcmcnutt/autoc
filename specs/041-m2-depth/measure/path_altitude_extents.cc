// 041 P0-2 — HAT analysis instrument: measure the ALTITUDE EXTENT of the M1
// training paths, in the virtual frame and in crrcsim AGL.
//
// Why this exists: P0-4 decides the arena band, and the band cannot be chosen
// without knowing how much vertical room the targets themselves demand. The
// numbers below are MEASURED from the same generator autoc runs, not read off
// the generator source (SpiralClimb's -50 literal is a control-point input, not
// the realized extent after cubic interpolation).
//
// Build: see README.md in this directory.
#include <cstdio>
#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include "autoc/eval/pathgen.h"
#include "autoc/eval/aircraft_state.h"

int main(int argc, char** argv) {
    const std::string method = (argc > 1) ? argv[1] : "aeroStandard";
    const int numPaths = (argc > 2) ? std::stoi(argv[2]) : 6;
    const unsigned int seed = (argc > 3)
        ? static_cast<unsigned int>(std::stoul(argv[3])) : 67890u;

    auto paths = generateSmoothPaths(method, numPaths, SIM_PATH_BOUNDS,
                                     SIM_PATH_BOUNDS, seed);

    // AGL of the virtual origin, measured in P0-1: crrcsim launches the CG at
    // 82 ft (launch.altitude) + 0.125 ft (hb1 dZLow) - 0.1 ft (Davis ground)
    // = 82.025 ft = 25.0012 m above the crrcsim z=0 datum, and autoc's
    // pathOriginOffset puts the virtual origin there.
    const double kOriginAglM = 25.0012;

    printf("method=%s numPaths=%d seed=%u  (radius=%.1f height=%.1f)\n",
           method.c_str(), numPaths, seed,
           static_cast<double>(SIM_PATH_BOUNDS),
           static_cast<double>(SIM_PATH_BOUNDS));
    printf("virtual z is NED down-positive; AGL = %.4f - z\n\n", kOriginAglM);
    printf("%-5s %8s %9s %9s %9s %9s %9s\n",
           "path", "pts", "z_min", "z_max", "agl_min", "agl_max", "radius_max");

    double allAglMin = std::numeric_limits<double>::max();
    double allAglMax = std::numeric_limits<double>::lowest();
    double allRadMax = 0.0;

    for (size_t p = 0; p < paths.size(); ++p) {
        double zmin = std::numeric_limits<double>::max();
        double zmax = std::numeric_limits<double>::lowest();
        double radmax = 0.0;
        for (const auto& seg : paths[p]) {
            const double z = static_cast<double>(seg.start[2]);
            const double x = static_cast<double>(seg.start[0]);
            const double y = static_cast<double>(seg.start[1]);
            zmin = std::min(zmin, z);
            zmax = std::max(zmax, z);
            radmax = std::max(radmax, std::sqrt(x * x + y * y));
        }
        const double aglMax = kOriginAglM - zmin;   // most negative z = highest
        const double aglMin = kOriginAglM - zmax;
        allAglMin = std::min(allAglMin, aglMin);
        allAglMax = std::max(allAglMax, aglMax);
        allRadMax = std::max(allRadMax, radmax);
        printf("%-5zu %8zu %9.2f %9.2f %9.2f %9.2f %9.2f\n",
               p, paths[p].size(), zmin, zmax, aglMin, aglMax, radmax);
    }

    printf("\nALL PATHS: agl_min=%.2f m  agl_max=%.2f m  span=%.2f m  radius_max=%.2f m\n",
           allAglMin, allAglMax, allAglMax - allAglMin, allRadMax);
    printf("Entry (virtual origin) AGL = %.4f m\n", kOriginAglM);
    printf("Target band relative to entry: %+.2f m below .. %+.2f m above\n",
           kOriginAglM - allAglMin, allAglMax - kOriginAglM);
    return 0;
}
