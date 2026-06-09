#pragma once

#include <string>

// 034 FR-015 → 035 FR-P07b — run-id prefix.
//
// Originally (034 T033) M1/M2 shared one bucket and were distinguished by a
// leading run-id prefix ("autoc-" vs "tracker-"). 035's per-mode-bucket
// contract makes the BUCKET the discriminator, so the run-id prefix is now
// UNIFORM ("autoc-") for every mode — the selector's SetPrefix("autoc-")
// then matches every mode's runs in its own bucket. The `tracker-` branch is
// retired. Kept as a function (taking mode) for call-site stability.
namespace autoc {

inline std::string runIdPrefixForMode(const std::string& /*mode*/) {
  return "autoc-";  // FR-P07b: uniform across modes; bucket is the discriminator
}

}  // namespace autoc
