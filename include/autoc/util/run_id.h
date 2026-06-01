#pragma once

#include <string>

// 034 FR-015 (T033/T034) — run-id prefix routing.
//
// M1 (pathgen) and M2 (tracker) runs share one S3 bucket, so their run-ids
// (and the dmp keys under them) must be distinguishable by a leading prefix:
// pathgen runs get "autoc-", tracker runs get "tracker-". The mode→prefix
// decision lives in this pure function so it is unit-testable in isolation
// (generate_iso8601_timestamp itself lives in autoc.cc alongside main() and
// is not linkable into the test binary).
namespace autoc {

inline std::string runIdPrefixForMode(const std::string& mode) {
  return mode == "tracker" ? "tracker-" : "autoc-";
}

}  // namespace autoc
