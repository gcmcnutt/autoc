#ifndef AUTOC_NN_INPUT_MASK_H
#define AUTOC_NN_INPUT_MASK_H

// 041 T049 — resolve NN input SLOT NAMES to an ablation mask.
//
// WHY A SHARED HEADER. The mask is built by whoever drives an ablation (today
// `autoc` in eval mode; the `nn_ablate` driver next) and consumed by the worker.
// Building it in two places would let the name→index mapping drift, and a
// drifted mapping ablates the wrong column while reporting the right name —
// the exact "plausible but wrong" shape 041 exists to retire.
//
// ⚠️ NO NEW NAMING INFRASTRUCTURE. Names come from the EXISTING metadata tables
// (`kPathgenInputMeta` / `kTrackerInputMeta`), which are already
// `static_assert`-ed against their enum COUNT. That assert is what makes this
// resolver total: every slot has exactly one name, and the mask is exactly
// COUNT long.
//
// ⚠️ An unrecognised name is a HARD ERROR listing the valid set — never a
// silent no-op (Constitution VII). A typo'd `--zero-input GYROP` that quietly
// ablated nothing would report "no effect", which is a *finding-shaped* lie.

#include <algorithm>
#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "autoc/nn/nn_inputs.h"

namespace autoc {
namespace nn {

// Split a comma-separated slot list, trimming whitespace and dropping empties
// (so "A, B," and "A,B" mean the same thing).
inline std::vector<std::string> splitSlotNames(const std::string& csv) {
    std::vector<std::string> out;
    std::string cur;
    std::istringstream ss(csv);
    while (std::getline(ss, cur, ',')) {
        const size_t b = cur.find_first_not_of(" \t");
        const size_t e = cur.find_last_not_of(" \t");
        if (b == std::string::npos) continue;
        out.push_back(cur.substr(b, e - b + 1));
    }
    return out;
}

// Build a COUNT-length mask (1 = ablate) from slot names, for the given mode.
//
// `names` empty ⇒ empty mask ⇒ the unablated baseline. That is deliberately the
// same representation the training path uses, so "no ablation" is one state
// rather than two that could disagree.
inline std::vector<uint8_t> buildInputMask(const std::vector<std::string>& names,
                                           bool tracker) {
    const SensorInputMeta* meta = tracker ? kTrackerInputMeta : kPathgenInputMeta;
    const int count = tracker ? static_cast<int>(TrackerInput::COUNT)
                              : static_cast<int>(PathgenInput::COUNT);
    if (names.empty()) return {};

    std::vector<uint8_t> mask(static_cast<size_t>(count), 0);
    for (const std::string& want : names) {
        int found = -1;
        for (int i = 0; i < count; ++i) {
            if (want == meta[i].name) { found = i; break; }
        }
        if (found < 0) {
            std::ostringstream msg;
            msg << "unknown NN input slot '" << want << "' for "
                << (tracker ? "tracker" : "pathgen") << " mode. Valid names:";
            for (int i = 0; i < count; ++i) {
                msg << (i % 6 == 0 ? "\n  " : " ") << meta[i].name;
            }
            throw std::runtime_error(msg.str());
        }
        // Repeating a name is harmless and idempotent; it is not an error,
        // because `--zero-input A,A` expresses the same intent as `--zero-input A`.
        mask[static_cast<size_t>(found)] = 1;
    }
    return mask;
}

inline std::vector<uint8_t> buildInputMask(const std::string& csv, bool tracker) {
    return buildInputMask(splitSlotNames(csv), tracker);
}

// Human-readable echo of what a mask actually ablates — printed by the driver
// so a run's log states the ablation in slot names, not as a byte vector.
inline std::string describeInputMask(const std::vector<uint8_t>& mask, bool tracker) {
    if (mask.empty()) return "(none — unablated baseline)";
    const SensorInputMeta* meta = tracker ? kTrackerInputMeta : kPathgenInputMeta;
    std::ostringstream out;
    bool first = true;
    for (size_t i = 0; i < mask.size(); ++i) {
        if (!mask[i]) continue;
        if (!first) out << ",";
        out << meta[i].name;
        first = false;
    }
    return out.str();
}

}  // namespace nn
}  // namespace autoc

#endif  // AUTOC_NN_INPUT_MASK_H
