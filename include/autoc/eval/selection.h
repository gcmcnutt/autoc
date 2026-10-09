#pragma once

#include <vector>
#include <string>
#include "autoc/eval/fitness_decomposition.h"

enum class SelectionMode {
    SUM,        // Legacy: tournament on scalar fitness (lower is better)
    MINIMAX,    // Tournament on worst-scenario fitness (lower is better)
    LEXICASE    // Epsilon-lexicase: single dimension per scenario (score, lower is better)
};

SelectionMode parseSelectionMode(const std::string& str);
const char* selectionModeToString(SelectionMode mode);

// 043 t5 — how the excess-rotation measure (excess_rotation.h) enters the pool.
//
// The measure is `craft_rotation − path_rotation`, and path_rotation is the SAME
// for every genome on a scenario. So a standalone rotation case is won by whoever
// rotates least — a passive flyer that never follows the path wins it on every
// scenario. 043-t4 ran it that way (Independent) and the population never
// converged on tracking: pop-avg fitness was a third of t3's by gen 40 under
// IDENTICAL wind, flat from gen 200, elite unreplaced from gen 281 (t4 MANIFEST
// § 7). Tracking had fallen from 1/2 of the selection starts to 1/3, and the
// other 2/3 were winnable without tracking.
//
// Conditional makes the rotation case a TIE-BREAK INSIDE every score case on the
// same scenario: survivors within epsilon on `score`, then within epsilon on
// `excess_rotation` among those. Smoothness only discriminates between genomes
// that track equally well — "mostly about tracking as close as possible; a
// smooth line should fly smooth" (operator 2026-10-05). The pool is back to
// 2 × num_scenarios (score⊕rotation, energy), the t3 share for tracking.
enum class ExcessRotationAxisMode {
    Off,          // pool unchanged — t3 and every pinned run before t4 reproduce bitwise
    Independent,  // t4: third co-equal case per scenario (kept for ablation/replay only)
    Conditional   // t5: tie-break inside the score case
};
// "independent" | "conditional" (trimmed of inline '#'/';' comments); anything
// else throws — the mode is never defaulted when the axis is enabled.
ExcessRotationAxisMode parseExcessRotationAxisMode(const std::string& str);
const char* excessRotationAxisModeToString(ExcessRotationAxisMode mode);

// 043 t5 — selection telemetry. Pure counters (no PRNG, no effect on the pick),
// accumulated across the lexicase_select calls of one breeding pass and drained
// by the trainer for the per-gen `#GenSel` line. The question it answers at
// gen 5 instead of gen 400 (t4 cost 25 h to learn its axis was inert on
// trackers): does the Conditional tie-break actually BITE — remove candidates —
// and over how many survivors does it get to choose?
struct LexicaseStats {
    long selections = 0;         // lexicase_select calls
    long cases_applied = 0;      // test cases evaluated (all fields)
    long tiebreaks_applied = 0;  // tie-breaks evaluated (Conditional only; >1 candidate going in)
    long tiebreaks_bit = 0;      // tie-breaks that removed >= 1 candidate
    long tiebreak_in = 0;        // Σ candidates entering a tie-break
    long tiebreak_out = 0;       // Σ candidates leaving it
};
LexicaseStats drainLexicaseStats();  // returns the accumulated counters and zeroes them

// Lexicase selection (022; 027/035 multi-axis). Each selection event shuffles
// the (scenario × field) test-case pool and, per test case, keeps candidates
// within epsilon of the best. Returns an index into the population.
//
// Epsilon mode (035 FR-003):
//   use_mad_epsilon == false (constant): eps = max(0.5 floor, |best|·`epsilon`)
//                                        — bit-reproduces prior runs (SC-003).
//   use_mad_epsilon == true  (mad):      eps = per-axis MAD (median abs deviation)
//                                        of the field over the surviving candidates.
int lexicase_select(const std::vector<std::vector<ScenarioScore>>& all_scores,
                    int pop_size, bool use_mad_epsilon, double epsilon = 0.05,
                    bool include_prediction_axis = false,   // 038 US3 — EnablePredictorHead gate
                    ExcessRotationAxisMode rot_axis_mode = ExcessRotationAxisMode::Off);  // 043 T102/t5

// Minimax fitness: worst-case scenario drives selection.
double minimax_fitness(const std::vector<ScenarioScore>& scores);
