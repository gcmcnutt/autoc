#pragma once

// 030 M7a — CrashReason factored out of protocol.h to break the
// circular include path between protocol.h and arena.h.
//
// Both headers need CrashReason: protocol.h defines EvalResults (which
// contains crashReasonList) and arena.h provides the
// arenaEgressToCrashReason convenience for stepper integration. Before
// this split, arena.h included protocol.h directly, which post-M7a tried
// to include arena.h back — guard-skipped, leaving FlightArena
// undefined when EvalData referenced it.
//
// Definition lives here; protocol.h re-includes this header so existing
// protocol.h consumers see CrashReason unchanged.

enum class CrashReason {
    None,             // Still flying
    Boot,             // Startup/initialization error
    Sim,              // Physics crash (ground impact, structural failure)
    Eval,             // Out of bounds (too far, too low, too high)
    TimeLimit,        // Simulation time cap reached (was: Time)
    RabbitComplete,   // Rabbit reached end of path — normal completion (was: Distance)
};

// Is this a real crash (penalizable) or normal termination?
inline bool isCrash(CrashReason reason) {
    return reason == CrashReason::Boot ||
           reason == CrashReason::Sim ||
           reason == CrashReason::Eval;
}
