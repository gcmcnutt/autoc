// ACRO rate PID — pure math helper, shared between CRRCSim inputdev and tests.
// See spec 026, research §"Sim PID implementation notes". Single-axis FF+P+I.
#pragma once

#include <algorithm>

struct AcroPidGains {
  double ff;     // feedforward on desired rate
  double p;      // proportional on rate error
  double i;      // integral on rate error
  double scale;  // PID-output divisor → final output in [-1,+1] after clamp
};

struct AcroPidStep {
  double desiredRate;   // rad/s (NN [-1,+1] × ACRO_MAX_RATE)
  double measuredRate;  // rad/s (filtered body rate)
  double dt;            // seconds
};

struct AcroPidResult {
  double output;       // pre-clamp output (caller clamps to [-1,+1])
  double clamped;      // post-clamp output
  double newIntegral;  // integrator state after this step (rad)
  double ffTerm;       // post-scale feedforward contribution
  double pTerm;        // post-scale proportional contribution
  double iTerm;        // post-scale integral contribution
  double error;        // desiredRate - measuredRate (rad/s)
  bool saturated;      // true if clamp altered output
};

// Single-axis ACRO PID step. Anti-windup via symmetric clamp on integrator
// before it feeds the I term — keeps the PID predictable when error stays
// large for many ticks (e.g. step input + slow physics).
inline AcroPidResult acroPidStep(const AcroPidGains& g,
                                 const AcroPidStep& s,
                                 double prevIntegral,
                                 double integralLimit) {
  AcroPidResult r;
  r.error = s.desiredRate - s.measuredRate;
  double integ = prevIntegral + r.error * s.dt;
  if (integ >  integralLimit) integ =  integralLimit;
  if (integ < -integralLimit) integ = -integralLimit;
  r.newIntegral = integ;
  r.ffTerm = (g.ff * s.desiredRate) / g.scale;
  r.pTerm  = (g.p  * r.error)       / g.scale;
  r.iTerm  = (g.i  * integ)         / g.scale;
  r.output = r.ffTerm + r.pTerm + r.iTerm;
  r.clamped = std::clamp(r.output, -1.0, 1.0);
  r.saturated = (r.clamped != r.output);
  return r;
}
