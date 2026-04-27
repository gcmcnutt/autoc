#include <main.h>
#include <ArduinoEigenDense.h>

#include <autoc/eval/aircraft_state.h>

using namespace Eigen;

AircraftState aircraftState;

void controllerSetup()
{
}

// Single 20Hz loop (MSP_LOOP_INTERVAL_MSEC = 50ms).
// Every tick: send cached RC commands to INAV (heartbeat + command delivery).
// Every Nth tick: fetch INAV state, run NN eval, update cache, THEN send.
//
// Fixed-schedule accumulator: nextLoopTime advances by a rigid INTERVAL
// each tick, NOT = millis(). A late tick doesn't permanently shift the
// schedule forward; it just produces a non-zero lateness that gets
// recorded into loopStats. Summarized once per engage span in stopAutoc
// (no per-tick log noise). See specs/024-sim-real-fidelity/plot_xiao_cadence.py
// for the pre-fix flight-log analysis that motivated this.
static unsigned long nextLoopTime = 0;
static int loopCounter = 0;

void controllerUpdate()
{
  unsigned long now = millis();
  if (nextLoopTime == 0) {
    nextLoopTime = now;  // first-call alignment
  }
  if ((long)(now - nextLoopTime) < 0) {
    return;  // not yet
  }

  uint32_t lateness = (uint32_t)(now - nextLoopTime);
  nextLoopTime += MSP_LOOP_INTERVAL_MSEC;
  loopStats.recordTick(lateness);

  // If we're so far behind we'd burst >3 catch-up ticks, resync schedule
  // to now. Protects MSP bus and flash log from a saturating sequence.
  if (lateness >= 3 * MSP_LOOP_INTERVAL_MSEC) {
    nextLoopTime = now + MSP_LOOP_INTERVAL_MSEC;
  }

  if ((loopCounter % MSP_NN_EVAL_DIVISOR) == 0)
  {
    // NN tick: fetch state, eval NN, update cache, send immediately
    mspUpdateState();
  }

  // Every tick: send cached RC commands
  mspSetControls();

  loopCounter++;
}
