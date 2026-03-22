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
// This eliminates the separate ISR ticker and its 0-50ms send jitter.
static unsigned long lastLoopTime = 0;
static int loopCounter = 0;

void controllerUpdate()
{
  unsigned long now = millis();

  if (now - lastLoopTime >= MSP_LOOP_INTERVAL_MSEC)
  {
    lastLoopTime = now;

    if ((loopCounter % MSP_NN_EVAL_DIVISOR) == 0)
    {
      // NN tick: fetch state, eval NN, update cache, send immediately
      mspUpdateState();    // fetches INAV state, runs NN, caches RC commands
    }

    // Every tick: send cached RC commands (heartbeat or fresh from NN eval above)
    mspSetControls();

    loopCounter++;
  }
}
