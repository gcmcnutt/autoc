#include <main.h>
#include <ArduinoEigenDense.h>

#include <autoc/eval/aircraft_state.h>

using namespace Eigen;

AircraftState aircraftState;

void controllerSetup()
{
}

void controllerUpdate()
{
  // Unified timing loop - 100ms sensor/NN updates (10Hz, matches sim)
  static unsigned long lastUpdateTime = 0;
  unsigned long now = millis();

  // 100ms cycle: Update sensors and NN control
  if (now - lastUpdateTime >= MSP_UPDATE_INTERVAL_MSEC)
  {
    lastUpdateTime = now;
    
    // Update sensor data, aircraft state, and GP control (all integrated)
    mspUpdateState();
  }
}
