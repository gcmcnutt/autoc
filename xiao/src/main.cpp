#include <main.h>

void setup()
{
  // fault capture vectors + previous-reboot evidence snapshot — before
  // anything that can crash (fault_guard.h)
  faultGuardInstall();

  // first thing to do, show a RED ledfrom off to red
  ledSetup();

  // logger
  consoleInit();

  // Wait for WSL serial port to reconnect after reboot
  delay(8000);
  Serial.println("\n\n========== XIAO-GP BOOT ==========");

  // why did we reboot? (fault/watchdog breadcrumb from the previous run)
  faultGuardReport();

  // flash logger (init after console for debug output)
  flashLoggerInit();

  // msp link
  msplinkSetup();

  // bluetooth sync handler
  blueToothSetup();

  // flight controller
  controllerSetup();

  // last: watchdog live from here on (unstoppable; loop() feeds it)
  faultGuardArmWatchdog();

  Serial.println("========== BOOT COMPLETE ==========\n");
}

void loop()
{
  faultGuardFeed();

  heartBeatLED();
  blueToothLoop();

  controllerUpdate();

  // Flush flash logger at end of loop
  flashLoggerFlushCheck();
}
