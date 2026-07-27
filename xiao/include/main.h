#pragma once

#include <Arduino.h>
#include <ArduinoBLE.h>
#include <HardwareSerial.h>
#include "MSP.h"
#include "state.h"
#include "flash_logger.h"
#include "fault_guard.h"
#include <autoc/types.h>

// Forward declarations
struct AircraftState;

// Define RGB pins
#define RED_PIN 11
#define GREEN_PIN 12
#define BLUE_PIN 13

#define HEARTBEAT_LED RED_PIN
#define BLINK_INTERVAL_MSEC 250

// Single unified loop interval: 50ms (20Hz). Every tick sends cached RC commands.
// Every Nth tick (N = MSP_NN_EVAL_DIVISOR) also fetches state and runs NN eval.
#define MSP_LOOP_INTERVAL_MSEC 50
#define MSP_NN_EVAL_DIVISOR 1     // 039 FR-011: NN eval EVERY tick = 20Hz (037 cadence
                                  // decision; training runs at 20 Hz since t10).
                                  // Re-entry safety at every-tick eval: the loop is
                                  // single-threaded (no ticker) — fetch/eval/send run
                                  // sequentially inside one controllerUpdate() tick, so
                                  // the mspBusLocked guard can never see contention; an
                                  // over-budget tick shows up as a loopStats overrun,
                                  // not an overlapping MSP transaction.
#define MSP_REPLY_TIMEOUT_MSEC 50
#define MSP_LOS_INTERVAL_MSEC 2000

#define MSP_DEFAULT_CHANNEL_VALUE 1500
#define MSP_ARM_CHANNEL 8
#define MSP_ARMED_THRESHOLD 1600
#define MSP_ARM_CYCLE_COUNT 0     // No delay — INAV's 790ms freshness guard is sufficient
#define MSP_PATH_SELECT_CHANNEL 9  // INAV RC channel 10 → MSP array index 9 (6-position switch: 1000-2000 → paths 0-5)

// MSP flight mode flags
#define MSP_MODE_MSPRCOVERRIDE 30

// Define log levels
enum LogLevel
{
  DEBUG = 0,
  INFO = 1,
  WARNING = 2,
  ERROR = 3
};

// Controller-loop cadence stats — populated by controllerUpdate() per tick,
// reset at engage-span start (pipelineStats.reset sites), summarized when
// the engage span ends (inside stopAutoc). See research.md §4 / plot_xiao_cadence.py.
struct LoopStats {
  uint32_t ticks;        // total ticks since reset (denominator)
  uint32_t overruns;     // ticks late by >= MSP_LOOP_INTERVAL_MSEC
  uint32_t resyncs;      // ticks late by >= 3*INTERVAL (forced schedule skip)
  uint32_t maxLateMs;    // worst per-tick lateness observed
  uint32_t totalLateMs;  // cumulative lateness (for average)

  void reset() { ticks = overruns = resyncs = maxLateMs = totalLateMs = 0; }

  void recordTick(uint32_t lateMs) {
    ticks++;
    totalLateMs += lateMs;
    if (lateMs > maxLateMs) maxLateMs = lateMs;
    if (lateMs >= MSP_LOOP_INTERVAL_MSEC) overruns++;
    if (lateMs >= 3 * MSP_LOOP_INTERVAL_MSEC) resyncs++;
  }
};
extern LoopStats loopStats;

// utils
void consoleInit();
void logPrint(LogLevel level, const char* format, ...);
void setLogLevel(LogLevel level);

// interfaces
void msplinkSetup();
void mspUpdateState();
void controllerSetup();
void controllerUpdate();
void mspSetControls();
void ledSetup();

void heartBeatLED();
void blueToothSetup();
void blueToothLoop();
void blueToothSetEnabled(bool enabled);

// GP rabbit path following functions
void convertMSPStateToAircraftState(AircraftState& aircraftState);
int getRabbitPathIndex(unsigned long elapsed_msec);
int convertRollToMSPChannel(gp_scalar gp_command);
int convertPitchToMSPChannel(gp_scalar gp_command);
int convertThrottleToMSPChannel(gp_scalar gp_command);
