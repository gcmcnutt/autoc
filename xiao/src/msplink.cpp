#include <main.h>
#include <autoc/eval/aircraft_state.h>
#include <embedded_pathgen_selector.h>
#include <autoc/eval/sensor_math.h>
#include <nn_program.h>
#include <mbed.h>
#include <vector>
#include <cmath>
#include <algorithm>

MSP msp;

State state;

// NN Rabbit Path Following System
static EmbeddedPathSelector path_generator;
static std::vector<Path> flight_path;
static AircraftState aircraft_state;
static Path gp_path_segment; // Current path segment for NN evaluator

// Test origin anchoring (captured at autoc enable; aircraft_state is expressed relative to this origin)
static gp_vec3 test_origin_offset(0.0f, 0.0f, 0.0f);
static bool test_origin_set = false;

// Nav Control Timing and State
static unsigned long rabbit_start_time = 0;
static volatile bool rabbit_active = false;
static int current_path_index = 0;
constexpr gp_scalar XIAO_RABBIT_SPEED_MPS = 13.0f;  // Rabbit speed for xiao (m/s)
static gp_scalar rabbit_odometer = 0.0f;             // Distance along path (meters)
static int selected_path_index = 0;  // Path selected from RC channel (0-5)
static bool servo_reset_required = false;

// MSP Control Output Caching and scheduling
static volatile int cached_roll_cmd = MSP_DEFAULT_CHANNEL_VALUE;
static volatile int cached_pitch_cmd = MSP_DEFAULT_CHANNEL_VALUE;
static volatile int cached_throttle_cmd = MSP_DEFAULT_CHANNEL_VALUE;
static volatile uint32_t cached_cmd_sequence = 0;
static volatile uint32_t cached_eval_start_us = 0;
static volatile uint32_t cached_eval_complete_us = 0;
// Re-entry guard — prevents overlapping MSP transactions if NN eval exceeds one loop tick.
// No ISR contention (ticker removed), but MSP fetch + NN eval could exceed 50ms.
static volatile bool mspBusLocked = false;

static constexpr gp_scalar GP_RAD_TO_DEG = static_cast<gp_scalar>(57.2957795f);
static constexpr gp_scalar GP_HALF_PI = static_cast<gp_scalar>(1.57079633f);
static constexpr gp_scalar GP_INV_1000 = static_cast<gp_scalar>(0.001f);

// Pipeline timing stats — measures fetch→eval→send per NN tick
struct PipelineStats {
  uint32_t samples;
  uint32_t fetchSumUs, fetchMinUs, fetchMaxUs;   // MSP fetch duration
  uint32_t evalSumUs, evalMinUs, evalMaxUs;       // NN eval duration
  uint32_t sendSumUs, sendMinUs, sendMaxUs;       // MSP send duration
  uint32_t totalSumUs, totalMinUs, totalMaxUs;    // fetch + eval + send
  uint32_t intervalSumUs, intervalMinUs, intervalMaxUs;  // tick-to-tick interval
  uint32_t intervalCount;
  uint32_t lastTickUs;  // for interval measurement

  void reset() {
    samples = intervalCount = 0;
    fetchSumUs = evalSumUs = sendSumUs = totalSumUs = intervalSumUs = 0;
    fetchMinUs = evalMinUs = sendMinUs = totalMinUs = intervalMinUs = UINT32_MAX;
    fetchMaxUs = evalMaxUs = sendMaxUs = totalMaxUs = intervalMaxUs = 0;
    lastTickUs = 0;
  }

  void recordTick(uint32_t fetchUs, uint32_t evalUs, uint32_t sendUs, uint32_t tickStartUs) {
    uint32_t total = fetchUs + evalUs + sendUs;
    samples++;
    fetchSumUs += fetchUs; fetchMinUs = std::min(fetchMinUs, fetchUs); fetchMaxUs = std::max(fetchMaxUs, fetchUs);
    evalSumUs += evalUs; evalMinUs = std::min(evalMinUs, evalUs); evalMaxUs = std::max(evalMaxUs, evalUs);
    sendSumUs += sendUs; sendMinUs = std::min(sendMinUs, sendUs); sendMaxUs = std::max(sendMaxUs, sendUs);
    totalSumUs += total; totalMinUs = std::min(totalMinUs, total); totalMaxUs = std::max(totalMaxUs, total);

    if (lastTickUs > 0 && tickStartUs > lastTickUs) {
      uint32_t interval = tickStartUs - lastTickUs;
      intervalSumUs += interval;
      intervalMinUs = std::min(intervalMinUs, interval);
      intervalMaxUs = std::max(intervalMaxUs, interval);
      intervalCount++;
    }
    lastTickUs = tickStartUs;
  }
};
static PipelineStats pipelineStats;

// Aircraft state tracking for position/velocity calculation
static gp_vec3 last_valid_position(0.0f, 0.0f, 0.0f);
static bool have_valid_position = false;
static bool was_system_armed = false;

// Safety timeout for single test run (60 seconds max per run)
#define GP_MAX_SINGLE_RUN_MSEC (60 * 1000)
#define MSP_BUS_LOCK_TIMEOUT_USEC (MSP_REPLY_TIMEOUT_MSEC * 2000UL)

// Forward declarations for MSP scheduling helpers
static void updateCachedCommands(int roll, int pitch, int throttle, uint32_t evalStartUs);
static bool tryLockMspBusFromTask();
static bool lockMspBusBlockingFromTask();
static void releaseMspBusFromTask();
static bool performMspRequest(uint16_t command, void *buffer, size_t size);
static void performMspSendLocked();

static void resetPositionHistory()
{
  last_valid_position = gp_vec3(0.0f, 0.0f, 0.0f);
  have_valid_position = false;
}

static void stopAutoc(const char *reason, bool requireServoReset)
{
  bool wasAutoc = state.autoc_enabled;
  bool wasRabbit = rabbit_active;
  bool latchBefore = servo_reset_required;
  state.autoc_enabled = false;
  rabbit_active = false;
  test_origin_set = false;
  test_origin_offset = gp_vec3(0.0f, 0.0f, 0.0f);
  state.autoc_countdown = 0;
  resetPositionHistory();
  analogWrite(GREEN_PIN, 255);

  if (requireServoReset)
  {
    servo_reset_required = true;
  }

  if (wasAutoc || wasRabbit || (requireServoReset && !latchBefore))
  {
    logPrint(INFO, "Nav Control: Autoc disabled (%s) - pilot has control", reason);
  }

  if (wasRabbit)
  {
    // Log pipeline timing stats
    if (pipelineStats.samples > 0) {
      gp_scalar n = static_cast<gp_scalar>(pipelineStats.samples);
      logPrint(INFO, "MSP pipeline: samples=%u fetch=%.1f/%.1f/%.1fms eval=%.1f/%.1f/%.1fms send=%.1f/%.1f/%.1fms total=%.1f/%.1f/%.1fms",
        pipelineStats.samples,
        pipelineStats.fetchMinUs*GP_INV_1000, (pipelineStats.fetchSumUs/n)*GP_INV_1000, pipelineStats.fetchMaxUs*GP_INV_1000,
        pipelineStats.evalMinUs*GP_INV_1000, (pipelineStats.evalSumUs/n)*GP_INV_1000, pipelineStats.evalMaxUs*GP_INV_1000,
        pipelineStats.sendMinUs*GP_INV_1000, (pipelineStats.sendSumUs/n)*GP_INV_1000, pipelineStats.sendMaxUs*GP_INV_1000,
        pipelineStats.totalMinUs*GP_INV_1000, (pipelineStats.totalSumUs/n)*GP_INV_1000, pipelineStats.totalMaxUs*GP_INV_1000);
      if (pipelineStats.intervalCount > 0) {
        gp_scalar ni = static_cast<gp_scalar>(pipelineStats.intervalCount);
        logPrint(INFO, "MSP interval: avg=%.1fms min=%.1fms max=%.1fms",
          (pipelineStats.intervalSumUs/ni)*GP_INV_1000,
          pipelineStats.intervalMinUs*GP_INV_1000,
          pipelineStats.intervalMaxUs*GP_INV_1000);
      }
    }
  }
}

static gp_vec3 neuVectorToNedMeters(const int32_t vec_cm[3])
{
  const gp_scalar inv100 = static_cast<gp_scalar>(0.01f);
  gp_scalar north = static_cast<gp_scalar>(vec_cm[0]) * inv100;
  gp_scalar east = static_cast<gp_scalar>(vec_cm[1]) * inv100;
  gp_scalar down = -static_cast<gp_scalar>(vec_cm[2]) * inv100;
  return gp_vec3(north, east, down);
}

static gp_quat neuQuaternionToNed(const float q[4])
{
  // INAV sends body->earth quaternion (confirmed via bench testing).
  // GP expects earth->body, so we take the conjugate.
  // Bench tests show ALL vector components need sign flip.
  gp_quat attitude(q[0], -q[1], -q[2], -q[3]);  // Full conjugate
  if (attitude.norm() == 0.0f)
  {
    return gp_quat::Identity();
  }
  attitude.normalize();
  return attitude;
}

// Pipeline timing points (set during mspUpdateState, read during mspSetControls)
static uint32_t pipeTickStartUs = 0;
static uint32_t pipeFetchEndUs = 0;
static uint32_t pipeEvalEndUs = 0;

static void mspUpdateNavControl()
{
  // Check for disarm or failsafe conditions before NN control
  // Only check if we're currently enabled to avoid repeat logging
  if (state.autoc_enabled && rabbit_active)
  {
    bool isArmed = state.status_valid && state.status.flightModeFlags & (1UL << MSP_MODE_ARM);
    bool isFailsafe = state.status_valid && state.status.flightModeFlags & (1UL << MSP_MODE_FAILSAFE);

    if (!isArmed || isFailsafe)
    {
      unsigned long test_run_duration = millis() - rabbit_start_time;
      if (isFailsafe)
      {
        logPrint(INFO, "Nav Control: INAV failsafe activated (%.1fs) - disabling autoc", test_run_duration * GP_INV_1000);
      }
      else
      {
        logPrint(INFO, "Nav Control: Aircraft disarmed (%.1fs) - disabling autoc", test_run_duration * GP_INV_1000);
      }
      stopAutoc(isFailsafe ? "failsafe" : "disarmed", true);
      return;
    }
  }

  // NN rabbit path following control - only if autoc is still enabled
  if (!state.autoc_enabled || !rabbit_active || flight_path.empty())
  {
    return; // Exit early if NN control has been disabled
  }

  // Proceed with NN control
  unsigned long current_time = millis();
  unsigned long elapsed_msec = current_time - rabbit_start_time;

  // Check termination conditions
  if (elapsed_msec > GP_MAX_SINGLE_RUN_MSEC)
  {
    logPrint(INFO, "Nav Control: Test run timeout (%.1fs) - stopping rabbit", elapsed_msec * GP_INV_1000);
    stopAutoc("timeout", true);
    return;
  }

  // Advance rabbit odometer each tick
  gp_scalar dt_sec = static_cast<gp_scalar>(SIM_TIME_STEP_MSEC) / 1000.0f;
  rabbit_odometer += XIAO_RABBIT_SPEED_MPS * dt_sec;

  // Find current path segment based on rabbit odometer
  current_path_index = getRabbitPathIndex(elapsed_msec);

  // End of path check
  if (current_path_index >= (int)flight_path.size() - 1)
  {
    logPrint(INFO, "Nav Control: End of path reached (%.1fs) - stopping rabbit", elapsed_msec * GP_INV_1000);
    stopAutoc("path complete", true);
    return;
  }

  // NN evaluation - calculate new commands
  if (current_path_index < (int)flight_path.size())
  {
    gp_path_segment = flight_path[current_path_index];

    // Set current path index and elapsed time for NN evaluation
    aircraft_state.setThisPathIndex(current_path_index);
    aircraft_state.setSimTimeMsec(elapsed_msec);
    aircraft_state.setRabbitOdometer(rabbit_odometer);
    aircraft_state.setRabbitSpeed(XIAO_RABBIT_SPEED_MPS);

    // Full path provider for interpolation and forecast lookahead
    VectorPathProvider pathProvider(flight_path, aircraft_state.getThisPathIndex());
    uint32_t eval_start_us = micros();

    // Compute current-step sensors for history recording
    gp_scalar dphi_now = executeGetDPhi(pathProvider, aircraft_state, rabbit_odometer, 0.0f);
    gp_scalar dtheta_now = executeGetDTheta(pathProvider, aircraft_state, rabbit_odometer, 0.0f);
    gp_vec3 targetPos = getInterpolatedTargetPosition(
        pathProvider, rabbit_odometer, 0.0f);
    gp_scalar dist_now = (targetPos - aircraft_state.getPosition()).norm();

    // Capture temporal history before NN evaluation
    aircraft_state.recordErrorHistory(dphi_now, dtheta_now, dist_now, millis());

    // Run NN: gathers 29 inputs, forward pass, sets pitch/roll/throttle commands
    generatedNNProgram(pathProvider, aircraft_state, 0.0f);

    // Convert NN-controlled aircraft commands to MSP RC values and cache them
    int roll_cmd = convertRollToMSPChannel(aircraft_state.getRollCommand());
    int pitch_cmd = convertPitchToMSPChannel(aircraft_state.getPitchCommand());
    int throttle_cmd = convertThrottleToMSPChannel(aircraft_state.getThrottleCommand());
    updateCachedCommands(roll_cmd, pitch_cmd, throttle_cmd, eval_start_us);
    pipeEvalEndUs = micros();

    // Log compact NN I/O: 29 inputs, 3 outputs (tanh), 3 RC commands
    // Inputs: [0-5]dPhi [6-11]dTheta [12-17]dist [18]dDist/dt [19-22]quat [23]airspeed [24]alpha [25]beta [26-28]cmdFeedback
    const float* in = aircraft_state.getNNInputs();
    const float* out = aircraft_state.getNNOutputs();
    logPrint(INFO,
             "NN: idx=%d in=[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f] out=[%.3f,%.3f,%.3f] rc=[%d,%d,%d] rabbit=[%.2f,%.2f,%.2f]",
             current_path_index,
             in[0], in[1], in[2], in[3], in[4], in[5],       // dPhi history+forecast
             in[6], in[7], in[8], in[9], in[10], in[11],     // dTheta history+forecast
             in[12], in[13], in[14], in[15], in[16], in[17],  // dist history+forecast
             in[18],                                           // dDist/dt
             in[19], in[20], in[21], in[22],                   // quaternion w,x,y,z
             in[23],                                           // airspeed
             in[24], in[25],                                   // alpha, beta
             in[26], in[27], in[28],                           // cmd feedback
             out[0], out[1], out[2],                           // NN outputs (tanh)
             roll_cmd, pitch_cmd, throttle_cmd,                // RC commands
             targetPos.x(), targetPos.y(), targetPos.z());     // rabbit world position (origin-relative)
  }
}

void msplinkSetup()
{
  // Initialize MSPLink input serial1 port
  Serial1.begin(115200);
  msp.begin(Serial1, MSP_REPLY_TIMEOUT_MSEC);
  logPrint(INFO, "MSPLink Reader Started");

  // set 'valid' values for now
  for (int i = 0; i < MSP_MAX_SUPPORTED_CHANNELS; i++)
  {
    state.command_buffer.channel[i] = MSP_DEFAULT_CHANNEL_VALUE;
  }

  // No ticker — single 20Hz loop in controllerUpdate() handles sends
  pipelineStats.reset();
}

void mspUpdateState()
{
  pipeTickStartUs = micros();
  state.resetState();
  state.setAsOfMsec(millis());

  // get status
  state.status_valid = performMspRequest(MSP_STATUS, &state.status, sizeof(state.status));
  if (!state.status_valid)
  {
    stopAutoc("MSP status failure", true);
    logPrint(ERROR, "*** CRITICAL: Failed to get MSP_STATUS - aborting MSP update cycle");
    return; // Critical error - can't continue without status
  }

  // Local state (position / velocity / quaternion)
  state.local_state_valid = performMspRequest(MSP2_INAV_LOCAL_STATE, &state.local_state, sizeof(state.local_state));
  if (state.local_state_valid)
  {
    // Convert INAV timestamp from microseconds to milliseconds
    state.inavSampleTimeMsec = state.local_state.timestamp_us / 1000;
  }
  // Local state already provides quaternion data; if unavailable, orientation will be held from previous sample
  if (!state.local_state_valid && (state.autoc_enabled || rabbit_active))
  {
    stopAutoc("MSP local state failure", true);
  }

  // RC channels
  state.rc_valid = performMspRequest(MSP_RC, &state.rc, sizeof(state.rc));
  if (!state.rc_valid && (state.autoc_enabled || rabbit_active))
  {
    stopAutoc("MSP RC failure", true);
  }

  // Sample path selector channel every cycle (for pre-flight validation and GP State logging)
  int pathSelectorRcValue = MSP_DEFAULT_CHANNEL_VALUE;
  int pathSelectorIndex = 0;  // Default to path 0 (StraightAndLevel)

  if (state.rc_valid) {
    pathSelectorRcValue = state.rc.channelValue[MSP_PATH_SELECT_CHANNEL];
    // Map 1000-2000 → 0-5 (6-position switch)
    // Each position gets ~167μs range: 1000, 1200, 1400, 1600, 1800, 2000
    int clamped = constrain(pathSelectorRcValue, 1000, 2000);
    pathSelectorIndex = min(5, (clamped - 1000) * 6 / 1001);  // Scale to 0-5
  }

  // ok, let's see what we fetched and updated.
  // first, check if we have a valid status
  bool isArmed = state.status_valid && state.status.flightModeFlags & (1 << MSP_MODE_ARM);
  if (isArmed)
  {
    analogWrite(BLUE_PIN, 0);
  }
  else
  {
    analogWrite(BLUE_PIN, 255);
  }

  // Manage flash logging and BLE state based on arm transitions
  if (isArmed != was_system_armed)
  {
    if (isArmed)
    {
      blueToothSetEnabled(false);
      if (!flashLoggerBeginFlight())
      {
        logPrint(ERROR, "Flash logger failed to start new flight on arm");
      }
    }
    else
    {
      flashLoggerEndFlight();
      blueToothSetEnabled(true);
    }
    was_system_armed = isArmed;
  }

  // then, check the servo channel to see if can auto-enable
  bool hasServoActivation = state.rc_valid && state.rc.channelValue[MSP_ARM_CHANNEL] > MSP_ARMED_THRESHOLD;

  if (!isArmed && (state.autoc_enabled || rabbit_active))
  {
    stopAutoc("disarmed", true);
  }

  bool hadServoLatch = servo_reset_required;
  if (!hasServoActivation)
  {
    if (hadServoLatch)
    {
      logPrint(INFO, "Nav Control: Servo reset detected - autoc re-arm allowed");
    }
    servo_reset_required = false;
    state.autoc_countdown = 0;
  }
  else if (!servo_reset_required && isArmed)
  {
    state.autoc_countdown++;
  }
  else
  {
    state.autoc_countdown = 0;
  }

  // Countdown timer logic to determine autoc_enabled state
  bool new_autoc_enabled = state.autoc_countdown > MSP_ARM_CYCLE_COUNT;

  // Handle state transitions
  if (new_autoc_enabled && !state.autoc_enabled)
  {
    resetPositionHistory();

    if (state.local_state_valid)
    {
      test_origin_offset = neuVectorToNedMeters(state.local_state.pos); // capture absolute INAV position at enable
      test_origin_set = true;

      // Use cached path selector value from cycle sampling
      int pathIndex = pathSelectorIndex;
      selected_path_index = pathIndex;  // Store armed path index for reference

      // Generate selected path with seed for reproducibility
      const char* pathNames[] = {
        "StraightAndLevel",
        "SpiralClimb",
        "HorizontalFigureEight",
        "FortyFiveDegreeAngledLoop",
        "HighPerchSplitS",
        "SeededRandomB"
      };

      uint32_t generation_start_us = micros();
      // Generate path at canonical origin (0,0,0) - craft is already at virtual (0,0,0)
      path_generator.generatePath(pathIndex, 0.0f, EMBEDDED_PATH_SEED);
      uint32_t generation_duration_us = micros() - generation_start_us;

      path_generator.copyToVector(flight_path);

      // Log path generation summary
      logPrint(INFO, "Path armed: %d=%s, %d/%d segments, origin=(0,0,0), seed=%u, time=%.1fms",
               pathIndex, pathNames[pathIndex], (int)flight_path.size(), MAX_EMBEDDED_PATH_SEGMENTS,
               EMBEDDED_PATH_SEED, generation_duration_us / 1000.0f);

      if (path_generator.wasTruncated()) {
        logPrint(WARNING, "*** Path was TRUNCATED at MAX_EMBEDDED_PATH_SEGMENTS=%d ***",
                 MAX_EMBEDDED_PATH_SEGMENTS);
      }

      rabbit_start_time = millis();
      rabbit_active = true;
      rabbit_odometer = 0.0f;
      pipelineStats.reset();
      // No ticker — single 20Hz loop in controllerUpdate() handles sends
      current_path_index = 0;

      state.autoc_enabled = true;
      servo_reset_required = false;
      analogWrite(GREEN_PIN, 0);
      logPrint(INFO, "NN Control: Switch enabled - origin NED=[%.2f, %.2f, %.2f] - program=%s",
               test_origin_offset.x(), test_origin_offset.y(), test_origin_offset.z(),
               generatedNNProgramSource);
    }
    else
    {
      stopAutoc("missing local state", true);
      logPrint(ERROR, "*** FATAL: No valid local state available for NN control - cannot enable autoc");
    }
  }
  else if (!new_autoc_enabled && state.autoc_enabled)
  {
    if (rabbit_active)
    {
      unsigned long test_run_duration = millis() - rabbit_start_time;
      logPrint(INFO, "Nav Control: Switch disabled (%.1fs) - stopping test run", test_run_duration * GP_INV_1000);
    }
    if (!isArmed)
    {
      stopAutoc("disarmed", true);
    }
    else if (!hasServoActivation)
    {
      stopAutoc("servo switch", false);
    }
    else
    {
      stopAutoc("autoc cancelled", true);
    }
  }

  // Mark end of MSP fetch phase
  pipeFetchEndUs = micros();

  // Update aircraft state on every MSP cycle for continuous position/velocity tracking
  convertMSPStateToAircraftState(aircraft_state);

  // Log raw vs origin-relative state for correlation
  gp_vec3 pos_raw = gp_vec3::Zero();
  gp_vec3 vel_raw = gp_vec3::Zero();
  if (state.local_state_valid)
  {
    pos_raw = neuVectorToNedMeters(state.local_state.pos);
    vel_raw = neuVectorToNedMeters(state.local_state.vel);
  }
  else if (have_valid_position)
  {
    pos_raw = last_valid_position;
  }
  gp_vec3 pos_rel = aircraft_state.getPosition();
  gp_vec3 vel_rel = aircraft_state.getVelocity();
  gp_quat q = aircraft_state.getOrientation();
  if (q.norm() > 0.0f)
  {
    q.normalize();
  }
  bool armed = state.status_valid && state.status.flightModeFlags & (1UL << MSP_MODE_ARM);
  bool failsafe = state.status_valid && state.status.flightModeFlags & (1UL << MSP_MODE_FAILSAFE);

  logPrint(INFO,
           "Nav State: pos_raw=[%.2f,%.2f,%.2f] pos=[%.2f,%.2f,%.2f] vel=[%.2f,%.2f,%.2f] quat=[%.3f,%.3f,%.3f,%.3f] armed=%s fs=%s servo=%s autoc=%s rabbit=%s path=%d",
           pos_raw.x(), pos_raw.y(), pos_raw.z(),
           pos_rel.x(), pos_rel.y(), pos_rel.z(),
           vel_rel.x(), vel_rel.y(), vel_rel.z(),
           q.w(), q.x(), q.y(), q.z(),
           armed ? "Y" : "N",
           failsafe ? "Y" : "N",
           hasServoActivation ? "Y" : "N",
           state.autoc_enabled ? "Y" : "N",
           rabbit_active ? "Y" : "N",
           pathSelectorIndex);

  // Update NN control and cache commands when enabled
  mspUpdateNavControl();
}

void mspSetControls()
{
  if (!rabbit_active)
  {
    return;
  }
  performMspSendLocked();

  // Record pipeline timing on NN eval ticks (when pipeTickStartUs was set this cycle)
  if (pipeEvalEndUs > pipeTickStartUs && pipeTickStartUs > 0)
  {
    uint32_t sendEndUs = micros();
    uint32_t fetchUs = pipeFetchEndUs - pipeTickStartUs;
    uint32_t evalUs = pipeEvalEndUs - pipeFetchEndUs;
    uint32_t sendUs = sendEndUs - pipeEvalEndUs;
    pipelineStats.recordTick(fetchUs, evalUs, sendUs, pipeTickStartUs);
    pipeTickStartUs = 0;  // prevent double-counting on send-only ticks
  }
}

static void updateCachedCommands(int roll, int pitch, int throttle, uint32_t evalStartUs)
{
  uint32_t evalEndUs = micros();
  // No ISR contention — single-threaded 20Hz loop
  cached_roll_cmd = roll;
  cached_pitch_cmd = pitch;
  cached_throttle_cmd = throttle;
  cached_eval_start_us = evalStartUs;
  cached_eval_complete_us = evalEndUs;
  cached_cmd_sequence++;
}

static bool tryLockMspBusFromTask()
{
  bool locked = false;
  noInterrupts();
  if (!mspBusLocked)
  {
    mspBusLocked = true;
    locked = true;
  }
  interrupts();
  return locked;
}

static bool lockMspBusBlockingFromTask()
{
  unsigned long start = micros();
  while (!tryLockMspBusFromTask())
  {
    if (micros() - start >= MSP_BUS_LOCK_TIMEOUT_USEC)
    {
      logPrint(ERROR, "MSP bus lock timeout - MSP command %s", state.autoc_enabled ? "during autoc" : "idle");
      return false;
    }
    delayMicroseconds(50);
  }
  return true;
}

static void releaseMspBusFromTask()
{
  mspBusLocked = false;
}

static bool performMspRequest(uint16_t command, void *buffer, size_t size)
{
  if (!lockMspBusBlockingFromTask())
  {
    return false;
  }
  bool success = msp.request(command, buffer, size);
  releaseMspBusFromTask();
  return success;
}

static void performMspSendLocked()
{
  state.command_buffer.channel[0] = cached_roll_cmd;
  state.command_buffer.channel[1] = cached_pitch_cmd;
  state.command_buffer.channel[2] = cached_throttle_cmd;
  msp.send(MSP_SET_RAW_RC, &state.command_buffer, sizeof(state.command_buffer));
}

// Old per-send log infrastructure removed — replaced by PipelineStats

// Convert MSP state data to AircraftState for GP evaluator
void convertMSPStateToAircraftState(AircraftState &aircraftState)
{
  if (!state.local_state_valid)
  {
    if (!have_valid_position)
    {
      return;
    }
  }

  gp_vec3 position_raw = have_valid_position ? last_valid_position : gp_vec3(0.0f, 0.0f, 0.0f);
  gp_vec3 position_rel = position_raw;
  gp_vec3 velocity = gp_vec3::Zero();
  gp_quat orientation = aircraftState.getOrientation();

  if (state.local_state_valid)
  {
    position_raw = neuVectorToNedMeters(state.local_state.pos);
    velocity = neuVectorToNedMeters(state.local_state.vel);
    orientation = neuQuaternionToNed(state.local_state.q);

    last_valid_position = position_raw;
    have_valid_position = true;
  }
  if (test_origin_set)
  {
    position_rel = position_raw - test_origin_offset; // express aircraft_state in virtual-origin frame
  }
  else
  {
    position_rel = position_raw;
  }

  // Paths now generate at canonical (0,0,0); craft already at virtual (0,0,0) when armed
  // If no new quaternion data, retain previous orientation from aircraft state

  gp_scalar speed_magnitude = velocity.norm();

  aircraftState.setPosition(position_rel);
  aircraftState.setOrientation(orientation);
  aircraftState.setVelocity(velocity);
  aircraftState.setRelVel(speed_magnitude);
  aircraftState.setSimTimeMsec(state.asOfMsec);
}

// Find path index based on rabbit odometer (distance along path)
int getRabbitPathIndex(unsigned long /* elapsed_msec */)
{
  if (flight_path.empty())
    return 0;

  // Linear scan from current point to find the path segment just beyond the odometer
  for (size_t i = current_path_index; i < flight_path.size(); i++)
  {
    if (flight_path[i].distanceFromStart >= rabbit_odometer)
    {
      return (int)i;
    }
  }

  // If odometer has gone beyond the path, return last segment
  return (int)(flight_path.size() - 1);
}

// MSP channel conversion functions with correct polarity
int convertRollToMSPChannel(gp_scalar gp_command)
{
  // Roll: GP +1.0 = roll right = MSP 2000 (DIRECT mapping)
  gp_scalar clamped = CLAMP_DEF(gp_command, -1.0f, 1.0f);
  return (int)(1500.0f + clamped * 500.0f);
}

int convertPitchToMSPChannel(gp_scalar gp_command)
{
  // Pitch: GP +1.0 = pitch up = MSP 1000 (INVERTED mapping to match CRRCSim)
  gp_scalar clamped = CLAMP_DEF(gp_command, -1.0f, 1.0f);
  return (int)(1500.0f - clamped * 500.0f);
}

int convertThrottleToMSPChannel(gp_scalar gp_command)
{
  // Throttle: GP +1.0 = full throttle = MSP 2000 (DIRECT mapping)
  gp_scalar clamped = CLAMP_DEF(gp_command, -1.0f, 1.0f);
  return (int)(1500.0f + clamped * 500.0f);
}
// Ticker infrastructure removed — single 20Hz loop in controllerUpdate() handles all sends.
