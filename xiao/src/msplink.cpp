#include <main.h>
#include <autoc/eval/aircraft_state.h>
#include <embedded_pathgen_selector.h>
#include <autoc/eval/sensor_math.h>
#include <autoc/nn/nn_input_computation.h>
#include <autoc/imu/inav_quat_convention.h>
#include <nn_program.h>
#include <flight_log.h>
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

// 039 FR-001 / D5 — engage-scoped arena. Resolved in the span-activation
// path (below) from the codegen template + the engage point, BEFORE
// rabbit_active goes true; every NN eval reads it via nnActiveArena().
// Zero-initialized static until the first engage — never read before then
// (rabbit_active gates all consumers).
static autoc::eval::EngageArena engage_arena = {};

// Link-time contract with the generated NN code (039 D5): the consumer owns
// the active arena. Returns the current span's resolved virtual-frame arena.
const autoc::eval::FlightArena& nnActiveArena() { return engage_arena.virtual_arena; }

// Nav Control Timing and State
static unsigned long rabbit_start_time = 0;
static volatile bool rabbit_active = false;
static int current_path_index = 0;

// 039 US3 — binary flight-log span bookkeeping (flight_log_format.h).
static uint16_t span_id_counter = 0;     // flight-unique, reset at arm
static bool nn_warmup_tick = false;      // true for the first tick after engage (recurrent_reset marker)

// 039 T019 — DWT cycle count of one unrolled NN eval; measured once per boot
// at the first eval (one number per firmware image), carried in every
// SpanSummary + reported on the console.
static uint32_t dwt_eval_cycles = 0;
constexpr gp_scalar XIAO_RABBIT_SPEED_MPS = 12.0f;  // Rabbit speed for xiao (m/s) — matches autoc.ini RabbitSpeedNominal
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

// Controller-loop cadence stats (struct declared in main.h).
// Populated by controllerUpdate() per tick; reset alongside pipelineStats
// at engage-span start; summarized in stopAutoc when the span ends.
LoopStats loopStats;

// Aircraft state tracking for position/velocity calculation
static gp_vec3 last_valid_position(0.0f, 0.0f, 0.0f);
static bool have_valid_position = false;
static bool was_system_armed = false;
// v3 flag-transition events: last states WRITTEN to the flight log (seeded
// right after kEventArm so transitions decode without prior context).
static bool logged_failsafe_state = false;
static bool logged_servo_state = false;

// Safety timeout for single test run (60 seconds max per run)
#define GP_MAX_SINGLE_RUN_MSEC (60 * 1000)
#define MSP_BUS_LOCK_TIMEOUT_USEC (MSP_REPLY_TIMEOUT_MSEC * 2000UL)

// Forward declarations for MSP scheduling helpers
static void printCandidateBanner();
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

// Console string for a DisengageReason (the enum is what goes into the
// v3 kEventDisengageReason record; the string is display-only).
static const char* disengageReasonStr(flightlog::DisengageReason reason)
{
  switch (reason)
  {
    case flightlog::kReasonServoSwitch: return "servo switch";
    case flightlog::kReasonFailsafe: return "failsafe";
    case flightlog::kReasonDisarmed: return "disarmed";
    case flightlog::kReasonTimeout: return "timeout";
    case flightlog::kReasonPathComplete: return "path complete";
    case flightlog::kReasonMspStateFailure: return "MSP autoc state failure";
    case flightlog::kReasonMissingLocalState: return "missing local state";
    case flightlog::kReasonAutocCancelled: return "autoc cancelled";
    default: return "unknown";
  }
}

static void stopAutoc(flightlog::DisengageReason reason, bool requireServoReset)
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
    logPrint(INFO, "Nav Control: Autoc disabled (%s) - pilot has control",
             disengageReasonStr(reason));
  }

  if (wasRabbit)
  {
    // 039 US3 — span-summary record: loopStats + MSP pipeline stats into the
    // binary log (the latency memo's flight-side numbers, FR-011/FR-014 —
    // the console lines below are event-class, once per span).
    {
      flightlog::SpanSummary ss;
      memset(&ss, 0, sizeof(ss));
      ss.timestamp_ms = millis();
      ss.span_id = span_id_counter;
      ss.ticks = loopStats.ticks;
      ss.overruns = loopStats.overruns;
      ss.resyncs = loopStats.resyncs;
      ss.max_late_ms = loopStats.maxLateMs;
      ss.total_late_ms = loopStats.totalLateMs;
      ss.samples = pipelineStats.samples;
      if (pipelineStats.samples > 0) {
        ss.fetch_min_us = pipelineStats.fetchMinUs;
        ss.fetch_avg_us = pipelineStats.fetchSumUs / pipelineStats.samples;
        ss.fetch_max_us = pipelineStats.fetchMaxUs;
        ss.eval_min_us = pipelineStats.evalMinUs;
        ss.eval_avg_us = pipelineStats.evalSumUs / pipelineStats.samples;
        ss.eval_max_us = pipelineStats.evalMaxUs;
        ss.send_min_us = pipelineStats.sendMinUs;
        ss.send_avg_us = pipelineStats.sendSumUs / pipelineStats.samples;
        ss.send_max_us = pipelineStats.sendMaxUs;
        ss.total_min_us = pipelineStats.totalMinUs;
        ss.total_avg_us = pipelineStats.totalSumUs / pipelineStats.samples;
        ss.total_max_us = pipelineStats.totalMaxUs;
      }
      if (pipelineStats.intervalCount > 0) {
        ss.interval_min_us = pipelineStats.intervalMinUs;
        ss.interval_avg_us = pipelineStats.intervalSumUs / pipelineStats.intervalCount;
        ss.interval_max_us = pipelineStats.intervalMaxUs;
      }
      ss.dwt_eval_cycles = dwt_eval_cycles;
      flightLogSpanSummary(ss);
      flightLogEvent(flightlog::kEventDisengage, span_id_counter);
      // v3: why the span ended + an INAV clock anchor at the boundary
      flightLogEvent(flightlog::kEventDisengageReason, reason);
      flightLogEvent(flightlog::kEventInavClock, (uint32_t)state.inavSampleTimeMsec);
    }

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
    // Controller loop cadence — ERROR level if any tick overran the
    // MSP_LOOP_INTERVAL_MSEC budget during this engage span, INFO otherwise.
    if (loopStats.ticks > 0) {
      LogLevel ctlLevel = (loopStats.overruns > 0) ? ERROR : INFO;
      float avgLate = (float)loopStats.totalLateMs / (float)loopStats.ticks;
      logPrint(ctlLevel,
        "ctl loop: ticks=%u overruns=%u resyncs=%u maxLate=%ums avgLate=%.2fms",
        loopStats.ticks, loopStats.overruns, loopStats.resyncs,
        loopStats.maxLateMs, avgLate);
    }
    // Logging health for the bench review (FR-008): drops must be visible.
    if (flightLogTicksDropped() > 0) {
      logPrint(WARNING, "flight log: %lu ticks logged, %lu DROPPED under buffer pressure",
               (unsigned long)flightLogTicksLogged(), (unsigned long)flightLogTicksDropped());
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

// INAV quaternion → aerospace q_EB (static-lookup-valid ONLY).
// Delegates to autoc::imu::inavQuatToAerospaceEB().
//
// ⚠️ Result is NOT a kinematically-valid NED q_EB. Do not feed into
// Kalman/Mahony/Madgwick filters that also integrate body rates. Valid
// for: direction cosines, Euler extraction, body-axis-in-world rotation
// for rendering. See docs/COORDINATE_CONVENTIONS.md
// "INAV NEU ↔ aerospace NED reflection" and the header for the full
// rationale and empirical derivation (flight-20260417 at-rest accel
// verification).
//
// Kept as a thin wrapper (not inlined at call sites) so grep still finds
// "neuQuaternionToNed" as the boundary.
static gp_quat neuQuaternionToNed(const float q[4])
{
  return autoc::imu::inavQuatToAerospaceEB(q);
}

// Pipeline timing points (set during mspUpdateState, read during mspSetControls)
static uint32_t pipeTickStartUs = 0;
static uint32_t pipeFetchEndUs = 0;
static uint32_t pipeEvalEndUs = 0;

// 039 FR-014 (operator 2026-07-11): 1 Hz console heartbeat carrying the old
// per-tick "Nav State:" content — the base diagnostic to watch on the bench.
// Called from BOTH the healthy path and the MSP-fetch-failure path (mspOK=N,
// last-known state) so the console is never silent while the loop runs.
// pathIdx = -1 when the selector could not be sampled this cycle.
static void consoleHeartbeat(bool hasServoActivation, int pathIdx)
{
  static unsigned long last_heartbeat_ms = 0;
  unsigned long now_ms = millis();
  if (now_ms - last_heartbeat_ms < 1000)
  {
    return;
  }
  last_heartbeat_ms = now_ms;

  gp_vec3 pos_raw = have_valid_position ? last_valid_position : gp_vec3::Zero();
  gp_vec3 pos_rel = aircraft_state.getPosition();
  gp_vec3 vel = aircraft_state.getVelocity();
  gp_quat q = aircraft_state.getOrientation();
  if (q.norm() > 0.0f)
  {
    q.normalize();
  }
  gp_vec3 gyro = aircraft_state.getGyroRates();  // aerospace convention (rad/s)
  logPrint(INFO,
           "hb: mspOK=%s pos_raw=[%.2f,%.2f,%.2f] pos=[%.2f,%.2f,%.2f] vel=[%.2f,%.2f,%.2f] quat=[%.3f,%.3f,%.3f,%.3f] gyro=[%.2f,%.2f,%.2f] armed=%s fs=%s servo=%s autoc=%s rabbit=%s path=%d span=%u ticks=%lu drops=%lu",
           state.autoc_state_valid ? "Y" : "N",
           pos_raw.x(), pos_raw.y(), pos_raw.z(),
           pos_rel.x(), pos_rel.y(), pos_rel.z(),
           vel.x(), vel.y(), vel.z(),
           q.w(), q.x(), q.y(), q.z(),
           gyro.x(), gyro.y(), gyro.z(),
           state.isArmed() ? "Y" : "N",
           state.isFailsafe() ? "Y" : "N",
           hasServoActivation ? "Y" : "N",
           state.autoc_enabled ? "Y" : "N",
           rabbit_active ? "Y" : "N",
           pathIdx,
           (unsigned)span_id_counter,
           (unsigned long)flightLogTicksLogged(),
           (unsigned long)flightLogTicksDropped());
}

static void mspUpdateNavControl()
{
  // Check for disarm or failsafe conditions before NN control
  // Only check if we're currently enabled to avoid repeat logging
  if (state.autoc_enabled && rabbit_active)
  {
    bool isArmed = state.isArmed();
    bool isFailsafe = state.isFailsafe();

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
      stopAutoc(isFailsafe ? flightlog::kReasonFailsafe : flightlog::kReasonDisarmed, true);
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
    stopAutoc(flightlog::kReasonTimeout, true);
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
    stopAutoc(flightlog::kReasonPathComplete, true);
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

    // Direction cosines: compute target unit vector in body frame (023)
    // Same pattern as minisim.cc and crrcsim inputdev_autoc.cpp
    gp_vec3 targetPos = getInterpolatedTargetPosition(
        pathProvider, rabbit_odometer, 0.0f);
    gp_vec3 craftToTarget = targetPos - aircraft_state.getPosition();
    gp_vec3 target_local = aircraft_state.getOrientation().inverse() * craftToTarget;
    float dist_now = static_cast<float>(target_local.norm());

    // Path tangent for singularity fallback (dist < 1e-4m)
    gp_vec3 posAhead = getInterpolatedTargetPosition(pathProvider, rabbit_odometer, 0.5f);
    gp_vec3 tangent = posAhead - targetPos;
    double tn = tangent.norm();
    gp_vec3 tangent_body = (tn > 1e-6)
        ? aircraft_state.getOrientation().inverse() * (tangent / tn)
        : gp_vec3::UnitX();

    gp_vec3 dir = computeTargetDir(target_local, dist_now, tangent_body);

    // Record direction cosines to temporal history before NN evaluation
    aircraft_state.setRabbitPosition(targetPos);
    aircraft_state.recordErrorHistory(dir, dist_now, millis());

    // Run NN: gathers 37 inputs (038 contract), forward pass, sets
    // pitch/roll/throttle commands. T019: the first eval after boot is
    // cycle-counted via DWT (one number per firmware image).
    if (dwt_eval_cycles == 0) {
      uint32_t c0 = DWT->CYCCNT;
      generatedNNProgram(pathProvider, aircraft_state, 0.0f);
      dwt_eval_cycles = DWT->CYCCNT - c0;
      logPrint(INFO, "NN eval cost: %lu cycles (%.1f us @64MHz, gather+forward, unrolled)",
               (unsigned long)dwt_eval_cycles, dwt_eval_cycles / 64.0f);
    } else {
      generatedNNProgram(pathProvider, aircraft_state, 0.0f);
    }

    // Convert NN-controlled aircraft commands to MSP RC values and cache them
    int roll_cmd = convertRollToMSPChannel(aircraft_state.getRollCommand());
    int pitch_cmd = convertPitchToMSPChannel(aircraft_state.getPitchCommand());
    int throttle_cmd = convertThrottleToMSPChannel(aircraft_state.getThrottleCommand());
    updateCachedCommands(roll_cmd, pitch_cmd, throttle_cmd, eval_start_us);
    pipeEvalEndUs = micros();

    // 039 US3 (T013/FR-014): per-tick recording goes to the binary flight log
    // ONLY — the interim NN: text line is deleted (no parallel writers,
    // Constitution III). Honest recording: the 37 floats are the post-gather
    // values the NN consumed this tick (NNInputs layout = PathgenInput slot
    // order = the log's scale-table order). v2 telemetry: craft pos/vel
    // (virtual frame / NED) + ground-truth rabbit for the renderer.
    const float* in = reinterpret_cast<const float*>(&aircraft_state.getNNInputs());
    const float* out = aircraft_state.getNNOutputs();
    const uint16_t rc_sent[3] = {(uint16_t)roll_cmd, (uint16_t)pitch_cmd,
                                 (uint16_t)throttle_cmd};
    const gp_vec3 craft_pos = aircraft_state.getPosition();
    const gp_vec3 craft_vel = aircraft_state.getVelocity();
    const float pos_t[3] = {(float)craft_pos.x(), (float)craft_pos.y(), (float)craft_pos.z()};
    const float vel_t[3] = {(float)craft_vel.x(), (float)craft_vel.y(), (float)craft_vel.z()};
    const float rabbit_t[3] = {(float)targetPos.x(), (float)targetPos.y(), (float)targetPos.z()};
    flightLogTick(current_time, in, out, pos_t, vel_t, rabbit_t, nn_warmup_tick,
                  (int8_t)selected_path_index, rc_sent,
                  state.autoc_state_valid);
    nn_warmup_tick = false;
  }
}

// 039 candidate identity banner (FR-002 item 1): topology + ids baked by
// nn2cpp; must match the intended flight candidate. Printed at setup and at
// every autoc engage (operator 2026-07-11, matching the old switch-enable
// identity print — boot-time output is unseen since the monitor attaches
// after USB re-enumeration).
static void printCandidateBanner()
{
  const autoc::eval::FlightArena& tpl = generatedNNProgramArenaTemplate();
  logPrint(INFO,
           "NN candidate: topology=%s inputs=%d weights=%d weight_id=%02x%02x%02x%02x%02x%02x%02x%02x firmware_id=%02x%02x%02x%02x%02x%02x%02x%02x arenaTemplate=[R=%.0f F=%.0f C=%.0f]",
           generatedNNTopologyString, generatedNNInputCount, generatedNNWeightCount,
           generatedNNWeightId[0], generatedNNWeightId[1], generatedNNWeightId[2],
           generatedNNWeightId[3], generatedNNWeightId[4], generatedNNWeightId[5],
           generatedNNWeightId[6], generatedNNWeightId[7],
           generatedNNFirmwareId[0], generatedNNFirmwareId[1], generatedNNFirmwareId[2],
           generatedNNFirmwareId[3], generatedNNFirmwareId[4], generatedNNFirmwareId[5],
           generatedNNFirmwareId[6], generatedNNFirmwareId[7],
           tpl.radius_m, tpl.floor_agl_m, tpl.ceiling_agl_m);
}

void msplinkSetup()
{
  // Initialize MSPLink input serial1 port
  Serial1.begin(115200);
  msp.begin(Serial1, MSP_REPLY_TIMEOUT_MSEC);
  logPrint(INFO, "MSPLink Reader Started");

  printCandidateBanner();

  // set 'valid' values for now
  for (int i = 0; i < MSP_MAX_SUPPORTED_CHANNELS; i++)
  {
    state.command_buffer.channel[i] = MSP_DEFAULT_CHANNEL_VALUE;
  }

  // No ticker — single 20Hz loop in controllerUpdate() handles sends
  pipelineStats.reset();
  loopStats.reset();

  // 039 T019 — enable the DWT cycle counter for the one-shot NN eval cost
  // measurement (037 eval-cycle-harness design: DWT->CYCCNT, Cortex-M4).
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void mspUpdateState()
{
  pipeTickStartUs = micros();
  state.resetState();
  state.setAsOfMsec(millis());

  // Single consolidated MSP call: nav + status + rc
  state.autoc_state_valid = performMspRequest(MSP2_AUTOC_STATE, &state.autoc_state, sizeof(state.autoc_state));
  if (state.autoc_state_valid)
  {
    state.inavSampleTimeMsec = state.autoc_state.timestamp_us / 1000;
  }
  if (!state.autoc_state_valid)
  {
    if (state.autoc_enabled || rabbit_active)
    {
      stopAutoc(flightlog::kReasonMspStateFailure, true);
    }
    flightLogEvent(flightlog::kEventFetchTimeout, 1);
    // Rate-limit the console error to 1 Hz (at divisor=1 this path fires at
    // 20 Hz when INAV is absent — e.g. bench without the FC powered).
    {
      static unsigned long last_fetch_err_ms = 0;
      static uint32_t fetch_errs_suppressed = 0;
      unsigned long now_err_ms = millis();
      if (now_err_ms - last_fetch_err_ms >= 1000)
      {
        logPrint(ERROR, "*** CRITICAL: Failed to get MSP2_AUTOC_STATE (%lu more suppressed) - aborting MSP update cycle",
                 (unsigned long)fetch_errs_suppressed);
        last_fetch_err_ms = now_err_ms;
        fetch_errs_suppressed = 0;
      }
      else
      {
        fetch_errs_suppressed++;
      }
    }
    consoleHeartbeat(false, -1);
    return;
  }

  // Sample path selector channel every cycle (for pre-flight validation and GP State logging)
  int pathSelectorRcValue = state.rcChannel(MSP_PATH_SELECT_CHANNEL);
  // Map 1000-2000 → 0-5 (6-position switch)
  int clamped = constrain(pathSelectorRcValue, 1000, 2000);
  int pathSelectorIndex = min(5, (clamped - 1000) * 6 / 1001);

  bool isArmed = state.isArmed();
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
      else
      {
        // 039 US3: FileHeader opens the flight file (identity + scale table);
        // span ids restart per flight.
        span_id_counter = 0;
        flightLogBeginFile(generatedNNFirmwareId, generatedNNWeightId,
                           generatedNNProgramSource, MSP_LOOP_INTERVAL_MSEC);
        flightLogEvent(flightlog::kEventArm, flashLoggerGetCurrentFlightNumber());
        // v3 arm→disarm self-containment: INAV clock anchor pair + initial
        // failsafe/servo states (transitions are logged as they happen).
        flightLogEvent(flightlog::kEventInavClock, (uint32_t)state.inavSampleTimeMsec);
        logged_failsafe_state = state.isFailsafe();
        logged_servo_state = state.autoc_state_valid &&
                             state.rcChannel(MSP_ARM_CHANNEL) > MSP_ARMED_THRESHOLD;
        flightLogEvent(flightlog::kEventFailsafe, logged_failsafe_state ? 1 : 0);
        flightLogEvent(flightlog::kEventServoSwitch, logged_servo_state ? 1 : 0);
      }
    }
    else
    {
      // v3: closing INAV clock anchor, then disarm — before logging suspends
      flightLogEvent(flightlog::kEventInavClock, (uint32_t)state.inavSampleTimeMsec);
      flightLogEvent(flightlog::kEventDisarm, 0);
      flashLoggerEndFlight();
      blueToothSetEnabled(true);
    }
    was_system_armed = isArmed;
  }

  // then, check the servo channel to see if can auto-enable
  bool hasServoActivation = state.autoc_state_valid && state.rcChannel(MSP_ARM_CHANNEL) > MSP_ARMED_THRESHOLD;

  // v3: failsafe / servo-switch transition events while the flight file is open
  if (isArmed)
  {
    const bool fs_now = state.isFailsafe();
    if (fs_now != logged_failsafe_state)
    {
      logged_failsafe_state = fs_now;
      flightLogEvent(flightlog::kEventFailsafe, fs_now ? 1 : 0);
    }
    if (hasServoActivation != logged_servo_state)
    {
      logged_servo_state = hasServoActivation;
      flightLogEvent(flightlog::kEventServoSwitch, hasServoActivation ? 1 : 0);
    }
  }

  if (!isArmed && (state.autoc_enabled || rabbit_active))
  {
    stopAutoc(flightlog::kReasonDisarmed, true);
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

    if (state.autoc_state_valid)
    {
      test_origin_offset = neuVectorToNedMeters(state.autoc_state.pos); // capture absolute INAV position at enable
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

      // 039 FR-001 — re-center the arena on the engage point and zero the
      // recurrent NN state (nn_reset) BEFORE the span goes live.
      //
      // ⚠️ 041 P2-3: NOT a ±K rule any more. The band is asymmetric (+60 up /
      // −10 down), so the up and down extents are resolved separately —
      // a ±K placement would have given the aircraft ±35 m here: 25 m less room
      // above than it trained with and 25 m more below, with every logged number
      // looking entirely reasonable.
      engage_arena = autoc::eval::resolveEngageArena(
          generatedNNProgramArenaTemplate(), test_origin_offset);
      generatedNNProgramReset();
      logPrint(INFO,
               "Engage: arena origin NED=[%.2f,%.2f,%.2f] floorZ=%.1f ceilZ=%.1f up=%.1f down=%.1f - NN state reset",
               engage_arena.origin_ned.x(), engage_arena.origin_ned.y(),
               engage_arena.origin_ned.z(), engage_arena.floor_z_ned,
               engage_arena.ceiling_z_ned, engage_arena.up_m, engage_arena.down_m);

      // 039 US3 — EngageHeader with the RESOLVED arena (FR-001 provenance);
      // the first tick after this carries recurrent_reset = 1.
      span_id_counter++;
      {
        const float origin[3] = {
            (float)engage_arena.origin_ned.x(),
            (float)engage_arena.origin_ned.y(),
            (float)engage_arena.origin_ned.z()};
        flightLogEngage(span_id_counter, millis(), origin,
                        (float)engage_arena.floor_z_ned,
                        (float)engage_arena.ceiling_z_ned,
                        (int16_t)pathIndex);
        flightLogEvent(flightlog::kEventEngage, span_id_counter);
        // v3: INAV clock anchor at the engage boundary
        flightLogEvent(flightlog::kEventInavClock, (uint32_t)state.inavSampleTimeMsec);
      }
      nn_warmup_tick = true;

      rabbit_start_time = millis();
      rabbit_active = true;
      rabbit_odometer = 0.0f;
      pipelineStats.reset();
      loopStats.reset();
      // No ticker — single 20Hz loop in controllerUpdate() handles sends
      current_path_index = 0;

      // Pre-fill history buffer with initial geometry so the NN starts with
      // consistent direction cosines instead of zeros (023 resetHistory).
      // Same pattern as CRRCSim engage — uses path[0].start as target,
      // path tangent as singularity fallback.
      if (!flight_path.empty()) {
        gp_vec3 pathStart = flight_path[0].start;
        gp_vec3 tangent;
        if (flight_path.size() > 1)
          tangent = flight_path[1].start - flight_path[0].start;
        else
          tangent = gp_vec3::UnitX();
        double tn = tangent.norm();
        if (tn > 1e-6) tangent = tangent / tn;
        else tangent = gp_vec3::UnitX();
        aircraft_state.resetHistory(pathStart, tangent);
      }

      state.autoc_enabled = true;
      servo_reset_required = false;
      analogWrite(GREEN_PIN, 0);
      printCandidateBanner();
      logPrint(INFO, "NN Control: Switch enabled - origin NED=[%.2f, %.2f, %.2f] - program=%s",
               test_origin_offset.x(), test_origin_offset.y(), test_origin_offset.z(),
               generatedNNProgramSource);
    }
    else
    {
      stopAutoc(flightlog::kReasonMissingLocalState, true);
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
      stopAutoc(flightlog::kReasonDisarmed, true);
    }
    else if (!hasServoActivation)
    {
      stopAutoc(flightlog::kReasonServoSwitch, false);
    }
    else
    {
      stopAutoc(flightlog::kReasonAutocCancelled, true);
    }
  }

  // Mark end of MSP fetch phase
  pipeFetchEndUs = micros();

  // Update aircraft state on every MSP cycle for continuous position/velocity tracking
  convertMSPStateToAircraftState(aircraft_state);

  // 039 FR-014 console split: the per-tick "Nav State:" text line is DELETED
  // (all control-loop data lives in the binary flight log). The console gets
  // the 1 Hz nav-state heartbeat instead (consoleHeartbeat above).
  consoleHeartbeat(hasServoActivation, pathSelectorIndex);

  // Armed-but-not-engaged flight breadcrumb (raw INAV frame, 25 B/tick):
  // keeps the arm→disarm trace continuous for the renderer's all-flight
  // view; during spans the TickRecord carries the (virtual-frame) telemetry.
  if (isArmed && !rabbit_active && state.autoc_state_valid)
  {
    gp_vec3 p_raw = neuVectorToNedMeters(state.autoc_state.pos);
    gp_vec3 v_raw = neuVectorToNedMeters(state.autoc_state.vel);
    gp_quat q_raw = neuQuaternionToNed(state.autoc_state.q);
    const float fs_pos[3] = {(float)p_raw.x(), (float)p_raw.y(), (float)p_raw.z()};
    const float fs_vel[3] = {(float)v_raw.x(), (float)v_raw.y(), (float)v_raw.z()};
    const float fs_quat[4] = {(float)q_raw.w(), (float)q_raw.x(), (float)q_raw.y(), (float)q_raw.z()};
    flightLogFlightState(millis(), fs_pos, fs_vel, fs_quat);
  }

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
  uint16_t recvSize = 0;
  bool success = msp.request(command, buffer, size, &recvSize);
  releaseMspBusFromTask();

  // 041 P5-1 — REJECT a short reply. MSP::recv zero-fills the tail of an
  // undersized payload and still returns true, so a xiao flashed against
  // un-upgraded INAV would read accel = [0,0,0] and say nothing: the policy
  // would fly on four silent zeros it never trained with. That failure mode is
  // exactly what this feature exists to end, so it must be loud.
  //
  // The test is >=, not ==, on purpose: trailing fields appended by a LATER
  // INAV keep every existing offset valid and MSP::recv discards the surplus,
  // which is the property that makes flashing INAV ahead of the xiao safe.
  if (success && recvSize < size)
  {
    static unsigned long last_short_ms = 0;
    unsigned long now_ms = millis();
    if (now_ms - last_short_ms >= 1000)
    {
      logPrint(ERROR,
               "*** MSP cmd 0x%04x SHORT REPLY: %u bytes, expected >= %u — FC firmware is older than this xiao build. Flash INAV.",
               (unsigned)command, (unsigned)recvSize, (unsigned)size);
      last_short_ms = now_ms;
    }
    success = false;
  }
  return success;
}

static void performMspSendLocked()
{
  state.command_buffer.channel[0] = cached_roll_cmd;
  state.command_buffer.channel[1] = cached_pitch_cmd;
  state.command_buffer.channel[2] = cached_throttle_cmd;
  // CH6 (index 5) = 1000 → forces MANUAL mode (no INAV stabilization)
  // msp_override_channels bitmask must include bit 5 (CLI: set msp_override_channels = 47)
  state.command_buffer.channel[5] = 1000;
  msp.send(MSP_SET_RAW_RC, &state.command_buffer, sizeof(state.command_buffer));
}

// Old per-send log infrastructure removed — replaced by PipelineStats

// Convert MSP state data to AircraftState for GP evaluator
void convertMSPStateToAircraftState(AircraftState &aircraftState)
{
  if (!state.autoc_state_valid)
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

  if (state.autoc_state_valid)
  {
    position_raw = neuVectorToNedMeters(state.autoc_state.pos);
    velocity = neuVectorToNedMeters(state.autoc_state.vel);
    orientation = neuQuaternionToNed(state.autoc_state.q);

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

  // Gyro rates from INAV (deci-deg/s → rad/s, with sign correction)
  // INAV convention: pitch+ = nose DOWN, yaw+ = nose LEFT (inverted from aerospace RHR)
  // Standard aerospace RHR: pitch+ = nose UP, yaw+ = nose RIGHT
  // See COORDINATE_CONVENTIONS.md for full transform table.
  if (state.autoc_state_valid)
  {
    const gp_scalar deciDegToRadS = static_cast<gp_scalar>(M_PI / 1800.0f); // deci-deg/s → rad/s
    gp_scalar p =  static_cast<gp_scalar>(state.autoc_state.gyro[0]) * deciDegToRadS;  // roll: no sign change
    gp_scalar q = -static_cast<gp_scalar>(state.autoc_state.gyro[1]) * deciDegToRadS;  // pitch: NEGATE
    gp_scalar r = -static_cast<gp_scalar>(state.autoc_state.gyro[2]) * deciDegToRadS;  // yaw: NEGATE
    aircraftState.setGyroRates(gp_vec3(p, q, r));

    // 041 P5-3 — body specific force, milli-g FLU on the wire → g FRD here.
    // The SAME y/z flip as the quat's (w, x, -y, -z) and the gyro's pitch/yaw
    // negation above: this is the third quantity through the one boundary, and
    // it is deliberately the ONLY place the flip happens.
    // Steady level flight therefore reads [0, 0, -1] here, matching what the
    // sim stores via autoc/eval/specific_force.h. ⚠️ Do NOT "correct" that
    // against INAV's bench table (+1 g on its normal axis) — that table is FLU.
    // Bench-proven on the wire 2026-08-22; see COORDINATE_CONVENTIONS.md.
    // Stored UNSCALED in g; kAccelScale_g is applied at the NN slot write.
    const gp_scalar milliGToG = static_cast<gp_scalar>(0.001);
    gp_scalar ax =  static_cast<gp_scalar>(state.autoc_state.accel[0]) * milliGToG;  // x fwd: no sign change
    gp_scalar ay = -static_cast<gp_scalar>(state.autoc_state.accel[1]) * milliGToG;  // y: NEGATE (FLU left → FRD right)
    gp_scalar az = -static_cast<gp_scalar>(state.autoc_state.accel[2]) * milliGToG;  // z: NEGATE (FLU up → FRD down)
    aircraftState.setSpecificForceG(gp_vec3(ax, ay, az));
  }
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
