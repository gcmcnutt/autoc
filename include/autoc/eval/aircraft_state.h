/* aircraft state and path types — shared between desktop and embedded */
#pragma once

#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <stdexcept>
#include <string>
#include "autoc/types.h"
#include "autoc/nn/topology.h"
#include "autoc/nn/nn_input_computation.h"

#ifndef ARDUINO
#include <cereal/cereal.hpp>
#endif
#include <vector>
// Portable math macros — work on desktop (std::) and embedded (Arduino)
#define CLAMP_DEF(v, lo, hi) ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))
#define ATAN2_DEF(y, x) std::atan2f(y, x)
#define ABS_DEF(v) std::fabs(v)
#define SQRT_DEF(v) std::sqrt(v)
#define MIN_DEF(a, b) ((a) < (b) ? (a) : (b))
#define MAX_DEF(a, b) ((a) > (b) ? (a) : (b))

#define SIM_MAX_ROLL_RATE_RADSEC (static_cast<gp_scalar>(M_PI))
#define SIM_MAX_PITCH_RATE_RADSEC (static_cast<gp_scalar>(M_PI))

#define SIM_INITIAL_VELOCITY static_cast<gp_scalar>(20.0f)
#define SIM_THROTTLE_SCALE static_cast<gp_scalar>(10.0f)
#define SIM_CRASH_PENALTY static_cast<gp_scalar>(300.0f)  // Large penalty to ensure path completion
#define SIM_INITIAL_ALTITUDE static_cast<gp_scalar>(-25.0f)
#define SIM_INITIAL_THROTTLE static_cast<gp_scalar>(0.0f)
#define SIM_INITIAL_LOCATION_DITHER static_cast<gp_scalar>(30.0f)
#define SIM_PATH_BOUNDS static_cast<gp_scalar>(40.0f)
#define SIM_PATH_RADIUS_LIMIT static_cast<gp_scalar>(70.0f)
#define SIM_MIN_ELEVATION static_cast<gp_scalar>(-7.0f)
#define SIM_MAX_ELEVATION static_cast<gp_scalar>(-120.0f)

#define SIM_TOTAL_TIME_MSEC (100 * 1000)
#define SIM_TIME_STEP_MSEC (100)
#define SIM_MAX_INTERVAL_MSEC (SIM_TIME_STEP_MSEC * 5)

/*
 * some generic path information about routes
 */
class Path {
public:
  gp_vec3 start;
  gp_vec3 orientation;
  gp_scalar distanceFromStart;
  gp_scalar radiansFromStart;

  // Default constructor for backward compatibility
  Path() : start(gp_vec3::Zero()), orientation(gp_vec3::UnitX()),
           distanceFromStart(0.0f), radiansFromStart(0.0f) {}

  // Constructor to ensure all fields are properly initialized
  Path(const gp_vec3& start_pos, const gp_vec3& orient,
       gp_scalar distance, gp_scalar radians)
    : start(start_pos), orientation(orient), distanceFromStart(distance),
      radiansFromStart(radians) {}

  // Generic constructor to cast external Eigen scalars into gp_scalar
  template <typename Scalar>
  Path(const Eigen::Matrix<Scalar, 3, 1>& start_pos, const Eigen::Matrix<Scalar, 3, 1>& orient,
       Scalar distance, Scalar radians)
    : start(start_pos.template cast<gp_scalar>()), orientation(orient.template cast<gp_scalar>()),
      distanceFromStart(static_cast<gp_scalar>(distance)),
      radiansFromStart(static_cast<gp_scalar>(radians)) {}

  void sanitize() {
    auto sanitizeScalar = [](gp_scalar value, gp_scalar fallback = 0.0f) {
      return std::isfinite(value) ? value : fallback;
    };
    for (int i = 0; i < 3; ++i) {
      start[i] = sanitizeScalar(start[i]);
      gp_scalar defaultOrient = (i == 0) ? 1.0f : 0.0f;
      orientation[i] = sanitizeScalar(orientation[i], defaultOrient);
    }
    distanceFromStart = sanitizeScalar(distanceFromStart);
    radiansFromStart = sanitizeScalar(radiansFromStart);
  }

  void dump(std::ostream& os) {
    char buf[256];
    snprintf(buf, sizeof(buf), "Path: (%f, %f, %f), Odometer: %f, Turnmeter: %f",
      start[0], start[1], start[2], distanceFromStart, radiansFromStart);
    os << buf;
  }

#ifndef ARDUINO
  template<class Archive>
  void serialize(Archive& ar, const std::uint32_t /*version*/) {
    ar(start, orientation, distanceFromStart, radiansFromStart);
  }
#endif
};
#ifndef ARDUINO
CEREAL_CLASS_VERSION(Path, 1)
#endif

// Maximum offset steps for path interpolation (±1 second at 100ms/step)
constexpr int MAX_OFFSET_STEPS = 10;

// Portable path provider interface for unified GP evaluation
// Abstracts path access for both vector and single-path environments
class PathProvider {
public:
    virtual ~PathProvider() = default;
    virtual const Path& getPath(int index) const = 0;
    virtual int getCurrentIndex() const = 0;
    virtual int getPathSize() const = 0;

    // Get the distance of the last waypoint in the path
    gp_scalar getMaxDistance() const {
        int size = getPathSize();
        if (size == 0) return 0;
        return getPath(size - 1).distanceFromStart;
    }
};

// Vector-based path provider
class VectorPathProvider : public PathProvider {
private:
    const std::vector<Path>& paths;
    int currentIndex;
    
public:
    VectorPathProvider(const std::vector<Path>& p, int current = 0) 
        : paths(p), currentIndex(current) {}
    
    const Path& getPath(int index) const override {
        if (index < 0) index = 0;
        if (index >= (int)paths.size()) index = (int)paths.size() - 1;
        return paths[index];
    }
    
    int getCurrentIndex() const override { return currentIndex; }
    int getPathSize() const override { return (int)paths.size(); }
    void setCurrentIndex(int index) { currentIndex = index; }
};

// Single path provider
class SinglePathProvider : public PathProvider {
private:
    const Path& singlePath;
    int currentIndex;
    
public:
    SinglePathProvider(const Path& p, int current = 0) 
        : singlePath(p), currentIndex(current) {}
    
    const Path& getPath(int index) const override {
        // For single path, always return the same path regardless of index
        return singlePath;
    }
    
    int getCurrentIndex() const override { return currentIndex; }
    int getPathSize() const override { return 1; }
    void setCurrentIndex(int index) { currentIndex = index; }
};

// Forward declaration for AircraftState
struct AircraftState;

// REMOVED: getPathIndex() - replaced by getInterpolatedTargetPosition() in gp_evaluator_portable.cc
// The old discrete index lookup caused jitter sensitivity; interpolation provides smooth sensor values.

/*
* portable aircraft state
 */
struct AircraftState {
  public:

    AircraftState()
      : thisPathIndex(0),
        dRelVel(0.0f),
        velocity(gp_vec3::Zero()),
        aircraft_orientation(gp_quat::Identity()),
        position(gp_vec3::Zero()),
        simTimeMsec(0),
        pitchCommand(0.0f),
        rollCommand(0.0f),
        throttleCommand(0.0f),
        wind_velocity(gp_vec3::Zero()),
        rabbitPosition(gp_vec3::Zero()),
        rabbitOdometer_(0.0f),
        rabbitSpeed_(0.0f) {}
    AircraftState(int thisPathIndex, gp_scalar relVel, gp_vec3 vel, gp_quat orientation,
      gp_vec3 pos, gp_scalar pc, gp_scalar rc, gp_scalar tc,
      unsigned long int timeMsec)
      : thisPathIndex(thisPathIndex), dRelVel(relVel), velocity(vel), aircraft_orientation(orientation), position(pos), simTimeMsec(timeMsec),
      pitchCommand(pc), rollCommand(rc), throttleCommand(tc),
      wind_velocity(gp_vec3::Zero()),
      rabbitPosition(gp_vec3::Zero()) {
    }

    // Casting ctor for external Eigen scalar types while migrating callers to float
    template <typename Scalar>
    AircraftState(int thisPathIndex, Scalar relVel, const Eigen::Matrix<Scalar, 3, 1>& vel, const Eigen::Quaternion<Scalar>& orientation,
      const Eigen::Matrix<Scalar, 3, 1>& pos, Scalar pc, Scalar rc, Scalar tc,
      unsigned long int timeMsec)
      : thisPathIndex(thisPathIndex),
        dRelVel(static_cast<gp_scalar>(relVel)),
        velocity(vel.template cast<gp_scalar>()),
        aircraft_orientation(orientation.template cast<gp_scalar>()),
        position(pos.template cast<gp_scalar>()),
        simTimeMsec(timeMsec),
        pitchCommand(static_cast<gp_scalar>(pc)),
        rollCommand(static_cast<gp_scalar>(rc)),
        throttleCommand(static_cast<gp_scalar>(tc)),
        wind_velocity(gp_vec3::Zero()),
        rabbitPosition(gp_vec3::Zero()) {}

    // generate setters and getters
    int getThisPathIndex() const { return thisPathIndex; }
    void setThisPathIndex(int index) { thisPathIndex = index; }

    gp_scalar getRelVel() const { return dRelVel; }
    void setRelVel(gp_scalar relVel) { dRelVel = relVel; }
    
    gp_vec3 getVelocity() const { return velocity; }
    void setVelocity(const gp_vec3& vel) { velocity = vel; }

    gp_quat getOrientation() const { return aircraft_orientation; }
    void setOrientation(gp_quat orientation) { aircraft_orientation = orientation; }

    gp_vec3 getPosition() const { return position; }
    void setPosition(gp_vec3 pos) { position = pos; }

    // Position is always in virtual coordinates (origin at test start).
    // Producers (CRRCSim, minisim, xiao) convert raw→virtual at boundary.
    // See docs/COORDINATE_CONVENTIONS.md "Virtual Frame" section.

    unsigned long int getSimTimeMsec() const { return simTimeMsec; }
    void setSimTimeMsec(unsigned long int timeMsec) { simTimeMsec = timeMsec; }

    gp_scalar getPitchCommand() const { return pitchCommand; }
    gp_scalar setPitchCommand(gp_scalar pitch) { return (pitchCommand = CLAMP_DEF(pitch, -1.0f, 1.0f)); }
    gp_scalar getRollCommand() const { return rollCommand; }
    gp_scalar setRollCommand(gp_scalar roll) { return (rollCommand = CLAMP_DEF(roll, -1.0f, 1.0f)); }
    gp_scalar getThrottleCommand() const { return throttleCommand; }
    gp_scalar setThrottleCommand(gp_scalar throttle) { return (throttleCommand = CLAMP_DEF(throttle, -1.0f, 1.0f)); }

    // Filtered commands (post-RC-smoothing, what actually reaches FDM surfaces)
    gp_scalar getFilteredPitchCommand() const { return filteredPitchCommand_; }
    void setFilteredPitchCommand(gp_scalar v) { filteredPitchCommand_ = v; }
    gp_scalar getFilteredRollCommand() const { return filteredRollCommand_; }
    void setFilteredRollCommand(gp_scalar v) { filteredRollCommand_ = v; }
    gp_scalar getFilteredThrottleCommand() const { return filteredThrottleCommand_; }
    void setFilteredThrottleCommand(gp_scalar v) { filteredThrottleCommand_ = v; }

    gp_vec3 getWindVelocity() const { return wind_velocity; }
    void setWindVelocity(const gp_vec3& wind) { wind_velocity = wind; }

    gp_vec3 getRabbitPosition() const { return rabbitPosition; }
    void setRabbitPosition(const gp_vec3& pos) { rabbitPosition = pos; }

    gp_scalar getRabbitOdometer() const { return rabbitOdometer_; }
    void setRabbitOdometer(gp_scalar odo) { rabbitOdometer_ = odo; }

    gp_scalar getRabbitSpeed() const { return rabbitSpeed_; }
    void setRabbitSpeed(gp_scalar speed) { rabbitSpeed_ = speed; }

    // Body-frame angular rates (p, q, r) in rad/s — standard aerospace RHR convention
    // Roll (p): positive = right wing down
    // Pitch (q): positive = nose up
    // Yaw (r): positive = nose right
    // NOTE: INAV gyro has inverted pitch/yaw signs — negate at consumer boundary.
    // CRRCSim FDM provides these in standard convention (no conversion needed).
    gp_vec3 getGyroRates() const { return gyroRates_; }
    void setGyroRates(const gp_vec3& rates) { gyroRates_ = rates; }

    // NN I/O capture — record what the NN actually saw and produced
    void setNNData(const NNInputs& inputs, const float* outputs, int numOutputs) {
      nnInputs_ = inputs;
      for (int i = 0; i < NN_OUTPUT_COUNT && i < numOutputs; i++) nnOutputs_[i] = outputs[i];
      hasNNData_ = true;
    }
    bool hasNNData() const { return hasNNData_; }
    const NNInputs& getNNInputs() const { return nnInputs_; }
    const float* getNNOutputs() const { return nnOutputs_; }

    // =========================================================================
    // Temporal history for GP nodes - see specs/TEMPORAL_STATE.md
    // =========================================================================
    static constexpr int HISTORY_SIZE = 10;  // 1 sec at 10Hz

    // Record current target direction and distance to history (call before NN eval each tick)
    void recordErrorHistory(const gp_vec3& targetDir, gp_scalar distance, unsigned long timeMs) {
      targetDirHistory_[historyIndex_] = targetDir;
      distHistory_[historyIndex_] = distance;
      timeHistory_[historyIndex_] = timeMs;
      historyIndex_ = (historyIndex_ + 1) % HISTORY_SIZE;
      if (historyCount_ < HISTORY_SIZE) historyCount_++;
    }

    // Get historical target direction unit vector (n=0 is most recent, n=1 is one tick ago, etc.)
    // Returns zero vector if history not available. Uses CLAMP_DEF for portability.
    gp_vec3 getHistoricalTargetDir(int n) const {
      if (historyCount_ == 0) return gp_vec3::Zero();
      n = CLAMP_DEF(n, 0, historyCount_ - 1);
      int idx = (historyIndex_ - 1 - n + HISTORY_SIZE) % HISTORY_SIZE;
      return targetDirHistory_[idx];
    }

    gp_scalar getHistoricalDist(int n) const {
      if (historyCount_ == 0) return static_cast<gp_scalar>(0.0f);
      n = CLAMP_DEF(n, 0, historyCount_ - 1);
      int idx = (historyIndex_ - 1 - n + HISTORY_SIZE) % HISTORY_SIZE;
      return distHistory_[idx];
    }

    unsigned long getHistoricalTime(int n) const {
      if (historyCount_ == 0) return 0;
      n = CLAMP_DEF(n, 0, historyCount_ - 1);
      int idx = (historyIndex_ - 1 - n + HISTORY_SIZE) % HISTORY_SIZE;
      return timeHistory_[idx];
    }

    int getHistoryCount() const { return historyCount_; }

    void clearHistory() {
      historyIndex_ = 0;
      historyCount_ = 0;
    }

    // Pre-fill all history slots with the current geometry so the NN starts
    // with consistent direction cosines and closing_rate = 0.  Call at
    // engage/scenario start AFTER position, orientation, and path are known.
    // targetPos: world-frame position of the rabbit/path start
    // pathTangent: unit vector along path at targetPos (singularity fallback)
    void resetHistory(const gp_vec3& targetPos, const gp_vec3& pathTangent) {
      clearHistory();
      gp_vec3 craftToTarget = targetPos - position;
      gp_vec3 target_local = aircraft_orientation.inverse() * craftToTarget;
      float distance = static_cast<float>(target_local.norm());

      gp_vec3 tangent_body = aircraft_orientation.inverse() * pathTangent;
      float tn = static_cast<float>(tangent_body.norm());
      if (tn > 1e-6f) tangent_body = tangent_body / tn;
      else tangent_body = gp_vec3::UnitX();

      gp_vec3 dir = computeTargetDir(target_local, distance, tangent_body);

      for (int h = 0; h < HISTORY_SIZE; h++) {
        recordErrorHistory(dir, distance, 0);
      }
    }

    void minisimAdvanceState(gp_scalar dt) {
      gp_scalar dtSec = dt / 1000.0f;

      // get current roll state, compute left/right force (positive roll is right)
      gp_scalar delta_roll = std::remainder(rollCommand * dtSec * SIM_MAX_ROLL_RATE_RADSEC, static_cast<gp_scalar>(M_PI));

      // get current pitch state, compute up/down force (positive pitch is up)
      gp_scalar delta_pitch = std::remainder(pitchCommand * dtSec * SIM_MAX_PITCH_RATE_RADSEC, static_cast<gp_scalar>(M_PI));

      // adjust velocity as a function of throttle (-1:1)
      // throttle = -1.0 → 0.5x base velocity (10 m/s)
      // throttle =  0.0 → 1.0x base velocity (20 m/s)
      // throttle = +1.0 → 1.5x base velocity (30 m/s)
      dRelVel = SIM_INITIAL_VELOCITY * (1.0f + throttleCommand * 0.5f);

      // Save pre-rotation orientation for gyro rate computation
      gp_quat q_prev = aircraft_orientation;

      // Convert pitch and roll updates to quaternions (in the body frame)
      gp_quat delta_roll_quat(Eigen::AngleAxis<gp_scalar>(delta_roll, gp_vec3::UnitX()));
      gp_quat delta_pitch_quat(Eigen::AngleAxis<gp_scalar>(delta_pitch, gp_vec3::UnitY()));

      // Apply the roll and pitch adjustments to the aircraft's orientation
      aircraft_orientation = aircraft_orientation * delta_roll_quat;
      aircraft_orientation = aircraft_orientation * delta_pitch_quat;

      // Normalize the resulting quaternion
      aircraft_orientation.normalize();

      // Compute actual body angular rates from quaternion delta (rad/s)
      // delta_q = q_prev.inverse() * q_new = rotation in body frame over dt
      // angular velocity = 2 * vec(delta_q) / dt (small angle approx)
      if (dtSec > 0.0f) {
        gp_quat dq = q_prev.inverse() * aircraft_orientation;
        if (dq.w() < 0) { dq.coeffs() = -dq.coeffs(); }  // ensure shortest path
        gyroRates_ = gp_vec3(
            2.0f * dq.x() / dtSec,   // p (roll rate, rad/s)
            2.0f * dq.y() / dtSec,   // q (pitch rate, rad/s)
            2.0f * dq.z() / dtSec);  // r (yaw rate, rad/s)
      }

      // Define the initial velocity vector in the body frame
      gp_vec3 velocity_body(dRelVel * dtSec, 0.0f, 0.0f);

      // Rotate the velocity vector using the updated quaternion
      gp_vec3 velocity_world = aircraft_orientation * velocity_body;

      // Store the actual velocity vector (convert from distance per timestep to velocity)
      velocity = velocity_world / dtSec;

      // adjust position
      position += velocity_world;
    }

  private:
    int thisPathIndex;
    gp_scalar dRelVel;
    gp_vec3 velocity;  // Actual velocity vector (north, east, down)
    gp_quat aircraft_orientation;
    gp_vec3 position;
    unsigned long int simTimeMsec;
    gp_scalar pitchCommand;
    gp_scalar rollCommand;
    gp_scalar throttleCommand;

    // Filtered commands (post-RC smoothing pt3 filter, what FDM surfaces see)
    gp_scalar filteredPitchCommand_ = 0.0f;
    gp_scalar filteredRollCommand_ = 0.0f;
    gp_scalar filteredThrottleCommand_ = 0.0f;

    // Wind diagnostic fields (for debugging non-determinism)
    gp_vec3 wind_velocity;  // Wind vector (north, east, down) from calculate_wind()

    // Interpolated rabbit position for renderer playback
    gp_vec3 rabbitPosition;  // Exact target position NN was tracking this tick

    // Odometer-based rabbit tracking
    gp_scalar rabbitOdometer_;  // Distance along path (meters)
    gp_scalar rabbitSpeed_;     // Current rabbit speed (m/s)

    // Body-frame angular rates (rad/s, standard aerospace RHR)
    gp_vec3 gyroRates_ = gp_vec3::Zero();

    // NN I/O capture — actual values presented to/produced by the neural net
    NNInputs nnInputs_ = {};                 // Normalized inputs as NN sees them
    float nnOutputs_[NN_OUTPUT_COUNT] = {0};  // Raw tanh outputs
    bool hasNNData_ = false;

    // Temporal history — target direction (unit vec in body frame) and distance
    gp_vec3 targetDirHistory_[HISTORY_SIZE];  // unit vectors in body frame
    gp_scalar distHistory_[HISTORY_SIZE] = {0};
    unsigned long timeHistory_[HISTORY_SIZE] = {0};
    int historyIndex_ = 0;   // Next write position (ring buffer)
    int historyCount_ = 0;   // Valid samples (0 to HISTORY_SIZE)

#ifndef ARDUINO
    friend class cereal::access;
    template<class Archive>
    void serialize(Archive& ar, const std::uint32_t version) {
      ar(thisPathIndex, dRelVel, velocity, aircraft_orientation, position,
         pitchCommand, rollCommand, throttleCommand, simTimeMsec, wind_velocity,
         rabbitPosition, hasNNData_);
      if (hasNNData_) {
        uint32_t inputCount = NN_INPUT_COUNT;
        uint32_t outputCount = NN_OUTPUT_COUNT;
        ar(inputCount, outputCount);
        if (inputCount != NN_INPUT_COUNT || outputCount != NN_OUTPUT_COUNT) {
          throw std::runtime_error(
            "AircraftState deserialization: NN topology mismatch — "
            "serialized inputs=" + std::to_string(inputCount) +
            " outputs=" + std::to_string(outputCount) +
            " but compiled with inputs=" + std::to_string(NN_INPUT_COUNT) +
            " outputs=" + std::to_string(NN_OUTPUT_COUNT) +
            ". Regenerate training data with current binary.");
        }
        float* rawInputs = reinterpret_cast<float*>(&nnInputs_);
        for (uint32_t i = 0; i < inputCount; i++)
          ar(rawInputs[i]);
        for (uint32_t i = 0; i < outputCount; i++)
          ar(nnOutputs_[i]);
      }
      ar(rabbitOdometer_, rabbitSpeed_);
      ar(filteredPitchCommand_, filteredRollCommand_, filteredThrottleCommand_);
    }
#endif
};
#ifndef ARDUINO
CEREAL_CLASS_VERSION(AircraftState, 1)
#endif

// Physics trace entry - captures complete FDM state at a single timestep
// Uses crrcsim native types for bit-exact copying (SCALAR = double)
// WARNING: Do NOT introduce type conversions - causes rounding errors!
struct PhysicsTraceEntry {
  // Simulation metadata
  uint32_t step;          // Timestep number
  double simTimeMsec;     // Simulation time in milliseconds (native: double)
  double dtSec;           // Timestep delta in seconds (SCALAR)

  // Worker identity (for multi-process determinism debugging)
  int32_t workerId;       // Worker ID (0-7 typically)
  int32_t workerPid;      // Worker process ID
  int32_t evalCounter;    // Evaluation counter on this worker

  // Position, velocity, acceleration (world frame) - all SCALAR (double)
  double pos[3];          // Position [x, y, z]
  double vel[3];          // Velocity [x, y, z]
  double acc[3];          // Acceleration [x, y, z]
  double accPast[3];      // Previous acceleration

  // Orientation and rotation - all SCALAR (double)
  double quat[4];         // Quaternion [x, y, z, w]
  double quatDotPast[4];  // Previous quaternion derivative
  double omegaBody[3];    // Angular velocity in body frame
  double omegaDotBody[3]; // Angular acceleration
  double rate[3];         // Rate (may alias omegaBody)
  double ratePast[3];     // Previous rate

  // Aerodynamic state - all SCALAR (double)
  double alpha;           // Angle of attack (rad)
  double beta;            // Sideslip angle (rad)
  double vRelWind;        // Airspeed magnitude
  double velRelGround[3]; // Velocity relative to ground
  double velRelAir[3];    // Velocity relative to air
  double vLocal[3];       // Local velocity
  double vLocalDot[3];    // Local velocity derivative

  // Aero calculation details - all SCALAR (double)
  double cosAlpha, sinAlpha, cosBeta;     // Trig values
  double CL, CD;                          // Lift and drag coefficients
  double CL_left, CL_cent, CL_right;     // Spanwise lift distribution
  double CL_wing;                         // Wing lift coefficient
  double Cl, Cm, Cn;                      // Moment coefficients
  double QS;                              // Dynamic pressure × ref area

  // Forces and moments (body frame) - all SCALAR (double)
  double forceBody[3];    // Total force
  double momentBody[3];   // Total moment

  // Environment - all SCALAR (double)
  double wind[3];         // Wind velocity
  double localAirmass[3]; // Local airmass velocity
  double gustBody[6];     // Gust in body frame: [v_V_gust_body (3), v_R_omega_gust_body (3)]
  double density;         // Air density
  double gravity;         // Gravity magnitude
  double geocentricLat, geocentricLon, geocentricR;  // Geocentric position

  // Control inputs - all SCALAR (double) from TSimInputs
  double pitchCommand;    // GP pitch command
  double rollCommand;     // GP roll command
  double throttleCommand; // GP throttle command
  double elevator;        // Sim elevator input
  double aileron;         // Sim aileron input
  double rudder;          // Sim rudder input
  double throttle;        // Sim throttle input

  // RNG state - exact integer types
  uint16_t rngState16;    // 16-bit RNG state
  uint32_t rngState32;    // 32-bit RNG state

  // Path tracking
  int32_t pathIndex;      // Current path index

  PhysicsTraceEntry() { memset(this, 0, sizeof(*this)); }

#ifndef ARDUINO
  template <class Archive>
  void serialize(Archive& ar) {
    ar(step, simTimeMsec, dtSec);
    ar(workerId, workerPid, evalCounter);
    ar(cereal::binary_data(pos, sizeof(pos)));
    ar(cereal::binary_data(vel, sizeof(vel)));
    ar(cereal::binary_data(acc, sizeof(acc)));
    ar(cereal::binary_data(accPast, sizeof(accPast)));
    ar(cereal::binary_data(quat, sizeof(quat)));
    ar(cereal::binary_data(quatDotPast, sizeof(quatDotPast)));
    ar(cereal::binary_data(omegaBody, sizeof(omegaBody)));
    ar(cereal::binary_data(omegaDotBody, sizeof(omegaDotBody)));
    ar(cereal::binary_data(rate, sizeof(rate)));
    ar(cereal::binary_data(ratePast, sizeof(ratePast)));
    ar(alpha, beta, vRelWind);
    ar(cereal::binary_data(velRelGround, sizeof(velRelGround)));
    ar(cereal::binary_data(velRelAir, sizeof(velRelAir)));
    ar(cereal::binary_data(vLocal, sizeof(vLocal)));
    ar(cereal::binary_data(vLocalDot, sizeof(vLocalDot)));
    ar(cosAlpha, sinAlpha, cosBeta);
    ar(CL, CD, CL_left, CL_cent, CL_right, CL_wing, Cl, Cm, Cn, QS);
    ar(cereal::binary_data(forceBody, sizeof(forceBody)));
    ar(cereal::binary_data(momentBody, sizeof(momentBody)));
    ar(cereal::binary_data(wind, sizeof(wind)));
    ar(cereal::binary_data(localAirmass, sizeof(localAirmass)));
    ar(cereal::binary_data(gustBody, sizeof(gustBody)));
    ar(density, gravity, geocentricLat, geocentricLon, geocentricR);
    ar(pitchCommand, rollCommand, throttleCommand);
    ar(elevator, aileron, rudder, throttle);
    ar(rngState16, rngState32, pathIndex);
  }
#endif
};
