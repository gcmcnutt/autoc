#include <MSP.h>

/*
 * track and prepare state of device
 *
 * a state is a set of values from Inav -- status, rc, attitude, gps, etc.
 * the state is updated periodically by querying Inav via MSP
 */
class State
{
public:
  void resetState()
  {
    autoc_state_valid = false;
    altitude_valid = false;
    waypoint_valid = false;
    inavSampleTimeMsec = 0;
  }

  void setAsOfMsec(unsigned long asOfMsec)
  {
    this->asOfMsec = asOfMsec;
  }

  // Convenience accessors for consolidated autoc_state fields
  bool isArmed() const { return autoc_state_valid && autoc_state.flightModeFlags & (1UL << MSP_MODE_ARM); }
  bool isFailsafe() const { return autoc_state_valid && autoc_state.flightModeFlags & (1UL << MSP_MODE_FAILSAFE); }
  uint16_t rcChannel(int ch) const { return ch == 8 ? autoc_state.rcChannel8 : autoc_state.rcChannel9; }

  unsigned long asOfMsec;
  unsigned long inavSampleTimeMsec; // INAV's timestamp in msec (from timestamp_us / 1000)
  bool autoc_state_valid;
  bool altitude_valid;
  bool waypoint_valid;

  // alignas(4): msp_autoc_state_t is packed (align 1), so without this the
  // member lands at offset 11 and q[]/pos[]/vel[] are misaligned. Callers
  // decay those arrays to plain pointers (neuQuaternionToNed et al.), and
  // -Ofast emits VLDR there, which HardFaults on unaligned addresses. All
  // word-sized fields inside the struct sit at 4-byte offsets, so aligning
  // the base is sufficient — keep it that way if fields change.
  alignas(4) msp_autoc_state_t autoc_state;  // consolidated nav + status + rc
  msp_altitude_t altitude;
  msp_waypoint_t waypoint;

  int autoc_countdown;
  bool autoc_enabled;
  msp_set_raw_rc_t command_buffer;

private:
};

extern State state;
