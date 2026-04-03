# Flight Plan — 021 Series

## Flight 1: Autoc Test + Dynamics Data Collection (with streamer)

**Goal**: Test current NN (27 inputs, direct control, gyro rates) on real hardware.
Collect dynamics data for sim validation. Streamer on for continuity with prior flights.

**Prerequisites**: C+D complete (INAV MSP gyro extension, xiao firmware with new NN weights)

### Sequence
1. **Pre-flight**: arm, verify blackbox recording, verify xiao log started
2. **Manual cruise** (30s): establish level flight ~60% power, note trim
3. **Autoc span 1** (10-15s): engage autoc, observe behavior, disengage
4. **Recovery + climb** back to altitude
5. **Autoc span 2** (10-15s): engage again, different heading/altitude
6. **Recovery**
7. **Dynamics collection — MANUAL mode**:
   - Roll: full left stick 2s → release → full right stick 2s → release (at cruise)
   - Pitch: full pull 2s → release → full push 1s → release (at cruise)
   - Throttle: idle 3s → full 3s → cruise (note speed change)
8. **Optional autoc span 3** if altitude/battery permits
9. **Land with 20% reserve**

### Data Products
- Blackbox: GYRO_RAW + ACC + QUAT + SERVOS + RC_COMMAND + MOTORS @ 1/32
- Xiao log: NN inputs/outputs, gyro rates, quaternion, autoc spans
- Video if possible (for qualitative assessment)

### Analysis
- Autoc spans: does the NN track? Compare to Mar 27 (tumble) — expect improvement
- Dynamics: roll/pitch rate vs command at cruise → compare to sim
- Gyro rates: verify xiao receives correct values (polarity, magnitude)
- Clean Z data: verify blackbox decode < 10 failed frames

---

## Flight 2: Mechanical Tune — CG + Trim (NO streamer)

**Goal**: Establish proper CG and trim for the bare airframe. This is the baseline
configuration for all future flights and sim tuning.

**Prerequisites**: Flight 1 analysis complete. Remove streamer.

### CG Adjustment Protocol
Repeat until neutral:
1. Trim for level cruise at ~60% power
2. Chop to idle, hands off stick
3. Observe:
   - **Nose tucks and accelerates into dive**: CG too far forward → move battery aft
   - **Nose rises, mushes, stalls**: CG too far aft → move battery forward
   - **Nose drops gently, settles to stable glide**: CG correct
4. Note glide speed and descent angle

### Trim Validation
Once CG is set:
1. Level cruise at 60% — note stick position for level flight
2. Full throttle level — does it pitch up? (thrust line above CG)
3. Idle level — does it pitch down? (expected, gravity)
4. 45° bank turn — does it hold altitude at same throttle?
5. Note all trim positions for hb1_streamer.xml Cm_0 calibration

### Speed Envelope
1. Level cruise at various throttle: 40%, 60%, 80%, 100%
2. Note GPS speed at each power setting
3. Stall check: slow to minimum controllable speed, note behavior
4. Glide: idle, best glide speed, descent rate

### Data Products
- Blackbox: full flight
- Notes: CG position (mm from LE), trim values, speeds at each power setting
- Glide speed and angle → directly calibrates hb1_streamer.xml drag model

---

## Flight 3: ACRO PID Tune (NO streamer, CG set)

**Goal**: Tune INAV ACRO mode PID for the bare airframe. Establish rate limits
and response characteristics for future ACRO-mode training.

**Prerequisites**: Flight 2 CG/trim complete

### ACRO PID Tuning Protocol
Start in MANUAL mode, switch to ACRO for each test, back to MANUAL to recover.

1. **Roll rate check**:
   - ACRO mode, half stick right → observe rate → release (should stop)
   - Full stick right → observe rate → release
   - Note: does it hold rate? Does it stop cleanly? Oscillation?
   - If oscillating: reduce P gain
   - If sluggish: increase FF or P

2. **Pitch rate check**:
   - ACRO mode, half pull → observe pitch rate → release
   - Full pull → observe → release
   - Same tuning: oscillation → reduce P, sluggish → increase FF

3. **Rate limits**:
   - Full stick roll: what rate does it achieve? Compare to `roll_rate` config
   - Full stick pitch: same
   - Adjust `roll_rate`/`pitch_rate` config if max rate is too fast or too slow
   - Target: max rates that feel controllable and match sim expectations

4. **Expo check**:
   - With expo=0 (linear), is center stick too sensitive?
   - If so, set rc_expo=20-40 for comfortable manual ACRO flying
   - Note: for autoc, expo should be 0 (NN handles its own scaling)

5. **Trim in ACRO**:
   - Level flight in ACRO — does it hold attitude with stick centered?
   - If nose drops: adjust `fw_pitch_trim` in INAV CLI
   - This is the trim the ACRO rate PID needs in sim

### Data Products
- Blackbox: ACRO segments with known stick inputs
- INAV CLI config: final PID values, rate limits, trim
- Rate response data → directly sets CRRCSim ACRO PID gains for future training

---

## Between Flights 3 and 4: Sim Update + Retrain

**Goal**: Update hb1_streamer.xml and CRRCSim config from Flight 2-3 measured data.

1. **hb1_streamer.xml**: update Cm_0, CD_prof, CG_arm from Flight 2 CG/trim/glide data
2. **ACRO PID** (if using): set CRRCSim gains from Flight 3 measured rate response
3. **ACRO trim**: set pitch trim offset from Flight 3 ACRO trim measurement
4. **Retrain**: BIG production run with updated sim model
5. **Eval**: verify convergence, compare response curves to flight data

## Flight 4+: Autoc with Tuned Airframe + Retrained NN

**Goal**: Fly autoc with properly tuned CG, trim, known rate response,
and NN trained on a sim that matches the measured aircraft.

**Prerequisites**: Flights 2-3 complete, sim updated with measured parameters,
NN retrained with updated hb1_streamer.xml

---

## General Notes

- Each flight is one battery, one objective. Don't combine.
- Always start with 30s manual cruise to establish baseline before any tests.
- Keep 20% battery reserve for landing.
- Call out maneuvers for video/audio record.
- Download blackbox + xiao logs immediately after each flight.
- Analyze before next flight — each flight's data informs the next.
