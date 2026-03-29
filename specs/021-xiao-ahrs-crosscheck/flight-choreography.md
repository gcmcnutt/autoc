# Characterization Flight Choreography

**Date**: 2026-03-28
**Goal**: Maximum useful data from a single battery / single prop flight
**Mode**: MANUAL throughout (no autoc). ACRO segments if comfortable.
**Duration**: ~3-4 minutes usable (budget for climb + landing reserve)

## Pre-Flight Ground Check

1. Power on, let AHRS settle 30s
2. Hold aircraft level: confirm accSmooth[2] ≈ +1G (via configurator or OSD)
3. Tilt right wing down: confirm gyroADC[0] positive (if live view available)
4. Arm, verify blackbox recording (LED or OSD indicator)

## Flight Sequence (~3 min airborne)

### Phase 1: Climb + Trim (30s)
- Launch, climb to safe altitude (~60-80m AGL)
- Establish straight and level cruise, note trim
- **Data**: baseline attitude, speed, throttle for level flight

### Phase 2: Roll Characterization (45s)
Do each maneuver once, don't waste time repeating.

```
2a. CRUISE SPEED (~14 m/s):
    - Wings level → full left stick → hold 1 full roll → release → level out
    - Wings level → full right stick → hold 1 full roll → release → level out
    - Wings level → half left stick → hold 2s → release
    - Wings level → half right stick → hold 2s → release

2b. FAST (~18+ m/s, shallow dive to gain speed):
    - Full right roll → 1 roll → level
    - Half left roll → hold 2s → level

2c. SLOW (~10 m/s, nose up, power back):
    - Half right roll → hold 2s → level
```

### Phase 3: Pitch Characterization (30s)

```
3a. CRUISE SPEED:
    - Level → full pull (nose up) → hold 2s → release → recover
    - Level → full push (nose down) → hold 1s → release → recover
    - Level → half pull → hold 2s → release

3b. FAST:
    - Full pull → hold 2s → release (watch speed, will climb fast)

3c. SLOW:
    - Half pull → hold 2s → release (near stall, be ready to recover)
```

### Phase 4: Throttle Steps (20s)

```
4a. Level cruise → chop throttle to idle → hold 3s → note descent rate → full power
4b. Level cruise → full power → hold 3s → note speed increase → back to cruise
```

### Phase 5: Combined / Turning (30s)

```
5a. 45° bank left turn, steady, hold 5s (coordinated turn data)
5b. 45° bank right turn, steady, hold 5s
5c. Steep turn: 60°+ bank left, hold 3s (high load factor)
5d. Roll reversal: 45° left → immediate 45° right (dynamic response)
```

### Phase 6: ACRO Mode (if comfortable, 30s)

Switch TX to ACRO mode. Same aircraft, INAV PID now active.

```
6a. Level flight in ACRO — note any trim difference from MANUAL
6b. Half stick roll left → hold 2s → release → does it level itself? (No — ACRO doesn't self-level)
6c. Full roll → 1 rotation → center stick → note how fast rate damps
6d. Pitch pull → hold 2s → release → note pitch rate damping
```

This tells us how the INAV rate PID behaves — the response the NN will see.

### Phase 7: Autoc Spans (if time/altitude permits, 20s)

Switch back to MANUAL, enable autoc (CH5 or however armed).

```
7a. Engage autoc at cruise, wings level → let it run 5-10s → disengage
7b. If altitude permits, one more engage → disengage
```

Same NN as Mar 27 — will tumble, but now we have clean blackbox data
(1/32 rate, GYRO_RAW + ACC) to correlate with xiao log.

### Phase 8: Landing (reserve)

- RTH or manual approach
- Aim for 20% battery reserve

## Key Data Products

| Maneuver | What we learn |
|----------|--------------|
| 2a-c (rolls) | Roll rate vs command at 3 speeds → validates sim Cl_da/Cl_p |
| 3a-c (pitches) | Pitch rate vs command → validates sim Cm_de |
| 4a-b (throttle) | Speed response to power → validates sim CD_prof/thrust |
| 5a-d (turns) | Coupled dynamics, load factor, turn rate |
| 6a-d (ACRO) | INAV rate PID response → what CRRCSim must match |
| 7a-b (autoc) | Clean Z data + AHRS cross-check (if xiao AHRS implemented) |

## Analysis Scripts

After flight, decode blackbox and run:
```bash
# Response curves (existing)
python3 specs/019-improved-crrcsim/flight_response.py --axis all --csv <file>

# New: gyro rate analysis
python3 specs/021-xiao-ahrs-crosscheck/characterize_response.py --csv <file>
```

## Notes

- Don't spend time setting up perfect entry for each maneuver — just go
- Each maneuver is ~3-5s, 30s of straight-and-level between phases is wasted time
- Call out maneuvers verbally for the video record if possible
- If battery gets low, skip Phase 6/7 and land — Phases 2-5 are highest priority
