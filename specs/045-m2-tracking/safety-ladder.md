# 045 — Safety ladder (FR-015a, FR-015b; T060)

The safety changes, after the re-fly, one per step. Sources: spec § Clarifications 2026-11-12, [tasks.md](tasks.md) Phase 10,
[research.md](research.md) R16–R18, [fault-timelines.md](fault-timelines.md). Nothing on this page has flown yet.

## The rule

- **One structural change per step.** Write its tests first, implement it, run the build gate.
- **Bench before and after**: the step's B / H items from the T056 "before" record, repeated ×3 on the new build or setting, as the
  after record.
- **A happy-path g800 flight with that step alone.**
- **A short note**, here and in MANIFEST § Safety ladder: the happy path is unchanged, and the fault behaviour is as designed.
- **Only then the next step.** Fault-path-only steps come first; steps that change every disengage, the MSP path or INAV come
  later.
- Pairing two fault-path-only steps on one flight is the operator's call, recorded below with its reason.
- Before step 1 (none of it changes behaviour): the T056 before record, fault-timelines.md, the ANGLE trace (research R17, T057)
  and [pilot-brief.md](pilot-brief.md). `pilot-brief.md` is updated **before** the flight of any step the pilot will feel.

## Proposed order (proposed — operator approves or reorders)

| step | change | tasks | why here |
|---|---|---|---|
| **L1** | mode bits by permanent id (B-3), so the stop watches the real FAILSAFE bit (gives B-5 with no state-message change) | T080–T082 | fault-path only; fixes a confirmed bug (bit 19 is NAV ALTHOLD, bench A3); every failsafe-aware step, LAND included, needs it |
| **L2** | no resume after an in-flight xiao reboot (B-2a) | T083–T085 | fault-path only; removes the surprise re-engage (R18) that the pilot now covers by hand with RC9 |
| **L3** | bounded run on bad output (B-4, first half) | T086–T088 | fault-path only; closes the `flight_path.empty()` timeout skip and stops on NaN / out-of-range output |
| **L4** | neutral, motor-off frames for ≥ 3 ticks on every stop, then quiet (B-1) | T089–T090 | throttle to 0 first for every stop the xiao chooses; it changes every disengage, so it comes after the fault-only steps, with the pilot briefed first |
| **L5** | the companion's MSP traffic (R-3): measure first, guard only a path that can exceed a cap | T091–T092 | the measurement may close it with no code change; a guard is its own step per path |
| **L6** | control-loop watchdog (B-4, second half) | T093–T094 | a wedge resets in ~1 s instead of 30 s; needs L2 so the reset cannot re-engage |
| **L7** | INAV failsafe settings, one setting per step, once the debate settles | T095 | configuration, but it changes the loss-of-signal path; waits for the debate below, and LAND waits for L1 |
| **L8** | fork: no non-MSP bytes evaluated on the companion's UART (R-7) | T096 | first fork change (a new build surface); removes the bootloader / CLI exposure while disarmed in the air |
| **L9** | fork: flight-controller watchdog (R-4) | T097–T098 | the worst case (a hang with the motor running), but the largest change: IWDG, blocking-time measurement, three targets |

Then T099 closes the ladder: fault-timelines.md gets the after column for every step flown, pilot-brief.md matches the firmware and
configuration of record, and MANIFEST § Safety ladder lists each step with its flight.

## Per-step record

One row per step, filled as it goes. The before record is T056 for every step.

| step | change (commit / build / setting) | tests | bench after record (items ×3, file) | flight (date, log, genome) | happy path unchanged? | fault behaviour as designed? | note |
|---|---|---|---|---|---|---|---|
| L1 | | | B3, H1 | | | | |
| L2 | | | H3 | | | | |
| L3 | | | (bad-output hook) | | | | |
| L4 | | | B4, H7 | | | | |
| L5 | | | B10 | | | | |
| L6 | | | forced stall | | | | |
| L7 | | | H1, H2 (per setting) | | | | |
| L8 | | | stray `R` disarmed; USB CLI | | | | |
| L9 | | | H4 (forced hang) | | | | |

**Pairings** (operator's call, with reason): none.

## The failsafe-configuration debate: where it stands (2026-11-12)

**Decided**: DROP stays the loss-of-signal procedure for the re-fly, and until a tested alternative is adopted (FR-015b; operator:
"generally the safest"). `failsafe_delay` is bench-tested at 5 first (10 / 5 / 3 in RAM, T056 B7). Re-enabling after a failsafe
must take fewer pilot actions than today's (throttle low, ARM off, ARM on, throttle up; 5 with RC9 off).

**Open**: DROP vs LAND (R-1); RTH-with-climb later (trees around the park); the `failsafe_delay` value; `failsafe_off_delay`;
the LAND angles (`failsafe_fw_roll_angle` / `yaw_rate`: circle vs wings-level); the transmitter-side re-arm (R-6).

**Inputs**: T056 (timings and action counts, ×3; not yet run) and fault-timelines.md. What the source says so far:

| option | throttle | surfaces after the fault | control back on link return | pilot actions | risks / dependencies |
|---|---|---|---|---|---|
| **DROP** (today) | off at 1.2 s, by an in-air disarm | **no ANGLE**: the companion's last rate command, held until the link returns (T057). With L4: zero rate (holds attitude) | steering at the first good frame (unpowered); power after ≥ ~3.7 s and a re-arm | 5 | bootloader / CLI exposure while disarmed (until L5 / L8); the launch-mode trap on re-arm without a GPS heading |
| **LAND** (R-1) | off at 1.2 s, **still armed** | forced ANGLE: half roll from inverted, full-authority pull from vertical, then a −20° / +10° glide with no estimate (T057); levels to the AHRS's level | on stick motion, 0.7 s after the link is up; no re-arm | 1 (with L1), 2 without | **needs L1**, or control returns to a still-sending companion; `failsafe_off_delay` 200 disarms after 20 s and then needs 30 s of good link, and 0 disables the timed disarm (source; bench to confirm); a circling glide over the park |
| `failsafe_delay` 5 (R-2) | stage 1 → 0.7 s | — | — | — | the companion-held window 1.4 s → 0.9 s; more in-air disarms on 0.7–1.2 s glitches under DROP, cheap under LAND |

**Order dependencies**: LAND only after L1. L4 changes what DROP leaves on the surfaces (zero rate instead of the last rate
command), so the DROP-vs-LAND comparison should be re-read after L4. R-6 changes a re-arm path: bench-timed with its action count
in the same step before it flies (T095).
