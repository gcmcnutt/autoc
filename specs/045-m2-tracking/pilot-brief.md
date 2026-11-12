# Pilot brief: today's firmware and configuration (045 re-fly)

Applies to: article 1, INAV fork `38ff0d29` with the 045 change set, the xiao flight firmware with its safety behaviour unchanged
since 11-08, `failsafe_procedure` DROP, `failsafe_delay` 10. Timings come from the source; the bench-measured values arrive with
T056. Details: [fault-timelines.md](fault-timelines.md).

1. **Your escape is RC9 (override) off.** With the TX link up, INAV stops using the xiao's channels within about one receiver
   frame, whatever the xiao is doing (verified in INAV's source, research R8). Your sticks and your RC6 mode switch are live again
   at once. *(If the link itself is down, RC9 is frozen too.)*

2. **RC6 MANUAL does nothing while autoc is engaged.** The xiao holds RC6. Flip RC9 off first; then RC6 works.

3. **RC8 RTH / POSHOLD works only with a valid position estimate.** With a bad estimate INAV refuses it without telling you, or
   goes to an emergency landing. Do not count on it as an escape.

4. **After any stop, INAV keeps flying the xiao's last command, throttle included, for about 1.4 s**, then hands you the sticks.
   This is every stop except your own RC9 flip and a disarm: path end, timeout, an xiao fault or reboot. Be ready at the end of
   those 1.4 s, with your throttle stick where you want it.

5. **After any xiao glitch or reboot, flip RC9 off and leave it off.** If RC9 stays high, autoc can re-engage by itself about 9 s
   later (about 40 s after a hang), from wherever the aircraft is, in the middle of whatever you are doing. This is the whole
   mitigation for the re-fly: the xiao firmware is left alone.

6. **The DROP re-arm sequence.** If the link is lost for 1.2 s or more, INAV disarms in the air and the motor stops. Until the link
   returns, the surfaces hold the xiao's last command, so the aircraft may keep rotating. From the first good frame after the link
   returns, your sticks steer, motor still off. Then:
   1. RC9 off.
   2. Throttle low.
   3. ARM switch off.
   4. Wait: at least ~4 s after the link came back (INAV needs 0.7 s + 3 s of good link, with ARM off).
   5. Roll and pitch sticks centred, ARM switch on.
   6. Throttle up.

   If after the re-arm the roll and pitch sticks do nothing, and raising the throttle pitches the nose up with no motor, INAV has
   gone into launch mode (it does this only when the GPS has no heading). Disarm and glide on the sticks.
