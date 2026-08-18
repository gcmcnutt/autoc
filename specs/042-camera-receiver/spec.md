# 042 — camera receiver (seed, 2026-08-17; strategy discussion pending)

**Origin**: sensor-side follow-on to 031 (see `specs/031-beacon-camera/WRAP.md`, "what leaves 031"). 031
proved the toolchain — Pi 3A+ + OV9281 at 250 fps reads the Gold code and tracks a static beacon in real
time (`firmware/beacon-receiver/pi/beacon_track.py`). 042 turns that into a **receiver**: a robust,
responsive tracker for beacons slewing across the field, at the control loop's rate, on the platform we
have — and only then scales the hardware.

## Platform decision (carried in)
**Pi 3A+ at 250 fps / ~115–121 Hz beacons** until the tracker proves itself. Every algorithmic question
is per-frame and scales with the host; the **Pi 5 (dual CSI, ~3× CPU, likely 453+ fps with the patched
640×200 mode) is the scaling step, gated on "fancy tracker in the ballpark at high slew rate."**

## Candidate scope (to be shaped in the strategy discussion)
1. **Native tracker-bank** (C/NEON, same rpicam-raw pipe in / same JSON out as the Python prototype):
   sliding-window matched filter → **20 Hz fixes** each integrating a full word; **motion-compensated
   trackers** (per-instance velocity estimate, spatial window shifted frame-by-frame so a fast crosser
   stays fixed in the tracker's frame); N instances on candidate ROIs; full-field search only for
   acquisition (threaded off the capture path); CEP reflecting velocity uncertainty; DPLL per tracker.
2. **Sampling margin**: camera at its 280 ceiling (2.3 spc) or 'H' retuned ~105 Hz (2.4).
3. **Lens/filter/photometry**: ELP-L156 1.56 mm + 850 filter (in transit) measured on the ruled mat; the
   **sky-background frame through a real 850 filter — empirical #1, still open** — sets the daylight budget;
   defocus knob (PSF 2–4 px vs corrB + centroid); RI corner falloff.
4. **AGC as a controller**: exposure = saturation/well controller (near/sunny), gain = read-noise controller
   (far/dark) — measured split; exposure tracks range; sun-transit max-energy gate (031 journal 2a).
5. **Two-emitter / CDMA on the camera**: paired-pod orthogonality, near–far, port(A)/starboard(B) identity
   → the 041 L/R-by-code handoff. Needs cube B (031 A7).
6. **Interface to 041**: (x, y, CEP, q, code) per beacon at 20 Hz in the M2 grid frame (320×200,
   0.304°/px, f·θ); what the predictor needs on occlusion (blind-time budget table in camera-era-knobs).

## Explicitly NOT 042 (until proven needed)
Pi 5 / FPGA host; the flight recorder; the 120° single-lens question beyond measuring the ELP-L156 (the
birded pair stays the physics-honest route and is a 041 topology-fork input, not a 042 build).

## Inputs / references
- `specs/031-beacon-camera/camera-era-knobs.md` — the physics contract (photon/SNR, ¼-power code-rate law,
  blind-time table, filter verdict, sampling/defocus, measured f·θ 95×61)
- `specs/031-beacon-camera/handoff-041-camera-model.md` — the 041 camera-model handoff
- `firmware/beacon-receiver/` — README, HIGH-FPS-PLAN (per-frame ceiling + Pi 5 verdict), `pi/` tools
  (`beacon_track.py`, `beacon_display.py`, `live.sh`, `focus_view.py`, `fps_probe.py`, patched `ov9282.c`)
- `specs/031-beacon-camera/bench-journal.md` — living bench state (instruments, traps, field tests)
