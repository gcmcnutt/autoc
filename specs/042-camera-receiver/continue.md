# 042 — continue here (handoff, rewritten 2026-08-22 at a deliberate pause)

> **STATUS: PAUSED 2026-08-22.** Operator moved to **041** (INAV + xiao updates for an M1 flight test).
> 042 is at a clean stopping point: working tree clean, **10/10 tests green**, everything pushed. This file
> is the resume point — read it and the two results docs it names before touching anything.

## ⚠️ THE BENCH HAS BEEN RE-TASKED — do not assume the 042 rig is as you left it

Verified at the pause:

| | state at pause |
|---|---|
| **PSU** | **12.599 V / 0.163 A, output ON** — repurposed to a 3S-class rail for 041. **NOT the 4.2 V beacon point.** Do not assume; read it before energising anything. |
| **USB** | `/dev/serial/by-id` now holds the **XIAO nRF52840** and an **INAV STM32 VCP**. The StepFPGA (DAPLink) and the emitter (mEDBG) are **unplugged**. |
| **StepFPGA** | powered on but disconnected from USB; irrelevant to camera work either way (see below) |
| **beaconpi5** | **unreachable at the pause**, on both `192.168.1.197` (LAN) and `100.97.242.96` (tailscale). Probably powered down. The `/data` fixtures below are therefore **unverified as of the pause**. |

`host/ports.py` resolves the decoder and emitter by USB identity, so plugging them back in is enough — no
ttyACM numbers to chase.

## What this pause bought — the measurement campaign

Four things are now measured that were previously assumed, each with its own results doc:

1. **The coherence knee is 1–2 °/s** — confirmed three independent ways: theory (1.18), `pan1`'s
   coherence-ratio, and `pan2`'s lock-rate against fiducial truth.
   [stage1-pan1-analysis.md](results/stage1-pan1-analysis.md),
   [stage1-pan2-analysis.md](results/stage1-pan2-analysis.md)
2. **Deliberately gentle hand motion is 15–18 °/s median** — an order of magnitude past that knee. The gap
   is geometric, not tunable.
3. **Cold acquisition takes a median 2.95 s** at the real 53 µs operating point, static, n=5 —
   **7× over** the 0.40 s relock bar. [stage1-acquisition-latency.md](results/stage1-acquisition-latency.md)
4. **The acquisition failure mechanism**, which was NOT what the compute budget predicted:
   K (candidate count) is fine at 276–1292 everywhere; **the beacon simply ranks ~126th–230th** among
   detections and only the top 3 are seeded, so it enters the bank on 6–25 % of passes.
   [stage1-K-census.md](results/stage1-K-census.md)

Plus: **the StepFPGA contributes nothing measurable** to clutter (K 1277 vs 1292). Powering it down for
camera work is tidiness, not a fix — settled properly, because the obvious first-lock A/B could not
(Mann-Whitney U=9 against a critical 2).

## RESUME HERE — T076 first, then T080

**T080** (accumulated |Δ²| detector) is the big cheap win: offline on `pan2.bcnr` it moves the beacon from
rank 1842→1 still and 276→2 moving, tripling top-3 seeding, and **the moving case becomes as good as the
still case**. [stage1-detector-ranking.md](results/stage1-detector-ranking.md)

**It was attempted on 2026-08-22 and deliberately backed out.** It reached 16/17 golden-parity checks and
then regressed while chasing the last one; the tree is clean. Read this before retrying:

- **Do T076 first.** `hold_max_age_ms = 150` kills a track before the 258 ms needed to rebuild a 31-chip
  window, so tracks die spuriously mid-clip. That is what makes the golden test sensitive to
  *re-acquisition timing*, which is what the detector change perturbs. Fix the death and the two changes
  become separable.
- **Use a CONTINUOUS EWMA accumulator, not a burst.** `acc -= acc>>3; acc += |Δ²|`, fed every frame. The
  burst version (accumulate only while a pass is in flight) delays seeds ~10 frames — worst exactly when a
  track has just died. Costs `reduce4` per frame instead of 1-in-25, ~14 % of a core against ~36 ms of
  margin. `reduce4_neon` is bit-exact vs scalar and equivalence-tested, so using it is legitimate.
- **Three real bugs found during the attempt, latent for ANY multi-frame detector** (so T050 will meet them
  too): (a) the apply block is gated on `sched_completed()`, which fires exactly once — a detector that
  completes later than the cost model leaves seeds unapplied until the next pass, ~25 frames stale;
  (b) the whole acquisition block sits inside `if (!have_confirmed)`, so any in-flight state left set when a
  track confirms **wedges acquisition permanently**; (c) seeds snap to coarse-cell centres (4 M2 px here) —
  a 3×3 centroid on the accumulator gives sub-cell placement.

**Do NOT schedule a tuning phase.** Operator call (D1): the knee is 1–2 °/s and tuning moves it to maybe
2–3, against 15–18 °/s of hand motion. T076 is two correctness bugs, not tuning.

## Fixtures and tooling

- `beaconpi5:/data/pan2.bcnr` — 60 s continuous, pan→tilt→circular, fiducial exposure, **the regression
  fixture**: ground-truthed at 0.18°, with a known-bad baseline (3 % locked at 10–20 °/s) to beat.
  Also `static_bench.bcnr`, `static_fid.bcnr`, `static_bench_fpgaon.bcnr`. **Unverified at the pause** — the
  Pi was unreachable.
- `~/beacon-clips/pan1.bcnr` on the DGX (the burst-mode clip; md5 `10c2cf687d9e8040`).
- Tools: `oracle.py` (code-matched), `fiducial_truth.py` (per-frame pose from the ArUco), `candidate_census.py`
  (K), `check_placement.py`, `pi/preview.py` (live browser view), `targets/` (printable sheets + generator).
- **OpenCV is installed on the Pi at `~/cv`** — run truth extraction *there* (37 s) rather than moving
  4.2 GB over its WiFi link.

## Traps worth re-reading before resuming

1. **`--record-mode continuous`** or you get burst islands the container contract forbids correlating
   across — that is how `pan1` became unable to measure reacquire at all.
2. **`frames_dropped` and the timestamp dt spread are the only honest capture witnesses.** Seq counts
   *delivered* frames, so a drop leaves no gap (`sink_record.c`).
3. **Fiducial truth must be anchored** — `fiducial_truth.py`'s default reference ("brightest high-pass
   pixel") landed on an LED light bar and gave a 42 px single-axis bias. Validate against a known-good
   stretch before trusting truth.
4. **Every rate is rated at 288 fps / 120 Hz** — spec §2.5.1. A 480-fps-designed test must run ×0.600 here.
5. **AGC is a confound** (operator, 2026-08-22): validate detector changes on **replay**, where the recorded
   exposure is fixed, before trusting a live A/B.
6. Pull clips over the **LAN** IP, not tailscale (relayed via SFO — 3.9 s vs 0.019 s for a page). Re-check
   the Pi's LAN address on resume; `192.168.1.197` stopped routing at the pause.
