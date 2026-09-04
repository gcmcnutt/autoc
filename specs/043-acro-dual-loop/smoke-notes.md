# 043 — smoke / eval / manual-fly notes

Prepared 2026-08-25. Three ways to exercise the always-on `Cntrl_InavFwRate`
ACRO rate loop before the T044/T045 gates and the bake. All need an X display
(the crrcsim workers/visual/interactive use one) — that's your environment.

## Files prepared

⛔ **DELETED 2026-09-04 — the three `autoc-043-*.ini` files are gone.** Per-feature configs go stale the
moment `autoc.ini` moves, and a stale config silently evaluates a different aircraft than the one trained
(that is exactly how `autoc-eval.ini` drifted into being a novel-path harness while still claiming to
mirror training). The commands below are kept for provenance; recreate a one-off by patching a **copy** of
`autoc.ini` / `autoc-eval.ini` in a temp dir rather than adding a tracked file.

| file (deleted) | what it was |
|---|---|
| `autoc-043-smoke-m1.ini` | M1 smoke: basic-m1 shape (1 path × 16 winds, pop 3000, 400 gens) + the new craft IMU/CmQ σ + `ExpectedScenarioCount=16`. ACRO is always-on via `autoc_config.xml`. |
| `autoc-043-smoke-m1-eval.ini` | **visual** eval companion (`crrcsim-visual.sh`, DISPLAY :1) — replays a smoke genome. |
| `autoc-043-t2-eval.ini` | the T045 determinism mirror. Superseded: `autoc-eval.ini` now mirrors `autoc.ini` exactly, and `scripts/eval_suite.sh <nn01> 0 <master_seed>` runs that gate directly. |

⭐ **The 043 change is not an ini knob.** `Cntrl_InavFwRate` loads from the
`<config><controllers>` node in the **model file**
`crrcsim/models/hb1_streamer.xml` (moved there 2026-08-30, the `fdm_mcopter01`
per-model pattern). So the rate loop follows the *airframe*, not the run config:
every path (headless worker, visual eval, manual fly) gets it automatically, and
loading a different model cannot silently inherit hb1's INAV gains.

⭐ **crrcsim is parameterised by OVERRIDES, not per-use config copies** (see
`scripts/crrcsim-visual.sh`): the visual worker is just the same launch with
`-g autoc_config-eval.xml` (video=1) instead of `autoc_config.xml` (video=0),
plus env hooks like `VALGRIND_CMD`. So there is no separate manual/visual XML —
swap the `-g` config. `autoc_config-eval.xml` already is the video-on config
(video=1, mouse input, controllers node) and doubles as the manual-fly config.

## 1. M1 smoke (does the ACRO plant still train?)

```
scripts/train.sh autoc-043-smoke-m1.ini logs/autoc-043-t1-smoke.log
scripts/generate_pngs.sh m1 logs/autoc-043-t1-smoke.log     # monitor
```

**Watch:**
- The crrcsim worker log (`/tmp/crrcsim/…`) must print
  **`[FDM] model-local controllers loaded: 1`**. If it says `none (no
  <config><controllers> node)` the chase is flying MANUAL — wrong model, or the
  node was lost. A present-but-broken node (unknown name / missing gain key) is
  now **fatal**, not silent.
- `[AUTOC] cadence: … dt=0.005 …` (200 Hz substep, T003).
- Best fitness should climb about as solidly as the basic-m1 baseline. A flat /
  stalled climb = the ACRO plant may be hard to train → that's the **T044**
  signal, decide before the bake.
- Startup already verified here (2026-08-25): config parses, `ExpectedScenarioCount:
  16` passes the regiment gate, the prefetch table prints the new `misR…cmQ`
  columns, master seed printed.

## 2. Visual eval (see the flow)

After the smoke finishes: note the **Effective master seed** + S3 run id from the
log, extract the genome, then fill the two placeholders in the eval ini.

```
./build/nnextractor <s3-run-prefix-or-key> -g <gen> -o nn_weights.dat
#   ^ -g now takes the ACTUAL generation (043 T022), like dmp-dump --gen
# edit autoc-043-smoke-m1-eval.ini:  Seed = <master seed>,  NNWeightFile = nn_weights.dat
DISPLAY=:1 ./build/autoc -i autoc-043-smoke-m1-eval.ini      # visual worker draws the chase
```

(For the **headless** eval-vs-training bitwise determinism check — T045 — instead
set `WorkerProgram = ./scripts/crrcsim.sh` in the eval ini; the eval fitness must
match the stored training fitness bit-for-bit.)

## 3. Manual flight (does it FEEL like ACRO?)

Reuse the video-on eval config (video=1 + mouse + controllers) — just drop
`-i AUTOC` so crrcsim runs interactively instead of as a worker:

```
cd crrcsim && DISPLAY=:0 ../build/crrcsim/crrcsim -g autoc_config-eval.xml
#   ⛔ NO '-i AUTOC' — that makes it a worker that connects to a parent.
#   Mouse flies by default (aileron=mouse-x, elevator=mouse-y). A joystick is
#   better for feel: set <inputMethod method="Joystick"> (override the config,
#   don't fork it — the crrcsim-visual.sh pattern).
```

**The ACRO tell** (contrast with ANGLE):
- **Deflect and hold** → the aircraft keeps *rotating* at a rate set by stick
  position (not a fixed bank). Pitch is far less responsive than roll —
  `pitch_rate 12` vs `roll_rate 36` (3:1), and pitch goes nearly pure
  feed-forward above ~40 °/s (research.md R8).
- **Release to center** → rotation **STOPS** and the current attitude is *held* —
  it does **NOT** roll back to level. If it self-levels, something built ANGLE
  (FR-019a) — but the model has no attitude term, and the T043 sweep already
  proved no self-levelling, so a level-off in manual flight would point at the
  input path, not the loop.
- Zero command in a bank drifts only slowly (seconds), with no restoring bias.

⚠️ Manual flight was **not** run here (no interactive display in this
environment); the config is prepared and the loop math + auto-routing are
verified by the contract/sweep tests.
