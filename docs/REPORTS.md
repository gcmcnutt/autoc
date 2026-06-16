# Generating the standard run reports (off the dmps)

How the per-run PNGs are produced (035-onward). All analytics come from the
**dmp** (via `dmp-dump`) or the run **`.log`** — `data.dat` is retired (035
FR-P05). The pre-035 `data.dat`-fed scripts are historical and untouched.

## Quick path (037+): one script

The maintained dmp-fed plotters now live in **`src/analytics/`** and are driven by
one wrapper (038 P0-C — the per-feature copies in `specs/034…`/`specs/035…` are
the historical originals; 035-era ones stay put, the 037 copies were consolidated
into `src/analytics/`):

```bash
scripts/generate_pngs.sh m1|m2 <logfile> [--out DIR] [--compare NAME:LOG ...]
```

It derives run-id / bucket / gen / config / name from the run `.log`, fetches the
CSVs via `dmp-dump`, and emits the full set (m1 = 4 reports, m2 = 6). The
run-summary + dynamics_progress S3 fetches are **incrementally cached** per
`(run-id + dmp-dump build)` under `/tmp/generate_pngs_cache/`, so re-plotting a
growing run only fetches new gens. `--compare NAME:LOG` overlays an earlier run's
fitness-over-time on the evolution chart.

The manual per-report recipes below remain valid (they document the individual
plotter flags); the dmp-fed ones now resolve under `src/analytics/`.

## Naming
Artifacts collate lexicographically as `autoc-<feature>-t<N>-<details>`, e.g.
`autoc-035-t4-m1-energy` — `t<N>` is the Nth experiment in the feature. Pass it
as the plot `--label`/`--run-name`; output PNGs are `<name>_<report>.png` and
land in the run's **feature** dir (`specs/<feature>/`), even when the script
invoked lives in an earlier feature's dir.

## Inputs
- **Run `.log`** (in `logs/`, gitignored) — carries `#NNGen` (per-gen best/avg/
  worst/energy/…), `#GenCrash`, and (tracker only) `#GenDiag`.
- **`dmp-dump`** — reads the S3 dmps (zstd-aware). Bucket is the mode: `autoc-m1`
  (M1), `autoc-m2` (M2), `autoc-eval`. `-i <ini>` supplies S3 creds.
  - `s3://autoc-m1/` (no run) → **latest run, latest gen** (`findLatestRun`).
  - `s3://autoc-m1/<run-id>/` → a **specific run** (use this to re-plot a
    finished/dead run so a newer run doesn't shadow it as "latest").
  - `--csv-only` → per-tick CSV (one gen).  `--run-summary` → per-gen CSV over
    the whole run (best_fitness, mean energy/stability/streak, crashes, per-axis
    dctrl/mag, per-path rotation rates).

## The 3 standard M1 reports

Set once:
```bash
D=specs/035-energy-lexicase-objective
NAME=autoc-035-t4-m1-energy            # the run label
LOG=logs/$NAME.log
RUN="s3://autoc-m1/"                    # or s3://autoc-m1/<run-id>/ for a specific run
GEN=$(grep '#NNGen gen=' "$LOG" | tail -1 | grep -oE 'gen=[0-9]+' | cut -d= -f2)
```

**1. evolution_progress** (fitness/streak/sigma + crash panel) — from the `.log`:
```bash
# 037+ runs (T005 slimmed the log — crash panel now parses #GenCrash):
python3 src/analytics/plot_evolution_progress.py \
  --focus "$NAME:$LOG" --run-name "$NAME" --crash-log "$LOG" \
  --total-gens 800 --out $D/${NAME}_evolution_progress.png
# pre-037 logs (which still carry per-scenario [N] CRASH/OK lines): use
# specs/034-energy-objective-cleanup/plot_evolution_progress.py unchanged.
```
> `--crash-log` is required for the crash-rate panel. **037 T005 removed the
> per-scenario `[N] CRASH/OK` lines from the training log**, so the 037 script
> derives the panel from the per-gen `#GenCrash` summary (crashes = hullStrike
> + eval + sim + boot). Per-scenario detail now lives only in the dmp
> (`dmp-dump --meta-only`: crash_reason, score, energy/stability, max_streak,
> streak_steps, max_multiplier, steps).

**2. per_axis_aggressiveness** (6-panel histogram + budget goal lines) — one gen,
from `--csv-only`:
```bash
./build/dmp-dump "$RUN" --csv-only -i autoc.ini > /tmp/t.csv
python3 $D/per_axis_aggressiveness.py /tmp/t.csv --label "$NAME" --gen "$GEN" \
  -o $D/${NAME}_per_axis_aggressiveness.png
```

**3. per_axis_time_series** (4-panel over generations: dCtrl bang-bang detector,
mag saturation detector, per-path roll-rate, per-path pitch-rate) — from
`--run-summary` (iterates every gen dmp, so it's the slow one):
```bash
./build/dmp-dump "$RUN" --run-summary -i autoc.ini > /tmp/t_summary.csv
python3 $D/plot_per_axis_time_series.py /tmp/t_summary.csv --label "$NAME" \
  --total-gens 800 -o $D/${NAME}_per_axis_time_series.png
```

## M2 (tracker) adds two
- **gen_diag** — `python3 specs/034-energy-objective-cleanup/plot_gen_diag.py
  --in $LOG --label $NAME --out …_gen_diag.png` (parses `#GenDiag`, **tracker
  only** — M1 logs don't emit it).
- **intercept_analysis** (7-panel closure-dynamics + sensor-utility chart,
  **tracker only**: A closest-approach traces, B/C outTh-vs-spn0/dspn policy,
  D spn0-vs-dist sensor fidelity, E encounter histogram, F hull-strikes, G
  near-misses) — one gen, from `--csv-only` (the tracker CSV carries
  `tgX..tgZ,trX..trZ,spn0,dspn,blC0,brC0,tltS,tltC`):
```bash
./build/dmp-dump "$RUN" --csv-only -i autoc-tracker.ini > /tmp/t.csv
python3 specs/035-energy-lexicase-objective/intercept_analysis.py --csv /tmp/t.csv \
  --label "$NAME" --gen "$GEN" \
  -o specs/035-energy-lexicase-objective/${NAME}_intercept_analysis.png
```

## Progress proxy: gen duration ≈ fitness (log-only, free)

Per-gen wall-clock (`durationSec` in `#GenSimStats`) is an almost-linear readout of population
fitness — no dmp fetch needed. On t6 (035, gens 1–586): **corr(duration, avg fitness) = −0.95,
corr(duration, best fitness) = −0.97, corr(duration, rabbitComplete) = +0.72.** Mechanism: better
fitness → controllers survive longer → more sim-timesteps per eval → longer gen / lower `rate`.

| rabbitComplete | mean gen duration | mean avg-fitness |
|---|---|---|
| 0 (all crash)  | ~67 s  | −1205 |
| 250–290        | ~144 s | −3876 |
| 290–295        | ~188 s | −6906 |

> Table is the 10 Hz era (035-t6). At 20 Hz (037+) expect ~+20% duration at the
> same regime (FDM substep cost is duration-based and unchanged; NN/RPC/recording
> double): 037-t6 all-crash gens run ~75–80 s at the same pop 5000 / 294.

Uses:
- **Watch the slowing gen time as a live progress signal** — rising duration = still improving; a
  **plateau in duration ≈ fine-tuning has flattened**. Read it straight from the `.log`, no analytics run.
- Duration tracks **best fitness (−0.97) even better than the completion count (+0.72)** — once
  ~290/294 complete, the count saturates but flights keep deepening, so duration stays sensitive in
  the fine-tuning tail.
- Caveats: the variation ramp inflates late-run fitness, and **concurrent analytics
  (`--run-summary` S3 fetches, plotting) steal CPU and add per-gen duration spikes** — don't read a
  spike as a fitness change if you were running reports against the live run.

## Notes
- `dmp-dump` config chatter goes to stderr; stdout is pure CSV/YAML — safe to pipe.
- The per-axis budget goal lines are `dctrl ≤ 0.27`, `mag ≤ 0.67` (sum-over-axes
  ≤ 0.80 / ≤ 2.00), the smooth-control target shared with 034.
- `--run-summary` over a long run is many S3 fetches; use `--stride N` to sample.
- **Incremental run-summary (avoid re-fetching cached gens):** `--run-summary --since-gen N`
  skips gens `< N` (and suppresses the header), so you fetch only new gens and append to a cached
  CSV. Workflow when re-plotting a still-growing run:
  ```bash
  LAST=$(tail -1 /tmp/run_summary.csv | cut -d, -f1)          # highest cached gen
  cp /tmp/run_summary.csv /tmp/merged.csv
  ./build/dmp-dump s3://autoc-m1/ --run-summary --since-gen $((LAST+1)) -i autoc.ini >> /tmp/merged.csv
  head -1 /tmp/merged.csv > /tmp/sorted.csv                    # dedup + numeric sort by gen
  tail -n +2 /tmp/merged.csv | sort -t, -k1,1n -u >> /tmp/sorted.csv
  ```
  e.g. re-plotting t6 at gen 670 with gens 1–565 cached fetched only ~104 new gens: **2.5 min vs
  ~15 min**. (A transparent per-dmp `/tmp` cache is still backlog — this is the manual interim.)
