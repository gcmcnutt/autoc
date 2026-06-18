# Generating the standard run reports (off the dmps)

How the per-run PNGs are produced (035-onward). All analytics come from the
**dmp** (via `dmp-dump`) or the run **`.log`** — `data.dat` is retired (035
FR-P05). The pre-035 `data.dat`-fed scripts are historical and untouched.

## The script (standard path)

One wrapper produces the whole package from a run logfile:

```bash
scripts/generate_pngs.sh m1|m2 <logfile> [--out DIR] [--compare NAME:LOG ...]
```

- Derives run-id / bucket (`autoc-m1`|`autoc-m2`) / latest-gen / config
  (`autoc.ini`|`autoc-tracker.ini`) / name from the run `.log`; fetches the CSVs via
  `dmp-dump`; writes `<name>_<report>.png` into the run's feature dir (`specs/<NNN>-*/`,
  derived from the name) or `--out DIR`.
- **Incremental + cached**: the two slow S3 paths — `--run-summary` and
  `dynamics_progress` — cache per `(run-id + dmp-dump build)` under
  `/tmp/generate_pngs_cache/`, so re-plotting a growing run fetches only new gens.
  `--full-summary` forces a refetch; `--no-cache` disables the cache.
- `--compare NAME:LOG` (repeatable) overlays an earlier run's fitness-over-time on the
  evolution chart.

Plotters live in **`src/analytics/`** (deps: `src/analytics/requirements.txt`). The
per-feature copies under `specs/034…`/`specs/035…` are the historical originals (035-era
stay put; the 037 copies were consolidated into `src/analytics/`).

## Naming
Artifacts collate lexicographically as `autoc-<feature>-t<N>-<details>`, e.g.
`autoc-035-t4-m1-energy` — `t<N>` is the Nth experiment in the feature. It is the run
`.log` basename and the plot `--label`/`--run-name`; output PNGs are `<name>_<report>.png`
and land in the run's **feature** dir (`specs/<feature>/`).

## Inputs
- **Run `.log`** (in `logs/`, gitignored) — carries `#NNGen` (per-gen best/avg/worst/
  energy/…), `#GenCrash`, and (tracker only) `#GenDiag`. The script reads run-id / gen
  from it.
- **`dmp-dump`** — reads the S3 dmps (zstd-aware). Bucket is the mode: `autoc-m1` (M1),
  `autoc-m2` (M2), `autoc-eval`. `-i <ini>` supplies S3 creds.
  - `s3://autoc-m1/<run-id>/` → a **specific run** (the script always uses this form, from
    the log's run-id, so a newer run can't shadow it as "latest").
  - `--csv-only` → per-tick CSV (one gen).  `--run-summary` → per-gen CSV over the whole
    run.  `--run-summary --since-gen N` → only gens ≥ N (header suppressed) — the
    incremental primitive the script's cache is built on.

## The packages — what each report contains

**M1 package (4)** — `generate_pngs.sh m1 <log>`:
1. **evolution_progress** — fitness/streak/sigma + crash panel (from the `.log`; the crash
   panel parses the per-gen `#GenCrash` summary on 037+). `--compare` overlays land here.
2. **per_axis_aggressiveness** — 6-panel dctrl/amplitude histograms + budget goal lines
   (one gen, `--csv-only`).
3. **per_axis_time_series** — 4-panel over generations: dCtrl bang-bang detector, mag
   saturation detector, per-path roll-rate, per-path pitch-rate (`--run-summary`).
4. **dynamics_progress** — per-gen de-alias + tracking-quality trajectories (lag-1
   autocorr, sign-flip %, saturation %, regime occupancy, target distance); per-gen
   `--csv-only`, stride-sampled.

**M2 package (6)** — `generate_pngs.sh m2 <log>` — the 4 above **plus** (tracker-only):
5. **gen_diag** — parses `#GenDiag` (M1 logs don't emit it).
6. **intercept_analysis** — 7-panel closure-dynamics + sensor-utility: closest-approach
   traces, outTh-vs-spn0/dspn policy, spn0-vs-dist sensor fidelity, encounter histogram,
   hull-strikes, near-misses (one gen, `--csv-only`).

## Manual invocation (reference — individual flags / debugging)

The script is the standard path; reach for raw `dmp-dump` + a single plotter only to tweak
one chart or debug. Pattern (M2 shown; M1 = `autoc.ini` + `s3://autoc-m1/…`, and skip the
last two tracker-only plots):
```bash
NAME=autoc-037-t11-m2; LOG=logs/$NAME.log; D=specs/037-20hz-control-loop
RUN="s3://autoc-m2/<run-id>/"     # specific run so a newer one can't shadow it
GEN=$(grep '#NNGen gen=' "$LOG" | tail -1 | grep -oE 'gen=[0-9]+' | cut -d= -f2)
./build/dmp-dump "$RUN" --csv-only    -i autoc-tracker.ini > /tmp/t.csv
./build/dmp-dump "$RUN" --run-summary -i autoc-tracker.ini > /tmp/t_summary.csv
python3 src/analytics/plot_evolution_progress.py --focus "$NAME:$LOG" --run-name "$NAME" \
  --crash-log "$LOG" --total-gens 800 --out $D/${NAME}_evolution_progress.png
python3 src/analytics/per_axis_aggressiveness.py  /tmp/t.csv --label "$NAME" --gen "$GEN" -o $D/${NAME}_per_axis_aggressiveness.png
python3 src/analytics/plot_per_axis_time_series.py /tmp/t_summary.csv --label "$NAME" --total-gens 800 -o $D/${NAME}_per_axis_time_series.png
python3 src/analytics/dynamics_progress.py --run "$RUN" --gens 1-$GEN --stride 5 -i autoc-tracker.ini --label "$NAME" -o $D/${NAME}_dynamics_progress.png
python3 src/analytics/plot_gen_diag.py       --in "$LOG" --label "$NAME" --out $D/${NAME}_gen_diag.png            # M2 only
python3 src/analytics/intercept_analysis.py  --csv /tmp/t.csv --label "$NAME" --gen "$GEN" -o $D/${NAME}_intercept_analysis.png  # M2 only
```
> 037 T005 removed the per-scenario `[N] CRASH/OK` lines from the training log, so the
> evolution crash panel derives from the per-gen `#GenCrash` summary (crashes = hullStrike
> + eval + sim + boot). Pre-037 logs (which still carry the per-scenario lines): use the
> historical `specs/034-energy-objective-cleanup/plot_evolution_progress.py`. Per-scenario
> crash detail now lives in the dmp (`dmp-dump --meta-only`).

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
- `--run-summary` over a long run is many S3 fetches; use `--stride N` to sample, or just
  let the script's `(run-id + dmp-dump build)` cache fetch only new gens. The cache forks on
  a dmp-dump rebuild, so old-binary and new-binary rows never splice. (`--since-gen` is the
  underlying primitive; the manual workflow it replaced is in git history.)
- A per-gen analytics **SQLite store** is the planned successor to the CSV cache (BACKLOG).
