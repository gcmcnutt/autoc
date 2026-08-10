# 041 artifacts

What lives here, what does not, and the provenance a later reader will otherwise have to re-derive.

## What belongs here

| item | task | why here |
|---|---|---|
| `pre-break/` — per-tick + `--physics` CSVs from **both pinned comparators** | T011a | ⛔ Extracted **before** T044's version bump, which makes those dmps unreadable to a 041 binary. This is the only surviving route to the prior-M1 per-regime baseline (SC-007a) and the blind-gap distribution (FR-024b) |
| Final M1 / M2 `data.dat` snapshots | T065, T098 (FR-022) | The next training launch overwrites `data.dat` in the repo root |
| Archived `nn_weights*.dat` for every pinned run | T065, T098 (FR-010) | A dmp preserves *numbers*; only the weight format preserves a **controller you can re-fly** — the gap that made the 038 baseline unloadable |
| The two-variant INAV build/deploy sequence | T001a | Written when Phase 6 runs; points at `specs/020-pre-flight-pipeline/plan.md` for the commands rather than restating them |

## What does not belong here

Anything re-derivable from S3. Whole dmp sets, per-generation dumps, generated plots — those live in the
per-mode buckets and are fetched on demand. This directory is for things that are **destroyed by the next
run** or **unreadable after the next commit**, not for a second copy of durable storage.

## ⚠️ git tracks the CSVs but NOT the `.dat` files

The root [`.gitignore`](../../../.gitignore) has an unanchored `*.dat` rule (line 6), so **every `.dat` file
placed here is ignored**, including the FR-010 weight archives and the FR-022 `data.dat` snapshots. The
`/*.csv` rule is root-anchored, so `pre-break/*.csv` **is** tracked normally.

This is deliberate rather than an oversight to fix:

- `data.dat` is a per-tick trace and runs to tens of MB — the wrong thing to put in git history.
- The **durable** copy of anything `.dat` is **S3**, beside its dmp (that is what FR-010 means by
  "archived alongside"). The copy here is a working convenience for the machine that ran the bake.
- Consequence to keep in mind: **`.dat` files here do not sync across machines.** If a `.dat` artifact is
  needed elsewhere, fetch it from S3 — do not assume a checkout carries it.

If a future feature genuinely needs a small `.dat` in git, add a targeted `!` negation rather than
loosening the global rule.

## Pinned comparators (T004)

Both verified object-by-object on **2026-08-07**, 800/800 objects tagged `retain=keep`. Recorded in-repo per
Constitution VIII.3 — provenance lives in the repo, not in a shell history.

| role | bucket / prefix | state |
|---|---|---|
| **Prior M1 baseline** (SC-007a, FR-011c) | `autoc-m1/autoc-9223370253553029228-2026-07-06T01:35:46.579Z/` | 800/800 `retain=keep` ✅ |
| **040-t4 M2 comparator** (FR-024b blind-gap source) | `autoc-m2/autoc-9223370251039771221-2026-08-04T03:43:24.586Z/` | 800/800 `retain=keep` ✅ |

⚠️ **Both become unreadable to a 041 binary at T044.** The prior M1 is worse off still: its genome has 37
inputs, so a 041 binary cannot load it *at all* — it can never be re-evaluated or re-ablated, and the
recorded dmp is the only route to that baseline. That is what makes T011a's extraction unrecoverable if
skipped, and why `pre-break/` is the first thing in this directory.

Reminder on the filename encoding: dmp filenames encode `10000 − actual_gen`, so `gen9200.dmp` is
generation 800. Invert before comparing anything across runs.

## Camera model facts that are *not* changes (T041c)

Recorded because a later reader will otherwise spend a session re-deriving them:

- **The projection is already equidistant** (`crrcsim`-independent; `src/eval/camera_projection.cc:158-184`,
  since 040 T031). Bearings are θ in radians against a single uniform `radPerPx`. 041 does **not** touch it.
  ⚠️ Historical cost: 038-t9's switch to equidistant near-froze evolution (~57 generations with the elite
  unreplaced), because the rectilinear tan-stretch had been an accidental training aid at the frame edge.
  Do not re-open the projection casually.
- **`CameraDetectionRangeM = 100.0` is independently corroborated** by the 031 photon budget: bright-day
  post-correlation SNR ≈ 22 at 100 m, ÷4 at 4×4 defocus → ≈ 5.5 against a ×4.5 detection threshold. The
  value was assumed; it now has a physical second opinion. Not a change.

The one camera change 041 *does* make is T041a: `CameraPixelsV 240 → 200`, giving V = 75° instead of 90°.
Fitness-affecting, so it lands in the A1 bundle only.
