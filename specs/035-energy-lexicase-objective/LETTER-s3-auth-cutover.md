# LETTER — S3 auth fix + per-mode bucket cutover (handoff to the admin-authed system)

**Date:** 2026-06-05  **Branch:** `035-energy-lexicase-objective`  **Author:** prior session (no IAM access here)

This is a temp handoff. The host where this was written runs as IAM user `autoc-generator`,
which CANNOT do `iam:PutUserPolicy` / bucket admin. You're on a system that CAN. Do the steps,
verify, commit, push back — then delete this letter. **Scope is S3 auth + bucket cutover only.**

**PARALLEL EXECUTION:** this runs concurrently with the originating session, which continues the
US1 energy work (`src/eval/…`, `tests/*energy*`) on the same `035-energy-lexicase-objective`
branch. Your changes are confined to **inis + `s3-admin-setup.sh` + this letter**; the US1 work is
confined to **`src/eval/` + energy tests** — disjoint file sets, so the merge is clean. **Whoever
pushes second does the merge** (`git pull --no-rebase`, resolve only if a shared file like
`tasks.md` was touched on both sides — mark T021/T022 done on your side). Stay out of `src/eval/*`.

## Intent
Make the per-mode buckets usable and turn on retain-tagging:
1. Grant `autoc-generator` the missing S3 perms (operate on `autoc-m1/m2/eval` + Get/PutObjectTagging).
2. Flip each ini's `S3Bucket` to its per-mode bucket (035 T022).
3. Enable `S3ObjectTagging=retain=expire` so uploads are ephemeral (FR-P10) + lifecycle can reap.

Buckets `autoc-m1/m2/eval` **already exist** (don't create). Account `499918285206`.

## Steps

### 1. IAM grant (the actual fix)
Run `specs/035-energy-lexicase-objective/s3-admin-setup.sh` on the admin-authed system
(set `REGION` to the autoc-storage region first). It applies one **additive** inline policy
`autoc-s3-rw-tagging` to `autoc-generator`: ListBucket/GetBucketLocation + Get/Put/Delete/
Get+PutObjectTagging on `autoc-m1/m2/eval/storage/eval-arm`. (Lifecycle + milestone-pin steps are
commented-optional at the bottom — likely already done by the 2026-06-02 retention pass; apply if not.)

### 2. Verify (as `autoc-generator`, i.e. this repo's default profile)
```
aws s3api put-object --bucket autoc-m1 --key _probe --body /etc/hostname --tagging retain=expire
aws s3api delete-object --bucket autoc-m1 --key _probe
```
Both must succeed (they fail AccessDenied today). That's the whole point of the grant.

### 3. T022 — ini `S3Bucket` cutover
| ini | flip S3Bucket → |
|---|---|
| `autoc.ini` | `autoc-m1` (from autoc-storage) |
| `autoc-basic-m1.ini` | `autoc-m1` |
| `autoc-eval.ini` | `autoc-eval` (from autoc-eval-arm) |
| `autoc-eval-visual.ini` | `autoc-eval` |
| `autoc-tracker.ini` | `autoc-m2` — **but see GOTCHA below first** |
| `autoc-eval-tracker.ini` | `autoc-eval` — **same GOTCHA (reads a source)** |

### 4. Enable tagging
Add `S3ObjectTagging = retain=expire` to the training/eval inis (the key exists, default empty;
just add the line). Suggest all 6. Without it, no lifecycle reaping; with it but no IAM grant,
every upload fails — so do step 1 first.

### 5. Verify a short bake
Run a few gens of `autoc.ini` (M1) and confirm `gen<N>.dmp.zst` lands in `autoc-m1` with tag
`retain=expire` (`aws s3api get-object-tagging`). No `S3 upload failed` warnings in the log.

### 6. Commit + push back
Commit the ini changes (`feat(035 T022): per-mode bucket cutover + retain tagging`). Return.

## GOTCHA — M2 source bucket (decide before flipping autoc-tracker.ini)
`source_dmp_loader` fetches `TrackerSourceRun` from **`s3Bucket`** (the same bucket M2 writes to).
The M2 source is an M1 run (currently `autoc-9223370256441628515-…/gen9410.dmp` in `autoc-storage`).
If you set `autoc-tracker.ini` `S3Bucket=autoc-m2`, the loader will look for that source key in
`autoc-m2` and **fail loud** (it isn't there). Options:
- **(a)** Copy the M1 source dmp into `autoc-m2` first (`aws s3 cp --copy-props none`), then flip; OR
- **(b)** Leave `autoc-tracker.ini` (and `autoc-eval-tracker.ini`) on `autoc-storage` until the
  source-bucket story is settled (there is no separate `TrackerSourceBucket` config today —
  adding one is a code change, out of scope for this letter).
Recommend **(b)** unless you also copy the source. The M1/eval inis (step 3) have no such coupling.
Also: the `ini-config.md` contract defaults `LexicaseEpsilonMode=mad`, but **MAD is not implemented
yet (US1 T034 pending)** — leave epsilon mode alone; it's a no-op until that code lands.

## Do NOT touch (US1 in flight on this branch)
`src/eval/selection.cc`, `src/eval/fitness_decomposition.*`, `tests/*energy*` — the energy
lexicase work (T029/T032/T033 done; **T034 MAD-ε pending**) is mid-implementation. Returning with
just the S3/ini changes keeps the merge clean.

## State at handoff (HEAD `26010a3`)
- GATE-1 green; energy axis + convex metric committed; dmp-dump `--run-summary` lands analytics on
  the dmps; constitution v1.5.0 (build-discipline). Training logs live in `logs/` (gitignored).
