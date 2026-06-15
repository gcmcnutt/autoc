# OPS NOTES

Operational / infra state that isn't derivable from code. Most recent first.
Propagated via git so every working copy (and Claude instance) sees it.

## 2026-06-15 — M2 tracker source repointed to t10 (the new best-ever M1)

037 closed GO: **t10** (20 Hz + 0.8 s window + honest servo) is the best M1 ever
(gen 800, best −52567.70, pctInStreak 39.6%, 294/294 crash-free, robust 96–100%
on the eval-suite env sweep). It supersedes the 035-t6 M1 source the tracker inis
pointed at. **`autoc-tracker.ini` + `autoc-eval-tracker.ini` `TrackerSourceRun`
repointed** to `autoc-9223370255480237935-2026-06-13T18:15:37.872Z/gen9200.dmp.zst`
(`TrackerSourceBucket=autoc-m1`, gen9200 = gen 800). Source pinned `retain=keep`.
The old 035-t6 source (`autoc-…2026-06-06…/gen9200.dmp.zst`) is still in autoc-m1
but no longer referenced. M2 (US1b) bake is being prepared off this.

## 2026-06-15 — S3 storage decommission + IAM narrowing

**Decision (gcmcnutt):** the current M1 run is the best M1 ever and supersedes the
old milestones, so the **legacy buckets `autoc-storage` and `autoc-eval-arm` are
being DELETED** by the admin-authed system. The loss of the old M2 tracker source
(`033-pop8000-wind36-r1` / `gen9432.dmp`, which `autoc-tracker.ini` had pointed at)
was reviewed and **accepted** — if a future M2 bake needs it, it is gone.

**Active storage going forward:** per-mode buckets only —
`autoc-m1` (M1/pathgen), `autoc-m2` (M2/tracker), `autoc-eval` (eval).
All three carry the 30-day tag-driven lifecycle
(`retain=expire` → expire @30d; `retain=keep` → pinned; untagged → never matched).
Verified working 2026-06-15: the generator default-tags new dmps `retain=expire`
(hardcoded in `s3PutDmpBlob`, see [035 lifecycle-and-pinning](specs/035-energy-lexicase-objective/lifecycle-and-pinning.md)),
milestones are `retain=keep`. `autoc-storage` was confirmed purging on schedule
(868 GB peak 6/2 → ~719 GB by 6/14) before the decision to delete it outright.

**IAM narrowing — DONE 2026-06-15** (buckets deleted, then applied). The
`autoc-generator` IAM user now has exactly two inline policies:
- `autoc-generator` — SQS only (`autoc-tasks`). Unchanged.
- `autoc-s3-rw-tagging` — RW+tagging + list scoped to `autoc-m1`/`m2`/`eval` only.

The legacy `autoc-s3` inline policy (referenced only `autoc-storage`,
`autoc-eval-arm`, and the never-existed `autoc-storage-arm`) was **deleted**, and
the two dead ARNs were trimmed from `autoc-s3-rw-tagging`. The admin setup script
[specs/035-…/s3-admin-setup.sh](specs/035-energy-lexicase-objective/s3-admin-setup.sh)
`ALL_BUCKETS` was reduced to m1/m2/eval so a re-run won't re-grant dead scope.
