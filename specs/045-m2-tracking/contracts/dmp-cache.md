# Contract: local dmp cache (F1)

- **Enable**: env var `AUTOC_DMP_CACHE=<dir>`, read once per process in `src/util/s3_run_selector.cc`. Unset ⇒ one startup line
  "dmp cache: disabled (AUTOC_DMP_CACHE unset)" and today's behaviour.
- **Layout**: `<dir>/<bucket>/<key>`, where `<key>` is the S3 key (e.g. `autoc-…/gen9200.dmp.zst`). Bytes are the **compressed**
  object, exactly as stored in S3.
- **Write-through** (`s3PutDmpBlob`): after the S3 put succeeds, write `<path>.tmp.<pid>` then `rename` to `<path>`. A failure to
  write the cache after a successful put ⇒ **fail loud** with the path (the run must not silently lose its cache).
- **Read-through** (`s3GetDmpBlob`): cache hit ⇒ read and inflate; an inflate failure ⇒ **fail loud** with the path (no silent
  re-fetch). Miss ⇒ S3 GET, then fill the cache (tmp + rename), then inflate.
- **No invalidation**: keys are unique per run-id + gen and never rewritten.
- **Eviction**: `scripts/dmp_cache_sweep.sh` (cross-version utility, so it lives in top-level scripts/): drops entries whose S3
  object is tagged `retain=expire` and older than 30 days; never touches `retain=keep`.
- **Out of scope**: LIST calls (`listAll`, renderer) stay direct.
- Tests: hit / miss / fill, tmp-file atomicity, unwritable dir ⇒ throw, corrupt entry ⇒ throw, unset ⇒ disabled.
