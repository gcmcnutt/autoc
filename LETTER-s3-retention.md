# Letter to my other self on promaxgb10-4331 — autoc-storage retention, IAM, and compression

**From:** Claude (admin-account AWS session, 2026-06-02)
**To:** Claude working on the autoc training box (promaxgb10-4331)
**Re:** S3 `autoc-storage` spring cleaning — what I did, what you need to wire into the code + constitution

---

## 1. What just happened on the AWS side

`autoc-storage` (us-west-2) had grown to **~868 GB / 56,471 objects** — per-run prefixes
`autoc-9223370<id>-<ISO timestamp>/genNNNN.dmp`, one ~40 MB population dump per generation.
It was ~84% of the entire AWS bill. I installed a **tag-driven 30-day retention scheme**:

- Object tag **`retain=expire`** → lifecycle rule **`expire-unmarked-runs-30d`** deletes it 30 days
  after object creation.
- Object tag **`retain=keep`** → preserved forever.
- **No tag → never matched → never auto-deleted** (fail-safe by design).

The lifecycle JSON lives in the repo at `autoc-storage-lifecycle.json`. It is a single rule:
filter `Tag retain=expire`, `Expiration: 30 days`.

### Milestone runs I marked `retain=keep` (do NOT let these expire)
| S3 prefix | run | why |
|---|---|---|
| `autoc-9223370259105171692-2026-05-02…` | **pastonly3** (029 M1) | "first M2 pass", flew 5/3 & 5/17, source for 032-phase1 |
| `autoc-9223370257807536859-2026-05-17…` | **032-phase1** (M2) | big tracker milestone, baked from pastonly3 |
| `autoc-9223370256935631488-2026-05-27…` | **033-pop8000-wind36-r1** (M1) | **LIVE** `autoc-tracker.ini` source (gen9432.dmp) |
| `autoc-9223370259246861370-2026-05-01…` | **pastonly2** (029 M1) | 029 baseline, gen 800 |
| `autoc-9223370259505713290-2026-04-28…` | gen-800 orphan | no reference found; kept provisionally, may be tossed |

Everything else (incl. the Mar–Apr flown gen-400 M1 runs) is being tagged `retain=expire`.

### ⚠️ Broken pointer you should fix in the repo
`autoc-eval-tracker.ini:42` sets `TrackerSourceRun = autoc-9223370258388840205-2026-05-11…/gen9999.dmp`
(commented "pastonly3 reference dmp gen9200") — **that prefix has 0 objects in the bucket; it does not
exist.** The eval-tracker source is dangling. Point it at a real run (e.g. the same source as
`autoc-tracker.ini`, the 05-27 run) or re-upload the intended dmp.

---

## 2. IAM — `autoc-generator` needs tagging permission

The upload identity is IAM user **`autoc-generator`**, inline policy **`autoc-s3`**. It currently allows
only `s3:PutObject`, `s3:GetObject`, `s3:ListBucket` on `autoc-storage` (and stale refs to the now-deleted
`autoc-storage-arm`). To tag objects on upload you must add **`s3:PutObjectTagging`** (and
`s3:GetObjectTagging` for verification). Setting a tag inline during `PutObject` via the `x-amz-tagging`
header/SDK `Tagging` field requires `s3:PutObjectTagging` in addition to `s3:PutObject`.

Suggested `autoc-s3` statement (resource scoped to autoc-storage only; drop the dead autoc-storage-arm refs):
```json
{
  "Effect": "Allow",
  "Action": ["s3:PutObject","s3:GetObject","s3:ListBucket","s3:PutObjectTagging","s3:GetObjectTagging"],
  "Resource": ["arn:aws:s3:::autoc-storage","arn:aws:s3:::autoc-storage/*",
               "arn:aws:s3:::autoc-eval-arm","arn:aws:s3:::autoc-eval-arm/*"]
}
```
(I left the live policy unchanged — this is yours to apply intentionally with the code change.)

---

## 3. Code change — tag new uploads `retain=expire` by default

For the bucket to stay self-cleaning, the upload path must set `retain=expire` on every new dump at
`PutObject` time (SDK: set the object `Tagging` to `retain=expire`). New runs then auto-expire after 30
days **unless** someone re-tags that run's objects `retain=keep`.

**Promoting a run to "keep"** (after a flight, a milestone, or selection as an M2 source) should be a
deliberate one-liner, e.g.:
```bash
aws s3api list-objects-v2 --bucket autoc-storage --prefix "<run-prefix>/" --query 'Contents[].Key' --output text \
 | tr '\t' '\n' | xargs -d '\n' -P16 -I{} aws s3api put-object-tagging --bucket autoc-storage --key {} \
   --tagging 'TagSet=[{Key=retain,Value=keep}]'
```
Consider a tiny `autoc-pin <run-prefix>` / `autoc-unpin` helper script in the repo so pinning is trivial
and self-documenting.

---

## 4. Compression — strongly worth doing (operator request)

The gen dumps are highly redundant. I tested one real 40 MB `gen9200.dmp`:

| codec | ratio | savings |
|---|---|---|
| gzip -9 | **2.07×** | 52% |
| xz -6 | **3.29×** | 70% |
| zstd | (untested here — no binary) ~expected 2.5–3× at high level, *much* faster than xz |

The operator guessed ~4×; reality is ~2× (gzip) to ~3.3× (xz) for these float-weight dumps — not 4×, but
still large: at xz, 868 GB → ~265 GB, and it compounds with the 30-day retention.

**Recommendation:** compress on upload. **zstd** is the pragmatic choice (near-xz ratio, fast enough to not
bottleneck training; level ~10–19). Store as `genNNNN.dmp.zst`. **The catch:** the read path must decompress —
update `src/eval/source_dmp_loader.cc` (and anything else that fetches dumps, incl. the renderer/nnextractor
and `TrackerSourceRun` resolution) to transparently inflate `.zst` (and ideally still accept legacy plain
`.dmp`). Validate round-trip byte-equality before trusting it. A standalone "compressibility/throughput"
spike on a few real dumps with zstd levels is a sensible first step before committing the codec.

---

## 5. Constitution incorporation (your repo task)

Add a principle/section — suggest **"Training-Artifact Lifecycle & Retention"** — capturing:

1. **Ephemeral by default.** Every uploaded run dump is tagged `retain=expire` and auto-deleted 30 days
   after creation. Training output is disposable unless explicitly promoted.
2. **Explicit pinning.** A run is preserved only by tagging it `retain=keep` (via `autoc-pin`). Milestones —
   flown controllers, M2 source libraries, documented baselines — MUST be pinned, and the pin recorded in
   the relevant spec's outcome report (with its S3 prefix, as the 029 reports already do).
3. **Provenance lives in the repo, not the bucket.** Flight reports / outcome docs cite the exact S3
   prefix; the bucket is not a system of record. (This is why the 029 logs lacking the prefix was survivable
   — the flight reports had it. New logs DO record the prefix; keep that.)
4. **Compression on upload** once the loader supports it (§4).
5. **Fail-loud, no silent fallback** (existing Principle VII): the dump loader must error if a
   `TrackerSourceRun` key is missing rather than silently substituting — see the dangling
   `autoc-eval-tracker.ini` pointer in §1 as exactly the failure this guards against.

---

## Quick checklist for you
- [ ] Add `s3:PutObjectTagging`/`s3:GetObjectTagging` to `autoc-generator` `autoc-s3` policy (§2)
- [ ] Tag new uploads `retain=expire` in the upload path (§3)
- [ ] Add `autoc-pin`/`autoc-unpin` helper (§3)
- [ ] Spike zstd compression + teach the loader to inflate `.zst` (§4)
- [ ] Fix dangling `autoc-eval-tracker.ini` TrackerSourceRun (§1)
- [ ] Write the retention principle into the constitution (§5)

— prior Claude, admin session 2026-06-02
