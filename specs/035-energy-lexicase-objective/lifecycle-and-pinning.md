# Lifecycle & Pinning — autoc S3 dmps (035 FR-P10–P12, Principle VIII)

Training dmps are **ephemeral by default**. `src/autoc.cc` tags every uploaded
dmp `retain=expire` at PutObject (hardcoded in `s3PutDmpBlob`, FR-P10); the
bucket lifecycle policy (`contracts/lifecycle-policy.json`) then expires those
objects after the configured window. This keeps per-gen dmp churn from
accumulating cost while milestone runs are kept explicitly.

> **Fail-fast (Constitution VII):** tagging is not configurable and there is no
> warn-and-continue. The tag rides the PutObject, so it needs the IAM grant
> `s3:PutObject` + `s3:PutObjectTagging` on the bucket (admin prereq T021);
> without it `s3PutDmpBlob` throws and the run aborts rather than silently
> uploading untagged (or not at all). The grant + per-mode buckets + lifecycle
> are now in place (T021), so the ini bucket flip (T022) is live.

## Pinning a run so it never expires (`autoc-pin`)

To keep a run (a milestone, a flight source, a baseline), re-tag its objects
`retain=keep`. Manual one-liner (the "autoc-pin" operation, FR-P12):

```bash
# Pin every dmp under a run prefix so lifecycle never expires it.
BUCKET=autoc-m2
PREFIX=autoc-9223370257807536859-2026-05-17T.../   # the run to keep
aws s3api list-objects-v2 --bucket "$BUCKET" --prefix "$PREFIX" \
    --query 'Contents[].Key' --output text \
  | tr '\t' '\n' \
  | while read -r key; do
      aws s3api put-object-tagging --bucket "$BUCKET" --key "$key" \
        --tagging 'TagSet=[{Key=retain,Value=keep}]'
    done
```

(Single object: `aws s3api put-object-tagging --bucket <b> --key <k>
--tagging 'TagSet=[{Key=retain,Value=keep}]'`.)

## The rule: a pinned run MUST be recorded in an outcome doc

Pinning in S3 alone is invisible on checkout. Whenever a run is pinned, record
its prefix + the *why* in the feature's `outcome.md` (Principle VIII). The
canonical milestone-pin list lives in `tasks.md` T021 (the 2026-06-02 admin
retention pass): pastonly3, 032-phase1 (035 M2 baseline FR-005b), the live
`autoc-tracker.ini` source, pastonly2, and the gen-800 orphan.

## Per-mode buckets (FR-P08)

`autoc-m1` (M1/pathgen), `autoc-m2` (M2/tracker), `autoc-eval` (eval). The
bucket is the mode discriminator (the run-id prefix is uniform `autoc-` per
FR-P07b). Apply `contracts/lifecycle-policy.json` to each bucket. Bucket
creation + IAM `s3:*ObjectTagging` grants are the admin prereq (T021); until
then runs stay on `autoc-storage`/`autoc-eval-arm` and the ini flip (T022) is
deferred.
