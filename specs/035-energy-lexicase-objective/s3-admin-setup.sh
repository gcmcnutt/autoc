#!/usr/bin/env bash
# 035 T021 — update the autoc-generator S3 identity (ADMIN-AUTHED system only).
#
# The buckets (autoc-m1/m2/eval) already exist. autoc-generator (the training IAM
# user) currently CANNOT operate on them and lacks s3:PutObjectTagging even on
# autoc-storage — so dmp uploads to the per-mode buckets fail and the
# retain=expire tag (FR-P10) fails AccessDenied. This grants the missing perms.
#
# Paste into a system with IAM admin (this host's default profile = autoc-generator,
# which cannot do iam:PutUserPolicy). Self-contained; review before running.
#
# After this: T022 (flip S3Bucket per ini to autoc-m1/m2/eval). The retain=expire
# object tag is hardcoded in s3PutDmpBlob (fail-fast) — no ini knob to set.
set -euo pipefail

IAM_USER="autoc-generator"
# Buckets the user needs object + tagging + list perms on (per-mode + legacy in use):
ALL_BUCKETS=(autoc-m1 autoc-m2 autoc-eval autoc-storage autoc-eval-arm)

echo "== grant autoc-generator: Get/Put/Delete + Get/PutObjectTagging + ListBucket =="
bucket_arns=""; object_arns=""
for b in "${ALL_BUCKETS[@]}"; do
  bucket_arns="$bucket_arns\"arn:aws:s3:::$b\","
  object_arns="$object_arns\"arn:aws:s3:::$b/*\","
done
bucket_arns="${bucket_arns%,}"; object_arns="${object_arns%,}"

POLICY_JSON=$(cat <<JSON
{
  "Version": "2012-10-17",
  "Statement": [
    { "Sid": "ListBuckets", "Effect": "Allow",
      "Action": ["s3:ListBucket", "s3:GetBucketLocation"],
      "Resource": [ $bucket_arns ] },
    { "Sid": "ObjectRWAndTagging", "Effect": "Allow",
      "Action": ["s3:GetObject", "s3:PutObject", "s3:DeleteObject",
                 "s3:GetObjectTagging", "s3:PutObjectTagging"],
      "Resource": [ $object_arns ] }
  ]
}
JSON
)
# Inline policy is ADDITIVE — it won't clobber autoc-generator's existing
# autoc-storage access; it just unions in the per-mode buckets + tagging.
aws iam put-user-policy --user-name "$IAM_USER" \
  --policy-name autoc-s3-rw-tagging --policy-document "$POLICY_JSON"
echo "  applied inline policy 'autoc-s3-rw-tagging' to $IAM_USER"

echo
echo "== verify (run as autoc-generator afterwards) =="
echo "  aws s3api put-object --bucket autoc-m1 --key _probe --body /etc/hostname --tagging retain=expire"
echo "  aws s3api delete-object --bucket autoc-m1 --key _probe"

# ---------------------------------------------------------------------------
# OPTIONAL (likely already done by the 2026-06-02 admin retention pass — skip if so)
# ---------------------------------------------------------------------------
# Tag-driven 30-day lifecycle on each bucket (retain=expire -> 30d; untagged never matched):
#   LIFE='{ "Rules": [ { "ID":"expire-unmarked-runs-30d","Status":"Enabled",
#     "Filter":{"Tag":{"Key":"retain","Value":"expire"}},"Expiration":{"Days":30} } ] }'
#   for b in autoc-m1 autoc-m2 autoc-eval autoc-storage; do
#     aws s3api put-bucket-lifecycle-configuration --bucket "$b" --lifecycle-configuration "$LIFE"
#   done
#
# Pin milestone runs in autoc-storage (retain=keep, must NOT expire). Numeric run-id
# prefixes are unique. From 035 tasks.md T021:
#   pin() { aws s3api list-objects-v2 --bucket autoc-storage --prefix "$1" \
#       --query 'Contents[].Key' --output text | tr '\t' '\n' | while read -r k; do
#       [ -n "$k" ] && aws s3api put-object-tagging --bucket autoc-storage --key "$k" \
#         --tagging 'TagSet=[{Key=retain,Value=keep}]'; done; }
#   pin autoc-9223370259105171692   # pastonly3 (029 M1) — 032-phase1 source
#   pin autoc-9223370257807536859   # 032-phase1 (M2) — 035 M2 baseline FR-005b
#   pin autoc-9223370256935631488   # 033-pop8000-wind36-r1 (M1)
#   pin autoc-9223370259246861370   # pastonly2 (029 M1) baseline
#   pin autoc-9223370259505713290   # gen-800 orphan (provisional)
#   pin autoc-9223370256441628515   # live autoc-tracker.ini source (gen9410.dmp) — verify before lifecycle
