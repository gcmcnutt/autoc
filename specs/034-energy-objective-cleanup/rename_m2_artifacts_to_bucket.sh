#!/usr/bin/env bash
# One-off: copy-rename the in-flight M2 (tracker) run's S3 dmps so the
# UNCHANGED auto-selector (SetPrefix("autoc-")) can find them — no code rebuild.
#
# Why: the M2 run was launched before the per-mode-bucket decision (2026-06-04),
# so it wrote to the shared training bucket under a `tracker-` run-id prefix:
#     s3://autoc-storage/tracker-<id>/genN.dmp
# nnextractor/renderer auto-select with SetPrefix("autoc-") in a single bucket,
# so they can't see it. New contract = one bucket per mode + `autoc-` naming
# (see specs/035-energy-lexicase-objective/spec.md "S3 storage contract").
# This script moves the run into the M2 bucket and renames the prefix
# tracker- -> autoc-, leaving the selector + run-id naming "as before".
#
# S3 has no rename → this is a copy (aws s3 sync, idempotent/re-runnable).
# RUN IT AFTER the M2 bake finishes (so all gens are present); safe to re-run
# mid-flight + again at the end (sync only copies new objects).
#
# Retire path: once FR-P07 (normalized prefix/bucket-agnostic selector) lands
# pre-035, this rename is no longer needed — tools read the M2 bucket directly.

set -euo pipefail

# --- config ------------------------------------------------------------------
PROFILE="${AWS_PROFILE:-default}"
SRC_BUCKET="autoc-storage"
RUN_ID="9223370256301596645-2026-06-04T06:06:19.162Z"   # the in-flight M2 run
SRC_PREFIX="tracker-${RUN_ID}"                            # current (T033) naming
DST_BUCKET="${M2_BUCKET:-autoc-storage-tracker}"          # <-- SET to the real M2 bucket (env M2_BUCKET=... overrides)
DST_PREFIX="autoc-${RUN_ID}"                              # "as before" naming the selector expects
DRY_RUN="${DRY_RUN:-1}"                                   # 1 = preview only; DRY_RUN=0 to execute
# -----------------------------------------------------------------------------

echo "Source: s3://${SRC_BUCKET}/${SRC_PREFIX}/"
echo "Dest:   s3://${DST_BUCKET}/${DST_PREFIX}/"
echo "Profile: ${PROFILE}   DryRun: ${DRY_RUN}"
echo

echo "=== source object count ==="
aws --profile "${PROFILE}" s3 ls "s3://${SRC_BUCKET}/${SRC_PREFIX}/" | grep -c '\.dmp' || true

# Ensure the destination bucket exists (no-op if already there). Uncomment if
# the bucket isn't provisioned yet (may need --region):
# aws --profile "${PROFILE}" s3 mb "s3://${DST_BUCKET}"

SYNC_ARGS=(--profile "${PROFILE}" s3 sync
           "s3://${SRC_BUCKET}/${SRC_PREFIX}/"
           "s3://${DST_BUCKET}/${DST_PREFIX}/"
           --exclude '*' --include 'gen*.dmp')

if [[ "${DRY_RUN}" == "1" ]]; then
  echo "=== DRY RUN (add --dryrun preview) ==="
  aws "${SYNC_ARGS[@]}" --dryrun
  echo
  echo "Re-run with DRY_RUN=0 (and M2_BUCKET set) to execute."
else
  echo "=== copying ==="
  aws "${SYNC_ARGS[@]}"
  echo "=== dest object count ==="
  aws --profile "${PROFILE}" s3 ls "s3://${DST_BUCKET}/${DST_PREFIX}/" | grep -c '\.dmp' || true
  echo "Done. Verify, then optionally delete the source:"
  echo "  aws --profile ${PROFILE} s3 rm --recursive s3://${SRC_BUCKET}/${SRC_PREFIX}/"
fi
