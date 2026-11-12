#!/usr/bin/env bash
# dmp_cache_sweep.sh — evict local dmp-cache entries the archive no longer keeps.
# 045 F1 (specs/045-m2-tracking/contracts/dmp-cache.md, docs/toolchains.md § Artifact stores).
#
# The cache (AUTOC_DMP_CACHE) holds <dir>/<bucket>/<run-id>/gen<N>.dmp.zst. This sweep follows the archive's own
# retention classes (Constitution VIII):
#   retain=keep               -> keep, always
#   retain=expire, < N days   -> keep (the archive still has it)
#   retain=expire, >= N days  -> remove (the bucket lifecycle reaps it at 30 days)
#   object gone from S3       -> remove (the lifecycle already reaped it; keep-tagged objects are never reaped)
#   no retain tag             -> keep, with a warning (unknown class: decide by hand)
# Per run it samples the first, middle and last entry; if the three agree, the verdict covers the run, otherwise every entry
# is checked. Stray "*.tmp.*" files (a writer that died mid-write) older than a day are removed too.
# Dry-run by default: prints what it would remove. --apply removes. Any AWS error other than "object gone" stops the sweep.
set -euo pipefail

usage() {
  cat >&2 <<'EOF'
usage: scripts/dmp_cache_sweep.sh -d <cache-dir> [--apply] [--days N] [--profile P]
  -d DIR       the dmp cache root (the value of AUTOC_DMP_CACHE)
  --apply      remove; without it, only report what would be removed
  --days N     age threshold for retain=expire entries (default 30, the bucket lifecycle)
  --profile P  AWS CLI profile (default: the CLI's default)
EOF
  exit 2
}

DIR="" APPLY=0 DAYS=30 PROFILE=()
while [ $# -gt 0 ]; do
  case "$1" in
    -d) DIR="${2:-}"; shift 2 ;;
    --apply) APPLY=1; shift ;;
    --days) DAYS="${2:-}"; shift 2 ;;
    --profile) PROFILE=(--profile "${2:-}"); shift 2 ;;
    -h|--help) usage ;;
    *) echo "dmp_cache_sweep: unknown argument: $1" >&2; usage ;;
  esac
done
[ -n "$DIR" ] || usage
[ -d "$DIR" ] || { echo "dmp_cache_sweep: not a directory: $DIR" >&2; exit 1; }
[[ "$DAYS" =~ ^[0-9]+$ ]] || { echo "dmp_cache_sweep: --days must be a whole number: $DAYS" >&2; exit 1; }

NOW=$(date +%s)

# verdict BUCKET KEY -> keep | remove | unknown   (one AWS call for the tag, one more for the age of expire objects)
verdict() {
  local bucket="$1" key="$2" out lm secs
  if ! out=$(aws s3api get-object-tagging "${PROFILE[@]}" --bucket "$bucket" --key "$key" \
               --query "TagSet[?Key=='retain'].Value" --output text 2>&1); then
    if grep -q -e 'NoSuchKey' -e 'Not Found' <<<"$out"; then echo remove; return; fi
    echo "dmp_cache_sweep: AWS error on s3://$bucket/$key: $out" >&2; exit 1
  fi
  case "$out" in
    keep) echo keep ;;
    expire)
      lm=$(aws s3api head-object "${PROFILE[@]}" --bucket "$bucket" --key "$key" --query LastModified --output text)
      secs=$(date -d "$lm" +%s)
      if [ $(( (NOW - secs) / 86400 )) -ge "$DAYS" ]; then echo remove; else echo keep; fi ;;
    *) echo unknown ;;
  esac
}

removed_files=0 removed_bytes=0 kept_runs=0 unknown_runs=0

remove_file() {  # path
  local sz
  sz=$(stat -c %s "$1")
  removed_files=$((removed_files + 1)); removed_bytes=$((removed_bytes + sz))
  if [ "$APPLY" -eq 1 ]; then rm -f -- "$1"; fi
}

for bdir in "$DIR"/*/; do
  [ -d "$bdir" ] || continue
  bucket=$(basename "$bdir")
  for rdir in "$bdir"*/; do
    [ -d "$rdir" ] || continue
    run=$(basename "$rdir")
    # stray tmp files from a writer that died mid-write
    while IFS= read -r -d '' t; do echo "  tmp   $t"; remove_file "$t"; done \
      < <(find "$rdir" -type f -name '*.tmp.*' -mmin +1440 -print0)
    mapfile -t keys < <(cd "$bdir" && find "$run" -type f ! -name '*.tmp.*' | sort)
    n=${#keys[@]}
    [ "$n" -gt 0 ] || continue
    s0=$(verdict "$bucket" "${keys[0]}")
    s1=$(verdict "$bucket" "${keys[$((n / 2))]}")
    s2=$(verdict "$bucket" "${keys[$((n - 1))]}")
    if [ "$s0" = "$s1" ] && [ "$s1" = "$s2" ]; then
      case "$s0" in
        keep)    kept_runs=$((kept_runs + 1)); echo "keep   $bucket/$run ($n entries)" ;;
        unknown) unknown_runs=$((unknown_runs + 1)); echo "WARN   $bucket/$run ($n entries): no retain tag; kept" >&2 ;;
        remove)  echo "remove $bucket/$run ($n entries)"; for k in "${keys[@]}"; do remove_file "$bdir$k"; done ;;
      esac
    else
      echo "mixed  $bucket/$run ($n entries): checking every entry"
      for k in "${keys[@]}"; do
        v=$(verdict "$bucket" "$k")
        if [ "$v" = remove ]; then echo "  remove $k"; remove_file "$bdir$k"; fi
      done
    fi
    if [ "$APPLY" -eq 1 ]; then rmdir --ignore-fail-on-non-empty "$rdir"; fi
  done
done

gb=$(awk -v b="$removed_bytes" 'BEGIN { printf "%.2f", b / 1e9 }')
if [ "$APPLY" -eq 1 ]; then verb="removed"; else verb="would remove (dry run; --apply to remove)"; fi
echo "dmp_cache_sweep: $verb $removed_files files, $gb GB; kept $kept_runs runs; $unknown_runs runs without a retain tag"
