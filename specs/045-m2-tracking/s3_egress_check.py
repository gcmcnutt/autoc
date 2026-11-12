#!/usr/bin/env python3
"""045 US1 (T009 / T013, SC-001): count S3 GET.OBJECT requests on one run's dmps from the S3 server access logs.

The training box's IAM user cannot read the access-log bucket, so run this with a profile that can
(s3:ListBucket + s3:GetObject on autoc-s3-access-logs). Access logs arrive with a delay of up to a few hours.

    python3 specs/045-m2-tracking/s3_egress_check.py --profile admin --mode m1 \\
        --run autoc-9223370...Z --day 2026-11-12 [--day 2026-11-13]

Prints GET counts per key and per requester (remote IP, IAM principal, user-agent family), so SC-001 reads directly:
zero GETs from the training box on keys it wrote itself (after write-through), at most one per key otherwise
(after read-through).
"""
import argparse
import collections
import os
import re
import subprocess
import sys
import tempfile

# S3 server access log line (fields up to user-agent; later fields ignored).
LINE = re.compile(
    r'^(?P<owner>\S+) (?P<bucket>\S+) \[(?P<time>[^\]]+)\] (?P<ip>\S+) (?P<requester>\S+) (?P<reqid>\S+) '
    r'(?P<op>\S+) (?P<key>\S+) "(?P<uri>[^"]*)" (?P<status>\S+) (?P<err>\S+) (?P<sent>\S+) (?P<size>\S+) '
    r'(?P<total>\S+) (?P<turn>\S+) "(?P<ref>[^"]*)" "(?P<ua>[^"]*)"')


def fetch_logs(profile, log_bucket, prefix, days, dest):
    for day in days:
        cmd = ["aws", "s3", "cp", f"s3://{log_bucket}/{prefix}", dest, "--recursive", "--quiet",
               "--exclude", "*", "--include", f"{day}-*"]
        if profile:
            cmd[1:1] = ["--profile", profile]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            sys.exit(f"s3_egress_check: aws s3 cp failed ({' '.join(cmd)}):\n{r.stderr.strip()}")


def ua_family(ua):
    return ua.split("/")[0] if ua and ua != "-" else "-"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run", required=True, help="run-id (the key prefix), e.g. autoc-9223370...Z")
    ap.add_argument("--mode", default="m1", help="log prefix under the log bucket: m1 | m2 | eval (default m1)")
    ap.add_argument("--day", action="append", required=True, help="UTC day of the logs, YYYY-MM-DD (repeatable)")
    ap.add_argument("--profile", default=None, help="AWS CLI profile that can read the access-log bucket")
    ap.add_argument("--log-bucket", default="autoc-s3-access-logs")
    ap.add_argument("--logs-dir", default=None, help="use already-downloaded logs in this dir (skips the download)")
    args = ap.parse_args()

    with tempfile.TemporaryDirectory(prefix="s3_egress_") as tmp:
        logs = args.logs_dir or tmp
        if not args.logs_dir:
            fetch_logs(args.profile, args.log_bucket, f"{args.mode}/", args.day, logs)
        per_key = collections.Counter()
        per_who = collections.Counter()
        bytes_who = collections.Counter()
        lines = bad = 0
        for root, _, files in os.walk(logs):
            for f in files:
                with open(os.path.join(root, f), errors="replace") as fh:
                    for line in fh:
                        lines += 1
                        m = LINE.match(line)
                        if not m:
                            bad += 1
                            continue
                        if m["op"] != "REST.GET.OBJECT" or not m["key"].startswith(args.run + "/"):
                            continue
                        who = (m["ip"], m["requester"].rsplit("/", 1)[-1], ua_family(m["ua"]))
                        per_key[m["key"]] += 1
                        per_who[who] += 1
                        sent = m["sent"]
                        bytes_who[who] += int(sent) if sent.isdigit() else 0

    print(f"access-log lines read: {lines} ({bad} unparsed); run {args.run}; days {', '.join(args.day)}")
    if not per_key:
        print("GET.OBJECT on this run: 0")
        return
    print(f"GET.OBJECT on this run: {sum(per_key.values())} over {len(per_key)} keys; "
          f"max per key {max(per_key.values())}")
    print("by requester (remote IP, principal, user agent):")
    for who, n in per_who.most_common():
        print(f"  {n:6d} GETs  {bytes_who[who] / 1e9:8.3f} GB  {who[0]}  {who[1]}  {who[2]}")
    repeats = [(k, n) for k, n in per_key.items() if n > 1]
    if repeats:
        print(f"keys fetched more than once: {len(repeats)}")
        for k, n in sorted(repeats)[:20]:
            print(f"  {n}x  {k}")


if __name__ == "__main__":
    main()
