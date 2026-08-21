#!/usr/bin/env bash
# live.sh -- one command for the live ASCII scope: beacon_trackd on the Pi, scope here.
#
#   ./live.sh                          # tcp path (default): trackd LISTENS, scope dials in
#   ./live.sh --pipe                   # ssh-pipe path instead (no ports; json over stdout)
#   ./live.sh --duration 60            # anything else is passed through to beacon_trackd
#   PI=pi@x.y.z.w ./live.sh            # different host
#
# Prefer the tcp path: the sink LISTENS (src/io/emit_record.h) so the daemon does not depend on a client
# existing -- you can Ctrl-C and restart the scope without disturbing the tracker or the camera. It DROPS
# records rather than block when the scope is slow, which is right for a display and wrong for capture; use
# --record / a binary: sink if you need a gap-free stream.
#
# REWRITTEN 2026-08-21. The old version was stale three ways and could not have run: it pointed at the
# retired 3A+ (100.87.61.53, camera removed), it ran the beacon_track.py PROTOTYPE (T067 retires it) rather
# than beacon_trackd, and it passed --chip 121, a rate this single-rate 120 Hz platform no longer has. It
# also fed pi/beacon_display.py, whose flat per-line JSON schema predates beacon_trackd's nested tracks[]
# records. The current display is tools/ascii_scope.py.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"
PI="${PI:-pi@100.97.242.96}"
PORT="${PORT:-5555}"
PI_REPO="${PI_REPO:-autoc-beacon}"
TRACKD="build/firmware/beacon-receiver/beacon_trackd"
CONFIG="${CONFIG:-firmware/beacon-receiver/beacon-bench.ini}"
SCOPE="$REPO/firmware/beacon-receiver/tools/ascii_scope.py"

MODE=tcp; ARGS=()
for a in "$@"; do case "$a" in --pipe) MODE=pipe ;; *) ARGS+=("$a") ;; esac; done

if [[ "$MODE" == pipe ]]; then
  exec ssh -o ServerAliveInterval=10 "$PI" \
    "cd $PI_REPO && stdbuf -oL $TRACKD --config $CONFIG --source live --emit json:- ${ARGS[*]:-}" 2>/dev/null \
    | python3 "$SCOPE" --source json:-
fi

# tcp: start trackd detached on the Pi, wait for the listen socket, then dial in.
# The launch MUST be a subshell -- `setsid nohup ... &` alone leaves ssh waiting on the channel even with
# all three streams redirected, and this hung intermittently while being written. `( ... & )` reaps it.
ssh -n "$PI" "cd $PI_REPO && pkill -x beacon_trackd 2>/dev/null; sleep 0.3; \
  ( setsid nohup $TRACKD --config $CONFIG --source live --emit tcp:0.0.0.0:$PORT ${ARGS[*]:-} \
    >/tmp/trackd.out 2>/tmp/trackd.err </dev/null & )" || true
up=0
for _ in $(seq 30); do
  if ssh -n "$PI" "ss -ltn | grep -q ':$PORT'"; then up=1; break; fi
  sleep 0.5
done
if [[ $up -eq 0 ]]; then
  echo "trackd never opened $PORT — its stderr on the Pi:" >&2
  ssh -n "$PI" "tail -20 /tmp/trackd.err" >&2
  exit 1
fi
trap 'ssh -n "$PI" "pkill -x beacon_trackd" 2>/dev/null || true' EXIT
echo "trackd listening on ${PI#*@}:$PORT — dialing in (Ctrl-C to stop both)…" >&2
python3 "$SCOPE" --source "tcp:${PI#*@}:$PORT"
