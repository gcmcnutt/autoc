#! /bin/bash
#
# train.sh — standard autoc training launcher.
#
# Starts an autoc GA run fully detached so it survives terminal/SSH teardown
# AND agent-session teardown (the harness reaps its own background tasks; see
# memory reference_autoc_launch_command). The run reparents to `systemd --user`,
# in its own session, with core dumps enabled.
#
# Usage:  scripts/train.sh <ini-file> <logfile>
#   <ini-file>  config passed to autoc -i  (e.g. autoc.ini, autoc-tracker.ini)
#   <logfile>   stdout+stderr destination  (e.g. logs/autoc-035-t7-m1-energy.log)
# Both are REQUIRED. Paths are resolved relative to the repo root.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BIN="${REPO_DIR}/build/autoc"

usage() {
  echo "usage: scripts/train.sh <ini-file> <logfile> [extra autoc args...]" >&2
  echo "  e.g. scripts/train.sh autoc.ini logs/autoc-035-t7-m1-energy.log" >&2
  exit 2
}

[ $# -ge 2 ] || usage
INI="$1"
LOG="$2"
shift 2
# 041 T049 — anything after <logfile> is forwarded verbatim to autoc, so an
# ablation run launches through the SAME detached path as every other run
# rather than being invoked by hand outside Constitution IX.
EXTRA_ARGS=("$@")

# --- Validate (fail loud, per project constitution) -------------------------
cd "${REPO_DIR}"

[ -x "${BIN}" ]  || { echo "error: autoc binary not found/executable: ${BIN}" >&2; exit 1; }
[ -f "${INI}" ]  || { echo "error: ini file not found: ${INI}" >&2; exit 1; }

# Refuse to clobber an existing log — pick a new run name instead.
if [ -e "${LOG}" ]; then
  echo "error: logfile already exists: ${LOG}" >&2
  echo "       choose a new run name (don't overwrite a prior run's log)." >&2
  exit 1
fi

# Note: multiple runs against the same ini are allowed (e.g. several smaller
# runs with reduced worker counts) — only the per-run logfile must be unique.

mkdir -p "$(dirname "${LOG}")"

# --- Launch detached --------------------------------------------------------
# setsid  -> new session, no controlling tty (survives terminal SIGHUP and the
#            Bash-tool / harness process-group cleanup; reparents to systemd --user)
# nohup   -> belt-and-suspenders SIGHUP ignore
# stdbuf  -> line-buffer so `tail -f` catches per-gen lines as they happen
#            (ordering: nohup stdbuf <binary>, stdbuf innermost)
# ulimit  -> enable core dumps (system default was reset by a driver update);
#            core_pattern is relative, so core.autoc.<pid> lands in repo root.
ulimit -c unlimited
EXTRA_ARGS_Q=""
for a in ${EXTRA_ARGS+"${EXTRA_ARGS[@]}"}; do
  EXTRA_ARGS_Q="${EXTRA_ARGS_Q} '$(printf '%s' "$a" | sed "s/'/'\\\\''/g")'"
done

setsid bash -c "ulimit -c unlimited; cd '${REPO_DIR}'; \
  exec nohup stdbuf -oL -eL '${BIN}' -i '${INI}' ${EXTRA_ARGS_Q} > '${LOG}' 2>&1 < /dev/null" &
PID=$!          # in a non-interactive script the backsetsid child isn't a group
disown || true  # leader, so setsid exec-chains in place: $! is the autoc pid

# --- Report -----------------------------------------------------------------
sleep 3
# Validate the captured pid is really autoc; fall back to newest match if the
# exec chain surprised us.
if [ ! -d "/proc/${PID}" ] || [ "$(cat /proc/${PID}/comm 2>/dev/null)" != "autoc" ]; then
  PID="$(pgrep -nf "build/autoc -i ${INI}" || true)"
fi

if [ -z "${PID}" ]; then
  echo "warning: launched but no autoc pid found yet; check ${LOG}" >&2
  tail -n 5 "${LOG}" 2>/dev/null || true
  exit 0
fi

echo "autoc training launched:"
echo "  pid        : ${PID}"
echo "  ini        : ${INI}"
echo "  log        : ${LOG}"
echo "  ownership  : $(ps -o ppid=,pgid=,sid=,tty=,stat= -p "${PID}" | tr -s ' ')  (PPID should be systemd --user)"
echo "  core limit : $(grep -i 'core' /proc/${PID}/limits | awk '{print $5}') (soft)"
echo "  tail ${LOG}:"
sleep 2
tail -n 4 "${LOG}" 2>/dev/null | sed 's/^/    /' || true
