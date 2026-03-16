#! /bin/bash

# CRRCSim launcher — called by autoc threadpool for each worker
# Args: BASE INSTANCE PORT
BASE=$1
INSTANCE=$2
PORT=$3

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROGDIR=${SCRIPT_DIR}/crrcsim
PROG=build/crrcsim
CRRCSIM_LOGDIR=/tmp/crrcsim
CRRCSIM_LOG=${CRRCSIM_LOGDIR}/autoc_crrcsim-${BASE}.${INSTANCE}.$$.log
export AUTOC_WORKER_INDEX=${INSTANCE}

mkdir -p "${CRRCSIM_LOGDIR}"

case "$INSTANCE" in
  *)
    DISPLAY=:1
    ;;
esac

cd $PROGDIR

DISPLAY=$DISPLAY stdbuf -o0 -e0 $VALGRIND_CMD $PROG -g autoc_config.xml -p "$PORT" -i AUTOC > $CRRCSIM_LOG 2>&1
