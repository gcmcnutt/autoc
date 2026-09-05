#! /bin/bash

# CRRCSim launcher — called by autoc threadpool for each worker
# Args: BASE INSTANCE PORT
BASE=$1
INSTANCE=$2
PORT=$3

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="${SCRIPT_DIR}/.."
PROGDIR=${REPO_DIR}/crrcsim
PROG=${REPO_DIR}/build/crrcsim/crrcsim
CRRCSIM_LOGDIR=/tmp/crrcsim
CRRCSIM_LOG=${CRRCSIM_LOGDIR}/autoc_crrcsim-${BASE}.${INSTANCE}.$$.log
export AUTOC_WORKER_INDEX=${INSTANCE}

mkdir -p "${CRRCSIM_LOGDIR}"

# Display for the visual worker. Inherits the caller's DISPLAY (WSLg is :0 here);
# override with AUTOC_VISUAL_DISPLAY=:N when driving a VNC/Xvfb server instead.
# Fail loud rather than silently defaulting to a display that may not exist --
# a wrong value here fails inside crrcsim's log, far from the operator.
DISPLAY="${AUTOC_VISUAL_DISPLAY:-${DISPLAY:?no DISPLAY set and AUTOC_VISUAL_DISPLAY unset}}"

# ⚠️ TEMPORARY (2026-09-04) -- force Mesa onto the llvmpipe software rasterizer.
# WHY: on this WSLg host the D3D12 gallium driver (Intel Arc iGPU, Mesa 25.2.8)
# removed the GPU device mid-run ("D3D12: Removing Device." in the crrcsim log),
# and the next SDL_GL_SwapBuffers segfaulted inside libgallium
# (crrc_graphics.cpp:841). That killed crrcsim, autoc's receiveRPC read threw
# "connection closed by peer" uncaught, and a second attempt took the whole
# MACHINE down. Cores: /var/core/core-crrcsim-25701-*, core-autoc-25684-*.
# ⛔ NOT a fix -- it trades the GPU for the CPU (expect low fps; the sim is
# lockstepped to autoc so the FLIGHT is unaffected, only wall-clock). Remove
# once the WSLg/D3D12 stack is trusted again, or when running on a host with a
# working hardware GL path.
# NOTE: LIBGL_ALWAYS_SOFTWARE=1 does NOT work here -- WSLg ignores it and stays
# on D3D12. GALLIUM_DRIVER is the override that actually takes.
# Set AUTOC_VISUAL_GALLIUM= (empty) to retest the hardware path.
export GALLIUM_DRIVER="${AUTOC_VISUAL_GALLIUM-llvmpipe}"

cd $PROGDIR

DISPLAY=$DISPLAY stdbuf -o0 -e0 $VALGRIND_CMD $PROG -g autoc_config-eval.xml -p "$PORT" -i AUTOC > $CRRCSIM_LOG 2>&1
