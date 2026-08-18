#!/bin/bash
# live.sh -- one-shot: collector on the Pi (json, line-flushed) piped into the DGX curses display.
# Usage: firmware/beacon-receiver/pi/live.sh [extra collector args, e.g. --chip 121 --shutter 200]
PI=${PI:-pi@100.87.61.53}
HERE=$(cd "$(dirname "$0")" && pwd)
exec ssh -o ServerAliveInterval=10 "$PI" "stdbuf -oL -eL python3 -u beacon_track.py --json --fps 250 --chip 121 --shutter 200 --gain 1 --report 0.5 $*" \
   | stdbuf -iL -oL python3 -u "$HERE/beacon_display.py"
