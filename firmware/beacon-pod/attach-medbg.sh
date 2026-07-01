#!/usr/bin/env bash
# Attach the ATtiny416-XNANO's on-board mEDBG (VID:PID 03eb:2145) into WSL so pymcuprog / PlatformIO can
# upload + monitor. Re-run after any re-plug or reboot — the busid changes (esp. through a hub), but the
# force-bind persists, so this needs no admin. (One-time bind was: admin PowerShell `usbipd bind --force`.)
set -e
BUSID=$(usbipd.exe list | grep -i '03eb:2145' | awk '{print $1}' | tr -d '\r')
[ -z "$BUSID" ] && { echo "mEDBG (03eb:2145) not found — is the XNANO plugged in?"; exit 1; }
echo "mEDBG at busid $BUSID → attaching to WSL…"
usbipd.exe attach --wsl --busid "$BUSID" 2>&1 | grep -viE 'info:' || true
sleep 2
~/.venvs/avr/bin/pymcuprog ping -d attiny416 2>&1 | grep -iE 'Ping response|Error|Unable' \
  && echo "ready: Upload/Monitor from VS Code will work" || echo "attach may have failed — check 'usbipd.exe list'"
