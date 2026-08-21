#!/usr/bin/env python3
"""Stable bench serial-port resolution — by DEVICE IDENTITY, not by enumeration order.

The bench has two CDC-ACM devices on the same hub and /dev/ttyACM<N> is assigned in plug order, so the
mapping FLIPS between sessions. It has already flipped once: set_c.py documents
  ttyACM0 = DAPLink (decoder) / ttyACM1 = mEDBG (emitter)
while continue.md's table (written when the StepFPGA was unplugged) documents ttyACM0 = mEDBG. Both were
true on the day they were written. Hardcoding either one means the harness talks to the wrong board and
reports a hardware failure that is really a cabling artefact.

/dev/serial/by-id/ carries the USB descriptors, so it is stable across replugs and plug order:
  usb-ARM_DAPLink_CMSIS-DAP_<serial>-if01  -> STEPLink / decoder BCN telemetry
  usb-ATMEL_mEDBG_CMSIS-DAP_<serial>-if01  -> ATtiny416 emitter pod command link
Env overrides (BCN_PORT / EMITTER_PORT) win, so a one-off odd rig can still be forced from the shell.
"""
import glob, os, sys

BY_ID = "/dev/serial/by-id"
DECODER_GLOB = "usb-ARM_DAPLink_CMSIS-DAP_*-if01"
EMITTER_GLOB = "usb-ATMEL_mEDBG_CMSIS-DAP_*-if01"


def _resolve(env, pattern, what):
    override = os.environ.get(env)
    if override:
        return override
    hits = sorted(glob.glob(os.path.join(BY_ID, pattern)))
    if len(hits) == 1:
        return hits[0]
    if not hits:
        raise SystemExit(f"{what} not found: no {BY_ID}/{pattern}. Is it plugged in? "
                         f"(override with {env}=/dev/ttyACMx)")
    raise SystemExit(f"{what} ambiguous — {len(hits)} matches for {pattern}: {hits}. Set {env}.")


def decoder_port():
    """StepFPGA BCN telemetry CDC (was 'COM3' on the Windows bench)."""
    return _resolve("BCN_PORT", DECODER_GLOB, "StepFPGA decoder (DAPLink CDC)")


def emitter_port():
    """ATtiny416 emitter pod command link, through the mEDBG UART bridge."""
    return _resolve("EMITTER_PORT", EMITTER_GLOB, "emitter pod (mEDBG CDC)")


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "both"
    if which in ("decoder", "both"):
        print(f"decoder {decoder_port()} -> {os.path.realpath(decoder_port())}")
    if which in ("emitter", "both"):
        print(f"emitter {emitter_port()} -> {os.path.realpath(emitter_port())}")
