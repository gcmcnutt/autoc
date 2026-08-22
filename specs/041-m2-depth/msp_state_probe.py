#!/usr/bin/env python3
"""P5-2 bench instrument: read MSP2_AUTOC_STATE (0x210E) straight off the FC.

Why host-side and not via the xiao: it proves the WIRE. A convention error cannot
hide behind a xiao-side bug, and no firmware flash is needed to run the check.

Prints the accel triple both as INAV sends it (native FLU, milli-g) and after the
FLU->FRD flip that xiao/src/msplink.cpp applies -- the same (+x, -y, -z) as the
quaternion's (w, x, -y, -z) and the gyro's pitch/yaw negation.

P5-2 acceptance, in FRD g, three static attitudes:
    level             [ 0,  0, -1]
    nose up           [+1,  0,  0]
    right wing down   [ 0, -1,  0]

Usage: ./msp_state_probe.py [-p /dev/ttyACM2] [-n 20] [--raw]
"""

import argparse
import struct
import sys
import time

import serial

MSP2_AUTOC_STATE = 0x210E

# Payload layout, must track msp_autoc_state_t in xiao/include/MSP.h and the
# writer in ~/inav/src/main/fc/fc_msp.c (case MSP2_INAV_LOCAL_STATE).
#   u32 timestamp_us | 3x i32 pos (cm, NEU) | 3x i32 vel (cm/s, NEU)
#   4x f32 quat (w,x,y,z) | u32 flightModeFlags | 2x u16 rc8, rc9
#   3x i16 gyro (deci-deg/s, FLU) | 3x i16 accel (milli-g, FLU)   <- 041 P5-1
STATE_FMT = "<I3i3i4fI2H3h3h"
STATE_LEN = struct.calcsize(STATE_FMT)          # 64 with accel, 58 without
STATE_LEN_PRE_ACCEL = STATE_LEN - 6


def crc8_dvb_s2(crc, byte):
    crc ^= byte
    for _ in range(8):
        crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def msp2_frame(cmd, payload=b""):
    body = struct.pack("<BHH", 0, cmd, len(payload)) + payload
    crc = 0
    for b in body:
        crc = crc8_dvb_s2(crc, b)
    return b"$X<" + body + bytes([crc])


def msp2_read(port, want_cmd, timeout=1.0):
    """Return (payload, ok_crc) for the next reply matching want_cmd, or None."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        if port.read(1) != b"$":
            continue
        if port.read(2) != b"X>":
            continue
        head = port.read(5)
        if len(head) != 5:
            continue
        flag, cmd, size = struct.unpack("<BHH", head)
        payload = port.read(size)
        crc_byte = port.read(1)
        if len(payload) != size or len(crc_byte) != 1:
            continue
        crc = 0
        for b in head + payload:
            crc = crc8_dvb_s2(crc, b)
        if cmd == want_cmd:
            return payload, crc == crc_byte[0]
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default="/dev/ttyACM2",
                    help="INAV USB VCP (default: %(default)s)")
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("-n", "--samples", type=int, default=20,
                    help="samples to average (default: %(default)s)")
    ap.add_argument("--raw", action="store_true",
                    help="print every sample instead of the average")
    args = ap.parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.2) as port:
        port.reset_input_buffer()
        req = msp2_frame(MSP2_AUTOC_STATE)

        first = True
        acc_sum = [0.0, 0.0, 0.0]
        gyro_sum = [0.0, 0.0, 0.0]
        got = 0

        for _ in range(args.samples):
            port.write(req)
            reply = msp2_read(port, MSP2_AUTOC_STATE)
            if reply is None:
                print("no reply (is the FC powered, and is this the right port?)",
                      file=sys.stderr)
                return 2
            payload, crc_ok = reply
            if not crc_ok:
                print("CRC mismatch -- discarding sample", file=sys.stderr)
                continue

            if first:
                print(f"payload: {len(payload)} bytes (expect {STATE_LEN})")
                if len(payload) == STATE_LEN_PRE_ACCEL:
                    print(f"*** FC is running PRE-P5-1 firmware ({STATE_LEN_PRE_ACCEL} bytes, "
                          "no accel). Flash the 041 build.", file=sys.stderr)
                    return 3
                if len(payload) != STATE_LEN:
                    print(f"*** unexpected payload size -- writer and this parser disagree",
                          file=sys.stderr)
                    return 3
                first = False

            f = struct.unpack(STATE_FMT, payload)
            # field indices into STATE_FMT: 0 ts | 1-3 pos | 4-6 vel | 7-10 quat
            #                                11 flags | 12-13 rc | 14-16 gyro | 17-19 accel
            gyro = f[14:17]
            accel = f[17:20]
            got += 1
            for i in range(3):
                gyro_sum[i] += gyro[i] / 10.0          # deci-deg/s -> deg/s
                acc_sum[i] += accel[i] / 1000.0        # milli-g    -> g

            if args.raw:
                print(f"  FLU g [{accel[0]/1000: .3f},{accel[1]/1000: .3f},{accel[2]/1000: .3f}]"
                      f"  gyro dps [{gyro[0]/10: .1f},{gyro[1]/10: .1f},{gyro[2]/10: .1f}]")
            time.sleep(0.02)

        if got == 0:
            print("no valid samples", file=sys.stderr)
            return 2

        flu = [s / got for s in acc_sum]
        # The msplink boundary conversion, mirrored: FLU -> FRD is (+x, -y, -z).
        frd = [flu[0], -flu[1], -flu[2]]
        dps = [s / got for s in gyro_sum]

        print(f"\nn={got}")
        print(f"  accel FLU (as INAV sends it, g)   [{flu[0]: .3f}, {flu[1]: .3f}, {flu[2]: .3f}]")
        print(f"  accel FRD (post-msplink flip, g)  [{frd[0]: .3f}, {frd[1]: .3f}, {frd[2]: .3f}]"
              "   <- compare against P5-2")
        print(f"  |accel| = {sum(v * v for v in flu) ** 0.5:.3f} g   (expect ~1.000 at rest)")
        print(f"  gyro INAV-native (deg/s)          [{dps[0]: .1f}, {dps[1]: .1f}, {dps[2]: .1f}]")
        print("\n  P5-2 expects, in FRD g:  level [0,0,-1]   nose up [+1,0,0]   "
              "right wing down [0,-1,0]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
