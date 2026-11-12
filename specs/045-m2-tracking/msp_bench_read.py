#!/usr/bin/env python3
"""msp_bench_read.py — read-only MSP queries for the 045 bench checklist (§ A): identity, box-bit layout, status, GPS, plan settings.
Sends only GET-type requests; writes nothing. Close INAV Configurator first. Usage: python3 msp_bench_read.py [/dev/ttyACM1]"""
import struct, sys, time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM1"


def crc8(crc, b):
    crc ^= b
    for _ in range(8):
        crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def frame(cmd, payload=b""):
    hdr = struct.pack("<BHH", 0, cmd, len(payload))
    c = 0
    for b in hdr + payload: c = crc8(c, b)
    return b"$X<" + hdr + payload + bytes([c])


def req(s, cmd, payload=b"", timeout=1.0):
    s.reset_input_buffer(); s.write(frame(cmd, payload)); t0 = time.time(); buf = b""
    while time.time() - t0 < timeout:
        buf += s.read(256)
        i = buf.find(b"$X>")
        if i >= 0 and len(buf) >= i + 8:
            flags, c, n = struct.unpack("<BHH", buf[i + 3:i + 8])
            if len(buf) >= i + 9 + n:
                if c != cmd: buf = buf[i + 3:]; continue
                return buf[i + 8:i + 8 + n]
        if b"$X!" in buf: return None
    return None


def setting(s, name):
    r = req(s, 0x1003, name.encode() + b"\0")  # MSP2_COMMON_SETTING (get by name)
    return r


def main():
    s = serial.Serial(PORT, 115200, timeout=0.05)
    time.sleep(0.2)
    print("FC_VARIANT", req(s, 2)); v = req(s, 3); print("FC_VERSION", tuple(v) if v else v)
    b = req(s, 4); print("BOARD_INFO", b[:4] if b else b, b)
    print("BUILD_INFO", req(s, 5))
    names = req(s, 116); ids = req(s, 119)
    if names and ids:
        nl = names.decode(errors="replace").strip(";").split(";")
        print("BOX bit -> permanentId : name")
        for bit, (pid, nm) in enumerate(zip(ids, nl)):
            print(f"  bit {bit:2d} -> {pid:3d} : {nm}")
    st = req(s, 0x2000)  # MSP2_INAV_STATUS
    if st:
        cyc, i2c, sens, cpu, prof = struct.unpack("<HHHHB", st[:9]); arm = struct.unpack("<I", st[9:13])[0]
        print(f"INAV_STATUS cycle {cyc} us, cpu {cpu} %, sensors 0x{sens:04x}, armingFlags 0x{arm:08x}")
        print("  raw tail", st[13:].hex())
    g = req(s, 106)  # MSP_RAW_GPS
    if g:
        fix, nsat = g[0], g[1]; lat, lon = struct.unpack("<ii", g[2:10]); alt, spd, crs, hdop = struct.unpack("<hHHH", g[10:18])
        print(f"RAW_GPS fix {fix} sats {nsat} hdop {hdop / 100:.2f}")
    gs = req(s, 166)  # MSP_GPSSTATISTICS
    if gs:
        lastDt, errs, tmo, pkts, hdop, eph, epv = struct.unpack("<HIIIHHH", gs[:22]); print(f"GPSSTATS lastMessageDt {lastDt} ms errors {errs} timeouts {tmo} packets {pkts} eph {eph} epv {epv}")
    for n in ["gps_ublox_use_galileo", "gps_ublox_use_glonass", "gps_ublox_use_beidou", "gps_dyn_model", "gps_ublox_nav_hz",
              "gps_min_sats", "init_gyro_cal", "inav_w_acc_bias", "inav_max_eph_epv", "inav_default_alt_sensor",
              "failsafe_procedure", "failsafe_delay", "failsafe_recovery_delay", "failsafe_off_delay", "failsafe_throttle",
              "motorstop_on_low", "msp_override_channels", "align_mag_roll", "align_mag_pitch", "align_mag_yaw", "align_mag",
              "align_board_roll", "align_board_pitch", "align_board_yaw", "ins_gravity_cmss", "reboot_character",
              "blackbox_rate_denom", "debug_mode", "fw_p_level", "nav_rth_altitude", "nav_rth_climb_first"]:
        r = setting(s, n)
        print(f"  set {n:26s} raw {r.hex() if r else r}")
    s.close()


if __name__ == "__main__":
    main()
