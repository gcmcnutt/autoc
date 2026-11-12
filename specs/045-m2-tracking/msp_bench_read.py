#!/usr/bin/env python3
"""msp_bench_read.py — read-only MSP queries for the 045 bench checklist. Sends only GET-type requests; writes nothing,
never enters the CLI. Close INAV Configurator first (only one program can hold the port).

Modes:
  (default)      § A dump: identity, box-bit layout, status, GPS, the plan's settings
  --svinfo       per-satellite C/N0 (checklist D4) via MSP_GPSSVINFO. INAV 8.0 answers it with a compatibility stub (no
                 satellites, fc_msp.c MSP_GPSSVINFO), so this reports the stub and prints the coarse MSP proxy instead
                 (sats, HDOP, EPH / EPV). Per-satellite C/N0 on INAV 8.0 needs the CLI `gpssats` command (disarmed only;
                 leaving the CLI with `exit` reboots the FC) or `gpspassthrough` to u-center.
  --watch S      every --interval seconds for S seconds (0 = until Ctrl-C): I2C error count (checklist A10), cycle time,
                 CPU load, armed, sats, HDOP, EPH / EPV, GPS errors / timeouts. Works armed too (MSP reads are allowed
                 while armed), so it covers motor-on vs motor-off and xiao-powered vs unpowered comparisons (D2, D4).
                 --csv FILE also writes the rows. --label TEXT tags them (e.g. "motor 50%").

Port: --port PATH, or auto (default): the FC is the device whose udev ID_MODEL contains "STM32" (the /dev/ttyACM*
numbers swap between sessions)."""
import argparse
import csv
import glob
import struct
import subprocess
import sys
import time

import serial

MSP_FC_VARIANT, MSP_FC_VERSION, MSP_BOARD_INFO, MSP_BUILD_INFO = 2, 3, 4, 5
MSP_RAW_GPS = 106
MSP_BOXNAMES, MSP_BOXIDS = 116, 119
MSP_GPSSVINFO = 164
MSP_GPSSTATISTICS = 166
MSP2_COMMON_SETTING = 0x1003
MSP2_INAV_STATUS = 0x2000
ARMING_FLAG_ARMED = 1 << 2  # fc/runtime_config.h
FIX_NAMES = {0: "none", 1: "2D", 2: "3D"}  # io/gps.h gpsFixType_e


def crc8(crc, b):
    crc ^= b
    for _ in range(8):
        crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def frame(cmd, payload=b""):
    hdr = struct.pack("<BHH", 0, cmd, len(payload))
    c = 0
    for b in hdr + payload:
        c = crc8(c, b)
    return b"$X<" + hdr + payload + bytes([c])


def req(s, cmd, payload=b"", timeout=1.0):
    s.reset_input_buffer(); s.write(frame(cmd, payload)); t0 = time.time(); buf = b""
    while time.time() - t0 < timeout:
        buf += s.read(256)
        i = buf.find(b"$X>")
        if i >= 0 and len(buf) >= i + 8:
            flags, c, n = struct.unpack("<BHH", buf[i + 3:i + 8])
            if len(buf) >= i + 9 + n:
                if c != cmd:
                    buf = buf[i + 3:]; continue
                return buf[i + 8:i + 8 + n]
        if b"$X!" in buf:
            return None
    return None


def setting(s, name):
    return req(s, MSP2_COMMON_SETTING, name.encode() + b"\0")  # get by name


def find_fc_port():
    for p in sorted(glob.glob("/dev/ttyACM*")):
        r = subprocess.run(["udevadm", "info", "-q", "property", "-n", p], capture_output=True, text=True)
        model = next((l.split("=", 1)[1] for l in r.stdout.splitlines() if l.startswith("ID_MODEL=")), "")
        if "STM32" in model:
            return p
    sys.exit("msp_bench_read: no FC found (no /dev/ttyACM* with ID_MODEL containing STM32); pass --port")


# --- parsers ---------------------------------------------------------------------------------------------------------

def parse_status(st):
    cyc, i2c, sens, cpu, prof = struct.unpack("<HHHHB", st[:9])
    arm = struct.unpack("<I", st[9:13])[0]
    return {"cycle_us": cyc, "i2c_errors": i2c, "sensors": sens, "cpu_pct": cpu, "arming_flags": arm,
            "armed": int(bool(arm & ARMING_FLAG_ARMED))}


def parse_raw_gps(g):
    fix, nsat = g[0], g[1]
    alt, spd, crs, hdop = struct.unpack("<hHHH", g[10:18])
    return {"fix": fix, "sats": nsat, "hdop": hdop / 100.0}


def parse_gps_stats(gs):
    last_dt, errs, tmo, pkts, hdop, eph, epv = struct.unpack("<HIIIHHH", gs[:22])
    return {"gps_dt_ms": last_dt, "gps_errors": errs, "gps_timeouts": tmo, "gps_packets": pkts,
            "eph_m": eph / 100.0, "epv_m": epv / 100.0}


def parse_svinfo(p):
    """Legacy MSP_GPSSVINFO: U8 numCh, then per channel U8 chn, U8 svid, U8 quality, U8 cno.
    INAV 8.0 stub: U8 1, then U8 0, U8 0, U8 hdop, U8 hdop (5 bytes, no satellites). Returns (is_stub, [(svid, q, cno)])."""
    if p is None:
        return None, []
    if len(p) == 5 and p[0] == 1 and p[1] == 0 and p[2] == 0:
        return True, []
    n = p[0]
    sats = []
    for k in range(n):
        o = 1 + 4 * k
        if o + 4 > len(p):
            break
        chn, svid, q, cno = p[o:o + 4]
        sats.append((svid, q, cno))
    return False, sats


# --- modes -----------------------------------------------------------------------------------------------------------

def dump(s):
    print("FC_VARIANT", req(s, MSP_FC_VARIANT)); v = req(s, MSP_FC_VERSION); print("FC_VERSION", tuple(v) if v else v)
    b = req(s, MSP_BOARD_INFO); print("BOARD_INFO", b[:4] if b else b, b)
    print("BUILD_INFO", req(s, MSP_BUILD_INFO))
    names = req(s, MSP_BOXNAMES); ids = req(s, MSP_BOXIDS)
    if names and ids:
        nl = names.decode(errors="replace").strip(";").split(";")
        print("BOX bit -> permanentId : name")
        for bit, (pid, nm) in enumerate(zip(ids, nl)):
            print(f"  bit {bit:2d} -> {pid:3d} : {nm}")
    st = req(s, MSP2_INAV_STATUS)
    if st:
        d = parse_status(st)
        print(f"INAV_STATUS cycle {d['cycle_us']} us, cpu {d['cpu_pct']} %, i2c errors {d['i2c_errors']}, "
              f"sensors 0x{d['sensors']:04x}, armingFlags 0x{d['arming_flags']:08x}")
        print("  raw tail", st[13:].hex())
    g = req(s, MSP_RAW_GPS)
    if g:
        d = parse_raw_gps(g); print(f"RAW_GPS fix {FIX_NAMES.get(d['fix'], d['fix'])} sats {d['sats']} hdop {d['hdop']:.2f}")
    gs = req(s, MSP_GPSSTATISTICS)
    if gs:
        d = parse_gps_stats(gs)
        print(f"GPSSTATS lastMessageDt {d['gps_dt_ms']} ms errors {d['gps_errors']} timeouts {d['gps_timeouts']} "
              f"packets {d['gps_packets']} eph {d['eph_m']:.2f} m epv {d['epv_m']:.2f} m")
    for n in ["gps_ublox_use_galileo", "gps_ublox_use_glonass", "gps_ublox_use_beidou", "gps_dyn_model", "gps_ublox_nav_hz",
              "gps_min_sats", "init_gyro_cal", "inav_w_acc_bias", "inav_max_eph_epv", "inav_default_alt_sensor",
              "failsafe_procedure", "failsafe_delay", "failsafe_recovery_delay", "failsafe_off_delay", "failsafe_throttle",
              "motorstop_on_low", "msp_override_channels", "align_mag_roll", "align_mag_pitch", "align_mag_yaw", "align_mag",
              "align_board_roll", "align_board_pitch", "align_board_yaw", "ins_gravity_cmss", "reboot_character",
              "blackbox_rate_denom", "debug_mode", "fw_p_level", "nav_rth_altitude", "nav_rth_climb_first"]:
        r = setting(s, n)
        print(f"  set {n:26s} raw {r.hex() if r else r}")


def svinfo(s):
    p = req(s, MSP_GPSSVINFO)
    stub, sats = parse_svinfo(p)
    if stub is None:
        print("GPSSVINFO: no reply")
    elif stub:
        print("GPSSVINFO: INAV 8.0 compatibility stub (no per-satellite data over MSP).")
        print("  Per-satellite C/N0: CLI `gpssats` (disarmed; `exit` reboots the FC) or `gpspassthrough` + u-center.")
    else:
        print(f"GPSSVINFO: {len(sats)} channels")
        for svid, q, cno in sats:
            print(f"  sv {svid:3d}  quality 0x{q:02x}  C/N0 {cno:2d} dB-Hz")
        if sats:
            c = sorted(x[2] for x in sats if x[2] > 0)
            if c:
                print(f"  C/N0 mean {sum(c) / len(c):.1f}  median {c[len(c) // 2]}  max {c[-1]} dB-Hz over {len(c)} sats")
    print("coarse proxy (MSP):")
    g = req(s, MSP_RAW_GPS); gs = req(s, MSP_GPSSTATISTICS)
    if g:
        d = parse_raw_gps(g); print(f"  fix {FIX_NAMES.get(d['fix'], d['fix'])} sats {d['sats']} hdop {d['hdop']:.2f}")
    if gs:
        d = parse_gps_stats(gs); print(f"  eph {d['eph_m']:.2f} m epv {d['epv_m']:.2f} m errors {d['gps_errors']} "
                                       f"timeouts {d['gps_timeouts']}")


def watch(s, duration, interval, csv_path, label):
    fields = ["t_s", "label", "i2c_errors", "i2c_delta", "cycle_us", "cpu_pct", "armed", "fix", "sats", "hdop",
              "eph_m", "epv_m", "gps_errors", "gps_timeouts"]
    out = None
    if csv_path:
        f = open(csv_path, "a", newline="")
        out = csv.DictWriter(f, fieldnames=fields)
        if f.tell() == 0:
            out.writeheader()
    t0 = time.time(); i2c0 = None
    print("  t_s  i2c (Δ)   cyc  cpu arm fix sats  hdop   eph   epv  gpsErr  label")
    try:
        while duration == 0 or time.time() - t0 <= duration:
            row = {"t_s": round(time.time() - t0, 1), "label": label}
            st = req(s, MSP2_INAV_STATUS)
            if st:
                row.update(parse_status(st))
                if i2c0 is None:
                    i2c0 = row["i2c_errors"]
                row["i2c_delta"] = row["i2c_errors"] - i2c0
            g = req(s, MSP_RAW_GPS)
            if g:
                row.update(parse_raw_gps(g))
            gs = req(s, MSP_GPSSTATISTICS)
            if gs:
                row.update(parse_gps_stats(gs))
            print(f"{row['t_s']:5.1f} {row.get('i2c_errors', '?'):>4} ({row.get('i2c_delta', '?'):>3}) "
                  f"{row.get('cycle_us', '?'):>5} {row.get('cpu_pct', '?'):>3} {row.get('armed', '?'):>3} "
                  f"{row.get('fix', '?'):>3} {row.get('sats', '?'):>4} {row.get('hdop', 0):5.2f} "
                  f"{row.get('eph_m', 0):5.2f} {row.get('epv_m', 0):5.2f} {row.get('gps_errors', '?'):>7}  {label}",
                  flush=True)
            if out:
                out.writerow({k: row.get(k, "") for k in fields}); f.flush()
            time.sleep(interval)
    except KeyboardInterrupt:
        pass


def selftest():
    # status: cycle 500, i2c 50, sensors 0x002f, cpu 12, profile 0, arming flags ARMED
    st = struct.pack("<HHHHBI", 500, 50, 0x2F, 12, 0, ARMING_FLAG_ARMED) + b"\0" * 8
    d = parse_status(st)
    assert (d["cycle_us"], d["i2c_errors"], d["cpu_pct"], d["armed"]) == (500, 50, 12, 1), d
    assert parse_svinfo(bytes([1, 0, 0, 1, 1]))[0] is True                      # INAV 8.0 stub
    stub, sats = parse_svinfo(bytes([2, 0, 5, 0x1F, 41, 1, 12, 0x07, 33]))     # legacy layout, 2 channels
    assert stub is False and sats == [(5, 0x1F, 41), (12, 0x07, 33)], sats
    g = bytes([3, 13]) + struct.pack("<ii", 0, 0) + struct.pack("<hHHH", 10, 0, 0, 137)
    assert parse_raw_gps(g) == {"fix": 3, "sats": 13, "hdop": 1.37}
    gs = struct.pack("<HIIIHHH", 100, 2, 1, 999, 137, 176, 265)
    d = parse_gps_stats(gs)
    assert (d["eph_m"], d["epv_m"], d["gps_errors"]) == (1.76, 2.65, 2), d
    print("msp_bench_read selftest: PASS (status, svinfo stub + legacy, raw gps, gps stats)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port_pos", nargs="?", help=argparse.SUPPRESS)  # back-compat: positional port
    ap.add_argument("--port", default="auto")
    ap.add_argument("--svinfo", action="store_true")
    ap.add_argument("--watch", type=float, metavar="S", help="watch for S seconds (0 = until Ctrl-C)")
    ap.add_argument("--interval", type=float, default=1.0)
    ap.add_argument("--csv", help="append --watch rows to this CSV")
    ap.add_argument("--label", default="", help="tag for --watch rows")
    ap.add_argument("--selftest", action="store_true", help="check the payload parsers offline (no FC)")
    a = ap.parse_args()
    if a.selftest:
        selftest(); return
    port = a.port_pos or (find_fc_port() if a.port == "auto" else a.port)
    s = serial.Serial(port, 115200, timeout=0.05)
    time.sleep(0.2)
    print(f"port {port}")
    try:
        if a.svinfo:
            svinfo(s)
        elif a.watch is not None:
            watch(s, a.watch, a.interval, a.csv, a.label)
        else:
            dump(s)
    finally:
        s.close()


if __name__ == "__main__":
    main()
