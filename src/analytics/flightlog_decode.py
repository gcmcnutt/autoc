#!/usr/bin/env python3
"""039 US3 (T015) — desktop decoder for the versioned binary flight log (v1).

Implements specs/039-xiao-20hz-flight/contracts/flight-log-format.md against
the wire format defined in xiao/include/flight_log_format.h (the single
shared definition). Reads a downloaded flight_NNN.bin, reconstructs float
fields via the header-carried CRC-verified scale table, and emits a CSV
consumable by the sim-vs-real per-axis tooling.

Loud-fail policy (Constitution V): unknown magic/version, scale-CRC mismatch,
unknown record type, or a truncated record ⇒ non-zero exit with a clear
message — never a best-effort parse. tick_counter gaps and drop counts are
REPORTED (never interpolated).

Usage:
  flightlog_decode.py flight_001.bin -o flight_ticks.csv
  flightlog_decode.py flight_001.bin              # report only, no CSV
"""

import argparse
import struct
import sys
import zlib

FORMAT_VERSION = 2
MAGIC = 0x314C4641  # "AFL1" little-endian

# Record type bytes (flight_log_format.h RecordType)
T_PAD, T_FILE, T_ENGAGE, T_TICK, T_EVENT, T_SUMMARY, T_FLIGHT = 0, 1, 2, 3, 4, 5, 6

EVENT_NAMES = {
    1: "ARM", 2: "DISARM", 3: "ENGAGE", 4: "DISENGAGE",
    5: "FETCH_TIMEOUT", 6: "LOG_DROP", 7: "FLASH_FULL",
}

NUM_INPUTS, NUM_OUTPUTS = 37, 3
NUM_SCALED = NUM_INPUTS + NUM_OUTPUTS + 9  # v2: + pos[3], vel[3], rabbit[3]

# Wire structs — little-endian, packed (raw-ok: hardware byte layout; this
# decode boundary is where values return to float domain).
FILE_HDR = struct.Struct("<BIB8s8sH49fI")     # 224 B (v2: 49 scale entries)
ENGAGE_HDR = struct.Struct("<BIH3fffh")       # 29 B
TICK_REC = struct.Struct("<BIH37h3h3h3h3hBb3HB")  # 114 B (v2: +pos/vel/rabbit)
EVENT_REC = struct.Struct("<BIBI")            # 10 B
SUMMARY_REC = struct.Struct("<BIH5I13I3I2II")  # 103 B
FLIGHT_REC = struct.Struct("<BI3h3h4h")       # 25 B armed-not-engaged breadcrumb

RECORD_SIZE = {
    T_FILE: FILE_HDR.size,
    T_ENGAGE: ENGAGE_HDR.size,
    T_TICK: TICK_REC.size,
    T_EVENT: EVENT_REC.size,
    T_SUMMARY: SUMMARY_REC.size,
    T_FLIGHT: FLIGHT_REC.size,
}

# Column names: PathgenInput slot order (autoc/nn/nn_inputs.h enum names).
INPUT_NAMES = (
    [f"target_x_{i}" for i in range(6)]
    + [f"target_y_{i}" for i in range(6)]
    + [f"target_z_{i}" for i in range(6)]
    + [f"dist_{i}" for i in range(6)]
    + ["closing_rate", "quat_w", "quat_x", "quat_y", "quat_z", "airspeed",
       "gyro_p", "gyro_q", "gyro_r",
       "dist_to_boundary", "inward_body_x", "inward_body_y", "inward_body_z"]
)
OUTPUT_NAMES = ["out_roll", "out_pitch", "out_throttle"]
TELEM_NAMES = ["pos_n", "pos_e", "pos_d", "vel_n", "vel_e", "vel_d",
               "rabbit_n", "rabbit_e", "rabbit_d"]  # v2, scale slots 40..48

SUMMARY_FIELDS = [
    "ticks", "overruns", "resyncs", "max_late_ms", "total_late_ms",
    "samples",
    "fetch_min_us", "fetch_avg_us", "fetch_max_us",
    "eval_min_us", "eval_avg_us", "eval_max_us",
    "send_min_us", "send_avg_us", "send_max_us",
    "total_min_us", "total_avg_us", "total_max_us",
    "interval_min_us", "interval_avg_us", "interval_max_us",
    "ticks_logged", "ticks_dropped", "dwt_eval_cycles",
]


def fail(msg):
    print(f"ERROR: {msg}", file=sys.stderr)
    sys.exit(1)


def decode(blob):
    """Walk the record stream. Returns (header, spans, events, warnings).

    spans = list of dicts: {engage: {...}, ticks: [row...], summary: {...}}
    A tick row is a dict keyed by CSV column name.
    """
    pos = 0
    n = len(blob)
    header = None
    scales = None
    spans = []
    events = []
    warnings = []
    flight_states = []  # armed-not-engaged breadcrumbs (raw INAV frame)
    current = None  # span accumulator

    while pos < n:
        t = blob[pos]
        if t == T_PAD:
            pos += 1
            continue
        size = RECORD_SIZE.get(t)
        if size is None:
            fail(f"unknown record type 0x{t:02x} at offset {pos} — "
                 f"corrupt stream or format mismatch (decoder is v{FORMAT_VERSION})")
        if pos + size > n:
            fail(f"truncated record (type 0x{t:02x}) at offset {pos}: "
                 f"needs {size} B, {n - pos} remain")
        raw = blob[pos:pos + size]
        pos += size

        if t == T_FILE:
            f = FILE_HDR.unpack(raw)
            (_, magic, version, fw_id, wt_id, tick_ms), rest = f[:6], f[6:]
            scale_vals, crc = rest[:NUM_SCALED], rest[NUM_SCALED]
            if magic != MAGIC:
                fail(f"bad magic 0x{magic:08x} (want 0x{MAGIC:08x}) — not a flight log")
            if version != FORMAT_VERSION:
                fail(f"format_version {version} not supported (decoder is v{FORMAT_VERSION}) "
                     f"— refusing best-effort parse")
            # CRC over the scale floats exactly as stored (little-endian bytes)
            scale_bytes = raw[24:24 + 4 * NUM_SCALED]  # 49 floats in v2
            if zlib.crc32(scale_bytes) & 0xFFFFFFFF != crc:
                fail("scale_table_crc mismatch — header corrupt; refusing to decode")
            scales = scale_vals
            header = {
                "format_version": version,
                "firmware_id": fw_id.hex(),
                "weight_id": wt_id.hex(),
                "tick_ms": tick_ms,
            }

        elif t == T_ENGAGE:
            if header is None:
                fail("EngageHeader before FileHeader — stream corrupt")
            f = ENGAGE_HDR.unpack(raw)
            current = {
                "engage": {
                    "engage_timestamp_ms": f[1],
                    "span_id": f[2],
                    "origin_n": f[3], "origin_e": f[4], "origin_d": f[5],
                    "floor_z_ned": f[6], "ceiling_z_ned": f[7],
                    "path_index": f[8],
                },
                "ticks": [],
                "summary": None,
            }
            spans.append(current)

        elif t == T_TICK:
            if scales is None:
                fail("TickRecord before FileHeader — stream corrupt")
            f = TICK_REC.unpack(raw)
            ts, counter = f[1], f[2]
            q_in = f[3:3 + NUM_INPUTS]
            q_out = f[40:43]
            q_telem = f[43:52]  # pos[3], vel[3], rabbit[3]
            reset, path_idx = f[52], f[53]
            rc = f[54:57]
            valid = f[57]
            row = {"timestamp_ms": ts, "tick_counter": counter}
            if current is not None:
                row["span_id"] = current["engage"]["span_id"]
            else:
                row["span_id"] = -1
                warnings.append(f"tick at offset {pos - size} outside any span")
            for i, name in enumerate(INPUT_NAMES):
                row[name] = q_in[i] / scales[i]
            for i, name in enumerate(OUTPUT_NAMES):
                row[name] = q_out[i] / scales[NUM_INPUTS + i]
            for i, name in enumerate(TELEM_NAMES):
                row[name] = q_telem[i] / scales[NUM_INPUTS + NUM_OUTPUTS + i]
            row.update({
                "recurrent_reset": reset, "path_index": path_idx,
                "rc_roll": rc[0], "rc_pitch": rc[1], "rc_throttle": rc[2],
                "state_valid": valid,
            })
            (current["ticks"] if current is not None else spans_orphan(spans)).append(row)

        elif t == T_EVENT:
            f = EVENT_REC.unpack(raw)
            events.append({
                "timestamp_ms": f[1],
                "code": f[2],
                "name": EVENT_NAMES.get(f[2], f"UNKNOWN_{f[2]}"),
                "value": f[3],
            })

        elif t == T_FLIGHT:
            if scales is None:
                fail("FlightState before FileHeader — stream corrupt")
            f = FLIGHT_REC.unpack(raw)
            flight_states.append({
                "timestamp_ms": f[1],
                "pos_raw_n": f[2] / scales[40], "pos_raw_e": f[3] / scales[41],
                "pos_raw_d": f[4] / scales[42],
                "vel_n": f[5] / scales[43], "vel_e": f[6] / scales[44],
                "vel_d": f[7] / scales[45],
                "quat_w": f[8] / 32767.0, "quat_x": f[9] / 32767.0,
                "quat_y": f[10] / 32767.0, "quat_z": f[11] / 32767.0,
            })

        elif t == T_SUMMARY:
            f = SUMMARY_REC.unpack(raw)
            summary = {"timestamp_ms": f[1], "span_id": f[2]}
            summary.update(dict(zip(SUMMARY_FIELDS, f[3:])))
            if current is not None and current["engage"]["span_id"] == f[2]:
                current["summary"] = summary
            else:
                warnings.append(f"span summary for span {f[2]} without matching engage")

    if header is None:
        fail("no FileHeader found — not a valid flight log (or empty download)")
    return header, spans, events, warnings, flight_states


def spans_orphan(spans):
    """Bucket for ticks outside a span (should not happen; kept visible)."""
    if not spans or spans[-1].get("engage", {}).get("span_id") != -1:
        spans.append({"engage": {"span_id": -1}, "ticks": [], "summary": None})
    return spans[-1]["ticks"]


def report(header, spans, events, warnings, flight_states, out=sys.stderr):
    p = lambda *a: print(*a, file=out)
    p(f"flight log v{header['format_version']}  tick={header['tick_ms']} ms  "
      f"firmware_id={header['firmware_id']}  weight_id={header['weight_id']}")
    for ev in events:
        p(f"  event t={ev['timestamp_ms']:>9} {ev['name']:<14} value={ev['value']}")
    total_gaps = 0
    for s in spans:
        e = s["engage"]
        ticks = s["ticks"]
        p(f"  span {e['span_id']}: {len(ticks)} ticks"
          + (f", path={e.get('path_index')}, origin_ned=({e.get('origin_n', 0):.1f},"
             f"{e.get('origin_e', 0):.1f},{e.get('origin_d', 0):.1f}), "
             f"floor_z={e.get('floor_z_ned', 0):.1f}, ceiling_z={e.get('ceiling_z_ned', 0):.1f}"
             if e["span_id"] != -1 else " [ORPHAN — ticks outside spans]"))
        # tick_counter gap report (contract: REPORTED, never interpolated)
        gaps = []
        for prev, cur in zip(ticks, ticks[1:]):
            d = cur["tick_counter"] - prev["tick_counter"]
            if d != 1:
                gaps.append((prev["tick_counter"], cur["tick_counter"]))
        if ticks and ticks[0]["tick_counter"] != 0:
            p(f"    WARNING: first tick_counter is {ticks[0]['tick_counter']}, expected 0")
        for a, b in gaps:
            p(f"    GAP: tick_counter {a} -> {b} ({b - a - 1} ticks missing)")
        total_gaps += len(gaps)
        if s["summary"]:
            m = s["summary"]
            p(f"    summary: ticks={m['ticks']} overruns={m['overruns']} resyncs={m['resyncs']} "
              f"maxLate={m['max_late_ms']}ms "
              f"fetch={m['fetch_min_us'] / 1e3:.1f}/{m['fetch_avg_us'] / 1e3:.1f}/{m['fetch_max_us'] / 1e3:.1f}ms "
              f"eval={m['eval_min_us'] / 1e3:.1f}/{m['eval_avg_us'] / 1e3:.1f}/{m['eval_max_us'] / 1e3:.1f}ms "
              f"send={m['send_min_us'] / 1e3:.1f}/{m['send_avg_us'] / 1e3:.1f}/{m['send_max_us'] / 1e3:.1f}ms "
              f"total={m['total_min_us'] / 1e3:.1f}/{m['total_avg_us'] / 1e3:.1f}/{m['total_max_us'] / 1e3:.1f}ms "
              f"logged={m['ticks_logged']} dropped={m['ticks_dropped']} "
              f"dwtEvalCycles={m['dwt_eval_cycles']}")
            if m["ticks_dropped"]:
                p(f"    WARNING: {m['ticks_dropped']} ticks dropped under buffer pressure")
        else:
            p("    WARNING: no span summary (disengage record missing?)")
    for w in warnings:
        p(f"  WARNING: {w}")
    p(f"  totals: {len(spans)} spans, {sum(len(s['ticks']) for s in spans)} ticks, "
      f"{total_gaps} gaps, {len(flight_states)} flight-state breadcrumbs")


def write_csv(spans, path):
    import csv
    cols = (["span_id", "timestamp_ms", "tick_counter"] + INPUT_NAMES + OUTPUT_NAMES
            + TELEM_NAMES
            + ["recurrent_reset", "path_index", "rc_roll", "rc_pitch", "rc_throttle",
               "state_valid"])
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=cols)
        w.writeheader()
        for s in spans:
            for row in s["ticks"]:
                w.writerow(row)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("binfile", help="downloaded flight_NNN.bin")
    ap.add_argument("-o", "--output", help="tick CSV output path")
    ap.add_argument("--flightpath", help="optional CSV of armed-not-engaged breadcrumbs (raw frame)")
    args = ap.parse_args()

    with open(args.binfile, "rb") as fh:
        blob = fh.read()
    header, spans, events, warnings, flight_states = decode(blob)
    report(header, spans, events, warnings, flight_states)
    if args.output:
        write_csv(spans, args.output)
        n = sum(len(s["ticks"]) for s in spans)
        print(f"wrote {n} ticks -> {args.output}", file=sys.stderr)
    if args.flightpath and flight_states:
        import csv as _csv
        with open(args.flightpath, "w", newline="") as fh:
            w = _csv.DictWriter(fh, fieldnames=list(flight_states[0].keys()))
            w.writeheader()
            w.writerows(flight_states)
        print(f"wrote {len(flight_states)} flight states -> {args.flightpath}", file=sys.stderr)


if __name__ == "__main__":
    main()
