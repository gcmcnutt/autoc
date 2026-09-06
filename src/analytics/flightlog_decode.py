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

FORMAT_VERSION = 5
MAGIC = 0x314C4641  # "AFL1" little-endian

# Record type bytes (flight_log_format.h RecordType)
T_PAD, T_FILE, T_ENGAGE, T_TICK, T_EVENT, T_SUMMARY, T_FLIGHT = 0, 1, 2, 3, 4, 5, 6

EVENT_NAMES = {
    1: "ARM", 2: "DISARM", 3: "ENGAGE", 4: "DISENGAGE",
    5: "FETCH_TIMEOUT", 6: "LOG_DROP", 7: "FLASH_FULL",
    # v3 arm→disarm self-containment
    8: "DISENGAGE_REASON",  # value: DisengageReason enum (REASON_NAMES)
    9: "FAILSAFE",          # value: 1 entered / 0 cleared
    10: "SERVO_SWITCH",     # value: 1 active / 0 released
    11: "INAV_CLOCK",       # value: INAV ms at this record's timestamp_ms (xiao ms)
}

# kEventDisengageReason values (flight_log_format.h DisengageReason)
REASON_NAMES = {
    0: "unknown", 1: "servo switch", 2: "failsafe", 3: "disarmed",
    4: "timeout", 5: "path complete", 6: "MSP state failure",
    7: "missing local state", 8: "autoc cancelled",
}

# v4 (041 P5-3): the NN input block grew 37 -> 45. MUST track PathgenInput in
# include/autoc/nn/nn_inputs.h and kNumInputs in xiao/include/flight_log_format.h
# — INPUT_NAMES below is the readable copy of that enum and is length-checked.
NUM_INPUTS, NUM_OUTPUTS = 45, 3
NUM_SCALED = NUM_INPUTS + NUM_OUTPUTS + 10  # v2: + pos[3], vel[3], rabbit[3]; v5: + step_score
# Telemetry scale-table bases, DERIVED (were literal 40/43/46, sized for 37).
SCALE_POS_BASE = NUM_INPUTS + NUM_OUTPUTS
SCALE_VEL_BASE = SCALE_POS_BASE + 3
SCALE_RABBIT_BASE = SCALE_VEL_BASE + 3
SCALE_STEP_SCORE_BASE = SCALE_RABBIT_BASE + 3  # v5

# Wire structs — little-endian, packed (raw-ok: hardware byte layout; this
# decode boundary is where values return to float domain).
FILE_HDR = struct.Struct(f"<BIB8s8s96sH6f{NUM_SCALED}fI")  # v5: +6 cone floats
ENGAGE_HDR = struct.Struct("<BIH3fffh")       # 29 B
TICK_REC = struct.Struct(f"<BIH{NUM_INPUTS}h3h3h3h3hBb3HBh")  # v5: +step_score
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
       # 041: five channels inserted BEFORE dist_to_boundary — the reason the
       # v3 table silently mis-labelled everything from slot 33 on.
       "accel_x", "accel_y", "accel_z",
       "specific_energy", "boundary_closure_rate",
       "dist_to_boundary", "inward_body_x", "inward_body_y", "inward_body_z",
       "score_grad_x", "score_grad_y", "score_grad_z"]
)
assert len(INPUT_NAMES) == NUM_INPUTS, (
    f"INPUT_NAMES has {len(INPUT_NAMES)} entries but the wire carries "
    f"{NUM_INPUTS}; every column after the mismatch would be mislabelled")
# 043 (2026-09-01) — ORDER FIX: the NN output vector is PITCH-first.
# tools/dmp_dump.cc names the same three slots "out_pt,out_rl,out_th", and the
# xiao maps slot0 -> 1500 - x*500 (the INVERTED pitch transform per
# docs/COORDINATE_CONVENTIONS.md) and slot1 -> 1500 + x*500 (roll). This file
# had them swapped, so every decoded log reported roll as pitch and vice versa.
# Caught on the 043 bench run: out_roll correlated -1.0000 with rc_pitch.
OUTPUT_NAMES = ["out_pitch", "out_roll", "out_throttle"]
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
            (_, magic, version, fw_id, wt_id, program, tick_ms), rest = f[:7], f[7:]
            cone_vals = rest[:6]                      # v5 fitness cone
            scale_vals, crc = rest[6:6 + NUM_SCALED], rest[6 + NUM_SCALED]
            if magic != MAGIC:
                fail(f"bad magic 0x{magic:08x} (want 0x{MAGIC:08x}) — not a flight log")
            if version != FORMAT_VERSION:
                fail(f"format_version {version} not supported (decoder is v{FORMAT_VERSION}) "
                     f"— refusing best-effort parse")
            # CRC over the scale floats exactly as stored (little-endian bytes)
            # program[96] then tick_ms(2) then the 6 v5 cone floats precede scales
            _scale_off = 1 + 4 + 1 + 8 + 8 + 96 + 2 + 24
            scale_bytes = raw[_scale_off:_scale_off + 4 * NUM_SCALED]
            if zlib.crc32(scale_bytes) & 0xFFFFFFFF != crc:
                fail("scale_table_crc mismatch — header corrupt; refusing to decode")
            scales = scale_vals
            header = {
                "format_version": version,
                "firmware_id": fw_id.hex(),
                "weight_id": wt_id.hex(),
                "program": program.split(b"\x00", 1)[0].decode(errors="replace"),
                "tick_ms": tick_ms,
                # v5 — the cone the flight was scored against, from the constants
                # nn2cpp baked into the firmware. Replay uses THESE, never the
                # live autoc.ini, which may have drifted since the flight.
                "cone": {
                    "dist_scale_behind": cone_vals[0],
                    "dist_scale_ahead": cone_vals[1],
                    "cone_angle_deg": cone_vals[2],
                    "streak_threshold": cone_vals[3],
                    "streak_ramp_sec": cone_vals[4],
                    "streak_multiplier_max": cone_vals[5],
                },
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
            # DERIVED offsets. These were literal 40/43/52/54/57, sized for a
            # 37-slot input block: at 45 slots they read the outputs as inputs
            # and the telemetry as outputs, so pos/vel/rabbit decoded to frozen
            # nonsense (caught by the synthetic-v4 round-trip, 2026-08-22).
            _o = 3 + NUM_INPUTS                  # first output field
            _t = _o + NUM_OUTPUTS                # pos[3], vel[3], rabbit[3]
            q_in = f[3:_o]
            q_out = f[_o:_t]
            q_telem = f[_t:_t + 9]
            reset, path_idx = f[_t + 9], f[_t + 10]
            rc = f[_t + 11:_t + 14]
            valid = f[_t + 14]
            step_score_q = f[_t + 15]  # v5
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
                # v5 — the score the FIRMWARE computed for this tick, not a
                # desktop re-derivation. The segment tangent it used is not in
                # the log, so reconstructing flips the in-streak call on ~1% of
                # ticks. In pathgen/M1 the rabbit is ground truth, so this is
                # the true score; a tracker/M2 flight would be scoring against
                # an ESTIMATE and must not be pooled with it.
                "step_score": step_score_q / scales[SCALE_STEP_SCORE_BASE],
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
                "pos_raw_n": f[2] / scales[SCALE_POS_BASE],
                "pos_raw_e": f[3] / scales[SCALE_POS_BASE + 1],
                "pos_raw_d": f[4] / scales[SCALE_POS_BASE + 2],
                "vel_n": f[5] / scales[SCALE_VEL_BASE],
                "vel_e": f[6] / scales[SCALE_VEL_BASE + 1],
                "vel_d": f[7] / scales[SCALE_VEL_BASE + 2],
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
    p(f"  program: {header['program']}")
    for ev in events:
        annot = ""
        if ev["code"] == 8:  # DISENGAGE_REASON
            annot = f"  ({REASON_NAMES.get(ev['value'], '?')})"
        p(f"  event t={ev['timestamp_ms']:>9} {ev['name']:<16} value={ev['value']}{annot}")
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
    # ⚠️ Units, stated every run. The NN input columns are POST-SCALE NN units,
    # exactly as the policy consumed them -- NOT physical. Reading one as
    # physical is the bug that made the renderer draw every chase vector 26x too
    # short (041, 2026-08-22). pos/vel/rabbit ARE physical (m, m/s).
    p("  NOTE: NN input columns are POST-SCALE NN units, not physical -- "
      "dist = m/26, gyro = rad/s/6, accel = g/8, airspeed & bClR = m/s/13, "
      "closing_rate = m/s/16. pos/vel/rabbit ARE physical.")
    p(f"  totals: {len(spans)} spans, {sum(len(s['ticks']) for s in spans)} ticks, "
      f"{total_gaps} gaps, {len(flight_states)} flight-state breadcrumbs")


def write_csv(spans, path):
    import csv
    cols = (["span_id", "timestamp_ms", "tick_counter"] + INPUT_NAMES + OUTPUT_NAMES
            + TELEM_NAMES
            + ["recurrent_reset", "path_index", "rc_roll", "rc_pitch", "rc_throttle",
               "state_valid", "step_score"])
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
