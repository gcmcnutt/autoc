#!/usr/bin/env python3
"""039 US3 (T015) — desktop decoder for the versioned binary flight log (v5 / v6).

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

# v6 (044 FR-016): the header carries the input layout — counts, layout version,
# layout signature and the ordered slot-name table — and every variable-width part
# of the stream is sized FROM THE FILE. v5 (the 043 flights) is still read, from
# the frozen description below. Anything else is refused (Constitution V).
SUPPORTED_VERSIONS = (5, 6)
MAGIC = 0x314C4641  # "AFL1" little-endian
SLOT_NAME_LEN = 24   # v6 header slot-name field (kNNSlotNameLen, include/autoc/nn/nn_inputs.h)

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

# FROZEN description of the v5 format (043 flights, input layout v3): its slot
# names in order, exactly as PathgenInput stood then. v5 files carry no name
# table, so this list IS the v5 contract — it never changes; v6 files describe
# themselves. Columns are the lowercased enum names, same as v6.
V5_INPUT_NAMES = (
    [f"target_{a}_{t}" for a in "xyz" for t in ("tm5", "tm4", "tm3", "tm2", "tm1", "now")]
    + [f"dist_{t}" for t in ("tm5", "tm4", "tm3", "tm2", "tm1", "now")]
    + ["closing_rate", "quat_w", "quat_x", "quat_y", "quat_z", "airspeed",
       "gyro_p", "gyro_q", "gyro_r", "accel_x", "accel_y", "accel_z",
       "specific_energy", "boundary_closure_rate", "dist_to_boundary",
       "inward_body_x", "inward_body_y", "inward_body_z",
       "score_grad_x", "score_grad_y", "score_grad_z"]
)
V5_NUM_OUTPUTS = 3

# Fixed-width records (independent of the layout).
ENGAGE_HDR = struct.Struct("<BIH3fffh")       # 29 B
EVENT_REC = struct.Struct("<BIBI")            # 10 B
SUMMARY_REC = struct.Struct("<BIH5I13I3I2II")  # 103 B
FLIGHT_REC = struct.Struct("<BI3h3h4h")       # 25 B armed-not-engaged breadcrumb
FILE_PREFIX = struct.Struct("<BIB")           # type, magic, format_version
V6_LAYOUT = struct.Struct("<HHHQ")            # num_inputs, num_outputs, layout_version, signature


class Layout:
    """Everything whose width depends on the input layout, built from the header."""

    def __init__(self, version, names, n_out, layout_version=None, signature=None):
        self.version, self.names, self.n_in, self.n_out = version, list(names), len(names), n_out
        self.layout_version, self.signature = layout_version, signature
        self.n_scaled = self.n_in + self.n_out + 10  # + pos[3], vel[3], rabbit[3], step_score
        self.scale_pos_base = self.n_in + self.n_out
        self.scale_vel_base = self.scale_pos_base + 3
        self.scale_rabbit_base = self.scale_vel_base + 3
        self.scale_step_score_base = self.scale_rabbit_base + 3
        self.prefix_size = FILE_PREFIX.size + (V6_LAYOUT.size + SLOT_NAME_LEN * self.n_in if version >= 6 else 0)
        self.file_tail = struct.Struct(f"<8s8s96sH6f{self.n_scaled}fI")
        self.file_size = self.prefix_size + self.file_tail.size
        self.tick = struct.Struct(f"<BIH{self.n_in}h{self.n_out}h3h3h3hBb3HBh")
        self.record_size = {T_FILE: self.file_size, T_ENGAGE: ENGAGE_HDR.size, T_TICK: self.tick.size,
                            T_EVENT: EVENT_REC.size, T_SUMMARY: SUMMARY_REC.size, T_FLIGHT: FLIGHT_REC.size}
        self.columns = [n.lower() for n in self.names]


def layout_signature(names, layout_version):
    """FNV-1a 64 over each slot name + NUL, then the layout version (4 B LE) —
    the same hash as nnLayoutSignature (nn_inputs.h) / headerLayoutSignature."""
    h = 0xcbf29ce484222325
    for n in names:
        for b in n.encode():
            h = ((h ^ b) * 0x100000001b3) & 0xFFFFFFFFFFFFFFFF
        h = (h * 0x100000001b3) & 0xFFFFFFFFFFFFFFFF
    for k in range(4):
        h = ((h ^ ((layout_version >> (8 * k)) & 0xFF)) * 0x100000001b3) & 0xFFFFFFFFFFFFFFFF
    return h


def read_layout(blob, pos):
    """Parse the FileHeader's layout part at `pos` → Layout (loud-fails on anything unknown)."""
    if pos + FILE_PREFIX.size > len(blob):
        fail("truncated FileHeader")
    _, magic, version = FILE_PREFIX.unpack_from(blob, pos)
    if magic != MAGIC:
        fail(f"bad magic 0x{magic:08x} (want 0x{MAGIC:08x}) — not a flight log")
    if version not in SUPPORTED_VERSIONS:
        fail(f"format_version {version} not supported (decoder reads v{SUPPORTED_VERSIONS}) "
             f"— refusing best-effort parse")
    if version == 5:
        return Layout(5, V5_INPUT_NAMES, V5_NUM_OUTPUTS)
    p = pos + FILE_PREFIX.size
    if p + V6_LAYOUT.size > len(blob):
        fail("truncated v6 FileHeader layout block")
    n_in, n_out, lver, sig = V6_LAYOUT.unpack_from(blob, p)
    p += V6_LAYOUT.size
    if p + SLOT_NAME_LEN * n_in > len(blob):
        fail("truncated v6 slot-name table")
    names = [blob[p + SLOT_NAME_LEN * i:p + SLOT_NAME_LEN * (i + 1)].split(b"\x00", 1)[0].decode()
             for i in range(n_in)]
    if layout_signature(names, lver) != sig:
        fail(f"v6 header slot-name table does not hash to its layout signature {sig:016x} — "
             f"header corrupt; refusing to build columns from it")
    return Layout(6, names, n_out, lver, sig)


# 043 (2026-09-01) — ORDER FIX: the NN output vector is PITCH-first.
# tools/dmp_dump.cc names the same three slots "out_pt,out_rl,out_th", and the
# xiao maps slot0 -> 1500 - x*500 (the INVERTED pitch transform per
# docs/COORDINATE_CONVENTIONS.md) and slot1 -> 1500 + x*500 (roll). This file
# had them swapped, so every decoded log reported roll as pitch and vice versa.
# Caught on the 043 bench run: out_roll correlated -1.0000 with rc_pitch.
OUTPUT_NAMES = ["out_pitch", "out_roll", "out_throttle"]
TELEM_NAMES = ["pos_n", "pos_e", "pos_d", "vel_n", "vel_e", "vel_d",
               "rabbit_n", "rabbit_e", "rabbit_d"]  # v2, the scale slots after the outputs

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

    layout = None
    while pos < n:
        t = blob[pos]
        if t == T_PAD:
            pos += 1
            continue
        if layout is None:
            if t != T_FILE:
                fail(f"record type 0x{t:02x} at offset {pos} before the FileHeader — stream corrupt")
            layout = read_layout(blob, pos)
        size = layout.record_size.get(t)
        if size is None:
            fail(f"unknown record type 0x{t:02x} at offset {pos} — "
                 f"corrupt stream or format mismatch (log v{layout.version})")
        if pos + size > n:
            fail(f"truncated record (type 0x{t:02x}) at offset {pos}: "
                 f"needs {size} B, {n - pos} remain")
        raw = blob[pos:pos + size]
        pos += size

        if t == T_FILE:
            f = layout.file_tail.unpack(raw[layout.prefix_size:])
            (fw_id, wt_id, program, tick_ms), rest = f[:4], f[4:]
            cone_vals = rest[:6]                      # v5 fitness cone
            scale_vals, crc = rest[6:6 + layout.n_scaled], rest[6 + layout.n_scaled]
            # CRC over the scale floats exactly as stored (little-endian bytes):
            # ids, program[96], tick_ms(2) and the 6 cone floats precede them.
            _scale_off = layout.prefix_size + 8 + 8 + 96 + 2 + 24
            scale_bytes = raw[_scale_off:_scale_off + 4 * layout.n_scaled]
            if zlib.crc32(scale_bytes) & 0xFFFFFFFF != crc:
                fail("scale_table_crc mismatch — header corrupt; refusing to decode")
            scales = scale_vals
            header = {
                "format_version": layout.version,
                "num_inputs": layout.n_in,
                "num_outputs": layout.n_out,
                "layout_version": layout.layout_version,
                "layout_signature": (f"{layout.signature:016x}" if layout.signature is not None
                                     else "(v5: frozen layout v3)"),
                "input_columns": layout.columns,
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
            f = layout.tick.unpack(raw)
            ts, counter = f[1], f[2]
            # Offsets from the FILE's layout (v6 header / frozen v5 description).
            _o = 3 + layout.n_in                 # first output field
            _t = _o + layout.n_out               # pos[3], vel[3], rabbit[3]
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
            for i, name in enumerate(layout.columns):
                row[name] = q_in[i] / scales[i]
            for i, name in enumerate(OUTPUT_NAMES):
                row[name] = q_out[i] / scales[layout.n_in + i]
            for i, name in enumerate(TELEM_NAMES):
                row[name] = q_telem[i] / scales[layout.scale_pos_base + i]
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
                "step_score": step_score_q / scales[layout.scale_step_score_base],
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
                "pos_raw_n": f[2] / scales[layout.scale_pos_base],
                "pos_raw_e": f[3] / scales[layout.scale_pos_base + 1],
                "pos_raw_d": f[4] / scales[layout.scale_pos_base + 2],
                "vel_n": f[5] / scales[layout.scale_vel_base],
                "vel_e": f[6] / scales[layout.scale_vel_base + 1],
                "vel_d": f[7] / scales[layout.scale_vel_base + 2],
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
    p(f"  layout: {header['num_inputs']} inputs / {header['num_outputs']} outputs, "
      f"signature {header['layout_signature']}")
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


def write_csv(header, spans, path):
    import csv
    cols = (["span_id", "timestamp_ms", "tick_counter"] + header["input_columns"] + OUTPUT_NAMES
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
    ap.add_argument("binfile", nargs="+", help="downloaded flight_NNN.bin (several with --check)")
    ap.add_argument("-o", "--output", help="tick CSV output path")
    ap.add_argument("--flightpath", help="optional CSV of armed-not-engaged breadcrumbs (raw frame)")
    ap.add_argument("--check", action="store_true",
                    help="044 FR-016 ctest mode: decode every file by its header alone, "
                         "require at least one span with ticks, print one line each")
    args = ap.parse_args()

    if args.check:
        for path in args.binfile:
            with open(path, "rb") as fh:
                header, spans, events, warnings, flight_states = decode(fh.read())
            nt = sum(len(s["ticks"]) for s in spans)
            if nt == 0:
                fail(f"{path}: decoded but has no ticks")
            print(f"ok {path}: v{header['format_version']}, {header['num_inputs']} inputs "
                  f"({header['input_columns'][0]} .. {header['input_columns'][-1]}), "
                  f"signature {header['layout_signature']}, {len(spans)} spans, {nt} ticks")
        return
    if len(args.binfile) != 1:
        fail("one flight log at a time (several only with --check)")
    with open(args.binfile[0], "rb") as fh:
        blob = fh.read()
    header, spans, events, warnings, flight_states = decode(blob)
    report(header, spans, events, warnings, flight_states)
    if args.output:
        write_csv(header, spans, args.output)
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
