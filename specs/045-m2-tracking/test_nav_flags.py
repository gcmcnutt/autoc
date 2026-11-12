#!/usr/bin/env python3
"""test_nav_flags.py — 045 T052: the blackbox fault flags of nav_span_scan.py (T053) on committed 11-08 excerpts.

Run (from anywhere; plain asserts, exits non-zero on a failure):
  python3 specs/045-m2-tracking/test_nav_flags.py

Fixtures (fixtures/, cut from flight-results/flight-20261108/blackbox_log_2026-11-08_160651.01.csv and its .gps.csv):
  bb_1108_span1.csv / .gps.csv  engaged span 1 (373.07–389.78 s in the log): the 2.30 s freeze, the 106 m XY reset on reacquire,
                                the 25 m Z step, LOW_SATS (5 sats) — must flag;
  bb_1108_span2.csv / .gps.csv  engaged span 2 (406.18–428.07 s): must flag nothing.
Each main excerpt keeps only the columns nav_span_scan.py reads (nav_span_scan.BB_COLS + the slow-frame failsafePhase) and every
row from 0.5 s before the first to 0.5 s after the last mspOverrideFlags == 2 row; the GPS excerpt keeps whole rows over the same
window. Regenerate (needs the full decode, MANIFEST § Preservation), which also checks that each excerpt flags exactly what the
full log flags for that span:
  python3 specs/045-m2-tracking/test_nav_flags.py --make-fixtures
"""
import csv
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import nav_span_scan as nss  # noqa: E402

FIX = os.path.join(HERE, "fixtures")
SPAN1, SPAN2 = os.path.join(FIX, "bb_1108_span1"), os.path.join(FIX, "bb_1108_span2")
SOURCE = os.path.join(nss.REPO, nss.KNOWN["1108"][0])
MARGIN = 0.5
TICK = 0.05


def args(*extra):
    return nss.build_parser().parse_args(["--blackbox", *extra])


def one_span(stem, *extra):
    spans, notes = nss.flag_log(stem, args(*extra))
    assert len(spans) == 1, f"{stem}: {len(spans)} engaged spans, expected 1"
    return spans[0], notes


def of(span, kind):
    return [e for e in span["events"] if e[0] == kind]


def near(x, y, tol=TICK / 2):
    return abs(x - y) <= tol


# ------------------------------------------------------------------------------------------------------------------- tests
def test_span1_flags_freeze_and_reset():
    s, notes = one_span(SPAN1)
    assert notes == [], notes
    (fr,) = of(s, "FROZEN")
    assert near(fr[3], 2.30) and near(fr[1], 7.75), f"freeze {fr}"
    (xy,) = of(s, "STEP_XY")
    assert 100 < float(xy[4]) < 110, f"XY reset magnitude {xy}"
    assert near(xy[1], fr[2]), f"the XY reset is on reacquire, at the freeze's end: {xy} vs {fr}"
    (z,) = of(s, "STEP_Z")
    assert 20 < float(z[4]) < 30 and near(z[1], 15.90), f"Z step {z}"
    (inv,) = of(s, "INVALID_XY")
    assert inv[1] <= fr[1] and inv[2] >= fr[2] and inv[4] == "1000", f"INVALID_XY must cover the freeze: {inv}"
    assert of(s, "UNTRUSTED_XY"), "navFlags position-trusted bit clears with the freeze"
    low = of(s, "LOW_SATS")
    assert low and all(e[4] == "5" for e in low), f"LOW_SATS {low}"
    for kind in ("INVALID_Z", "UNTRUSTED_Z", "NO_FIX", "GPS_GAP", "FAILSAFE"):
        assert not of(s, kind), f"unexpected {kind}: {of(s, kind)}"


def test_span2_flags_nothing():
    s, notes = one_span(SPAN2)
    assert notes == [], notes
    assert s["events"] == [], s["events"]
    assert s["clean"].all()


def test_eph_cap_is_at_or_above():
    # INAV holds est.eph just over the cap once invalid; the log rounds it to the cap (1000 cm on 11-08)
    assert of(one_span(SPAN1, "--eph-cap-cm", "1000")[0], "INVALID_XY")
    s = one_span(SPAN1, "--eph-cap-cm", "1001")[0]
    assert not of(s, "INVALID_XY") and of(s, "FROZEN") and of(s, "STEP_XY"), "the cap gates INVALID only"
    assert of(one_span(SPAN1, "--eph-cap-cm", "998")[0], "INVALID_Z"), "EPV peaks at 998 cm in span 1"


def test_frozen_needs_consecutive_ticks():
    assert of(one_span(SPAN1, "--frozen-min", "46")[0], "FROZEN")  # 2.30 s = 46 ticks
    assert not of(one_span(SPAN1, "--frozen-min", "47")[0], "FROZEN")


def write_csv(path, header, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def test_gps_gap_and_failsafe_synthetic():
    """8 s at 240 Hz, engaged 1–7 s, 10 m/s north on a clean estimate. GPS at 10 Hz with two 0.4 s holes, the second holding a
    re-written copy of the frame before it (INAV's periodic home + GPS frame), and RX_LOSS_DETECTED for 0.5 s."""
    with tempfile.TemporaryDirectory() as d:
        stem = os.path.join(d, "synthetic")
        rows = []
        for k in range(8 * 240):
            t = k / 240
            msp = 2 if 1.0 <= t < 7.0 else 1
            phase = "RX_LOSS_DETECTED" if 4.01 <= t < 4.51 else "IDLE"  # off the tick grid: ticks 4.05–4.50 s
            rows.append([round(t * 1e6), round(1000 * t), 0, 5000, 1000, 0, 0, 300, 500, 37, msp, phase])
        write_csv(stem + ".csv", nss.BB_COLS + [nss.FAILSAFE_COL], rows)
        g, prev = [], None
        for k in range(80):
            t = 0.05 + 0.1 * k
            if 2.0 < t < 2.3 or 5.1 < t < 5.4:  # drops 2.05–2.25 and 5.15–5.35: holes 1.95–2.35 and 5.05–5.45 s
                if near(t, 5.25, 0.01):  # the copy: same content as the last real frame, new time
                    g.append([round(t * 1e6)] + prev[1:])
                continue
            prev = [round(t * 1e6), 2, 9, f"{37.4 + 1e-6 * k:.7f}", "-121.9951750", 10]
            g.append(prev)
        write_csv(stem + ".gps.csv", ["time (us)", "GPS_fixType", "GPS_numSat", "GPS_coord[0]", "GPS_coord[1]", "GPS_altitude"], g)
        s, notes = one_span(stem)
        assert notes == [], notes
        gaps = of(s, "GPS_GAP")
        assert len(gaps) == 2 and all(near(e[3], 0.4, 0.011) for e in gaps), f"gaps {gaps}"
        assert near(gaps[0][1], 0.95, 0.011) and near(gaps[1][1], 4.05, 0.011), f"gap starts {gaps}"
        (fs,) = of(s, "FAILSAFE")
        assert fs[4] == "RX_LOSS_DETECTED" and near(fs[1], 3.05) and near(fs[3], 0.5), f"failsafe {fs}"
        assert {e[0] for e in s["events"]} == {"GPS_GAP", "FAILSAFE"}, s["events"]
        os.remove(stem + ".gps.csv")
        assert any("GPS_GAP not checked" in n for n in one_span(stem)[1]), "a missing GPS CSV is reported, not ignored"


def test_span_mapping():
    bb = [(373.07, 16.71), (406.18, 21.89), (455.55, 21.06), (493.05, 21.24)]  # 11-08 blackbox spans (start, duration)
    xs = [("x.csv#1", 371.94, 16.98), ("x.csv#2", 405.09, 21.88), ("x.csv#3", 454.54, 21.78), ("x.csv#4", 492.08, 20.85)]
    res, problems = nss.match_spans(bb, xs, 1.0)
    assert problems == [] and [r[0] for r in res if r[1]] == ["x.csv#1", "x.csv#2", "x.csv#3", "x.csv#4"], (res, problems)
    res, problems = nss.match_spans(bb, [("x.csv#0", 360.0, 1.2)] + xs, 1.0)  # an extra short xiao span shifts the order
    assert problems and not any(r[1] for r in res), (res, problems)


def test_drop_arguments():
    with tempfile.TemporaryDirectory() as d:
        xiao = os.path.join(d, "flight_log_test.csv")  # span 1 as the xiao logged it on 11-08: 341 ticks from 371.94 s
        write_csv(xiao, ["span_id", "timestamp_ms"], [[1, 371940 + 50 * i] for i in range(341)])
        _, drops = nss.blackbox_mode(args("--bb", SPAN1 + ".csv", "--xiao", xiao))
        assert drops == ["flight_log_test.csv#1"], drops
        _, drops = nss.blackbox_mode(args("--bb", SPAN2 + ".csv", "--xiao", xiao))
        assert drops == [], drops


# --------------------------------------------------------------------------------------------------------------- fixtures
def make_fixtures():
    import numpy as np
    bb = nss.load_nav(SOURCE)
    spans = nss.bb_spans(bb["t"], bb["msp"], 3.0)
    keep = nss.BB_COLS + [nss.FAILSAFE_COL]
    os.makedirs(FIX, exist_ok=True)
    full, _ = nss.flag_log(SOURCE, args())
    for n, out in ((1, SPAN1), (2, SPAN2)):
        lo, hi = spans[n - 1][0] - MARGIN, spans[n - 1][1] + MARGIN
        for src, dst, cols in ((SOURCE + ".csv", out + ".csv", keep), (SOURCE + ".gps.csv", out + ".gps.csv", None)):
            with open(src) as f, open(dst, "w", newline="") as g:
                r, w = csv.reader(f), csv.writer(g, lineterminator="\n")
                h = [c.strip() for c in next(r)]
                ix = [h.index(c) for c in cols] if cols else list(range(len(h)))
                w.writerow([h[i] for i in ix])
                it = h.index("time (us)")
                for row in r:
                    try:
                        t = int(row[it]) / 1e6
                    except (ValueError, IndexError):
                        continue
                    if lo <= t <= hi:
                        w.writerow([row[i].strip() for i in ix])
            print(f"wrote {os.path.relpath(dst, nss.REPO)}: {os.path.getsize(dst) / 1024:.0f} KiB")
        s, _ = one_span(out)
        ref = full[n - 1]
        assert s["events"] == ref["events"] and np.array_equal(s["rh"], ref["rh"]) and np.array_equal(s["rv"], ref["rv"]), \
            f"span {n}: the excerpt does not reproduce the full log's flags"
        print(f"  span {n}: excerpt flags = full-log flags ({len(ref['events'])} events)")


def main():
    if sys.argv[1:] == ["--make-fixtures"]:
        make_fixtures()
        return 0
    if sys.argv[1:]:
        print(__doc__)
        return 2
    tests = [(k, v) for k, v in globals().items() if k.startswith("test_") and callable(v)]
    failed = 0
    for name, fn in tests:
        try:
            out = sys.stdout
            if name == "test_drop_arguments":
                sys.stdout = open(os.devnull, "w")  # blackbox_mode's report is not the test's output
            try:
                fn()
            finally:
                if sys.stdout is not out:
                    sys.stdout.close()
                    sys.stdout = out
            print(f"PASS {name}")
        except Exception as e:  # an assert, or a missing / extra event breaking an unpack
            failed += 1
            print(f"FAIL {name}: {type(e).__name__}: {e}")
    print(f"{len(tests) - failed}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
