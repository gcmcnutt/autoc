"""041 T009 — one source of truth for the control cadence, so tick-denominated
metrics are surfaced in SECONDS consistently across every report.

WHY THIS EXISTS
---------------
`avgMaxStreak` (and `maxLost`, and every closing-rate) is counted in TICKS. At
20 Hz a 5-second streak reads 100; at 10 Hz the same flight reads 50. Reporting
the raw count makes a cadence change look like a competence change — and 041
uses exactly these metrics to judge whether the M1 bake regressed (037 P-O11).

`pctInStreak` is NOT tick-denominated: it is a ratio of in-streak steps to total
steps, so it is already cadence-independent and must NOT be scaled. It is named
here only because the pair is quoted together and the asymmetry is easy to
forget.

WHERE THE CADENCE COMES FROM
----------------------------
The **log**, not the ini. `autoc` prints its full resolved config at startup
(`ControlIntervalMsec: 50`), so a run's log is self-describing. Reading the
current ini instead means re-analysing a historical 10 Hz run with today's 20 Hz
value and halving every duration silently — the exact class of error
Constitution VII exists to prevent.

There is deliberately NO silent default. A log with no cadence line raises.
"""

import re

_CTRL_MSEC = re.compile(r"ControlIntervalMsec[ =:]+(\d+)")


class CadenceUnknown(RuntimeError):
    """The log does not state its control cadence and no override was given."""


def ctrl_msec_from_log(path):
    """Return the ControlIntervalMsec the run recorded, or None if absent.

    Scans the whole file: the config print is near the top, but eval-mode logs
    and restarts can push it down, and a wrong-but-plausible answer is worse
    than a slow one.
    """
    try:
        with open(path, errors="replace") as f:
            for line in f:
                m = _CTRL_MSEC.search(line)
                if m:
                    return int(m.group(1))
    except OSError:
        return None
    return None


def tick_sec_from_log(path, override=None, label=None):
    """Seconds per tick for `path`.

    The log wins. `override` (a --tick-sec flag) is accepted only when the log
    is silent, or when it agrees; a disagreement raises rather than picking a
    winner, because either answer would silently rescale every duration in the
    report.
    """
    who = label or path
    msec = ctrl_msec_from_log(path)

    if msec is not None:
        from_log = msec / 1000.0
        if override is not None and abs(override - from_log) > 1e-9:
            raise CadenceUnknown(
                f"{who}: --tick-sec {override:g}s disagrees with the run's own "
                f"ControlIntervalMsec = {msec} ms ({from_log:g}s). The log is "
                f"authoritative; drop the flag, or you are rescaling every "
                f"duration in this report by {override / from_log:.2f}x."
            )
        return from_log

    if override is not None:
        return override

    raise CadenceUnknown(
        f"{who}: no 'ControlIntervalMsec' line found, so the control cadence is "
        f"unknown and tick-denominated metrics (avgMaxStreak, maxLost) cannot be "
        f"converted to seconds. Pass --tick-sec explicitly (0.05 = 20 Hz, "
        f"0.10 = 10 Hz for pre-037 runs). Refusing to guess."
    )


def ticks_to_sec(values, tick_sec):
    """Scale a tick-denominated series to seconds. Works for lists or arrays."""
    try:
        return values * tick_sec          # numpy array
    except TypeError:
        return [v * tick_sec for v in values]
