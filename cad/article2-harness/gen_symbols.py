#!/usr/bin/env python3
"""
Generate article2-harness.kicad_sym -- the module symbols for the article 2 harness.

Provenance of every pin map (do not "tidy" these without re-checking the source;
pin errors are the expensive kind):

  FC   HGLRC Specter F722 Lite. Pad labels from the official HGLRC "SPECTER F722
       Lite FC" manual. The board has pads on BOTH FACES, so the symbol is split
       into two units:
         unit 1 = TOP face    (UART1, UART3, ELRS cluster, CAM/VTX, LED, BB-, VIN)
         unit 2 = BOTTOM face (the 4-in-1/ESC pad row BAT GND M1-M4 CR RX6, and
                               the GPS row 5V GND TX4 RX4 SDA SCL)
       Which MCU pin each pad lands on comes from the Betaflight unified config
       HGLR-HGLRCF722MINI.config, cross-checked against the INAV target
       src/main/target/HGLRCF722LITE/{target.h,target.c}.

  XIAO Seeed XIAO nRF52840, standard XIAO 14-pin form factor. Serial1 = D6 (TX)
       / D7 (RX). The 5V pin IS USB VBUS, wired straight through.

  GPS  Matek M10Q-5883. Pin order CONFIRMED by the operator 2026-10-12 from the
       module itself: the FORWARD connector (ahead of the front-facing arrow),
       read left to right, is
         1 red 5V | 2 yellow RX | 3 blue TX | 4 green CL | 5 white DA | 6 black GND
       Note RX comes BEFORE TX. An earlier draft of this file had them swapped.

  ESC  ~40 A BLHeli ESC with SBEC, drawn as its three real connectors:
         unit 1 = J_PACK   heavy pack leads, BAT+ / BAT-
         unit 2 = J_CTRL   the servo-style lead: SIG in, BEC +5V out, BEC GND out
         unit 3 = J_PHASE  the three motor phases
       BAT- and BEC_GND are separate PINS because they are separate WIRES; they
       are common only inside the ESC. That internal tie is the system star
       ground. See harness-spec.md section 2a.

Grid: every pin endpoint is a multiple of 1.27 mm.

Usage:  python3 gen_symbols.py > article2-harness.kicad_sym
"""

import sys

LIB_VERSION = "20241209"

PWR_IN, PWR_OUT, BIDI, INPUT, OUTPUT, PASSIVE = (
    "power_in", "power_out", "bidirectional", "input", "output", "passive")

ANGLE = {"L": 0, "R": 180, "T": 270, "B": 90}


def pin(etype, name, number, side, pos, half_w, half_h, length=2.54):
    a = ANGLE[side]
    if side == "L":
        x, y = -(half_w + length), pos
    elif side == "R":
        x, y = (half_w + length), pos
    elif side == "T":
        x, y = pos, (half_h + length)
    else:
        x, y = pos, -(half_h + length)
    for v in (x, y):
        assert abs(round(v / 1.27) * 1.27 - v) < 1e-6, f"{name} off 1.27 grid: {v}"
    return f"""      (pin {etype} line
        (at {x:g} {y:g} {a})
        (length {length:g})
        (name "{name}" (effects (font (size 1.27 1.27))))
        (number "{number}" (effects (font (size 1.27 1.27))))
      )"""


def spread(n, pitch=2.54):
    start = -((n - 1) * pitch) / 2.0
    return [round((start + i * pitch) / 1.27) * 1.27 for i in range(n)]


def place(spec, half_w, half_h, pitch_tb=5.08, pitch_lr=2.54):
    """spec: {side: [(etype, name, number), ...]} -> list of pin s-exprs."""
    out = []
    for side, items in spec.items():
        if not items:
            continue
        pitch = pitch_tb if side in "TB" else pitch_lr
        coords = spread(len(items), pitch)
        if side in "LR":
            coords = list(reversed(coords))   # first item listed sits at the top
        for (et, nm, num), c in zip(items, coords):
            out.append(pin(et, nm, num, side, c, half_w, half_h))
    return out


def unit_block(name, idx, half_w, half_h, pins, unit_name=None):
    uname = f'\n      (unit_name "{unit_name}")' if unit_name else ""
    body = "\n".join(pins)
    return f"""    (symbol "{name}_{idx}_1"{uname}
      (rectangle
        (start {-half_w:g} {half_h:g})
        (end {half_w:g} {-half_h:g})
        (stroke (width 0.254) (type default))
        (fill (type background))
      )
{body}
    )"""


def symbol(name, value, desc, units, ref="U"):
    """units: list of (half_w, half_h, pins, unit_name)."""
    top_h = max(u[1] for u in units)
    blocks = "\n".join(
        unit_block(name, i + 1, hw, hh, p, un)
        for i, (hw, hh, p, un) in enumerate(units))
    return f"""  (symbol "{name}"
    (exclude_from_sim no)
    (in_bom yes)
    (on_board yes)
    (property "Reference" "{ref}"
      (at 0 {top_h + 7.62:g} 0)
      (effects (font (size 1.27 1.27)))
    )
    (property "Value" "{value}"
      (at 0 {-(top_h + 7.62):g} 0)
      (effects (font (size 1.27 1.27)))
    )
    (property "Footprint" ""
      (at 0 0 0) (effects (font (size 1.27 1.27))) (hide yes)
    )
    (property "Datasheet" ""
      (at 0 0 0) (effects (font (size 1.27 1.27))) (hide yes)
    )
    (property "Description" "{desc}"
      (at 0 0 0) (effects (font (size 1.27 1.27))) (hide yes)
    )
{blocks}
  )"""


# ---------------------------------------------------------------- FC
def fc():
    HW1, HH1 = 33.02, 20.32
    u1 = place({
        "L": [(INPUT, "CAM", "CAM"), (PWR_OUT, "5V_T1", "5V_T1"),
              (PASSIVE, "VTX", "VTX"),
              (INPUT, "RX1", "RX1"), (OUTPUT, "TX1", "TX1"),
              (PWR_OUT, "GND_T1", "GND_T1")],
        "T": [(PASSIVE, "BAT_T", "BAT_T"), (PASSIVE, "VIN", "VIN"),
              (PWR_OUT, "5V_T2", "5V_T2"), (PASSIVE, "GND_T2", "GND_T2")],
        "R": [(INPUT, "RX2", "RX2"), (OUTPUT, "TX2", "TX2"),
              (PASSIVE, "ES-5V", "ES5V"), (PASSIVE, "5V_T3", "5V_T3"),
              (PASSIVE, "GND_T3", "GND_T3")],
        "B": [(OUTPUT, "LED", "LED"), (PASSIVE, "BB-", "BBM"),
              (PASSIVE, "5V_T4", "5V_T4"), (PASSIVE, "GND_T4", "GND_T4"),
              (PASSIVE, "BAT_T2", "BAT_T2"), (PASSIVE, "GND_T5", "GND_T5"),
              (INPUT, "RX3", "RX3"), (OUTPUT, "TX3", "TX3")],
    }, HW1, HH1)

    HW2, HH2 = 30.48, 15.24
    u2 = place({
        "T": [(PWR_IN, "BAT_B", "BAT_B"), (PWR_IN, "GND_B", "GND_B"),
              (OUTPUT, "M1", "M1"), (OUTPUT, "M2", "M2"),
              (OUTPUT, "M3", "M3"), (OUTPUT, "M4", "M4"),
              (INPUT, "CR", "CR"), (INPUT, "RX6", "RX6")],
        "B": [(PWR_OUT, "5V_B", "5V_B"), (PWR_OUT, "GND_B2", "GND_B2"),
              (OUTPUT, "TX4", "TX4"), (INPUT, "RX4", "RX4"),
              (BIDI, "SDA", "SDA"), (BIDI, "SCL", "SCL")],
    }, HW2, HH2)

    desc = ("HGLRC Specter F722 Lite. STM32F722, ICM42688P, DPS310, 16 MB flash, "
            "onboard ELRS on UART2 (short-connect pad), 5V 2A BEC, 2-6S. "
            "Unit 1 = TOP face pads, unit 2 = BOTTOM face pads. BAT_B feeds both "
            "the regulator and the vbat divider (ADC ch 1, PC2). Only M1-M4 have "
            "pads. UART5 has no pads; UART6 is RX-only.")
    return symbol("FC_HGLRC_SpecterF722Lite", "Specter F722 Lite", desc,
                  [(HW1, HH1, u1, "TOP face"), (HW2, HH2, u2, "BOTTOM face")])


# ---------------------------------------------------------------- XIAO
def xiao():
    HW, HH = 11.43, 11.43
    p = place({
        "L": [(BIDI, "D0/A0", "1"), (BIDI, "D1/A1", "2"), (BIDI, "D2/A2", "3"),
              (BIDI, "D3/A3", "4"), (BIDI, "D4/SDA", "5"), (BIDI, "D5/SCL", "6"),
              (BIDI, "D6/TX", "7")],
        "R": [(PWR_IN, "5V/VBUS", "14"), (PWR_IN, "GND", "13"),
              (PWR_OUT, "3V3", "12"), (BIDI, "D10/MOSI", "11"),
              (BIDI, "D9/MISO", "10"), (BIDI, "D8/SCK", "9"), (BIDI, "D7/RX", "8")],
    }, HW, HH)
    desc = ("Seeed XIAO nRF52840. Serial1 = D6 (TX) / D7 (RX). Powered from the FC "
            "TOP-face 5V pad. WARNING: the 5V pin IS USB VBUS, wired straight "
            "through - FC rail plus a powered USB cable backfeeds the host port.")
    return symbol("MCU_Seeed_XIAO_nRF52840", "XIAO nRF52840", desc,
                  [(HW, HH, p, None)])


# ---------------------------------------------------------------- GPS
def gps():
    HW, HH = 15.24, 10.16
    p = place({
        "L": [(PWR_IN, "5V (red)", "1"),
              (INPUT, "RX (yellow)", "2"),
              (OUTPUT, "TX (blue)", "3"),
              (BIDI, "CL (green)", "4"),
              (BIDI, "DA (white)", "5"),
              (PWR_IN, "GND (black)", "6")],
    }, HW, HH)
    desc = ("Matek M10Q-5883 GNSS + QMC5883L compass. 4-9 V, 13 mA, 9600 baud "
            "default. FORWARD connector (ahead of the front-facing arrow), left to "
            "right: red 5V, yellow RX, blue TX, green CL, white DA, black GND. "
            "RX is before TX - confirmed on the module, not inferred.")
    return symbol("GPS_Matek_M10Q_5883", "M10Q-5883", desc, [(HW, HH, p, None)])


# ---------------------------------------------------------------- ESC
def esc():
    HW1, HH1 = 13.97, 6.35
    u1 = place({"L": [(PWR_IN, "BAT+ (60A)", "1"), (PWR_IN, "BAT- (60A)", "2")]},
               HW1, HH1)
    HW2, HH2 = 13.97, 7.62
    u2 = place({"L": [(INPUT, "SIG", "3")],
                "R": [(PWR_OUT, "BEC+5V", "4"), (PWR_OUT, "BEC_GND", "5")]},
               HW2, HH2)
    HW3, HH3 = 12.7, 7.62
    u3 = place({"R": [(PWR_OUT, "A", "6"), (PWR_OUT, "B", "7"), (PWR_OUT, "C", "8")]},
               HW3, HH3)
    desc = ("~40 A BLHeli ESC with SBEC (5 V, ~3 A), drawn as its three real "
            "connectors. BAT- and BEC_GND are separate wires, common ONLY inside "
            "the ESC - that internal tie is the harness star ground. Phase order "
            "is arbitrary; swap any two to reverse the motor.")
    return symbol("ESC_40A_BLHeli_SBEC", "ESC 40A + SBEC", desc,
                  [(HW1, HH1, u1, "J_PACK"), (HW2, HH2, u2, "J_CTRL"),
                   (HW3, HH3, u3, "J_PHASE")])


# ---------------------------------------------------------------- misc
def servo():
    HW, HH = 8.89, 6.35
    p = place({"L": [(INPUT, "SIG", "1"), (PWR_IN, "V+", "2"), (PWR_IN, "GND", "3")]},
              HW, HH)
    desc = ("Goteck GS-5010MG, 1.4 kg.cm @ 6 V metal-gear micro, 4.8-6 V. V+ AND "
            "GND both come from the ESC BEC; only SIG comes from the FC.")
    return symbol("Servo_Goteck_GS5010MG", "GS-5010MG", desc, [(HW, HH, p, None)])


def motor():
    HW, HH = 8.89, 6.35
    p = place({"L": [(PWR_IN, "A", "1"), (PWR_IN, "B", "2"), (PWR_IN, "C", "3")]},
              HW, HH)
    return symbol("Motor_BLDC_3ph", "BLDC motor", "3-phase brushless outrunner.",
                  [(HW, HH, p, None)])


def battery():
    HW, HH = 8.89, 6.35
    p = place({"R": [(PWR_OUT, "+", "1"), (PWR_OUT, "-", "2")]}, HW, HH)
    return symbol("Battery_LiPo_3S", "3S LiPo", "3S LiPo pack, XT60.",
                  [(HW, HH, p, None)])


def main():
    out = ["(kicad_symbol_lib",
           f"  (version {LIB_VERSION})",
           '  (generator "autoc-article2")',
           '  (generator_version "10.0")']
    out += [fc(), xiao(), gps(), esc(), servo(), motor(), battery()]
    out.append(")")
    sys.stdout.write("\n".join(out) + "\n")


if __name__ == "__main__":
    main()
