#!/usr/bin/env python3
"""
Generate article2-harness.kicad_sch -- the article 2 interconnect schematic.

DESIGN RULE: net labels on pins, NOT drawn wires. There is no board to fabricate;
the deliverable is a readable interconnect document. Point-to-point routing is
what made the earlier beacon sheets a tangle and a model is bad at it. Labels make
the netlist exact and let each module own a region of the sheet.

GROUND RULE: the four ground nets are deliberately DISTINCT because they are
distinct WIRES. They are bonded only inside a module - GND_PACK to GND_BEC inside
the ESC, GND_PACK to GND_XIAO/GND_GPS inside the FC. Drawing them as one net would
hide the single most important decision in this harness. See harness-spec.md 2a.

Netlist source of record is harness-spec.md section 6; check.sh proves they agree.

Regenerate:  python3 gen_sheet.py > article2-harness.kicad_sch
"""

import hashlib
import re
import sys

PROJECT = "article2-harness"
LIBNAME = "article2-harness"
SYMLIB = "article2-harness.kicad_sym"
PAPER = "A3"
ROOT_UUID = "2a1f0c64-0000-4000-8000-a17c1e020001"


def uid(*parts):
    h = hashlib.sha1(("article2:" + ":".join(str(p) for p in parts)).encode()).hexdigest()
    return f"{h[0:8]}-{h[8:12]}-4{h[13:16]}-8{h[17:20]}-{h[20:32]}"


# ---------------------------------------------------------------- symbol lib
def load_lib(path):
    """Per-unit pin geometry and body box out of the .kicad_sym."""
    s = open(path).read()
    syms = {}
    starts = [m.start() for m in re.finditer(r'^  \(symbol "', s, re.M)]
    starts.append(len(s) - 2)
    for i in range(len(starts) - 1):
        blk = s[starts[i]:starts[i + 1]].rstrip()
        name = re.match(r'^  \(symbol "([^"]+)"', blk).group(1)
        units = {}
        usub = [m for m in re.finditer(r'^    \(symbol "%s_(\d+)_1"' % re.escape(name),
                                       blk, re.M)]
        bounds = [m.start() for m in usub] + [len(blk)]
        for j, m in enumerate(usub):
            idx = int(m.group(1))
            ub = blk[bounds[j]:bounds[j + 1]]
            rect = re.search(r'\(rectangle\s*\(start ([-\d.]+) ([-\d.]+)\)\s*'
                             r'\(end ([-\d.]+) ([-\d.]+)\)', ub)
            pins = {}
            for pm in re.finditer(
                    r'\(pin (\w+) line\s*\n\s*\(at ([-\d.]+) ([-\d.]+) (\d+)\)\s*\n'
                    r'\s*\(length [\d.]+\)\s*\n\s*\(name "([^"]+)"[^\n]*\n'
                    r'\s*\(number "([^"]+)"', ub):
                et, x, y, ang, pname, pnum = pm.groups()
                pins[pname] = dict(num=pnum, x=float(x), y=float(y),
                                   ang=int(ang), etype=et)
            units[idx] = dict(pins=pins, half_h=abs(float(rect.group(2))))
        syms[name] = dict(block=blk, units=units)
    return syms


LIB = load_lib(SYMLIB)


def lib_symbols_section():
    out = ["\t(lib_symbols"]
    for name, d in LIB.items():
        b = d["block"].replace(f'(symbol "{name}"', f'(symbol "{LIBNAME}:{name}"', 1)
        out.append("\n".join("\t" + ln for ln in b.split("\n")))
    out.append("\t)")
    return "\n".join(out)


# ---------------------------------------------------------------- placement
# ref -> (symbol, value, {unit: (x, y)})
PLACE = {
    "BT1":  ("Battery_LiPo_3S", "3S LiPo", {1: (48.26, 63.50)}),
    "U4":   ("ESC_40A_BLHeli_SBEC", "ESC 40A + SBEC",
             {1: (48.26, 104.14),     # J_PACK
              2: (48.26, 149.86),     # J_CTRL
              3: (48.26, 194.31)}),   # J_PHASE
    "MOT1": ("Motor_BLDC_3ph", "BLDC motor", {1: (48.26, 228.60)}),
    "U1":   ("FC_HGLRC_SpecterF722Lite", "Specter F722 Lite",
             {1: (205.74, 71.12),     # TOP face
              2: (205.74, 180.34)}),  # BOTTOM face
    "U2":   ("MCU_Seeed_XIAO_nRF52840", "XIAO nRF52840", {1: (355.60, 78.74)}),
    "U3":   ("GPS_Matek_M10Q_5883", "M10Q-5883", {1: (355.60, 173.99)}),
    "S1":   ("Servo_Goteck_GS5010MG", "GS-5010MG", {1: (180.34, 231.14)}),
    "S2":   ("Servo_Goteck_GS5010MG", "GS-5010MG", {1: (266.70, 231.14)}),
}

# (ref, unit, pin name) -> net label.  Mirrors harness-spec.md section 6.
NETS = {
    # ---- pack rail -----------------------------------------------------
    ("BT1", 1, "+"):              "VBAT",
    ("U4",  1, "BAT+ (60A)"):     "VBAT",
    ("U1",  2, "BAT_B"):          "VBAT",        # the bypass wire -> vbat sense
    # ---- GND_PACK: the star. Motor return + the FC's single ground spoke.
    ("BT1", 1, "-"):              "GND_PACK",
    ("U4",  1, "BAT- (60A)"):     "GND_PACK",
    ("U1",  2, "GND_B"):          "GND_PACK",
    # ---- GND_BEC: servo return only. Bonded to GND_PACK INSIDE the ESC.
    ("U4",  2, "BEC_GND"):        "GND_BEC",
    ("S1",  1, "GND"):            "GND_BEC",
    ("S2",  1, "GND"):            "GND_BEC",
    ("U4",  2, "BEC+5V"):         "5V_BEC",
    ("S1",  1, "V+"):             "5V_BEC",
    ("S2",  1, "V+"):             "5V_BEC",
    # ---- motor ---------------------------------------------------------
    ("U4",  3, "A"): "PHASE_A", ("MOT1", 1, "A"): "PHASE_A",
    ("U4",  3, "B"): "PHASE_B", ("MOT1", 1, "B"): "PHASE_B",
    ("U4",  3, "C"): "PHASE_C", ("MOT1", 1, "C"): "PHASE_C",
    # ---- FC outputs. ONLY the signal wire leaves the FC for each actuator.
    ("U1",  2, "M3"):             "THROTTLE",
    ("U4",  2, "SIG"):            "THROTTLE",
    ("U1",  2, "M1"):             "SERVO1",
    ("S1",  1, "SIG"):            "SERVO1",
    ("U1",  2, "M2"):             "SERVO2",
    ("S2",  1, "SIG"):            "SERVO2",
    # ---- xiao: UART1 + power from the FC TOP-face left edge --------------
    ("U1",  1, "5V_T1"):          "5V_XIAO",
    ("U2",  1, "5V/VBUS"):        "5V_XIAO",
    ("U1",  1, "GND_T1"):         "GND_XIAO",
    ("U2",  1, "GND"):            "GND_XIAO",
    ("U1",  1, "RX1"):            "XIAO_TX",
    ("U2",  1, "D6/TX"):          "XIAO_TX",
    ("U1",  1, "TX1"):            "XIAO_RX",
    ("U2",  1, "D7/RX"):          "XIAO_RX",
    # ---- GPS: UART4 + I2C + power, all six from the FC BOTTOM-face row ----
    ("U1",  2, "5V_B"):           "5V_GPS",
    ("U3",  1, "5V (red)"):       "5V_GPS",
    ("U1",  2, "GND_B2"):         "GND_GPS",
    ("U3",  1, "GND (black)"):    "GND_GPS",
    ("U1",  2, "TX4"):            "GPS_RX",
    ("U3",  1, "RX (yellow)"):    "GPS_RX",
    ("U1",  2, "RX4"):            "GPS_TX",
    ("U3",  1, "TX (blue)"):      "GPS_TX",
    ("U1",  2, "SCL"):            "I2C_SCL",
    ("U3",  1, "CL (green)"):     "I2C_SCL",
    ("U1",  2, "SDA"):            "I2C_SDA",
    ("U3",  1, "DA (white)"):     "I2C_SDA",
}

NO_CONNECT = [
    ("U1", 1, "CAM"), ("U1", 1, "VTX"), ("U1", 1, "RX2"), ("U1", 1, "TX2"),
    ("U1", 1, "ES-5V"), ("U1", 1, "5V_T3"), ("U1", 1, "GND_T3"),
    ("U1", 1, "BAT_T"), ("U1", 1, "VIN"), ("U1", 1, "5V_T2"), ("U1", 1, "GND_T2"),
    ("U1", 1, "LED"), ("U1", 1, "BB-"), ("U1", 1, "5V_T4"), ("U1", 1, "GND_T4"),
    ("U1", 1, "BAT_T2"), ("U1", 1, "GND_T5"), ("U1", 1, "RX3"), ("U1", 1, "TX3"),
    ("U1", 2, "M4"), ("U1", 2, "CR"), ("U1", 2, "RX6"),
    ("U2", 1, "D0/A0"), ("U2", 1, "D1/A1"), ("U2", 1, "D2/A2"), ("U2", 1, "D3/A3"),
    ("U2", 1, "D4/SDA"), ("U2", 1, "D5/SCL"), ("U2", 1, "3V3"),
    ("U2", 1, "D8/SCK"), ("U2", 1, "D9/MISO"), ("U2", 1, "D10/MOSI"),
]

NOTES = [
    (130.81, 36.83, "ARTICLE 2 HARNESS  -  net labels, not drawn wires.  Netlist of record: harness-spec.md S6."),
    (130.81, 41.91, "PROPOSED / NOT BUILT.  Items marked (!) in harness-spec.md S8 are unverified."),
    (120.65, 118.11, "U1 jumpers: ELRS short-connect pad = BRIDGED (internal RX on UART2)."),
    (120.65, 123.19, "UART1 = xiao MSP   UART2 = ELRS CRSF   UART3 = spare   UART4 = GPS"),
    (120.65, 128.27, "UART5 has no pads.  UART6 is RX-only.  M1-M4 are the only output pads."),
    (120.65, 135.89, "U1 unit 1 = TOP face pads.  U1 unit 2 = BOTTOM face pads (ESC row + GPS row)."),
    (297.18, 101.60, "(!) XIAO 5V pin IS USB VBUS. FC rail plus a"),
    (297.18, 106.68, "powered USB cable backfeeds the host port."),
    (297.18, 199.39, "GPS FORWARD connector, left to right:"),
    (297.18, 204.47, "red 5V | yellow RX | blue TX | green CL | white DA | black GND"),
    (297.18, 209.55, "(RX before TX - confirmed on the module.)"),
    (107.95, 203.20, "M1,M2 = TIM8 -> timer_output_mode 7 SERVOS @ 50 Hz"),
    (107.95, 208.28, "M3 = TIM1 -> timer_output_mode 0 MOTORS, STANDARD (400 Hz)"),
    # ---- the ground block ----
    (20.32, 251.46, "GROUND TOPOLOGY  -  star point is the ESC pack-negative pad.  Four distinct WIRES, bonded only INSIDE a module:"),
    (20.32, 256.54, "   GND_PACK   BT1 -  ->  U4 BAT-(60A).   U1 GND_B taps the same pads.   Carries motor return (0-40 A)."),
    (20.32, 261.62, "   GND_BEC    U4 BEC_GND  ->  S1/S2 GND.   Carries servo return (~2 A peak).   Bonded to GND_PACK INSIDE U4."),
    (20.32, 266.70, "   GND_XIAO   U1 GND_T1 -> U2 GND.      GND_GPS   U1 GND_B2 -> U3 GND.      Both bonded to GND_PACK INSIDE U1."),
    (20.32, 274.32, "(!) Do NOT take servo V+/GND from the FC - that routes servo return through the gyro's own voltage reference."),
    (20.32, 279.40, "(!) Do NOT land the ESC lead's BEC_GND on an FC pad - same problem, plus a loop.  Split the lead: only SIG goes to U1."),
]


# ---------------------------------------------------------------- emit
def pin_sheet_xy(ref, unit, pname):
    sym, _, units = PLACE[ref]
    px, py = units[unit]
    p = LIB[sym]["units"][unit]["pins"][pname]
    return (px + p["x"], py - p["y"], p["ang"])


LABEL_ROT = {0: 180, 180: 0, 270: 90, 90: 270}


def emit_symbol(ref, unit):
    sym, value, units = PLACE[ref]
    x, y = units[unit]
    u = LIB[sym]["units"][unit]
    pins = "\n".join(
        f'\t\t(pin "{p["num"]}"\n\t\t\t(uuid "{uid(ref, unit, "pin", p["num"])}")\n\t\t)'
        for p in u["pins"].values())
    return f"""\t(symbol
\t\t(lib_id "{LIBNAME}:{sym}")
\t\t(at {x:g} {y:g} 0)
\t\t(unit {unit})
\t\t(exclude_from_sim no)
\t\t(in_bom yes)
\t\t(on_board yes)
\t\t(dnp no)
\t\t(uuid "{uid(ref, unit)}")
\t\t(property "Reference" "{ref}"
\t\t\t(at {x:g} {y - (u["half_h"] + 3.81):g} 0)
\t\t\t(effects (font (size 1.27 1.27)))
\t\t)
\t\t(property "Value" "{value}"
\t\t\t(at {x:g} {y + (u["half_h"] + 3.81):g} 0)
\t\t\t(effects (font (size 1.27 1.27)))
\t\t)
{pins}
\t\t(instances
\t\t\t(project "{PROJECT}"
\t\t\t\t(path "/{ROOT_UUID}"
\t\t\t\t\t(reference "{ref}")
\t\t\t\t\t(unit {unit})
\t\t\t\t)
\t\t\t)
\t\t)
\t)"""


def emit_label(ref, unit, pname, net):
    x, y, ang = pin_sheet_xy(ref, unit, pname)
    rot = LABEL_ROT[ang]
    just = "left" if rot in (0, 90) else "right"
    return f"""\t(label "{net}"
\t\t(at {x:g} {y:g} {rot})
\t\t(effects
\t\t\t(font (size 1.27 1.27))
\t\t\t(justify {just} bottom)
\t\t)
\t\t(uuid "{uid("lbl", ref, unit, pname)}")
\t)"""


def emit_nc(ref, unit, pname):
    x, y, _ = pin_sheet_xy(ref, unit, pname)
    return (f'\t(no_connect\n\t\t(at {x:g} {y:g})\n'
            f'\t\t(uuid "{uid("nc", ref, unit, pname)}")\n\t)')


def emit_text(x, y, s):
    return f"""\t(text "{s}"
\t\t(at {x:g} {y:g} 0)
\t\t(effects
\t\t\t(font (size 1.27 1.27))
\t\t\t(justify left bottom)
\t\t)
\t\t(uuid "{uid("txt", x, y)}")
\t)"""


def main():
    for key in list(NETS) + NO_CONNECT:
        ref, unit, pname = key
        sym = PLACE[ref][0]
        if unit not in PLACE[ref][2]:
            sys.exit(f"ERROR: {ref} unit {unit} is not placed")
        if pname not in LIB[sym]["units"][unit]["pins"]:
            sys.exit(f"ERROR: {ref} unit {unit} has no pin {pname!r}")
    both = set(NETS) & set(NO_CONNECT)
    if both:
        sys.exit(f"ERROR: pins both netted and NC: {both}")
    covered = set(NETS) | set(NO_CONNECT)
    missing = 0
    for ref, (sym, _v, units) in PLACE.items():
        for unit in units:
            for pname in LIB[sym]["units"][unit]["pins"]:
                if (ref, unit, pname) not in covered:
                    print(f"WARNING: {ref}.u{unit}.{pname} neither netted nor NC",
                          file=sys.stderr)
                    missing += 1
    if missing:
        sys.exit(f"ERROR: {missing} pins unaccounted for")

    out = ["(kicad_sch",
           "\t(version 20260306)",
           '\t(generator "autoc-article2")',
           '\t(generator_version "10.0")',
           f'\t(uuid "{ROOT_UUID}")',
           f'\t(paper "{PAPER}")',
           "\t(title_block",
           '\t\t(title "Article 2 - FC / xiao / GPS / ESC harness")',
           '\t\t(company "autoc")',
           '\t\t(comment 1 "Interconnect only - net labels, no drawn wires")',
           '\t\t(comment 2 "Grounds are 4 distinct wires, bonded only inside modules")',
           '\t\t(comment 3 "PROPOSED - not built, not verified on hardware")',
           "\t)",
           lib_symbols_section()]
    for x, y, s in NOTES:
        out.append(emit_text(x, y, s))
    for ref, (_s, _v, units) in PLACE.items():
        for unit in units:
            out.append(emit_symbol(ref, unit))
    for (ref, unit, pname), net in NETS.items():
        out.append(emit_label(ref, unit, pname, net))
    for ref, unit, pname in NO_CONNECT:
        out.append(emit_nc(ref, unit, pname))
    out.append('\t(sheet_instances\n\t\t(path "/"\n\t\t\t(page "1")\n\t\t)\n\t)')
    out.append(")")
    sys.stdout.write("\n".join(out) + "\n")


if __name__ == "__main__":
    main()
