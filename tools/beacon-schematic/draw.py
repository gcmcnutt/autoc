#!/usr/bin/env python3
"""Beacon-pod schematic -- rendered via schemdraw.

Refdes labels here MUST match `specs/031-beacon-camera/schematic.md` so the
rendered PNG + the markdown net list stay in sync.

Layout strategy: explicit (x, y) block placement on a grid; no autoflow.
Each functional subsystem (power entry, boost, LED string, MCU, supervisor,
DIM net) lives in its own labelled box; inter-block connections are drawn as
labelled net-stubs referenced by net name rather than physical routing.

Text uses ASCII only (no Unicode arrows / micro-symbols / inequality glyphs)
because schemdraw's bbox-measurement path parses text via XML, and stray
'<' / Unicode chars trip its mathtext parser.

Outputs:
  specs/031-beacon-camera/cad/beacon-pod-schematic.png  (printable, white bg)
"""

from __future__ import annotations

from pathlib import Path

import schemdraw
import schemdraw.elements as elm

OUTPUT_DIR = (
    Path(__file__).resolve().parent.parent.parent
    / "specs" / "031-beacon-camera" / "cad"
)
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)


def boost_converter(d: schemdraw.Drawing, x: float, y: float) -> None:
    """Block: boost converter U1 + L1 + D1 + C1 -> V_LED."""
    d += elm.Label().label("BOOST CONVERTER", fontsize=12, color="navy").at((x, y + 5))

    u1 = d.add(
        elm.Ic(
            pins=[
                elm.IcPin(name="VIN", side="left", pin="5"),
                elm.IcPin(name="GND", side="left", pin="2"),
                elm.IcPin(name="SW", side="right", pin="1"),
                elm.IcPin(name="FB", side="right", pin="3"),
                elm.IcPin(name="DIM", side="bottom", pin="4"),
            ],
            edgepadW=0.8, edgepadH=0.8, pinspacing=1.2,
            label="U1\nLM3410X\nA1",
        ).at((x, y))
    )
    d += elm.Line().left(1).at(u1.VIN)
    d += elm.Label().label("V_BAT", color="red", fontsize=9).at((u1.VIN[0] - 1.5, u1.VIN[1]))
    d += elm.Line().left(1).at(u1.GND)
    d += elm.Label().label("GND", fontsize=9).at((u1.GND[0] - 1.5, u1.GND[1]))
    d += elm.Line().down(1).at(u1.DIM)
    d += elm.Label().label("DIM", color="green", fontsize=9).at((u1.DIM[0], u1.DIM[1] - 1.4))

    d += elm.Line().right(1.5).at(u1.SW)
    sw_node = (u1.SW[0] + 1.5, u1.SW[1])
    d += elm.Dot().at(sw_node)

    l1 = d.add(
        elm.Inductor()
        .up(2)
        .at(sw_node)
        .label("L1\n22 uH\nshielded\nA5", loc="left", fontsize=9)
    )
    d += elm.Line().up(0.5).at(l1.end).color("red")
    d += elm.Label().label("V_BAT", color="red", fontsize=9).at((l1.end[0], l1.end[1] + 0.7))

    d1 = d.add(
        elm.Diode()
        .right(2.5)
        .at(sw_node)
        .label("D1 MBR130 (A6) Schottky", loc="bottom", fontsize=8)
    )
    vled_node = d1.end
    d += elm.Dot().at(vled_node)
    d += elm.Label().label("V_LED", color="darkorange", fontsize=10).at((vled_node[0] + 0.6, vled_node[1] + 0.3))

    d += elm.Line().down(0.5).at(vled_node)
    c1 = d.add(
        elm.Capacitor()
        .down(1.5)
        .at((vled_node[0], vled_node[1] - 0.5))
        .label("C1\n4.7 uF\n25 V\nA8", loc="right", fontsize=8)
    )
    d += elm.Ground().at(c1.end)

    d += elm.Line().right(1).at(u1.FB)
    d += elm.Label().label("FB from R1\n(LED-string sense)", color="blue", fontsize=8).at((u1.FB[0] + 2.5, u1.FB[1]))


def led_string(d: schemdraw.Drawing, x: float, y: float) -> None:
    d += elm.Label().label(
        "LED STRING\n5x Lumileds L1IZ 850 nm (BOM B1)\nseries: apex first, then 4 sides",
        fontsize=11, color="navy",
    ).at((x, y + 1))

    d += elm.Label().label("V_LED", color="darkorange", fontsize=10).at((x, y))

    start_y = y - 0.7
    leds = []
    labels = [
        "LED1 (apex)",
        "LED2 (side)",
        "LED3 (side)",
        "LED4 (side)",
        "LED5 (side)",
    ]
    pos_y = start_y
    for lbl in labels:
        ledN = d.add(
            elm.Diode()
            .down(1.0)
            .at((x, pos_y))
            .label(lbl, loc="right", fontsize=8)
            .color("darkorange")
        )
        leds.append(ledN)
        pos_y = ledN.end[1]

    r1_start = leds[-1].end
    r1 = d.add(
        elm.Resistor()
        .down(1.5)
        .at(r1_start)
        .label("R1\n0.62 ohm 1pct\nA7\nFB = 190 mV\nso I_LED ~ 306 mA", loc="right", fontsize=8)
    )
    d += elm.Ground().at(r1.end)

    d += elm.Dot().at(r1_start).color("blue")
    d += elm.Line().left(1).at(r1_start).color("blue")
    d += elm.Label().label("to FB (U1 pin 3)", color="blue", fontsize=8).at((r1_start[0] - 2.5, r1_start[1]))


def mcu_block(d: schemdraw.Drawing, x: float, y: float) -> None:
    d += elm.Label().label("MCU", fontsize=12, color="navy").at((x, y + 5.5))

    u2 = d.add(
        elm.Ic(
            pins=[
                elm.IcPin(name="VDD", side="left", pin="1"),
                elm.IcPin(name="GND", side="left", pin="8"),
                elm.IcPin(name="PA6 DIAG", side="right", pin="2"),
                elm.IcPin(name="PA7 spare", side="right", pin="3"),
                elm.IcPin(name="PA1 CSEL0", side="right", pin="4"),
                elm.IcPin(name="PA2 CSEL1", side="right", pin="5"),
                elm.IcPin(name="PA0 UPDI", side="right", pin="6"),
                elm.IcPin(name="PA3 DIM", side="right", pin="7"),
            ],
            edgepadW=0.9, edgepadH=0.5, pinspacing=0.9,
            label="U2\nATtiny412\nA2\nSOIC-8\n20 MHz int RC",
        ).at((x, y))
    )

    d += elm.Line().left(1).at(u2.VDD)
    d += elm.Label().label("V_BAT", color="red", fontsize=9).at((u2.VDD[0] - 1.5, u2.VDD[1]))
    d += elm.Line().left(1).at(u2.GND)
    d += elm.Label().label("GND", fontsize=9).at((u2.GND[0] - 1.5, u2.GND[1]))

    d += elm.Label().label(
        "Decoupling at VDD:\nC3 1 uF (A10) + C4 100 nF (A11)\nclose to MCU pin",
        fontsize=8,
    ).at((u2.VDD[0] - 2, u2.VDD[1] - 2)).color("gray")

    pa3 = u2['PA3 DIM']
    d += elm.Line().right(1).at(pa3)
    d += elm.Label().label("to DIM net (open-drain emulated)", color="green", fontsize=9).at((pa3[0] + 2.5, pa3[1]))

    pa6 = u2['PA6 DIAG']
    d += elm.Line().right(1).at(pa6)
    d += elm.Label().label("R3 1k (A16) and D2 green LED (A15) to GND", color="darkgreen", fontsize=8).at((pa6[0] + 5, pa6[1]))

    pa1 = u2['PA1 CSEL0']
    pa2 = u2['PA2 CSEL1']
    d += elm.Line().right(1).at(pa1)
    d += elm.Label().label("CSEL0 from JP1 (bridge to GND for bit=0; pull-up)", color="purple", fontsize=8).at((pa1[0] + 5.5, pa1[1]))
    d += elm.Line().right(1).at(pa2)
    d += elm.Label().label("CSEL1 from JP2 (bridge to GND for bit=0)", color="purple", fontsize=8).at((pa2[0] + 5.5, pa2[1]))

    pa0 = u2['PA0 UPDI']
    d += elm.Line().right(1).at(pa0)
    d += elm.Label().label("UPDI on J2 (A21), programming only", color="brown", fontsize=8).at((pa0[0] + 4.5, pa0[1]))


def supervisor_block(d: schemdraw.Drawing, x: float, y: float) -> None:
    d += elm.Label().label("VOLTAGE SUPERVISOR (LiPo UVLO)", fontsize=11, color="navy").at((x, y + 3.5))

    u3 = d.add(
        elm.Ic(
            pins=[
                elm.IcPin(name="VCC", side="left"),
                elm.IcPin(name="GND", side="left"),
                elm.IcPin(name="OUT", side="right"),
            ],
            edgepadW=0.8, edgepadH=0.5, pinspacing=1.4,
            label="U3\nMCP1316T\n-29LE/OT\nA4\n3.3 V trip\nopen-drain",
        ).at((x, y))
    )
    d += elm.Line().left(1).at(u3.VCC)
    d += elm.Label().label("V_BAT", color="red", fontsize=9).at((u3.VCC[0] - 1.5, u3.VCC[1]))
    d += elm.Line().left(1).at(u3.GND)
    d += elm.Label().label("GND", fontsize=9).at((u3.GND[0] - 1.5, u3.GND[1]))
    d += elm.Line().right(1).at(u3.OUT)
    d += elm.Label().label("OUT (open-drain) to DIM net\npulls LOW at V_BAT 3.3 V", color="green", fontsize=9).at((u3.OUT[0] + 3.5, u3.OUT[1]))

    d += elm.Label().label("C7 100 nF (A11) at VCC", fontsize=8).at((u3.VCC[0] - 2, u3.VCC[1] - 1.5)).color("gray")


def power_entry_block(d: schemdraw.Drawing, x: float, y: float) -> None:
    d += elm.Label().label("POWER ENTRY", fontsize=12, color="navy").at((x, y + 3))

    bt1 = d.add(
        elm.Battery()
        .up(2)
        .at((x, y))
        .label("BT1\n1S LiPo\n100 mAh (C1)", loc="left", fontsize=9)
    )
    d += elm.Line().right(1).at(bt1.end)
    d += elm.Label().label("J1\nJST-PH\nA14", fontsize=8).at((bt1.end[0] + 1.5, bt1.end[1] + 0.3))
    d += elm.Line().right(1.5).at((bt1.end[0] + 1.5, bt1.end[1])).color("red")
    d += elm.Label().label("V_BAT rail (red)", color="red", fontsize=10).at((bt1.end[0] + 4, bt1.end[1] + 0.3))

    d += elm.Line().right(1).at(bt1.start)
    d += elm.Line().right(1.5).at((bt1.start[0] + 1.5, bt1.start[1]))
    d += elm.Label().label("GND rail", fontsize=10).at((bt1.start[0] + 3, bt1.start[1] - 0.3))
    d += elm.Ground().at((bt1.start[0] + 2.5, bt1.start[1]))

    d += elm.Label().label(
        "V_BAT bulk + decoupling (close to entry):\n"
        "  C2  22 uF / 10 V X7R 1210  (A9)\n"
        "  C5  2.2 uF / 0603           (A12)\n"
        "  C6  100 nF / 0603           (A11)",
        fontsize=8,
    ).at((bt1.end[0] + 4, bt1.end[1] - 2.5)).color("gray")


def dim_net_legend(d: schemdraw.Drawing, x: float, y: float) -> None:
    d += elm.Label().label("DIM NET (WIRED-AND)", fontsize=12, color="navy").at((x, y + 3))

    legend = (
        "Three drivers + one pull-up, all sharing the DIM node:\n"
        "\n"
        "  V_BAT --- R2 10 kohm (A13) ---,\n"
        "                                +--> LM3410X U1 pin 4 (DIM, high-Z input)\n"
        "  U2 PA3 (open-drain emul.) ---,\n"
        "  U3 OUT  (open-drain)        --'\n"
        "\n"
        "Truth table:\n"
        "  supervisor HIGH-Z  AND  MCU HIGH-Z (chip=1)  ==> DIM=HIGH  ==> LEDs ON\n"
        "  supervisor HIGH-Z  AND  MCU LOW    (chip=0)  ==> DIM=LOW   ==> LEDs OFF\n"
        "  supervisor LOW (UVLO)  AND  MCU any          ==> DIM=LOW   ==> LEDs OFF\n"
        "\n"
        "No DC contention path. Supervisor LOW always wins.\n"
        "Soft-start ~ 20 us (datasheet SU): 0.2 pct of 10 ms chip period."
    )
    d += elm.Label().label(legend, fontsize=8.5, halign="left").at((x, y - 1)).color("darkgreen")


def title_block(d: schemdraw.Drawing, x: float, y: float) -> None:
    d += elm.Label().label(
        "031-Beacon-Camera Phase 1 -- Beacon Pod Schematic", fontsize=14,
    ).at((x, y))
    d += elm.Label().label(
        "Source: tools/beacon-schematic/draw.py    Refdes / BOM: schematic.md + verified-bom.md    Spec: FR-1.2 / FR-1.2.1 / FR-1.7",
        fontsize=9,
    ).at((x, y - 0.7)).color("gray")
    d += elm.Label().label(
        "LM3410X (1.6 MHz) boost driver   DIM-only wired-AND control (LM3410 has no separate EN)   5x Lumileds L1IZ 850 nm in series at 300 mA",
        fontsize=9,
    ).at((x, y - 1.4)).color("gray")


def footer_block(d: schemdraw.Drawing, x: float, y: float) -> None:
    d += elm.Label().label(
        "Notes:\n"
        " - Power rails are drawn as labelled net-stubs (red = V_BAT, orange = V_LED, black = GND, green = DIM, blue = FB sense). All rail-tagged endpoints are bonded.\n"
        " - Hand-build order: boost loop (U1 + L1 + D1 + C1) tight FIRST, then MCU + supervisor decoupling close to V_BAT entry, then LED string + diag LED last.\n"
        " - Refdes labels match specs/031-beacon-camera/schematic.md (net list + connection tables there are the source of truth for hand-build).\n"
        " - Regenerate this PNG/SVG/PDF by running: python3 tools/beacon-schematic/draw.py",
        fontsize=8.5, halign="left",
    ).at((x, y)).color("dimgray")


def draw_schematic() -> schemdraw.Drawing:
    d = schemdraw.Drawing()
    d.config(fontsize=10, unit=2.0)

    title_block(d, x=0, y=22)

    power_entry_block(d, x=0, y=19)
    boost_converter(d, x=12, y=18)
    led_string(d, x=23, y=20.5)

    dim_net_legend(d, x=0, y=14)

    mcu_block(d, x=0, y=8)
    supervisor_block(d, x=15, y=8)

    footer_block(d, x=0, y=0)

    return d


def main() -> None:
    d = draw_schematic()
    png_path = OUTPUT_DIR / "beacon-pod-schematic.png"

    # transparent=False gives a solid-white background (no checkerboard in image viewers).
    d.save(str(png_path), dpi=180, transparent=False)

    print(f"Wrote: {png_path}")


if __name__ == "__main__":
    main()
