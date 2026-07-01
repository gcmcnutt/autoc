# ATtiny412 beacon-pod firmware — one-time tooling setup

**Spec ref**: [research.md R2](../../specs/031-beacon-camera/research.md).
**Goal**: get from a fresh DGX install to "I can flash a hex onto an ATtiny412" in under 30 minutes.

## What you need

| Item | Purpose | Cost |
|---|---|---|
| **avr-gcc + avr-libc + binutils-avr** | C compiler for AVR (ATtiny412 is in the `atmega_attiny` family) | Free (apt package) |
| **pymcuprog** | Host-side flash tool that speaks Microchip UPDI over serial | Free (pip) |
| **USB-UART adapter** | Provides the TX line that doubles as UPDI | ~$5 (any FT232 / CH340 / CP2102) |
| **4.7 kΩ resistor** | Pull-up between USB-UART TX and the ATtiny412 UPDI pin (the "serialUPDI" trick) | ~$0.10 |
| **Microchip ATtiny412 Curiosity Nano** *(eval board path)* | All-in-one MCU + UPDI programmer + USB; eliminates the USB-UART + resistor for first bring-up | ~$10 |
| **Microchip ATtiny412 SOIC-8 bare** *(production-pod path)* | Soldered into the hand-built pod once firmware is verified | ~$0.60 each |

The Curiosity Nano path is recommended for first bring-up (T031). After firmware passes scope verification on the Nano, flash the same `.hex` onto a bare SOIC-8 on the hand-built pod (T033) via serialUPDI.

## Install steps (Ubuntu / DGX Linux)

### 1. avr-gcc toolchain

```bash
sudo apt update
sudo apt install gcc-avr avr-libc binutils-avr
```

Verify:
```bash
avr-gcc --version    # expect: avr-gcc (GCC) 5.4.0 or later
avr-objcopy --version
avr-size --version
```

### 2. pymcuprog (Microchip UPDI flasher)

```bash
pip install pymcuprog
# OR if you want isolation:
pipx install pymcuprog
```

Verify:
```bash
pymcuprog --version  # expect: pymcuprog 3.x or later
pymcuprog --help     # confirm it lists 'write', 'read', 'erase'
```

### 3. USB-UART adapter (serialUPDI cable)

**Wiring** (the "serialUPDI" trick — Microchip's pinout):

```
USB-UART adapter            4.7 kΩ              ATtiny412
─────────────────                                ───────────
TX  ──────────────┬────────/\/\/\─────────┬──── UPDI (pin 6)
                  │                        │
RX  ──────────────┘                        │
                                           │
GND ───────────────────────────────────────┴──── GND  (pin 4)

3V3 / 5V (from adapter or external) ─────────── VCC  (pin 1)
```

- TX and RX of the USB-UART are tied together via the 4.7 kΩ resistor, with the UPDI pin connected to that junction. This is half-duplex single-wire UPDI over serial.
- VCC can come from the adapter's 3.3 V or 5 V rail, OR from a separate bench supply. ATtiny412 is 1.8-5.5 V tolerant.
- Common chips that work: **FT232 (recommended — reliable timing)**, CH340, CP2102. Avoid CH341 (some variants have flaky baud-rate setting).

Reference: [Microchip TB3216 — Getting Started with UPDI](https://ww1.microchip.com/downloads/en/Appnotes/TB3216-Getting-Started-with-UPDI-DS90003216.pdf) (the canonical doc).

### 4. Verify the toolchain end-to-end (smoke test)

With the ATtiny412 wired per above and the USB-UART at `/dev/ttyUSB0` (Linux assigns this automatically — confirm with `dmesg | tail` after plugging in):

```bash
cd firmware/beacon-pod/
make CODE_ID=0           # builds beacon-pod.hex
make flash CODE_ID=0     # flashes to ATtiny412 via serialUPDI
```

Expected output of `make flash`:
- Three `Connecting to UART` lines
- `Pinging device... OK`
- `Programming flash... OK`
- `Verifying flash... OK`

If the `make flash` step fails:
- **No `/dev/ttyUSB0`**: USB-UART not detected — check `dmesg | tail`; try a different USB cable; some FT232 boards need driver install on first plug.
- **Connection timeout**: wiring problem; double-check the 4.7 kΩ resistor placement and UPDI pin number (pin 6 on SOIC-8).
- **Permission denied on /dev/ttyUSB0**: add yourself to the `dialout` group: `sudo usermod -a -G dialout $USER`, then log out + back in.

### 5. Optional — Curiosity Nano shortcut

If you have the Microchip **ATtiny412 Curiosity Nano** dev board (recommended for T031), skip the USB-UART wiring entirely. The Nano has the UPDI programmer + USB built in:

```bash
# Plug in the Curiosity Nano via USB-C. Linux assigns /dev/ttyACM0 (not ttyUSB0).
make flash CODE_ID=0 SERIAL=/dev/ttyACM0
```

(The Makefile's `flash` target hardcodes `/dev/ttyUSB0` — for the Nano, either edit the `SERIAL := /dev/ttyACM0` line or pass it as a make var.)

## What this does NOT cover

- **Bench-flashing the hand-built pod**: same wiring as above but you solder the UPDI pin to a header or pad on the pod PCB / perfboard. The `hand-prototype-guide.md` shows the header placement.
- **Debugging via UPDI**: `pymcuprog` supports breakpoints + reads. See `pymcuprog --help` for the full subcommand list. Not needed for Phase 1 firmware (no debugger required).
- **OTA updates**: not supported. UPDI is the only flash path. To re-flash a deployed pod, pull the battery, hook up the UPDI cable, flash, reinsert battery.

## Quick check — is this DGX ready?

```bash
which avr-gcc pymcuprog && \
  ls /dev/ttyUSB* /dev/ttyACM* 2>&1 | head && \
  echo "Ready to flash"
```

All three commands returning real paths (not "not found") = ready.

---

## ATtiny416 Xplained Nano (XNANO) — PlatformIO + on-board mEDBG (working bring-up, 2026-06-30)

The **eval** path (distinct from the 412 serialUPDI above): `firmware/beacon-pod/platformio.ini` env `xnano416`,
bare-C (no Arduino framework — `atmelmegaavr` provides the ATtiny device pack, so **no apt/avr-libc, no sudo**).
Flashed through the board's on-board **mEDBG** debugger via `pymcuprog`.

**One-time (per boot) — share the mEDBG into WSL.** The mEDBG is a *composite* device (HID debug + CDC COM),
so a plain `usbipd bind` won't attach — it needs `--force`. In an **Administrator** PowerShell on Windows:
```
usbipd bind --force --busid <busid>     # <busid> = the 03eb:2145 device in `usbipd list` (persists across reboots)
usbipd attach --wsl --busid <busid>     # re-run after each reboot (attach does not persist)
```
Confirm in WSL: `lsusb | grep 03eb` and `~/.venvs/avr/bin/pymcuprog ping -d attiny416` → `Ping response: 1E9221`.

**pymcuprog** (system pip is PEP-668 locked, so use a venv):
```
python3 -m venv ~/.venvs/avr && ~/.venvs/avr/bin/pip install pymcuprog
```

**Build + flash** (xiao-style; open `firmware/beacon-pod/` as its own VS Code workspace folder for the PIO buttons):
```
~/.platformio/penv/bin/platformio run -e xnano416            # build (bare-C)
~/.platformio/penv/bin/platformio run -e xnano416 -t upload  # → pymcuprog write via mEDBG → 416
```
LED0 = **PB5** (active-low), SW0 = PB4 (from the board def).

**Debugging**: PlatformIO has **no hardware debug** for tinyAVR (board `debug:{}`). Real UPDI hardware debug
(breakpoints/step/watch) = **MPLAB X on Windows** driving the mEDBG natively (no usbip). PIO `debug_tool=simavr`
gives simulator-only stepping if desired.
