# host/ — StepFPGA telemetry reader (USB-serial → WSL)

Reads the correlator's UART telemetry (fpga-toolchain-plan §3.6 block 7) on the host. The frame contract is
in [`beacon_telemetry/frame.py`](beacon_telemetry/frame.py); the gateware must emit it. Buildable + testable
**now** against a mock — no hardware, no board needed.

```bash
cd firmware/beacon-decoder-stepfpga/host
python -m pytest tests/ -q              # parser + mock stream (no hardware)
pip install -e .                        # installs pyserial + the `beacon-telemetry` CLI
beacon-telemetry --port /dev/ttyACM0 --log run.log   # live (after usbipd attach, below)
```

## Getting the telemetry UART into WSL (usbipd)

The StepFPGA is a **composite USB device** (busid `2-1`, VID:PID `0d28:0204`): one cable = the STEPLink
**`D:` flash volume** + the **telemetry UART (Windows COM3)** + CMSIS-DAP. WSL2 cannot reach a Windows COM
port directly, so the UART is bridged with **usbipd-win**:

```powershell
# 1. ONE-TIME, elevated PowerShell (Administrator) — share the device:
usbipd bind --busid 2-1
```
```bash
# 2. per session, from WSL (no admin) — map it into this WSL distro:
usbipd.exe attach --wsl --busid 2-1
ls /dev/ttyACM0                         # the UART appears here (CDC-ACM)
# ... read telemetry ...
usbipd.exe detach --busid 2-1           # release it back to Windows
```

### ⚠️ The `D:` vs telemetry trade-off (one device, two homes)

`usbipd attach` moves the **whole** composite into WSL — so while attached, **`D:` disappears from Windows**
and [`../build.sh --flash`](../build.sh) (which copies the `.jed` to `D:\` on the Windows side) **won't work**.
Work in two modes:

| Mode | Device home | Do |
|---|---|---|
| **Flash** | Windows (detached) | `usbipd.exe detach --busid 2-1` → `./build.sh --flash` |
| **Telemetry** | WSL (attached) | `usbipd.exe attach --wsl --busid 2-1` → `beacon-telemetry --port /dev/ttyACM0` |

Flashing is occasional (gateware changes); telemetry is continuous during bench work — so the usual loop is
**flash once → attach → observe**. *(Possible future simplification: when attached, the mass-storage also
appears in WSL as a block device, so a fully-WSL flash-by-mount could replace the Windows copy — unproven,
not needed yet.)*

The same usbipd path serves the **emitter** bring-up: its serialUPDI USB-UART adapter (a separate device,
arriving in a couple of days) gets its own `bind`/`attach` so `pymcuprog` can flash the ATtiny412 from WSL.

## Status

- Parser + mock + tests: **done** (hardware-free).
- Live read: ready, pending (a) the gateware emitting frames (FPGA F3) and (b) a usbipd `bind` (admin, once).
