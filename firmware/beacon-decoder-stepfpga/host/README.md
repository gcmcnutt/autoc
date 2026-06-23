# host/ — StepFPGA telemetry reader

Reads the correlator's UART telemetry (fpga-toolchain-plan §3.6 block 7). Frame contract in
[`beacon_telemetry/frame.py`](beacon_telemetry/frame.py); the gateware emits it. Parser + mock are testable
with no hardware; the live read runs **on the Windows side via interop** (proven 2026-06-23).

```bash
cd firmware/beacon-decoder-stepfpga/host
python3 -c "import tests.test_frame"   # or: pip install -e .[dev] && python3 -m pytest tests/   (no hardware)
./monitor.sh COM3 5                    # LIVE: read COM3 on Windows for 5 s, print telemetry lines
./monitor.sh COM3 20 | python3 -c 'import sys; from beacon_telemetry import frames_from_lines; \
    [print(f) for f in frames_from_lines(sys.stdin)]'   # parse BCN frames (once the gateware emits them)
```

## Reading telemetry — Windows-side via interop (the proven path)

The StepFPGA's UART is the LPC/STEPLink USB-CDC, **Windows COM3**. We read it **on Windows** and pipe the
output into WSL — exactly like the build drives Diamond via interop:

- [`read_com.ps1`](read_com.ps1) — .NET `SerialPort` reader (115200 8N1, DTR on); [`monitor.sh`](monitor.sh)
  is the WSL wrapper (`cp` to the `/mnt/c` sandbox → `powershell.exe -File`), counterpart to `../build.sh`.
- **The board never leaves Windows**, so `D:` stays flashable (`../build.sh --flash`) *and* COM3 stays
  readable at the same time. No usbipd, no detach/attach dance.

**Verified 2026-06-23**: with the `blink` gateware emitting the LED byte as hex, `./monitor.sh` streamed
`…E8 E9 EA … FF 00 01…` — code → FPGA → UART (pad A2) → LPC → COM3 → PowerShell → WSL.

### Why NOT usbipd-attach into WSL (rejected)

Two reasons, both hit live:
1. The board is a **composite** device — `usbipd attach` pulls the **whole** thing into WSL, so **`D:`
   leaves Windows** and flashing breaks (you'd have to detach to flash, re-attach to read).
2. Worse, the **WSL `/dev/ttyACM0` CDC read hangs uninterruptibly** (no data ever surfaced; `timeout`
   couldn't kill the read). The Windows-side read has neither problem.

`beacon_telemetry.read_serial()` (pyserial on `/dev/ttyUSB*`) remains for a *non-composite* adapter that
behaves under usbipd — e.g. the **emitter's** serialUPDI FT232 (a separate device, no `D:` tension). Decide
its transport when that hardware lands; pymcuprog-on-Windows is the fallback if its WSL read also misbehaves.

## Frame contract (gateware → host)

`BCN,seq,adc,corrA,lockA,marginA,corrB,lockB,marginB\n` — see [`frame.py`](beacon_telemetry/frame.py).
`frames_from_lines()` parses any line source (Windows-read pipe, recorded log, or the mock), so the parser is
transport-agnostic. The F1 `blink` emits a simpler `HH` hex line (proves the pipe); F3/F4 emit real BCN frames.

## Status

- Parser + mock + tests: **done** (hardware-free).
- **Live transport: PROVEN** end-to-end via `monitor.sh` (Windows-side) against the `blink` telemetry build.
- Pending: the gateware emitting real **BCN** frames (FPGA F3/F4) — then `monitor.sh | frames_from_lines`.
