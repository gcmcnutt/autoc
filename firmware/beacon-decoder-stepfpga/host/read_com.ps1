param([string]$Port="COM3",[int]$Baud=115200,[int]$Seconds=5)
# Read the StepFPGA telemetry on the WINDOWS side (COM3) — invoked from WSL via interop, like the build.
# No usbipd attach needed: the board stays on Windows, so D: stays flashable AND COM3 stays readable.
# (WSL /dev/ttyACM0 CDC reads hang uninterruptibly — verified 2026-06-23 — so Windows-side is the path.)
$p = New-Object System.IO.Ports.SerialPort $Port,$Baud,([System.IO.Ports.Parity]::None),8,([System.IO.Ports.StopBits]::One)
$p.ReadTimeout = 400
$p.DtrEnable = $true
$p.RtsEnable = $true
try { $p.Open() } catch { Write-Output ("OPEN-FAIL: " + $_.Exception.Message); exit 1 }
$end = (Get-Date).AddSeconds($Seconds); $n = 0
while ((Get-Date) -lt $end) {
  try { $l = $p.ReadLine(); $n++; Write-Output $l.Trim() }   # one telemetry line per read
  catch [TimeoutException] { }
}
$p.Close()
[Console]::Error.WriteLine("--- $n lines in $Seconds s on $Port @ $Baud ---")
