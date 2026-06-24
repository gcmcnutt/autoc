param([int]$Mask=0,[int]$Seconds=5,[string]$Port="COM3",[int]$Baud=115200,[switch]$Local)
# Put the StepFPGA in REMOTE mode and set the knob mask, then read telemetry for $Seconds.
#   '+' = REMOTE (USB owns), 0x80|mask = 7-bit knobs, '-' = LOCAL (with -Local).
#   mask bits: 0=enA 1=enB 2=enN(noise) 3=inj-1bit 4=inj-2bit 5=weak-signal 6=DC-floor
$p = New-Object System.IO.Ports.SerialPort $Port,$Baud,([System.IO.Ports.Parity]::None),8,([System.IO.Ports.StopBits]::One)
$p.DtrEnable = $true; $p.ReadTimeout = 200
$p.Open()
if ($Local) { $p.Write([byte[]]@(0x2D),0,1) }                       # '-' back to switches
else {
  $p.Write([byte[]]@(0x2B),0,1); Start-Sleep -Milliseconds 20       # '+' remote on
  $b = [byte](0x80 -bor ($Mask -band 0x7F))
  $p.Write([byte[]]@($b),0,1)                                       # mask
}
$deadline = (Get-Date).AddSeconds($Seconds); $n = 0
while((Get-Date) -lt $deadline){ try{ $line = $p.ReadLine(); Write-Output $line; $n++ } catch {} }
$p.Close()
[Console]::Error.WriteLine(("--- mask=0x{0:X2}; {1} lines in {2}s ---" -f ($Mask -band 0x7F),$n,$Seconds))
