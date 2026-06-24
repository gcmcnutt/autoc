param([int]$Mask=0,[int]$Seconds=5,[string]$Port="COM3",[int]$Baud=115200)
# Send one knob command (byte 0x40|mask) to the StepFPGA, then read telemetry for $Seconds.
# mask bits: 0=code-select(B) 1=inject-1-bit 2=inject-2-bit 3=weak-signal 4=high-floor.
$p = New-Object System.IO.Ports.SerialPort $Port,$Baud,([System.IO.Ports.Parity]::None),8,([System.IO.Ports.StopBits]::One)
$p.DtrEnable = $true; $p.ReadTimeout = 200
$p.Open()
$b = [byte](0x40 -bor ($Mask -band 0x1F))
$p.Write([byte[]]@($b),0,1)
$deadline = (Get-Date).AddSeconds($Seconds); $n = 0
while((Get-Date) -lt $deadline){ try{ $line = $p.ReadLine(); Write-Output $line; $n++ } catch {} }
$p.Close()
[Console]::Error.WriteLine(("--- sent 0x{0:X2} (mask={1}); {2} lines in {3}s ---" -f $b,$Mask,$n,$Seconds))
