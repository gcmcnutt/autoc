param([int]$MaskA=1,[int]$MaskB=0,[double]$PreSec=2,[double]$PostSec=3,[string]$Port="COM3",[int]$Baud=115200)
# Capture a lock transition: set mask A, read PreSec, emit "MARK", set mask B, read PostSec, release override.
# Lets the host time latency-to-lock-change across an edge (A4d-4 LOS / acquisition timing).
$p = New-Object System.IO.Ports.SerialPort $Port,$Baud,([System.IO.Ports.Parity]::None),8,([System.IO.Ports.StopBits]::One)
$p.DtrEnable = $true; $p.ReadTimeout = 100
$p.Open()
$p.Write([byte[]]@(0x2B),0,1); Start-Sleep -Milliseconds 20            # '+' remote
function Drain([double]$sec){ $d=(Get-Date).AddSeconds($sec); while((Get-Date) -lt $d){ try{ Write-Output $p.ReadLine() }catch{} } }
$p.Write([byte[]]@([byte](0x80 -bor ($MaskA -band 0x7F))),0,1); Drain $PreSec
Write-Output "MARK"
$p.Write([byte[]]@([byte](0x80 -bor ($MaskB -band 0x7F))),0,1); Drain $PostSec
$p.Write([byte[]]@(0x2D),0,1); Start-Sleep -Milliseconds 20            # release override
$p.Close()
