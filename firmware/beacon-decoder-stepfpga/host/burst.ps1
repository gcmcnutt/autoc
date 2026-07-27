param([string]$Port="COM3",[int]$Baud=115200,[int]$OnMask=1,[int]$Span=0,[int]$Freq=128,[double]$Settle=4,[double]$Watch=6)
# Burst-dropout test (A4d-3): lock, then apply a periodic occlusion of $Span CONSECUTIVE chips (USB 'K') and
# watch whether the FSM HOLD + flywheel coast it. $Span=0 disables. Both beacons are squelched during the burst.
$p = New-Object System.IO.Ports.SerialPort $Port,$Baud,([System.IO.Ports.Parity]::None),8,([System.IO.Ports.StopBits]::One)
$p.DtrEnable = $true; $p.ReadTimeout = 120; $p.Open()
function S($b){ $p.Write([byte[]]@([byte]$b),0,1) }
S 0x2B; Start-Sleep -Milliseconds 20
S (0x80 -bor $OnMask)
if ($Freq -ge 0) { S 0x46; S ([byte]($Freq -band 0xFF)) }      # emitter-B rate (if exercising B)
Start-Sleep -Milliseconds ([int]($Settle*1000))                # acquire + DPLL settle (full quality)
S 0x4B; S ([byte]($Span -band 0xFF))                           # 'K' = burst span (chips)
$p.DiscardInBuffer(); Write-Output "MARK"
$d=(Get-Date).AddSeconds($Watch); while((Get-Date) -lt $d){ try{ Write-Output $p.ReadLine() }catch{} }
S 0x4B; S 0x00; S 0x2D; Start-Sleep -Milliseconds 20; $p.Close()  # span 0, release
