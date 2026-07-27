param([string]$Port="COM3",[int]$Baud=115200,[int]$OnMask=1,[int]$Freq=128,[double]$Settle=6,[double]$Recover=3)
# TRUE-COLD acquire: lock + learn the rate, then 'Z' = flush the flywheel (forget rate + drop lock) with the
# signal STILL ON -> the correlator re-acquires from scratch (nominal template, MINLOCK). recA = cold latency.
$p = New-Object System.IO.Ports.SerialPort $Port,$Baud,([System.IO.Ports.Parity]::None),8,([System.IO.Ports.StopBits]::One)
$p.DtrEnable = $true; $p.ReadTimeout = 100
$p.Open()
function Send1($b){ $p.Write([byte[]]@([byte]$b),0,1) }
function Drain($s){ $d=(Get-Date).AddSeconds($s); while((Get-Date) -lt $d){ try{ Write-Output $p.ReadLine() }catch{} } }
Send1 0x2B; Start-Sleep -Milliseconds 20
Send1 (0x80 -bor $OnMask)
if ($Freq -ge 0) { Send1 0x46; Send1 ([byte]($Freq -band 0xFF)) }   # set emitter-B rate (if testing skew)
Drain $Settle                                                       # lock + DPLL learns the rate
Send1 0x5A                                                          # 'Z' flush flywheel (true cold)
$p.DiscardInBuffer(); Write-Output "MARK"
Drain $Recover                                                      # cold re-acquire (signal still on)
Send1 0x2D; Start-Sleep -Milliseconds 20
$p.Close()
