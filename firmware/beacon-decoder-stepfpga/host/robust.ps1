param([string]$Port="COM3",[int]$Baud=115200,[int]$OnMask=1,[int]$FreqA=-1,[int]$FreqB=-1,[double]$Settle=6,[double]$Recover=7)
# Skew code A (USB 'E') and/or B ('F'), lock+settle, flush flywheel ('Z'), then measure COLD re-acquire + q-climb.
$p = New-Object System.IO.Ports.SerialPort $Port,$Baud,([System.IO.Ports.Parity]::None),8,([System.IO.Ports.StopBits]::One)
$p.DtrEnable = $true; $p.ReadTimeout = 120; $p.Open()
function S($b){ $p.Write([byte[]]@([byte]$b),0,1) }
S 0x2B; Start-Sleep -Milliseconds 20
S (0x80 -bor $OnMask)
if ($FreqA -ge 0) { S 0x45; S ([byte]($FreqA -band 0xFF)) }    # 'E' skew emitter A
if ($FreqB -ge 0) { S 0x46; S ([byte]($FreqB -band 0xFF)) }    # 'F' skew emitter B
Start-Sleep -Milliseconds ([int]($Settle*1000))
S 0x5A                                                          # 'Z' flush flywheel (true cold)
$p.DiscardInBuffer(); Write-Output "MARK"
$d=(Get-Date).AddSeconds($Recover); while((Get-Date) -lt $d){ try{ Write-Output $p.ReadLine() }catch{} }
S 0x45; S 0x80; S 0x46; S 0x80; S 0x2D; Start-Sleep -Milliseconds 20; $p.Close()  # reset rates, release
