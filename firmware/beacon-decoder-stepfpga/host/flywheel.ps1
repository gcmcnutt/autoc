param([string]$Port="COM3",[int]$Baud=115200,[int]$OnMask=1,[double]$Outage=1.5)
# Flywheel demo: A on (acquire + LEARN rate) -> off for $Outage s -> A on (re-acquire). The re-acquire is
# WARM (1-period, rate held) if $Outage < the ~10 s coast, COLD (2-period, rate stale) if longer.
# Emits MARK at the re-acquire edge so the host can time periods-to-confirmed-lock.
$p = New-Object System.IO.Ports.SerialPort $Port,$Baud,([System.IO.Ports.Parity]::None),8,([System.IO.Ports.StopBits]::One)
$p.DtrEnable = $true; $p.ReadTimeout = 100
$p.Open()
function Send1($b){ $p.Write([byte[]]@([byte]$b),0,1) }
function Drain($s){ $d=(Get-Date).AddSeconds($s); while((Get-Date) -lt $d){ try{ Write-Output $p.ReadLine() }catch{} } }
Send1 0x2B; Start-Sleep -Milliseconds 20          # '+' remote
Send1 (0x80 -bor $OnMask); Drain 3.0              # A on: acquire + stable (learn the rate)
Send1 0x80; Drain $Outage                          # off for $Outage seconds (rate held if < coast)
Write-Output "MARK"; Send1 (0x80 -bor $OnMask); Drain 2.5   # re-acquire
Send1 0x2D; Start-Sleep -Milliseconds 20          # '-' release
$p.Close()
