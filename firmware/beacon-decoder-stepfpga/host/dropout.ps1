param([string]$Port="COM3",[int]$Baud=115200,[int]$OnMask=1,[int]$DropMask=0,[double]$Settle=5,[double]$Drop=2,[double]$Recover=3)
# Dropout-recovery test: acquire + let the DPLL sync the rate, force a dropout for $Drop s, restore, measure
# recovery. MARK at the restore edge. $DropMask=0 = full blackout; =4 = noise-only (A removed, noise stays).
$p = New-Object System.IO.Ports.SerialPort $Port,$Baud,([System.IO.Ports.Parity]::None),8,([System.IO.Ports.StopBits]::One)
$p.DtrEnable = $true; $p.ReadTimeout = 100
$p.Open()
function Send1($b){ $p.Write([byte[]]@([byte]$b),0,1) }
function Drain($s){ $d=(Get-Date).AddSeconds($s); while((Get-Date) -lt $d){ try{ Write-Output $p.ReadLine() }catch{} } }
Send1 0x2B; Start-Sleep -Milliseconds 20                 # '+' remote
Send1 (0x80 -bor $OnMask);   Drain $Settle                # acquire + DPLL rate sync
Send1 (0x80 -bor $DropMask); Drain $Drop                  # DROPOUT (rate held = flywheel)
$p.DiscardInBuffer()                                       # flush buffered dropout frames -> clean recovery read
Write-Output "MARK"; Send1 (0x80 -bor $OnMask); Drain $Recover   # restore -> measure recovery
Send1 0x2D; Start-Sleep -Milliseconds 20                 # '-' release
$p.Close()
