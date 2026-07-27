param(
  [string]$Port="COM3",[int]$Baud=115200,
  [int]$Mask=1,                 # 7-bit knob mask: [0]enA [1]enB [2]enN [3]inj1 [4]inj2 [5]weak [6]floor
  [int]$FreqA=-1,[int]$FreqB=-1,# 'E'/'F' emitter skew (128=nominal); -1 = leave nominal
  [int]$AmpA=-1,[int]$AmpB=-1,[int]$AmpN=-1,  # 'A'/'B'/'G' magnitudes (×6); -1 = default
  [int]$Burst=-1,               # 'K' dropout span in chips; -1 = none
  [double]$Settle=5,[double]$Watch=6,
  [string]$Action="none",       # none | flush | drop
  [double]$DropSec=8,[int]$DropMask=0,          # for Action=drop: blackout DropMask for DropSec, then restore
  [int]$Ext=-1                  # s4 'X' code-B source: 1=external emitter, 0=synthetic; -1 = leave (default ext)
)
# One acceptance scenario: configure the sim, let it settle, optionally perturb (flush / timed dropout),
# MARK the edge, then stream telemetry for $Watch s. acceptance.py parses the post-MARK frames.
$p = New-Object System.IO.Ports.SerialPort $Port,$Baud,([System.IO.Ports.Parity]::None),8,([System.IO.Ports.StopBits]::One)
$p.DtrEnable = $true; $p.ReadTimeout = 120; $p.Open()
function S($b){ $p.Write([byte[]]@([byte]$b),0,1) }
S 0x2B; Start-Sleep -Milliseconds 20                              # REMOTE
if ($Ext -ge 0) { S 0x58; S ([byte]$Ext) }                       # 's4: X' select code-B source (ext/synth)
S (0x80 -bor ($Mask -band 0x7F))
if ($FreqA -ge 0) { S 0x45; S ([byte]$FreqA) }
if ($FreqB -ge 0) { S 0x46; S ([byte]$FreqB) }
if ($AmpA  -ge 0) { S 0x41; S ([byte]$AmpA) }
if ($AmpB  -ge 0) { S 0x42; S ([byte]$AmpB) }
if ($AmpN  -ge 0) { S 0x47; S ([byte]$AmpN) }
if ($Burst -ge 0) { S 0x4B; S ([byte]$Burst) }
Start-Sleep -Milliseconds ([int]($Settle*1000))                  # acquire + DPLL settle
switch ($Action) {
  "flush" { S 0x5A }                                             # true-cold
  "drop"  { S (0x80 -bor ($DropMask -band 0x7F)); Start-Sleep -Milliseconds ([int]($DropSec*1000));
            S (0x80 -bor ($Mask -band 0x7F)) }                   # blackout then restore (re-acquire)
}
$p.DiscardInBuffer(); Write-Output "MARK"
$d=(Get-Date).AddSeconds($Watch); while((Get-Date) -lt $d){ try{ Write-Output $p.ReadLine() }catch{} }
S 0x45; S 0x80; S 0x46; S 0x80; S 0x4B; S 0x00; S 0x2D          # reset skews/burst, release to LOCAL
Start-Sleep -Milliseconds 20; $p.Close()
