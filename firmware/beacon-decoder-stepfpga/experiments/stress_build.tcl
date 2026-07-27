# Build the stress design through PAR + timing trace (no Export/flash — the .mrp + .twr reports are the goal).
set PROJ "stress"; set IMPL "impl1"; set TOP "stress"; set DEV "LCMXO2-4000HC-4MG132C"
if {[catch {prj_project open "$PROJ.ldf"}]} { prj_project new -name $PROJ -impl $IMPL -dev $DEV -synthesis "synplify" }
proc rf {f} { set h [open $f r]; set d [read $h]; close $h; return $d }
if {[file exists "stress_pins.lpf"]} {
  if {![file exists "$PROJ.lpf"] || [rf "stress_pins.lpf"] ne [rf "$PROJ.lpf"]} { file copy -force "stress_pins.lpf" "$PROJ.lpf" }
}
foreach f [glob -nocomplain *.v] { catch {prj_src add $f} }
catch {prj_impl option top $TOP}
prj_project save
prj_run PAR -impl $IMPL    ;# synth → translate → map (→ .mrp util) → par → trce (→ .twr timing)
prj_project close
