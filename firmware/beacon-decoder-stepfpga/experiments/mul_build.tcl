# Build the pipelined multiplier through PAR+trce with RETIMING enabled (no flash). Reports → .mrp + .twr.
set PROJ "pmul"; set IMPL "impl1"; set TOP "top"; set DEV "LCMXO2-4000HC-4MG132C"
if {[catch {prj_project open "$PROJ.ldf"}]} { prj_project new -name $PROJ -impl $IMPL -dev $DEV -synthesis "synplify" }
# Enable Synplify retiming+pipelining and MAP register-retiming; push the synthesis frequency target high so
# the tool optimizes for speed and distributes the multiply across the output pipeline registers.
# NOTE: brace-quote values with spaces — bare quotes get split by Tcl, leaving the option unchanged.
catch {prj_strgy set_value {PROP_SYN_EdfRunRetiming=Pipelining and Retiming}}
catch {prj_strgy set_value PROP_MAP_RegRetiming=True}
catch {prj_strgy set_value PROP_SYN_EdfFrequency=300}
catch {prj_strgy set_value PROP_LST_EdfFrequency=300}
proc rf {f} { set h [open $f r]; set d [read $h]; close $h; return $d }
if {[file exists "mul_pins.lpf"]} {
  if {![file exists "$PROJ.lpf"] || [rf "mul_pins.lpf"] ne [rf "$PROJ.lpf"]} { file copy -force "mul_pins.lpf" "$PROJ.lpf" }
}
foreach f [glob -nocomplain *.v] { catch {prj_src add $f} }
catch {prj_impl option top $TOP}
prj_project save
prj_run PAR -impl $IMPL
prj_project close
