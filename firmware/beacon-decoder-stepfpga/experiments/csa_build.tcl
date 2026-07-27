# Build the carry-save pipelined multiplier through PAR+trce (no flash). Structural pipeline → no retiming
# needed. High synthesis-frequency target so PAR pushes for speed. Reports → .mrp (fit) + .twr (Fmax).
set PROJ "csa"; set IMPL "impl1"; set TOP "top"; set DEV "LCMXO2-4000HC-4MG132C"
if {[catch {prj_project open "$PROJ.ldf"}]} { prj_project new -name $PROJ -impl $IMPL -dev $DEV -synthesis "synplify" }
catch {prj_strgy set_value PROP_SYN_EdfFrequency=300}
catch {prj_strgy set_value PROP_LST_EdfFrequency=300}
proc rf {f} { set h [open $f r]; set d [read $h]; close $h; return $d }
if {[file exists "csa_pins.lpf"]} {
  if {![file exists "$PROJ.lpf"] || [rf "csa_pins.lpf"] ne [rf "$PROJ.lpf"]} { file copy -force "csa_pins.lpf" "$PROJ.lpf" }
}
foreach f [glob -nocomplain *.v] { catch {prj_src add $f} }
catch {prj_impl option top $TOP}
prj_project save
prj_run PAR -impl $IMPL
prj_project close
