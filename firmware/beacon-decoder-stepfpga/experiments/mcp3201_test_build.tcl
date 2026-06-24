set PROJ "mcp3201_test"; set IMPL "impl1"; set TOP "mcp3201_test"; set DEV "LCMXO2-4000HC-4MG132C"
if {[catch {prj_project open "$PROJ.ldf"}]} { prj_project new -name $PROJ -impl $IMPL -dev $DEV -synthesis "synplify" }
proc rf {f} { set h [open $f r]; set d [read $h]; close $h; return $d }
if {[file exists "$PROJ\_pins.lpf"]} {
  if {![file exists "$PROJ.lpf"] || [rf "$PROJ\_pins.lpf"] ne [rf "$PROJ.lpf"]} { file copy -force "$PROJ\_pins.lpf" "$PROJ.lpf" }
}
foreach f [glob -nocomplain *.v] { catch {prj_src add $f} }
catch {prj_impl option top $TOP}
prj_project save
prj_run Export -impl $IMPL -task Jedecgen
prj_project close
