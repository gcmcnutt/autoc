# Diamond batch build for the STEP-MXO2, driven by build.sh via pnmainc.exe from WSL.
# Runs in the /mnt/c sandbox (CWD = native C:\, no UNC). Device + part from threeN1 prior art:
#   LCMXO2-4000HC, package MG132C (CSBGA132), speed 4, Commercial.
# Diamond Tcl API (3.x): prj_project / prj_src / prj_impl / prj_run (confirmed by the §3.6 logs).
# Incremental: prj_run only re-runs stages whose inputs changed (Diamond tracks via file mtime).

set PROJ  "stepfpga"
set IMPL  "impl1"
set TOP   "blink"
set DEV   "LCMXO2-4000HC-4MG132C"

# Open the project if it already exists in the sandbox (incremental); else create it.
if {[catch {prj_project open "$PROJ.ldf"}]} {
  prj_project new -name "$PROJ" -impl "$IMPL" -dev "$DEV" -synthesis "synplify"
}

# Constraints: the project's active preference file is <proj>.lpf (auto-created empty). A SECOND .lpf added
# via prj_src is auto-excluded, so inject our constraints into the active file instead. Only overwrite when
# the content actually differs — preserves Diamond's incremental detection (no needless map re-run).
proc readfile {f} { set h [open $f r]; set d [read $h]; close $h; return $d }
if {[file exists "blink.lpf"]} {
  if {![file exists "$PROJ.lpf"] || [readfile "blink.lpf"] ne [readfile "$PROJ.lpf"]} {
    file copy -force "blink.lpf" "$PROJ.lpf"
  }
}

foreach f [glob -nocomplain *.v] { catch {prj_src add $f} }
catch {prj_impl option top "$TOP"}

prj_project save
# Export -task Jedecgen = bitgen -jedec → the .jed the STEPLink flashes (the §3.6 flow). -task Bitgen would
# yield a .bit (SRAM-style, wrong for flash MachXO2); no -task runs timing only. Output → impl1/$PROJ_$IMPL.jed.
prj_run Export -impl "$IMPL" -task Jedecgen
prj_project close
