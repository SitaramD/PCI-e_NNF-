# =============================================================================
# create_vivado_project.tcl
# -----------------------------------------------------------------------------
# Builds a Vivado project with all RTL + testbenches so you can use the GUI
# (Run Simulation -> Run Behavioral Simulation).
#
#   Windows (Vivado 2022.2 Tcl Shell, from the repository root):
#       vivado -mode batch -source scripts/create_vivado_project.tcl
#   then open build/vivado/pcie_gen1_x1.xpr
#
# The part only matters if you synthesize. Default is the Kria K26 SOM
# (KV260/KR260). For the Alveo U55C use xcu55c-fsvh2892-2L-e.
# =============================================================================
set root [file normalize [file join [file dirname [info script]] ..]]
set part "xck26-sfvc784-2LV-c"

create_project pcie_gen1_x1 [file join $root build vivado] -part $part -force

# ---- design sources ----
set rtl_files {}
foreach f [split [string trim [read [open [file join $root sim filelist_rtl.f]]]] "\n"] {
  lappend rtl_files [file join $root [string trim $f]]
}
add_files -norecurse -fileset sources_1 $rtl_files
set_property file_type SystemVerilog [get_files -of_objects [get_filesets sources_1]]
set_property top pcie_ep_top [get_filesets sources_1]

# ---- simulation sources ----
set tb_files [list \
  [file join $root tb models pcie_channel.sv] \
  [file join $root tb rc pcie_rc_bfm.sv] \
  [file join $root tb top tb_pcie_top.sv] \
  [file join $root tb unit tb_8b10b.sv] \
  [file join $root tb unit tb_crc_scrambler.sv] \
  [file join $root tb unit tb_phy_link.sv] ]
add_files -norecurse -fileset sim_1 $tb_files
set_property file_type SystemVerilog [get_files -of_objects [get_filesets sim_1] *.sv]
set_property top tb_pcie_top [get_filesets sim_1]
set_property top_lib xil_defaultlib [get_filesets sim_1]
set_property -name {xsim.simulate.runtime} -value {all} -objects [get_filesets sim_1]

update_compile_order -fileset sources_1
update_compile_order -fileset sim_1
puts "Project created: [file join $root build vivado pcie_gen1_x1.xpr]"
