#---------------------------------------------------------
# MOTOR CONTROL TOP SYNTHESIS SCRIPT
# Cadence Genus - 90nm Technology
# Reference: sample alu_system script structure
#---------------------------------------------------------

#---------------------------------------------------------
# SET LIBRARY PATH
#---------------------------------------------------------
set_db init_lib_search_path \
    /cadence/install/FOUNDRY-01/digital/90nm/dig/lib

#---------------------------------------------------------
# READ LIBRARY
#---------------------------------------------------------
read_libs slow.lib

#---------------------------------------------------------
# READ RTL FILES
# Submodules first, top module last
#---------------------------------------------------------
read_hdl -sv duty_frequency_controller.v
read_hdl -sv pid_controller.v
read_hdl -sv three_phase_pwm.v
read_hdl -sv deadtime_inserter.v
read_hdl -sv virtual_motor_model.v
read_hdl -sv fault_injector.v
read_hdl -sv virtual_adc.v
read_hdl -sv virtual_hall_generator.v
read_hdl -sv sensor_interface.v
read_hdl -sv protection_unit.v
read_hdl -sv sensorless_controller.v
read_hdl -sv torque_estimator.v
read_hdl -sv motor_control_top.v

#---------------------------------------------------------
# ELABORATE TOP MODULE
#---------------------------------------------------------
elaborate motor_control_top

#---------------------------------------------------------
# CONSTRAINTS
# File is named constraint.sdc in your directory
#---------------------------------------------------------
read_sdc ./constraint.sdc

#---------------------------------------------------------
# SYNTHESIS
#---------------------------------------------------------
syn_gen
syn_map
syn_opt

#---------------------------------------------------------
# REPORTS
#---------------------------------------------------------
report_area   > area.rpt
report_timing > timing.rpt
report_power  > power.rpt
report_qor    > qor.rpt

#---------------------------------------------------------
# OUTPUT NETLIST
#---------------------------------------------------------
write_hdl > motor_control_netlist.v

#---------------------------------------------------------
# OUTPUT SDC
#---------------------------------------------------------
write_sdc > motor_control_postsyn.sdc

#---------------------------------------------------------
# DONE
#---------------------------------------------------------
puts "Synthesis complete"
puts "Netlist : motor_control_netlist.v"
puts "SDC     : motor_control_postsyn.sdc"
puts "Reports : area.rpt timing.rpt power.rpt qor.rpt"
