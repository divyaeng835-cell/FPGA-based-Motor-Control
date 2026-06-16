cat > constraint.sdc << 'EOF'
#---------------------------------------------------------
# SDC CONSTRAINTS
# Design : motor_control_top
# Tool   : Cadence Genus 90nm
#---------------------------------------------------------

# Clock 100 MHz — period 10 ns
create_clock \
    -name clk \
    -period 10.0 \
    -waveform {0 5} \
    [get_ports clk]

set_clock_uncertainty -setup 0.1  [get_clocks clk]
set_clock_uncertainty -hold  0.05 [get_clocks clk]
set_clock_transition         0.1  [get_clocks clk]

# Input delays
set_input_delay -clock clk -max 2.0 [get_ports rst_n]
set_input_delay -clock clk -min 0.5 [get_ports rst_n]
set_input_delay -clock clk -max 2.0 [get_ports motor_enable]
set_input_delay -clock clk -min 0.5 [get_ports motor_enable]
set_input_delay -clock clk -max 2.0 [get_ports mode_sel]
set_input_delay -clock clk -min 0.5 [get_ports mode_sel]
set_input_delay -clock clk -max 2.0 [get_ports speed_sw]
set_input_delay -clock clk -min 0.5 [get_ports speed_sw]
set_input_delay -clock clk -max 2.0 [get_ports fault_clear_btn]
set_input_delay -clock clk -min 0.5 [get_ports fault_clear_btn]
set_input_delay -clock clk -max 2.0 [get_ports inject_en]
set_input_delay -clock clk -min 0.5 [get_ports inject_en]
set_input_delay -clock clk -max 2.0 [get_ports fault_inject_sel]
set_input_delay -clock clk -min 0.5 [get_ports fault_inject_sel]

# Output delays gate signals
set_output_delay -clock clk -max 2.0 [get_ports gate_ah]
set_output_delay -clock clk -min 0.5 [get_ports gate_ah]
set_output_delay -clock clk -max 2.0 [get_ports gate_al]
set_output_delay -clock clk -min 0.5 [get_ports gate_al]
set_output_delay -clock clk -max 2.0 [get_ports gate_bh]
set_output_delay -clock clk -min 0.5 [get_ports gate_bh]
set_output_delay -clock clk -max 2.0 [get_ports gate_bl]
set_output_delay -clock clk -min 0.5 [get_ports gate_bl]
set_output_delay -clock clk -max 2.0 [get_ports gate_ch]
set_output_delay -clock clk -min 0.5 [get_ports gate_ch]
set_output_delay -clock clk -max 2.0 [get_ports gate_cl]
set_output_delay -clock clk -min 0.5 [get_ports gate_cl]

# Output delays status and debug
set_output_delay -clock clk -max 4.0 [get_ports status_led]
set_output_delay -clock clk -min 0.0 [get_ports status_led]
set_output_delay -clock clk -max 4.0 [get_ports uart_tx]
set_output_delay -clock clk -min 0.0 [get_ports uart_tx]
set_output_delay -clock clk -max 4.0 [get_ports dbg_ia]
set_output_delay -clock clk -min 0.0 [get_ports dbg_ia]
set_output_delay -clock clk -max 4.0 [get_ports dbg_ib]
set_output_delay -clock clk -min 0.0 [get_ports dbg_ib]
set_output_delay -clock clk -max 4.0 [get_ports dbg_ic]
set_output_delay -clock clk -min 0.0 [get_ports dbg_ic]
set_output_delay -clock clk -max 4.0 [get_ports dbg_speed]
set_output_delay -clock clk -min 0.0 [get_ports dbg_speed]
set_output_delay -clock clk -max 4.0 [get_ports dbg_torque]
set_output_delay -clock clk -min 0.0 [get_ports dbg_torque]
set_output_delay -clock clk -max 4.0 [get_ports dbg_fault_code]
set_output_delay -clock clk -min 0.0 [get_ports dbg_fault_code]
set_output_delay -clock clk -max 4.0 [get_ports dbg_pwm_inh]
set_output_delay -clock clk -min 0.0 [get_ports dbg_pwm_inh]

# False paths async inputs
set_false_path -from [get_ports rst_n]
set_false_path -from [get_ports fault_clear_btn]
set_false_path -from [get_ports inject_en]

# Multicycle path PID integrator
set_multicycle_path -setup 4 \
    -from [get_cells u_pid/integrator_reg*]
set_multicycle_path -hold  3 \
    -from [get_cells u_pid/integrator_reg*]

# Max fanout and transition
set_max_fanout    32  [current_design]
set_max_transition 0.5 [current_design]
EOF
