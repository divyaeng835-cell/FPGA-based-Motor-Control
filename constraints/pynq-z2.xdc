############################################################
# PYNQ-Z2 Constraints for motor_control_wrapper
############################################################

########################
# BUTTON INPUTS
########################

# fault_clear_btn_0 -> BTN1
set_property PACKAGE_PIN D20 [get_ports fault_clear_btn_0]
set_property IOSTANDARD LVCMOS33 [get_ports fault_clear_btn_0]

# inject_en_0 -> BTN2
set_property PACKAGE_PIN L20 [get_ports inject_en_0]
set_property IOSTANDARD LVCMOS33 [get_ports inject_en_0]

# mode_sel_0 -> SW1
set_property PACKAGE_PIN M19 [get_ports mode_sel_0]
set_property IOSTANDARD LVCMOS33 [get_ports mode_sel_0]

# motor_enable_0 -> SW0
set_property PACKAGE_PIN M20 [get_ports motor_enable_0]
set_property IOSTANDARD LVCMOS33 [get_ports motor_enable_0]

############################################################
# STATUS LEDs (ONLY 4 LEDs EXIST ON PYNQ-Z2)
############################################################

set_property PACKAGE_PIN R14 [get_ports {status_led_0[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {status_led_0[0]}]

set_property PACKAGE_PIN P14 [get_ports {status_led_0[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {status_led_0[1]}]

set_property PACKAGE_PIN N16 [get_ports {status_led_0[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {status_led_0[2]}]

set_property PACKAGE_PIN M14 [get_ports {status_led_0[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {status_led_0[3]}]

############################################################
# PWM OUTPUTS -> PMODA
############################################################

# gate_ah_0
set_property PACKAGE_PIN Y18 [get_ports gate_ah_0]
set_property IOSTANDARD LVCMOS33 [get_ports gate_ah_0]

# gate_al_0
set_property PACKAGE_PIN Y19 [get_ports gate_al_0]
set_property IOSTANDARD LVCMOS33 [get_ports gate_al_0]

# gate_bh_0
set_property PACKAGE_PIN Y16 [get_ports gate_bh_0]
set_property IOSTANDARD LVCMOS33 [get_ports gate_bh_0]

# gate_bl_0
set_property PACKAGE_PIN Y17 [get_ports gate_bl_0]
set_property IOSTANDARD LVCMOS33 [get_ports gate_bl_0]

# gate_ch_0
set_property PACKAGE_PIN U18 [get_ports gate_ch_0]
set_property IOSTANDARD LVCMOS33 [get_ports gate_ch_0]

# gate_cl_0
set_property PACKAGE_PIN U19 [get_ports gate_cl_0]
set_property IOSTANDARD LVCMOS33 [get_ports gate_cl_0]

############################################################
# PWM INHIBIT DEBUG
############################################################

set_property PACKAGE_PIN W18 [get_ports dbg_pwm_inh_0]
set_property IOSTANDARD LVCMOS33 [get_ports dbg_pwm_inh_0]

############################################################
# UART TX
############################################################

set_property PACKAGE_PIN W19 [get_ports uart_tx_0]
set_property IOSTANDARD LVCMOS33 [get_ports uart_tx_0]

############################################################
# OPTIONAL: Relax DRC (NOT RECOMMENDED FOR FINAL DESIGN)
############################################################
# set_property SEVERITY {Warning} [get_drc_checks NSTD-1]
# set_property SEVERITY {Warning} [get_drc_checks UCIO-1]
