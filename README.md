# FPGA-based-Motor-Control-IP

## FPGA-based motor control IP for industrial drives and EV applications.
## 📌 Overview
This project delivers a production-grade Motor Control IP targeting industrial drive systems and electric vehicle (EV) powertrain applications.
The architecture is fully modular — each submodule is independently testable — and the top-level integrates them into a complete closed-loop motor control system.

The IP was validated through:

  -> RTL simulation on Cadence NC Launch with a comprehensive 9-test testbench
  
  -> FPGA implementation on PYNQ-Z2 (Zynq-7000) and Spartan-7 via Xilinx Vivado (with IP packaging)
  
  -> ASIC flow through Cadence Genus (synthesis → netlist) and Cadence Innovus (P&R → GDSII)


## 🏗️ System Architecture

                      ┌──────────────────────────────────────────────────┐
                      │              motor_control_top                   │
                      │                                                  │
    speed_sw ────────►│  duty_frequency_controller  ──► three_phase_pwm  │
    mode_sel ────────►│  (V/Hz sinusoidal or direct)    (centre-aligned) │
                      │         │                             │          │
                      │   pid_controller ◄──── virtual_motor_model       │
                      │   (speed loop)          (plant simulation)       │
                      │                                 │                │
                      │   deadtime_inserter × 3 ◄───────┘                │
                      │   (500 ns dead-band)             │               │
                      │                          virtual_hall_generator  │
                      │   sensorless_controller ◄── sensor_interface     │
                      │   (BEMF zero-crossing)      (Hall + SPI ADC)     │
                      │                                                  │
                      │   fault_injector → protection_unit               │
                      │   torque_estimator                               │
                      │   UART telemetry (115200 baud)                   │
                      └──────────────────────────────────────────────────┘
  

## Block Diagram
<img width="1016" height="760" alt="image" src="https://github.com/user-attachments/assets/e77ebe07-43f6-4f51-8fcf-580dcc062e84" />


## Repository Structure
          FPGA-based-Motor-Control-IP/
          │
          ├── README.md
          │
          ├── rtl/
          │   ├── motor_control_top.v
          │   ├── duty_frequency_controller.v
          │   ├── pid_controller.v
          │   ├── three_phase_pwm.v
          │   ├── deadtime_inserter.v
          │   ├── virtual_motor_model.v
          │   ├── sensor_interface.v
          │   ├── protection_unit.v
          │   ├── sensorless_controller.v
          │   ├── torque_estimator.v
          │   ├── fault_injector.v
          │   ├── virtual_adc.v
          │   └── virtual_hall_generator.v
          │
          ├── tb/
          │   └── tb_motor_control_top.v
          │
          ├── wrapper/
          │   ├── boolean_top.v
          │   └── motor_control_wrapper.v
          │
          ├── constraints/
          │   ├── pynq_z2.xdc
          │   └── spartan7.xdc
          │
          ├── genus/
          │   ├── script.tcl
          │   └── constraint.sdc
          │
          ├── pnr/
          │   ├── clock_tree.png
          │   └── GDSII.png
          │
          ├── output/
          │   ├── waveform.png
          │   ├── spartan7_output.jpg
          │   └── pynq_z2_output.jpg
          │______ README.md


## 🔧 Module Descriptions

## motor_control_top.v

Top-level integrator. Instantiates all submodules, applies global PWM inhibit on fault, and implements an 8N1 UART TX state machine streaming 4-byte telemetry frames (fault code · speed · Irms · torque) continuously at 115200 baud.

## duty_frequency_controller.v

Generates three-phase sinusoidal duty cycles using a 256-entry sine LUT (inferred as BRAM). Supports V/Hz mode (angle accumulates per PWM sync pulse proportional to speed_cmd) and direct duty override mode via mode_sel.

## pid_controller.v

Fixed-point PID with configurable Kp/Ki/Kd, anti-windup clamping, and derivative-on-measurement (avoids derivative kick on setpoint step). Uses Q8.8 arithmetic with 32-bit accumulators.

## three_phase_pwm.v

Centre-aligned (up-down counter) PWM for three phases. Issues a sync_pulse at counter zero for angle accumulation. All duties clamped to period − 1 to prevent counter lock-up.

## deadtime_inserter.v

Per-phase dead-time insertion using a two-stage registered edge detector and 8-bit blanking counter. Both high-side and low-side are forced off for 500 ns on every switching edge — prevents shoot-through.

## virtual_motor_model.v

Behavioural BLDC plant model in Q24.8 fixed-point arithmetic. Simulates phase currents (di/dt), back-EMF, and rotor speed using Euler integration. Outputs 12-bit ADC-scaled values (mid-rail = 2048).

## sensorless_controller.v

BEMF zero-crossing detector with 8-tap boxcar filter and hysteresis. Uses coarse barrel-shift approximation for rotor angle interpolation between zero-crossing events — fully synthesisable without DSP48 dividers.

## protection_unit.v

Latching fault detection for OC / OV / OT / SC with priority encoding (SC > OC > OV > OT). 50 ms hardware lockout prevents premature re-enable. Fault code output for UART telemetry.

## fault_injector.v

Hardware fault injection engine with configurable pulse duration. Supports four fault scenarios selectable via fault_sel[3:0]. Used for validation of protection_unit response in simulation and on hardware.

## torque_estimator.v

Estimates torque (T = Kt × Irms), shaft power (P = Eb × Irms), and RMS current using three-phase current sum approximation. Single-cycle registered pipeline output.

## sensor_interface.v

Hall sensor debouncer (50-cycle filter) with sector decoder and inter-edge period measurement for speed estimation. SPI ADC capture state machine cycles through 3 channels (Phase A/B current, DC bus / temperature).

## virtual_adc.v · virtual_hall_generator.v

Loopback models enabling closed-loop simulation without physical hardware. Virtual ADC serialises parallel motor model outputs over SPI. Virtual Hall encoder maps rotor angle to 6-sector Hall patterns.


## 🖥️ Simulation — Cadence NC Launch
<img width="802" height="586" alt="image" src="https://github.com/user-attachments/assets/3635cfb9-aa93-4c34-8201-6f9a40c52886" /> 


## ⚙️ FPGA Implementation — Xilinx Vivado
## Boards supported

PYNQ-Z2 — Zynq-7000 SoC (xc7z020clg400-1)
Spartan-7 — (xc7s50csga324-1)

## Steps

Open Vivado → Create Project → Add all files from rtl/
Select target board and add matching .xdc from constraints/
Run Synthesis → Implementation → Generate Bitstream
Program board via Hardware Manager

## IP Packaging
The design is packaged as a reusable Vivado IP core (ip_package/component.xml), allowing drag-and-drop integration into any block design.


<img width="1631" height="906" alt="image" src="https://github.com/user-attachments/assets/2a0f9e9d-fd4c-475a-a93c-a02d6bf11d0b" />


## 🔬 ASIC Flow — Cadence Genus + Innovus (GDSII)
The IP was also taken through a full ASIC implementation flow:
## Synthesis — Cadence Genus
# genus_constraints.tcl (see synthesis/)
read_hdl rtl/*.v
elaborate motor_control_top
read_sdc  synthesis/constraints.sdc
synthesize -to_mapped
report area   > reports/genus_area_report.rpt
report power  > reports/genus_power_report.rpt
write_hdl     > synthesis/netlist/motor_ctrl_netlist.v

## Place & Route — Cadence Innovus
# innovus_floorplan.tcl (see pnr/)
read_netlist  synthesis/netlist/motor_ctrl_netlist.v
read_sdc      synthesis/constraints.sdc
init_design
floorPlan -site ...
place_design
route_design
streamOut pnr/gdsii/motor_ctrl_top.gds

<img width="891" height="829" alt="image" src="https://github.com/user-attachments/assets/ac6945d5-00b0-4f0b-b31c-3cf189b5f31d" />





