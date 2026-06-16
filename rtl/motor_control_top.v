module motor_control_top #(
    parameter CLK_FREQ_HZ = 100_000_000,
    parameter PWM_FREQ_HZ = 20_000,
    parameter ADC_WIDTH   = 12,
    parameter CNT_WIDTH   = 16,
    parameter DATA_WIDTH  = 16
)(
    // ── System ────────────────────────────────────────────────────────────────
    input  wire        clk,
    input  wire        rst_n,
 
    // ── Control inputs (PYNQ-Z2: SW0, SW1, BTN0-BTN3, Arduino header) ────────
    input  wire        motor_enable,       // SW0  - master enable
    input  wire        mode_sel,           // SW1  - 0=V/Hz  1=direct duty
    input  wire [7:0]  speed_sw,           // Arduino A0-A7 - speed command
    input  wire        fault_clear_btn,    // BTN1 - clear latched faults
    input  wire        inject_en,          // BTN2 - trigger fault injection
    input  wire [3:0]  fault_inject_sel,   // BTN3+SW0-SW2 - fault type select
 
    // ── Gate drive outputs (PMOD JA pins 1-6) ─────────────────────────────────
    output wire        gate_ah, gate_al,   // Phase A high/low side
    output wire        gate_bh, gate_bl,   // Phase B high/low side
    output wire        gate_ch, gate_cl,   // Phase C high/low side
 
    // ── Status (PYNQ-Z2: LD0-LD3 + LD4/LD5 RGB) ───────────────────────────────
    output wire [7:0]  status_led,
 
    // ── UART telemetry (PYNQ-Z2 USB-UART bridge pin) ─────────────────────────
    output wire        uart_tx,
 
    // ── ILA / debug probes ────────────────────────────────────────────────────
    output wire [11:0] dbg_ia,
    output wire [11:0] dbg_ib,
    output wire [11:0] dbg_ic,
    output wire [15:0] dbg_speed,
    output wire [15:0] dbg_torque,
    output wire [3:0]  dbg_fault_code,
    output wire        dbg_pwm_inh
);
 
    // =========================================================================
    // Local parameters  (FIX: no SystemVerilog cast - use localparam)
    // =========================================================================
    // PWM half-period: 100_000_000 / (2 * 20_000) = 2500 ticks
    localparam [CNT_WIDTH-1:0] PWM_HALF_PERIOD = CLK_FREQ_HZ / (2 * PWM_FREQ_HZ);
    localparam [7:0]           DT_CYCLES        = 8'd50;   // 500 ns dead-time @ 100 MHz
    localparam [CNT_WIDTH-1:0] BAUD_DIV         = CLK_FREQ_HZ / 115200; // ~868
 
    // =========================================================================
    // Internal wires
    // =========================================================================
 
    // PWM carrier outputs (before dead-time)
    wire pwm_a_h, pwm_b_h, pwm_c_h;
    wire pwm_sync;
 
    // Duty cycle / period from controller
    wire [CNT_WIDTH-1:0] duty_a_ctrl, duty_b_ctrl, duty_c_ctrl;
    wire [CNT_WIDTH-1:0] period_ctrl;
 
    // Speed command extended to CNT_WIDTH (FIX: clean zero-extension, no $signed)
    wire [CNT_WIDTH-1:0] speed_cmd_ext = {{(CNT_WIDTH-8){1'b0}}, speed_sw};
 
    // PID output
    wire signed [DATA_WIDTH-1:0] pid_output;
 
    // Protection
    wire        pwm_inhibit;
    wire [3:0]  fault_code;
    wire        fault_any;
    wire        fault_oc, fault_ov, fault_ot, fault_sc;
 
    // Hall / sensor interface
    wire [2:0]           hall_sector;
    wire [15:0]          hall_period;
    wire                 hall_valid;
    wire [ADC_WIDTH-1:0] adc_ch0, adc_ch1, adc_ch2;
 
    // Sensorless controller
    wire [15:0]          rotor_angle;
    wire [15:0]          rotor_speed;
    wire [2:0]           next_sector;
    wire                 zc_detected;
 
    // Torque estimator
    wire [23:0]          torque_est, power_est;
    wire [ADC_WIDTH-1:0] irms_est;
 
    // Virtual motor model
    wire [ADC_WIDTH-1:0] vm_ia, vm_ib, vm_ic;
    wire [ADC_WIDTH-1:0] vm_bemf_a, vm_bemf_b, vm_bemf_c;
    wire [DATA_WIDTH-1:0] vm_speed;
    wire [ADC_WIDTH-1:0] vm_vdc_half;
 
    // Virtual Hall
    wire vhall_a, vhall_b, vhall_c;
 
    // Virtual ADC SPI bus (internal loopback)
    wire vadc_sdo;
    wire sens_sck, sens_cs_n;
    wire [1:0] sens_ch_sel;
 
    // Fault injector outputs
    wire [ADC_WIDTH-1:0] fi_ia, fi_ib, fi_ic, fi_vbus, fi_temp;
    wire                 fi_active;
 
    // Raw gate signals before global inhibit
    wire g_ah_raw, g_al_raw;
    wire g_bh_raw, g_bl_raw;
    wire g_ch_raw, g_cl_raw;
 
    // =========================================================================
    // u_dfc : duty_frequency_controller
    // =========================================================================
    duty_frequency_controller #(
        .CNT_WIDTH   (CNT_WIDTH),
        .ANGLE_WIDTH (8)
    ) u_dfc (
        .clk            (clk),
        .rst_n          (rst_n),
        .enable         (motor_enable),
        .mode_sel       (mode_sel),
        .speed_cmd      (speed_cmd_ext),
        .pwm_period     (PWM_HALF_PERIOD),
        .modulation_idx (8'd200),
        .pwm_sync       (pwm_sync),
        .duty_a_in      ({CNT_WIDTH{1'b0}}),
        .duty_b_in      ({CNT_WIDTH{1'b0}}),
        .duty_c_in      ({CNT_WIDTH{1'b0}}),
        .period_out     (period_ctrl),
        .duty_a         (duty_a_ctrl),
        .duty_b         (duty_b_ctrl),
        .duty_c         (duty_c_ctrl)
    );
 
    // =========================================================================
    // u_pid : pid_controller  (speed loop)
    // =========================================================================
    pid_controller #(
        .DATA_WIDTH (DATA_WIDTH),
        .ACC_WIDTH  (32)
    ) u_pid (
        .clk       (clk),
        .rst_n     (rst_n),
        .enable    (motor_enable),
        .setpoint  ($signed(speed_cmd_ext)),
        .feedback  ($signed(vm_speed)),
        .kp        (16'd512),
        .ki        (16'd64),
        .kd        (16'd16),
        .out_min   (-16'sd1000),
        .out_max   ( 16'sd1000),
        .pid_out   (pid_output),
        .saturated ()
    );
 
    // =========================================================================
    // u_pwm : three_phase_pwm
    // =========================================================================
    three_phase_pwm #(
        .CNT_WIDTH (CNT_WIDTH)
    ) u_pwm (
        .clk        (clk),
        .rst_n      (rst_n),
        .enable     (motor_enable & ~pwm_inhibit),
        .period     (period_ctrl),
        .duty_a     (duty_a_ctrl),
        .duty_b     (duty_b_ctrl),
        .duty_c     (duty_c_ctrl),
        .pwm_a_h    (pwm_a_h),
        .pwm_b_h    (pwm_b_h),
        .pwm_c_h    (pwm_c_h),
        .sync_pulse (pwm_sync)
    );
 
    // =========================================================================
    // u_dt_a/b/c : deadtime_inserter  (one per phase)
    // =========================================================================
    deadtime_inserter #(.DT_CYCLES(DT_CYCLES)) u_dt_a (
        .clk             (clk),
        .rst_n           (rst_n),
        .deadtime_cycles (DT_CYCLES),
        .pwm_in          (pwm_a_h),
        .gate_h          (g_ah_raw),
        .gate_l          (g_al_raw)
    );
 
    deadtime_inserter #(.DT_CYCLES(DT_CYCLES)) u_dt_b (
        .clk             (clk),
        .rst_n           (rst_n),
        .deadtime_cycles (DT_CYCLES),
        .pwm_in          (pwm_b_h),
        .gate_h          (g_bh_raw),
        .gate_l          (g_bl_raw)
    );
 
    deadtime_inserter #(.DT_CYCLES(DT_CYCLES)) u_dt_c (
        .clk             (clk),
        .rst_n           (rst_n),
        .deadtime_cycles (DT_CYCLES),
        .pwm_in          (pwm_c_h),
        .gate_h          (g_ch_raw),
        .gate_l          (g_cl_raw)
    );
 
    // Global protection inhibit applied as AND mask
    assign gate_ah = g_ah_raw & ~pwm_inhibit;
    assign gate_al = g_al_raw & ~pwm_inhibit;
    assign gate_bh = g_bh_raw & ~pwm_inhibit;
    assign gate_bl = g_bl_raw & ~pwm_inhibit;
    assign gate_ch = g_ch_raw & ~pwm_inhibit;
    assign gate_cl = g_cl_raw & ~pwm_inhibit;
 
    // =========================================================================
    // u_motor : virtual_motor_model
    // =========================================================================
    virtual_motor_model #(
        .ADC_WIDTH  (ADC_WIDTH),
        .DATA_WIDTH (DATA_WIDTH),
        .L_SHIFT    (4),
        .J_SHIFT    (4)
    ) u_motor (
        .clk            (clk),
        .rst_n          (rst_n),
        .enable         (motor_enable),
        .gh_a           (gate_ah), .gl_a (gate_al),
        .gh_b           (gate_bh), .gl_b (gate_bl),
        .gh_c           (gate_ch), .gl_c (gate_cl),
        .vdc            (16'd3300),
        .r_scaled       (16'd26),
        .kt_scaled      (16'd128),
        .ia_out         (vm_ia),
        .ib_out         (vm_ib),
        .ic_out         (vm_ic),
        .bemf_a         (vm_bemf_a),
        .bemf_b         (vm_bemf_b),
        .bemf_c         (vm_bemf_c),
        .speed_out      (vm_speed),
        .vdc_half_out   (vm_vdc_half)
    );
 
    // =========================================================================
    // u_fi : fault_injector
    // =========================================================================
    fault_injector #(
        .ADC_WIDTH       (ADC_WIDTH),
        .INJECT_DURATION (16'd500)
    ) u_fi (
        .clk           (clk),
        .rst_n         (rst_n),
        .inject_en     (inject_en),
        .fault_sel     (fault_inject_sel),
        .duration      (16'd5000),
        .ia_in         (vm_ia),
        .ib_in         (vm_ib),
        .ic_in         (vm_ic),
        .vbus_in       (12'd206),        // FIX: explicit 12-bit constant (~33V scaled)
        .temp_in       (12'd1200),
        .ia_out        (fi_ia),
        .ib_out        (fi_ib),
        .ic_out        (fi_ic),
        .vbus_out      (fi_vbus),
        .temp_out      (fi_temp),
        .inject_active (fi_active)
    );
 
    // =========================================================================
    // u_vadc : virtual_adc  (internal SPI loopback)
    // =========================================================================
    virtual_adc #(.ADC_WIDTH(ADC_WIDTH)) u_vadc (
        .clk      (clk),
        .rst_n    (rst_n),
        .ch0_data (fi_ia),
        .ch1_data (fi_ib),
        .ch2_data (fi_ic),
        .sck      (sens_sck),
        .cs_n     (sens_cs_n),
        .ch_sel   (sens_ch_sel),
        .sdo      (vadc_sdo)
    );
 
    // =========================================================================
    // u_vhall : virtual_hall_generator
    // =========================================================================
    virtual_hall_generator u_vhall (
        .clk         (clk),
        .rst_n       (rst_n),
        .rotor_angle (rotor_angle),
        .hall_a      (vhall_a),
        .hall_b      (vhall_b),
        .hall_c      (vhall_c)
    );
 
    // =========================================================================
    // u_sens : sensor_interface
    // =========================================================================
    sensor_interface #(
        .ADC_WIDTH      (ADC_WIDTH),
        .DEBOUNCE_TICKS (8'd50)
    ) u_sens (
        .clk         (clk),
        .rst_n       (rst_n),
        .hall_a      (vhall_a),
        .hall_b      (vhall_b),
        .hall_c      (vhall_c),
        .adc_sdo     (vadc_sdo),
        .adc_sck     (sens_sck),
        .adc_cs_n    (sens_cs_n),
        .adc_ch_sel  (sens_ch_sel),
        .hall_sector (hall_sector),
        .hall_period (hall_period),
        .hall_valid  (hall_valid),
        .adc_ch0     (adc_ch0),
        .adc_ch1     (adc_ch1),
        .adc_ch2     (adc_ch2)
    );
 
    // =========================================================================
    // u_prot : protection_unit
    // =========================================================================
    protection_unit #(
        .ADC_WIDTH      (ADC_WIDTH),
        .LOCKOUT_CYCLES (24'd5_000_000)
    ) u_prot (
        .clk              (clk),
        .rst_n            (rst_n),
        .phase_a_current  (fi_ia),
        .phase_b_current  (fi_ib),
        .phase_c_current  (fi_ic),
        .dc_bus_voltage   (fi_vbus),
        .temperature      (fi_temp),
        .oc_threshold     (12'd3400),
        .ov_threshold     (12'd3800),
        .temp_threshold   (12'd3500),
        .sc_threshold     (12'd3900),
        .fault_clear      (fault_clear_btn),
        .pwm_inhibit      (pwm_inhibit),
        .fault_oc         (fault_oc),
        .fault_ov         (fault_ov),
        .fault_ot         (fault_ot),
        .fault_sc         (fault_sc),
        .fault_any        (fault_any),
        .fault_code       (fault_code)
    );
 
    // =========================================================================
    // u_sens_ctrl : sensorless_controller
    // =========================================================================
    sensorless_controller #(
        .ADC_WIDTH   (ADC_WIDTH),
        .ANGLE_WIDTH (16)
    ) u_sens_ctrl (
        .clk         (clk),
        .rst_n       (rst_n),
        .enable      (motor_enable),
        .va          (vm_bemf_a),
        .vb          (vm_bemf_b),
        .vc          (vm_bemf_c),
        .vdc_half    (vm_vdc_half),
        .sector_in   (hall_sector),
        .rotor_angle (rotor_angle),
        .rotor_speed (rotor_speed),
        .next_sector (next_sector),
        .zc_detected (zc_detected)
    );
 
    // =========================================================================
    // u_torq : torque_estimator
    // =========================================================================
    torque_estimator #(
        .ADC_WIDTH (ADC_WIDTH),
        .OUT_WIDTH (24)
    ) u_torq (
        .clk        (clk),
        .rst_n      (rst_n),
        .enable     (motor_enable),
        .ia         (vm_ia),
        .ib         (vm_ib),
        .ic         (vm_ic),
        .bemf_mag   (vm_bemf_a),
        .kt_scaled  (16'd128),
        .torque_est (torque_est),
        .power_est  (power_est),
        .irms_est   (irms_est)
    );
 
    // =========================================================================
    // Status LED mapping (8 bits)
    //  [7] motor_enable  [6] fault_any  [5] fi_active
    //  [4] hall_valid    [3] zc_detected  [2:0] hall_sector
    // =========================================================================
    assign status_led = {
        motor_enable,
        fault_any,
        fi_active,
        hall_valid,
        zc_detected,
        hall_sector[2],
        hall_sector[1],
        hall_sector[0]
    };
 
    // =========================================================================
    // ILA debug probes
    // =========================================================================
    assign dbg_ia         = vm_ia;
    assign dbg_ib         = vm_ib;
    assign dbg_ic         = vm_ic;
    assign dbg_speed      = rotor_speed;
    assign dbg_torque     = torque_est[15:0];
    assign dbg_fault_code = fault_code;
    assign dbg_pwm_inh    = pwm_inhibit;
 
    // =========================================================================
    // UART TX  -  8N1 @ 115200 baud
    // Continuously streams 4-byte telemetry: fault_code | speed | irms | torque
    // =========================================================================
    reg [9:0]  uart_shift;
    reg [15:0] baud_cnt;
    reg [3:0]  uart_bit_cnt;
    reg        uart_busy;
    reg [1:0]  tx_byte_sel;
    reg [7:0]  tx_data_r;
 
    assign uart_tx = uart_busy ? uart_shift[0] : 1'b1;
 
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            uart_shift    <= 10'h3FF;
            baud_cnt      <= 16'd0;
            uart_bit_cnt  <= 4'd0;
            uart_busy     <= 1'b0;
            tx_byte_sel   <= 2'd0;
            tx_data_r     <= 8'd0;
        end else begin
            if (!uart_busy) begin
                case (tx_byte_sel)
                    2'd0: tx_data_r <= {4'd0,   fault_code};
                    2'd1: tx_data_r <= vm_speed[7:0];
                    2'd2: tx_data_r <= irms_est[7:0];
                    2'd3: tx_data_r <= torque_est[7:0];
                endcase
                tx_byte_sel  <= tx_byte_sel + 1'b1;
                uart_shift   <= {1'b1, tx_data_r, 1'b0};  // stop, data[7:0], start
                uart_bit_cnt <= 4'd10;
                baud_cnt     <= 16'd0;
                uart_busy    <= 1'b1;
            end else begin
                if (baud_cnt == BAUD_DIV - 1'b1) begin
                    baud_cnt     <= 16'd0;
                    uart_shift   <= {1'b1, uart_shift[9:1]};
                    uart_bit_cnt <= uart_bit_cnt - 1'b1;
                    if (uart_bit_cnt == 4'd1)
                        uart_busy <= 1'b0;
                end else begin
                    baud_cnt <= baud_cnt + 1'b1;
                end
            end
        end
    end
 
endmodule
