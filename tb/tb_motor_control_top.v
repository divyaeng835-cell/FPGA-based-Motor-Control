module tb_motor_control_top;
 
    // =========================================================================
    // Parameters
    // =========================================================================
    localparam CLK_PERIOD = 10;   // 10 ns = 100 MHz
 
    // =========================================================================
    // DUT signals
    // =========================================================================
    reg        clk             = 0;
    reg        rst_n           = 0;
    reg        motor_enable    = 0;
    reg        mode_sel        = 0;
    reg [7:0]  speed_sw        = 8'd0;
    reg        fault_clear_btn = 0;
    reg        inject_en       = 0;
    reg [3:0]  fault_inject_sel = 4'd0;
 
    wire       gate_ah, gate_al;
    wire       gate_bh, gate_bl;
    wire       gate_ch, gate_cl;
    wire [7:0] status_led;
    wire       uart_tx;
    wire [11:0] dbg_ia, dbg_ib, dbg_ic;
    wire [15:0] dbg_speed, dbg_torque;
    wire [3:0]  dbg_fault_code;
    wire        dbg_pwm_inh;
 
    // =========================================================================
    // DUT instantiation
    // =========================================================================
    motor_control_top #(
        .CLK_FREQ_HZ   (100_000_000),
        .PWM_FREQ_HZ   (20_000)
    ) dut (
        .clk              (clk),
        .rst_n            (rst_n),
        .motor_enable     (motor_enable),
        .mode_sel         (mode_sel),
        .speed_sw         (speed_sw),
        .fault_clear_btn  (fault_clear_btn),
        .inject_en        (inject_en),
        .fault_inject_sel (fault_inject_sel),
        .gate_ah (gate_ah), .gate_al (gate_al),
        .gate_bh (gate_bh), .gate_bl (gate_bl),
        .gate_ch (gate_ch), .gate_cl (gate_cl),
        .status_led       (status_led),
        .uart_tx          (uart_tx),
        .dbg_ia           (dbg_ia),
        .dbg_ib           (dbg_ib),
        .dbg_ic           (dbg_ic),
        .dbg_speed        (dbg_speed),
        .dbg_torque       (dbg_torque),
        .dbg_fault_code   (dbg_fault_code),
        .dbg_pwm_inh      (dbg_pwm_inh)
    );
 
    // =========================================================================
    // Clock generator
    // =========================================================================
    always #(CLK_PERIOD/2) clk = ~clk;
 
    // =========================================================================
    // Tasks
    // =========================================================================
    task wait_clk(input integer n);
        repeat(n) @(posedge clk);
    endtask
 
    task print_status(input [127:0] msg);
        $display("[%0t ns] %s | LED=%08b | Ia=%0d Ib=%0d Ic=%0d | Spd=%0d | Tq=%0d | FaultCode=%04b | Inhibit=%b",
            $time, msg, status_led, dbg_ia, dbg_ib, dbg_ic,
            dbg_speed, dbg_torque, dbg_fault_code, dbg_pwm_inh);
    endtask
 
    // =========================================================================
    // Test sequence
    // =========================================================================
    initial begin
        $dumpfile("motor_control_sim.vcd");
        $dumpvars(0, tb_motor_control_top);
 
        $display("========================================");
        $display(" Motor Control IP Testbench Start");
        $display("========================================");
 
        // -----------------------------------------------------------------
        // Test 1: Reset
        // -----------------------------------------------------------------
        rst_n        = 0;
        motor_enable = 0;
        speed_sw     = 8'd0;
        wait_clk(20);
 
        rst_n = 1;
        wait_clk(10);
        print_status("POST-RESET");
 
        // -----------------------------------------------------------------
        // Test 2: Enable motor, V/Hz mode, low speed
        // -----------------------------------------------------------------
        $display("\n--- TEST 2: Motor Enable, V/Hz mode @ low speed ---");
        motor_enable = 1;
        mode_sel     = 0;      // V/Hz
        speed_sw     = 8'd30;  // Low speed command
        wait_clk(500);
        print_status("VHz-LOW-SPD");
 
        // -----------------------------------------------------------------
        // Test 3: Speed ramp
        // -----------------------------------------------------------------
        $display("\n--- TEST 3: Speed ramp ---");
        repeat(5) begin
            speed_sw = speed_sw + 8'd20;
            wait_clk(200);
            print_status("RAMP");
        end
 
        // -----------------------------------------------------------------
        // Test 4: Direct duty override mode
        // -----------------------------------------------------------------
        $display("\n--- TEST 4: Direct duty override ---");
        mode_sel = 1;
        wait_clk(300);
        print_status("DIRECT-MODE");
 
        // Back to V/Hz
        mode_sel = 0;
        wait_clk(100);
 
        // -----------------------------------------------------------------
        // Test 5: Overcurrent fault injection
        // -----------------------------------------------------------------
        $display("\n--- TEST 5: Overcurrent fault injection ---");
        fault_inject_sel = 4'b0001;  // Overcurrent
        inject_en        = 1;
        wait_clk(10);
        inject_en        = 0;
        wait_clk(100);
        print_status("OC-FAULT");
 
        // Verify PWM inhibit
        if (dbg_pwm_inh)
            $display("PASS: PWM inhibited on overcurrent fault.");
        else
            $display("WARN: PWM not inhibited - check protection_unit thresholds.");
 
        // Check gate outputs are all zero
        if ({gate_ah, gate_al, gate_bh, gate_bl, gate_ch, gate_cl} == 6'b0)
            $display("PASS: All gates off during fault.");
        else
            $display("WARN: Gates not fully off.");
 
        // -----------------------------------------------------------------
        // Test 6: Fault clear
        // -----------------------------------------------------------------
        $display("\n--- TEST 6: Fault clear ---");
        wait_clk(600);  // Allow lockout to expire (5M cycles in real design, shortened for sim)
        fault_clear_btn = 1;
        wait_clk(5);
        fault_clear_btn = 0;
        wait_clk(100);
        print_status("AFTER-CLEAR");
 
        // -----------------------------------------------------------------
        // Test 7: Overvoltage injection
        // -----------------------------------------------------------------
        $display("\n--- TEST 7: Overvoltage fault ---");
        fault_inject_sel = 4'b0010;
        inject_en        = 1;
        wait_clk(10);
        inject_en        = 0;
        wait_clk(100);
        print_status("OV-FAULT");
 
        wait_clk(200);
        fault_clear_btn = 1;
        wait_clk(5);
        fault_clear_btn = 0;
        wait_clk(50);
 
        // -----------------------------------------------------------------
        // Test 8: Short-circuit injection (fastest protection path)
        // -----------------------------------------------------------------
        $display("\n--- TEST 8: Short-circuit fault ---");
        fault_inject_sel = 4'b1000;
        inject_en        = 1;
        wait_clk(5);
        inject_en        = 0;
        wait_clk(50);
        print_status("SC-FAULT");
 
        // -----------------------------------------------------------------
        // Test 9: Disable motor
        // -----------------------------------------------------------------
        $display("\n--- TEST 9: Motor disable ---");
        motor_enable = 0;
        wait_clk(100);
        print_status("DISABLED");
 
        if ({gate_ah, gate_bh, gate_ch} == 3'b000)
            $display("PASS: High-side gates off when disabled.");
 
        // -----------------------------------------------------------------
        // Done
        // -----------------------------------------------------------------
        $display("\n========================================");
        $display(" Simulation Complete");
        $display("========================================");
        $finish;
    end
 
    // Watchdog
    initial begin
        #50_000_000;
        $display("WATCHDOG TIMEOUT");
        $finish;
    end
 
endmodule  
