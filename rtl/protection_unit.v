module protection_unit #(
    parameter ADC_WIDTH       = 12,
    parameter LOCKOUT_CYCLES  = 24'd5_000_000  // ~50ms @ 100MHz lockout
)(
    input  wire                  clk,
    input  wire                  rst_n,
 
    // ADC sampled values (unsigned, full-scale = max voltage/current/temp)
    input  wire [ADC_WIDTH-1:0]  phase_a_current,
    input  wire [ADC_WIDTH-1:0]  phase_b_current,
    input  wire [ADC_WIDTH-1:0]  phase_c_current,
    input  wire [ADC_WIDTH-1:0]  dc_bus_voltage,
    input  wire [ADC_WIDTH-1:0]  temperature,
 
    // Configurable thresholds
    input  wire [ADC_WIDTH-1:0]  oc_threshold,     // Overcurrent limit
    input  wire [ADC_WIDTH-1:0]  ov_threshold,     // Overvoltage limit
    input  wire [ADC_WIDTH-1:0]  temp_threshold,   // Over-temperature limit
    input  wire [ADC_WIDTH-1:0]  sc_threshold,     // Short-circuit (fast trip)
 
    // Manual fault clear (active high pulse)
    input  wire                  fault_clear,
 
    // Protection outputs
    output reg                   pwm_inhibit,      // Immediate PWM disable
    output reg                   fault_oc,         // Overcurrent fault
    output reg                   fault_ov,         // Overvoltage fault
    output reg                   fault_ot,         // Over-temperature fault
    output reg                   fault_sc,         // Short-circuit fault
    output reg                   fault_any,        // OR of all faults
    output reg [3:0]             fault_code        // Encoded fault type
);
 
    // -------------------------------------------------------------------------
    // Combinational detection (one clock latency for registered outputs)
    // -------------------------------------------------------------------------
    wire oc_detect = (phase_a_current > oc_threshold) ||
                     (phase_b_current > oc_threshold) ||
                     (phase_c_current > oc_threshold);
 
    wire ov_detect = (dc_bus_voltage  > ov_threshold);
    wire ot_detect = (temperature     > temp_threshold);
    wire sc_detect = (phase_a_current > sc_threshold)  ||
                     (phase_b_current > sc_threshold)  ||
                     (phase_c_current > sc_threshold);
 
    // -------------------------------------------------------------------------
    // Lockout counter - holds fault active for LOCKOUT_CYCLES after clear
    // -------------------------------------------------------------------------
    reg [23:0] lockout_cnt;
    reg        lockout_active;
 
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lockout_cnt    <= 24'd0;
            lockout_active <= 1'b0;
        end else begin
            if (fault_any && !lockout_active) begin
                lockout_active <= 1'b1;
                lockout_cnt    <= LOCKOUT_CYCLES;
            end else if (lockout_active) begin
                if (fault_clear) begin
                    lockout_active <= 1'b0;
                    lockout_cnt    <= 24'd0;
                end else if (lockout_cnt != 24'd0) begin
                    lockout_cnt <= lockout_cnt - 1'b1;
                end
            end
        end
    end
 
    // -------------------------------------------------------------------------
    // Registered fault outputs
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fault_oc    <= 1'b0;
            fault_ov    <= 1'b0;
            fault_ot    <= 1'b0;
            fault_sc    <= 1'b0;
            fault_any   <= 1'b0;
            fault_code  <= 4'h0;
            pwm_inhibit <= 1'b0;
        end else begin
            if (fault_clear && !lockout_active) begin
                // Clear faults only when lockout expired
                fault_oc    <= 1'b0;
                fault_ov    <= 1'b0;
                fault_ot    <= 1'b0;
                fault_sc    <= 1'b0;
                fault_any   <= 1'b0;
                fault_code  <= 4'h0;
                pwm_inhibit <= 1'b0;
            end else begin
                // Latch faults (sticky until cleared)
                if (sc_detect) fault_sc <= 1'b1;
                if (oc_detect) fault_oc <= 1'b1;
                if (ov_detect) fault_ov <= 1'b1;
                if (ot_detect) fault_ot <= 1'b1;
 
                fault_any   <= fault_sc | fault_oc | fault_ov | fault_ot |
                               sc_detect | oc_detect | ov_detect | ot_detect;
                pwm_inhibit <= fault_sc | fault_oc | fault_ov | fault_ot |
                               sc_detect | oc_detect | ov_detect | ot_detect;
 
                // Priority encoding: SC > OC > OV > OT
                if      (sc_detect || fault_sc) fault_code <= 4'h8;
                else if (oc_detect || fault_oc) fault_code <= 4'h4;
                else if (ov_detect || fault_ov) fault_code <= 4'h2;
                else if (ot_detect || fault_ot) fault_code <= 4'h1;
                else                            fault_code <= 4'h0;
            end
        end
    end
 
endmodule
