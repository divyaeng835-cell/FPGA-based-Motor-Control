module torque_estimator #(
    parameter ADC_WIDTH  = 12,
    parameter OUT_WIDTH  = 24   // Output torque/power width
)(
    input  wire                     clk,
    input  wire                     rst_n,
    input  wire                     enable,
 
    // Motor currents (unsigned ADC counts)
    input  wire [ADC_WIDTH-1:0]     ia,      // Phase A current
    input  wire [ADC_WIDTH-1:0]     ib,      // Phase B current
    input  wire [ADC_WIDTH-1:0]     ic,      // Phase C current
 
    // Estimated back-EMF (from sensorless controller or ADC)
    input  wire [ADC_WIDTH-1:0]     bemf_mag,
 
    // Motor torque constant Kt (Q8.8 fixed-point: Kt * 256)
    input  wire [15:0]              kt_scaled,
 
    // Outputs
    output reg  [OUT_WIDTH-1:0]     torque_est,    // T = Kt * Irms (Q8.8 * ADC)
    output reg  [OUT_WIDTH-1:0]     power_est,     // P = Eb * Irms (ADC * ADC)
    output reg  [ADC_WIDTH-1:0]     irms_est       // RMS current estimate
);
 
    // -------------------------------------------------------------------------
    // RMS current approximation: Irms ≈ (|Ia| + |Ib| + |Ic|) / 3
    // For BLDC with 120-deg conduction, only two phases conduct at a time
    // but we average all three for generality.
    // -------------------------------------------------------------------------
    wire [ADC_WIDTH+1:0] isum = ia + ib + ic;
    wire [ADC_WIDTH-1:0] iavg = isum[ADC_WIDTH+1:2]; // Divide by ~3 (use >>2 approx)
 
    // -------------------------------------------------------------------------
    // Torque: T = Kt * Irms  (result in Q8 format)
    // -------------------------------------------------------------------------
    wire [OUT_WIDTH-1:0] torque_raw = kt_scaled * iavg;
 
    // -------------------------------------------------------------------------
    // Power: P = Eb * Irms
    // -------------------------------------------------------------------------
    wire [OUT_WIDTH-1:0] power_raw = bemf_mag * iavg;
 
    // -------------------------------------------------------------------------
    // Register outputs with 1-cycle pipeline
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            torque_est <= {OUT_WIDTH{1'b0}};
            power_est  <= {OUT_WIDTH{1'b0}};
            irms_est   <= {ADC_WIDTH{1'b0}};
        end else if (enable) begin
            torque_est <= torque_raw;
            power_est  <= power_raw;
            irms_est   <= iavg;
        end
    end
 
endmodule
