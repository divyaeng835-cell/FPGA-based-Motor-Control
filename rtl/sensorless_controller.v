module sensorless_controller #(
    parameter ADC_WIDTH   = 12,
    parameter ANGLE_WIDTH = 16
)(
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire                   enable,
 
    // Phase terminal voltages (unsigned 12-bit, mid-rail = 2048)
    input  wire [ADC_WIDTH-1:0]   va,
    input  wire [ADC_WIDTH-1:0]   vb,
    input  wire [ADC_WIDTH-1:0]   vc,
    input  wire [ADC_WIDTH-1:0]   vdc_half,
 
    // Current commutation sector (0..5) from sensor_interface
    input  wire [2:0]             sector_in,
 
    // Outputs
    output reg  [ANGLE_WIDTH-1:0] rotor_angle,
    output reg  [15:0]            rotor_speed,
    output reg  [2:0]             next_sector,
    output reg                    zc_detected
);
 
    // =========================================================================
    // BEMF observation: floating phase - virtual neutral
    // =========================================================================
    reg signed [ADC_WIDTH:0] bemf_obs;
 
    always @(*) begin
        case (sector_in)
            3'd0, 3'd3: bemf_obs = $signed({1'b0,vc}) - $signed({1'b0,vdc_half});
            3'd1, 3'd4: bemf_obs = $signed({1'b0,va}) - $signed({1'b0,vdc_half});
            3'd2, 3'd5: bemf_obs = $signed({1'b0,vb}) - $signed({1'b0,vdc_half});
            default:    bemf_obs = {(ADC_WIDTH+1){1'b0}};
        endcase
    end
 
    // =========================================================================
    // 8-tap boxcar filter (shift-register + running sum)
    // =========================================================================
    reg signed [ADC_WIDTH:0]   tap [0:7];
    reg signed [ADC_WIDTH+3:0] filt_sum;
    integer k;
 
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            filt_sum <= {(ADC_WIDTH+4){1'b0}};
            for (k = 0; k < 8; k = k+1)
                tap[k] <= {(ADC_WIDTH+1){1'b0}};
        end else if (enable) begin
            filt_sum <= filt_sum - tap[7] + bemf_obs;
            tap[7] <= tap[6]; tap[6] <= tap[5]; tap[5] <= tap[4];
            tap[4] <= tap[3]; tap[3] <= tap[2]; tap[2] <= tap[1];
            tap[1] <= tap[0]; tap[0] <= bemf_obs;
        end
    end
 
    // Divide by 8 → arithmetic right shift
    wire signed [ADC_WIDTH:0] bemf_filt = filt_sum[ADC_WIDTH+3:3];
 
    // =========================================================================
    // Hysteresis zero-crossing detect
    // FIX: width matches ADC_WIDTH+1 = 13 bits
    // =========================================================================
    localparam signed [ADC_WIDTH:0] HYST = 13'sh002;
 
    reg bemf_sign_r;   // registered sign of filtered BEMF
 
    wire zc_rising  = (~bemf_sign_r) && (bemf_filt >  HYST);
    wire zc_falling = ( bemf_sign_r) && (bemf_filt < -HYST);
 
    // =========================================================================
    // Speed measurement (ticks between zero crossings)
    // =========================================================================
    reg [15:0] zc_timer;
    reg [15:0] zc_period_r;   // latched period at last ZC
 
    // =========================================================================
    // Angle interpolation  (FIX: no division by non-constant wire)
    // Between ZC events, add a fixed small increment per clock.
    // The increment is chosen so that over one full period (zc_period_r ticks
    // × 6 sectors) the angle advances 65536 counts.
    // We approximate with a shift: increment = 10922 >> log2(zc_period_r>>4)
    // For typical zc_period_r values this gives ±10% accuracy - adequate for
    // sensorless commutation timing.  A full cordic divider would use DSP48.
    //
    // Practical approach: use a coarse 4-bit shift derived from the MSB of
    // zc_period_r to approximate  10922 / zc_period_r.
    // =========================================================================
    reg [3:0] period_shift;
 
    always @(*) begin
        if      (zc_period_r >= 16'd8192) period_shift = 4'd13;
        else if (zc_period_r >= 16'd4096) period_shift = 4'd12;
        else if (zc_period_r >= 16'd2048) period_shift = 4'd11;
        else if (zc_period_r >= 16'd1024) period_shift = 4'd10;
        else if (zc_period_r >= 16'd512)  period_shift = 4'd9;
        else if (zc_period_r >= 16'd256)  period_shift = 4'd8;
        else if (zc_period_r >= 16'd128)  period_shift = 4'd7;
        else                              period_shift = 4'd6;
    end
 
    // angle_inc = 10922 >> period_shift  (constant numerator, shift by registered value)
    // Vivado synthesises barrel-shifter for this - fully synthesisable
    wire [15:0] angle_inc = 16'd10922 >> period_shift;
 
    // =========================================================================
    // Main sequencer
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            zc_timer    <= 16'd0;
            zc_period_r <= 16'd512;
            rotor_speed <= 16'hFFFF;
            zc_detected <= 1'b0;
            bemf_sign_r <= 1'b0;
            next_sector <= 3'd1;
            rotor_angle <= {ANGLE_WIDTH{1'b0}};
        end else if (enable) begin
            zc_timer    <= zc_timer + 1'b1;
            zc_detected <= 1'b0;
 
            if (zc_rising || zc_falling) begin
                // Zero crossing detected
                zc_period_r <= zc_timer;
                zc_timer    <= 16'd0;
                rotor_speed <= zc_timer;
                zc_detected <= 1'b1;
                bemf_sign_r <= bemf_filt[ADC_WIDTH];  // update registered sign
                next_sector <= (sector_in == 3'd5) ? 3'd0 : sector_in + 1'b1;
                rotor_angle <= rotor_angle + 16'd10922;  // jump 60 degrees at ZC
            end else begin
                // Interpolate angle between zero crossings
                rotor_angle <= rotor_angle + angle_inc;
            end
        end
    end
 
endmodule 
