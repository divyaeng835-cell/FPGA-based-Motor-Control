
module duty_frequency_controller #(
    parameter CNT_WIDTH   = 16,
    parameter ANGLE_WIDTH = 8      // 256 steps per electrical cycle
)(
    input  wire                      clk,
    input  wire                      rst_n,
    input  wire                      enable,
 
    // 0 = V/Hz sinusoidal mode, 1 = direct duty override
    input  wire                      mode_sel,
 
    // V/Hz inputs
    input  wire [CNT_WIDTH-1:0]      speed_cmd,      // step_inc per PWM period (0..255)
    input  wire [CNT_WIDTH-1:0]      pwm_period,     // PWM half-period ticks
    input  wire [7:0]                modulation_idx, // 0..255 = 0..100% modulation
 
    // PWM sync pulse: advance angle once per carrier period
    input  wire                      pwm_sync,
 
    // Direct override duty cycles
    input  wire [CNT_WIDTH-1:0]      duty_a_in,
    input  wire [CNT_WIDTH-1:0]      duty_b_in,
    input  wire [CNT_WIDTH-1:0]      duty_c_in,
 
    // Outputs
    output reg  [CNT_WIDTH-1:0]      period_out,
    output reg  [CNT_WIDTH-1:0]      duty_a,
    output reg  [CNT_WIDTH-1:0]      duty_b,
    output reg  [CNT_WIDTH-1:0]      duty_c
);
 
    // =========================================================================
    // Sine ROM: 256 × 8-bit unsigned  (sin*127 + 128)
    // Vivado infers BRAM when rom_style = "block"
    // =========================================================================
    (* rom_style = "block" *)
    reg [7:0] sine_lut [0:255];
    integer i;
    initial begin
        for (i = 0; i < 256; i = i + 1)
            sine_lut[i] = $rtoi($sin(2.0 * 3.14159265358979 * i / 256.0) * 127.0 + 128.0);
    end
 
    // =========================================================================
    // Angle accumulator - advances ONCE per PWM sync pulse
    // =========================================================================
    reg [ANGLE_WIDTH-1:0] angle_a;
    reg                   sync_prev;
    wire                  sync_rise = pwm_sync & ~sync_prev;
 
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            angle_a   <= 8'd0;
            sync_prev <= 1'b0;
        end else begin
            sync_prev <= pwm_sync;
            if (enable && !mode_sel && sync_rise)
                angle_a <= angle_a + speed_cmd[7:0];
        end
    end
 
    // 120° and 240° offsets in 256-step circle
    wire [ANGLE_WIDTH-1:0] angle_b = angle_a + 8'd85;
    wire [ANGLE_WIDTH-1:0] angle_c = angle_a + 8'd171;
 
    // Registered LUT read (1-cycle latency, clean for BRAM)
    reg [7:0] sine_a_r, sine_b_r, sine_c_r;
    always @(posedge clk) begin
        sine_a_r <= sine_lut[angle_a];
        sine_b_r <= sine_lut[angle_b];
        sine_c_r <= sine_lut[angle_c];
    end
 
    // =========================================================================
    // Scale: duty = (sine * modulation_idx * period) / 65025
    // Approximation: duty = (sine * mod * period) >> 16
    // FIX: use 25-bit intermediate to avoid truncation (8+8+16=32 internal)
    // =========================================================================
    wire [31:0] da_raw = (sine_a_r * modulation_idx * pwm_period) >> 16;
    wire [31:0] db_raw = (sine_b_r * modulation_idx * pwm_period) >> 16;
    wire [31:0] dc_raw = (sine_c_r * modulation_idx * pwm_period) >> 16;
 
    // =========================================================================
    // Output register + mode mux
    // =========================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            period_out <= {CNT_WIDTH{1'b0}};
            duty_a     <= {CNT_WIDTH{1'b0}};
            duty_b     <= {CNT_WIDTH{1'b0}};
            duty_c     <= {CNT_WIDTH{1'b0}};
        end else if (enable) begin
            period_out <= pwm_period;
            if (mode_sel) begin
                duty_a <= duty_a_in;
                duty_b <= duty_b_in;
                duty_c <= duty_c_in;
            end else begin
                // Clamp to period-1 to protect PWM counter
                duty_a <= (da_raw[CNT_WIDTH-1:0] >= pwm_period) ?
                           pwm_period - 1'b1 : da_raw[CNT_WIDTH-1:0];
                duty_b <= (db_raw[CNT_WIDTH-1:0] >= pwm_period) ?
                           pwm_period - 1'b1 : db_raw[CNT_WIDTH-1:0];
                duty_c <= (dc_raw[CNT_WIDTH-1:0] >= pwm_period) ?
                           pwm_period - 1'b1 : dc_raw[CNT_WIDTH-1:0];
            end
        end
    end
 
endmodule  
