module deadtime_inserter #(
    parameter DT_CYCLES = 8'd50
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] deadtime_cycles,
    input  wire       pwm_in,
    output reg        gate_h,
    output reg        gate_l
);
 
    reg pwm_d1, pwm_d2;
    reg [7:0] dt_cnt;
    reg       in_blanking;
 
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_d1      <= 1'b0;
            pwm_d2      <= 1'b0;
            dt_cnt      <= 8'd0;
            in_blanking <= 1'b0;
            gate_h      <= 1'b0;
            gate_l      <= 1'b0;       // FIX: was undefined at reset
        end else begin
            pwm_d1 <= pwm_in;
            pwm_d2 <= pwm_d1;
 
            // Edge on pwm_d1 vs pwm_d2 (both registered, no latch)
            if (pwm_d1 != pwm_d2) begin
                in_blanking <= 1'b1;
                dt_cnt      <= deadtime_cycles;
                gate_h      <= 1'b0;
                gate_l      <= 1'b0;
            end else if (in_blanking) begin
                if (dt_cnt == 8'd0) begin
                    in_blanking <= 1'b0;
                    gate_h      <= pwm_d2;
                    gate_l      <= ~pwm_d2;
                end else begin
                    dt_cnt <= dt_cnt - 1'b1;
                    gate_h <= 1'b0;
                    gate_l <= 1'b0;
                end
            end else begin
                gate_h <= pwm_d2;
                gate_l <= ~pwm_d2;
            end
        end
    end
 
endmodule   
