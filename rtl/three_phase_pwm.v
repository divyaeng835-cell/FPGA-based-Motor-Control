module three_phase_pwm #(
    parameter CNT_WIDTH  = 16,
    parameter PERIOD_DEF = 16'd2500
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  enable,
 
    input  wire [CNT_WIDTH-1:0]  period,
    input  wire [CNT_WIDTH-1:0]  duty_a,
    input  wire [CNT_WIDTH-1:0]  duty_b,
    input  wire [CNT_WIDTH-1:0]  duty_c,
 
    output reg                   pwm_a_h,
    output reg                   pwm_b_h,
    output reg                   pwm_c_h,
    output reg                   sync_pulse
);
 
    // Clamp duties to period to prevent counter lock-up
    wire [CNT_WIDTH-1:0] da = (duty_a >= period) ? period - 1'b1 : duty_a;
    wire [CNT_WIDTH-1:0] db = (duty_b >= period) ? period - 1'b1 : duty_b;
    wire [CNT_WIDTH-1:0] dc = (duty_c >= period) ? period - 1'b1 : duty_c;
 
    reg [CNT_WIDTH-1:0] cnt;
    reg                 cnt_dir;
 
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt     <= {CNT_WIDTH{1'b0}};
            cnt_dir <= 1'b0;
        end else if (!enable) begin
            cnt     <= {CNT_WIDTH{1'b0}};
            cnt_dir <= 1'b0;
        end else begin
            if (!cnt_dir) begin                          // counting up
                if (cnt >= period) begin
                    cnt_dir <= 1'b1;
                    cnt     <= period - 1'b1;
                end else begin
                    cnt <= cnt + 1'b1;
                end
            end else begin                               // counting down
                if (cnt == {CNT_WIDTH{1'b0}}) begin
                    cnt_dir <= 1'b0;
                    cnt     <= {{(CNT_WIDTH-1){1'b0}}, 1'b1}; // FIX: proper width
                end else begin
                    cnt <= cnt - 1'b1;
                end
            end
        end
    end
 
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_a_h <= 1'b0; pwm_b_h <= 1'b0; pwm_c_h <= 1'b0;
            sync_pulse <= 1'b0;
        end else if (!enable) begin
            pwm_a_h <= 1'b0; pwm_b_h <= 1'b0; pwm_c_h <= 1'b0;
            sync_pulse <= 1'b0;
        end else begin
            pwm_a_h    <= (cnt < da);
            pwm_b_h    <= (cnt < db);
            pwm_c_h    <= (cnt < dc);
            sync_pulse <= (cnt == {CNT_WIDTH{1'b0}});
        end
    end
 
endmodule                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         `timescale 1ns / 1ps
