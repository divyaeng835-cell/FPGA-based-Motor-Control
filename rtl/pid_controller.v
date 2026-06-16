
module pid_controller #(
    parameter DATA_WIDTH = 16,
    parameter ACC_WIDTH  = 32
)(
    input  wire                          clk,
    input  wire                          rst_n,
    input  wire                          enable,
 
    input  wire signed [DATA_WIDTH-1:0]  setpoint,
    input  wire signed [DATA_WIDTH-1:0]  feedback,
 
    input  wire        [DATA_WIDTH-1:0]  kp,
    input  wire        [DATA_WIDTH-1:0]  ki,
    input  wire        [DATA_WIDTH-1:0]  kd,
 
    input  wire signed [DATA_WIDTH-1:0]  out_min,
    input  wire signed [DATA_WIDTH-1:0]  out_max,
 
    output reg  signed [DATA_WIDTH-1:0]  pid_out,
    output reg                           saturated
);
 
    wire signed [DATA_WIDTH-1:0] error = setpoint - feedback;
 
    reg signed [DATA_WIDTH-1:0] error_r;
    reg signed [DATA_WIDTH-1:0] feedback_prev;
 
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            error_r       <= {DATA_WIDTH{1'b0}};
            feedback_prev <= {DATA_WIDTH{1'b0}};
        end else if (enable) begin
            error_r       <= error;
            feedback_prev <= feedback;
        end
    end
 
    // FIX: $signed cast on gains for correct signed*signed multiply
    wire signed [ACC_WIDTH-1:0] p_term =
        $signed({{1'b0}, kp}) * error_r;
 
    reg signed [ACC_WIDTH-1:0] integrator;
 
    wire signed [ACC_WIDTH-1:0] integrator_next =
        integrator + ($signed({{1'b0}, ki}) * error_r);
 
    wire aw_clamp = saturated &&
                   (((pid_out >= out_max) && (error_r > 0)) ||
                    ((pid_out <= out_min) && (error_r < 0)));
 
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            integrator <= {ACC_WIDTH{1'b0}};
        else if (enable && !aw_clamp)
            integrator <= integrator_next;
    end
 
    wire signed [DATA_WIDTH-1:0] d_input = feedback_prev - feedback;
    wire signed [ACC_WIDTH-1:0]  d_term  =
        $signed({{1'b0}, kd}) * d_input;
 
    wire signed [ACC_WIDTH-1:0] sum_raw =
        (p_term + integrator + d_term) >>> 8;
 
    // FIX: proper sign-extension for saturation limits
    wire signed [ACC_WIDTH-1:0] max_ext =
        {{(ACC_WIDTH-DATA_WIDTH){out_max[DATA_WIDTH-1]}}, out_max};
    wire signed [ACC_WIDTH-1:0] min_ext =
        {{(ACC_WIDTH-DATA_WIDTH){out_min[DATA_WIDTH-1]}}, out_min};
 
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pid_out   <= {DATA_WIDTH{1'b0}};
            saturated <= 1'b0;
        end else if (enable) begin
            if (sum_raw > max_ext) begin
                pid_out   <= out_max;
                saturated <= 1'b1;
            end else if (sum_raw < min_ext) begin
                pid_out   <= out_min;
                saturated <= 1'b1;
            end else begin
                pid_out   <= sum_raw[DATA_WIDTH-1:0];
                saturated <= 1'b0;
            end
        end
    end
 
endmodule   
