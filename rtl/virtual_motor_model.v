`timescale 1ns / 1ps
 
module virtual_motor_model #(
    parameter ADC_WIDTH  = 12,
    parameter DATA_WIDTH = 16,
    // L and J represented as right-shift amounts (divide by 2^N)
    // L_SHIFT=4 → L_eff = 1/16; J_SHIFT=4 → J_eff = 1/16
    parameter L_SHIFT    = 4,
    parameter J_SHIFT    = 4
)(
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire                   enable,
 
    // Gate drive inputs (high-side / low-side per phase)
    input  wire                   gh_a, gl_a,
    input  wire                   gh_b, gl_b,
    input  wire                   gh_c, gl_c,
 
    // Motor parameters (constant inputs, tie to literals in top)
    input  wire [DATA_WIDTH-1:0]  vdc,           // DC bus (ADC counts scale)
    input  wire [DATA_WIDTH-1:0]  r_scaled,      // R * 256
    input  wire [DATA_WIDTH-1:0]  kt_scaled,     // Kt * 256
 
    // Simulated sensor outputs (12-bit unsigned, mid-rail = 2048)
    output reg  [ADC_WIDTH-1:0]   ia_out,
    output reg  [ADC_WIDTH-1:0]   ib_out,
    output reg  [ADC_WIDTH-1:0]   ic_out,
    output reg  [ADC_WIDTH-1:0]   bemf_a,
    output reg  [ADC_WIDTH-1:0]   bemf_b,
    output reg  [ADC_WIDTH-1:0]   bemf_c,
    output reg  [DATA_WIDTH-1:0]  speed_out,
    output reg  [ADC_WIDTH-1:0]   vdc_half_out
);
 
    // =========================================================================
    // Phase voltage from gate states: Van = ±Vdc/2 or 0
    // =========================================================================
    wire signed [DATA_WIDTH:0] vhalf = $signed({1'b0, vdc[DATA_WIDTH-1:1]});
 
    reg signed [DATA_WIDTH:0] van, vbn, vcn;
    always @(*) begin
        van = gh_a ? vhalf : (gl_a ? -vhalf : {(DATA_WIDTH+1){1'b0}});
        vbn = gh_b ? vhalf : (gl_b ? -vhalf : {(DATA_WIDTH+1){1'b0}});
        vcn = gh_c ? vhalf : (gl_c ? -vhalf : {(DATA_WIDTH+1){1'b0}});
    end
 
    // =========================================================================
    // State registers (Q24.8 fixed-point: value = reg / 256)
    // =========================================================================
    reg signed [31:0] ia_s, ib_s, ic_s;
    reg signed [31:0] omega_s;
 
    // BEMF = Kt * omega / 256  (both in Q8 so divide by 256)
        // BEMF = Kt * omega / 256  (both in Q8 so divide by 256)
    // Pipeline stage: register ea_s to break the multiply out of the
    // same-cycle chain that feeds ia_s/ib_s/ic_s. Shared by all 3 phases.
    wire signed [31:0] ea_comb = ($signed({1'b0, kt_scaled}) * omega_s) >>> 16;
    reg  signed [31:0] ea_s;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            ea_s <= 32'sd0;
        else
            ea_s <= ea_comb;
    end
    wire signed [31:0] eb_s = ea_s;   // simplified symmetric model
    wire signed [31:0] ec_s = ea_s;
 
    // Resistive drop: R * ia / 256
        // Resistive drop: R * ia / 256
    // Pipeline stage: register vr_a/b/c to break the multiply out of the
    // same-cycle chain that feeds back into ia_s/ib_s/ic_s.
    wire signed [31:0] vr_a_comb = ($signed({1'b0, r_scaled}) * ia_s) >>> 8;
    wire signed [31:0] vr_b_comb = ($signed({1'b0, r_scaled}) * ib_s) >>> 8;
    wire signed [31:0] vr_c_comb = ($signed({1'b0, r_scaled}) * ic_s) >>> 8;
    reg  signed [31:0] vr_a, vr_b, vr_c;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vr_a <= 32'sd0;
            vr_b <= 32'sd0;
            vr_c <= 32'sd0;
        end else begin
            vr_a <= vr_a_comb;
            vr_b <= vr_b_comb;
            vr_c <= vr_c_comb;
        end
    end
 
    // di/dt numerator: (V - E - R*i)  then divide by L using shift
    wire signed [31:0] dnum_a = $signed({{15{van[DATA_WIDTH]}}, van}) - ea_s - vr_a;
    wire signed [31:0] dnum_b = $signed({{15{vbn[DATA_WIDTH]}}, vbn}) - eb_s - vr_b;
    wire signed [31:0] dnum_c = $signed({{15{vcn[DATA_WIDTH]}}, vcn}) - ec_s - vr_c;
 
    // Divide by L (shift right by L_SHIFT) then integration step (>>4)
    wire signed [31:0] dia = dnum_a >>> (L_SHIFT + 4);
    wire signed [31:0] dib = dnum_b >>> (L_SHIFT + 4);
    wire signed [31:0] dic = dnum_c >>> (L_SHIFT + 4);
 
    // Torque = Kt * Iavg / 256
    reg signed [31:0] isum_s;
    always @(posedge clk or negedge rst_n) begin
      if (!rst_n)
        isum_s <= 32'sd0;
      else
        isum_s <= ia_s + ib_s + ic_s;
      end
    // Stage 1: compute and register iavg_s (breaks the two multiplies apart)
      reg signed [31:0] iavg_r;
      always @(posedge clk or negedge rst_n) begin
         if (!rst_n)
           iavg_r <= 32'sd0;
         else
           iavg_r <= (isum_s * 32'sd21845) >>> 16;  // ≈ isum_s / 3
      end

// Stage 2: torque and domega now use the registered iavg_r (only ONE multiply per cycle)
      // Pipeline stage: register torque_s so the multiply doesn't chain
// directly into the omega_s add + clamp in the same cycle.
      wire signed [31:0] torque_comb = ($signed({1'b0, kt_scaled}) * iavg_r) >>> 8;
      reg  signed [31:0] torque_s;
      always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
           torque_s <= 32'sd0;
        else
           torque_s <= torque_comb;
      end
      wire signed [31:0] domega = torque_s >>> (J_SHIFT + 4);
    // Saturation limits (±8A scaled: 8*256 = 2048)
    localparam signed [31:0] IMAX =  32'sh00000800;
    localparam signed [31:0] IMIN = -32'sh00000800;
    localparam signed [31:0] WMAX =  32'sh00010000;
 
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ia_s    <= 32'sd0;
            ib_s    <= 32'sd0;
            ic_s    <= 32'sd0;
            omega_s <= 32'sd0;
        end else if (enable) begin
            // Integrate and clamp - single always block, no multi-driver
            ia_s    <= ($signed(ia_s + dia) > IMAX) ? IMAX :
                       ($signed(ia_s + dia) < IMIN) ? IMIN : ia_s + dia;
            ib_s    <= ($signed(ib_s + dib) > IMAX) ? IMAX :
                       ($signed(ib_s + dib) < IMIN) ? IMIN : ib_s + dib;
            ic_s    <= ($signed(ic_s + dic) > IMAX) ? IMAX :
                       ($signed(ic_s + dic) < IMIN) ? IMIN : ic_s + dic;
            omega_s <= ($signed(omega_s + domega) < 0) ? 32'sd0 :
                       ($signed(omega_s + domega) > WMAX) ? WMAX :
                       omega_s + domega;
        end
    end
 
    // =========================================================================
    // Output scaling: ADC mid-rail = 2048, range 0..4095
    // current_adc = clamp(2048 + ia_s/256, 0, 4095)
    // =========================================================================
    function automatic [ADC_WIDTH-1:0] to_adc;
        input signed [31:0] val_q8;
        reg signed [31:0] v;
        begin
            v = 32'sd2048 + (val_q8 >>> 8);
            if (v > 32'sd4095)     to_adc = {ADC_WIDTH{1'b1}};
            else if (v < 32'sd0)   to_adc = {ADC_WIDTH{1'b0}};
            else                   to_adc = v[ADC_WIDTH-1:0];
        end
    endfunction
 
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ia_out       <= 12'd2048;
            ib_out       <= 12'd2048;
            ic_out       <= 12'd2048;
            bemf_a       <= 12'd2048;
            bemf_b       <= 12'd2048;
            bemf_c       <= 12'd2048;
            speed_out    <= 16'd0;
            vdc_half_out <= 12'd2048;
        end else begin
            ia_out       <= to_adc(ia_s);
            ib_out       <= to_adc(ib_s);
            ic_out       <= to_adc(ic_s);
            bemf_a       <= to_adc(ea_s <<< 4);  // scale bemf up for visibility
            bemf_b       <= to_adc(eb_s <<< 4);
            bemf_c       <= to_adc(ec_s <<< 4);
            speed_out    <= omega_s[15:0];
            vdc_half_out <= vdc[DATA_WIDTH-1 : DATA_WIDTH-ADC_WIDTH];
        end
    end
 
endmodule   
