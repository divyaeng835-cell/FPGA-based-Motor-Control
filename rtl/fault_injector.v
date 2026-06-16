module fault_injector #(
    parameter ADC_WIDTH = 12,
    parameter INJECT_DURATION = 16'd500  // Fault pulse width in clocks
)(
    input  wire                      clk,
    input  wire                      rst_n,
 
    // Fault control
    input  wire                      inject_en,   // Enable fault injection
    input  wire [3:0]                fault_sel,   // Which fault to inject:
                                                  //  0001 = overcurrent
                                                  //  0010 = overvoltage
                                                  //  0100 = over-temperature
                                                  //  1000 = phase short-circuit
    // Injection duration override (0 = use default)
    input  wire [15:0]               duration,
 
    // Pass-through inputs (from virtual motor model)
    input  wire [ADC_WIDTH-1:0]      ia_in,
    input  wire [ADC_WIDTH-1:0]      ib_in,
    input  wire [ADC_WIDTH-1:0]      ic_in,
    input  wire [ADC_WIDTH-1:0]      vbus_in,
    input  wire [ADC_WIDTH-1:0]      temp_in,
 
    // Injected outputs (to protection_unit)
    output reg  [ADC_WIDTH-1:0]      ia_out,
    output reg  [ADC_WIDTH-1:0]      ib_out,
    output reg  [ADC_WIDTH-1:0]      ic_out,
    output reg  [ADC_WIDTH-1:0]      vbus_out,
    output reg  [ADC_WIDTH-1:0]      temp_out,
 
    // Status
    output reg                       inject_active
);
 
    // =========================================================================
    // Pulse counter for fault duration
    // =========================================================================
    reg [15:0] cnt;
    wire [15:0] dur_use = (duration == 16'd0) ? INJECT_DURATION : duration;
 
    // Override values for each fault type
    localparam OC_INJECT_VAL = 12'hFFF;  // Max current → overcurrent
    localparam OV_INJECT_VAL = 12'hFFF;  // Max voltage → overvoltage
    localparam OT_INJECT_VAL = 12'hFFF;  // Max temp   → over-temp
    localparam SC_INJECT_VAL = 12'hFFF;  // Max current → short-circuit
 
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt            <= 16'd0;
            inject_active  <= 1'b0;
            ia_out         <= {ADC_WIDTH{1'b0}};
            ib_out         <= {ADC_WIDTH{1'b0}};
            ic_out         <= {ADC_WIDTH{1'b0}};
            vbus_out       <= {ADC_WIDTH{1'b0}};
            temp_out       <= {ADC_WIDTH{1'b0}};
        end else begin
            if (inject_en && !inject_active) begin
                // Start injection
                inject_active <= 1'b1;
                cnt           <= dur_use;
            end else if (inject_active) begin
                if (cnt == 16'd0) begin
                    inject_active <= 1'b0;
                end else begin
                    cnt <= cnt - 1'b1;
                end
            end
 
            // Mux: inject or passthrough
            if (inject_active) begin
                case (fault_sel)
                    4'b0001: begin  // Overcurrent
                        ia_out   <= OC_INJECT_VAL;
                        ib_out   <= OC_INJECT_VAL;
                        ic_out   <= OC_INJECT_VAL;
                        vbus_out <= vbus_in;
                        temp_out <= temp_in;
                    end
                    4'b0010: begin  // Overvoltage
                        ia_out   <= ia_in;
                        ib_out   <= ib_in;
                        ic_out   <= ic_in;
                        vbus_out <= OV_INJECT_VAL;
                        temp_out <= temp_in;
                    end
                    4'b0100: begin  // Over-temperature
                        ia_out   <= ia_in;
                        ib_out   <= ib_in;
                        ic_out   <= ic_in;
                        vbus_out <= vbus_in;
                        temp_out <= OT_INJECT_VAL;
                    end
                    4'b1000: begin  // Short-circuit (extreme current spike)
                        ia_out   <= SC_INJECT_VAL;
                        ib_out   <= SC_INJECT_VAL;
                        ic_out   <= SC_INJECT_VAL;
                        vbus_out <= vbus_in;
                        temp_out <= temp_in;
                    end
                    default: begin
                        ia_out   <= ia_in;
                        ib_out   <= ib_in;
                        ic_out   <= ic_in;
                        vbus_out <= vbus_in;
                        temp_out <= temp_in;
                    end
                endcase
            end else begin
                ia_out   <= ia_in;
                ib_out   <= ib_in;
                ic_out   <= ic_in;
                vbus_out <= vbus_in;
                temp_out <= temp_in;
            end
        end
    end
 
endmodule
