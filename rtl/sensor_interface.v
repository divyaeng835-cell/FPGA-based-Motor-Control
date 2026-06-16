module sensor_interface #(
    parameter DEBOUNCE_TICKS = 8'd50,   // Hall debounce filter cycles
    parameter ADC_WIDTH      = 12
)(
    input  wire        clk,
    input  wire        rst_n,
 
    // Hall sensor inputs (raw from FPGA pin / virtual hall generator)
    input  wire        hall_a,
    input  wire        hall_b,
    input  wire        hall_c,
 
    // SPI-like ADC interface (virtual or real)
    // Each ADC channel clocked out serially MSB-first
    input  wire        adc_sdo,        // Serial data out from ADC
    output reg         adc_sck,        // ADC clock
    output reg         adc_cs_n,       // ADC chip select (active low)
    output reg  [1:0]  adc_ch_sel,     // Channel select (for mux)
 
    // Decoded outputs
    output reg  [2:0]  hall_sector,    // Motor electrical sector 0..5
    output reg  [15:0] hall_period,    // Ticks between Hall edges (speed)
    output reg         hall_valid,     // High when Hall pattern is valid
 
    // ADC captured values
    output reg  [ADC_WIDTH-1:0] adc_ch0,  // Phase A current
    output reg  [ADC_WIDTH-1:0] adc_ch1,  // Phase B current / DC bus
    output reg  [ADC_WIDTH-1:0] adc_ch2   // Temperature / Phase C
);
 
    // =========================================================================
    // HALL SENSOR DECODER
    // =========================================================================
    // Standard BLDC Hall truth table (120-degree commutation):
    //  H[C,B,A]: 001=0, 011=1, 010=2, 110=3, 100=4, 101=5
    // =========================================================================
 
    // Debounce
    reg [7:0]  db_cnt;
    reg [2:0]  hall_sync, hall_stable, hall_prev;
 
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hall_sync   <= 3'd0;
            hall_stable <= 3'd0;
            hall_prev   <= 3'd0;
            db_cnt      <= 8'd0;
        end else begin
            // Two-stage sync for metastability
            hall_sync <= {hall_c, hall_b, hall_a};
            if (hall_sync != hall_stable) begin
                if (db_cnt == DEBOUNCE_TICKS) begin
                    hall_prev   <= hall_stable;
                    hall_stable <= hall_sync;
                    db_cnt      <= 8'd0;
                end else begin
                    db_cnt <= db_cnt + 1'b1;
                end
            end else begin
                db_cnt <= 8'd0;
            end
        end
    end
 
    // Hall pattern to sector decode
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hall_sector <= 3'd0;
            hall_valid  <= 1'b0;
        end else begin
            case (hall_stable)
                3'b001: begin hall_sector <= 3'd0; hall_valid <= 1'b1; end
                3'b011: begin hall_sector <= 3'd1; hall_valid <= 1'b1; end
                3'b010: begin hall_sector <= 3'd2; hall_valid <= 1'b1; end
                3'b110: begin hall_sector <= 3'd3; hall_valid <= 1'b1; end
                3'b100: begin hall_sector <= 3'd4; hall_valid <= 1'b1; end
                3'b101: begin hall_sector <= 3'd5; hall_valid <= 1'b1; end
                default: begin hall_sector <= 3'd0; hall_valid <= 1'b0; end
            endcase
        end
    end
 
    // Speed measurement: count clocks between any Hall edge
    reg [15:0] speed_cnt;
    wire hall_edge = (hall_stable != hall_prev);
 
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            speed_cnt   <= 16'd0;
            hall_period <= 16'hFFFF;
        end else begin
            if (hall_edge) begin
                hall_period <= speed_cnt;
                speed_cnt   <= 16'd0;
            end else begin
                speed_cnt <= (speed_cnt == 16'hFFFF) ? speed_cnt : speed_cnt + 1'b1;
            end
        end
    end
 
    // =========================================================================
    // SIMPLE SERIAL ADC CAPTURE (SPI Mode 0, 12-bit, multiplexed)
    // State machine cycles through 3 channels
    // =========================================================================
    localparam ADC_IDLE   = 2'd0,
               ADC_SELECT = 2'd1,
               ADC_SHIFT  = 2'd2,
               ADC_STORE  = 2'd3;
 
    reg [1:0]  adc_state;
    reg [4:0]  adc_bit_cnt;
    reg [ADC_WIDTH-1:0] adc_shift;
    reg [1:0]  ch_cnt;
 
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            adc_state   <= ADC_IDLE;
            adc_bit_cnt <= 5'd0;
            adc_shift   <= {ADC_WIDTH{1'b0}};
            adc_sck     <= 1'b0;
            adc_cs_n    <= 1'b1;
            adc_ch_sel  <= 2'd0;
            ch_cnt      <= 2'd0;
            adc_ch0     <= {ADC_WIDTH{1'b0}};
            adc_ch1     <= {ADC_WIDTH{1'b0}};
            adc_ch2     <= {ADC_WIDTH{1'b0}};
        end else begin
            case (adc_state)
                ADC_IDLE: begin
                    adc_cs_n    <= 1'b1;
                    adc_sck     <= 1'b0;
                    adc_ch_sel  <= ch_cnt;
                    adc_state   <= ADC_SELECT;
                end
 
                ADC_SELECT: begin
                    adc_cs_n    <= 1'b0;   // Assert CS
                    adc_bit_cnt <= 5'd12;
                    adc_shift   <= {ADC_WIDTH{1'b0}};
                    adc_state   <= ADC_SHIFT;
                end
 
                ADC_SHIFT: begin
                    adc_sck <= ~adc_sck;
                    if (adc_sck) begin
                        // Sample on rising edge
                        adc_shift   <= {adc_shift[ADC_WIDTH-2:0], adc_sdo};
                        adc_bit_cnt <= adc_bit_cnt - 1'b1;
                        if (adc_bit_cnt == 5'd1) begin
                            adc_state <= ADC_STORE;
                        end
                    end
                end
 
                ADC_STORE: begin
                    adc_cs_n <= 1'b1;
                    adc_sck  <= 1'b0;
                    case (ch_cnt)
                        2'd0: adc_ch0 <= adc_shift;
                        2'd1: adc_ch1 <= adc_shift;
                        2'd2: adc_ch2 <= adc_shift;
                        default: ;
                    endcase
                    ch_cnt    <= (ch_cnt == 2'd2) ? 2'd0 : ch_cnt + 1'b1;
                    adc_state <= ADC_IDLE;
                end
            endcase
        end
    end
 
endmodule  
