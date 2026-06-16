module virtual_adc #(
    parameter ADC_WIDTH = 12
)(
    input  wire                      clk,
    input  wire                      rst_n,
 
    // Parallel inputs from virtual motor model
    input  wire [ADC_WIDTH-1:0]      ch0_data,   // Phase A current
    input  wire [ADC_WIDTH-1:0]      ch1_data,   // Phase B current / DC bus
    input  wire [ADC_WIDTH-1:0]      ch2_data,   // Temperature / Phase C
 
    // SPI interface TO sensor_interface (simulating ADC hardware)
    input  wire                      sck,         // From sensor_interface
    input  wire                      cs_n,        // Chip select from sensor_interface
    input  wire [1:0]                ch_sel,      // Channel select from sensor_interface
    output reg                       sdo          // Serial data out
);
 
    // =========================================================================
    // Latch selected channel data on falling CS edge
    // =========================================================================
    reg [ADC_WIDTH-1:0] latch_data;
    reg [3:0]           bit_ptr;
    reg                 sck_prev;
 
    // Channel multiplexer
    wire [ADC_WIDTH-1:0] ch_mux = (ch_sel == 2'd0) ? ch0_data :
                                   (ch_sel == 2'd1) ? ch1_data :
                                   (ch_sel == 2'd2) ? ch2_data :
                                                      {ADC_WIDTH{1'b0}};
 
    // Detect CS falling edge
    reg cs_n_prev;
    wire cs_fall = cs_n_prev & ~cs_n;
 
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cs_n_prev  <= 1'b1;
            sck_prev   <= 1'b0;
            latch_data <= {ADC_WIDTH{1'b0}};
            bit_ptr    <= 4'd11;
            sdo        <= 1'b0;
        end else begin
            cs_n_prev <= cs_n;
            sck_prev  <= sck;
 
            if (cs_fall) begin
                // Latch data at start of transaction
                latch_data <= ch_mux;
                bit_ptr    <= 4'd11;  // Start at MSB
            end else if (!cs_n) begin
                // Detect falling edge of SCK from sensor_interface
                // (sensor samples on rising, we shift on falling)
                if (sck_prev && !sck) begin
                    sdo     <= latch_data[bit_ptr];
                    bit_ptr <= (bit_ptr == 4'd0) ? 4'd11 : bit_ptr - 1'b1;
                end
            end else begin
                sdo <= 1'b0;
            end
        end
    end
 
endmodule
