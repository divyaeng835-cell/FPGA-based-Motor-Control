module virtual_hall_generator (
    input  wire        clk,
    input  wire        rst_n,
 
    // Rotor electrical angle (0..65535 = 0..360 degrees)
    input  wire [15:0] rotor_angle,
 
    // Simulated Hall outputs (connect to sensor_interface hall inputs)
    output reg         hall_a,
    output reg         hall_b,
    output reg         hall_c
);
 
    // Each sector = 65536 / 6 = 10922 angle counts wide
    // Sector boundaries:
    //   S0:     0 ..10921  → {C,B,A} = 001
    //   S1: 10922 ..21844  → 011
    //   S2: 21845 ..32767  → 010
    //   S3: 32768 ..43690  → 110
    //   S4: 43691 ..54613  → 100
    //   S5: 54614 ..65535  → 101
 
    wire [2:0] hall_pattern;
 
    assign hall_pattern =
        (rotor_angle < 16'd10922)                              ? 3'b001 :
        (rotor_angle < 16'd21845)                              ? 3'b011 :
        (rotor_angle < 16'd32768)                              ? 3'b010 :
        (rotor_angle < 16'd43691)                              ? 3'b110 :
        (rotor_angle < 16'd54614)                              ? 3'b100 :
                                                                 3'b101 ;
 
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hall_a <= 1'b0;
            hall_b <= 1'b0;
            hall_c <= 1'b0;
        end else begin
            // {hall_c, hall_b, hall_a} = hall_pattern
            hall_a <= hall_pattern[0];
            hall_b <= hall_pattern[1];
            hall_c <= hall_pattern[2];
        end
    end
 
endmodule
