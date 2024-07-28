//############################################################################
//
//   File Name   : pseudo_SD.v
//   Module Name : pseudo_SD
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//############################################################################

`ifdef RTL
    `define CYCLE_TIME 40.0
`endif
`ifdef GATE
    `define CYCLE_TIME 40.0
`endif

module pseudo_SD (
    clk,
    MOSI,
    MISO
);

input clk;
input MOSI;
output reg MISO;

// register for command
reg [63:0] SD [0:65535]; // legal range : 0 ~ 65535
reg [5:0] CMD;
reg [31:0] addr_sd;
reg [6:0]  CRC7_receive;
reg [39:0] CRC7_in;


// data blcok
reg [7:0] start_token ;
reg [63:0] SD_RW_DATA ;
reg [15:0] CRC16;
reg [15:0] CRC16_receive;
reg [7:0] Data_response;

// Parameter
parameter SD_p_r = "../00_TESTBED/SD_init.dat";
real CYCLE = `CYCLE_TIME;
integer i;
integer times;
// Initial all the signal and reg
initial begin
    $readmemh(SD_p_r, SD); // Search data from SD
    MISO = 1'b1;

    CMD = 0;
    addr_sd = 0;
    CRC7_receive = 0;
    //CRC7_in = 0;

    start_token = 8'hFE;
    SD_RW_DATA = 0;
    CRC16 = 0;
    Data_response  = 8'b00000101;
end

// Read the input cmd
always @(negedge clk) begin
    if(MOSI === 1'b0) begin // start bit should be 0
        receive_cmd_task;
    end

    //else do nothing
end


//////////////////////////////////////////////////////////////////////
// Write your own task here
//////////////////////////////////////////////////////////////////////

// SPEC SD-1: Command format should be correct.
// SPEC SD-2: The address should be within the legal range (0~65535).
// SPEC SD-3: CRC-7 check should be correct.
// SPEC SD-4: CRC-16-CCITT check should be correct.
// SPEC SD-5: Time between each transmission should be correct. (Only integer time units is allowed).

task receive_cmd_task; begin
    #CYCLE; // wait for 1 cycle

    if (MOSI === 1'b0) begin
        $display("*                        SPEC SD-1 FAIL                                 *");
        $display("* All output signals should be reset after the reset signal is asserted  *");
        $display("* line 88  *");
    end

    #CYCLE;

    // 6-bit command
    for (i = 0; i < 6 ; i = i + 1) begin
        CMD = (CMD << 1) + MOSI; 
        #CYCLE;
    end

    // 32 bit Command Argument
    for (i = 0; i < 32 ; i = i + 1) begin
        addr_sd = (addr_sd << 1) + MOSI; 
        #CYCLE;
    end

    // Check the range of command
    if (addr_sd > 65535) begin
        
        $display("*                       SPEC SD-2 FAIL                                 *");
        $display("*      The address should be within the legal range (0~65535).         *");
        repeat(2) #CYCLE;
        $finish;
    end

    for (i = 0; i < 7 ; i = i + 1 ) begin
        CRC7_receive = (CRC7_receive << 1) + MOSI;
        #CYCLE;
    end

    CRC7_in = {2'b01,CMD, addr_sd};
    
    if (CRC7_receive !== CRC7(CRC7_in)) begin
        $display("*                         SPEC SD-3 FAIL                               *");
        $display("*                  CRC-7 check should be correct.                      *");
        repeat(2) #CYCLE;
        $finish;
    end

    if(MOSI === 1'b0)begin
          $display("*                        SPEC SD-1 FAIL                                 *");
        $display("* All output signals should be reset after the reset signal is asserted  *");
        $display("* line 131 end bit error  *");
    end
    #CYCLE;

    // wait 0~8 units, unit = 8 cycle
    times = $urandom_range(8);
    repeat(8*times) #CYCLE;

    // response from SD card through MISO
    for (i = 0 ; i < 8 ; i = i + 1 ) begin
        MISO = 0;
        #CYCLE;
    end

    MISO = 1;

    if(CMD == 17)begin
        read_task;
    end else if ( CMD == 24 ) begin
        write_task;
    end else begin
        $display("*                        SPEC SD-1 FAIL                                 *");
        $display("* All output signals should be reset after the reset signal is asserted  *");
        $display("* line 154  *");
    end

end
endtask


//////////////////////////////////////////////////////////////////////

task read_task; begin
    times = $urandom_range(32,1);
    repeat(8*times) #CYCLE;

    for (i = 0 ; i < 8  ; i = i + 1 ) begin
        MISO = start_token[7-i];
        #CYCLE;
    end

    SD_RW_DATA = SD[addr_sd];

    for ( i  = 0 ; i < 64 ; i = i + 1) begin
        MISO = SD_RW_DATA[63-i];
        #CYCLE;    
    end   


    CRC16 = CRC16_CCITT(SD_RW_DATA);
    for ( i  = 0 ; i < 16 ; i = i + 1) begin
        MISO = CRC16[15-i];
        #CYCLE;
    end

    MISO = 1;

end endtask


task write_task; begin
    wait_to_receive_data;

    for ( i  = 0 ; i < 64 ; i = i + 1) begin
        SD_RW_DATA = (SD_RW_DATA << 1) + MOSI;
        #CYCLE;    
    end   

    for ( i  = 0 ; i < 16 ; i = i + 1) begin
        CRC16 = (CRC16 << 1) + MOSI;
        #CYCLE;                
    end

    if (CRC16 !== CRC16_CCITT(SD_RW_DATA) ) begin
        $display("*                         SPEC SD-4 FAIL                                *");
        $display("*             CRC-16-CCITT check should be correct.                     *");
        repeat(2) #CYCLE;
        $finish;
    end


    for (i  = 0 ; i < 8 ; i = i + 1) begin
        MISO = Data_response[7-i];
        #CYCLE;
    end

    times = $urandom_range(32);
    MISO = 0; // keep low until writing process complete
    repeat( 8 * times) #CYCLE;
    SD[addr_sd] = SD_RW_DATA;
    
    MISO = 1;// return to idle
end
endtask

integer latency;

// SPEC SD-5: Time between each transmission should be correct. (Only integer time units is allowed)
task wait_to_receive_data; begin

    latency = 0; //initialize the latency
    while (MOSI !== 0) begin
        @(negedge clk);
        latency = latency + 1;
    end
    // wait 0~8 cycle and 256
    if (latency > 256 + 8) begin  
        $display("*                         SPEC SD-5 FAIL                                 *");
        $display("*           Time between each transmission should be correct.            *");
        repeat(2)#CYCLE;
        $finish;
    end else if (latency < 8+8) begin
        $display("*                         SPEC SD-5 FAIL                                *");
        $display("*           Time between each transmission should be correct.            *");
        repeat(2)#CYCLE;
        $finish;
    end else if ((latency-8) % 8 !== 0 ) begin
        $display("*                         SPEC SD-5 FAIL                                 *");
        $display("*           Time between each transmission should be correct.            *");
        repeat(2)#CYCLE;
        $finish;
    end
    #CYCLE;
end
endtask


task YOU_FAIL_task; begin
    $display("*                        SPEC SD-1 FAIL                                 *");
    $display("* All output signals should be reset after the reset signal is asserted  *");
end endtask

function automatic [6:0] CRC7;  // Return 7-bit result
    input [39:0] data;  // 40-bit data input
    reg [6:0] crc;
    integer i;
    reg data_in, data_out;
    parameter polynomial = 7'h9;  // x^7 + x^3 + 1

    begin
        crc = 7'd0;
        for (i = 0; i < 40; i = i + 1) begin
            data_in = data[39-i];
            data_out = crc[6];
            crc = crc << 1;  // Shift the CRC
            if (data_in ^ data_out) begin
                crc = crc ^ polynomial;
            end
        end
        CRC7 = crc;
    end
endfunction

function automatic [15:0] CRC16_CCITT;
    // Try to implement CRC-16-CCITT function by yourself.
    input [63:0] data;  // 64-bit data input
    reg [15:0] crc;
    integer i;
    reg data_in, data_out;
    parameter polynomial = 16'h1021;  // X^16+X^12+X^5+1

    begin
        crc = 16'd0;
        for (i = 0; i < 64; i = i + 1) begin
            data_in = data[63-i];
            data_out = crc[15];
            crc = crc << 1;  // Shift the CRC
            if (data_in ^ data_out) begin
                crc = crc ^ polynomial;
            end
        end
        CRC16_CCITT = crc;
    end
endfunction

endmodule
