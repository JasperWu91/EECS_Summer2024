`ifdef RTL
    `define CYCLE_TIME 40.0
`endif
`ifdef GATE
    `define CYCLE_TIME 40.0
`endif

`include "../00_TESTBED/pseudo_DRAM.v"
`include "../00_TESTBED/pseudo_SD.v"

module PATTERN(
    // Input Signals
    clk,
    rst_n,
    in_valid,
    direction,
    addr_dram,
    addr_sd,
    // Output Signals
    out_valid,
    out_data,
    // DRAM Signals
    AR_VALID, AR_ADDR, R_READY, AW_VALID, AW_ADDR, W_VALID, W_DATA, B_READY,
	AR_READY, R_VALID, R_RESP, R_DATA, AW_READY, W_READY, B_VALID, B_RESP,
    // SD Signals
    MISO,
    MOSI
);

/* Input for design */
output reg        clk, rst_n;
output reg        in_valid;
output reg        direction;
output reg [12:0] addr_dram;
output reg [15:0] addr_sd;

/* Output for pattern */
input        out_valid;
input  [7:0] out_data; 

// DRAM Signals
// write address channel
input [31:0] AW_ADDR;
input AW_VALID;
output AW_READY;
// write data channel
input W_VALID;
input [63:0] W_DATA;
output W_READY;
// write response channel
output B_VALID;
output [1:0] B_RESP;
input B_READY;
// read address channel
input [31:0] AR_ADDR;
input AR_VALID;
output AR_READY;
// read data channel
output [63:0] R_DATA;
output R_VALID;
output [1:0] R_RESP;
input R_READY;

// SD Signals
output MISO;
input MOSI;

real CYCLE = `CYCLE_TIME;
integer pat_read;
integer PAT_NUM;
integer total_latency, latency;
integer i_pat;
integer fscanf_int;

initial begin
    pat_read = $fopen("../00_TESTBED/Input.txt", "r");
    reset_signal_task;

    i_pat = 0;
    total_latency = 0;
    fscanf_int = $fscanf(pat_read, "%d", PAT_NUM);
    for (i_pat = 1; i_pat <= PAT_NUM; i_pat = i_pat + 1) begin
        input_task;
        wait_out_valid_task;
        check_ans_task;
        total_latency = total_latency + latency;
        $display("PASS PATTERN NO.%4d", i_pat);
    end
    $fclose(pat_read);

    $writememh("../00_TESTBED/DRAM_final.dat", u_DRAM.DRAM);
    $writememh("../00_TESTBED/SD_final.dat", u_SD.SD);
    YOU_PASS_task;
end

//////////////////////////////////////////////////////////////////////
// Write your own task here
//////////////////////////////////////////////////////////////////////

// clk signal genarate
always #(CYCLE/2.0) clk = ~clk;


// SPEC MAIN-1: All output signals should be reset after the reset signal is asserted.
task reset_signal_task;begin

rst_n = 'b1;
in_valid = 'b0;
direction = 'bx;
addr_dram = 'bx;
addr_sd = 'bx;

force clk = 0;
#CYCLE; rst_n = 0;
#CYCLE; rst_n = 1;

// Check if all output signals are reset after the reset signal is asserted.

if (out_valid !== 0 || out_data !== 0 || AW_ADDR !== 0 || AW_VALID !== 0 || W_VALID !== 0 || W_DATA !== 0
    || B_READY !== 0 || AR_ADDR !== 0 || AR_VALID !== 0 || R_READY !== 0 || MOSI !== 1 ) begin
        $display("*************************************************************");
        $display("*                     SPEC MAIN-1 FAIL                      *");    
        $display("* All output signals should be reset after the reset signal *");
        $display("*************************************************************");
        repeat(2) #CYCLE;
        $finish;    
end
    #CYCLE; release clk;
end endtask


//  SPEC MAIN-2: The out_data should be reset when your out_valid is low.
always @(negedge clk) begin
    if (out_valid == 0 && out_data !== 0) begin
        $display("*************************************************************");
        $display("*                      SPEC MAIN-2 FAIL                     *");    
        $display("*  The out_data should be reset when your out_valid is low  *");
        $display("*************************************************************");
        repeat(2) #CYCLE;
        $finish;          
    end  
end

reg        direction_buffer;
reg [12:0] addr_dram_buffer;
reg [15:0] addr_sd_buffer;
integer times;

task input_task; begin
    // 0        3750       20877 
    fscanf_int = $fscanf(pat_read, "%d", direction_buffer);    // direction_buffer 讀取到 0
    fscanf_int = $fscanf(pat_read, "%d", addr_dram_buffer);    // addr_dram_buffer 讀取到 3750
    fscanf_int = $fscanf(pat_read, "%d", addr_sd_buffer);      // addr_sd_buffer 讀取到 20877

    // the next input pattern will come in 2~4 negative edge of the clk after out_valid is pulled dowm
    times = $urandom_range(4,1);
    repeat(times) @(negedge clk);
    in_valid = 1;
    direction = direction_buffer;
    addr_dram = addr_dram_buffer;
    addr_sd = addr_sd_buffer;

    @(negedge clk);
    in_valid = 1'b0; 
    direction = 'bx; // set the signal to unknown
    addr_dram = 'bx;
    addr_sd = 'bx;
end endtask

// The execution latency is limited in 10000 cycles
task wait_out_valid_task;begin
    latency = 0;
    while (out_valid !== 1'b1) begin
        latency = latency + 1'b1; // calculate execution latency
        if (latency == 10000) begin
            $display("*************************************************************");
            $display("*                      SPEC MAIN-3 FAIL                     *");    
            $display("*   The execution latency is limited in 10000 cycles        *");
            $display("*************************************************************");
            repeat(2)@(negedge clk);
            $finish;  
        end
        @(negedge clk);
    end
    total_latency = total_latency + latency;
end endtask


task check_ans_task; begin
    latency = 0;
    while (out_valid === 1) begin
        //The out_valid and out_data must be asserted in 8 cycles.
        latency = latency + 1;
        if (latency > 8) fail_4_task;

        // SPEC MAIN-6: The data in the DRAM and SD card should be correct when out_valid is high.
        // no matter the direction the data stored in addr_sd_temp in Sd should be the same as addr_dram_temp in DRAM.
        if(u_SD.SD[addr_sd_buffer] !== u_DRAM.DRAM[addr_dram_buffer])begin
            $display("************************************************************");     
            $display("*                     SPEC MAIN-6 FAIL                     *");
            $display("* The data in the DRAM and SD card should be correct when *");
            $display("*                 out_valid is high.                       *");
            $display("************************************************************");
            repeat(2)@(negedge clk);
            $finish;
        end

        // The out_data should be correct when out_valid is high
        case (latency)
            1: begin
                if(direction === 0) begin
                    if(out_data !== u_SD.SD[addr_sd_buffer][63:56])fail_5_task;
                end else begin
                    if(out_data !== u_DRAM.DRAM[addr_dram_buffer][63:56])fail_5_task;
                end
            end

            2: begin
                if(direction === 0) begin
                    if(out_data !== u_SD.SD[addr_sd_buffer][55:48])fail_5_task;
                end else begin
                    if(out_data !== u_DRAM.DRAM[addr_dram_buffer][55:48])fail_5_task;
                end
            end

                3: begin
                if(direction === 0) begin
                    if(out_data !== u_SD.SD[addr_sd_buffer][47:40]) fail_5_task;
                end else begin
                    if(out_data !== u_DRAM.DRAM[addr_dram_buffer][47:40]) fail_5_task;
                end
            end

            4: begin
                if(direction === 0) begin
                    if(out_data !== u_SD.SD[addr_sd_buffer][39:32]) fail_5_task;
                end else begin
                    if(out_data !== u_DRAM.DRAM[addr_dram_buffer][39:32]) fail_5_task;
                end
            end

            5: begin
                if(direction === 0) begin
                    if(out_data !== u_SD.SD[addr_sd_buffer][31:24]) fail_5_task;
                end else begin
                    if(out_data !== u_DRAM.DRAM[addr_dram_buffer][31:24]) fail_5_task;
                end
            end

            6: begin
                if(direction === 0) begin
                    if(out_data !== u_SD.SD[addr_sd_buffer][23:16]) fail_5_task;
                end else begin
                    if(out_data !== u_DRAM.DRAM[addr_dram_buffer][23:16]) fail_5_task;
                end
            end

            7: begin
                if(direction === 0) begin
                    if(out_data !== u_SD.SD[addr_sd_buffer][15:8]) fail_5_task;
                end else begin
                    if(out_data !== u_DRAM.DRAM[addr_dram_buffer][15:8]) fail_5_task;
                end
            end

            8: begin
                if(direction === 0) begin
                    if(out_data !== u_SD.SD[addr_sd_buffer][7:0]) fail_5_task;
                end else begin
                    if(out_data !== u_DRAM.DRAM[addr_dram_buffer][7:0]) fail_5_task;
                end
            end
            endcase
    @(negedge clk);
    end
    if (latency !== 8) fail_4_task; // latency < 8 fail.

end endtask
//////////////////////////////////////////////////////////////////////

task fail_4_task; begin
    $display("*************************************************************");
    $display("*                      SPEC MAIN-4 FAIL                     *");    
    $display("*  The out_valid and out_data must be asserted in 8 cycles  *");
    $display("*************************************************************");
    repeat(2)@(negedge clk);
    $finish; 
end endtask

task fail_5_task; begin
    $display("*************************************************************");
    $display("*                      SPEC MAIN-5 FAIL                     *");    
    $display("*  The out_data should be correct when out_valid is high    *");
    $display("*************************************************************");
    repeat(2)@(negedge clk);
    $finish; 
end endtask


task YOU_PASS_task; begin
    $display("*************************************************************************");
    $display("*                         Congratulations!                              *");
    $display("*                Your execution cycles = %5d cycles          *", total_latency);
    $display("*                Your clock period = %.1f ns          *", CYCLE);
    $display("*                Total Latency = %.1f ns          *", total_latency*CYCLE);
    $display("*************************************************************************");
    $finish;
end endtask

task YOU_FAIL_task; begin
    $display("*                              FAIL!                                    *");
    $display("*                    Error message from PATTERN.v                       *");
end endtask

pseudo_DRAM u_DRAM (
    .clk(clk),
    .rst_n(rst_n),
    // write address channel
    .AW_ADDR(AW_ADDR),
    .AW_VALID(AW_VALID),
    .AW_READY(AW_READY),
    // write data channel
    .W_VALID(W_VALID),
    .W_DATA(W_DATA),
    .W_READY(W_READY),
    // write response channel
    .B_VALID(B_VALID),
    .B_RESP(B_RESP),
    .B_READY(B_READY),
    // read address channel
    .AR_ADDR(AR_ADDR),
    .AR_VALID(AR_VALID),
    .AR_READY(AR_READY),
    // read data channel
    .R_DATA(R_DATA),
    .R_VALID(R_VALID),
    .R_RESP(R_RESP),
    .R_READY(R_READY)
);

pseudo_SD u_SD (
    .clk(clk),
    .MOSI(MOSI),
    .MISO(MISO)
);

endmodule
