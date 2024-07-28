//############################################################################
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//   File Name   : BRIDGE_encrypted.v
//   Module Name : BRIDGE
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//############################################################################

module BRIDGE(
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

// Input Signals
input clk, rst_n;
input in_valid;
input direction;
input [12:0] addr_dram;
input [15:0] addr_sd;

// Output Signals
output reg out_valid;
output reg [7:0] out_data;

// DRAM Signals
// write address channel
output reg [31:0] AW_ADDR;
output reg AW_VALID;
input AW_READY;
// write data channel
output reg W_VALID;
output reg [63:0] W_DATA;
input W_READY;
// write response channel
input B_VALID;
input [1:0] B_RESP;
output reg B_READY;
// read address channel
output reg [31:0] AR_ADDR;
output reg AR_VALID;
input AR_READY;
// read data channel
input [63:0] R_DATA;
input R_VALID;
input [1:0] R_RESP;
output reg R_READY;

// SD Signals
input MISO;
output reg MOSI;

//==============================================//
//       parameter & integer declaration        //
//==============================================//

parameter IDLE = 5'd0,
          READ_CMD = 5'd1, // if direction 0 => DRAM_R_VALID, 1 => SD_CMD_READ
          DRAM_READ_VALID = 5'd2,
          DRAM_READ_READY = 5'd3,
          SD_CMD_READ = 5'd4, // read the 48 bit command for SD card
          SD_RESPONSE_WAIT = 5'd5, // waiting for 0x00 response
          SD_WAIT_DATA = 5'd6,
          SD_OPERATION = 5'd7, // take the action according to 0/1 (Read or write)
          SD_DATA_WRITE = 5'd8,
          SD_RESPONSE_WAIT_2 = 5'd9,
          OUT_VALID = 5'd10,
          SD_COMPLETE = 5'd11,
          DRAM_WRITE_VALID = 5'd12,
          DRAM_WRITE_READY = 5'd13,
          DRAM_WRITE_RESPONSE = 5'd14,
          SD_DATA_READ_WAIT = 5'd15,
          SD_DATA_READ = 5'd16;


//==============================================//
//           reg & wire declaration             //
//==============================================//

reg [8:0] cnt;
reg [12:0] addr_dram_temp;
reg [15:0] addr_sd_temp;
reg direction_temp;

// read data of DRAM
reg [63:0] data_temp;

reg [39:0] CRC7_in;
wire [47:0] sd_rw_cmd;
reg  [47:0] sd_rw_cmd_temp;

wire [87:0] sd_write_data;
reg  [87:0] sd_write_data_temp;

reg [4:0] c_state, n_state;


//==============================================//
//                    FSM                       //
//==============================================//
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        c_state <= IDLE;
    end
    else c_state <= n_state ;
end
//==============================================//
//                 NEXT STATE                   //
//==============================================//

always @(*) begin
    case (c_state)
        IDLE : n_state <= (in_valid) ? READ_CMD : IDLE; 
        // direction = 0 : DRAM -> SD : DRAM read & SD write
        // direction = 1 : SD -> DRAM : SD read & DRAM write
        READ_CMD : n_state <= (direction_temp) ?  SD_CMD_READ :DRAM_READ_VALID;
        //AR_READY indicates that the slave is ready to accept an address signal.
        // Perform handshake
        DRAM_READ_VALID : n_state <= (AR_READY) ? DRAM_READ_READY : DRAM_READ_VALID;
        DRAM_READ_READY : n_state <= (R_VALID) ? SD_CMD_READ : DRAM_READ_READY;
        SD_CMD_READ : n_state <= (cnt == 47) ? SD_RESPONSE_WAIT : SD_CMD_READ;
        SD_RESPONSE_WAIT : n_state <= (MISO) ? SD_RESPONSE_WAIT : SD_WAIT_DATA;
        SD_WAIT_DATA : n_state <= (cnt == 12) ? SD_OPERATION : SD_WAIT_DATA;
        SD_OPERATION : begin
            if (direction_temp) begin
                n_state <= SD_DATA_READ_WAIT;
            end else n_state <= SD_DATA_WRITE;
        end
        SD_DATA_WRITE : n_state <= (cnt == 87) ? SD_RESPONSE_WAIT_2 : SD_DATA_WRITE;
        SD_RESPONSE_WAIT_2 :n_state <= (cnt > 8 && MISO == 1) ? OUT_VALID : SD_RESPONSE_WAIT_2; // MISO =1 mean data has been written in SD card;
        OUT_VALID : n_state <= (cnt == 7) ? IDLE : OUT_VALID;
        //SD_COMPLETE :
        DRAM_WRITE_VALID : n_state <= (AW_READY) ? DRAM_WRITE_READY :  DRAM_WRITE_VALID;
        DRAM_WRITE_READY :n_state <= (W_READY) ? DRAM_WRITE_RESPONSE :  DRAM_WRITE_READY;
        DRAM_WRITE_RESPONSE : n_state <= (B_VALID) ? OUT_VALID :  DRAM_WRITE_RESPONSE;
        SD_DATA_READ_WAIT: n_state <= (MISO == 0) ? SD_DATA_READ: SD_DATA_READ_WAIT;
        SD_DATA_READ: n_state <= (cnt == 63) ? DRAM_WRITE_VALID: SD_DATA_READ;

        default:  n_state <= IDLE;
    endcase
end

//==============================================//
//                  design                      //
//==============================================//

// Counter 
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) cnt <= 0;
    else if ((c_state ==SD_DATA_WRITE && cnt == 87) || (c_state == SD_RESPONSE_WAIT_2 && (cnt > 8 && MISO == 1))
            || c_state == IDLE ) cnt <= 0;
    else if (c_state == SD_CMD_READ || c_state == SD_WAIT_DATA
            || c_state == SD_RESPONSE_WAIT_2 || c_state == OUT_VALID
            || c_state == SD_DATA_READ || c_state == SD_DATA_WRITE) cnt <= cnt + 1;
    else  cnt <= 0;
end

// Store the pattern into temp 
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        direction_temp <= 0;
        addr_dram_temp <= 0;
        addr_sd_temp <= 0;
    end
    else if (c_state == IDLE && n_state == READ_CMD) begin
        addr_dram_temp <= addr_dram;
        addr_sd_temp <= addr_sd;
        direction_temp <= direction; 
    end
end
//==============================================//
//                    DRAM                      //
//==============================================//
//Read addr channel of DRAM
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        AR_ADDR <= 0;
        AR_VALID <= 0;
    end
    else if (c_state == DRAM_READ_VALID && n_state != DRAM_READ_READY ) begin
        AR_ADDR <= addr_dram_temp;
        AR_VALID <= 1;
    end
    else begin
        AR_ADDR <= 0;
        AR_VALID <= 0;
    end
end
//Read data channel
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) R_READY <= 0;
    else if (n_state == DRAM_READ_READY || (c_state == DRAM_READ_READY && n_state != SD_CMD_READ ) ) begin
        R_READY <= 1;
    end else R_READY <= 0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) data_temp <= 0;
    else if (c_state == DRAM_READ_READY && R_VALID ) data_temp <= R_DATA;
    else if (c_state == SD_DATA_READ) data_temp <= (data_temp << 1) | MISO ; // each cycle push MISO to the LSB of transfer data.
    else if (c_state == IDLE) data_temp <= 0; 
    //else remain the same
end

// Write address channel 
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        AW_ADDR <= 0;
        AW_VALID <= 0;
    end
    else if (c_state == DRAM_WRITE_VALID && n_state != DRAM_WRITE_READY ) begin
        AW_ADDR <= addr_dram_temp;
        AW_VALID <= 1;
    end
    else begin
        AW_ADDR <= 0;
        AW_VALID <= 0;
    end
end
// Write data channel
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)begin
        W_DATA <= 0;
        W_VALID <= 0;
    end
    else if (n_state == DRAM_WRITE_READY || (c_state == DRAM_WRITE_READY && n_state != DRAM_WRITE_RESPONSE ) ) begin
        W_DATA <= data_temp;
        W_VALID <= 1;
    end else begin
        W_DATA <= 0;
        W_VALID <= 0;
    end
end

// Write reponse channel of DRAM
always @(posedge clk or negedge rst_n) begin
     if (!rst_n) B_READY <= 0;
     else if ((c_state == DRAM_WRITE_READY || c_state == DRAM_WRITE_RESPONSE) && n_state != OUT_VALID ) begin
        B_READY <= 1;
     end else B_READY <= 0;
end

//==============================================//
//                      SD                      //
//==============================================//

// Generate write cmd to SD card
always @(posedge clk) begin
    if (direction_temp) begin
        CRC7_in <=  {2'b01,6'd17,16'b0,addr_sd_temp};
    end else begin
        CRC7_in <=  {2'b01,6'd24,16'b0,addr_sd_temp};
    end
    // Command argument is 32bit, so expand the 16bit addr with 0.
end
// sd_rw_cmd to combime the signal and send to  sd_rw_cmd_temp
assign sd_rw_cmd = {CRC7_in,CRC7(CRC7_in),1'b1};

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) sd_rw_cmd_temp <= 0;
    else if (n_state == SD_CMD_READ) sd_rw_cmd_temp <= sd_rw_cmd;
end


// Generate the write data block
assign sd_write_data =  {8'hFE, data_temp, CRC16_CCITT(data_temp)};
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) sd_write_data_temp <= 0;
    else if (n_state == SD_DATA_WRITE) sd_write_data_temp <= sd_write_data;
end

// MOSI output
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) MOSI <= 1;
        else if (c_state == SD_CMD_READ) begin
        MOSI <= sd_rw_cmd_temp[47-cnt]; // Control the input bit via the counter
    end else if (c_state == SD_DATA_WRITE) begin
        MOSI <= sd_write_data_temp[87-cnt];
    end else MOSI <=1;
end

//==============================================//
//             CRC function                     //
//==============================================//
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

//==============================================//
//                   OUTPUT                     //
//==============================================//

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) out_valid <= 0;
    else if(c_state == OUT_VALID) out_valid <= 1;
    else out_valid <= 0;
end

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) out_data <= 0;
    else if(c_state == OUT_VALID && cnt == 0) out_data <= data_temp[63:56];
    else if(c_state == OUT_VALID && cnt == 1) out_data <= data_temp[55:48];
    else if(c_state == OUT_VALID && cnt == 2) out_data <= data_temp[47:40];
    else if(c_state == OUT_VALID && cnt == 3) out_data <= data_temp[39:32];
    else if(c_state == OUT_VALID && cnt == 4) out_data <= data_temp[31:24];
    else if(c_state == OUT_VALID && cnt == 5) out_data <= data_temp[23:16];
    else if(c_state == OUT_VALID && cnt == 6) out_data <= data_temp[15:8];
    else if(c_state == OUT_VALID && cnt == 7) out_data <= data_temp[7:0];
    else out_data <= 0;
end


endmodule

