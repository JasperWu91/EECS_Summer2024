//############################################################################
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//   (C) Copyright Laboratory System Integration and Silicon Implementation
//   All Right Reserved
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//   EECS 2024 SUMMER TRAINING
//   Lab03 Exercise		: Siamese Neural Network 
//   Author     		: 
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//   File Name   : SNN.v
//   Module Name : SNN
//   Release version : V1.0 (Release Date: 2023-09)
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//############################################################################
//`include "/usr/synthesis/dw/sim_ver/DW_fp_mac.v"
//`include "/usr/synthesis/dw/sim_ver/DW_fp_cmp.v"
//`include "/usr/synthesis/dw/sim_ver/DW_fp_mult.v"
//`include "/usr/synthesis/dw/sim_ver/DW_fp_add.v"
//`include "/usr/synthesis/dw/sim_ver/DW_fp_sub.v"
//`include "/usr/synthesis/dw/sim_ver/DW_fp_div.v"
//`include "/usr/synthesis/dw/sim_ver/DW_fp_exp.v"
//`include "/usr/synthesis/dw/sim_ver/DW_fp_recip.v"
//`include "/usr/synthesis/dw/sim_ver/DW_fp_mac.v"

module SNN(
    //Input Port
    clk,
    rst_n,
    in_valid,
    Img,
    Kernel,
	Weight,
    Opt,
    //Output Port
    out_valid,
    out
    );

//---------------------------------------------------------------------
//   PARAMETER
//---------------------------------------------------------------------

// IEEE floating point parameter
parameter inst_sig_width = 23;
parameter inst_exp_width = 8;
parameter inst_ieee_compliance = 0;
parameter inst_arch_type = 0;
parameter inst_arch = 0;
parameter inst_faithful_round = 0;

input rst_n, clk, in_valid;
input [inst_sig_width+inst_exp_width:0] Img, Kernel, Weight;
input [1:0] Opt;

output reg	out_valid;
output reg [inst_sig_width+inst_exp_width:0] out;

parameter bit_width = 32; 
integer i,j,k;
//==============================================//
//                reg and wire                  //
//==============================================//
// kernel counter
reg  [7:0] kernel_cnt;
reg  [31:0] kernel_conv    [0:2][0:2];
reg  [31:0] kernel_conv_1  [0:2][0:2];
reg  [31:0] kernel_conv_2  [0:2][0:2];
reg  [31:0] kernel_conv_3  [0:2][0:2];

reg [31:0] k_map  [0:2][0:2];
reg signed [bit_width-1:0] image  [0:5][0:5];
reg signed [bit_width-1:0] weight [0:1][0:1];
reg signed [bit_width-1:0] weight1 [0:1][0:1];
reg signed [bit_width-1 : 0] img_conv [0:5][0:5];
reg signed [bit_width-1 : 0] img_conv_1 [0:5][0:5];
reg signed [bit_width-1 : 0] img_conv_2 [0:5][0:5];
reg signed [bit_width-1 : 0] img_conv_3 [0:5][0:5];
reg [1:0 ]opt;
reg [9:0] cnt; //the universal counter

reg signed [bit_width-1 : 0] f_map  [0:3][0:3]; // feature map after conv
reg signed [bit_width-1 : 0] feature_map [0:3];
reg signed [bit_width-1 : 0] feature_map_normalize [0:3];
reg signed [bit_width-1 : 0] encoding_map [0:3];
reg signed [bit_width-1 : 0] encoding_map_img1 [0:3];
reg signed [bit_width-1 : 0] encoding_map_img2 [0:3];


reg [bit_width-1 : 0] MACU_A [0:15] ;
reg [bit_width-1 : 0] MACU_B [0:15] ;
reg [bit_width-1 : 0] MACU_C [0:15] ;
wire [bit_width-1 : 0] MACU_out [0:15];

reg [2:0] mul_state;
reg [3:0] max_cs, max_ns;

reg [bit_width-1:0] cmp_a, cmp_b, cmp_c, cmp_d;
reg [bit_width-1:0] max_0_1, max_1_1, max_0_2, max_1_2;
reg [bit_width-1:0] f_map_2 [0:1][0:1];
wire [31:0] max0, max1;
reg [bit_width-1:0] max_result_0, max_result_1;

reg [5:0] idx ;
reg [1:0] c_layer ;
reg [bit_width-1:0] conv_out [0:3];

reg [3:0] conv_row;
reg [5:0] conv_cnt;
reg [1:0] conv_layer;
reg [3:0] conv_1_cnt;
reg [3:0] conv_2_cnt;
reg [3:0] conv_3_cnt;

reg conv_finish ;
reg conv_valid;
reg conv1_ready;
reg conv2_ready;
reg conv3_ready;
reg finish;
reg compare_complete;
reg  mul_complete;
reg L1_complete;
reg act_complete;
reg  conv_1_complete,conv_2_complete,conv_3_complete,conv_complete ;
reg max_pool_complete,nor_complete,compare_ready;
reg normalized ;

reg  [31:0] x_scale ;
reg  [31:0] SUB_A,SUB_B ;
wire [31:0] SUB_OUT;
reg  [31:0] SUB_A0,SUB_B0 ;
wire [31:0] SUB_OUT0;
reg  [31:0] SUB_A2,SUB_B2 ;
wire [31:0] SUB_OUT2;
reg [3:0] nor_cs;


reg [5:0] img_cnt;  
reg [3:0] img_input_cnt;
reg[3:0] conv_cs,conv_ns ;// state for conv
reg [1:0] img_store_cnt ; // count the number of encoding img

reg [31:0] temp0,output_result;
reg [2:0] sum_cs;

//parameter = 1, optimized for speed, could be changed for later use...
reg [31 : 0] EXP0_in , MACA, MACB,MACC;
reg [31 : 0] EXP_1;
wire [31:0] one = 32'h3F800000; // 1 
wire [31:0] two = 32'h40000000; // 2
wire [31:0] neg_one = 32'hBF800000; //-1
wire [31 :0] EXP0_out1 , MAC_OUT0;

reg [31 : 0] act_in, act_result;
reg [2:0] act_cnt;
reg [3:0] act_cs;

reg[2:0] compare_cs;
reg [31:0] cmp_1, cmp_2, cmp_3, cmp_4 , max01, max02, min01, min02 , temp_max , temp_min;
wire [31:0] max_1, max_2,min_1,min_2;
reg min_miax_complete;

reg [1:0] x, nx, y, ny;

reg [bit_width-1:0] ADD_A;
reg [bit_width-1:0] ADD_B;
wire [bit_width-1:0] ADD_OUT;
reg [bit_width-1:0] ADD_A1;
reg [bit_width-1:0] ADD_B1;
wire [bit_width-1:0] ADD_OUT1;

reg [31:0] MULA,MULB,MULA1,MULB1;
wire [31:0] MUL_OUT,MUL_OUT1;

reg [31 : 0] REC0_in;
wire [31 :0] REC0_out;
reg  [31:0] DIV_A,DIV_B ;
wire [31:0] DIV_OUT;

//==============================================//
//                 IP module                    //
//==============================================//

DW_fp_cmp #(inst_sig_width, inst_exp_width, inst_ieee_compliance)
	CMP_1 ( .a(cmp_a), .b(cmp_b), .zctr(1'b1), .aeqb(), 
	.altb(), .agtb(), .unordered(), 
	.z0(max0), .z1(), .status0(), 
	.status1() );

DW_fp_cmp #(inst_sig_width, inst_exp_width, inst_ieee_compliance)
	CMP_2 ( .a(cmp_c), .b(cmp_d), .zctr(1'b1), .aeqb(), 
	.altb(), .agtb(), .unordered(), 
	.z0(max1), .z1(), .status0(), 
	.status1() );

DW_fp_add #(inst_sig_width, inst_exp_width, inst_ieee_compliance)
ADD1 ( .a(ADD_A), .b(ADD_B), .rnd(3'd0), .z(ADD_OUT), .status() );	

DW_fp_add #(inst_sig_width, inst_exp_width, inst_ieee_compliance)
ADD2 ( .a(ADD_A1), .b(ADD_B1), .rnd(3'd0), .z(ADD_OUT1), .status() );	

DW_fp_mult #(inst_sig_width, inst_exp_width, inst_ieee_compliance,0) MULT1 ( 
.a(MULA), .b(MULB), .rnd(3'd0), .z(MUL_OUT), .status() );

DW_fp_mult #(inst_sig_width, inst_exp_width, inst_ieee_compliance,0) MULT2 ( 
.a(MULA1), .b(MULB1), .rnd(3'd0), .z(MUL_OUT1), .status() );


DW_fp_mac #(inst_sig_width, inst_exp_width, inst_ieee_compliance) MAC_0 (
.a(MACU_A[0]), .b(MACU_B[0]), .c(MACU_C[0]), .rnd(3'd0),  
.z(MACU_out[0]), .status() );

DW_fp_mac #(inst_sig_width, inst_exp_width, inst_ieee_compliance) MAC_1 (
.a(MACU_A[1]), .b(MACU_B[1]), .c(MACU_C[1]), .rnd(3'd0),  
.z(MACU_out[1]), .status() );

DW_fp_mac #(inst_sig_width, inst_exp_width, inst_ieee_compliance) MAC_2 (
.a(MACU_A[2]), .b(MACU_B[2]), .c(MACU_C[2]), .rnd(3'd0),  
.z(MACU_out[2]), .status() );

DW_fp_mac #(inst_sig_width, inst_exp_width, inst_ieee_compliance) MAC_3 (
.a(MACU_A[3]), .b(MACU_B[3]), .c(MACU_C[3]), .rnd(3'd0),  
.z(MACU_out[3]), .status() );

DW_fp_mac #(inst_sig_width, inst_exp_width, inst_ieee_compliance) MAC_4 (
.a(MACU_A[4]), .b(MACU_B[4]), .c(MACU_C[4]), .rnd(3'd0),  
.z(MACU_out[4]), .status() );

DW_fp_mac #(inst_sig_width, inst_exp_width, inst_ieee_compliance) MAC_5 (
.a(MACU_A[5]), .b(MACU_B[5]), .c(MACU_C[5]), .rnd(3'd0),  
.z(MACU_out[5]), .status() );

DW_fp_mac #(inst_sig_width, inst_exp_width, inst_ieee_compliance) MAC_6 (
.a(MACU_A[6]), .b(MACU_B[6]), .c(MACU_C[6]), .rnd(3'd0),  
.z(MACU_out[6]), .status() );

DW_fp_mac #(inst_sig_width, inst_exp_width, inst_ieee_compliance) MAC_7 (
.a(MACU_A[7]), .b(MACU_B[7]), .c(MACU_C[7]), .rnd(3'd0),  
.z(MACU_out[7]), .status() );

DW_fp_mac #(inst_sig_width, inst_exp_width, inst_ieee_compliance) MAC_8 (
.a(MACU_A[8]), .b(MACU_B[8]), .c(MACU_C[8]), .rnd(3'd0),
.z(MACU_out[8]), .status() );

DW_fp_mac #(inst_sig_width, inst_exp_width, inst_ieee_compliance) MAC_9 (
.a(MACU_A[9]), .b(MACU_B[9]), .c(MACU_C[9]), .rnd(3'd0),
.z(MACU_out[9]), .status() );

DW_fp_mac #(inst_sig_width, inst_exp_width, inst_ieee_compliance) MAC_10 (
.a(MACU_A[10]), .b(MACU_B[10]), .c(MACU_C[10]), .rnd(3'd0),
.z(MACU_out[10]), .status() );

DW_fp_mac #(inst_sig_width, inst_exp_width, inst_ieee_compliance) MAC_11 (
.a(MACU_A[11]), .b(MACU_B[11]), .c(MACU_C[11]), .rnd(3'd0),
.z(MACU_out[11]), .status() );

DW_fp_mac #(inst_sig_width, inst_exp_width, inst_ieee_compliance) MAC_12 (
.a(MACU_A[12]), .b(MACU_B[12]), .c(MACU_C[12]), .rnd(3'd0),
.z(MACU_out[12]), .status() );

DW_fp_mac #(inst_sig_width, inst_exp_width, inst_ieee_compliance) MAC_13 (
.a(MACU_A[13]), .b(MACU_B[13]), .c(MACU_C[13]), .rnd(3'd0),
.z(MACU_out[13]), .status() );

DW_fp_mac #(inst_sig_width, inst_exp_width, inst_ieee_compliance) MAC_14 (
.a(MACU_A[14]), .b(MACU_B[14]), .c(MACU_C[14]), .rnd(3'd0),
.z(MACU_out[14]), .status() );

DW_fp_mac #(inst_sig_width, inst_exp_width, inst_ieee_compliance) MAC_15 (
.a(MACU_A[15]), .b(MACU_B[15]), .c(MACU_C[15]), .rnd(3'd0),
.z(MACU_out[15]), .status() );

DW_fp_cmp #(inst_sig_width, inst_exp_width, inst_ieee_compliance)
	CMP_01 ( .a(cmp_1), .b(cmp_2), .zctr(1'b1), .aeqb(), 
	.altb(), .agtb(), .unordered(), 
	.z0(max_1), .z1(min_1), .status0(), 
	.status1() );

DW_fp_cmp #(inst_sig_width, inst_exp_width, inst_ieee_compliance)
	CMP_02 ( .a(cmp_3), .b(cmp_4), .zctr(1'b1), .aeqb(), 
	.altb(), .agtb(), .unordered(), 
	.z0(max_2), .z1(min_2), .status0(), 
	.status1() );

DW_fp_sub #(inst_sig_width, inst_exp_width, inst_ieee_compliance)
	SUB_1 ( .a(SUB_A), .b(SUB_B), .rnd(3'd0), .z(SUB_OUT), .status() );
DW_fp_sub #(inst_sig_width, inst_exp_width, inst_ieee_compliance)
	SUB_0 ( .a(SUB_A0), .b(SUB_B0), .rnd(3'd0), .z(SUB_OUT0), .status() );
DW_fp_sub #(inst_sig_width, inst_exp_width, inst_ieee_compliance)
	SUB_2 ( .a(SUB_A2), .b(SUB_B2), .rnd(3'd0), .z(SUB_OUT2), .status() );

DW_fp_div #(inst_sig_width, inst_exp_width, inst_ieee_compliance) DIV_1 
( .a( DIV_A), .b( DIV_B), .rnd(3'd0), .z(DIV_OUT), .status() 
);

//EXP0 computes z = e^a, where a is fp
DW_fp_exp #(inst_sig_width, inst_exp_width, inst_ieee_compliance) EXP0 (
.a(EXP0_in), .z(EXP0_out1), .status() );

DW_fp_mac #(inst_sig_width, inst_exp_width, inst_ieee_compliance) MAC0 (
.a( MACA), .b( MACB), .c(MACC), .rnd(3'd0),  
.z(MAC_OUT0), .status() );


DW_fp_recip #(inst_sig_width, inst_exp_width, inst_ieee_compliance) REC0 (
.a(REC0_in), .rnd(3'd0), .z(REC0_out), .status() );


//==============================================//
//                     FSM                      //
//==============================================//
reg [1:0] c_state, n_state; // For current state and next state
parameter IDLE = 2'd0,
		  READ = 2'd1,
		  CALC = 2'd2,
		  OUT  = 2'd3;

//  Current State Block 
always@(posedge clk or negedge rst_n) begin
    if(!rst_n)
        c_state <= IDLE; /* initial state */
    else 
        c_state <= n_state;
end

//  Next State Block       
always@(*) begin
    case(c_state)
		IDLE :  n_state = (in_valid)? READ : IDLE;
		READ :  n_state = (conv_valid)?  CALC : READ;
		CALC :  n_state =  (L1_complete) ? IDLE : CALC;
		default : n_state = IDLE;
    endcase
end

// initialize the signal
always @(posedge clk) begin
    if(c_state == IDLE && n_state == READ)begin
		opt <= Opt; //save the Opt	
    end
end
//==============================================//
//                 FSM Conv                     //
//==============================================//
//   Current State Block 
always@(posedge clk or negedge rst_n) begin
    if(!rst_n)
        conv_cs <= 4'd0; /* initial state */
    else if (c_state == IDLE) conv_cs <= 4'd0; 
    else   conv_cs <= conv_ns;
end
//      Next State Block       
always@(*) begin
    case(conv_cs)
		4'd0 :  conv_ns = (conv_valid && conv1_ready)? 4'b1 : 4'b0; //IDLE 
		4'd1 :  conv_ns = (conv_1_complete && conv2_ready )?  4'd2 : 4'd1; // conv 1st layer
		4'd2 :  conv_ns = (conv_2_complete && conv3_ready)?   4'd3 : 4'd2; // conv 2nd layer
		4'd3 :  conv_ns = (conv_3_complete) ? 4'd4 : 4'd3;// conv 3rd layer
		4'd4 :  conv_ns = 4'd5;// output to feature map
		4'd5 :  conv_ns = (compare_complete) ? 4'd6 : 4'd5; // max_pooling 
		4'd6 :  conv_ns = (mul_complete) ? 4'd7 : 4'd6; // Fully-connected 
		4'd7 :  conv_ns = (nor_complete) ? 4'd8 : 4'd7; // Normalization
		4'd8 :  conv_ns = (act_complete) ? 4'd9 : 4'd8; // Activation for encoding
		4'd9 :  conv_ns = (img_store_cnt == 2'd1 ) ? 4'd10: 4'd1; // wait for second img 
		4'd10 : conv_ns = (L1_complete) ? 4'd11 : 4'd10 ; //  L1 distance
		4'd11 : begin
			conv_ns = 4'd0; //output
		end
		default : conv_ns = 4'b0;
    endcase
end

// img counter
always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		img_cnt <= 0;
	end
	else if(c_state == IDLE) img_cnt <= 0;
	else  img_cnt <= (img_cnt == 6'd47)? 0 : img_cnt + 6'd1;
end

// img data input and padding  
//  15 31 47 63 79 95 
always @(posedge clk) begin
    if (in_valid || c_state == CALC) begin
		if (img_input_cnt < 3'd6) begin
			case (img_cnt)
				7'd15: begin
					for (i = 0; i < 6; i = i + 1) begin
						for (j = 0; j < 6; j = j + 1) begin
							img_conv_1[i][j] <= image[i][j];
						end
					end
				end
				7'd31: begin
					for (i = 0; i < 6; i = i + 1) begin
						for (j = 0; j < 6; j = j + 1) begin
							img_conv_2[i][j] <= image[i][j];
						end
					end
				end
				7'd47: begin
					for (i = 0; i < 6; i = i + 1) begin
						for (j = 0; j < 6; j = j + 1) begin
							img_conv_3[i][j] <= image[i][j];
						end
					end
				end
			endcase
		end
    end
end

always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		conv1_ready <= 1'b0;
		conv2_ready <= 1'b0;
		conv3_ready <= 1'b0;
	end
	else if(c_state == IDLE)begin
		conv1_ready <= 1'b0;
		conv2_ready <= 1'b0;
		conv3_ready <= 1'b0;
    end
	else if (img_input_cnt < 3'd6) begin
		case (img_cnt)
			7'd15: begin
				conv1_ready <= 1'b1;
				conv2_ready <= 1'b0;
				conv3_ready <= 1'b0;
			end
			7'd31: begin
				conv1_ready <= 1'b0;
				conv2_ready <= 1'b1;
			end
			7'd47: begin
				conv3_ready <= 1'b1;
			end
			default: begin
				conv1_ready <= conv1_ready;
				conv2_ready <= conv2_ready;
				conv3_ready <= conv3_ready;
			end
		endcase
	end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        conv_valid <= 1'b0;
    end else if (c_state == IDLE) begin
        conv_valid <= 1'b0;
    end else if (img_input_cnt < 3'd6 && img_cnt == 7'd15) begin
        conv_valid <= 1'b1;
    end
end

// image: store the given 16 image values by shift register
always @(posedge clk) begin
	if (in_valid ==1'b1 ) begin
		image[4][4] <= Img;
		image[4][3] <= image[4][4];
		image[4][2] <= image[4][3];
		image[4][1] <= image[4][2];
	
		image[3][4] <= image[4][1];
		image[3][3] <= image[3][4];
		image[3][2] <= image[3][3];
		image[3][1] <= image[3][2];

		image[2][4] <= image[3][1];
		image[2][3] <= image[2][4];
		image[2][2] <= image[2][3];
		image[2][1] <= image[2][2];

		image[1][4] <= image[2][1];
		image[1][3] <= image[1][4];
		image[1][2] <= image[1][3];
		image[1][1] <= image[1][2];
	end
end
// image padding
always @(*) begin
	// Padding
	image[0][0] <= (opt[0])? 32'b0 : image[1][1];
	image[0][1] <= (opt[0])? 32'b0 : image[1][1];
	image[0][2] <= (opt[0])? 32'b0 : image[1][2];
	image[0][3] <= (opt[0])? 32'b0 : image[1][3];
	image[0][4] <= (opt[0])? 32'b0 : image[1][4];
	image[0][5] <= (opt[0])? 32'b0 : image[1][4];

	image[1][0] <= (opt[0])? 32'b0 : image[1][1];
	image[1][5] <= (opt[0])? 32'b0 : image[1][4];

	image[2][0] <= (opt[0])? 32'b0 : image[2][1];
	image[2][5] <= (opt[0])? 32'b0 : image[2][4];

	image[3][0] <= (opt[0])? 32'b0 : image[3][1];
	image[3][5] <= (opt[0])? 32'b0 : image[3][4];

	image[4][0] <= (opt[0])? 32'b0 : image[4][1];
	image[4][5] <= (opt[0])? 32'b0 : image[4][4];

	image[5][0] <= (opt[0])? 32'b0 : image[4][1];
	image[5][1] <= (opt[0])? 32'b0 : image[4][1];
	image[5][2] <= (opt[0])? 32'b0 : image[4][2];
	image[5][3] <= (opt[0])? 32'b0 : image[4][3];
	image[5][4] <= (opt[0])? 32'b0 : image[4][4];
	image[5][5] <= (opt[0])? 32'b0 : image[4][4];
end


// kernel cnt
always @(posedge clk or negedge rst_n) begin
	if(!rst_n) kernel_cnt <= 8'd0;
	else if(c_state == IDLE) begin
		kernel_cnt <= 8'd0;
	end	else if(c_state == READ || c_state == CALC ) begin
		kernel_cnt <= kernel_cnt + 8'd1;
	end
	else kernel_cnt <= 8'd0;
end
// kernel_conv_1~3 storing
always @(posedge clk)begin
	if (kernel_cnt == 8'd8 || kernel_cnt == 8'd17 || kernel_cnt == 8'd26) begin
		case (kernel_cnt)
			8'd8: begin
				for (i = 0; i < 3; i = i + 1) begin
					for (j = 0; j < 3; j = j + 1) begin
						kernel_conv_1[i][j] <= k_map[i][j];
					end
				end
			end
			8'd17: begin
				for (i = 0; i < 3; i = i + 1) begin
					for (j = 0; j < 3; j = j + 1) begin
						kernel_conv_2[i][j] <= k_map[i][j];
					end
				end
			end
			8'd26: begin
				for (i = 0; i < 3; i = i + 1) begin
					for (j = 0; j < 3; j = j + 1) begin
						kernel_conv_3[i][j] <= k_map[i][j];
					end
				end
			end
			default: ; 
		endcase
	end
end


// kernel : store the given data by shift register
always @(posedge clk) begin
	if (in_valid ==1'b1 && Kernel) begin
		k_map[2][2] <= Kernel;
		k_map[2][1] <= k_map[2][2]; 
		k_map[2][0] <= k_map[2][1];

		k_map[1][2] <= k_map[2][0];
		k_map[1][1] <= k_map[1][2];
		k_map[1][0] <= k_map[1][1];
		
		k_map[0][2] <= k_map[1][0];
		k_map[0][1] <= k_map[0][2];
		k_map[0][0] <= k_map[0][1];
	end
end
// store weight
always @(posedge clk) begin
	if (in_valid ==1'b1 && Weight) begin
		weight1[1][1] <= Weight;
		weight1[1][0] <= weight1[1][1];
		weight1[0][1] <= weight1[1][0];
		weight1[0][0] <= weight1[0][1];
	end
end

always @(posedge clk)begin
	if (kernel_cnt == 8'd3 ) begin
		for (i = 0; i < 2; i = i + 1) begin
			for (j = 0; j < 2; j = j + 1) begin
				 weight[i][j] <= weight1[i][j];
			end
		end
	end
end


// x
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        x <= 2'd0;
    end else begin
        x <= nx;
    end
end

// nx
always @(*) begin
    if (!rst_n) begin
        nx = 2'd0;
	end 
	else if (conv_1_complete || conv_2_complete || conv_3_complete) begin
        nx = 2'd0;
    end 
	else if (conv_cs) begin
        case (conv_cs)
            4'd1, 4'd2, 4'd3: begin
                if (x < 2'd2) nx = x + 2'd1;
                else nx = 2'd0;
            end
            default: nx = 2'd0;
        endcase
    end
	else nx = 2'd0;
end

// y
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        y <= 2'd0;
    end else if (conv_1_complete || conv_2_complete || conv_3_complete) begin
        y <= 2'd0;
    end else begin
        y <= ny;
    end
end

// ny
always @(*) begin
    if (!rst_n) begin
        ny = 2'd0;
	end 
	else if (conv_1_complete || conv_2_complete || conv_3_complete) begin
        ny = 2'd0;
    end 
	else if (conv_cs) begin
        case (conv_cs)
            4'd1, 4'd2, 4'd3: begin
                if (x < 2'd2) ny = y;
                else begin
                    if (y < 2'd2) ny = y + 2'd1;
                    else ny = 2'd0;
                end
            end
            default: ny = 2'd0;
        endcase
    end
	else ny = 2'd0;
end

always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		conv_cnt <= 6'b0; 
		conv_row <= 4'b0;
		conv_layer <= 2'b0;
		c_layer <=  2'b0 ;
		max_pool_complete <= 1'd0;
	end
	else if(conv_cs == 4'b0)begin
		conv_cnt <= 6'b0; 
		conv_row <= 4'b0;
		conv_layer <= 2'b0;
		c_layer <=  2'b0 ;
		max_pool_complete <= 1'd0;
	end else if (conv_cs == 4'd9)begin
		conv_cnt <= 6'b0;
		max_pool_complete <= 1'd0;
	end
	else  begin
		conv_cnt <= conv_cnt + 1'b1;	
	end
end


always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		conv_1_cnt <= 3'd0;
	end
	else if (conv_cs == 4'd2) begin
        conv_1_cnt <= 3'd0;
    end
	else if (conv_cs == 4'd1 && conv_1_cnt == 4'd8) begin
        // conv_1_complete <= 3'b1;
    end
	else if (conv_cs == 4'd1 )conv_1_cnt <= conv_1_cnt + 3'd1;
	else conv_1_cnt <= 3'd0;
end

always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		conv_2_cnt <= 3'd0;
	end
	else if (conv_cs == 4'd2 && conv_2_cnt == 4'd9) begin
        //conv_2_complete <= 3'b1;
    end
	else if (conv_cs == 4'd2)begin 
		conv_2_cnt <= conv_2_cnt + 3'd1;
	end
	else conv_2_cnt <= 3'd0;
end

always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		conv_3_cnt <= 3'd0;
	end 
	else if (conv_cs == 4'd2) begin
        conv_3_cnt <= 3'd0;
    end
	else if (conv_cs == 4'd3 && conv_3_cnt == 4'd9) begin
      //conv_3_complete <= 3'b1;
    end
	else if (conv_cs == 4'd3 )begin 
		conv_3_cnt <= conv_3_cnt + 3'd1;
	end
	else conv_3_cnt <= 3'd0;
end
// conv_1_complete
always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		conv_1_complete <= 3'b0;
	end 
	else if(c_state == IDLE) conv_1_complete <= 3'b0;
	else if (conv_cs == 4'd1 && conv_1_cnt == 4'd8) begin
        conv_1_complete <= 3'b1;
    end
	else if (conv_cs == 4'd2)begin 
		conv_1_complete <= 3'b0;
	end
end
// conv_2_complete
always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		conv_2_complete <= 3'b0;
	end 
	else if(c_state == IDLE) conv_2_complete <= 3'b0;
	else if (conv_cs == 4'd2 && conv_2_cnt == 4'd9) begin
        conv_2_complete <= 3'b1;
    end
	else if(conv_cs == 4'b0 || c_state == IDLE)begin
		conv_2_complete <= 3'b0;
	end
	else if (conv_cs == 4'd3 )begin 
		conv_2_complete <= 3'b0;
	end
end
// conv_3_complete
always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		conv_3_complete <= 3'b0;
	end 
	else if(c_state == IDLE) conv_3_complete <= 3'b0;
	else if (conv_cs == 4'd3 && conv_3_cnt == 4'd9) begin
        conv_3_complete <= 3'b1;
	end
	else if(conv_cs == 4'b0 || c_state == IDLE)begin
		conv_3_complete <= 3'b0;
	end 
	else if ( conv_cs == 4'd4 ) begin
		conv_3_complete <= 3'b0;
	end
end 


always @(*) begin
    case (conv_cs)
        3'd1: begin
            for (i = 0; i < 6; i = i + 1) begin
                for (j = 0; j < 6; j = j + 1) begin
                    img_conv[i][j] = img_conv_1[i][j];
                end
            end
            for (i = 0; i < 3; i = i + 1) begin
                for (j = 0; j < 3; j = j + 1) begin
                    kernel_conv[i][j] = kernel_conv_1[i][j];
                end
            end
        end
        3'd2: begin
            for (i = 0; i < 6; i = i + 1) begin
                for (j = 0; j < 6; j = j + 1) begin
                    img_conv[i][j] = img_conv_2[i][j];
                end
            end
            for (i = 0; i < 3; i = i + 1) begin
                for (j = 0; j < 3; j = j + 1) begin
                    kernel_conv[i][j] = kernel_conv_2[i][j];
                end
            end
        end
        3'd3: begin
            for (i = 0; i < 6; i = i + 1) begin
                for (j = 0; j < 6; j = j + 1) begin
                    img_conv[i][j] = img_conv_3[i][j];
                end
            end
            for (i = 0; i < 3; i = i + 1) begin
                for (j = 0; j < 3; j = j + 1) begin
                    kernel_conv[i][j] = kernel_conv_3[i][j];
                end
            end
        end
        default: begin
            for (i = 0; i < 6; i = i + 1) begin
                for (j = 0; j < 6; j = j + 1) begin
                    img_conv[i][j] = 0;
                end
            end
            for (i = 0; i < 3; i = i + 1) begin
                for (j = 0; j < 3; j = j + 1) begin
                    kernel_conv[i][j] = 0;
                end
            end
        end
    endcase
end

// Convolution
always @(posedge clk) begin
	if (conv_cs && (conv_1_complete || conv_2_complete || conv_3_complete)) begin
			MACU_A[0] <=  0;
			MACU_B[0] <=  0;
			MACU_A[1] <=  0;
			MACU_B[1] <=  0;
			MACU_A[2] <=  0;
			MACU_B[2] <=  0;
			MACU_A[3] <=  0;
			MACU_B[3] <=  0;
			MACU_A[4] <=  0;
			MACU_B[4] <=  0;
			MACU_A[5] <=  0;
			MACU_B[5] <=  0;
			MACU_A[6] <=  0;
			MACU_B[6] <=  0;
			MACU_A[7] <=  0;
			MACU_B[7] <=  0;
			MACU_A[8] <=  0;
			MACU_B[8] <=  0;
			MACU_A[9] <=  0;
			MACU_B[9] <=  0;
			MACU_A[10] <=  0;
			MACU_B[10] <=  0;
			MACU_A[11] <=  0;
			MACU_B[11] <=  0;
			MACU_A[12] <=  0;
			MACU_B[12] <=  0;
			MACU_A[13] <=  0;
			MACU_B[13] <=  0;
			MACU_A[14] <=  0;
			MACU_B[14] <=  0;
			MACU_A[15] <=  0;
			MACU_B[15] <=  0;

			MACU_C[0] <= (conv_cnt  == 32'b0 ) ?  32'b0 : MACU_out [0];
			MACU_C[1] <= (conv_cnt  == 32'b0 ) ?  32'b0 : MACU_out [1];
			MACU_C[2] <= (conv_cnt  == 32'b0 ) ?  32'b0 : MACU_out [2];
			MACU_C[3] <= (conv_cnt  == 32'b0 ) ?  32'b0 : MACU_out [3];
			MACU_C[4] <= (conv_cnt  == 32'b0 ) ?  32'b0 : MACU_out [4];
			MACU_C[5] <= (conv_cnt  == 32'b0 ) ?  32'b0 : MACU_out [5];
			MACU_C[6] <= (conv_cnt  == 32'b0 ) ?  32'b0 : MACU_out [6];
			MACU_C[7] <= (conv_cnt  == 32'b0 ) ?  32'b0 : MACU_out [7];
			MACU_C[8] <= (conv_cnt  == 32'b0 ) ?  32'b0 : MACU_out [8];
			MACU_C[9] <= (conv_cnt  == 32'b0 ) ?  32'b0 : MACU_out [9];
			MACU_C[10] <= (conv_cnt  == 32'b0 ) ?  32'b0 : MACU_out [10];
			MACU_C[11] <= (conv_cnt  == 32'b0 ) ?  32'b0 : MACU_out [11];
			MACU_C[12] <= (conv_cnt  == 32'b0 ) ?  32'b0 : MACU_out [12];
			MACU_C[13] <= (conv_cnt  == 32'b0 ) ?  32'b0 : MACU_out [13];
			MACU_C[14] <= (conv_cnt  == 32'b0 ) ?  32'b0 : MACU_out [14];
			MACU_C[15] <= (conv_cnt  == 32'b0 ) ?  32'b0 : MACU_out [15];

	end
	else if (conv_cs) begin
		MACU_A[0] <= kernel_conv[y][x];
		MACU_B[0] <= img_conv[y][x];
		MACU_C[0] <= (conv_cnt  == 32'b0 ) ?  32'b0 : MACU_out [0];

		MACU_A[1] <= kernel_conv[y][x];
		MACU_B[1] <= img_conv[y][x+1];
		MACU_C[1] <=  (conv_cnt  == 32'b0 ) ?  32'b0 :MACU_out [1];

		MACU_A[2] <= kernel_conv[y][x];
		MACU_B[2] <= img_conv[y][x+2];
		MACU_C[2] <=  (conv_cnt  == 32'b0 ) ?  32'b0 :MACU_out [2];

		MACU_A[3] <= kernel_conv[y][x];
		MACU_B[3] <= img_conv[y][x+3];
		MACU_C[3] <= (conv_cnt  == 32'b0 ) ?  32'b0 : MACU_out [3];

		MACU_A[4] <= kernel_conv[y][x];
		MACU_B[4] <= img_conv[y+1][x];
		MACU_C[4] <=  (conv_cnt  == 32'b0 ) ?  32'b0 :MACU_out [4];

		MACU_A[5] <= kernel_conv[y][x];
		MACU_B[5] <= img_conv[y+1][x+1];
		MACU_C[5] <=  (conv_cnt  == 32'b0 ) ?  32'b0 :MACU_out [5];

		MACU_A[6] <= kernel_conv[y][x];
		MACU_B[6] <= img_conv[y+1][x+2];
		MACU_C[6] <=  (conv_cnt  == 32'b0 ) ?  32'b0 :MACU_out [6];

		MACU_A[7] <= kernel_conv[y][x];
		MACU_B[7] <= img_conv[y+1][x+3];
		MACU_C[7] <=  (conv_cnt  == 32'b0 ) ?  32'b0 :MACU_out [7];

		MACU_A[8] <= kernel_conv[y][x];
		MACU_B[8] <= img_conv[y+2][x];
		MACU_C[8] <=  (conv_cnt  == 32'b0 ) ?  32'b0 :MACU_out [8];

		MACU_A[9] <= kernel_conv[y][x];
		MACU_B[9] <= img_conv[y+2][x+1];
		MACU_C[9] <= (conv_cnt  == 32'b0 ) ?  32'b0 : MACU_out [9];

		MACU_A[10] <= kernel_conv[y][x];
		MACU_B[10] <= img_conv[y+2][x+2];
		MACU_C[10] <= (conv_cnt  == 32'b0 ) ?  32'b0 : MACU_out [10];

		MACU_A[11] <= kernel_conv[y][x];
		MACU_B[11] <= img_conv[y+2][x+3];
		MACU_C[11] <=  (conv_cnt  == 32'b0 ) ?  32'b0 :MACU_out [11];

		MACU_A[12] <= kernel_conv[y][x];
		MACU_B[12] <= img_conv[y+3][x];
		MACU_C[12] <=  (conv_cnt  == 32'b0 ) ?  32'b0 :MACU_out [12];

		MACU_A[13] <= kernel_conv[y][x];
		MACU_B[13] <= img_conv[y+3][x+1];
		MACU_C[13] <=  (conv_cnt  == 32'b0 ) ?  32'b0 :MACU_out [13];

		MACU_A[14] <= kernel_conv[y][x];
		MACU_B[14] <= img_conv[y+3][x+2];
		MACU_C[14] <= (conv_cnt  == 32'b0 ) ?  32'b0 : MACU_out [14];

		MACU_A[15] <= kernel_conv[y][x];
		MACU_B[15] <= img_conv[y+3][x+3];
		MACU_C[15] <=  (conv_cnt  == 32'b0 ) ?  32'b0 :MACU_out [15];
	end
 end
// when conv_cs == 3'b4 , output to f_map
// Output Convolution result to f_map
always @(posedge clk) begin
	if ( conv_cs == 4'd4 ) begin
		f_map[0][0] <= MACU_out [0];
		f_map[0][1] <= MACU_out [1];
		f_map[0][2] <= MACU_out [2];
		f_map[0][3] <= MACU_out [3];
		f_map[1][0] <= MACU_out [4];
		f_map[1][1] <= MACU_out [5];
		f_map[1][2] <= MACU_out [6];
		f_map[1][3] <= MACU_out [7];
		f_map[2][0] <= MACU_out [8];
		f_map[2][1] <= MACU_out [9];
		f_map[2][2] <= MACU_out [10];
		f_map[2][3] <= MACU_out [11];
		f_map[3][0] <= MACU_out [12];
		f_map[3][1] <= MACU_out [13];
		f_map[3][2] <= MACU_out [14];
		f_map[3][3] <= MACU_out [15];
	end
end

//max-pooling
always@(posedge clk or negedge rst_n) begin
	if(!rst_n) max_cs <= 4'd0; /* initial state */
    else if ( conv_cs == 4'd4 ) begin
		max_cs <= 4'b0;
	end
    else 
        max_cs <= max_ns;
end

always @(*) begin
	case (max_cs)
		4'd0 :  max_ns = (compare_ready)? 4'b1 : 4'b0; //IDLE 
		4'd1 :  max_ns = 4'd2; // compare 1  
		4'd2 :  max_ns = 4'd3; // compare 2
		4'd3 :  max_ns = 4'd4; // compare final
		4'd4 :  max_ns = 4'd5; // output max [0][0], [0]][1]
		4'd5 :  max_ns = 4'd6; // 
		4'd6 :  max_ns = 4'd7; // 
		4'd7 :  max_ns = 4'd8; // 
		4'd8 :  max_ns = 4'd9 ; //
		4'd9 :  max_ns = 4'd10 ; //
		4'd10 :  max_ns = 4'd11 ; //
		4'd11 :  max_ns = 4'd0 ; //
		default: begin
			conv_complete = 4'd0;
			max_ns = 4'd0 ; 
		end
	endcase
end

always @(posedge clk) begin
	if(max_cs == 4'd1 )begin
		cmp_a <= f_map[0][0];
		cmp_b <= f_map[0][1];
		cmp_c <= f_map[0][2];
		cmp_d <= f_map[0][3];

		
	end else if ( max_cs == 4'd2)begin
		max_0_1 <= max0;
		max_1_1 <= max1;
		cmp_a <= f_map[1][0];
		cmp_b <= f_map[1][1];
		cmp_c <= f_map[1][2];
		cmp_d <= f_map[1][3];

	end else if (  max_cs == 4'd3 )begin
		max_0_2 <= max0;
		max_1_2 <= max1;		
	end
	else if (max_cs == 4'd4)  begin
		cmp_a <= max_0_1;
		cmp_b <= max_0_2;
		cmp_c <= max_1_1;
		cmp_d <= max_1_2;

	end else if (max_cs == 4'd5) begin
		max_result_0 <= max0;
		max_result_1 <= max1;
	end
	else if(max_cs == 4'd6 )begin
		f_map_2[0][0] <= max_result_0;
		f_map_2[0][1] <= max_result_1;
		cmp_a <= f_map[2][0];
		cmp_b <= f_map[2][1];
		cmp_c <= f_map[2][2];
		cmp_d <= f_map[2][3];
	end else if ( max_cs == 4'd7)begin
		max_0_1 <= max0;
		max_1_1 <= max1;
		cmp_a <= f_map[3][0];
		cmp_b <= f_map[3][1];
		cmp_c <= f_map[3][2];
		cmp_d <= f_map[3][3];
	end else if (  max_cs == 4'd8 )begin
		max_0_2 <= max0;
		max_1_2 <= max1;	
	end
	else if (max_cs == 4'd9)  begin
		cmp_a <= max_0_1;
		cmp_b <= max_0_2;
		cmp_c <= max_1_1;
		cmp_d <= max_1_2;
	end 
	else if (max_cs == 4'd10) begin
		max_result_0 <= max0;
		max_result_1 <= max1;
	end 
	else if (max_cs == 4'd11) begin
		f_map_2[1][0] <= max_result_0;
		f_map_2[1][1] <= max_result_1;
	end
	else begin
		cmp_a <= 1'b0;
		cmp_b <= 1'b0;
		cmp_c <= 1'b0;
		cmp_d <= 1'b0;		
	end
end

// Fully connected:conv_cs == 4'd6
always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
        mul_state  <= 4'd0;
		MULA <= 32'd0;
		MULB <= 32'd0;
		MULA1 <= 32'd0;
		MULB1 <= 32'd0;
		ADD_A <= 32'd0;
		ADD_B <= 32'd0;

    end
	else if (conv_cs == 4'd6 && mul_complete == 0) begin
        case (mul_state )
            4'd0: begin
				MULA <= f_map_2[0][0];
				MULB <= weight[0][0];
				MULA1 <= f_map_2[0][1];
				MULB1 <= weight[1][0];
				mul_state <= 4'd1;
            end
            4'd1: begin
				ADD_A <= MUL_OUT;
				ADD_B <= MUL_OUT1;
                MULA <= f_map_2[0][0];
                MULB <= weight[0][1];
				MULA1 <= f_map_2[0][1];
                MULB1 <= weight[1][1];
                mul_state <= 4'd2;
            end
            4'd2: begin
				feature_map[0] <= ADD_OUT;
				ADD_A <= MUL_OUT;
				ADD_B <= MUL_OUT1;
                MULA <= f_map_2[1][0];
                MULB <= weight[0][0];
				MULA1 <= f_map_2[1][1];
                MULB1 <= weight[1][0];
                mul_state <= 4'd3;
            end
            4'd3: begin
				ADD_A <= MUL_OUT;
				ADD_B <= MUL_OUT1;
                feature_map[1] <= ADD_OUT;				
                MULA <= f_map_2[1][0];
                MULB <= weight[0][1];
				MULA1 <= f_map_2[1][1];
                MULB1 <= weight[1][1];
                mul_state <= 4'd4;
            end
            4'd4: begin
				ADD_A <= MUL_OUT;
				ADD_B <= MUL_OUT1;
				feature_map[2] <= ADD_OUT;
                mul_state <= 4'd5;
				
            end
            4'd5: begin
				feature_map[3] <= ADD_OUT;
                mul_state <= 4'd0; // Reset state or move to the next state
            end
            default: begin
				mul_state <= 4'd0;
			end
        endcase
	 end
end



always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        compare_cs <= 4'd0;
        min_miax_complete <= 1'd0;
		max01 <=32'd0;
		max02 <= 32'd0;
		min01 <= 32'd0;
		min02 <= 32'd0;
		cmp_1 <= 32'd0;
        cmp_2 <= 32'd0;
        cmp_3 <= 32'd0;
        cmp_4 <= 32'd0;
    end else if (c_state == IDLE) begin
        min_miax_complete <= 1'd0;
    end else if (conv_cs == 4'd6 && !mul_complete && mul_state == 4'd5) begin
        compare_cs <= 4'd0;
        min_miax_complete <= 1'd0;
    end else if (conv_ns == 4'd7 && !min_miax_complete) begin
        case (compare_cs)
            1'd0: begin
                cmp_1 <= feature_map[0];
                cmp_2 <= feature_map[1];
                cmp_3 <= feature_map[2];
                cmp_4 <= feature_map[3];
                compare_cs <= 2'd1;
            end
            2'd1: begin
                max01 <= max_1;
                max02 <= max_2;
                min01 <= min_1;
                min02 <= min_2;
                compare_cs <= 2'd2;
            end
            2'd2: begin
                cmp_1 <= max01;
                cmp_2 <= max02;
                cmp_3 <= min01;
                cmp_4 <= min02;
                compare_cs <= 2'd3;
            end
            2'd3: begin
                max01 <= max_1; // max01 saves the max one
                min02 <= min_2; // min02 saves the min one
                min_miax_complete <= 1'd1;
            end
            default: compare_cs <= 2'd0;
        endcase
    end
end


always @(posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		nor_cs <= 3'd0;
		x_scale <= 32'd0;
		SUB_A <= 32'd0;
		SUB_B <= 32'd0;
		DIV_A <= 32'd0;
		DIV_B <= 32'd0;
	end else if (conv_ns == 4'd7 && !min_miax_complete && compare_cs == 2'd3) begin
		nor_cs <= 3'd0;
		x_scale <= 32'd0;
	end
	else if (conv_ns == 4'd7 && min_miax_complete) begin
		case (nor_cs)
			3'd0 :  begin
				SUB_A <= max01;
				SUB_B <= min02;
				nor_cs <= 1'd1;
			end
			3'd1 : begin
				x_scale <= SUB_OUT ;
				SUB_A <= feature_map[0];
				SUB_B <= min02;
				nor_cs <= 3'd2;
			end
			3'd2 : begin		
				DIV_A <= SUB_OUT;
				DIV_B <= x_scale;
				feature_map_normalize[0] <= DIV_OUT;
				SUB_A <= feature_map[1];
				SUB_B <= min02;
				nor_cs <= 3'd3;
			end
			3'd3 : begin
				feature_map_normalize[0] <= DIV_OUT;
				DIV_A <= SUB_OUT;
				DIV_B <= x_scale;
				SUB_A <= feature_map[2];
				SUB_B <= min02;
				nor_cs <= 3'd4;
			end
			3'd4 : begin
				feature_map_normalize[1] <= DIV_OUT;
				DIV_A <= SUB_OUT;
				DIV_B <= x_scale;
				SUB_A <= feature_map[3];
				SUB_B <= min02;
				nor_cs <= 3'd5;
			end
			3'd5 : begin
				feature_map_normalize[2] <= DIV_OUT;
				DIV_A <= SUB_OUT;
				DIV_B <= x_scale;
				nor_cs <= 3'd6;
			end
			3'd6 : begin
				feature_map_normalize[3] <= DIV_OUT;
				nor_cs <= 3'd6;
			end
			default: nor_cs <= 2'd0;
		endcase
	end
end





always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		act_cs <= 4'd0;
		act_in <= 32'd0;
		act_cnt <= 3'd0;
		act_result <= 32'b0;
		EXP0_in <= 32'b0;
		EXP_1 <= 32'b0;
		MACA<= 32'b0;
		MACB<= 32'b0;
		MACC<= 32'b0;
		REC0_in <=32'b0;
		SUB_A0<=32'b0;
		SUB_B0<=32'b0;

	end 
	else if (conv_ns == 4'd7 && min_miax_complete) begin
		case (nor_cs)
			3'd6 : begin
				act_cs <= 3'd0;
				act_cnt <= 3'd0;
				act_result <= 32'b0;
			end
		endcase
	end
	else if (conv_cs == 4'd8 && nor_complete) begin
		case (act_cs)
			4'd0 : begin
				act_in <= feature_map_normalize[act_cnt];
				act_cs <= 4'd1;
			end
			4'd1 : begin
				EXP0_in <= act_in;
				act_cs <= 4'd2;
			end
			4'd2 : begin
				EXP_1 <= EXP0_out1;
				act_cs <= 4'd3;
			end
			4'd3 : begin
				MACA <= EXP_1;
				MACB <= (opt[1] == 0)? one:EXP_1;
				MACC <=  one;
				act_cs <= 4'd4;
			end
			4'd4 : begin
				REC0_in <= MAC_OUT0;
				act_cs <= 4'd5;
			end
			4'd5 : begin
				MACA <= REC0_out;
				MACB <= (opt[1] == 1)? two : one;
				MACC <=  neg_one;
				act_cs <= 4'd6;
			end
			4'd6 : begin
				SUB_A0 <= 32'b0;
				SUB_B0 <= MAC_OUT0;
				act_cs <= 4'd7;
			end
			4'd7 : begin
				act_result <= SUB_OUT0;
				act_cs <= 4'd8;
			end
			4'd8 : begin
				
				encoding_map[act_cnt] <= act_result;
				act_cnt <= act_cnt + 4'b1;
				act_cs <= 4'd9;
			end
			4'd9 : begin
				act_cs <= (act_cnt == 3'd4 ) ? 4'd10 : 4'd0 ;
			end
			4'd10 : begin
				//act_complete <= 1'b1; 
			end
			default: act_cs <= 4'd7;
		endcase
	end
end


always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		img_store_cnt <= 2'd0;
	end 
	else if (conv_cs == 4'd9) begin
		case (img_store_cnt)
			2'd0 : begin
				encoding_map_img1[0] <= encoding_map [0];
				encoding_map_img1[1] <= encoding_map [1];
				encoding_map_img1[2] <= encoding_map [2];
				encoding_map_img1[3] <= encoding_map [3];
				img_store_cnt <= img_store_cnt + 1'd1;
			end 
			2'd1 : begin
				encoding_map_img2[0] <= encoding_map [0];
				encoding_map_img2[1] <= encoding_map [1];
				encoding_map_img2[2] <= encoding_map [2];
				encoding_map_img2[3] <= encoding_map [3];
				img_store_cnt <= img_store_cnt + 1'd1;
			end 
			default: img_store_cnt <= 2'd0;
		endcase
	end
	else  if(c_state == IDLE)begin
		img_store_cnt <= 2'd0;

    end
	
end


// L1 distance
always @(posedge clk or negedge rst_n ) begin
	if(!rst_n) begin
		sum_cs <= 3'b0;
		L1_complete <= 1'd0;
		SUB_A2 <= 32'd0;
		SUB_B2 <= 32'd0;
		temp0 <= 32'd0;
		ADD_A1 <= 32'd0;
		ADD_B1 <= 32'd0;
		output_result<= 32'd0;
	end 
	else if(c_state == IDLE)begin
		L1_complete <= 1'd0;
    end
	else if (conv_cs == 4'd9) begin
		if (img_store_cnt == 2'd1) begin
			sum_cs <= 3'b0;
		end
	end
	else if (conv_cs == 4'd10) begin
		case (sum_cs)
			3'd0 : begin
				SUB_A2 <= encoding_map_img1[0];
				SUB_B2 <= encoding_map_img2[0];
				sum_cs <= 3'd1;
			end 
			3'd1 : begin
				temp0 <= {1'b0 , SUB_OUT2 [30:0]};
				SUB_A2 <= encoding_map_img1[1];
				SUB_B2 <= encoding_map_img2[1];
				sum_cs <= 3'd2;
			end 
			3'd2 : begin
				ADD_A1 <= temp0;
				ADD_B1 <= 32'b0;
				temp0 <= {1'b0 , SUB_OUT2 [30:0]};
				SUB_A2 <= encoding_map_img1[2];
				SUB_B2 <= encoding_map_img2[2];
				sum_cs <= 3'd3;

			end 
			3'd3 : begin
				ADD_A1 <= temp0;
				ADD_B1 <= ADD_OUT1;
				temp0 <= {1'b0 , SUB_OUT2 [30:0]};
				SUB_A2 <= encoding_map_img1[3];
				SUB_B2 <= encoding_map_img2[3];
				sum_cs <= 3'd4;

			end 
			3'd4 : begin
				ADD_A1 <= temp0;
				ADD_B1 <= ADD_OUT1;
				temp0 <= {1'b0 , SUB_OUT2 [30:0]};
				sum_cs <= 3'd5;
			end 
			3'd5 : begin
				ADD_A1 <= temp0;
				ADD_B1 <= ADD_OUT1;
				sum_cs <= 3'd6;
			end 
			3'd6 : begin
				output_result <= ADD_OUT1;
				sum_cs <= 3'd7;
			end
			3'd7 : begin
				L1_complete <= 1'd1;
			end
			default: sum_cs <= 3'd0;
		endcase
	end
end

// conntrol  compare_complete
always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		compare_complete  <= 1'd0;
	end
    else if(c_state == IDLE)begin
		compare_complete  <= 1'd0;
    end
    else if (conv_cs == 4'd9)begin
		compare_complete  <= 1'd0;
	end
	else if (max_cs == 4'd11) begin
		compare_complete <= 4'b1;
	end
end

// conntrol  img_input_cnt
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        img_input_cnt <= 0;
    end else if (c_state == IDLE) begin
        img_input_cnt <= 0;
    end else if ((in_valid || c_state == CALC) && img_input_cnt < 3'd6) begin
        if (img_cnt == 7'd15 || img_cnt == 7'd31 || img_cnt == 7'd47) begin
            img_input_cnt <= img_input_cnt + 1'b1;
        end
    end
end

// mul_complete
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        mul_complete <= 1'd0;
    end else if (c_state == IDLE || max_cs == 4'd11) begin
        mul_complete <= 1'd0;
    end else if (conv_cs == 4'd6 && !mul_complete) begin
        if (mul_state == 4'd5) begin
            mul_complete <= 1'd1;
        end
    end
end
// nor_complete
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        nor_complete <= 1'd0;
    end else if (conv_cs == 4'b0 || c_state == IDLE || conv_cs == 4'd9) begin
        nor_complete <= 1'd0;
    end else if (conv_ns == 4'd7 && min_miax_complete && nor_cs == 3'd6) begin
        nor_complete <= 1'd1;
    end
end

//act_complete
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        act_complete <= 1'd0;
    end else if (conv_ns == 4'd7 && min_miax_complete && nor_cs == 3'd6) begin
        act_complete <= 1'b0;
    end else if (conv_cs == 4'd8 && nor_complete && act_cs == 4'd10) begin
        act_complete <= 1'b1;
    end
end


//compare_ready
always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		compare_ready <= 1'd0;
	end
	else if ( conv_cs == 4'd4 ) begin
		compare_ready <= 1'd1;
	end
	else  if (max_cs == 4'd11) begin
		compare_ready <= 4'b0;
	end
end


// Comb ckt for output logic
always @(posedge clk  or negedge rst_n) begin
	if(!rst_n) begin
		out_valid <= 1'd0;
		out <= 1'd0;
	end
	else if ((conv_cs == 4'd11)) begin 
    out_valid <= 1'd1;
    out <= output_result;
	end else begin
		out_valid <= 1'd0;
		out <= 1'd0;
	end
end


endmodule


