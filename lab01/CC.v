module CC(
	in_n0,
	in_n1, 
	in_n2, 
	in_n3, 
    in_n4, 
	in_n5, 
	opt,
    equ,
	out_n
);
//================================================================
//   INPUT AND OUTPUT DECLARATION                         
//================================================================
input [3:0]in_n0;
input [3:0]in_n1;
input [3:0]in_n2;
input [3:0]in_n3;
input [3:0]in_n4;
input [3:0]in_n5;
input [2:0] opt;
input equ;
output signed[8:0] out_n;
//==================================================================
// reg & wire
//==================================================================

reg signed [4:0] arr [5:0];
reg signed [4:0] iarr [5:0];

reg signed [4:0] avg_val;
reg  [4:0] temp;
reg signed [8:0] eq0_result;
reg signed [8:0] eq1_result;
reg  [8:0] out_n;

reg signed [4:0] out0, out1, out2, out3, out4, out5;
reg signed [4:0] a[0:3], b[0:2], c[0:3], d[0:2], e[0:5], f[0:3], g[0:1];

//================================================================
//    DESIGN
//================================================================

always@(*) begin

//================================================================
//    Operation 1 : Signed or unsigned
//================================================================

    iarr[0] = (opt[0])? {in_n0[3],in_n0} : {1'b0,in_n0};
    iarr[1] = (opt[0])? {in_n1[3],in_n1} : {1'b0,in_n1};
    iarr[2] = (opt[0])? {in_n2[3],in_n2} : {1'b0,in_n2};
    iarr[3] = (opt[0])? {in_n3[3],in_n3} : {1'b0,in_n3};
    iarr[4] = (opt[0])? {in_n4[3],in_n4} : {1'b0,in_n4};
    iarr[5] = (opt[0])? {in_n5[3],in_n5} : {1'b0,in_n5};

//================================================================
//    Operation 2 : sorting 
//================================================================

    a[0] = ( iarr[0] > iarr[1] ) ? iarr[0] : iarr[1] ;
    a[1] = ( iarr[0] > iarr[1] ) ? iarr[1] : iarr[0] ;
    a[2] = ( a[1] > iarr[2] ) ? a[1] : iarr[2] ;
    a[3] = ( a[1] > iarr[2] ) ? iarr[2] : a[1] ;
    b[0] = ( a[0] > a[2] ) ? a[0] : a[2] ;
    b[1] = ( a[0] > a[2] ) ? a[2] : a[0] ;
    b[2] = a[3] ;

    c[0] = ( iarr[3] > iarr[4] ) ? iarr[3] : iarr[4] ;
    c[1] = ( iarr[3] > iarr[4] ) ? iarr[4] : iarr[3] ;
    c[2] = ( c[1] > iarr[5] ) ? c[1] : iarr[5] ;
    c[3] = ( c[1] > iarr[5] ) ? iarr[5] : c[1] ;
    d[0] = ( c[0] > c[2] ) ? c[0] : c[2] ;
    d[1] = ( c[0] > c[2] ) ? c[2] : c[0] ;
    d[2] = c[3] ;

    e[0] = ( b[0] > d[0] ) ? b[0] : d[0] ;
    e[1] = ( b[0] > d[0] ) ? d[0] : b[0] ;
    e[2] = ( b[1] > d[1] ) ? b[1] : d[1] ;
    e[3] = ( b[1] > d[1] ) ? d[1] : b[1] ;
    e[4] = ( b[2] > d[2] ) ? b[2] : d[2] ;
    e[5] = ( b[2] > d[2] ) ? d[2] : b[2] ;

    arr[5] = e[0] ;
    arr[0] = e[5] ;

    f[0] = ( e[1] > e[2] ) ? e[1] : e[2] ;
    f[1] = ( e[1] > e[2] ) ? e[2] : e[1] ;
    f[2] = ( e[3] > e[4] ) ? e[3] : e[4] ;
    f[3] = ( e[3] > e[4] ) ? e[4] : e[3] ;
    arr[4] = f[0] ;
    arr[1] = f[3] ;

    g[0] = ( f[1] > f[2] ) ? f[1] : f[2] ;
    g[1] = ( f[1] > f[2] ) ? f[2] : f[1] ;
    arr[3] = g[0] ;
    arr[2] = g[1] ;

    // If opt[1] = 1 , From largest to smallest 

    if(opt[1]) begin
        temp = arr[0];
        arr[0] = arr[5];
        arr[5] = temp;
        temp = arr[1];
        arr[1] = arr[4];
        arr[4] = temp;
        temp = arr[2];
        arr[2] = arr[3];
        arr[3] = temp;
    end

//================================================================
//    Operation 3 : Normalization 
//================================================================

    if(opt[2]) begin
        avg_val = (arr[0] + arr[5]) / 2 ;
        arr[0] = arr[0] - avg_val;
        arr[1] = arr[1] - avg_val;
        arr[2] = arr[2] - avg_val;
        arr[3] = arr[3] - avg_val;
        arr[4] = arr[4] - avg_val;
        arr[5] = arr[5] - avg_val;
           
	 end 

//================================================================
//    Output
//================================================================    

        eq0_result = (arr[0] -arr[1] * arr[2] + arr[5]) / 3;
        eq1_result = (arr[3] * 3) - (arr[0] * arr[4]);
        out_n=(equ == 0) ? eq0_result : ( eq1_result[8] ) ?  ~eq1_result+1:eq1_result;

end


endmodule 
