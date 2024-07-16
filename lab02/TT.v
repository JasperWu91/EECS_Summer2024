module TT(
    //Input Port
    clk,
    rst_n,
	in_valid,
    source,
    destination,

    //Output Port
    out_valid,
    cost
    );

input               clk, rst_n, in_valid;
input       [3:0]   source;
input       [3:0]   destination;
output  reg         out_valid;
output  reg [3:0]   cost;

//==============================================//
//             Parameter and Integer            //
//==============================================//

parameter Idle = 2'd0,
          Read = 2'd1,
          Bfs  = 2'd2, // After reading all the input data , executing BFS algorithm.
          Output_1 = 2'd3; // Output 1 when the source and destination is adjacent.

integer i, j;
genvar g_i, g_j; 

//==============================================//
//                reg and wire                  //
//==============================================//
reg [1:0] state, n_state ; // flag for state

reg adj [0:15][0:15];
reg visited [0:15]; //record visited nodes
reg [3:0] s,d; //record source , destination
reg [3:0] distance; // record the total distance
wire [3:0] distance_add_1 = distance + 4'd1; // distance + 1 for recursivley +


wire adj_next [0:15][0:15];  // To update the adjacent lists according to the input data.
generate
    for (  g_i = 0 ;  g_i <= 15 ;  g_i =  g_i + 1  ) begin : gen_1
        for (g_j = 0 ;  g_j <= 15 ;  g_j =  g_j + 1 ) begin : gen_2
            assign adj_next[g_i][g_j] = ((g_i == source && g_j == destination)  || (g_i == destination && g_j == source)  ) ? 1'b1 : adj[g_i][g_j];
        end
    end
endgenerate


//==============================================//
//                Searching method              //
//==============================================//
wor visit_next [0:15]; // List the nodes which is connected  to the current node.
generate
    for (  g_i = 0 ;  g_i <= 15 ;  g_i =  g_i + 1  ) begin : gen_3
        for (g_j = 0 ;  g_j <= 15 ;  g_j =  g_j + 1 ) begin : gen_4
            if (g_j != g_i) begin
                assign visit_next[g_i] =  (visited[g_j] && adj[g_j][g_i] );
            end else begin
                assign visit_next[g_i]  = visited[g_j];
            end
        end
    end
endgenerate


// Set the flag to determine when to stop
wire bfs_found = visit_next[d]; //Once find the path to destination, bfs complete.

wand bfs_end;
generate
    for (g_i = 0 ;  g_i <= 15 ;  g_i =  g_i + 1  ) begin :gen_5
        assign bfs_end = (visited[g_i] == visit_next[g_i]);     // if all the component of visited[]\is the same as visit_next, the bfs ends.
    end
endgenerate


//==============================================//
//              State transition                //
//==============================================//

always@(posedge clk or negedge rst_n) begin
    if(!rst_n)
        state <= Idle; /* initial state */
    else 
        state <= n_state;
end

//==============================================//
//              Next State Block                //
//==============================================//

always@(*) begin
    n_state <= state; // default state = original state
    case(state)
		Idle :  
            if(in_valid) n_state <= Read;
		Read : 
            if(!in_valid) begin
                if(bfs_found) n_state <= Output_1;
                else n_state <= Bfs;
            end
        Bfs : 
			if (bfs_found || bfs_end) n_state <= Idle; 
        Output_1 :  n_state <= Idle;
    endcase
end
//==============================================//
//     Main function for each stage             //
//==============================================//

always@(posedge clk)begin
    for (i = 0; i <= 15; i = i + 1) visited[i] <= 1'bx; //
	distance <= 4'bx;
    case (state)
		Idle :  begin

            // initializing the adj list 
            for (i = 0; i <= 15; i = i + 1)
                for (j = 0; j <= 15; j = j + 1) adj[i][j] <= 1'b0;
            // record the target destination 
            d <= destination;

            // initializing visited list
            for (i = 0; i <= 15; i = i + 1) visited[i] <= 1'b0;
            // record the source into the visited list
            visited[source] <= 1'b1;
            // initializing the distance
            distance <= 4'b0;
        end

        Read : begin
            if(in_valid) begin 
                // if there is new road, update to the adj list
                for (i = 0; i <= 15; i = i + 1)
                for (j = 0; j <= 15; j = j + 1) adj[i][j] <= adj_next[i][j];
    
                for (i = 0; i <= 15; i = i + 1) visited[i] <= visited[i]; // not change the value when in_valid = 1
                distance <= 4'd0;
            end
            else begin
                for (i = 0; i <= 15; i = i + 1) visited[i] <= visit_next[i]; 
                distance <= distance_add_1;
            end
        end    
        Bfs : begin
            for (i = 0; i <= 15; i = i + 1) visited[i] <= visit_next[i];
			distance <= distance_add_1;
        end
    endcase
end


// Comb ckt for output logic
always @(*) begin
    // initialize the signal 
	out_valid <= 1'b0;
	cost <= 4'b0;
	case (state)
		Bfs : begin
			if (bfs_found) begin
				out_valid <= 1'b1;
				cost <= distance_add_1;
			end
			else if (bfs_end) begin
				out_valid <= 1'b1;
				cost <= 4'd0;
			end
		end
		Output_1 : begin
			out_valid <= 1'b1;
			cost <= 4'd1;
		end
	endcase
end


endmodule 
