import cv32e40p_pkg::*;
//(*DONT_TOUCH = "TURE"*) 
module cv32e40p_mac_ops
(
	input logic						 clk,
	input logic						 rst_n,
	input logic						 enable_i,
	input logic [MAC_OP_WIDTH-1:0]   operator_i,
	input logic [31:0]				 operand_i1,
	input logic [31:0]				 operand_i2,
	input logic				         ex_ready_i,

	input logic [31:0]				 con_data_cnt,     

	// conv cal data modify
	input logic [31:0]				 con_data_cal_cnt,
	input logic	[31:0]				 wb_data_cnt,
	
	input logic [31:0]				 mem_rdata,

	output logic [31:0]				 mem_wdata,

	output logic 					 wb23_active,       //write conv result
	output logic					 wb_finish,         //conv finish signal

	// output logic [31:0]				 result_o,

	output logic					 con_active_o,
    output logic                     con_core_active_o,

	output logic					 ready_o,
	output logic [2:0]				 mac_flag,

	output logic					 con_model,

	// output logic [31:0]        		 con_data[15:0]
	output logic  					finish_s
);

//conv cal
enum logic [2:0] {
	IDLE_CON,
	GET_DATA,
	IFMAP_COMBMATRIX,
	IFMAP_COMPRESS,
	CAL_1,
	CAL_2,
	FINISH_CON
} con_cs,con_ns; 

//conv core format change 
enum logic [2:0] {
	IDLE_CORE,
	CON_CORE_DATA,
	COMBMATRIX,
	COMPRESS,
	FINISH_CON_CORE
}con_core_cs,con_core_ns;

//write result
enum logic {
	IDLE_WB,
	WB23_WDATA
}wb23_cs, wb23_ns ;

typedef enum logic [31:0] { 
	WEIGHT_1 = 32'd1,
	WEIGHT_3 = 32'd3,
	WEIGHT_5 = 32'd5,
	WEIGHT_7 = 32'd7
} common_weights_e;

function logic [31:0] compute_common_subexpr(input logic [31:0] x, input logic [31:0] weight);
        integer remaining_weight;
		logic [31:0] result;
        result = 0;
        // integer remaining_weight;
        remaining_weight = weight;

        // Decompose weight to utilize common subexpressions if possible
        for(int i = 0; i < 3; i ++ ) begin
			if(remaining_weight <= 0) break;
            if (remaining_weight >= WEIGHT_7) begin
                result += ((x << 2) + (x << 1) + x);
                remaining_weight -= WEIGHT_7;
            end else if (remaining_weight >= WEIGHT_5) begin
                result += ((x << 2) + x);
                remaining_weight -= WEIGHT_5;
            end else if (remaining_weight >= WEIGHT_3) begin
                result += ((x << 1) + x);
                remaining_weight -= WEIGHT_3;
            end else if (remaining_weight >= WEIGHT_1) begin
                result += x;
                remaining_weight -= WEIGHT_1;
            end
        end
        return result;
endfunction


//con_logic
logic   con_ready                 ;
logic   con_active                ;
logic   con_get_data              ;

//con_core
logic	con_core_ready				  ;
logic	con_core_active			  	  ;
logic	con_core_get_data			  ;
logic	con_core_finish				  ;

//write back 23
logic wb23_ready		;


logic [31:0] ifmap_row_wise_val [11:0][1:0];

logic [31:0] ifmap_matrix  [15:0]; //4*4 matrix unflod
logic [15:0] max                  ; //input matirx size

// logic ReluORNot;	//Judge Relu or Not



logic [31:0] core33_matrix	[8:0] ; //3*3 conv core
logic [15:0] max_core			  ; //core size

//compress kernel matirx	
logic [31:0] core_val_33_1[11:0]	;   	//kernerl conv33_1 val
logic [31:0] core_val_33_2[11:0]	;   	//kernerl conv33_2 val

logic [31:0] 	core_non_zero_val_all  [23:0]	;	//store all kernel val & num is 12 + 12 = 24
logic [3:0] 	core_non_zero_row_index_all[23:0]	;   	
logic [3:0] 	core_non_zero_col_index_all[23:0]	;
logic [4:0]   	core_non_zero_number 	;

//compress if_matirx	
logic [31:0] ifmap_val_33_1[11:0]	;   	//ifmap conv33_1 val
logic [31:0] ifmap_val_33_2[11:0]	;   	//ifmap conv33_2 val

logic [31:0] ifmap_row_wise_val_33_0[1:0]	;   	
logic [31:0] ifmap_row_wise_val_33_1[1:0]	;
logic [31:0] ifmap_row_wise_val_33_2[1:0]	;   	
logic [31:0] ifmap_row_wise_val_33_3[1:0]	;
logic [31:0] ifmap_row_wise_val_33_4[1:0]	;
logic [31:0] ifmap_row_wise_val_33_5[1:0]	;
logic [31:0] ifmap_row_wise_val_33_6[1:0]	;
logic [31:0] ifmap_row_wise_val_33_7[1:0]	;
logic [31:0] ifmap_row_wise_val_33_8[1:0]	;   	
logic [31:0] ifmap_row_wise_val_33_9[1:0]	;
logic [31:0] ifmap_row_wise_val_33_10[1:0]	;   	
logic [31:0] ifmap_row_wise_val_33_11[1:0]	;

//res conv
logic [31:0] conv_res_val[3:0]	    ;


//temp register
logic [31:0] core_val[23:0]			;

logic [31:0] temp_val [23:0][1:0]	;
logic 		 temp_row_index[11:0]	;


logic [4:0]	cal_cnt	;
logic cal_mult_enable;





logic wb23_flag ;
logic con_flag  ;
logic con_core_flag ;


assign wb_finish = wb23_cs ^ wb23_active;
assign mac_flag = {con_flag , con_core_flag , wb23_flag } ;
assign ready_o = con_ready & con_core_ready & wb23_ready    ;

assign con_active_o = con_active ;
assign con_core_active_o = con_core_active;


assign wb23_flag = (operator_i == WB23_OP) && enable_i;
assign con_flag = (operator_i == CON_OP) && enable_i ;
assign con_core_flag = (operator_i == CON_CORE_OP) && enable_i;

logic mode ;

//finish
always_ff@(posedge clk, negedge rst_n) begin
	if(~rst_n) begin
		finish_s <= 1'b0;
	end
	else begin
		if(operator_i == FINISH_OP && enable_i) begin
			finish_s <= 1'b1;
		end
	end
end

//cal_mul
always_ff@(posedge clk, negedge rst_n) begin
	if(~rst_n) begin
		cal_cnt <= 5'd0;
		for (int i = 0; i < 4; i++) begin
			conv_res_val[i] <= 'd0;
		end
		for (int i = 0; i < 24; i++) begin
			temp_val[i][0] <= 'd0;
			temp_val[i][1] <= 'd0;
			core_val[i] <= 'd0;
		end
		for (int i = 0; i < 16; i++) begin
			ifmap_matrix[i] <= 'd0;
		end
		for (int i = 0; i < 9; i++) begin
			core33_matrix[i] <= 'd0;
		end
		end
	else if(cal_mult_enable) begin
		cal_cnt <= cal_cnt + 5'd1;
	end else begin
		cal_cnt <= 5'd0;
	end
end

always_ff@(posedge clk, negedge rst_n) begin
	if(~rst_n) begin
		for(int i = 0; i < 24; i ++) begin
				temp_val[i][0] = 'd0;
				temp_val[i][1] = 'd0;
		end
			
		for(int i = 0; i < 12; i ++) begin
			temp_row_index[i] = 'd0;
		end
	end
	else begin
		if(cal_mult_enable) begin
			case (cal_cnt)
				0: begin
					for(int j = 0; j < 2; j ++) begin
						temp_val[0][j] = compute_common_subexpr(core_non_zero_val_all[0], ifmap_row_wise_val_33_0[j]);
						temp_row_index[0] = core_non_zero_row_index_all[0];
					end
				end
				1: begin
					for(int j = 0; j < 2; j ++) begin
						temp_val[1][j] = compute_common_subexpr(core_non_zero_val_all[1], ifmap_row_wise_val_33_1[j]);
						temp_row_index[1] = core_non_zero_row_index_all[1];
					end
				end 
				2: begin
					for(int j = 0; j < 2; j ++) begin
						temp_val[2][j] = compute_common_subexpr(core_non_zero_val_all[2], ifmap_row_wise_val_33_2[j]);
						temp_row_index[2] = core_non_zero_row_index_all[2];
					end
				end
				3: begin
					for(int j = 0; j < 2; j ++) begin
						temp_val[3][j] = compute_common_subexpr(core_non_zero_val_all[3], ifmap_row_wise_val_33_3[j]);
						temp_row_index[3] = core_non_zero_row_index_all[3];
					end
				end 
				4: begin
					for(int j = 0; j < 2; j ++) begin
						temp_val[4][j] = compute_common_subexpr(core_non_zero_val_all[4], ifmap_row_wise_val_33_4[j]);
						temp_row_index[4] = core_non_zero_row_index_all[4];
					end
				end
				5: begin
					for(int j = 0; j < 2; j ++) begin
						temp_val[5][j] = compute_common_subexpr(core_non_zero_val_all[5], ifmap_row_wise_val_33_5[j]);
						temp_row_index[5] = core_non_zero_row_index_all[5];
					end
				end 
				6: begin
					for(int j = 0; j < 2; j ++) begin
						temp_val[6][j] = compute_common_subexpr(core_non_zero_val_all[6], ifmap_row_wise_val_33_6[j]);
						temp_row_index[6] = core_non_zero_row_index_all[6];
					end
				end
				7: begin
					for(int j = 0; j < 2; j ++) begin
						temp_val[7][j] = compute_common_subexpr(core_non_zero_val_all[7], ifmap_row_wise_val_33_7[j]);
						temp_row_index[7] = core_non_zero_row_index_all[7];
					end
				end
				8: begin
					for(int j = 0; j < 2; j ++) begin
						temp_val[8][j] = compute_common_subexpr(core_non_zero_val_all[8], ifmap_row_wise_val_33_8[j]);
						temp_row_index[8] = core_non_zero_row_index_all[8];
					end
				end
				9: begin
					for(int j = 0; j < 2; j ++) begin
						temp_val[9][j] = compute_common_subexpr(core_non_zero_val_all[9], ifmap_row_wise_val_33_9[j]);
						temp_row_index[9] = core_non_zero_row_index_all[9];
					end
				end
				10: begin
					for(int j = 0; j < 2; j ++) begin
						temp_val[10][j] = compute_common_subexpr(core_non_zero_val_all[10], ifmap_row_wise_val_33_10[j]);
						temp_row_index[10] = core_non_zero_row_index_all[10];
					end
				end
				11: begin
					for(int j = 0; j < 2; j ++) begin
						temp_val[11][j] = compute_common_subexpr(core_non_zero_val_all[11], ifmap_row_wise_val_33_11[j]);
						temp_row_index[11] = core_non_zero_row_index_all[11];
					end
				end
 
				default: ;
			endcase
		end
	end
end

//get 4*4 input matrix
always_ff@(posedge clk, negedge rst_n) begin
	if(~rst_n) begin
		ifmap_matrix[0] <= 'd0 ;
		ifmap_matrix[1] <= 'd0 ;
		ifmap_matrix[2] <= 'd0 ;
		ifmap_matrix[3] <= 'd0 ;
		ifmap_matrix[4] <= 'd0 ;
		ifmap_matrix[5] <= 'd0 ;
		ifmap_matrix[6] <= 'd0 ;
		ifmap_matrix[7] <= 'd0 ;
		ifmap_matrix[8] <= 'd0 ;
		ifmap_matrix[9] <= 'd0 ;
		ifmap_matrix[10] <= 'd0 ;
		ifmap_matrix[11] <= 'd0 ;
		ifmap_matrix[12] <= 'd0 ;
		ifmap_matrix[13] <= 'd0 ;
		ifmap_matrix[14] <= 'd0 ;	
		ifmap_matrix[15] <= 'd0 ;		
	end
	else begin
		if(con_get_data && !con_model) begin
			ifmap_matrix[con_data_cal_cnt - 1] <= mem_rdata	;
		end
		else if(con_get_data && con_model) begin
			case(con_data_cnt)
					1:begin
						ifmap_matrix[2] <= mem_rdata;
						ifmap_matrix[0] <= ifmap_matrix[2];
					end
					2:begin
						ifmap_matrix[3] <= mem_rdata;
						ifmap_matrix[1] <= ifmap_matrix[3];
					end
					3:begin
						ifmap_matrix[6] <= mem_rdata;
						ifmap_matrix[4] <= ifmap_matrix[6];
					end
					4:begin
						ifmap_matrix[7] <= mem_rdata;
						ifmap_matrix[5] <= ifmap_matrix[7];
					end
					5:begin
						ifmap_matrix[10] <= mem_rdata;
						ifmap_matrix[8] <= ifmap_matrix[10];
					end
					6:begin
						ifmap_matrix[11] <= mem_rdata;
						ifmap_matrix[9]	<= ifmap_matrix[11];
					end
					7:begin
						ifmap_matrix[14] <= mem_rdata;
						ifmap_matrix[12] <= ifmap_matrix[14];
					end
					8:begin
						ifmap_matrix[15] <= mem_rdata;
						ifmap_matrix[13] <= ifmap_matrix[15];
					end
				endcase
		end
		else ;
	end    
end 


//conv core get and change format
//conv core get
always_ff@(posedge clk, negedge rst_n) begin
	if(~rst_n) begin
		core33_matrix[0] <= 'd0 ;
		core33_matrix[1] <= 'd0 ;
		core33_matrix[2] <= 'd0 ;
		core33_matrix[3] <= 'd0 ;
		core33_matrix[4] <= 'd0 ;
		core33_matrix[5] <= 'd0 ;
		core33_matrix[6] <= 'd0 ;
		core33_matrix[7] <= 'd0 ;
		core33_matrix[8] <= 'd0 ;

	end
	else begin
		if(con_core_get_data && max_core == 9) begin
			core33_matrix[con_data_cnt-1] <= mem_rdata;
		end
	end
end	


//conv kernel
//==========================================================================
// 状态寄存器更新
always_ff @(posedge clk or negedge rst_n) begin
    if (~rst_n) begin
        con_core_cs <= IDLE_CORE;
    end else begin
        con_core_cs <= con_core_ns;
    end
end

// 计算下一个状态 (Next State Logic)
always_comb begin
    con_core_ns = con_core_cs;  // 默认下一个状态为当前状态

    case (con_core_cs)
        IDLE_CORE: begin
            if (operator_i == CON_CORE_OP && enable_i) begin
                max_core = operand_i2[15:0];
                con_core_ns = CON_CORE_DATA;
            end
        end

        CON_CORE_DATA: begin
            if (con_data_cnt != max_core) begin
                con_core_ns = CON_CORE_DATA;
            end else begin
                con_core_ns = COMBMATRIX;
            end
        end

        COMBMATRIX: begin
            con_core_ns = COMPRESS;
        end

        COMPRESS: begin
            con_core_ns = FINISH_CON_CORE;
        end

        FINISH_CON_CORE: begin
            if (ex_ready_i)
                con_core_ns = IDLE_CORE;
        end

        default: begin
            con_core_ns = IDLE_CORE;
        end
    endcase
end

// 计算输出信号 (Output Logic)
always_comb begin
    // 默认输出
    con_core_ready = 1'b0;
    con_core_active = 1'b1;
    con_core_get_data = 1'b0;
    con_core_finish = 1'b0;

    case (con_core_cs)
        IDLE_CORE: begin
            con_core_ready 		= 1'b1;
			con_core_active 	= 1'b0	;
			con_core_get_data   = 1'b0	;
			con_core_finish		= 1'b0	;
        end

        CON_CORE_DATA: begin
            con_core_ready 		= 1'b0	;
			con_core_active 	= 1'b1	;
			con_core_get_data   = 1'b1	;
			con_core_finish		= 1'b0	;
        end

        COMBMATRIX: begin
			con_core_ready 		= 1'b0	;
			con_core_active 	= 1'b1	;
			con_core_get_data   = 1'b0	;
			con_core_finish		= 1'b0	;

            // 处理矩阵数据
            core_val_33_1[0] = core33_matrix[0];
            core_val_33_1[1] = core33_matrix[1];
            core_val_33_1[2] = core33_matrix[2];
            core_val_33_1[3] = 0;

            core_val_33_1[4] = core33_matrix[3];
            core_val_33_1[5] = core33_matrix[4];
            core_val_33_1[6] = core33_matrix[5];
            core_val_33_1[7] = 0;

            core_val_33_1[8] = core33_matrix[6];
            core_val_33_1[9] = core33_matrix[7];
            core_val_33_1[10] = core33_matrix[8];
            core_val_33_1[11] = 0;

            core_val_33_2[0] = 0;
            core_val_33_2[1] = core33_matrix[0];
            core_val_33_2[2] = core33_matrix[1];
            core_val_33_2[3] = core33_matrix[2];

            core_val_33_2[4] = 0;
            core_val_33_2[5] = core33_matrix[3];
            core_val_33_2[6] = core33_matrix[4];
            core_val_33_2[7] = core33_matrix[5];

            core_val_33_2[8] = 0;
            core_val_33_2[9] = core33_matrix[6];
            core_val_33_2[10] = core33_matrix[7];
            core_val_33_2[11] = core33_matrix[8];

            core_non_zero_number = 'd0;
            for (int i = 0; i < 24; i++) begin
                core_non_zero_val_all[i] = 'd0;
                core_non_zero_col_index_all[i] = 'd0;
                core_non_zero_row_index_all[i] = 'd0;
            end
        end

        COMPRESS: begin
			con_core_ready 		= 1'b0	;
			con_core_active 	= 1'b1	;
			con_core_get_data   = 1'b0	;
			con_core_finish		= 1'b0	;
            if (max_core == 9) begin
                for (int i = 0; i < 12; i++) begin
                    if (core_val_33_1[i] != 'd0) begin
                        core_non_zero_val_all[core_non_zero_number] = core_val_33_1[i];
                        core_non_zero_col_index_all[core_non_zero_number] = i;
                        core_non_zero_row_index_all[core_non_zero_number] = 0;
                        core_non_zero_number = core_non_zero_number + 'd1;
                    end
                    if (core_val_33_2[i] != 'd0) begin
                        core_non_zero_val_all[core_non_zero_number] = core_val_33_2[i];
                        core_non_zero_col_index_all[core_non_zero_number] = i;
                        core_non_zero_row_index_all[core_non_zero_number] = 1;
                        core_non_zero_number = core_non_zero_number + 'd1;
                    end
                end
            end
			for(int i = 0; i < 24; i ++) begin
				$display("core_non_zero_val_all[%0d] = %0d", i, core_non_zero_val_all[i]);
				$display("core_non_zero_col_index_all[%0d] = %0d", i, core_non_zero_col_index_all[i]);
				$display("core_non_zero_row_index_all[%0d] = %0d", i, core_non_zero_row_index_all[i]);
			end
			$display("core_non_zero_number = %0d", core_non_zero_number);
        end

        FINISH_CON_CORE: begin
            con_core_ready 		= 1'b1	;
			con_core_active 	= 1'b0	;
			con_core_get_data   = 1'b0	;
			con_core_finish		= 1'b1	;
        end

    endcase
end

//================================================
//conv cal 
always_ff @(posedge clk, negedge rst_n) begin
    if(~rst_n) begin
        con_cs <= IDLE_CON;
    end
    else begin
        con_cs <= con_ns;
    end 
end

always_comb begin
	
	con_ready   = 1'b0; 
    con_ns = con_cs;
	con_active  = 1'b1;

    case (con_cs)
        IDLE_CON : begin
            con_ready   =  1'b1    ;
            con_active  =  1'b0    ;
            con_get_data = 1'b0    ;
			if(operator_i == CON_OP && enable_i) begin
				//输入矩阵大小max，通过指令传入
				max = operand_i2[15:0];
				// ReluORNot = operand_i2[16];
				con_model = operand_i2[17];
				con_ns = GET_DATA;
			end	
        end

		GET_DATA: begin
			cal_mult_enable = 1'b0 ;
			
			con_ready   =  1'b0    ;
            con_active  =  1'b1    ;
            con_get_data = 1'b1    ;
			//修改为con_data_cal_cnt
			if(con_model == 0) begin
				if(con_data_cal_cnt < max) begin
					con_ns = GET_DATA;
				end 
				else begin
					//初始化结果矩阵
					for(int i = 0; i < 4; i ++ ) begin
						conv_res_val[i] = 0 ;
					end	
					
					
					con_ns = IFMAP_COMBMATRIX;
					
				end		
			end else if(con_model == 1) begin
				if(con_data_cal_cnt < max/2) begin
					con_ns = GET_DATA;
				end 
				else begin
					//初始化结果矩阵
					for(int i = 0; i < 4; i ++ ) begin
						conv_res_val[i] = 0 ;
					end	
					
					con_ns = IFMAP_COMBMATRIX;
					
				end		
			end
			else ;	
		end

		IFMAP_COMBMATRIX: begin
			cal_mult_enable = 1'b0 ;
			
			con_ready   =  1'b0    ;
            con_active  =  1'b1    ;
            con_get_data = 1'b0    ;

			ifmap_val_33_1[0] = ifmap_matrix[0];
			ifmap_val_33_1[1] = ifmap_matrix[1];
			ifmap_val_33_1[2] = ifmap_matrix[2];
			ifmap_val_33_1[3] = ifmap_matrix[3];

			ifmap_val_33_1[4] = ifmap_matrix[4];
			ifmap_val_33_1[5] = ifmap_matrix[5];
			ifmap_val_33_1[6] = ifmap_matrix[6];
			ifmap_val_33_1[7] = ifmap_matrix[7];

			ifmap_val_33_1[8] =  ifmap_matrix[8];
			ifmap_val_33_1[9] =  ifmap_matrix[9];
			ifmap_val_33_1[10] = ifmap_matrix[10];
			ifmap_val_33_1[11] = ifmap_matrix[11];

			ifmap_val_33_2[0] = ifmap_matrix[4];
			ifmap_val_33_2[1] = ifmap_matrix[5];
			ifmap_val_33_2[2] = ifmap_matrix[6];
			ifmap_val_33_2[3] = ifmap_matrix[7];

			ifmap_val_33_2[4] = ifmap_matrix[8];
			ifmap_val_33_2[5] = ifmap_matrix[9];
			ifmap_val_33_2[6] = ifmap_matrix[10];
			ifmap_val_33_2[7] = ifmap_matrix[11];

			ifmap_val_33_2[8] =   ifmap_matrix[12];
			ifmap_val_33_2[9] =   ifmap_matrix[13];
			ifmap_val_33_2[10] =  ifmap_matrix[14];
			ifmap_val_33_2[11] =  ifmap_matrix[15];

			con_ns = IFMAP_COMPRESS;
		end
		IFMAP_COMPRESS: begin
			cal_mult_enable = 1'b0 ;
			
			con_ready   =  1'b0    ;
            con_active  =  1'b1    ;
            con_get_data = 1'b0    ;

			ifmap_row_wise_val_33_0[0] = ifmap_val_33_1[0];
			ifmap_row_wise_val_33_0[1] = ifmap_val_33_2[0];
			
			ifmap_row_wise_val_33_1[0] = ifmap_val_33_1[1];
			ifmap_row_wise_val_33_1[1] = ifmap_val_33_2[1];
			
			ifmap_row_wise_val_33_2[0] = ifmap_val_33_1[2];
			ifmap_row_wise_val_33_2[1] = ifmap_val_33_2[2];
			
			ifmap_row_wise_val_33_3[0] = ifmap_val_33_1[3];
			ifmap_row_wise_val_33_3[1] = ifmap_val_33_2[3];
			
			ifmap_row_wise_val_33_4[0] = ifmap_val_33_1[4];
			ifmap_row_wise_val_33_4[1] = ifmap_val_33_2[4];
			
			ifmap_row_wise_val_33_5[0] = ifmap_val_33_1[5];
			ifmap_row_wise_val_33_5[1] = ifmap_val_33_2[5];
			
			ifmap_row_wise_val_33_6[0] = ifmap_val_33_1[6];
			ifmap_row_wise_val_33_6[1] = ifmap_val_33_2[6];
			
			ifmap_row_wise_val_33_7[0] = ifmap_val_33_1[7];
			ifmap_row_wise_val_33_7[1] = ifmap_val_33_2[7];
			
			ifmap_row_wise_val_33_8[0] = ifmap_val_33_1[8];
			ifmap_row_wise_val_33_8[1] = ifmap_val_33_2[8];
			
			ifmap_row_wise_val_33_9[0] = ifmap_val_33_1[9];
			ifmap_row_wise_val_33_9[1] = ifmap_val_33_2[9];

			ifmap_row_wise_val_33_10[0] = ifmap_val_33_1[10];
			ifmap_row_wise_val_33_10[1] = ifmap_val_33_2[10];
			
			ifmap_row_wise_val_33_11[0] = ifmap_val_33_1[11];
			ifmap_row_wise_val_33_11[1] = ifmap_val_33_2[11];
			
			
			for (int i = 0; i < 24 ; i ++ ) begin
				core_val[i] = 'd0;
			end
			
			con_ns = CAL_1;
		end

		CAL_1: begin
			cal_mult_enable = 1'b1 ;

			con_ready   =  1'b0    ;
            con_active  =  1'b1    ;
            con_get_data = 1'b0    ;

				if(max_core == 'd9) begin
					ifmap_row_wise_val[0] = ifmap_row_wise_val_33_0;
					ifmap_row_wise_val[1] = ifmap_row_wise_val_33_1;
					ifmap_row_wise_val[2] = ifmap_row_wise_val_33_2;
					ifmap_row_wise_val[3] = ifmap_row_wise_val_33_3;
					ifmap_row_wise_val[4] = ifmap_row_wise_val_33_4;
					ifmap_row_wise_val[5] = ifmap_row_wise_val_33_5;
					ifmap_row_wise_val[6] = ifmap_row_wise_val_33_6;
					ifmap_row_wise_val[7] = ifmap_row_wise_val_33_7;
					ifmap_row_wise_val[8] = ifmap_row_wise_val_33_8;
					ifmap_row_wise_val[9] = ifmap_row_wise_val_33_9;
					ifmap_row_wise_val[10] = ifmap_row_wise_val_33_10;
					ifmap_row_wise_val[11] = ifmap_row_wise_val_33_11;
					if(cal_cnt < 12) begin
						con_ns = CAL_1;
					end else begin
						// for(int i = 0; i < 24; i ++) begin
						// 	// $display("temp_val[%0d][0] = %d", i, temp_val[i][0]);
						// 	// $display("temp_val[%0d][1] = %d", i, temp_val[i][1]);
						// end
						// for(int i = 0; i < 12 ; i ++) begin
						// 	$display("temp_row_index[%0d] = %d", i, temp_row_index[i]);
						// end
						con_ns = CAL_2;
					end


				end			
			else ; 
			// con_ns = CAL_2;


		end

		CAL_2: begin
			cal_mult_enable = 1'b0 ;
			
			con_ready   =  1'b0    ;
            con_active  =  1'b1    ;
            con_get_data = 1'b0    ;

			//初始化结果矩阵
			for(int i = 0; i < 4; i ++ ) begin
				conv_res_val[i] = 0 ;
			end	

			if(max_core == 'd9) begin
				for(int i = 0; i < 12; i ++) begin
					if(temp_row_index[i] == 'd0) begin
						conv_res_val[0] = conv_res_val[0] + temp_val[i][0];
						conv_res_val[2] = conv_res_val[2] + temp_val[i][1];
					end
					else if(temp_row_index[i] == 'd1) begin
						conv_res_val[1] = conv_res_val[1] + temp_val[i][0];
						conv_res_val[3] = conv_res_val[3] + temp_val[i][1];
					end
				end
				// 打印 conv_res_val 的所有值
				// $display("conv_res_val[0] = %d", conv_res_val[0]);
				// $display("conv_res_val[1] = %d", conv_res_val[1]);
				// $display("conv_res_val[2] = %d", conv_res_val[2]);
				// $display("conv_res_val[3] = %d", conv_res_val[3]);
			end
			else ;

			con_ns = FINISH_CON ;

		end
		
		FINISH_CON: begin
			cal_mult_enable = 1'b0 ;
			
			con_ready	= 1'b1;
			con_active 	= 1'b0;
			con_get_data = 1'b0;
			if(ex_ready_i)
				con_ns = IDLE_CON ;
		end

    endcase
end



//write back
always_ff @( posedge clk, negedge rst_n ) begin
	if(~rst_n) begin
		wb23_cs <= IDLE_WB;
	end
	else begin
		wb23_cs <= wb23_ns ;
	end
end

 always_comb
 begin
	wb23_ready = 1'b1;
	wb23_ns = wb23_cs;
	wb23_active = 1'b0;
	case (wb23_cs)
		IDLE_WB: begin
			wb23_active = 1'b0;
			wb23_ready = 1'b1;
			if(operator_i == WB23_OP && enable_i) begin
				
				wb23_ns = WB23_WDATA ;
			end
		end 
		
		WB23_WDATA: begin
			wb23_active = 1'b1;
			wb23_ready = 1'b0;
			if(max_core == 9) begin
				if(wb_data_cnt < 4) begin
					case (wb_data_cnt)
						0:mem_wdata = conv_res_val[0];
						1:mem_wdata = conv_res_val[1];
						2:mem_wdata = conv_res_val[2];
						3:begin 
							mem_wdata = conv_res_val[3];
							wb23_ready = 1'b1;
							wb23_ns = IDLE_WB;
						end
						default: mem_wdata = 32'h0;
					endcase
				end
			end
			else begin
				wb23_ns = IDLE_WB;
				wb23_active = 1'b0;
				wb23_ready = 1'b1;
			end
		end
		
	endcase
 end
endmodule
