module cpu(im_in_1, im_in_2, op_out);
 input [7:0]im_in_1; //First byte of instruction
 input [7:0]im_in_2; //Second byte of instruction
 output [3:0]op_out; //Op code bits
 //output [1:0]ra_out; //First register ref bits
 //output [1:0]rb_out; //second register ref bits
 //output [7:0]imm_out; //immediate bits
 
 assign op_out = im_in_1[7:4]; //Assign op code bits
 //assign ra_out = im_in_1[3:2]; //Assign first register ref bits
 //assign rb_out = im_in_1[1:0]; //Assign second register ref bits
 //assign imm_out = im_in_2[7:0]; //Assign immediate bits
 
 controller con1 (
 .op_in(op_out),
 .test()
 );
 
endmodule

//Controller module
module controller (op_in, test);
 input [3:0]op_in; //Op code input
 //reg [3:0]alu_sel; //ALU control bits
 //reg reg_wb; //Register wb control bit
 //output [3:0]alu_sel_res; //ALU control bits result
 //output reg_wb_res; //Register wb control bit result
 output [3:0]test;
 
 assign test = op_in;
/*
always @(op_in) begin
 case(op_in) //Controller setup 
    4'b0000 : begin //NOP 0
	 alu_sel = 4'b0000; 
	 reg_wb = 0; 
	 end
	 
    4'b0001 : begin //ADD 1
	 alu_sel = 4'b0001; 
	 reg_wb = 1; 
	 end
	 
    4'b0010 : begin //SUB 2
	 alu_sel = 4'b0010; 
	 reg_wb = 1; 
	 end
	 
    4'b0011 : begin //NAND 3
	 alu_sel = 4'b0011; 
	 reg_wb = 1; 
	 end
	 
    4'b0100 : begin //SHL 4
	 alu_sel = 4'b0100; 
	 reg_wb = 1; 
	 end
	 
    4'b0101 : begin //SHR 5
	 alu_sel = 4'b0101; 
	 reg_wb = 1; 
	 end
	 
    4'b0110 : alu_sel = 4'b0110; //OUT 6
	 
    4'b0111 : alu_sel = 4'b0111; //IN 7
	 
    4'b1000 : begin //MOV 8
	 alu_sel = 4'b1000; 
	 reg_wb = 1; 
	 end
	 
    //4'b1001 : ;//BR 9
    //4'b1010 : ;//BR.Z/BR.N 10
    //4'b1011 : ;//BR.SUB 11
    //4'b1100 : ;//RETURN 12
    //4'b1101 : ;//LOAD 13
    //4'b1110 : ;//STORE 14
    //4'b1111 : ;//LOADIMM 15
 endcase 
end

assign alu_sel_res = alu_sel;
assign reg_wb_res = reg_wb;*/

endmodule

/*
//Register file module
module register_file(ra_in, rb_in, ra_out, rb_out, alu_result, reg_wb, wb_reg);
 reg [7:0] R0 = 8'b00000010; //Register 1
 reg [7:0] R1 = 8'b00000001; //Register 2
 reg [7:0] R2 = 8'b00000000; //Register 3
 reg [7:0] R3 = 8'b00000000; //Register 4
 input [1:0] ra_in; //Ra refernce bits
 input [1:0] rb_in //Rb reference bits
 input [0] reg_wb; //Register wb control bit
 input wb_reg; //Wb register 
 input [7:0] alu_result; //ALU result
 output [7:0] ra_out; //Ra output for ALU
 output [7:0] rb_out; //Rb output for ALU

 instruction_memory instance(
 .ra_out (ra_in), //Ra reference from instruction memory to ra reference for register file
 .rb_out (rb_in)  //Rb reference from instruction memroy to rb reference for register file
 );

always @(ra_in) begin
 case(ra_in) //Set ra output from target register and register for wb
    2'b00 : ra_out = R0, wb_reg = R0;
    2'b01 : ra_out = r1, wb_reg = R1;
    2'b10 : ra_out = R2, wb_reg = R2;
    2'b11 : ra_out = R3, wb_reg = R3;
    default : ra_out = NULL;
 endcase
end

always @(rb_in) begin
 case(rb_in) //Set rb output from target register
    2'b00 : rb_out = R0;
    2'b01 : rb_out = r1;
    2'b10 : rb_out = R2;
    2'b11 : rb_out = R3;
    default : rb_out = NULL;
 endcase
end

 ALU instance(
 .alu_result (alu_result) //ALU result from ALU to ALU result for register file
 );
 
 controller instance(
 .reg_wb (reg_wb) //Reg wb from controller to reg wb for register file
 );

always @(reg_wb) begin
 if (reg_wb = 1'b1) //If reg_wb is set from controller
    1'b1 : we_reg = alu_result; //Wb ALU result to target wb register
end
endmodule

//AlU module
module ALU(A, B, alu_sel, alu_result);
input [7:0] A; //Value 1
input [7:0] B; //Value 2
input [3:0] alu_sel; //ALU operation select bits
output [7:0] alu_result; //ALU result

controller instance(
.alu_sel (alu_sel) //ALU select from controller to ALU select for ALU
);

always @(alu_sel) begin
 case(alu_sel) //Select ALU operation with ALU select bits
 4'b0000: 
    alu_result = A; //NOP 0
 4'b0001: 
    alu_result = A + B; //ADD 1
 4'b0010: 
    alu_result = A - B; //SUB 2
 4'b0011: 
    alu_result = ~(A & B); //NAND 3
 4'b0100: 
    alu_result = A<<B;//SHL 4
 4'b0101: 
    alu_result = A>>B;//SHR 5
 //???4'b0110: //OUT 6
 //???4'b0111: //IN 7
 4'b1000: 
    alu_result = B; //MOV 8
 endcase
end
endmodule */