/*
	File: assignment4.v
	Author: Samuel Britton
	Modules:
		Assignment4  	-  The "main" testbench that runs and tests other modules
		ALUControl		-  The ALUControl module takes the instruction in and decides the operation for the ALU to preform
		Control 		-  The Control unit recieves an opcode and outputs the appropriate output signals
		DataMemory      -  The DataMemory module that stores 1024 32 bit values to simulate memory
		SignExtender 	-  The SignExtender module that takes 16 bit values and appends 0s to make it 32 bits
		RegisterFile	-  The RegisterFile contains 32 32 bit registers that contain values
		MIPSALU 		-  The MIPSALU module wil preform a predefined set of operations on A and B based on the contents of Zero and ALUctl and output the value to ALUout
		Add 			-  The Add module that increments a value
		PC 				-  The ProgramCounter module that keeps track of the current instruction
		IM 				-  The InstructionMemory module that reads from "memory"
		clock  			-  The module that constantly "ticks" used to represent a system's clock
*/

/*
	Module: Assignment1 - The "main" testbench that runs and tests other modules
	Author: Samuel Britton
	Ports: none
*/
module Assignment4;
	wire [31:0] PCtoAdd; 
	wire [31:0] addToPC;
	Add i1(addToPC,PCtoAdd,32'd1);		
	
	wire clockWire;
	Clock c1(clockWire);				
	
	PC p1(PCtoAdd,addToPC,clockWire);

	wire [31:0] IMOut;	
	IM im1(IMOut,PCtoAdd,clockWire);				
	

	reg [31:0]	aluA;
	reg [31:0]	aluB;
	wire [3:0] Ctl;
	wire[31:0]	aluOut;
	reg zero;
	MIPSALU alu(aluOut, zero, Ctl, aluA, aluB);

	wire [31:0] seOut;
	reg [15:0] seIn;
	SignExtender se(seOut,seIn);

	
	reg[31:0] address;
	reg memRead;
	reg dmMemWrite;
	wire[31:0] readData;
	reg[31:0] dmWriteData;
	DataMemory dm(readData,clockWire,address,dmWriteData,dmMemWrite,memRead);

	reg [5:0] read1,read2,writeReg;
	wire [31:0] data1,data2;
	reg[31:0] writeData;
	reg regWrite;
	RegisterFile regi(data1,data2,writeReg,writeData,regWrite,read1,read2,clockWire);
	
	wire regDest,ALUSrc,memWrite,memToReg,PCSrc,jump,branch;
	wire regWrite2,memRead2;
	Control con(regDest,regWrite2,ALUSrc,memWrite,memRead2,memToReg,PCSrc,jump,branch,IMOut[31:26],clockWire);

    ALUControl aluc(Ctl,IMOut);

	initial								
		begin				
			//This fills the Instruction memory with the address of the data to be read in
			$readmemb( "program2.dat", im1.my_memory); //fills my_memory of the instruction memory
			
			//No additional tests have to be in this block as the data file has cases for each of the required commands
			$monitor("Program binary:%b ALUSig:%b",IMOut,Ctl);

			#400 $finish;	//After 40 clock ticks it is terminated as to not let it run forever
		end			
endmodule

/*
	Module: ALUControl - The ALUControl module takes the instruction in and decides the operation for the ALU to preform
	Author: Samuel Britton
	Ports: 
		ALUSig(O/P) 		- The 4-bit value that determines what the ALU is to do
		Instruction(I/P)    - The 32-bit value used as input to figure out what ALUSig to use
*/
module ALUControl(ALUSig,Instruction);
    input [31:0] Instruction;
    output [3:0] ALUSig;
    reg    [3:0] ALUSig;
    always @(Instruction) begin
		casex(Instruction)
			(32'b000000xxxxxxxxxxxxxxxxxxxx100000):ALUSig = 4'b0010;//add
			(32'b000000xxxxxxxxxxxxxxxxxxxx100100):ALUSig = 4'b0000;//and
			(32'b000000xxxxxxxxxxxxxxxxxxxx011010):ALUSig = 4'b0001;//div
			(32'b000000xxxxxxxxxxxxxxxxxxxx011000):ALUSig = 4'b0011;//mul + mult
			(32'b000000xxxxxxxxxxxxxxxxxxxx100101):ALUSig = 4'b0001;//or
			(32'b000000xxxxxxxxxxxxxxxxxxxx000000):ALUSig = 4'b0100;//sl
			(32'b000000xxxxxxxxxxxxxxxxxxxx101010):ALUSig = 4'b0111;//slt
			(32'b000000xxxxxxxxxxxxxxxxxxxx000011):ALUSig = 4'b1001;//sra
			(32'b000000xxxxxxxxxxxxxxxxxxxx000010):ALUSig = 4'b0101;//srl
			(32'b000000xxxxxxxxxxxxxxxxxxxx100010):ALUSig = 4'b0110;//sub

			(32'b10x011xxxxxxxxxxxxxxxxxxxx100000):ALUSig = 4'b0010;//lw and sw

			(32'b001111xxxxxxxxxxxxxxxxxxxxxxxxxx):ALUSig = 4'b1000;//lui
			(32'b000100xxxxxxxxxxxxxxxxxxxxxxxxxx):ALUSig = 4'b1010;//beq
			(32'b000101xxxxxxxxxxxxxxxxxxxxxxxxxx):ALUSig = 4'b1011;//bne
			default: ALUSig = 4'b1111;
			//The remaining commands that need to be supported (j,jal,jr,mfhi,mflo,syscall) don't need the alu ((AS FAR AS I KNOW))
		endcase
    end


endmodule



/*
	Module: Control - The Control unit recieves an opcode and outputs the appropriate output signals
	Author: Samuel Britton
	Ports: 
		regDest(O/P) - True if destination number for the Write register is taken from rt
		regWrite(O/P)- True if the WriteRegister input is written with the value on the WriteData
		ALUSrc(O/P)  - True if the second alu operand is the sign-extended, lower 16 bits of the instruction
		memWrite(O/P)- True if data memory contents designated by address input are present at the WriteData input
		memRead(O/P) - True if data memory contents designated by address input are present at the ReadData output
		memToReg(O/P)- True if we are to use data from memory into write register
		PCSrc(O/P)   - True if PC overwritten by the branch target address
		jump(O/P)    - True if we are to jump
		branch(O/P)  - True if we are to branch
		IMin(I/P)    - The 6 bit value that determines the other output
		clock(I/P)	 - The clock that keeps the syncranous element in time
*/
module Control(regDest, regWrite, ALUSrc, memWrite, memRead, memToReg, PCSrc,jump,branch, IMin,clock);
	input [5:0] IMin;
	input zero,clock;
	output regDest,regWrite,ALUSrc,memWrite,memRead,memToReg,PCSrc,jump,branch;
	reg regDest,regWrite,ALUSrc,memWrite,memRead,memToReg,PCSrc,jump,branch;
	always @(IMin) begin
		case(IMin[5:0])
			(6'b000000): begin //Any of the R-types
				regDest = 1;
				ALUSrc = 0;
				memToReg = 0;
				regWrite = 1;
				memRead = 0;
				memWrite = 0;
				jump = 0;
				branch = 0;end
			(6'b001111): begin //the LUI command
				regDest  = 1;
				jump = 0;
				branch = 0;
				memRead = 0;
				memToReg = 0;
				memWrite = 0;
				ALUSrc = 1;
				regWrite = 1;end
			(6'b000100):begin //beq
				ALUSrc = 0;
				regWrite = 0;
				memRead = 0;
				memWrite = 0;
				branch = 1;
				jump = 0;end
			(6'b000101):begin //bne
				ALUSrc = 0;
				regWrite = 0;
				memRead = 0;
				memWrite = 0;
				branch = 1;
				jump = 0;end
			(6'b000101):begin //j
				regWrite = 0;
				memRead = 0;
				memWrite = 0;
				branch = 0;
				jump = 1;end
			(6'b000011):begin //jal - requires a register value to be stored (The incremented program counter) //This will be tough in the final project cause we'll need to save the address later
				regWrite = 0;//"That was a complicated instruction"
				ALUSrc = 1;//This won't use the ALU
				regWrite = 0;
				memRead = 1;
				memWrite = 0;
				branch = 0;
				jump = 1;end
			(6'b100011): begin //lw
				regDest = 0;
				regWrite = 1;
				ALUSrc = 1;
				memRead = 1;
				memWrite = 0;
				branch = 0;end
			(6'b101011): begin //sw
				regWrite = 0;
				ALUSrc = 0;
				memRead = 0;
				memWrite = 1;
				branch = 0;end
			default: 
			begin
			end
		endcase
	end
endmodule

/*
	Module: DataMemory - The DataMemory module that stores 1024 32 bit values to simulate memory
	Authors: Dr. Richard A. Goodrum, Ph.D. 
	Modified by: Samuel Britton
	Ports: 
		readData (O/P) - The data from "memory" that is read
		clock    (I/P) - The clock that tells datamemory when to update
		address  (I/P) - The address from wich to read or write
		writeData(I/P) - it's the data we're writing 
		memWrite (I/P) - the flag signaling we will be reading from memory
		memRead  (I/P) - the flag signaling we will be writing to memory
*/
module DataMemory(readData,clock,address,writeData,memWrite,memRead);	
		input [31:0] address;
		input [31:0] writeData;
		input memWrite,memRead;
		input clock;
		output[31:0] readData;
		reg [31:0] readData;
		reg[31:0] mem[0:1023];
		
		always @(clock)
			if(memRead)
				readData = mem[address];
		always @(clock)
			if(memWrite)
				mem[address]=writeData;
endmodule

/*
	Module: SignExtender - The SignExtender module that takes 16 bit values and appends 0s to make it 32 bits
	Author: Samuel Britton
	Ports: 
		Out(O/P) - The 32 bit extended value
		val(I/P) - The 16 bit 
*/
module SignExtender(out,in);
		input [15:0] in;
		output [31:0] out;
		assign out = {{16{in[15]}},in};
endmodule
/*
	Module: RegisterFile - The RegisterFile contains 32 32 bit registers that contain values
	Author: Dr. Richard A. Goodrum, Ph.D.
	Ports: 
		Data1(O/P) - the reg containing the data in register Read1
		Data2(O/P) - the reg containing the data in register Read2
		WriteReg (I/P) - The index of the register to be written to
		WriteData(I/P) - The Data to be saved in WriteReg
		regWrite (I/P) - The flag that determines if we write write to 
		Read1(O/P) - the index of the register to fill Data1 with
		Read2(O/P) - the index of the register to fill Data2 with
		
*/
module RegisterFile(Data1,Data2,WriteReg,WriteData,RegWrite,Read1,Read2,clock);
  input [5:0] Read1,Read2,WriteReg; // the register numbers to read or write
  input [31:0] WriteData; // data to write
  input RegWrite, // the write control
           clock; // the clock to trigger write
  output [31:0] Data1, Data2; // the register values read
  reg [31:0] RF [31:0]; // 32 registers each 32 bits long

  assign Data1 = RF[Read1];
  assign Data2 = RF[Read2];

  always begin// write the register with new value if Regwrite is high
      @(clock) 
      	if (RegWrite) 
      		RF[WriteReg] <= WriteData;
  end

endmodule

/*
	Module: MIPSALU - The MIPSALU module wil preform a predefined set of operations on A and B based on the contents of Zero and ALUctl and output the value to ALUout
	Author: Dr. Richard A. Goodrum, Ph.D.
	Ports: 
		ALUout(O/P) - The result of the operation
		Zero  (I/P) - The single bit that determines if the ALU produces 0 as an output
		ALUctl(I/P) - The 4 bit control that determines which operation is preformed
		A     (I/P) - The first of two values to preform the operation on
		B     (I/P) - The second of two values to preform the operation on
*/
module MIPSALU (ALUOut, Zero, ALUctl, A, B );
	input [3:0] ALUctl;
	input [31:0] A, B;
	output reg [31:0] ALUOut;
	input Zero;
	assign Zero = (ALUOut==0); // Zero is 1 if ALUOut is 0
	always @( ALUctl, A, B )
			case (ALUctl)
				0: ALUOut <= A & B;
				1: ALUOut <= A | B;
				2: ALUOut <= A + B;
				3: ALUOut <= A * B;
				4: ALUOut <= A << B;
				5: ALUOut <= A >> B;
				6: ALUOut <= A - B;
				7: ALUOut <= A < B ? 1 : 0;
				8: ALUOut <= A << 16;
				9: ALUOut <= A <<< B;
				10: ALUOut <= A == B ? 1 : 0;
				11: ALUOut <= A != B ? 1 : 0;
				12: ALUOut <= ~(A | B); // nor
				default: ALUOut <= 0;
			endcase
endmodule

/*
	Module: Add - The Add module that increments a value
	Author: Samuel Britton
	Ports: 
		Out(O/P) - the incremented value from input
		val(I/P) - The value to be incremented
*/
module Add(out,val,val2);
	input [31:0] val;
	input [31:0] val2;
	output[31:0] out;
	reg[31:0] out;
	always @(val) //whenever val changes, we immediatly increment
		out[31:0] = val+val2;
endmodule


/*
	Module: PC - The ProgramCounter module that keeps track of the current instruction
	Author: Samuel Britton
	Ports: 
		Out   (O/P) - the instruction address stored in the module
		in    (I/P) - The next value to be "stored" or outputted
		clock (I/P) - The clock that teells the PC when to update
*/
module PC(out, in, clock);
	input [31:0] in;
	input clock;
	output [31:0] out;
	reg [31:0] out;
	initial//hey you shouldn't have initial blocks here
		out <= 0; 
	always @(clock)begin //On every clock tick the program counter should start outputting the value from the add module
		out[31:0]=in[31:0];
		end
endmodule
/*
	Module: IM - The InstructionMemory module that reads from "memory"
	Author: Samuel Britton
	Ports: 
		Word   (O/P) - The value read from data.dat at location [addr]
		Addr   (I/P) - The Address of the memory to be read
*/
module IM(word, addr,clock);
	input clock;
	input [31:0] addr;
	output [31:0] word;
	reg [31:0] word;
	reg [31:0] my_memory [0:255]; //we put the contents from data in this. This is 8 bit words of length 256
	always @(addr)
		word[31:0] = my_memory[addr]; //outputs the word for every new address
	
endmodule

/*
 *  Module:	Clock		- A module which periodically changes the output signal.
 *  Author:	Dr. Richard A. Goodrum, Ph.D.
 *	Ports:
 *		clock- O/P	wire	A single bit which toggles on a periodic basis.  The period is dependent on the parameters LO and HI.
 */
module Clock( clock );		// Establish the name of the module and ports.
/*
 *	LO represents the number of ticks during which the output will be low.
 *  HI represents the number of ticks during which the output will be high.
 */
	parameter LO = 2, HI = 2;
	output reg clock;			// Establish a storage location for the oscillating output signal.

	initial						// Loop structure which executes only once during start-up.
		clock = 0;				// clock is intended to start as a low value.

	always						// Loop structure which executes continuously after start-up.
		begin					// Only a code block within the loop.
			#LO	clock = ~clock;	// Toggle the value of clock after LO ticks.
			#HI	clock = ~clock;	// Toggle the value of clock after HI ticks.
		end						// Close the code block within the loop.

endmodule						// End of code for this module.