/*
	File: assignment2.v
	Author: Samuel Britton
	Modules:
		Assignment1  	-  The "main" testbench that runs and tests other modules
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
module Assignment2;
	wire [31:0] PCtoAdd; 
	wire [31:0] addToPC;
	Add i1(addToPC,PCtoAdd,32'd1);			//  the structure here is  Add -> PC -> Add -> ....
	
	wire clockWire;
	Clock c1(clockWire);				// Clock has no inputs, only outputs
	
	PC p1(PCtoAdd,addToPC,clockWire);

	wire [31:0] IMOut;	
	IM im1(IMOut,PCtoAdd);				// In the current state, the IM's output is unused
	

	reg [31:0]	aluA;
	reg [31:0]	aluB;
	reg [3:0] Ctl;
	wire[31:0]	aluOut;
	reg zero;
	MIPSALU alu(aluOut, zero, Ctl, aluA, aluB);

	wire [31:0] seOut;
	reg [15:0] seIn;
	SignExtender se(seOut,seIn);

	//TODO add these input wires
	reg[31:0] address;
	reg memRead;
	reg dmMemWrite;
	wire[31:0] readData;
	reg[31:0] dmWriteData;
	DataMemory dm(readData,clockWire,address,dmWriteData,dmMemWrite,memRead);

	//TODO add these input wires
	reg [5:0] read1,read2,writeReg;
	wire [31:0] data1,data2;
	reg[31:0] writeData;
	reg regWrite;
//Data1,Data2,WriteReg,WriteData,RegWrite,Read1,Read2,clock);

	RegisterFile regi(data1,data2,writeReg,writeData,regWrite,read1,read2,clockWire);
	initial								
		begin	

			/*
				//IM test
			//$readmemb( "data.dat", im1.my_memory); //fills my_memory of the instruction memory
			*/

			
				//Sign extender test
			$monitor( $time," seIn=%b seOut= %b",seIn,seOut);
			#1 seIn = 16'd1;
			#1 seIn = 16'd2;
			#1 seIn = 16'b1000111111111111;
			#1 seIn = 16'd2;
			
			
			/*
				//ALU test
			$monitor($time, "   aluA:%h operation:%d aluB:%h = aluOut:%h",aluA,Ctl,aluB,aluOut);
			#1 zero = 0;
			#1 Ctl = 2; 
			#1 aluA = 16;
			#1 aluB = 15;
			#1 Ctl = 6;
			#1 Ctl = 0;
			*/

			
				
			/*
				//Data memory test
			$monitor( $time," data read=%d  data to Write:%d read?=%b  write?=%b ",readData,dmWriteData, memRead,dmMemWrite);
			$readmemb("memory",dm.mem);
			#5 memRead = 0;
			#5 dmMemWrite = 0;
			#5 address = 32'd1;
			#5 memRead = 1;
			#5 memRead = 0;
			#5 dmWriteData = 32'd545454;
			#5 dmMemWrite = 1;
			#5 dmMemWrite = 0;
			#5 memRead = 1;
			#5 memRead = 0;
			*/

			/*
				//Register test
			$monitor( $time,"Data to save=%d data1=%d  data2=%d",writeData, data1,data2);
			#5 writeData = 32'd123;
			#5 writeReg = 6'd2;
			#5 regWrite = 1;
			#5 regWrite = 0;
			#5 read1 = 6'd2;
			#5 writeData = 32'd54321;
			#5 writeReg = 6'd12;
			#5 regWrite = 1;
			#5 regWrite = 0;
			#5 read2 = 6'd12;
			*/




			

			#400 $finish;	//After 40 clock ticks it is terminated as to not let it run forever
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

  always
    begin
      // write the register with new value if Regwrite is high
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
				6: ALUOut <= A - B;
				7: ALUOut <= A < B ? 1 : 0;
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
module IM(word, addr);
	input [31:0] addr;
	output [31:0] word;
	reg [31:0] my_memory [0:255]; //we put the contents from data in this. This is 8 bit words of length 256

	assign	word[31:0] = my_memory[addr]; //outputs the word for every new address
	
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