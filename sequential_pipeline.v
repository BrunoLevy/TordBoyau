/**
 * Sequential pipeline
 * Used to test what maxfreq could be
 */
 
/******************************************************************************/

module Processor (
    input  wire        clk,
    input  wire        resetn,
    output wire [31:0] IO_mem_addr, // IO memory address
    input  wire [31:0] IO_mem_rdata, // data read from IO memory
    output wire [31:0] IO_mem_wdata, // data written to IO memory
    output wire        IO_mem_wr     // IO write flag
);

/******************************************************************************/
   
   /* state machine (removed in next step that has a true pipeline) */
   
   localparam F_bit = 0; localparam F_state = 1 << F_bit;
   localparam D_bit = 1; localparam D_state = 1 << D_bit;
   localparam E_bit = 2; localparam E_state = 1 << E_bit;
   localparam M_bit = 3; localparam M_state = 1 << M_bit;
   localparam W_bit = 4; localparam W_state = 1 << W_bit;

   reg [4:0] 	  state;
   wire           halt;
   
   always @(posedge clk) begin
      if(!resetn) begin
	 state  <= F_state;
      end else if(!halt) begin
	 state <= {state[3:0],state[4]};
      end
   end

   
/******************************************************************************/

 /* 
   Reminder for the 10 RISC-V codeops
   ----------------------------------
   ALUreg  // rd <- rs1 OP rs2   
   ALUimm  // rd <- rs1 OP Iimm
   Branch  // if(rs1 OP rs2) PC<-PC+Bimm
   JALR    // rd <- PC+4; PC<-rs1+Iimm
   JAL     // rd <- PC+4; PC<-PC+Jimm
   AUIPC   // rd <- PC + Uimm
   LUI     // rd <- Uimm   
   Load    // rd <- mem[rs1+Iimm]
   Store   // mem[rs1+Simm] <- rs2
   SYSTEM  // special
 */

/******************************************************************************/
   
   reg [63:0] cycle;   
   reg [63:0] instret;

   always @(posedge clk) begin
      cycle <= !resetn ? 0 : cycle + 1;
   end
   
/******************************************************************************/

   localparam NOP = 32'b0000000_00000_00000_000_00000_0110011; 
   
                      /***  F: Instruction fetch ***/   

   reg  [31:0] 	  F_PC;

   /** These two signals come from the Execute stage **/
   wire [31:0] 	  jumpOrBranchAddress;
   wire 	  jumpOrBranch;

   reg [31:0] PROGROM[0:16383]; // 16384 4-bytes words  
                                // 64 Kb of program ROM 
   initial begin
      $readmemh("PROGROM.hex",PROGROM);
   end

   always @(posedge clk) begin
      if(!resetn) begin
	 F_PC    <= 0;
      end else if(state[F_bit]) begin
	 FD_instr <= PROGROM[F_PC[15:2]];
	 FD_PC    <= F_PC;
	 F_PC     <= F_PC+4;
      end else if(state[M_bit] & jumpOrBranch) begin
	 F_PC  <= jumpOrBranchAddress;	 
      end      
   end
   
/******************************************************************************/
   reg [31:0] FD_PC;   
   reg [31:0] FD_instr;
/******************************************************************************/

                     /*** D: Instruction decode ***/

   /** These three signals come from the Writeback stage **/
   wire        wbEnable;
   wire [31:0] wbData;
   wire [4:0]  wbRdId;

   reg [31:0] registerFile [0:31];
   
   always @(posedge clk) begin
      if(state[D_bit]) begin
	 DE_PC    <= FD_PC;
	 
	 DE_isALUreg <= (FD_instr[6:2] == 5'b01100);
	 // DE_isALUimm <= (FD_instr[6:2] == 5'b00100);
	 DE_isBranch <= (FD_instr[6:2] == 5'b11000);
	 DE_isJALR   <= (FD_instr[6:2] == 5'b11001);
	 DE_isJAL    <= (FD_instr[6:2] == 5'b11011);
	 DE_isAUIPC  <= (FD_instr[6:2] == 5'b00101);
	 DE_isLUI    <= (FD_instr[6:2] == 5'b01101);
	 DE_isLoad   <= (FD_instr[6:2] == 5'b00000);
	 DE_isStore  <= (FD_instr[6:2] == 5'b01000);
	 DE_isCSRRS   <= (FD_instr[6:2] == 5'b11100) && (FD_instr[14:12] == 3'b010);
	 DE_isEBREAK  <= (FD_instr[6:2] == 5'b11100) && (FD_instr[14:12] == 3'b000);

	 DE_Uimm <= {FD_instr[31:12],{12{1'b0}}}; 
	 DE_Iimm <= {{21{FD_instr[31]}},FD_instr[30:20]};
	 DE_Simm <= {{21{FD_instr[31]}},FD_instr[30:25],FD_instr[11:7]};
	 DE_Bimm <= {{20{FD_instr[31]}},FD_instr[7],FD_instr[30:25],FD_instr[11:8],1'b0};
	 DE_Jimm <= {{12{FD_instr[31]}},FD_instr[19:12],FD_instr[20],FD_instr[30:21],1'b0};

	 DE_shamt <= FD_instr[24:20];
	 DE_rdId   <= FD_instr[11:7];
	 DE_funct3 <= FD_instr[14:12];
	 DE_funct7 <= FD_instr[30];
	 DE_csrId  <= {FD_instr[27],FD_instr[21]};
      end
   end

   always @(posedge clk) begin
      if(wbEnable) begin
	 registerFile[wbRdId] <= wbData;
      end
   end
   
/******************************************************************************/
   reg  [31:0] DE_PC;
   wire [31:0] DE_rs1 = registerFile[FD_instr[19:15]];
   wire [31:0] DE_rs2 = registerFile[FD_instr[24:20]];

   reg DE_isALUreg; 
// reg DE_isALUimm; 
   reg DE_isBranch; 
   reg DE_isJALR;   
   reg DE_isJAL;    
   reg DE_isAUIPC;  
   reg DE_isLUI;    
   reg DE_isLoad;   
   reg DE_isStore;
   reg DE_isEBREAK;
   reg DE_isCSRRS;

   reg [31:0] DE_Uimm;
   reg [31:0] DE_Iimm;
   reg [31:0] DE_Simm;
   reg [31:0] DE_Bimm;
   reg [31:0] DE_Jimm;      

   reg [4:0]  DE_shamt;
   
   reg [4:0]  DE_rdId;
   reg [1:0]  DE_csrId;
   reg [2:0]  DE_funct3;
   reg [5:5]  DE_funct7;
   
/******************************************************************************/

                     /*** E: Execute ***/

   /*********** the ALU *************************************************/

   wire [31:0] E_aluIn1 = DE_rs1;
   
   wire [31:0] E_aluIn2 = (DE_isALUreg | DE_isBranch) ? DE_rs2 : DE_Iimm;
   wire [4:0]  E_shamt  = DE_isALUreg ? DE_rs2[4:0] : DE_shamt; 

   wire E_minus = DE_funct7[5] & DE_isALUreg;
   wire E_arith_shift = DE_funct7[5];
   
   // The adder is used by both arithmetic instructions and JALR.
   wire [31:0] E_aluPlus = E_aluIn1 + E_aluIn2;

   // Use a single 33 bits subtract to do subtraction and all comparisons
   // (trick borrowed from swapforth/J1)
   wire [32:0] E_aluMinus = {1'b1, ~E_aluIn2} + {1'b0,E_aluIn1} + 33'b1;
   wire        E_LT  = 
                 (E_aluIn1[31] ^ E_aluIn2[31]) ? E_aluIn1[31] : E_aluMinus[32];
   wire        E_LTU = E_aluMinus[32];
   wire        E_EQ  = (E_aluMinus[31:0] == 0);

   // Flip a 32 bit word. Used by the shifter (a single shifter for
   // left and right shifts, saves silicium !)
   function [31:0] flip32;
      input [31:0] x;
      flip32 = {x[ 0], x[ 1], x[ 2], x[ 3], x[ 4], x[ 5], x[ 6], x[ 7], 
		x[ 8], x[ 9], x[10], x[11], x[12], x[13], x[14], x[15], 
		x[16], x[17], x[18], x[19], x[20], x[21], x[22], x[23],
		x[24], x[25], x[26], x[27], x[28], x[29], x[30], x[31]};
   endfunction

   wire [31:0] E_shifter_in = 
                      (DE_funct3==3'b001) ? flip32(E_aluIn1) : E_aluIn1;
   
   /* verilator lint_off WIDTH */
   wire [31:0] E_shifter = 
       $signed({E_arith_shift & E_aluIn1[31], E_shifter_in}) >>> E_aluIn2[4:0];
   /* verilator lint_on WIDTH */

   wire [31:0] E_leftshift = flip32(E_shifter);

   reg [31:0] E_aluOut;
   always @(*) begin
      case(DE_funct3)
	3'b000: E_aluOut = E_minus ? E_aluMinus[31:0] : E_aluPlus;
	3'b001: E_aluOut = E_leftshift;
	3'b010: E_aluOut = {31'b0, E_LT};
	3'b011: E_aluOut = {31'b0, E_LTU};
	3'b100: E_aluOut = E_aluIn1 ^ E_aluIn2;
	3'b101: E_aluOut = E_shifter;
	3'b110: E_aluOut = E_aluIn1 | E_aluIn2;
	3'b111: E_aluOut = E_aluIn1 & E_aluIn2;
      endcase
   end
   
   /*********** Branch, JAL, JALR ***********************************/

   reg E_takeBranch;
   always @(*) begin
      case (DE_funct3)
	3'b000: E_takeBranch = E_EQ;
	3'b001: E_takeBranch = !E_EQ;
	3'b100: E_takeBranch = E_LT;
	3'b101: E_takeBranch = !E_LT;
	3'b110: E_takeBranch = E_LTU;
	3'b111: E_takeBranch = !E_LTU;
	default: E_takeBranch = 1'b0;
      endcase 
   end
   
   wire E_JumpOrBranch = (
         DE_isJAL  || 
	 DE_isJALR || 
         (DE_isBranch && E_takeBranch)
   );

   wire [31:0] E_JumpOrBranchAddress =
	DE_isBranch ? DE_PC + DE_Bimm :
	DE_isJAL    ? DE_PC + DE_Jimm :
	/* JALR */           {E_aluPlus[31:1],1'b0} ;

   wire [31:0] E_result = 
	(DE_isJAL | DE_isJALR) ? DE_PC+4                :
	 DE_isLUI              ? DE_Uimm                :
	 DE_isAUIPC            ? DE_PC + DE_Uimm        : 
        E_aluOut                                        ;
	
   /**************************************************************/
   
   always @(posedge clk) begin
      if(state[E_bit]) begin
	 EM_rs2     <= DE_rs2;
	 EM_Eresult <= E_result;
	 EM_addr    <= DE_isStore ? DE_rs1 + DE_Simm : 
                                    DE_rs1 + DE_Iimm ;
	 EM_isLoad   <= DE_isLoad;
	 EM_isStore  <= DE_isStore;
	 EM_isBranch <= DE_isBranch;
	 EM_isCSRRS  <= DE_isCSRRS;

	 EM_rdId   <= DE_rdId;
	 EM_csrId  <= DE_csrId;
	 EM_funct3 <= DE_funct3;

	 EM_JumpOrBranch        <= E_JumpOrBranch;
	 EM_JumpOrBranchAddress <= E_JumpOrBranchAddress;
      end
   end

   assign halt = resetn & DE_isEBREAK;
   
/******************************************************************************/
   reg [31:0] EM_rs2;
   reg [31:0] EM_Eresult;
   reg [31:0] EM_addr;
   reg 	      EM_isLoad;
   reg 	      EM_isStore;
   reg 	      EM_isBranch;
   reg 	      EM_isCSRRS;
   reg [4:0]  EM_rdId;
   reg [1:0]  EM_csrId;
   reg [2:0]  EM_funct3;

   reg        EM_JumpOrBranch;
   reg [31:0] EM_JumpOrBranchAddress;
/******************************************************************************/

                     /*** M: Memory ***/

   wire M_isB = (EM_funct3[1:0] == 2'b00);
   wire M_isH = (EM_funct3[1:0] == 2'b01);

   /*************** STORE **************************/

   wire [31:0] M_STORE_data;
   assign M_STORE_data[ 7: 0] = EM_rs2[7:0];
   assign M_STORE_data[15: 8] = EM_addr[0] ? EM_rs2[7:0]  : EM_rs2[15: 8] ;
   assign M_STORE_data[23:16] = EM_addr[1] ? EM_rs2[7:0]  : EM_rs2[23:16] ;
   assign M_STORE_data[31:24] = EM_addr[0] ? EM_rs2[7:0]  :
			        EM_addr[1] ? EM_rs2[15:8] : EM_rs2[31:24] ;

   // The memory write mask:
   //    1111                     if writing a word
   //    0011 or 1100             if writing a halfword
   //                                (depending on EM_addr[1])
   //    0001, 0010, 0100 or 1000 if writing a byte
   //                                (depending on EM_addr[1:0])

   wire [3:0] M_STORE_wmask = M_isB ?
	                     (EM_addr[1] ?
		                (EM_addr[0] ? 4'b1000 : 4'b0100) :
		                (EM_addr[0] ? 4'b0010 : 4'b0001)
                             ) :
	                     M_isH ? (EM_addr[1] ? 4'b1100 : 4'b0011) :
                                      4'b1111 ;


   wire  M_isIO         = EM_addr[22];
   wire  M_isRAM        = !M_isIO;

   assign IO_mem_addr  = EM_addr;
   assign IO_mem_wr    = state[M_bit] & EM_isStore & M_isIO; 
   assign IO_mem_wdata = EM_rs2;

   wire [3:0] M_wmask = 
	      {4{EM_isStore & M_isRAM & state[M_bit]}} & M_STORE_wmask;
   
   reg [31:0] DATARAM [0:16383]; // 16384 4-bytes words 
                                 // 64 Kb of data RAM in total
   wire [13:0] M_word_addr = EM_addr[15:2];
   
   always @(posedge clk) begin
      MW_Mdata <= DATARAM[M_word_addr];
      if(M_wmask[0]) DATARAM[M_word_addr][ 7:0 ] <= M_STORE_data[ 7:0 ];
      if(M_wmask[1]) DATARAM[M_word_addr][15:8 ] <= M_STORE_data[15:8 ];
      if(M_wmask[2]) DATARAM[M_word_addr][23:16] <= M_STORE_data[23:16];
      if(M_wmask[3]) DATARAM[M_word_addr][31:24] <= M_STORE_data[31:24]; 
   end
   
   initial begin
      $readmemh("DATARAM.hex",DATARAM);
   end
   
   always @(posedge clk) begin
      if(state[M_bit]) begin
	 MW_Eresult   <= EM_Eresult;
	 MW_IOresult  <= IO_mem_rdata;
	 MW_addr      <= EM_addr;
	 MW_isLoad    <= EM_isLoad;
	 MW_isStore   <= EM_isStore;
	 MW_isBranch  <= EM_isBranch;
	 MW_isCSRRS   <= EM_isCSRRS;
	 MW_funct3    <= EM_funct3;
	 MW_rdId      <= EM_rdId;
	 case(EM_csrId) 
	   2'b00: MW_CSRresult = cycle[31:0];
	   2'b10: MW_CSRresult = cycle[63:32];
	   2'b01: MW_CSRresult = instret[31:0];
	   2'b11: MW_CSRresult = instret[63:32];	 
	 endcase 
	 if(!resetn) begin
	    instret <= 0;
	 end else begin
	    instret <= instret + 1;
	 end
      end
   end

/******************************************************************************/
   reg [31:0] MW_Eresult;
   reg [31:0] MW_addr;
   reg [31:0] MW_Mdata;
   reg [31:0] MW_IOresult;
   reg [31:0] MW_CSRresult;
   reg 	      MW_isLoad;
   reg 	      MW_isStore;
   reg 	      MW_isBranch;
   reg 	      MW_isCSRRS;
   reg [2:0]  MW_funct3;
   reg [4:0]  MW_rdId;
/******************************************************************************/

                     /*** W: WriteBack ***/
		     
   wire W_isB = (MW_funct3[1:0] == 2'b00);
   wire W_isH = (MW_funct3[1:0] == 2'b01);
   wire W_sext = !MW_funct3[2];		     
   wire W_isIO = MW_addr[22];

   /*************** LOAD ****************************/
   
   wire [15:0] W_LOAD_H=MW_addr[1] ? MW_Mdata[31:16]: MW_Mdata[15:0];
   wire  [7:0] W_LOAD_B=MW_addr[0] ? W_LOAD_H[15:8] : W_LOAD_H[7:0];
   wire        W_LOAD_sign=W_sext & (W_isB ? W_LOAD_B[7] : W_LOAD_H[15]);

   wire [31:0] W_Mresult = W_isB ? {{24{W_LOAD_sign}},W_LOAD_B} :
	                   W_isH ? {{16{W_LOAD_sign}},W_LOAD_H} :
                                                      MW_Mdata ;
   
   assign wbData = 
	       MW_isLoad  ? (W_isIO ? MW_IOresult : W_Mresult) :
	       MW_isCSRRS ? MW_CSRresult :
	       MW_Eresult;

   assign wbEnable =
        !MW_isBranch && !MW_isStore && (MW_rdId != 0);

   assign wbRdId = MW_rdId;
   
/******************************************************************************/
   assign jumpOrBranchAddress = EM_JumpOrBranchAddress;
   assign jumpOrBranch        = EM_JumpOrBranch;
/******************************************************************************/

`ifdef BENCH   
   always @(posedge clk) begin
      if(halt) $finish();
   end
`endif   

/******************************************************************************/
   
endmodule

 
