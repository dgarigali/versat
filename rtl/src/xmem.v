`timescale 1ns / 1ps
`include "xversat.vh"
`include "xmemdefs.vh"

module xmem #(
              parameter MEM_INIT_FILE="none",
	      parameter DATA_W = 32
              )
    (
    //control
    input                         clk,
    input                         rst,
    input                         run,
    output                        doneA,
    output                        doneB,

    //mem interface
    input                         we,
    input [`MEM_ADDR_W-1:0]       addr,
    input [DATA_W-1:0]            rdata,
    input                         valid,

    //input / output data
    input [2*`DATABUS_W-1:0]      flow_in,
    output [2*DATA_W-1:0]         flow_out,

    //configurations
    input [2*`MEMP_CONF_BITS-1:0] config_bits
    );

   //output databus
   wire [DATA_W-1:0]              outA;
   wire [DATA_W-1:0]              outB;
   assign flow_out = {outA, outB};

   //function to reverse bits
   function [`MEM_ADDR_W-1:0] reverseBits;
      input [`MEM_ADDR_W-1:0]     word;
      integer                     i;

      begin
	 for (i=0; i < `MEM_ADDR_W; i=i+1)
	   reverseBits[i]=word[`MEM_ADDR_W-1 - i];
      end
   endfunction

   //unpack configuration bits 
   wire [`MEM_ADDR_W-1:0] iterationsA = config_bits[2*`MEMP_CONF_BITS-1 -: `MEM_ADDR_W];                                           
   wire [`PERIOD_W-1:0]   perA        = config_bits[2*`MEMP_CONF_BITS-`MEM_ADDR_W-1 -: `PERIOD_W];                                 
   wire [`PERIOD_W-1:0]   dutyA       = config_bits[2*`MEMP_CONF_BITS-`MEM_ADDR_W-`PERIOD_W-1 -: `PERIOD_W];                       
   wire [`N_W-1:0]        selA        = config_bits[2*`MEMP_CONF_BITS-`MEM_ADDR_W-2*`PERIOD_W-1 -: `N_W];                          
   wire [`MEM_ADDR_W-1:0] startA      = config_bits[2*`MEMP_CONF_BITS-`MEM_ADDR_W-2*`PERIOD_W-`N_W-1 -: `MEM_ADDR_W];              
   wire [`MEM_ADDR_W-1:0] shiftA      = config_bits[2*`MEMP_CONF_BITS-`MEM_ADDR_W-2*`PERIOD_W-`N_W-`MEM_ADDR_W-1 -: `MEM_ADDR_W];  
   wire [`MEM_ADDR_W-1:0] incrA       = config_bits[2*`MEMP_CONF_BITS-`MEM_ADDR_W-2*`PERIOD_W-`N_W-2*`MEM_ADDR_W-1 -: `MEM_ADDR_W];
   wire [`PERIOD_W-1:0]   delayA      = config_bits[2*`MEMP_CONF_BITS-`MEM_ADDR_W-2*`PERIOD_W-`N_W-3*`MEM_ADDR_W-1 -: `PERIOD_W];  
   wire                   reverseA    = config_bits[2*`MEMP_CONF_BITS-`MEM_ADDR_W-2*`PERIOD_W-`N_W-3*`MEM_ADDR_W-`PERIOD_W-1 -: 1];
   wire                   extA        = config_bits[2*`MEMP_CONF_BITS-`MEM_ADDR_W-2*`PERIOD_W-`N_W-3*`MEM_ADDR_W-`PERIOD_W-1-1 -: 1];
   wire                   inA_wr      = config_bits[2*`MEMP_CONF_BITS-`MEM_ADDR_W-2*`PERIOD_W-`N_W-3*`MEM_ADDR_W-`PERIOD_W-1-1-1 -: 1];

   wire [`MEM_ADDR_W-1:0] iterationsB = config_bits[`MEMP_CONF_BITS-1 -: `MEM_ADDR_W];                                             
   wire [`PERIOD_W-1:0]   perB        = config_bits[`MEMP_CONF_BITS-`MEM_ADDR_W-1 -: `PERIOD_W];                                   
   wire [`PERIOD_W-1:0]   dutyB       = config_bits[`MEMP_CONF_BITS-`MEM_ADDR_W-`PERIOD_W-1 -: `PERIOD_W];                         
   wire [`N_W-1:0]        selB        = config_bits[`MEMP_CONF_BITS-`MEM_ADDR_W-2*`PERIOD_W-1 -: `N_W];                            
   wire [`MEM_ADDR_W-1:0] startB      = config_bits[`MEMP_CONF_BITS-`MEM_ADDR_W-2*`PERIOD_W-`N_W-1 -: `MEM_ADDR_W];                
   wire [`MEM_ADDR_W-1:0] shiftB      = config_bits[`MEMP_CONF_BITS-`MEM_ADDR_W-2*`PERIOD_W-`N_W-`MEM_ADDR_W-1 -: `MEM_ADDR_W];    
   wire [`MEM_ADDR_W-1:0] incrB       = config_bits[`MEMP_CONF_BITS-`MEM_ADDR_W-2*`PERIOD_W-`N_W-2*`MEM_ADDR_W-1 -: `MEM_ADDR_W];  
   wire [`PERIOD_W-1:0]   delayB      = config_bits[`MEMP_CONF_BITS-`MEM_ADDR_W-2*`PERIOD_W-`N_W-3*`MEM_ADDR_W-1 -: `PERIOD_W];    
   wire                   reverseB    = config_bits[`MEMP_CONF_BITS-`MEM_ADDR_W-2*`PERIOD_W-`N_W-3*`MEM_ADDR_W-`PERIOD_W-1 -: 1];  
   wire                   extB        = config_bits[`MEMP_CONF_BITS-`MEM_ADDR_W-2*`PERIOD_W-`N_W-3*`MEM_ADDR_W-`PERIOD_W-1-1 -: 1];
   wire                   inB_wr      = config_bits[`MEMP_CONF_BITS-`MEM_ADDR_W-2*`PERIOD_W-`N_W-3*`MEM_ADDR_W-`PERIOD_W-1-1-1 -: 1];

   // ports program ready
   wire readyA = |iterationsA;
   wire readyB = |iterationsB;

   //mem enables output by addr gen
   wire enA_int;
   wire enA = enA_int | valid;
   wire enB;

   //write enables
   wire wrA = valid? we : (enA_int & inA_wr & ~extA); //addrgen on & input on & input isn't address
   wire wrB = (enB & inB_wr & ~extB);

   //port addresses and enables
   wire [`MEM_ADDR_W-1:0] addrA, addrA_int, addrA_int2;
   wire [`MEM_ADDR_W-1:0] addrB, addrB_int, addrB_int2;

   //data inputs
   wire [DATA_W-1:0]     inA;
   wire [DATA_W-1:0]     inB;

   //input selection
   xinmux # ( 
	.DATA_W(DATA_W)
   ) muxa (
	.sel(selA),
	.data_in(flow_in),
	.data_out(inA)
	);

   xinmux # ( 
	.DATA_W(DATA_W)
   ) muxb (
	.sel(selB),
	.data_in(flow_in),
	.data_out(inB)
	);

   wire [DATA_W-1:0]     data_to_wrA = valid? rdata : inA ;

   //address generators
   xaddrgen addrgenA (
		      .clk(clk),
		      .rst(rst),
		      .init(run),
		      .run(run & readyA),
		      .pause(1'b0),
		      .iterations(iterationsA),
		      .period(perA),
		      .duty(dutyA),
		      .start(startA),
		      .shift(shiftA),
		      .incr(incrA),
		      .delay(delayA),
		      .addr(addrA_int),
		      .mem_en(enA_int),
		      .done(doneA)
		      );

   xaddrgen addrgenB (
		      .clk(clk),
		      .rst(rst),
		      .init(run),
		      .run(run & readyB),
		      .pause(1'b0),
		      .iterations(iterationsB),
		      .period(perB),
		      .duty(dutyB),
		      .start(startB),
		      .shift(shiftB),
		      .incr(incrB),
		      .delay(delayB),
		      .addr(addrB_int),
		      .mem_en(enB),
		      .done(doneB)
		      );

   //define addresses based on ext and rvrs
   assign addrA = valid? addr[`MEM_ADDR_W-1:0] : extB? inB[`MEM_ADDR_W-1:0] : addrA_int2[`MEM_ADDR_W-1:0];
   assign addrB = extA? inA[`MEM_ADDR_W-1:0] : addrB_int2[`MEM_ADDR_W-1:0];
   assign addrA_int2 = reverseA? reverseBits(addrA_int) : addrA_int;
   assign addrB_int2 = reverseB? reverseBits(addrB_int) : addrB_int;

   iob_t2p_mem #(
                   .MEM_INIT_FILE(MEM_INIT_FILE),
		   .DATA_W(DATA_W),
		   .ADDR_W(`MEM_ADDR_W))
   mem
     (
      .data_a(data_to_wrA),
      .data_b(inB),
      .addr_a(addrA[`MEM_ADDR_W-1:0]),
      .addr_b(addrB[`MEM_ADDR_W-1:0]),
      .en_a(enA),
      .en_b(enB),
      .we_a(wrA),
      .we_b(wrB),
      .q_a(outA),
      .q_b(outB),
      .clk(clk)
      );

endmodule
