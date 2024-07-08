//
// PCILeech FPGA.
//
// PCIe BAR PIO controller.
//
// The PCILeech BAR PIO controller allows for easy user-implementation on top
// of the PCILeech AXIS128 PCIe TLP streaming interface.
// The controller consists of a read engine and a write engine and pluggable
// user-implemented PCIe BAR implementations (found at bottom of the file).
//
// Considerations:
// - The core handles 1 DWORD read + 1 DWORD write per CLK max. If a lot of
//   data is written / read from the TLP streaming interface the core may
//   drop packet silently.
// - The core reads 1 DWORD of data (without byte enable) per CLK.
// - The core writes 1 DWORD of data (with byte enable) per CLK.
// - All user-implemented cores must have the same latency in CLKs for the
//   returned read data or else undefined behavior will take place.
// - 32-bit addresses are passed for read/writes. Larger BARs than 4GB are
//   not supported due to addressing constraints. Lower bits (LSBs) are the
//   BAR offset, Higher bits (MSBs) are the 32-bit base address of the BAR.
// - DO NOT edit read/write engines.
// - DO edit pcileech_tlps128_bar_controller (to swap bar implementations).
// - DO edit the bar implementations (at bottom of the file, if neccessary).
//
// Example implementations exists below, swap out any of the example cores
// against a core of your use case, or modify existing cores.
// Following test cores exist (see below in this file):
// - pcileech_bar_impl_zerowrite4k = zero-initialized read/write BAR.
//     It's possible to modify contents by use of .coe file.
// - pcileech_bar_impl_loopaddr = test core that loops back the 32-bit
//     address of the current read. Does not support writes.
// - pcileech_bar_impl_none = core without any reply.
// 
// (c) Ulf Frisk, 2024
// Author: Ulf Frisk, pcileech@frizk.net
//

`timescale 1ns / 1ps
`include "pcileech_header.svh"

module pcileech_tlps128_bar_controller(
    input                   rst,
    input                   clk,
    input                   bar_en,
    input [15:0]            pcie_id,
    input [31:0]            base_address_register,
    IfAXIS128.sink_lite     tlps_in,
    IfAXIS128.source        tlps_out
);
    
    // ------------------------------------------------------------------------
    // 1: TLP RECEIVE:
    // Receive incoming BAR requests from the TLP stream:
    // send them onwards to read and write FIFOs
    // ------------------------------------------------------------------------
    wire in_is_wr_ready;
    bit  in_is_wr_last;
    wire in_is_first    = tlps_in.tuser[0];
    wire in_is_bar      = bar_en && (tlps_in.tuser[8:2] != 0);
    wire in_is_rd       = (in_is_first && tlps_in.tlast && ((tlps_in.tdata[31:25] == 7'b0000000) || (tlps_in.tdata[31:25] == 7'b0010000) || (tlps_in.tdata[31:24] == 8'b00000010)));
    wire in_is_wr       = in_is_wr_last || (in_is_first && in_is_wr_ready && ((tlps_in.tdata[31:25] == 7'b0100000) || (tlps_in.tdata[31:25] == 7'b0110000) || (tlps_in.tdata[31:24] == 8'b01000010)));
    
    always @ ( posedge clk )
        if ( rst ) begin
            in_is_wr_last <= 0;
        end
        else if ( tlps_in.tvalid ) begin
            in_is_wr_last <= !tlps_in.tlast && in_is_wr;
        end
    
    wire [6:0]  wr_bar;
    wire [31:0] wr_addr;
    wire [3:0]  wr_be;
    wire [31:0] wr_data;
    wire        wr_valid;
    wire [87:0] rd_req_ctx;
    wire [6:0]  rd_req_bar;
    wire [31:0] rd_req_addr;
    wire        rd_req_valid;
    wire [87:0] rd_rsp_ctx;
    wire [31:0] rd_rsp_data;
    wire        rd_rsp_valid;
        
    pcileech_tlps128_bar_rdengine i_pcileech_tlps128_bar_rdengine(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        // TLPs:
        .pcie_id        ( pcie_id                       ),
        .tlps_in        ( tlps_in                       ),
        .tlps_in_valid  ( tlps_in.tvalid && in_is_bar && in_is_rd ),
        .tlps_out       ( tlps_out                      ),
        // BAR reads:
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_bar     ( rd_req_bar                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid                  ),
        .rd_rsp_ctx     ( rd_rsp_ctx                    ),
        .rd_rsp_data    ( rd_rsp_data                   ),
        .rd_rsp_valid   ( rd_rsp_valid                  )
    );

    pcileech_tlps128_bar_wrengine i_pcileech_tlps128_bar_wrengine(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        // TLPs:
        .tlps_in        ( tlps_in                       ),
        .tlps_in_valid  ( tlps_in.tvalid && in_is_bar && in_is_wr ),
        .tlps_in_ready  ( in_is_wr_ready                ),
        // outgoing BAR writes:
        .wr_bar         ( wr_bar                        ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid                      )
    );
    
    wire [87:0] bar_rsp_ctx[7];
    wire [31:0] bar_rsp_data[7];
    wire        bar_rsp_valid[7];
    
    assign rd_rsp_ctx = bar_rsp_valid[0] ? bar_rsp_ctx[0] :
                        bar_rsp_valid[1] ? bar_rsp_ctx[1] :
                        bar_rsp_valid[2] ? bar_rsp_ctx[2] :
                        bar_rsp_valid[3] ? bar_rsp_ctx[3] :
                        bar_rsp_valid[4] ? bar_rsp_ctx[4] :
                        bar_rsp_valid[5] ? bar_rsp_ctx[5] :
                        bar_rsp_valid[6] ? bar_rsp_ctx[6] : 0;
    assign rd_rsp_data = bar_rsp_valid[0] ? bar_rsp_data[0] :
                        bar_rsp_valid[1] ? bar_rsp_data[1] :
                        bar_rsp_valid[2] ? bar_rsp_data[2] :
                        bar_rsp_valid[3] ? bar_rsp_data[3] :
                        bar_rsp_valid[4] ? bar_rsp_data[4] :
                        bar_rsp_valid[5] ? bar_rsp_data[5] :
                        bar_rsp_valid[6] ? bar_rsp_data[6] : 0;
    assign rd_rsp_valid = bar_rsp_valid[0] || bar_rsp_valid[1] || bar_rsp_valid[2] || bar_rsp_valid[3] || bar_rsp_valid[4] || bar_rsp_valid[5] || bar_rsp_valid[6];
    
    pcileech_bar_impl_ar9287_wifi i_bar0(
        .rst                   ( rst                           ),
        .clk                   ( clk                           ),
        .wr_addr               ( wr_addr                       ),
        .wr_be                 ( wr_be                         ),
        .wr_data               ( wr_data                       ),
        .wr_valid              ( wr_valid && wr_bar[0]         ),
        .rd_req_ctx            ( rd_req_ctx                    ),
        .rd_req_addr           ( rd_req_addr                   ),
        .rd_req_valid          ( rd_req_valid && rd_req_bar[0] ),
        .base_address_register ( base_address_register         ),
        .rd_rsp_ctx            ( bar_rsp_ctx[0]                ),
        .rd_rsp_data           ( bar_rsp_data[0]               ),
        .rd_rsp_valid          ( bar_rsp_valid[0]              )
    );
    
    pcileech_bar_impl_loopaddr i_bar1(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[1]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[1] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[1]                ),
        .rd_rsp_data    ( bar_rsp_data[1]               ),
        .rd_rsp_valid   ( bar_rsp_valid[1]              )
    );
    
    pcileech_bar_impl_none i_bar2(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[2]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[2] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[2]                ),
        .rd_rsp_data    ( bar_rsp_data[2]               ),
        .rd_rsp_valid   ( bar_rsp_valid[2]              )
    );
    
    pcileech_bar_impl_none i_bar3(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[3]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[3] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[3]                ),
        .rd_rsp_data    ( bar_rsp_data[3]               ),
        .rd_rsp_valid   ( bar_rsp_valid[3]              )
    );
    
    pcileech_bar_impl_none i_bar4(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[4]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[4] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[4]                ),
        .rd_rsp_data    ( bar_rsp_data[4]               ),
        .rd_rsp_valid   ( bar_rsp_valid[4]              )
    );
    
    pcileech_bar_impl_none i_bar5(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[5]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[5] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[5]                ),
        .rd_rsp_data    ( bar_rsp_data[5]               ),
        .rd_rsp_valid   ( bar_rsp_valid[5]              )
    );
    
    pcileech_bar_impl_none i_bar6_optrom(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[6]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[6] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[6]                ),
        .rd_rsp_data    ( bar_rsp_data[6]               ),
        .rd_rsp_valid   ( bar_rsp_valid[6]              )
    );


endmodule



// ------------------------------------------------------------------------
// BAR WRITE ENGINE:
// Receives BAR WRITE TLPs and output BAR WRITE requests.
// Holds a 2048-byte buffer.
// Input flow rate is 16bytes/CLK (max).
// Output flow rate is 4bytes/CLK.
// If write engine overflows incoming TLP is completely discarded silently.
// ------------------------------------------------------------------------
module pcileech_tlps128_bar_wrengine(
    input                   rst,    
    input                   clk,
    // TLPs:
    IfAXIS128.sink_lite     tlps_in,
    input                   tlps_in_valid,
    output                  tlps_in_ready,
    // outgoing BAR writes:
    output bit [6:0]        wr_bar,
    output bit [31:0]       wr_addr,
    output bit [3:0]        wr_be,
    output bit [31:0]       wr_data,
    output bit              wr_valid
);

    wire            f_rd_en;
    wire [127:0]    f_tdata;
    wire [3:0]      f_tkeepdw;
    wire [8:0]      f_tuser;
    wire            f_tvalid;
    
    bit [127:0]     tdata;
    bit [3:0]       tkeepdw;
    bit             tlast;
    
    bit [3:0]       be_first;
    bit [3:0]       be_last;
    bit             first_dw;
    bit [31:0]      addr;

    fifo_141_141_clk1_bar_wr i_fifo_141_141_clk1_bar_wr(
        .srst           ( rst                           ),
        .clk            ( clk                           ),
        .wr_en          ( tlps_in_valid                 ),
        .din            ( {tlps_in.tuser[8:0], tlps_in.tkeepdw, tlps_in.tdata} ),
        .full           (                               ),
        .prog_empty     ( tlps_in_ready                 ),
        .rd_en          ( f_rd_en                       ),
        .dout           ( {f_tuser, f_tkeepdw, f_tdata} ),    
        .empty          (                               ),
        .valid          ( f_tvalid                      )
    );
    
    // STATE MACHINE:
    `define S_ENGINE_IDLE        3'h0
    `define S_ENGINE_FIRST       3'h1
    `define S_ENGINE_4DW_REQDATA 3'h2
    `define S_ENGINE_TX0         3'h4
    `define S_ENGINE_TX1         3'h5
    `define S_ENGINE_TX2         3'h6
    `define S_ENGINE_TX3         3'h7
    (* KEEP = "TRUE" *) bit [3:0] state = `S_ENGINE_IDLE;
    
    assign f_rd_en = (state == `S_ENGINE_IDLE) ||
                     (state == `S_ENGINE_4DW_REQDATA) ||
                     (state == `S_ENGINE_TX3) ||
                     ((state == `S_ENGINE_TX2 && !tkeepdw[3])) ||
                     ((state == `S_ENGINE_TX1 && !tkeepdw[2])) ||
                     ((state == `S_ENGINE_TX0 && !f_tkeepdw[1]));

    always @ ( posedge clk ) begin
        wr_addr     <= addr;
        wr_valid    <= ((state == `S_ENGINE_TX0) && f_tvalid) || (state == `S_ENGINE_TX1) || (state == `S_ENGINE_TX2) || (state == `S_ENGINE_TX3);
        
    end

    always @ ( posedge clk )
        if ( rst ) begin
            state <= `S_ENGINE_IDLE;
        end
        else case ( state )
            `S_ENGINE_IDLE: begin
                state   <= `S_ENGINE_FIRST;
            end
            `S_ENGINE_FIRST: begin
                if ( f_tvalid && f_tuser[0] ) begin
                    wr_bar      <= f_tuser[8:2];
                    tdata       <= f_tdata;
                    tkeepdw     <= f_tkeepdw;
                    tlast       <= f_tuser[1];
                    first_dw    <= 1;
                    be_first    <= f_tdata[35:32];
                    be_last     <= f_tdata[39:36];
                    if ( f_tdata[31:29] == 8'b010 ) begin       // 3 DW header, with data
                        addr    <= { f_tdata[95:66], 2'b00 };
                        state   <= `S_ENGINE_TX3;
                    end
                    else if ( f_tdata[31:29] == 8'b011 ) begin  // 4 DW header, with data
                        addr    <= { f_tdata[127:98], 2'b00 };
                        state   <= `S_ENGINE_4DW_REQDATA;
                    end 
                end
                else begin
                    state   <= `S_ENGINE_IDLE;
                end
            end 
            `S_ENGINE_4DW_REQDATA: begin
                state   <= `S_ENGINE_TX0;
            end
            `S_ENGINE_TX0: begin
                tdata       <= f_tdata;
                tkeepdw     <= f_tkeepdw;
                tlast       <= f_tuser[1];
                addr        <= addr + 4;
                wr_data     <= { f_tdata[0+00+:8], f_tdata[0+08+:8], f_tdata[0+16+:8], f_tdata[0+24+:8] };
                first_dw    <= 0;
                wr_be       <= first_dw ? be_first : (f_tkeepdw[1] ? 4'hf : be_last);
                state       <= f_tvalid ? (f_tkeepdw[1] ? `S_ENGINE_TX1 : `S_ENGINE_FIRST) : `S_ENGINE_IDLE;
            end
            `S_ENGINE_TX1: begin
                addr        <= addr + 4;
                wr_data     <= { tdata[32+00+:8], tdata[32+08+:8], tdata[32+16+:8], tdata[32+24+:8] };
                first_dw    <= 0;
                wr_be       <= first_dw ? be_first : (tkeepdw[2] ? 4'hf : be_last);
                state       <= tkeepdw[2] ? `S_ENGINE_TX2 : `S_ENGINE_FIRST;
            end
            `S_ENGINE_TX2: begin
                addr        <= addr + 4;
                wr_data     <= { tdata[64+00+:8], tdata[64+08+:8], tdata[64+16+:8], tdata[64+24+:8] };
                first_dw    <= 0;
                wr_be       <= first_dw ? be_first : (tkeepdw[3] ? 4'hf : be_last);
                state       <= tkeepdw[3] ? `S_ENGINE_TX3 : `S_ENGINE_FIRST;
            end
            `S_ENGINE_TX3: begin
                addr        <= addr + 4;
                wr_data     <= { tdata[96+00+:8], tdata[96+08+:8], tdata[96+16+:8], tdata[96+24+:8] };
                first_dw    <= 0;
                wr_be       <= first_dw ? be_first : (!tlast ? 4'hf : be_last);
                state       <= !tlast ? `S_ENGINE_TX0 : `S_ENGINE_FIRST;
            end
        endcase

endmodule



// ------------------------------------------------------------------------
// BAR READ ENGINE:
// Receives BAR READ TLPs and output BAR READ requests.
// ------------------------------------------------------------------------
module pcileech_tlps128_bar_rdengine(
    input                   rst,    
    input                   clk,
    // TLPs:
    input [15:0]            pcie_id,
    IfAXIS128.sink_lite     tlps_in,
    input                   tlps_in_valid,
    IfAXIS128.source        tlps_out,
    // BAR reads:
    output [87:0]           rd_req_ctx,
    output [6:0]            rd_req_bar,
    output [31:0]           rd_req_addr,
    output                  rd_req_valid,
    input  [87:0]           rd_rsp_ctx,
    input  [31:0]           rd_rsp_data,
    input                   rd_rsp_valid
);

    // ------------------------------------------------------------------------
    // 1: PROCESS AND QUEUE INCOMING READ TLPs:
    // ------------------------------------------------------------------------
    wire [10:0] rd1_in_dwlen    = (tlps_in.tdata[9:0] == 0) ? 11'd1024 : {1'b0, tlps_in.tdata[9:0]};
    wire [6:0]  rd1_in_bar      = tlps_in.tuser[8:2];
    wire [15:0] rd1_in_reqid    = tlps_in.tdata[63:48];
    wire [7:0]  rd1_in_tag      = tlps_in.tdata[47:40];
    wire [31:0] rd1_in_addr     = { ((tlps_in.tdata[31:29] == 3'b000) ? tlps_in.tdata[95:66] : tlps_in.tdata[127:98]), 2'b00 };
    wire [73:0] rd1_in_data;
    assign rd1_in_data[73:63]   = rd1_in_dwlen;
    assign rd1_in_data[62:56]   = rd1_in_bar;   
    assign rd1_in_data[55:48]   = rd1_in_tag;
    assign rd1_in_data[47:32]   = rd1_in_reqid;
    assign rd1_in_data[31:0]    = rd1_in_addr;
    
    wire        rd1_out_rden;
    wire [73:0] rd1_out_data;
    wire        rd1_out_valid;
    
    fifo_74_74_clk1_bar_rd1 i_fifo_74_74_clk1_bar_rd1(
        .srst           ( rst                           ),
        .clk            ( clk                           ),
        .wr_en          ( tlps_in_valid                 ),
        .din            ( rd1_in_data                   ),
        .full           (                               ),
        .rd_en          ( rd1_out_rden                  ),
        .dout           ( rd1_out_data                  ),    
        .empty          (                               ),
        .valid          ( rd1_out_valid                 )
    );
    
    // ------------------------------------------------------------------------
    // 2: PROCESS AND SPLIT READ TLPs INTO RESPONSE TLP READ REQUESTS AND QUEUE:
    //    (READ REQUESTS LARGER THAN 128-BYTES WILL BE SPLIT INTO MULTIPLE).
    // ------------------------------------------------------------------------
    
    wire [10:0] rd1_out_dwlen       = rd1_out_data[73:63];
    wire [4:0]  rd1_out_dwlen5      = rd1_out_data[67:63];
    wire [4:0]  rd1_out_addr5       = rd1_out_data[6:2];
    
    // 1st "instant" packet:
    wire [4:0]  rd2_pkt1_dwlen_pre  = ((rd1_out_addr5 + rd1_out_dwlen5 > 6'h20) || ((rd1_out_addr5 != 0) && (rd1_out_dwlen5 == 0))) ? (6'h20 - rd1_out_addr5) : rd1_out_dwlen5;
    wire [5:0]  rd2_pkt1_dwlen      = (rd2_pkt1_dwlen_pre == 0) ? 6'h20 : rd2_pkt1_dwlen_pre;
    wire [10:0] rd2_pkt1_dwlen_next = rd1_out_dwlen - rd2_pkt1_dwlen;
    wire        rd2_pkt1_large      = (rd1_out_dwlen > 32) || (rd1_out_dwlen != rd2_pkt1_dwlen);
    wire        rd2_pkt1_tiny       = (rd1_out_dwlen == 1);
    wire [11:0] rd2_pkt1_bc         = rd1_out_dwlen << 2;
    wire [85:0] rd2_pkt1;
    assign      rd2_pkt1[85:74]     = rd2_pkt1_bc;
    assign      rd2_pkt1[73:63]     = rd2_pkt1_dwlen;
    assign      rd2_pkt1[62:0]      = rd1_out_data[62:0];
    
    // Nth packet (if split should take place):
    bit  [10:0] rd2_total_dwlen;
    wire [10:0] rd2_total_dwlen_next = rd2_total_dwlen - 11'h20;
    
    bit  [85:0] rd2_pkt2;
    wire [10:0] rd2_pkt2_dwlen = rd2_pkt2[73:63];
    wire        rd2_pkt2_large = (rd2_total_dwlen > 11'h20);
    
    wire        rd2_out_rden;
    
    // STATE MACHINE:
    `define S2_ENGINE_REQDATA     1'h0
    `define S2_ENGINE_PROCESSING  1'h1
    (* KEEP = "TRUE" *) bit [0:0] state2 = `S2_ENGINE_REQDATA;
    
    always @ ( posedge clk )
        if ( rst ) begin
            state2 <= `S2_ENGINE_REQDATA;
        end
        else case ( state2 )
            `S2_ENGINE_REQDATA: begin
                if ( rd1_out_valid && rd2_pkt1_large ) begin
                    rd2_total_dwlen <= rd2_pkt1_dwlen_next;                             // dwlen (total remaining)
                    rd2_pkt2[85:74] <= rd2_pkt1_dwlen_next << 2;                        // byte-count
                    rd2_pkt2[73:63] <= (rd2_pkt1_dwlen_next > 11'h20) ? 11'h20 : rd2_pkt1_dwlen_next;   // dwlen next
                    rd2_pkt2[62:12] <= rd1_out_data[62:12];                             // various data
                    rd2_pkt2[11:0]  <= rd1_out_data[11:0] + (rd2_pkt1_dwlen << 2);      // base address (within 4k page)
                    state2 <= `S2_ENGINE_PROCESSING;
                end
            end
            `S2_ENGINE_PROCESSING: begin
                if ( rd2_out_rden ) begin
                    rd2_total_dwlen <= rd2_total_dwlen_next;                                // dwlen (total remaining)
                    rd2_pkt2[85:74] <= rd2_total_dwlen_next << 2;                           // byte-count
                    rd2_pkt2[73:63] <= (rd2_total_dwlen_next > 11'h20) ? 11'h20 : rd2_total_dwlen_next;   // dwlen next
                    rd2_pkt2[62:12] <= rd2_pkt2[62:12];                                     // various data
                    rd2_pkt2[11:0]  <= rd2_pkt2[11:0] + (rd2_pkt2_dwlen << 2);              // base address (within 4k page)
                    if ( !rd2_pkt2_large ) begin
                        state2 <= `S2_ENGINE_REQDATA;
                    end
                end
            end
        endcase
    
    assign rd1_out_rden = rd2_out_rden && (((state2 == `S2_ENGINE_REQDATA) && (!rd1_out_valid || rd2_pkt1_tiny)) || ((state2 == `S2_ENGINE_PROCESSING) && !rd2_pkt2_large));

    wire [85:0] rd2_in_data  = (state2 == `S2_ENGINE_REQDATA) ? rd2_pkt1 : rd2_pkt2;
    wire        rd2_in_valid = rd1_out_valid || ((state2 == `S2_ENGINE_PROCESSING) && rd2_out_rden);

    bit  [85:0] rd2_out_data;
    bit         rd2_out_valid;
    always @ ( posedge clk ) begin
        rd2_out_data    <= rd2_in_valid ? rd2_in_data : rd2_out_data;
        rd2_out_valid   <= rd2_in_valid && !rst;
    end

    // ------------------------------------------------------------------------
    // 3: PROCESS EACH READ REQUEST PACKAGE PER INDIVIDUAL 32-bit READ DWORDS:
    // ------------------------------------------------------------------------

    wire [4:0]  rd2_out_dwlen   = rd2_out_data[67:63];
    wire        rd2_out_last    = (rd2_out_dwlen == 1);
    wire [9:0]  rd2_out_dwaddr  = rd2_out_data[11:2];
    
    wire        rd3_enable;
    
    bit         rd3_process_valid;
    bit         rd3_process_first;
    bit         rd3_process_last;
    bit [4:0]   rd3_process_dwlen;
    bit [9:0]   rd3_process_dwaddr;
    bit [85:0]  rd3_process_data;
    wire        rd3_process_next_last = (rd3_process_dwlen == 2);
    wire        rd3_process_nextnext_last = (rd3_process_dwlen <= 3);
    
    assign rd_req_ctx   = { rd3_process_first, rd3_process_last, rd3_process_data };
    assign rd_req_bar   = rd3_process_data[62:56];
    assign rd_req_addr  = { rd3_process_data[31:12], rd3_process_dwaddr, 2'b00 };
    assign rd_req_valid = rd3_process_valid;
    
    // STATE MACHINE:
    `define S3_ENGINE_REQDATA     1'h0
    `define S3_ENGINE_PROCESSING  1'h1
    (* KEEP = "TRUE" *) bit [0:0] state3 = `S3_ENGINE_REQDATA;
    
    always @ ( posedge clk )
        if ( rst ) begin
            rd3_process_valid   <= 1'b0;
            state3              <= `S3_ENGINE_REQDATA;
        end
        else case ( state3 )
            `S3_ENGINE_REQDATA: begin
                if ( rd2_out_valid ) begin
                    rd3_process_valid       <= 1'b1;
                    rd3_process_first       <= 1'b1;                    // FIRST
                    rd3_process_last        <= rd2_out_last;            // LAST (low 5 bits of dwlen == 1, [max pktlen = 0x20))
                    rd3_process_dwlen       <= rd2_out_dwlen;           // PKT LENGTH IN DW
                    rd3_process_dwaddr      <= rd2_out_dwaddr;          // DWADDR OF THIS DWORD
                    rd3_process_data[85:0]  <= rd2_out_data[85:0];      // FORWARD / SAVE DATA
                    if ( !rd2_out_last ) begin
                        state3 <= `S3_ENGINE_PROCESSING;
                    end
                end
                else begin
                    rd3_process_valid       <= 1'b0;
                end
            end
            `S3_ENGINE_PROCESSING: begin
                rd3_process_first           <= 1'b0;                    // FIRST
                rd3_process_last            <= rd3_process_next_last;   // LAST
                rd3_process_dwlen           <= rd3_process_dwlen - 1;   // LEN DEC
                rd3_process_dwaddr          <= rd3_process_dwaddr + 1;  // ADDR INC
                if ( rd3_process_next_last ) begin
                    state3 <= `S3_ENGINE_REQDATA;
                end
            end
        endcase

    assign rd2_out_rden = rd3_enable && (
        ((state3 == `S3_ENGINE_REQDATA) && (!rd2_out_valid || rd2_out_last)) ||
        ((state3 == `S3_ENGINE_PROCESSING) && rd3_process_nextnext_last));
    
    // ------------------------------------------------------------------------
    // 4: PROCESS RESPONSES:
    // ------------------------------------------------------------------------
    
    wire        rd_rsp_first    = rd_rsp_ctx[87];
    wire        rd_rsp_last     = rd_rsp_ctx[86];
    
    wire [9:0]  rd_rsp_dwlen    = rd_rsp_ctx[72:63];
    wire [11:0] rd_rsp_bc       = rd_rsp_ctx[85:74];
    wire [15:0] rd_rsp_reqid    = rd_rsp_ctx[47:32];
    wire [7:0]  rd_rsp_tag      = rd_rsp_ctx[55:48];
    wire [6:0]  rd_rsp_lowaddr  = rd_rsp_ctx[6:0];
    wire [31:0] rd_rsp_addr     = rd_rsp_ctx[31:0];
    wire [31:0] rd_rsp_data_bs  = { rd_rsp_data[7:0], rd_rsp_data[15:8], rd_rsp_data[23:16], rd_rsp_data[31:24] };
    
    // 1: 32-bit -> 128-bit state machine:
    bit [127:0] tdata;
    bit [3:0]   tkeepdw = 0;
    bit         tlast;
    bit         first   = 1;
    wire        tvalid  = tlast || tkeepdw[3];
    
    always @ ( posedge clk )
        if ( rst ) begin
            tkeepdw <= 0;
            tlast   <= 0;
            first   <= 0;
        end
        else if ( rd_rsp_valid && rd_rsp_first ) begin
            tkeepdw         <= 4'b1111;
            tlast           <= rd_rsp_last;
            first           <= 1'b1;
            tdata[31:0]     <= { 22'b0100101000000000000000, rd_rsp_dwlen };            // format, type, length
            tdata[63:32]    <= { pcie_id[7:0], pcie_id[15:8], 4'b0, rd_rsp_bc };        // pcie_id, byte_count
            tdata[95:64]    <= { rd_rsp_reqid, rd_rsp_tag, 1'b0, rd_rsp_lowaddr };      // req_id, tag, lower_addr
            tdata[127:96]   <= rd_rsp_data_bs;
        end
        else begin
            tlast   <= rd_rsp_valid && rd_rsp_last;
            tkeepdw <= tvalid ? (rd_rsp_valid ? 4'b0001 : 4'b0000) : (rd_rsp_valid ? ((tkeepdw << 1) | 1'b1) : tkeepdw);
            first   <= 0;
            if ( rd_rsp_valid ) begin
                if ( tvalid || !tkeepdw[0] )
                    tdata[31:0]   <= rd_rsp_data_bs;
                if ( !tkeepdw[1] )
                    tdata[63:32]  <= rd_rsp_data_bs;
                if ( !tkeepdw[2] )
                    tdata[95:64]  <= rd_rsp_data_bs;
                if ( !tkeepdw[3] )
                    tdata[127:96] <= rd_rsp_data_bs;   
            end
        end
    
    // 2.1 - submit to output fifo - will feed into mux/pcie core.
    fifo_134_134_clk1_bar_rdrsp i_fifo_134_134_clk1_bar_rdrsp(
        .srst           ( rst                       ),
        .clk            ( clk                       ),
        .din            ( { first, tlast, tkeepdw, tdata } ),
        .wr_en          ( tvalid                    ),
        .rd_en          ( tlps_out.tready           ),
        .dout           ( { tlps_out.tuser[0], tlps_out.tlast, tlps_out.tkeepdw, tlps_out.tdata } ),
        .full           (                           ),
        .empty          (                           ),
        .prog_empty     ( rd3_enable                ),
        .valid          ( tlps_out.tvalid           )
    );
    
    assign tlps_out.tuser[1] = tlps_out.tlast;
    assign tlps_out.tuser[8:2] = 0;
    
    // 2.2 - packet count:
    bit [10:0]  pkt_count       = 0;
    wire        pkt_count_dec   = tlps_out.tvalid && tlps_out.tlast;
    wire        pkt_count_inc   = tvalid && tlast;
    wire [10:0] pkt_count_next  = pkt_count + pkt_count_inc - pkt_count_dec;
    assign tlps_out.has_data    = (pkt_count_next > 0);
    
    always @ ( posedge clk ) begin
        pkt_count <= rst ? 0 : pkt_count_next;
    end

endmodule



// ------------------------------------------------------------------------
// Example BAR implementation that does nothing but drop any read/writes
// silently without generating a response.
// This is only recommended for placeholder designs.
// Latency = N/A.
// ------------------------------------------------------------------------
module pcileech_bar_impl_none(
    input               rst,
    input               clk,
    // incoming BAR writes:
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // incoming BAR reads:
    input  [87:0]       rd_req_ctx,
    input  [31:0]       rd_req_addr,
    input               rd_req_valid,
    // outgoing BAR read replies:
    output bit [87:0]   rd_rsp_ctx,
    output bit [31:0]   rd_rsp_data,
    output bit          rd_rsp_valid
);

    initial rd_rsp_ctx = 0;
    initial rd_rsp_data = 0;
    initial rd_rsp_valid = 0;

endmodule



// ------------------------------------------------------------------------
// Example BAR implementation of "address loopback" which can be useful
// for testing. Any read to a specific BAR address will result in the
// address as response.
// Latency = 2CLKs.
// ------------------------------------------------------------------------
module pcileech_bar_impl_loopaddr(
    input               rst,
    input               clk,
    // incoming BAR writes:
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // incoming BAR reads:
    input [87:0]        rd_req_ctx,
    input [31:0]        rd_req_addr,
    input               rd_req_valid,
    // outgoing BAR read replies:
    output bit [87:0]   rd_rsp_ctx,
    output bit [31:0]   rd_rsp_data,
    output bit          rd_rsp_valid
);

    bit [87:0]      rd_req_ctx_1;
    bit [31:0]      rd_req_addr_1;
    bit             rd_req_valid_1;
    
    always @ ( posedge clk ) begin
        rd_req_ctx_1    <= rd_req_ctx;
        rd_req_addr_1   <= rd_req_addr;
        rd_req_valid_1  <= rd_req_valid;
        rd_rsp_ctx      <= rd_req_ctx_1;
        rd_rsp_data     <= rd_req_addr_1;
        rd_rsp_valid    <= rd_req_valid_1;
    end    

endmodule



// ------------------------------------------------------------------------
// Example BAR implementation of a 4kB writable initial-zero BAR.
// Latency = 2CLKs.
// ------------------------------------------------------------------------
module pcileech_bar_impl_zerowrite4k(
    input               rst,
    input               clk,
    // incoming BAR writes:
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // incoming BAR reads:
    input  [87:0]       rd_req_ctx,
    input  [31:0]       rd_req_addr,
    input               rd_req_valid,
    // outgoing BAR read replies:
    output bit [87:0]   rd_rsp_ctx,
    output bit [31:0]   rd_rsp_data,
    output bit          rd_rsp_valid
);

    bit [87:0]  drd_req_ctx;
    bit         drd_req_valid;
    wire [31:0] doutb;
    
    always @ ( posedge clk ) begin
        drd_req_ctx     <= rd_req_ctx;
        drd_req_valid   <= rd_req_valid;
        rd_rsp_ctx      <= drd_req_ctx;
        rd_rsp_valid    <= drd_req_valid;
        rd_rsp_data     <= doutb; 
    end
    
    bram_bar_zero4k i_bram_bar_zero4k(
        // Port A - write:
        .addra  ( wr_addr[11:2]     ),
        .clka   ( clk               ),
        .dina   ( wr_data           ),
        .ena    ( wr_valid          ),
        .wea    ( wr_be             ),
        // Port A - read (2 CLK latency):
        .addrb  ( rd_req_addr[11:2] ),
        .clkb   ( clk               ),
        .doutb  ( doutb             ),
        .enb    ( rd_req_valid      )
    );

endmodule



// ------------------------------------------------------------------------
// pcileech wifi BAR implementation
// Works with Qualcomm Atheros AR9287 chip wifi adapters 
// ------------------------------------------------------------------------
// W paste 
module pcileech_bar_impl_ar9287_wifi(
    input               rst,
    input               clk,
    // incoming BAR writes:
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // incoming BAR reads:
    input  [87:0]       rd_req_ctx,
    input  [31:0]       rd_req_addr,
    input               rd_req_valid,
    input  [31:0]       base_address_register,
    // outgoing BAR read replies:
    output reg [87:0]   rd_rsp_ctx,
    output reg [31:0]   rd_rsp_data,
    output reg          rd_rsp_valid
);

    reg [87:0]      drd_req_ctx;
    reg [31:0]      drd_req_addr;
    reg             drd_req_valid;

    reg [31:0]      dwr_addr;
    reg [31:0]      dwr_data;
    reg             dwr_valid;

    reg [31:0]      data_32;

    time number = 0;

    always @ (posedge clk) begin
        if (rst)
            number <= 0;

        number          <= number + 1;
        drd_req_ctx     <= rd_req_ctx;
        drd_req_valid   <= rd_req_valid;
        dwr_valid       <= wr_valid;
        drd_req_addr    <= rd_req_addr;
        rd_rsp_ctx      <= drd_req_ctx;
        rd_rsp_valid    <= drd_req_valid;
        dwr_addr        <= wr_addr;
        dwr_data        <= wr_data;

        if (drd_req_valid) begin
            case (({drd_req_addr[31:24], drd_req_addr[23:16], drd_req_addr[15:08], drd_req_addr[07:00]} - (base_address_register & ~32'h4)) & 32'hFFFF)
				32'h061C : rd_rsp_data <= 32'h00802D0C; // <- specal address register 
                32'h0628 : rd_rsp_data <= 32'h82E20000; // <- specal address register 
                32'h0630 : rd_rsp_data <= 32'h0000142B; // <- specal address register 
			    default : begin
                    case (({drd_req_addr[31:24], drd_req_addr[23:16], drd_req_addr[15:08], drd_req_addr[07:00]} - (base_address_register & ~32'h4)) & 32'h00FF)
							8'h00 : begin
							
								rd_rsp_data[7:0]   <= 8'h80;  // mac prefix 
								rd_rsp_data[15:8]  <= 8'h86;  // mac prefix 
								rd_rsp_data[23:16] <= 8'hF2;  // mac prefix
                            rd_rsp_data[31:24] <= ((0 + (number) % (15 + 1 - 0)) << 4) | (0 + (number + 3) % (15 + 1 - 0));
							end
									8'h04 : begin
									rd_rsp_data[7:0]   <= ((0 + (number + 6) % (15 + 1 - 0)) << 4) | (0 + (number + 9) % (15 + 1 - 0));
									rd_rsp_data[15:8]  <= ((0 + (number + 12) % (15 + 1 - 0)) << 4) | (0 + (number + 15) % (15 + 1 - 0));
									rd_rsp_data[31:16] <= 16'h0000;
									end
			
                16'h0008 : rd_rsp_data <= 32'h00000001;
                16'h000C : rd_rsp_data <= 32'hFA00008A;
                16'h0024 : rd_rsp_data <= 32'h080403C5;
                16'h0028 : rd_rsp_data <= 32'h00000144;
                16'h0030 : rd_rsp_data <= 32'h80000000;
                16'h0034 : rd_rsp_data <= 32'h803A0000;
                16'h0038 : rd_rsp_data <= 32'h80008040;
                16'h003C : rd_rsp_data <= 32'h00080040;
                16'h006C : rd_rsp_data <= 32'hC000003F;
                16'h0070 : rd_rsp_data <= 32'h01800000;
                16'h0080 : rd_rsp_data <= 32'h00000007;
                16'h0084 : rd_rsp_data <= 32'h00001700;
                16'h0088 : rd_rsp_data <= 32'h00000001;
                16'h008C : rd_rsp_data <= 32'h00000001;
                16'h0094 : rd_rsp_data <= 32'h00000018;
                16'h00A0 : rd_rsp_data <= 32'h880CA7CD;
                16'h00A4 : rd_rsp_data <= 32'h01290000;
                16'h00A8 : rd_rsp_data <= 32'h800FFFFF;
                16'h00AC : rd_rsp_data <= 32'h000FFFFF;
                16'h00B0 : rd_rsp_data <= 32'h00000024;
                16'h00B8 : rd_rsp_data <= 32'h00105668;
                16'h00CC : rd_rsp_data <= 32'h560A0000;
                16'h00D4 : rd_rsp_data <= 32'h88AA88AA;
                16'h00DC : rd_rsp_data <= 32'h88AA88AA;
                16'h00E0 : rd_rsp_data <= 32'h10060000;
                16'h00E4 : rd_rsp_data <= 32'h000F0017;
                16'h00E8 : rd_rsp_data <= 32'h00040200;
                16'h00F0 : rd_rsp_data <= 32'hC0008208;
                16'h0100 : rd_rsp_data <= 32'h27800200;
                16'h0104 : rd_rsp_data <= 32'h10001000;
                16'h010C : rd_rsp_data <= 32'h00400040;
                16'h0200 : rd_rsp_data <= 32'h00000060;
                16'h0210 : rd_rsp_data <= 32'h00000001;
                16'h0218 : rd_rsp_data <= 32'h00000040;
                16'h0220 : rd_rsp_data <= 32'hA0AAA0AA;
                16'h0228 : rd_rsp_data <= 32'hC3B7FFD7;
                16'h022C : rd_rsp_data <= 32'h0001001A;
                16'h0240 : rd_rsp_data <= 32'hFFFF0000;
                16'h0244 : rd_rsp_data <= 32'h00600000;
                16'h024C : rd_rsp_data <= 32'h00001FFB;
                16'h0250 : rd_rsp_data <= 32'h000000F0;
                16'h0254 : rd_rsp_data <= 32'h00000001;
                16'h0258 : rd_rsp_data <= 32'h3616C0EE;
                16'h041C : rd_rsp_data <= 32'hFF000000;
                16'h0420 : rd_rsp_data <= 32'h00000005;
                16'h0428 : rd_rsp_data <= 32'h82E20000;
                16'h0430 : rd_rsp_data <= 32'h0000142B;
                16'h0434 : rd_rsp_data <= 32'h00002000;
                16'h0450 : rd_rsp_data <= 32'h0000009A;
                16'h0454 : rd_rsp_data <= 32'h31276924;
                16'h0460 : rd_rsp_data <= 32'h0000142C;
                16'h0468 : rd_rsp_data <= 32'h00005300;
                16'h0484 : rd_rsp_data <= 32'hCD6D6F00;
                16'h0488 : rd_rsp_data <= 32'hCF76EA2C;
                16'h048C : rd_rsp_data <= 32'h000000BD;
                16'h051C : rd_rsp_data <= 32'h00100B03;
                16'h0528 : rd_rsp_data <= 32'h82E20000;
                16'h0530 : rd_rsp_data <= 32'h0000142B;
                16'h0534 : rd_rsp_data <= 32'h00002000;
                16'h0550 : rd_rsp_data <= 32'h0000009A;
                16'h0554 : rd_rsp_data <= 32'h31276924;
                16'h0560 : rd_rsp_data <= 32'h0000142C;
                16'h0568 : rd_rsp_data <= 32'h00005300;
                16'h0584 : rd_rsp_data <= 32'hCD6D6F00;
                16'h0588 : rd_rsp_data <= 32'hCF76EA2C;
                16'h058C : rd_rsp_data <= 32'h000000BD;
                16'h0634 : rd_rsp_data <= 32'h00002000;
                16'h0650 : rd_rsp_data <= 32'h0000009A;
                16'h0654 : rd_rsp_data <= 32'h31276924;
                16'h0660 : rd_rsp_data <= 32'h0000142C;
                16'h0668 : rd_rsp_data <= 32'h00005300;
                16'h0684 : rd_rsp_data <= 32'hCD6D6F00;
                16'h0688 : rd_rsp_data <= 32'hCF76EA2C;
                16'h068C : rd_rsp_data <= 32'h000000BD;
                16'h071C : rd_rsp_data <= 32'h00005204;
                16'h0728 : rd_rsp_data <= 32'h82E20000;
                16'h0730 : rd_rsp_data <= 32'h0000142B;
                16'h0734 : rd_rsp_data <= 32'h00002000;
                16'h0750 : rd_rsp_data <= 32'h0000009A;
                16'h0754 : rd_rsp_data <= 32'h31276924;
                16'h0760 : rd_rsp_data <= 32'h0000142C;
                16'h0768 : rd_rsp_data <= 32'h00005300;
                16'h0784 : rd_rsp_data <= 32'hCD6D6F00;
                16'h0788 : rd_rsp_data <= 32'hCF76EA2C;
                16'h078C : rd_rsp_data <= 32'h000000BD;
                16'h1000 : rd_rsp_data <= 32'h03000000;
                16'h1004 : rd_rsp_data <= 32'hC9B17000;
                16'h1008 : rd_rsp_data <= 32'h8F800680;
                16'h100C : rd_rsp_data <= 32'h0300CA7D;
                16'h1010 : rd_rsp_data <= 32'hCA7DB6C0;
                16'h1014 : rd_rsp_data <= 32'h00000470;
                16'h1080 : rd_rsp_data <= 32'h03000000;
                16'h1084 : rd_rsp_data <= 32'hC9A2C000;
                16'h1088 : rd_rsp_data <= 32'h69600680;
                16'h108C : rd_rsp_data <= 32'h0300C92D;
                16'h1090 : rd_rsp_data <= 32'hC92D0680;
                16'h1094 : rd_rsp_data <= 32'h000001F0;
                16'h1100 : rd_rsp_data <= 32'h03000000;
                16'h1104 : rd_rsp_data <= 32'hC994E800;
                16'h1108 : rd_rsp_data <= 32'hFFC00680;
                16'h110C : rd_rsp_data <= 32'h0280CA7C;
                16'h1110 : rd_rsp_data <= 32'hCA7DA540;
                16'h1114 : rd_rsp_data <= 32'h00000290;
                16'h1180 : rd_rsp_data <= 32'h03000000;
                16'h1184 : rd_rsp_data <= 32'hC994F000;
                16'h1188 : rd_rsp_data <= 32'hF3A00680;
                16'h118C : rd_rsp_data <= 32'h0280CA7C;
                16'h1190 : rd_rsp_data <= 32'hCA7DE540;
                16'h1194 : rd_rsp_data <= 32'h00000690;
                16'h1200 : rd_rsp_data <= 32'h02000000;
                16'h1204 : rd_rsp_data <= 32'hC9762800;
                16'h1208 : rd_rsp_data <= 32'hA62C0680;
                16'h120C : rd_rsp_data <= 32'h03C0CA7D;
                16'h1280 : rd_rsp_data <= 32'h02000000;
                16'h1284 : rd_rsp_data <= 32'hC9763000;
                16'h1288 : rd_rsp_data <= 32'h038C0680;
                16'h128C : rd_rsp_data <= 32'h03C0CA7D;
                16'h1300 : rd_rsp_data <= 32'h03000000;
                16'h1304 : rd_rsp_data <= 32'hC9A88800;
                16'h1308 : rd_rsp_data <= 32'hF62C0680;
                16'h130C : rd_rsp_data <= 32'h03C0C92D;
                16'h1310 : rd_rsp_data <= 32'hCA7D36C0;
                16'h1314 : rd_rsp_data <= 32'h00000260;
                16'h1380 : rd_rsp_data <= 32'h02000000;
                16'h1384 : rd_rsp_data <= 32'hC9A89000;
                16'h1388 : rd_rsp_data <= 32'h738C0680;
                16'h138C : rd_rsp_data <= 32'h03C0CA7D;
                16'h1700 : rd_rsp_data <= 32'h01000000;
                16'h1704 : rd_rsp_data <= 32'hC9F9A000;
                16'h1708 : rd_rsp_data <= 32'h000005C0;
                16'h1780 : rd_rsp_data <= 32'h01000000;
                16'h1784 : rd_rsp_data <= 32'hC9F99000;
                16'h1788 : rd_rsp_data <= 32'h000005C0;
                16'h1900 : rd_rsp_data <= 32'hCA7DB707;
                16'h1908 : rd_rsp_data <= 32'hCA7DE5A9;
                16'h1910 : rd_rsp_data <= 32'hCA7D03C8;
                16'h1918 : rd_rsp_data <= 32'hCA7D73C8;
                16'h1938 : rd_rsp_data <= 32'hC9F9A05C;
                16'h1948 : rd_rsp_data <= 32'hCA4CC85A;
                16'h1958 : rd_rsp_data <= 32'hC001106E;
                16'h195C : rd_rsp_data <= 32'hC011402C;
                16'h1960 : rd_rsp_data <= 32'hC02180BC;
                16'h1964 : rd_rsp_data <= 32'hC031206C;
                16'h1974 : rd_rsp_data <= 32'h00709052;
                16'h1980 : rd_rsp_data <= 32'hC001106E;
                16'h1984 : rd_rsp_data <= 32'hC00130CC;
                16'h1988 : rd_rsp_data <= 32'hC011402B;
                16'h198C : rd_rsp_data <= 32'hC011402C;
                16'h1990 : rd_rsp_data <= 32'hC02180BB;
                16'h1994 : rd_rsp_data <= 32'hC02180BC;
                16'h1998 : rd_rsp_data <= 32'hC031206B;
                16'h199C : rd_rsp_data <= 32'hC031206C;
                16'h19B8 : rd_rsp_data <= 32'h00709052;
                16'h19BC : rd_rsp_data <= 32'h00709051;
                16'h19C8 : rd_rsp_data <= 32'h0080A15A;
                16'h19D0 : rd_rsp_data <= 32'h00CA4C00;
                16'h19D4 : rd_rsp_data <= 32'h00CA4340;
                16'h19D8 : rd_rsp_data <= 32'h00CA3A40;
                16'h19DC : rd_rsp_data <= 32'h00CA3180;
                16'h19E0 : rd_rsp_data <= 32'h00CA28C0;
                16'h19E4 : rd_rsp_data <= 32'h00CA2000;
                16'h19E8 : rd_rsp_data <= 32'h00CA1700;
                16'h19EC : rd_rsp_data <= 32'h00CA0E40;
                16'h19F0 : rd_rsp_data <= 32'h00CA0580;
                16'h19F4 : rd_rsp_data <= 32'h00C9FCC0;
                16'h19F8 : rd_rsp_data <= 32'h00C9F3C0;
                16'h19FC : rd_rsp_data <= 32'h00C9EB00;
                16'h1A00 : rd_rsp_data <= 32'h00C9E240;
                16'h1A04 : rd_rsp_data <= 32'h00C9D980;
                16'h1A08 : rd_rsp_data <= 32'h00C9D080;
                16'h1A0C : rd_rsp_data <= 32'h00C9C7C0;
                16'h1A10 : rd_rsp_data <= 32'hA1101040;
                16'h1A14 : rd_rsp_data <= 32'hC011402C;
                16'h1A18 : rd_rsp_data <= 32'hA2100228;
                16'h1A20 : rd_rsp_data <= 32'hA3100420;
                16'h1A28 : rd_rsp_data <= 32'hA4100208;
                16'h1A30 : rd_rsp_data <= 32'hA5100440;
                16'h1A38 : rd_rsp_data <= 32'hA6130A29;
                16'h1A40 : rd_rsp_data <= 32'h82100228;
                16'h1A48 : rd_rsp_data <= 32'h83100628;
                16'h1A50 : rd_rsp_data <= 32'h84130E29;
                16'h1A58 : rd_rsp_data <= 32'h81922A02;
                16'h1A60 : rd_rsp_data <= 32'h20100000;
                16'h1A64 : rd_rsp_data <= 32'hC011402A;
                16'h1A68 : rd_rsp_data <= 32'h60100000;
                16'h1A70 : rd_rsp_data <= 32'h00503F13;
                16'h1A74 : rd_rsp_data <= 32'h75FC6DED;
                16'h1A80 : rd_rsp_data <= 32'h0060ECB0;
                16'h1A84 : rd_rsp_data <= 32'hD46A9504;
                16'h1A90 : rd_rsp_data <= 32'h00401A90;
                16'h1A94 : rd_rsp_data <= 32'h5B7316BA;
                16'h1AA0 : rd_rsp_data <= 32'h0040307C;
                16'h1AA4 : rd_rsp_data <= 32'h7022EBA5;
                16'h1AE0 : rd_rsp_data <= 32'h00200153;
                16'h1AE4 : rd_rsp_data <= 32'h0160B584;
                16'h1B00 : rd_rsp_data <= 32'h00100000;
                16'h1B04 : rd_rsp_data <= 32'hE52D67C8;
                16'h1B20 : rd_rsp_data <= 32'h00C99BC0;
                16'h1B24 : rd_rsp_data <= 32'h00C99300;
                16'h1B28 : rd_rsp_data <= 32'h00C98A00;
                16'h1B2C : rd_rsp_data <= 32'h00C98140;
                16'h1B30 : rd_rsp_data <= 32'h00C97880;
                16'h1B34 : rd_rsp_data <= 32'h00C96FC0;
                16'h1B38 : rd_rsp_data <= 32'h00C966C0;
                16'h1B3C : rd_rsp_data <= 32'h00C95E00;
                16'h1B40 : rd_rsp_data <= 32'h00C95540;
                16'h1B44 : rd_rsp_data <= 32'h00C94C80;
                16'h1B48 : rd_rsp_data <= 32'h00C94380;
                16'h1B80 : rd_rsp_data <= 32'h00CA7040;
                16'h1B84 : rd_rsp_data <= 32'h00CA72B0;
                16'h1B88 : rd_rsp_data <= 32'h00CA7280;
                16'h1B8C : rd_rsp_data <= 32'h00CA73D0;
                16'h1B90 : rd_rsp_data <= 32'h00CA7520;
                16'h1B94 : rd_rsp_data <= 32'h00CA76D0;
                16'h1B98 : rd_rsp_data <= 32'h00CA77F0;
                16'h1B9C : rd_rsp_data <= 32'h00CA7430;
                16'h1BA0 : rd_rsp_data <= 32'h00CA7C10;
                16'h1BA4 : rd_rsp_data <= 32'h00CA7580;
                16'h1BA8 : rd_rsp_data <= 32'h00CA50F0;
                16'h1BAC : rd_rsp_data <= 32'h00CA7C70;
                16'h1BB0 : rd_rsp_data <= 32'h00CA7CA0;
                16'h1BB4 : rd_rsp_data <= 32'h00CA7B80;
                16'h1BB8 : rd_rsp_data <= 32'h00CA7880;
                16'h1BBC : rd_rsp_data <= 32'h00CA7A60;
                16'h1BC0 : rd_rsp_data <= 32'h0CA4CF00;
                16'h1BC4 : rd_rsp_data <= 32'h00CA4D10;
                16'h1BC8 : rd_rsp_data <= 32'h00000088;
                16'h1BCC : rd_rsp_data <= 32'h00000098;
                16'h1BD0 : rd_rsp_data <= 32'hE0000008;
                16'h1BD4 : rd_rsp_data <= 32'h00CA72B0;
                16'h1BD8 : rd_rsp_data <= 32'h00000040;
                16'h1BDC : rd_rsp_data <= 32'h0C040001;
                16'h1BF0 : rd_rsp_data <= 32'h00C9BF00;
                16'h1BF4 : rd_rsp_data <= 32'h00C9B640;
                16'h1BF8 : rd_rsp_data <= 32'h00C9AD40;
                16'h1BFC : rd_rsp_data <= 32'h00C9A480;
                16'h1C00 : rd_rsp_data <= 32'h80811114;
                16'h1C08 : rd_rsp_data <= 32'h4C04F926;
                16'h1C10 : rd_rsp_data <= 32'h00001020;
                16'h1C40 : rd_rsp_data <= 32'h000000FC;
                16'h1C44 : rd_rsp_data <= 32'h07830000;
                16'h1C4C : rd_rsp_data <= 32'h80000000;
                16'h1C50 : rd_rsp_data <= 32'h00002000;
                16'h1C54 : rd_rsp_data <= 32'h00000001;
                16'h1C58 : rd_rsp_data <= 32'h81DD01DD;
                16'h1C5C : rd_rsp_data <= 32'h00000080;
                16'h1C60 : rd_rsp_data <= 32'hE00FFFFF;
                16'h1C64 : rd_rsp_data <= 32'h000FFFFF;
                16'h1D00 : rd_rsp_data <= 32'h80000008;
                16'h1D04 : rd_rsp_data <= 32'h00001FF8;
                16'h1D08 : rd_rsp_data <= 32'h30000700;
                16'h1D20 : rd_rsp_data <= 32'h80000008;
                16'h1D24 : rd_rsp_data <= 32'h00001FF8;
                16'h1D28 : rd_rsp_data <= 32'h10000400;
                16'h1D40 : rd_rsp_data <= 32'h80000008;
                16'h1D44 : rd_rsp_data <= 32'h00001FF8;
                16'h1D48 : rd_rsp_data <= 32'h00000400;
                16'h1D60 : rd_rsp_data <= 32'h80000008;
                16'h1D64 : rd_rsp_data <= 32'h00001FF8;
                16'h1D68 : rd_rsp_data <= 32'h00000400;
                16'h1D80 : rd_rsp_data <= 32'h80000008;
                16'h1D84 : rd_rsp_data <= 32'h00001FF8;
                16'h1D88 : rd_rsp_data <= 32'h00000400;
                16'h1DA0 : rd_rsp_data <= 32'h80000008;
                16'h1DA4 : rd_rsp_data <= 32'h00000678;
                16'h1DA8 : rd_rsp_data <= 32'h00000400;
                16'h1DC0 : rd_rsp_data <= 32'h80000008;
                16'h1DC4 : rd_rsp_data <= 32'h0000DE54;
                16'h1DC8 : rd_rsp_data <= 32'h00000400;
                16'h1DE0 : rd_rsp_data <= 32'h80000008;
                16'h1DE4 : rd_rsp_data <= 32'h00000140;
                16'h1DE8 : rd_rsp_data <= 32'h00000700;
                16'h1E20 : rd_rsp_data <= 32'h80400000;
                16'h1E64 : rd_rsp_data <= 32'h000007FF;
                16'h1E98 : rd_rsp_data <= 32'h00000003;
                16'h1E9C : rd_rsp_data <= 32'h0000000C;
                16'h1EA0 : rd_rsp_data <= 32'h00000007;
                16'h1EA4 : rd_rsp_data <= 32'h00020806;
                16'h1EA8 : rd_rsp_data <= 32'h07FF8008;
                16'h1EAC : rd_rsp_data <= 32'h00000260;
                16'h1EB0 : rd_rsp_data <= 32'h07FF0001;
                    endcase
                end
            endcase
        end else if (dwr_valid) begin
             case (({dwr_addr[31:24], dwr_addr[23:16], dwr_addr[15:08], dwr_addr[07:00]} - (base_address_register & ~32'h4)) & 32'h00FF) //
            endcase
        end else begin
                            rd_rsp_data[7:0]   <= ((0 + (number) % (15 + 1 - 0)) << 4) | (0 + (number + 3) % (15 + 1 - 0));
                            rd_rsp_data[15:8]  <= ((0 + (number + 6) % (15 + 1 - 0)) << 4) | (0 + (number + 9) % (15 + 1 - 0));
                            rd_rsp_data[23:16] <= ((0 + (number + 12) % (15 + 1 - 0)) << 4) | (0 + (number + 15) % (15 + 1 - 0));
                            rd_rsp_data[31:24] <= ((0 + (number) % (15 + 1 - 0)) << 4) | (0 + (number + 3) % (15 + 1 - 0));
        end
    end

endmodule
