`timescale 1ns/1ps
// =============================================================================
// axil_regfile.sv
// -----------------------------------------------------------------------------
// Example AXI4-Lite slave: 2**ADDR_BITS bytes of 32-bit registers with byte
// strobes. Sits behind BAR0 in the example design. Replace it with your own
// register block (for example the T2T axil_regs: fair_px @ 0x04,
// REG_MAX_NOTIONAL, ...).
//
// Register 0x000 is read-only and returns ID_VALUE, so software can check
// that BAR0 is mapped correctly.
// =============================================================================
module axil_regfile #(
  parameter int          ADDR_BITS = 12,
  parameter logic [31:0] ID_VALUE  = 32'hC0DE_0001
) (
  input  logic        clk,
  input  logic        rst,
  input  logic [31:0] s_axil_awaddr,
  input  logic        s_axil_awvalid,
  output logic        s_axil_awready,
  input  logic [31:0] s_axil_wdata,
  input  logic [3:0]  s_axil_wstrb,
  input  logic        s_axil_wvalid,
  output logic        s_axil_wready,
  output logic [1:0]  s_axil_bresp,
  output logic        s_axil_bvalid,
  input  logic        s_axil_bready,
  input  logic [31:0] s_axil_araddr,
  input  logic        s_axil_arvalid,
  output logic        s_axil_arready,
  output logic [31:0] s_axil_rdata,
  output logic [1:0]  s_axil_rresp,
  output logic        s_axil_rvalid,
  input  logic        s_axil_rready
);
  localparam int N = 1 << (ADDR_BITS - 2);

  logic [31:0] regs [N];
  logic [31:0] awaddr_q, wdata_q;
  logic [3:0]  wstrb_q;
  logic        aw_q, w_q;
  logic        clearing;                     // zero the registers after reset
  logic [ADDR_BITS-3:0] clr_idx;
  logic [ADDR_BITS-3:0] widx;

  assign widx           = awaddr_q[ADDR_BITS-1:2];
  assign s_axil_awready = !clearing && !aw_q && !s_axil_bvalid;
  assign s_axil_wready  = !clearing && !w_q  && !s_axil_bvalid;
  assign s_axil_bresp   = 2'b00;
  assign s_axil_rresp   = 2'b00;
  assign s_axil_arready = !clearing && !s_axil_rvalid;

  always_ff @(posedge clk) begin
    if (rst) begin
      aw_q          <= 1'b0;
      w_q           <= 1'b0;
      awaddr_q      <= 32'h0;
      wdata_q       <= 32'h0;
      wstrb_q       <= 4'h0;
      s_axil_bvalid <= 1'b0;
      s_axil_rvalid <= 1'b0;
      s_axil_rdata  <= 32'h0;
      clearing      <= 1'b1;
      clr_idx       <= '0;
    end else if (clearing) begin
      regs[clr_idx] <= 32'h0;
      clr_idx       <= clr_idx + 1'b1;
      if (clr_idx == '1) clearing <= 1'b0;
    end else begin
      // ---- write ----
      if (s_axil_awvalid && s_axil_awready) begin
        aw_q     <= 1'b1;
        awaddr_q <= s_axil_awaddr;
      end
      if (s_axil_wvalid && s_axil_wready) begin
        w_q     <= 1'b1;
        wdata_q <= s_axil_wdata;
        wstrb_q <= s_axil_wstrb;
      end
      if (aw_q && w_q) begin
        if (wstrb_q[0]) regs[widx][7:0]   <= wdata_q[7:0];
        if (wstrb_q[1]) regs[widx][15:8]  <= wdata_q[15:8];
        if (wstrb_q[2]) regs[widx][23:16] <= wdata_q[23:16];
        if (wstrb_q[3]) regs[widx][31:24] <= wdata_q[31:24];
        aw_q          <= 1'b0;
        w_q           <= 1'b0;
        s_axil_bvalid <= 1'b1;
      end
      if (s_axil_bvalid && s_axil_bready) s_axil_bvalid <= 1'b0;

      // ---- read ----
      if (s_axil_arvalid && s_axil_arready) begin
        s_axil_rvalid <= 1'b1;
        s_axil_rdata  <= (s_axil_araddr[ADDR_BITS-1:2] == '0) ? ID_VALUE
                                                              : regs[s_axil_araddr[ADDR_BITS-1:2]];
      end else if (s_axil_rvalid && s_axil_rready) begin
        s_axil_rvalid <= 1'b0;
      end
    end
  end

endmodule
