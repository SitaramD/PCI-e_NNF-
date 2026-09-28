`timescale 1ns/1ps
// =============================================================================
// pcie_rc_bfm.sv  (simulation only)
// -----------------------------------------------------------------------------
// Root-complex bus functional model.
//
// The PHY and Data Link Layer are the *same RTL* as the endpoint, built as a
// downstream port (DOWNSTREAM = 1). On top of the DLL this model provides
// testbench tasks that build TLPs and wait for completions:
//
//   cfg_rd (bdf, reg_dw,               data, status)
//   cfg_wr (bdf, reg_dw, data, be,           status)
//   mem_wr (addr, ndw, fbe, lbe)        payload in wr_buf[]
//   mem_rd (addr, ndw, fbe, lbe,             status)   data in rd_buf[]
//
// Requester ID of the root complex is 0x0000. Tags count 0..255.
// =============================================================================
module pcie_rc_bfm #(
  parameter int unsigned T_DETECT_QUIET  = 3_000_000,
  parameter int unsigned T_2MS           = 500_000,
  parameter int unsigned T_24MS          = 6_000_000,
  parameter int unsigned T_48MS          = 12_000_000,
  parameter int unsigned POLL_TS1_MIN    = 1024,
  parameter int unsigned REPLAY_TIMEOUT  = 3000,
  parameter int unsigned UPDATEFC_PERIOD = 7500,
  parameter int          CPL_TIMEOUT     = 200_000     // cycles
) (
  input  logic       clk,
  input  logic       rst,
  output logic [9:0] tx_sym,
  output logic       tx_eidle,
  input  logic       rx_clk,
  input  logic [9:0] rx_raw,
  input  logic       force_retrain,
  output logic       link_up,
  output logic       dl_up,
  output pcie_pkg::ltssm_e ltssm_state
);
  import pcie_pkg::*;

  // ---------------------------------------------------------------------------
  // PHY + DLL (downstream port)
  // ---------------------------------------------------------------------------
  logic       p_tx_valid, p_tx_sop, p_tx_eop, p_tx_dllp, p_tx_ready;
  logic [7:0] p_tx_data;
  logic       p_rx_valid, p_rx_sop, p_rx_eop, p_rx_dllp, p_rx_err, p_rx_edb;
  logic [7:0] p_rx_data;
  logic       in_l0, dll_retrain, rx_err_pulse;
  logic [1:0] dl_state;

  logic        tl_tx_valid = 1'b0, tl_tx_sop = 1'b0, tl_tx_eop = 1'b0;
  logic [7:0]  tl_tx_data  = 8'h00;
  logic [1:0]  tl_tx_fc_type = 2'd0;
  logic [11:0] tl_tx_fc_data = 12'd0;
  logic        tl_tx_ready;
  logic        tl_rx_valid, tl_rx_sop, tl_rx_eop;
  logic [7:0]  tl_rx_data;

  logic [15:0] stat_bad_tlp, stat_bad_dllp, stat_dup_tlp, stat_replays, stat_naks_rx;

  pcie_phy_top #(
    .DOWNSTREAM     (1'b1),
    .LINK_NUM       (8'd0),
    .T_DETECT_QUIET (T_DETECT_QUIET),
    .T_2MS          (T_2MS),
    .T_24MS         (T_24MS),
    .T_48MS         (T_48MS),
    .POLL_TS1_MIN   (POLL_TS1_MIN)
  ) u_phy (
    .clk          (clk),
    .rst          (rst),
    .tx_sym       (tx_sym),
    .tx_eidle     (tx_eidle),
    .rx_clk       (rx_clk),
    .rx_raw       (rx_raw),
    .rx_present   (1'b1),
    .dll_tx_valid (p_tx_valid),
    .dll_tx_data  (p_tx_data),
    .dll_tx_sop   (p_tx_sop),
    .dll_tx_eop   (p_tx_eop),
    .dll_tx_dllp  (p_tx_dllp),
    .dll_tx_ready (p_tx_ready),
    .dll_rx_valid (p_rx_valid),
    .dll_rx_data  (p_rx_data),
    .dll_rx_sop   (p_rx_sop),
    .dll_rx_eop   (p_rx_eop),
    .dll_rx_dllp  (p_rx_dllp),
    .dll_rx_err   (p_rx_err),
    .dll_rx_edb   (p_rx_edb),
    .retrain_req  (dll_retrain | force_retrain),
    .link_up      (link_up),
    .in_l0        (in_l0),
    .ltssm_state  (ltssm_state),
    .rx_err_pulse (rx_err_pulse)
  );

  pcie_dll_top #(
    .REPLAY_TIMEOUT  (REPLAY_TIMEOUT),
    .UPDATEFC_PERIOD (UPDATEFC_PERIOD),
    .ADV_PH          (8'd16),
    .ADV_PD          (12'd128),
    .ADV_NPH         (8'd16),
    .ADV_NPD         (12'd16),
    .ADV_CPLH        (8'd0),
    .ADV_CPLD        (12'd0)
  ) u_dll (
    .clk           (clk),
    .rst           (rst),
    .phy_link_up   (link_up),
    .phy_in_l0     (in_l0),
    .phy_tx_valid  (p_tx_valid),
    .phy_tx_data   (p_tx_data),
    .phy_tx_sop    (p_tx_sop),
    .phy_tx_eop    (p_tx_eop),
    .phy_tx_dllp   (p_tx_dllp),
    .phy_tx_ready  (p_tx_ready),
    .phy_rx_valid  (p_rx_valid),
    .phy_rx_data   (p_rx_data),
    .phy_rx_sop    (p_rx_sop),
    .phy_rx_eop    (p_rx_eop),
    .phy_rx_dllp   (p_rx_dllp),
    .phy_rx_err    (p_rx_err),
    .phy_rx_edb    (p_rx_edb),
    .tl_tx_valid   (tl_tx_valid),
    .tl_tx_data    (tl_tx_data),
    .tl_tx_sop     (tl_tx_sop),
    .tl_tx_eop     (tl_tx_eop),
    .tl_tx_fc_type (tl_tx_fc_type),
    .tl_tx_fc_data (tl_tx_fc_data),
    .tl_tx_ready   (tl_tx_ready),
    .tl_rx_valid   (tl_rx_valid),
    .tl_rx_data    (tl_rx_data),
    .tl_rx_sop     (tl_rx_sop),
    .tl_rx_eop     (tl_rx_eop),
    .tl_rx_ready   (1'b1),
    .dl_up         (dl_up),
    .dl_state      (dl_state),
    .retrain_req   (dll_retrain),
    .stat_bad_tlp  (stat_bad_tlp),
    .stat_bad_dllp (stat_bad_dllp),
    .stat_dup_tlp  (stat_dup_tlp),
    .stat_replays  (stat_replays),
    .stat_naks_rx  (stat_naks_rx)
  );

  // ---------------------------------------------------------------------------
  // Receive: collect completions by tag
  // ---------------------------------------------------------------------------
  localparam int MAXB = 12 + 4 * 32;

  logic [7:0] cpl_bytes [256][MAXB];
  int         cpl_len   [256];
  bit         cpl_got   [256];
  logic [7:0] rxb [MAXB];
  int         rxn = 0;
  int         n_cpl_rx = 0;

  initial for (int i = 0; i < 256; i++) cpl_got[i] = 1'b0;

  // Sampled on the falling edge: tl_rx_ready is always 1, so the byte visible
  // now is consumed at the next rising edge.
  always @(negedge clk) begin
    if (tl_rx_valid) begin
      int n;
      n = tl_rx_sop ? 0 : rxn;
      if (n < MAXB) rxb[n] = tl_rx_data;
      n++;
      rxn = n;
      if (tl_rx_eop) begin
        if (rxb[0][4:1] == 4'b0101) begin          // Cpl / CplD
          int tag;
          tag = rxb[10];
          for (int i = 0; i < MAXB; i++) cpl_bytes[tag][i] = rxb[i];
          cpl_len[tag] = n;
          cpl_got[tag] = 1'b1;
          n_cpl_rx++;
        end else begin
          $display("%t RC: received unexpected TLP type %02x (ignored)", $time, rxb[0]);
        end
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Transmit helper
  // ---------------------------------------------------------------------------
  logic [7:0]  txb    [MAXB];
  logic [31:0] wr_buf [32];     // payload for mem_wr
  logic [31:0] rd_buf [32];     // data from mem_rd
  logic [7:0]  next_tag = 8'd0;

  // Drive a TLP from txb[0..n-1] into the DLL. Signals change on the falling
  // edge and "ready" is sampled mid-cycle, so there is no race with the RTL
  // that samples on the rising edge.
  task automatic send_txb(input int n, input logic [1:0] ft, input logic [11:0] fd);
    int i;
    bit go;
    i = 0;
    @(negedge clk);
    tl_tx_valid   = 1'b1;
    tl_tx_data    = txb[0];
    tl_tx_sop     = 1'b1;
    tl_tx_eop     = (n == 1);
    tl_tx_fc_type = ft;
    tl_tx_fc_data = fd;
    while (i < n) begin
      go = tl_tx_ready;          // will this byte be taken at the next rising edge?
      @(negedge clk);
      if (go) begin
        i++;
        if (i < n) begin
          tl_tx_data = txb[i];
          tl_tx_sop  = 1'b0;
          tl_tx_eop  = (i == n - 1);
        end else begin
          tl_tx_valid = 1'b0;
          tl_tx_sop   = 1'b0;
          tl_tx_eop   = 1'b0;
        end
      end
    end
  endtask

  task automatic wait_cpl(input logic [7:0] tag, output bit ok);
    int t;
    t  = 0;
    ok = 1'b0;
    while (!cpl_got[tag] && t < CPL_TIMEOUT) begin
      @(posedge clk);
      t++;
    end
    ok = cpl_got[tag];
    if (!ok) $display("%t RC: *** completion timeout, tag %0d", $time, tag);
  endtask

  function automatic logic [7:0] alloc_tag();
    logic [7:0] t;
    t        = next_tag;
    next_tag = next_tag + 8'd1;
    cpl_got[t] = 1'b0;
    return t;
  endfunction

  // Header DW0..1 common part
  task automatic put_hdr01(input logic [7:0] fmt_type, input logic [9:0] len,
                           input logic [7:0] tag, input logic [3:0] lbe, input logic [3:0] fbe);
    txb[0] = fmt_type;
    txb[1] = 8'h00;
    txb[2] = {6'b000000, len[9:8]};
    txb[3] = len[7:0];
    txb[4] = 8'h00;           // requester ID = 0x0000
    txb[5] = 8'h00;
    txb[6] = tag;
    txb[7] = {lbe, fbe};
  endtask

  // ---------------------------------------------------------------------------
  // Configuration read / write (type 0)
  // ---------------------------------------------------------------------------
  task automatic cfg_rd(input logic [15:0] bdf, input logic [9:0] reg_dw,
                        output logic [31:0] data, output logic [2:0] status);
    logic [7:0] tag;
    bit ok;
    tag = alloc_tag();
    put_hdr01(TLP_CFGRD0, 10'd1, tag, 4'h0, 4'hF);
    txb[8]  = bdf[15:8];
    txb[9]  = bdf[7:0];
    txb[10] = {4'h0, reg_dw[9:6]};
    txb[11] = {reg_dw[5:0], 2'b00};
    send_txb(12, FC_NP, 12'd0);
    wait_cpl(tag, ok);
    status = ok ? cpl_bytes[tag][6][7:5] : 3'b111;
    data   = ok ? {cpl_bytes[tag][15], cpl_bytes[tag][14], cpl_bytes[tag][13], cpl_bytes[tag][12]} : 32'hx;
  endtask

  task automatic cfg_wr(input logic [15:0] bdf, input logic [9:0] reg_dw,
                        input logic [31:0] data, input logic [3:0] be,
                        output logic [2:0] status);
    logic [7:0] tag;
    bit ok;
    tag = alloc_tag();
    put_hdr01(TLP_CFGWR0, 10'd1, tag, 4'h0, be);
    txb[8]  = bdf[15:8];
    txb[9]  = bdf[7:0];
    txb[10] = {4'h0, reg_dw[9:6]};
    txb[11] = {reg_dw[5:0], 2'b00};
    txb[12] = data[7:0];
    txb[13] = data[15:8];
    txb[14] = data[23:16];
    txb[15] = data[31:24];
    send_txb(16, FC_NP, 12'd1);
    wait_cpl(tag, ok);
    status = ok ? cpl_bytes[tag][6][7:5] : 3'b111;
  endtask

  // ---------------------------------------------------------------------------
  // Memory write (posted) / read (non-posted), 32-bit addressing
  // ---------------------------------------------------------------------------
  task automatic mem_wr(input logic [31:0] addr, input int ndw,
                        input logic [3:0] fbe, input logic [3:0] lbe);
    logic [7:0] tag;
    tag = alloc_tag();
    put_hdr01(TLP_MWR32, 10'(ndw), tag, (ndw == 1) ? 4'h0 : lbe, fbe);
    txb[8]  = addr[31:24];
    txb[9]  = addr[23:16];
    txb[10] = addr[15:8];
    txb[11] = {addr[7:2], 2'b00};
    for (int i = 0; i < ndw; i++) begin
      txb[12 + 4*i + 0] = wr_buf[i][7:0];
      txb[12 + 4*i + 1] = wr_buf[i][15:8];
      txb[12 + 4*i + 2] = wr_buf[i][23:16];
      txb[12 + 4*i + 3] = wr_buf[i][31:24];
    end
    send_txb(12 + 4*ndw, FC_P, 12'((ndw + 3) / 4));
  endtask

  task automatic mem_rd(input logic [31:0] addr, input int ndw,
                        input logic [3:0] fbe, input logic [3:0] lbe,
                        output logic [2:0] status);
    logic [7:0] tag;
    bit ok;
    tag = alloc_tag();
    put_hdr01(TLP_MRD32, 10'(ndw), tag, (ndw == 1) ? 4'h0 : lbe, fbe);
    txb[8]  = addr[31:24];
    txb[9]  = addr[23:16];
    txb[10] = addr[15:8];
    txb[11] = {addr[7:2], 2'b00};
    send_txb(12, FC_NP, 12'd0);
    wait_cpl(tag, ok);
    status = ok ? cpl_bytes[tag][6][7:5] : 3'b111;
    if (ok && cpl_bytes[tag][0] == TLP_CPLD) begin
      for (int i = 0; i < ndw; i++)
        rd_buf[i] = {cpl_bytes[tag][12 + 4*i + 3], cpl_bytes[tag][12 + 4*i + 2],
                     cpl_bytes[tag][12 + 4*i + 1], cpl_bytes[tag][12 + 4*i + 0]};
    end
  endtask

  // Byte count field of the last completion received for a tag
  function automatic logic [11:0] cpl_byte_count(input logic [7:0] tag);
    return {cpl_bytes[tag][6][3:0], cpl_bytes[tag][7]};
  endfunction

  function automatic logic [15:0] cpl_completer_id(input logic [7:0] tag);
    return {cpl_bytes[tag][4], cpl_bytes[tag][5]};
  endfunction

endmodule
