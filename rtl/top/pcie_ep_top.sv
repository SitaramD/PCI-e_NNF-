`timescale 1ns/1ps
// =============================================================================
// pcie_ep_top.sv
// -----------------------------------------------------------------------------
// PCIe Gen1 x1 endpoint, written from scratch:
//
//          +-----------------------------------------------+
//  AXI4-   |  pcie_tl_ep   (config space, BAR0, MRd/MWr)   |
//  Lite <--|-----------------------------------------------|
//  master  |  pcie_dll_top (seq#, LCRC, Ack/Nak, replay,   |
//          |                flow control, DL state)        |
//          |-----------------------------------------------|
//          |  pcie_phy_top (LTSSM, TS1/TS2, SKP, framing,  |
//          |                scrambler, 8b/10b, EB, align)  |
//          +-----------------------------------------------+
//                 tx_sym[9:0] ->        <- rx_raw[9:0] @ rx_clk
//
// clk must be 250 MHz (one symbol per clock). All other parameters can be
// shortened for simulation (see tb/top/tb_pcie_top.sv).
// =============================================================================
module pcie_ep_top #(
  parameter logic [15:0] VENDOR_ID       = 16'h1234,
  parameter logic [15:0] DEVICE_ID       = 16'h0001,
  parameter logic [23:0] CLASS_CODE      = 24'h120000,
  parameter int          BAR0_BITS       = 12,
  parameter int unsigned T_DETECT_QUIET  = 3_000_000,
  parameter int unsigned T_2MS           = 500_000,
  parameter int unsigned T_24MS          = 6_000_000,
  parameter int unsigned T_48MS          = 12_000_000,
  parameter int unsigned POLL_TS1_MIN    = 1024,
  parameter int unsigned REPLAY_TIMEOUT  = 3000,
  parameter int unsigned UPDATEFC_PERIOD = 7500
) (
  input  logic        clk,
  input  logic        rst,

  // ---- serial side (transceiver in raw 10-bit mode) ----
  output logic [9:0]  tx_sym,
  output logic        tx_eidle,
  input  logic        rx_clk,
  input  logic [9:0]  rx_raw,
  input  logic        rx_present,

  // ---- AXI4-Lite master: BAR0 accesses ----
  output logic [31:0] m_axil_awaddr,
  output logic        m_axil_awvalid,
  input  logic        m_axil_awready,
  output logic [31:0] m_axil_wdata,
  output logic [3:0]  m_axil_wstrb,
  output logic        m_axil_wvalid,
  input  logic        m_axil_wready,
  input  logic [1:0]  m_axil_bresp,
  input  logic        m_axil_bvalid,
  output logic        m_axil_bready,
  output logic [31:0] m_axil_araddr,
  output logic        m_axil_arvalid,
  input  logic        m_axil_arready,
  input  logic [31:0] m_axil_rdata,
  input  logic [1:0]  m_axil_rresp,
  input  logic        m_axil_rvalid,
  output logic        m_axil_rready,

  // ---- status ----
  output pcie_pkg::ltssm_e ltssm_state,
  output logic        link_up,
  output logic        dl_up,
  output logic [15:0] completer_id,
  output logic [15:0] stat_bad_tlp,
  output logic [15:0] stat_replays
);
  import pcie_pkg::*;

  // PHY <-> DLL
  logic       p_tx_valid, p_tx_sop, p_tx_eop, p_tx_dllp, p_tx_ready;
  logic [7:0] p_tx_data;
  logic       p_rx_valid, p_rx_sop, p_rx_eop, p_rx_dllp, p_rx_err, p_rx_edb;
  logic [7:0] p_rx_data;
  logic       in_l0, retrain_req, rx_err_pulse;

  // DLL <-> TL
  logic        t_tx_valid, t_tx_sop, t_tx_eop, t_tx_ready;
  logic [7:0]  t_tx_data;
  logic [1:0]  t_tx_fc_type;
  logic [11:0] t_tx_fc_data;
  logic        t_rx_valid, t_rx_sop, t_rx_eop, t_rx_ready;
  logic [7:0]  t_rx_data;
  logic [1:0]  dl_state;
  logic [15:0] s_bad_dllp, s_dup, s_naks, s_ur, s_mwr, s_mrd;
  logic        mem_enable, bus_master;
  logic [31:0] bar0;

  pcie_phy_top #(
    .DOWNSTREAM     (1'b0),
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
    .rx_present   (rx_present),
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
    .retrain_req  (retrain_req),
    .link_up      (link_up),
    .in_l0        (in_l0),
    .ltssm_state  (ltssm_state),
    .rx_err_pulse (rx_err_pulse)
  );

  pcie_dll_top #(
    .REPLAY_TIMEOUT  (REPLAY_TIMEOUT),
    .UPDATEFC_PERIOD (UPDATEFC_PERIOD),
    .ADV_PH          (8'd8),
    .ADV_PD          (12'd64),
    .ADV_NPH         (8'd8),
    .ADV_NPD         (12'd8),
    .ADV_CPLH        (8'd0),   // endpoints advertise infinite completion credits
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
    .tl_tx_valid   (t_tx_valid),
    .tl_tx_data    (t_tx_data),
    .tl_tx_sop     (t_tx_sop),
    .tl_tx_eop     (t_tx_eop),
    .tl_tx_fc_type (t_tx_fc_type),
    .tl_tx_fc_data (t_tx_fc_data),
    .tl_tx_ready   (t_tx_ready),
    .tl_rx_valid   (t_rx_valid),
    .tl_rx_data    (t_rx_data),
    .tl_rx_sop     (t_rx_sop),
    .tl_rx_eop     (t_rx_eop),
    .tl_rx_ready   (t_rx_ready),
    .dl_up         (dl_up),
    .dl_state      (dl_state),
    .retrain_req   (retrain_req),
    .stat_bad_tlp  (stat_bad_tlp),
    .stat_bad_dllp (s_bad_dllp),
    .stat_dup_tlp  (s_dup),
    .stat_replays  (stat_replays),
    .stat_naks_rx  (s_naks)
  );

  pcie_tl_ep #(
    .VENDOR_ID  (VENDOR_ID),
    .DEVICE_ID  (DEVICE_ID),
    .CLASS_CODE (CLASS_CODE),
    .BAR0_BITS  (BAR0_BITS)
  ) u_tl (
    .clk            (clk),
    .rst            (rst),
    .dl_up          (dl_up),
    .rx_valid       (t_rx_valid),
    .rx_data        (t_rx_data),
    .rx_sop         (t_rx_sop),
    .rx_eop         (t_rx_eop),
    .rx_ready       (t_rx_ready),
    .tx_valid       (t_tx_valid),
    .tx_data        (t_tx_data),
    .tx_sop         (t_tx_sop),
    .tx_eop         (t_tx_eop),
    .tx_fc_type     (t_tx_fc_type),
    .tx_fc_data     (t_tx_fc_data),
    .tx_ready       (t_tx_ready),
    .m_axil_awaddr  (m_axil_awaddr),
    .m_axil_awvalid (m_axil_awvalid),
    .m_axil_awready (m_axil_awready),
    .m_axil_wdata   (m_axil_wdata),
    .m_axil_wstrb   (m_axil_wstrb),
    .m_axil_wvalid  (m_axil_wvalid),
    .m_axil_wready  (m_axil_wready),
    .m_axil_bresp   (m_axil_bresp),
    .m_axil_bvalid  (m_axil_bvalid),
    .m_axil_bready  (m_axil_bready),
    .m_axil_araddr  (m_axil_araddr),
    .m_axil_arvalid (m_axil_arvalid),
    .m_axil_arready (m_axil_arready),
    .m_axil_rdata   (m_axil_rdata),
    .m_axil_rresp   (m_axil_rresp),
    .m_axil_rvalid  (m_axil_rvalid),
    .m_axil_rready  (m_axil_rready),
    .completer_id   (completer_id),
    .mem_enable     (mem_enable),
    .bus_master     (bus_master),
    .bar0           (bar0),
    .stat_ur        (s_ur),
    .stat_mwr       (s_mwr),
    .stat_mrd       (s_mrd)
  );

endmodule
