`timescale 1ns/1ps
// =============================================================================
// pcie_phy_top.sv
// -----------------------------------------------------------------------------
// Physical layer (logical sub-block) = LTSSM + TX path + RX path.
//
// Serial side is a 10-bit parallel interface, i.e. what a transceiver in
// "raw" mode (8b/10b and comma alignment bypassed) gives you:
//   tx_sym  : 10-bit code group per clk, bit 0 sent first
//   rx_raw  : 10 bits per rx_clk, arbitrary word alignment
//   rx_clk  : recovered clock from the CDR
// =============================================================================
module pcie_phy_top #(
  parameter bit          DOWNSTREAM     = 1'b0,
  parameter logic [7:0]  LINK_NUM       = 8'd0,
  parameter int unsigned T_DETECT_QUIET = 3_000_000,
  parameter int unsigned T_2MS          = 500_000,
  parameter int unsigned T_24MS         = 6_000_000,
  parameter int unsigned T_48MS         = 12_000_000,
  parameter int unsigned POLL_TS1_MIN   = 1024
) (
  input  logic                 clk,
  input  logic                 rst,

  // ---- serial side ----
  output logic [9:0]           tx_sym,
  output logic                 tx_eidle,
  input  logic                 rx_clk,
  input  logic [9:0]           rx_raw,
  input  logic                 rx_present,

  // ---- DLL TX ----
  input  logic                 dll_tx_valid,
  input  logic [7:0]           dll_tx_data,
  input  logic                 dll_tx_sop,
  input  logic                 dll_tx_eop,
  input  logic                 dll_tx_dllp,
  output logic                 dll_tx_ready,

  // ---- DLL RX ----
  output logic                 dll_rx_valid,
  output logic [7:0]           dll_rx_data,
  output logic                 dll_rx_sop,
  output logic                 dll_rx_eop,
  output logic                 dll_rx_dllp,
  output logic                 dll_rx_err,
  output logic                 dll_rx_edb,

  // ---- control / status ----
  input  logic                 retrain_req,
  output logic                 link_up,
  output logic                 in_l0,
  output pcie_pkg::ltssm_e     ltssm_state,
  output logic                 rx_err_pulse
);
  import pcie_pkg::*;

  tx_mode_e   tx_mode;
  logic [7:0] tx_link, tx_lane, ts_link, ts_lane, link_num;
  logic       tx_link_pad, tx_lane_pad, ts_link_pad, ts_lane_pad;
  logic       ts_valid, ts_is_ts2, idle_sym, nonidle_sym;
  logic       ts_sent, ts_sent_ts2, idle_sent;
  logic       skp_add, skp_drop;

  pcie_ltssm #(
    .DOWNSTREAM     (DOWNSTREAM),
    .LINK_NUM       (LINK_NUM),
    .T_DETECT_QUIET (T_DETECT_QUIET),
    .T_2MS          (T_2MS),
    .T_24MS         (T_24MS),
    .T_48MS         (T_48MS),
    .POLL_TS1_MIN   (POLL_TS1_MIN)
  ) u_ltssm (
    .clk          (clk),
    .rst          (rst),
    .rx_present   (rx_present),
    .retrain_req  (retrain_req),
    .rx_ts_valid  (ts_valid),
    .rx_ts_is_ts2 (ts_is_ts2),
    .rx_link      (ts_link),
    .rx_link_pad  (ts_link_pad),
    .rx_lane      (ts_lane),
    .rx_lane_pad  (ts_lane_pad),
    .rx_idle      (idle_sym),
    .rx_nonidle   (nonidle_sym),
    .ts_sent      (ts_sent),
    .ts_sent_ts2  (ts_sent_ts2),
    .idle_sent    (idle_sent),
    .tx_mode      (tx_mode),
    .tx_link      (tx_link),
    .tx_link_pad  (tx_link_pad),
    .tx_lane      (tx_lane),
    .tx_lane_pad  (tx_lane_pad),
    .state        (ltssm_state),
    .link_up      (link_up),
    .in_l0        (in_l0),
    .link_num     (link_num)
  );

  pcie_phy_tx u_tx (
    .clk         (clk),
    .rst         (rst),
    .mode        (tx_mode),
    .ts_link     (tx_link),
    .ts_link_pad (tx_link_pad),
    .ts_lane     (tx_lane),
    .ts_lane_pad (tx_lane_pad),
    .ts_sent     (ts_sent),
    .ts_sent_ts2 (ts_sent_ts2),
    .idle_sent   (idle_sent),
    .dll_valid   (dll_tx_valid),
    .dll_data    (dll_tx_data),
    .dll_sop     (dll_tx_sop),
    .dll_eop     (dll_tx_eop),
    .dll_dllp    (dll_tx_dllp),
    .dll_ready   (dll_tx_ready),
    .tx_sym      (tx_sym),
    .tx_eidle    (tx_eidle)
  );

  pcie_phy_rx u_rx (
    .clk            (clk),
    .rst            (rst),
    .rx_clk         (rx_clk),
    .rx_raw         (rx_raw),
    .ts_valid       (ts_valid),
    .ts_is_ts2      (ts_is_ts2),
    .ts_link        (ts_link),
    .ts_link_pad    (ts_link_pad),
    .ts_lane        (ts_lane),
    .ts_lane_pad    (ts_lane_pad),
    .idle_sym       (idle_sym),
    .nonidle_sym    (nonidle_sym),
    .pkt_valid      (dll_rx_valid),
    .pkt_data       (dll_rx_data),
    .pkt_sop        (dll_rx_sop),
    .pkt_eop        (dll_rx_eop),
    .pkt_dllp       (dll_rx_dllp),
    .pkt_err        (dll_rx_err),
    .pkt_edb        (dll_rx_edb),
    .rx_err_pulse   (rx_err_pulse),
    .eb_skp_added   (skp_add),
    .eb_skp_dropped (skp_drop)
  );

endmodule
