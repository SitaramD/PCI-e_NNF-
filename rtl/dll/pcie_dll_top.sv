`timescale 1ns/1ps
// =============================================================================
// pcie_dll_top.sv
// -----------------------------------------------------------------------------
// Data Link Layer = DL control state machine + flow-control bookkeeping +
// pcie_dll_tx + pcie_dll_rx.
//
// DL control state machine (VC0 only):
//   DL_Inactive : physical LinkUp = 0. Everything is held in reset.
//   DL_Init1    : send InitFC1-P/NP/Cpl. Record the partner's InitFC1/InitFC2
//                 values; when all three types are recorded -> DL_Init2.
//   DL_Init2    : send InitFC2-P/NP/Cpl. On receiving any InitFC2, UpdateFC
//                 or TLP -> DL_Active.
//   DL_Active   : TLPs flow. dl_up = 1.
//
// Credits we advertise (receive side) are parameters. A value of 0 means
// "infinite" (required for completions at an endpoint that never waits on
// its own completion buffer).
// =============================================================================
module pcie_dll_top #(
  parameter int          RX_AW           = 11,
  parameter int          RB_AW           = 11,
  parameter int          RB_EW           = 3,
  parameter int unsigned REPLAY_TIMEOUT  = 3000,
  parameter int unsigned UPDATEFC_PERIOD = 7500,
  // Advertised credits: header (1 per TLP) and data (1 per 16 bytes)
  parameter logic [7:0]  ADV_PH   = 8'd8,
  parameter logic [11:0] ADV_PD   = 12'd64,
  parameter logic [7:0]  ADV_NPH  = 8'd8,
  parameter logic [11:0] ADV_NPD  = 12'd8,
  parameter logic [7:0]  ADV_CPLH = 8'd0,    // infinite
  parameter logic [11:0] ADV_CPLD = 12'd0    // infinite
) (
  input  logic        clk,
  input  logic        rst,

  // ---- PHY status ----
  input  logic        phy_link_up,
  input  logic        phy_in_l0,

  // ---- to PHY ----
  output logic        phy_tx_valid,
  output logic [7:0]  phy_tx_data,
  output logic        phy_tx_sop,
  output logic        phy_tx_eop,
  output logic        phy_tx_dllp,
  input  logic        phy_tx_ready,

  // ---- from PHY ----
  input  logic        phy_rx_valid,
  input  logic [7:0]  phy_rx_data,
  input  logic        phy_rx_sop,
  input  logic        phy_rx_eop,
  input  logic        phy_rx_dllp,
  input  logic        phy_rx_err,
  input  logic        phy_rx_edb,

  // ---- TL transmit ----
  input  logic        tl_tx_valid,
  input  logic [7:0]  tl_tx_data,
  input  logic        tl_tx_sop,
  input  logic        tl_tx_eop,
  input  logic [1:0]  tl_tx_fc_type,
  input  logic [11:0] tl_tx_fc_data,
  output logic        tl_tx_ready,

  // ---- TL receive ----
  output logic        tl_rx_valid,
  output logic [7:0]  tl_rx_data,
  output logic        tl_rx_sop,
  output logic        tl_rx_eop,
  input  logic        tl_rx_ready,

  // ---- status ----
  output logic        dl_up,
  output logic [1:0]  dl_state,          // 0 inactive, 1 init1, 2 init2, 3 active
  output logic        retrain_req,
  output logic [15:0] stat_bad_tlp,
  output logic [15:0] stat_bad_dllp,
  output logic [15:0] stat_dup_tlp,
  output logic [15:0] stat_replays,
  output logic [15:0] stat_naks_rx
);
  import pcie_pkg::*;

  localparam logic [1:0] DL_INACTIVE = 2'd0, DL_INIT1 = 2'd1, DL_INIT2 = 2'd2, DL_ACTIVE = 2'd3;

  logic dl_rst;
  assign dl_rst = rst || !phy_link_up;

  // ---------------------------------------------------------------------------
  // Receive side
  // ---------------------------------------------------------------------------
  logic        dllp_valid;
  logic [7:0]  dllp_type, dllp_hdr_fc;
  logic [11:0] dllp_seq, dllp_data_fc;
  logic        ack_req, nak_req, tlp_received;
  logic [11:0] acknak_seq;
  logic        rel_valid;
  logic [1:0]  rel_type;
  logic [11:0] rel_data;

  pcie_dll_rx #(.AW(RX_AW)) u_rx (
    .clk           (clk),
    .rst           (dl_rst),
    .tlp_enable    (dl_state == DL_INIT2 || dl_state == DL_ACTIVE),
    .phy_valid     (phy_rx_valid),
    .phy_data      (phy_rx_data),
    .phy_sop       (phy_rx_sop),
    .phy_eop       (phy_rx_eop),
    .phy_dllp      (phy_rx_dllp),
    .phy_err       (phy_rx_err),
    .phy_edb       (phy_rx_edb),
    .dllp_valid    (dllp_valid),
    .dllp_type     (dllp_type),
    .dllp_seq      (dllp_seq),
    .dllp_hdr_fc   (dllp_hdr_fc),
    .dllp_data_fc  (dllp_data_fc),
    .ack_req       (ack_req),
    .nak_req       (nak_req),
    .acknak_seq    (acknak_seq),
    .tlp_received  (tlp_received),
    .tl_valid      (tl_rx_valid),
    .tl_data       (tl_rx_data),
    .tl_sop        (tl_rx_sop),
    .tl_eop        (tl_rx_eop),
    .tl_ready      (tl_rx_ready),
    .rel_valid     (rel_valid),
    .rel_type      (rel_type),
    .rel_data      (rel_data),
    .stat_bad_tlp  (stat_bad_tlp),
    .stat_bad_dllp (stat_bad_dllp),
    .stat_dup_tlp  (stat_dup_tlp)
  );

  // ---------------------------------------------------------------------------
  // DLLP classification
  // ---------------------------------------------------------------------------
  logic [7:0] dtype;
  logic       is_ack, is_nak, is_init1, is_init2, is_upd;
  logic [1:0] fc_t;
  assign dtype    = {dllp_type[7:3], 3'b000};         // strip VC
  assign is_ack   = dllp_valid && dllp_type == DLLP_ACK;
  assign is_nak   = dllp_valid && dllp_type == DLLP_NAK;
  assign is_init1 = dllp_valid && (dtype == DLLP_INITFC1_P || dtype == DLLP_INITFC1_N || dtype == DLLP_INITFC1_C);
  assign is_init2 = dllp_valid && (dtype == DLLP_INITFC2_P || dtype == DLLP_INITFC2_N || dtype == DLLP_INITFC2_C);
  assign is_upd   = dllp_valid && (dtype == DLLP_UPDFC_P   || dtype == DLLP_UPDFC_N   || dtype == DLLP_UPDFC_C);
  assign fc_t     = dtype[5:4];                       // 00 P, 01 NP, 10 Cpl (same for Init/Update)

  // ---------------------------------------------------------------------------
  // DL control state machine + partner credit limits
  // ---------------------------------------------------------------------------
  logic [2:0]       fi1_got;
  logic [2:0][7:0]  lim_hdr;
  logic [2:0][11:0] lim_data;
  logic [2:0]       lim_hdr_inf, lim_data_inf;

  always_ff @(posedge clk) begin
    if (dl_rst) begin
      dl_state     <= DL_INACTIVE;
      fi1_got      <= 3'b000;
      lim_hdr      <= '0;
      lim_data     <= '0;
      lim_hdr_inf  <= 3'b000;
      lim_data_inf <= 3'b000;
    end else begin
      case (dl_state)
        DL_INACTIVE: dl_state <= DL_INIT1;
        DL_INIT1: begin
          if ((is_init1 || is_init2) && !fi1_got[fc_t]) begin
            fi1_got[fc_t]      <= 1'b1;
            lim_hdr[fc_t]      <= dllp_hdr_fc;
            lim_data[fc_t]     <= dllp_data_fc;
            lim_hdr_inf[fc_t]  <= (dllp_hdr_fc == 8'd0);
            lim_data_inf[fc_t] <= (dllp_data_fc == 12'd0);
          end
          if (fi1_got == 3'b111) dl_state <= DL_INIT2;
        end
        DL_INIT2: begin
          if (is_init2 || is_upd || tlp_received) dl_state <= DL_ACTIVE;
        end
        default: ;  // DL_ACTIVE
      endcase

      // UpdateFC: new credit limit (ignored for infinite types)
      if (is_upd && (dl_state == DL_ACTIVE || dl_state == DL_INIT2)) begin
        if (!lim_hdr_inf[fc_t])  lim_hdr[fc_t]  <= dllp_hdr_fc;
        if (!lim_data_inf[fc_t]) lim_data[fc_t] <= dllp_data_fc;
      end
    end
  end

  assign dl_up = (dl_state == DL_ACTIVE);

  // ---------------------------------------------------------------------------
  // Our receive credits: CREDITS_ALLOCATED grows as the TL drains the buffer
  // ---------------------------------------------------------------------------
  logic [2:0][7:0]  adv_hdr_init, adv_hdr_alloc;
  logic [2:0][11:0] adv_data_init, adv_data_alloc;
  logic [2:0]       updfc_req;

  assign adv_hdr_init[0]  = ADV_PH;    assign adv_data_init[0] = ADV_PD;
  assign adv_hdr_init[1]  = ADV_NPH;   assign adv_data_init[1] = ADV_NPD;
  assign adv_hdr_init[2]  = ADV_CPLH;  assign adv_data_init[2] = ADV_CPLD;

  always_ff @(posedge clk) begin
    if (dl_rst) begin
      adv_hdr_alloc  <= adv_hdr_init;
      adv_data_alloc <= adv_data_init;
      updfc_req      <= 3'b000;
    end else begin
      updfc_req <= 3'b000;
      if (rel_valid) begin
        if (adv_hdr_init[rel_type] != 8'd0)
          adv_hdr_alloc[rel_type]  <= adv_hdr_alloc[rel_type] + 8'd1;
        if (adv_data_init[rel_type] != 12'd0)
          adv_data_alloc[rel_type] <= adv_data_alloc[rel_type] + rel_data;
        if (adv_hdr_init[rel_type] != 8'd0 || adv_data_init[rel_type] != 12'd0)
          updfc_req[rel_type] <= 1'b1;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Transmit side
  // ---------------------------------------------------------------------------
  pcie_dll_tx #(
    .RB_AW           (RB_AW),
    .RB_EW           (RB_EW),
    .REPLAY_TIMEOUT  (REPLAY_TIMEOUT),
    .UPDATEFC_PERIOD (UPDATEFC_PERIOD)
  ) u_tx (
    .clk            (clk),
    .rst            (dl_rst),
    .st_init1       (dl_state == DL_INIT1),
    .st_init2       (dl_state == DL_INIT2),
    .st_active      (dl_state == DL_ACTIVE),
    .phy_l0         (phy_in_l0),
    .phy_valid      (phy_tx_valid),
    .phy_data       (phy_tx_data),
    .phy_sop        (phy_tx_sop),
    .phy_eop        (phy_tx_eop),
    .phy_dllp       (phy_tx_dllp),
    .phy_ready      (phy_tx_ready),
    .tl_valid       (tl_tx_valid),
    .tl_data        (tl_tx_data),
    .tl_sop         (tl_tx_sop),
    .tl_eop         (tl_tx_eop),
    .tl_fc_type     (tl_tx_fc_type),
    .tl_fc_data     (tl_tx_fc_data),
    .tl_ready       (tl_tx_ready),
    .adv_hdr_init   (adv_hdr_init),
    .adv_data_init  (adv_data_init),
    .adv_hdr_alloc  (adv_hdr_alloc),
    .adv_data_alloc (adv_data_alloc),
    .updfc_req      (updfc_req),
    .lim_hdr        (lim_hdr),
    .lim_data       (lim_data),
    .lim_hdr_inf    (lim_hdr_inf),
    .lim_data_inf   (lim_data_inf),
    .ack_req        (ack_req),
    .nak_req        (nak_req),
    .acknak_seq     (acknak_seq),
    .rx_ack         (is_ack),
    .rx_nak         (is_nak),
    .rx_acknak_seq  (dllp_seq),
    .retrain_req    (retrain_req),
    .stat_replays   (stat_replays),
    .stat_naks_rx   (stat_naks_rx)
  );

endmodule
