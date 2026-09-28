`timescale 1ns/1ps
// =============================================================================
// pcie_phy_tx.sv
// -----------------------------------------------------------------------------
// Physical-layer transmit path (logical sub-block), Gen1 x1.
//
// Every clock it picks one symbol, in this priority order:
//   1. continue the ordered set being sent (SKP or TS1/TS2)
//   2. continue the DLL packet being sent (payload bytes, then END)
//   3. at a boundary: SKP ordered set if one is due
//   4. at a boundary: new TS1/TS2 if the LTSSM asks for training sets
//   5. at a boundary: STP/SDP + new DLL packet (L0 only)
//   6. logical idle (D0.0), scrambled
// then scrambles it and 8b/10b-encodes it.
//
// DLL interface rules: once the DLL presents sop it must keep valid high for
// every byte through eop (the physical layer cannot stall inside a packet).
// Pipeline latency: 2 clocks (scramble register + encoder register).
// =============================================================================
module pcie_phy_tx (
  input  logic                 clk,
  input  logic                 rst,

  // ---- from LTSSM ----
  input  pcie_pkg::tx_mode_e   mode,
  input  logic [7:0]           ts_link,
  input  logic                 ts_link_pad,
  input  logic [7:0]           ts_lane,
  input  logic                 ts_lane_pad,
  output logic                 ts_sent,       // pulse: last symbol of a TS1/TS2
  output logic                 ts_sent_ts2,   // qualifies ts_sent
  output logic                 idle_sent,     // pulse: one logical idle symbol

  // ---- from DLL ----
  input  logic                 dll_valid,
  input  logic [7:0]           dll_data,
  input  logic                 dll_sop,
  input  logic                 dll_eop,
  input  logic                 dll_dllp,      // 1 = frame with SDP, 0 = STP
  output logic                 dll_ready,

  // ---- to serializer ----
  output logic [9:0]           tx_sym,
  output logic                 tx_eidle
);
  import pcie_pkg::*;

  typedef enum logic [1:0] {OS_NONE, OS_SKP, OS_TS} os_e;
  typedef enum logic [1:0] {PK_IDLE, PK_DATA, PK_END} pk_e;

  os_e         os_st;
  pk_e         pk_st;
  logic [3:0]  os_idx;
  logic        os_ts2, os_lpad, os_npad;
  logic [7:0]  os_link, os_lane;
  logic [11:0] skp_cnt;
  logic        skp_due;

  // Selected symbol (before scrambling)
  logic        sel_k, sel_noscr, sel_eidle;
  logic [7:0]  sel_d;

  // Next-state helpers
  logic        start_skp, start_ts, start_pkt, send_idle;

  always_comb begin
    sel_k     = 1'b0;
    sel_d     = IDLE_D;
    sel_noscr = 1'b0;
    sel_eidle = 1'b0;
    dll_ready = 1'b0;
    start_skp = 1'b0;
    start_ts  = 1'b0;
    start_pkt = 1'b0;
    send_idle = 1'b0;

    if (mode == TXM_EIDLE) begin
      sel_eidle = 1'b1;
    end else if (os_st == OS_SKP) begin
      sel_k = 1'b1;
      sel_d = K_SKP;
    end else if (os_st == OS_TS) begin
      sel_noscr = 1'b1;
      case (os_idx)
        4'd1:    begin sel_k = os_lpad; sel_d = os_lpad ? K_PAD : os_link; end
        4'd2:    begin sel_k = os_npad; sel_d = os_npad ? K_PAD : os_lane; end
        4'd3:    sel_d = TS_N_FTS;
        4'd4:    sel_d = TS_RATE_ID;
        4'd5:    sel_d = TS_TRN_CTL;
        default: sel_d = os_ts2 ? TS2_ID : TS1_ID;
      endcase
    end else if (pk_st == PK_DATA) begin
      dll_ready = 1'b1;
      sel_d     = dll_data;
    end else if (pk_st == PK_END) begin
      sel_k = 1'b1;
      sel_d = K_END;
    end else if (skp_due) begin
      start_skp = 1'b1;
      sel_k     = 1'b1;
      sel_d     = K_COM;
    end else if (mode == TXM_TS1 || mode == TXM_TS2) begin
      start_ts = 1'b1;
      sel_k    = 1'b1;
      sel_d    = K_COM;
    end else if (mode == TXM_L0 && dll_valid && dll_sop) begin
      start_pkt = 1'b1;
      sel_k     = 1'b1;
      sel_d     = dll_dllp ? K_SDP : K_STP;
    end else begin
      send_idle = 1'b1;                         // logical idle
    end
  end

  // ---- control state ----
  always_ff @(posedge clk) begin
    if (rst || mode == TXM_EIDLE) begin
      os_st       <= OS_NONE;
      pk_st       <= PK_IDLE;
      os_idx      <= 4'd0;
      os_ts2      <= 1'b0;
      os_lpad     <= 1'b1;
      os_npad     <= 1'b1;
      os_link     <= 8'h00;
      os_lane     <= 8'h00;
      skp_cnt     <= 12'd0;
      skp_due     <= 1'b0;
      ts_sent     <= 1'b0;
      ts_sent_ts2 <= 1'b0;
      idle_sent   <= 1'b0;
    end else begin
      ts_sent   <= 1'b0;
      idle_sent <= send_idle;

      // SKP scheduling
      if (start_skp) begin
        skp_cnt <= 12'd0;
        skp_due <= 1'b0;
      end else begin
        skp_cnt <= skp_cnt + 12'd1;
        if (skp_cnt >= 12'(SKP_INTERVAL)) skp_due <= 1'b1;
      end

      // Ordered sets
      case (os_st)
        OS_SKP: begin
          if (os_idx == 4'd3) os_st <= OS_NONE;
          os_idx <= os_idx + 4'd1;
        end
        OS_TS: begin
          if (os_idx == 4'd15) begin
            os_st       <= OS_NONE;
            ts_sent     <= 1'b1;
            ts_sent_ts2 <= os_ts2;
          end
          os_idx <= os_idx + 4'd1;
        end
        default: begin
          if (start_skp) begin
            os_st  <= OS_SKP;
            os_idx <= 4'd1;
          end else if (start_ts) begin
            os_st   <= OS_TS;
            os_idx  <= 4'd1;
            os_ts2  <= (mode == TXM_TS2);
            os_link <= ts_link;
            os_lpad <= ts_link_pad;
            os_lane <= ts_lane;
            os_npad <= ts_lane_pad;
          end
        end
      endcase

      // Packets
      case (pk_st)
        PK_IDLE: if (start_pkt) pk_st <= PK_DATA;
        PK_DATA: if (dll_valid && dll_eop) pk_st <= PK_END;
        default: pk_st <= PK_IDLE;             // PK_END
      endcase
    end
  end

  // ---- scrambler (stage 1) ----
  logic [15:0] lfsr;
  logic        s1_k, s1_eidle;
  logic [7:0]  s1_d;

  always_ff @(posedge clk) begin
    if (rst) begin
      lfsr     <= 16'hFFFF;
      s1_k     <= 1'b0;
      s1_d     <= 8'h00;
      s1_eidle <= 1'b1;
    end else begin
      s1_eidle <= sel_eidle;
      s1_k     <= sel_k;
      if (sel_eidle) begin
        lfsr <= 16'hFFFF;
        s1_d <= 8'h00;
      end else begin
        s1_d <= (!sel_k && !sel_noscr) ? (sel_d ^ lfsr_mask(lfsr)) : sel_d;
        if (sel_k && sel_d == K_COM)      lfsr <= 16'hFFFF;
        else if (sel_k && sel_d == K_SKP) lfsr <= lfsr;
        else                              lfsr <= lfsr_adv8(lfsr);
      end
    end
  end

  // ---- 8b/10b encoder (stage 2) ----
  logic enc_kerr;

  pcie_8b10b_enc u_enc (
    .clk  (clk),
    .rst  (rst),
    .en   (1'b1),
    .din  (s1_d),
    .kin  (s1_k),
    .dout (tx_sym),
    .kerr (enc_kerr)
  );

  always_ff @(posedge clk) begin
    if (rst) tx_eidle <= 1'b1;
    else     tx_eidle <= s1_eidle;
  end

endmodule
