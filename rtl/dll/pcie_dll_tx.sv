`timescale 1ns/1ps
// =============================================================================
// pcie_dll_tx.sv
// -----------------------------------------------------------------------------
// Data Link Layer - transmit side.
//
// TLP path
//   * The TL offers a TLP (valid/sop..eop) plus its flow-control class and
//     data-credit count on the sop beat. It is accepted only when the link
//     partner has advertised enough credits and the replay buffer has room.
//   * Accepted TLPs are stored in the replay buffer as
//       [seq hi][seq lo][TLP bytes ...][LCRC 4 bytes]
//     and NEXT_TRANSMIT_SEQ is incremented.
//   * A separate transmit pointer streams stored TLPs to the PHY. Replay
//     (after a Nak or a REPLAY_TIMER timeout) just moves the transmit pointer
//     back to the oldest un-acknowledged TLP.
//   * Ack/Nak received from the partner purge acknowledged TLPs.
//   * REPLAY_NUM rolls over after 4 replays without progress -> retrain_req.
//
// DLLP path (priority: Ack/Nak > InitFC / UpdateFC > TLP)
//   * InitFC1 / InitFC2 sent continuously while in DL_Init.
//   * UpdateFC sent when the TL frees receive-buffer space, and periodically.
// =============================================================================
module pcie_dll_tx #(
  parameter int          RB_AW            = 11,    // replay buffer = 2**RB_AW bytes
  parameter int          RB_EW            = 3,     // up to 2**RB_EW TLPs in flight
  parameter int          MAX_TLP_BYTES    = 256,   // largest TLP accepted from the TL
  parameter int unsigned REPLAY_TIMEOUT   = 3000,  // cycles (spec: ~711 symbol times x3 for 128B/x1)
  parameter int unsigned UPDATEFC_PERIOD  = 7500   // 30 us
) (
  input  logic              clk,
  input  logic              rst,

  // ---- DL state ----
  input  logic              st_init1,
  input  logic              st_init2,
  input  logic              st_active,
  input  logic              phy_l0,

  // ---- to PHY ----
  output logic              phy_valid,
  output logic [7:0]        phy_data,
  output logic              phy_sop,
  output logic              phy_eop,
  output logic              phy_dllp,
  input  logic              phy_ready,

  // ---- TLPs from TL ----
  input  logic              tl_valid,
  input  logic [7:0]        tl_data,
  input  logic              tl_sop,
  input  logic              tl_eop,
  input  logic [1:0]        tl_fc_type,    // valid with sop
  input  logic [11:0]       tl_fc_data,    // valid with sop
  output logic              tl_ready,

  // ---- our receive credits (what we advertise) ----
  input  logic [2:0][7:0]   adv_hdr_init,  // InitFC values (0 = infinite)
  input  logic [2:0][11:0]  adv_data_init,
  input  logic [2:0][7:0]   adv_hdr_alloc, // CREDITS_ALLOCATED for UpdateFC
  input  logic [2:0][11:0]  adv_data_alloc,
  input  logic [2:0]        updfc_req,     // pulse: send UpdateFC of this type

  // ---- partner's credits (from InitFC / UpdateFC) ----
  input  logic [2:0][7:0]   lim_hdr,
  input  logic [2:0][11:0]  lim_data,
  input  logic [2:0]        lim_hdr_inf,
  input  logic [2:0]        lim_data_inf,

  // ---- Ack/Nak we must send (from our receiver) ----
  input  logic              ack_req,
  input  logic              nak_req,
  input  logic [11:0]       acknak_seq,

  // ---- Ack/Nak we received ----
  input  logic              rx_ack,
  input  logic              rx_nak,
  input  logic [11:0]       rx_acknak_seq,

  // ---- status ----
  output logic              retrain_req,
  output logic [15:0]       stat_replays,
  output logic [15:0]       stat_naks_rx
);
  import pcie_pkg::*;

  localparam int RB_SIZE = 1 << RB_AW;
  localparam int NENT    = 1 << RB_EW;

  // ===========================================================================
  // Replay buffer storage
  // ===========================================================================
  logic [7:0]       rb [RB_SIZE];
  logic [RB_AW:0]   ent_start [NENT];
  logic [RB_AW:0]   ent_len   [NENT];
  logic [11:0]      ent_seq   [NENT];

  logic [RB_EW:0]   e_wr, e_rd, e_tx;       // entry indices (with wrap bit)
  logic [RB_AW:0]   b_wr;                   // next free byte
  logic [11:0]      next_tx_seq, ackd_seq;

  logic             rb_empty;
  logic [RB_AW:0]   b_oldest, b_used;
  assign rb_empty = (e_rd == e_wr);
  assign b_oldest = rb_empty ? b_wr : ent_start[e_rd[RB_EW-1:0]];
  assign b_used   = b_wr - b_oldest;

  // ===========================================================================
  // Credit check for the TLP offered by the TL
  // ===========================================================================
  logic [2:0][7:0]  cons_hdr;
  logic [2:0][11:0] cons_data;
  logic             credit_ok;

  always_comb begin
    logic [7:0]  dh;
    logic [11:0] dd;
    dh = lim_hdr[tl_fc_type]  - (cons_hdr[tl_fc_type] + 8'd1);
    dd = lim_data[tl_fc_type] - (cons_data[tl_fc_type] + tl_fc_data);
    credit_ok = (lim_hdr_inf[tl_fc_type]  || dh <= 8'd128) &&
                (lim_data_inf[tl_fc_type] || dd <= 12'd2048);
  end

  // ===========================================================================
  // Ingest: TL -> replay buffer
  // ===========================================================================
  typedef enum logic [2:0] {IG_IDLE, IG_SEQ_LO, IG_DATA, IG_LCRC} ig_e;
  ig_e            ig;
  logic [31:0]    ig_crc;
  logic [1:0]     ig_lidx;
  logic [RB_AW:0] ig_start;
  logic [1:0]     ig_type;
  logic [11:0]    ig_dcred;

  logic can_accept;
  assign can_accept = st_active && tl_valid && tl_sop && credit_ok &&
                      (e_wr - e_rd) < (RB_EW+1)'(NENT) &&
                      (b_used + (RB_AW+1)'(MAX_TLP_BYTES + 6)) <= (RB_AW+1)'(RB_SIZE);

  assign tl_ready = (ig == IG_DATA);

  logic [31:0] lcrc_final;
  assign lcrc_final = ~ig_crc;

  always_ff @(posedge clk) begin
    if (rst) begin
      ig          <= IG_IDLE;
      ig_crc      <= 32'hFFFFFFFF;
      ig_lidx     <= 2'd0;
      ig_start    <= '0;
      ig_type     <= 2'd0;
      ig_dcred    <= 12'd0;
      b_wr        <= '0;
      e_wr        <= '0;
      next_tx_seq <= 12'd0;
      cons_hdr    <= '0;
      cons_data   <= '0;
    end else begin
      case (ig)
        IG_IDLE: if (can_accept) begin
          // write sequence number high byte
          rb[b_wr[RB_AW-1:0]] <= {4'h0, next_tx_seq[11:8]};
          ig_crc   <= crc32_byte(32'hFFFFFFFF, {4'h0, next_tx_seq[11:8]});
          ig_start <= b_wr;
          ig_type  <= tl_fc_type;
          ig_dcred <= tl_fc_data;
          b_wr     <= b_wr + 1'b1;
          ig       <= IG_SEQ_LO;
        end
        IG_SEQ_LO: begin
          rb[b_wr[RB_AW-1:0]] <= next_tx_seq[7:0];
          ig_crc <= crc32_byte(ig_crc, next_tx_seq[7:0]);
          b_wr   <= b_wr + 1'b1;
          ig     <= IG_DATA;
        end
        IG_DATA: if (tl_valid) begin
          rb[b_wr[RB_AW-1:0]] <= tl_data;
          ig_crc <= crc32_byte(ig_crc, tl_data);
          b_wr   <= b_wr + 1'b1;
          if (tl_eop) begin
            ig      <= IG_LCRC;
            ig_lidx <= 2'd0;
          end
        end
        IG_LCRC: begin
          rb[b_wr[RB_AW-1:0]] <= lcrc_final[8*ig_lidx +: 8];
          b_wr    <= b_wr + 1'b1;
          ig_lidx <= ig_lidx + 2'd1;
          if (ig_lidx == 2'd3) begin
            ent_start[e_wr[RB_EW-1:0]] <= ig_start;
            ent_len  [e_wr[RB_EW-1:0]] <= b_wr + 1'b1 - ig_start;
            ent_seq  [e_wr[RB_EW-1:0]] <= next_tx_seq;
            e_wr        <= e_wr + 1'b1;
            next_tx_seq <= next_tx_seq + 12'd1;
            cons_hdr[ig_type]  <= cons_hdr[ig_type] + 8'd1;
            cons_data[ig_type] <= cons_data[ig_type] + ig_dcred;
            ig          <= IG_IDLE;
          end
        end
        default: ig <= IG_IDLE;
      endcase
    end
  end

  // ===========================================================================
  // Pending DLLP requests
  // ===========================================================================
  logic        ack_pend, nak_pend;
  logic [11:0] an_seq;
  logic [2:0]  upd_pend;
  logic [31:0] upd_timer;
  logic [1:0]  init_rr;                     // round-robin over P / NP / Cpl

  // ===========================================================================
  // Received Ack/Nak handling + replay control
  // ===========================================================================
  logic        rxan_pend, rxan_nak;
  logic [11:0] rxan_seq;
  logic        purging, purged_any;
  logic        replay_pend;
  logic [1:0]  replay_num;
  logic [31:0] replay_timer;

  function automatic logic seq_le(input logic [11:0] a, input logic [11:0] b); // a <= b (mod 4096)
    return 12'(b - a) < 12'd2048;
  endfunction

  // ===========================================================================
  // Transmit scheduler
  // ===========================================================================
  typedef enum logic [1:0] {TX_IDLE, TX_DLLP, TX_TLP} tx_e;
  tx_e            tx;
  logic [7:0]     dl_buf [6];
  logic [2:0]     dl_idx;
  logic [RB_AW:0] t_addr, t_left;
  logic           t_first;

  logic [7:0] fc_init1 [3];
  logic [7:0] fc_init2 [3];
  logic [7:0] fc_upd   [3];
  assign fc_init1[0] = DLLP_INITFC1_P;  assign fc_init1[1] = DLLP_INITFC1_N;  assign fc_init1[2] = DLLP_INITFC1_C;
  assign fc_init2[0] = DLLP_INITFC2_P;  assign fc_init2[1] = DLLP_INITFC2_N;  assign fc_init2[2] = DLLP_INITFC2_C;
  assign fc_upd[0]   = DLLP_UPDFC_P;    assign fc_upd[1]   = DLLP_UPDFC_N;    assign fc_upd[2]   = DLLP_UPDFC_C;

  // Pick the UpdateFC to send (lowest pending type)
  logic [1:0] upd_sel;
  always_comb begin
    upd_sel = 2'd0;
    if      (upd_pend[0]) upd_sel = 2'd0;
    else if (upd_pend[1]) upd_sel = 2'd1;
    else if (upd_pend[2]) upd_sel = 2'd2;
  end

  // Next DLLP to send, in priority order (used by the scheduler below)
  logic [47:0] nd;
  always_comb begin
    if (nak_pend || ack_pend)
      nd = dllp_pack(nak_pend ? DLLP_NAK : DLLP_ACK, 8'h00, {4'h0, an_seq[11:8]}, an_seq[7:0]);
    else if (st_init1)
      nd = dllp_pack_fc(fc_init1[init_rr], adv_hdr_init[init_rr], adv_data_init[init_rr]);
    else if (st_init2)
      nd = dllp_pack_fc(fc_init2[init_rr], adv_hdr_init[init_rr], adv_data_init[init_rr]);
    else
      nd = dllp_pack_fc(fc_upd[upd_sel], adv_hdr_alloc[upd_sel], adv_data_alloc[upd_sel]);
  end

  assign phy_valid = (tx != TX_IDLE);
  assign phy_dllp  = (tx == TX_DLLP);
  assign phy_data  = (tx == TX_DLLP) ? dl_buf[dl_idx] : rb[t_addr[RB_AW-1:0]];
  assign phy_sop   = (tx == TX_DLLP) ? (dl_idx == 3'd0) : t_first;
  assign phy_eop   = (tx == TX_DLLP) ? (dl_idx == 3'd5) : (t_left == (RB_AW+1)'(1));

  always_ff @(posedge clk) begin
    if (rst) begin
      tx           <= TX_IDLE;
      dl_idx       <= 3'd0;
      t_addr       <= '0;
      t_left       <= '0;
      t_first      <= 1'b0;
      e_tx         <= '0;
      e_rd         <= '0;
      ackd_seq     <= 12'hFFF;
      ack_pend     <= 1'b0;
      nak_pend     <= 1'b0;
      an_seq       <= 12'hFFF;
      upd_pend     <= 3'b000;
      upd_timer    <= '0;
      init_rr      <= 2'd0;
      rxan_pend    <= 1'b0;
      rxan_nak     <= 1'b0;
      rxan_seq     <= 12'd0;
      purging      <= 1'b0;
      purged_any   <= 1'b0;
      replay_pend  <= 1'b0;
      replay_num   <= 2'd0;
      replay_timer <= '0;
      retrain_req  <= 1'b0;
      stat_replays <= 16'h0;
      stat_naks_rx <= 16'h0;
      for (int i = 0; i < 6; i++) dl_buf[i] <= 8'h00;
    end else begin
      retrain_req <= 1'b0;

      // ---- latch Ack/Nak requests from our receiver ----
      if (nak_req) begin
        nak_pend <= 1'b1;
        ack_pend <= 1'b0;
        an_seq   <= acknak_seq;
      end else if (ack_req) begin
        if (!nak_pend) ack_pend <= 1'b1;
        an_seq <= acknak_seq;
      end

      // ---- UpdateFC requests ----
      upd_pend <= upd_pend | (st_active ? updfc_req : 3'b000);
      if (st_active) begin
        if (upd_timer >= UPDATEFC_PERIOD) begin
          upd_timer <= '0;
          for (int i = 0; i < 3; i++)
            if (adv_hdr_init[i] != 8'd0 || adv_data_init[i] != 12'd0) upd_pend[i] <= 1'b1;
        end else begin
          upd_timer <= upd_timer + 32'd1;
        end
      end

      // ---- latch Ack/Nak received from the partner ----
      if (st_active && (rx_ack || rx_nak)) begin
        // Accept only if ACKD_SEQ <= seq <= NEXT_TRANSMIT_SEQ-1
        if (12'(next_tx_seq - 12'd1 - rx_acknak_seq) <= 12'(next_tx_seq - 12'd1 - ackd_seq)) begin
          rxan_pend <= 1'b1;
          rxan_nak  <= rx_nak | (rxan_pend & rxan_nak);
          rxan_seq  <= rx_acknak_seq;
        end
        if (rx_nak) stat_naks_rx <= stat_naks_rx + 16'd1;
      end

      // ---- replay timer ----
      if (rb_empty || !phy_l0) begin
        if (rb_empty) replay_timer <= '0;
      end else if (replay_timer >= REPLAY_TIMEOUT) begin
        replay_timer <= '0;
        replay_pend  <= 1'b1;
      end else begin
        replay_timer <= replay_timer + 32'd1;
      end

      // ---- transmit state machine ----
      case (tx)
        TX_IDLE: begin
          if (purging) begin
            // Purge one acknowledged entry per cycle
            if (!rb_empty && seq_le(ent_seq[e_rd[RB_EW-1:0]], rxan_seq)) begin
              if (e_tx == e_rd) e_tx <= e_rd + 1'b1;
              e_rd       <= e_rd + 1'b1;
              purged_any <= 1'b1;
            end else begin
              purging <= 1'b0;
              if (purged_any) begin
                ackd_seq     <= rxan_seq;
                replay_num   <= 2'd0;
                replay_timer <= '0;
              end
              if (rxan_nak) replay_pend <= 1'b1;
            end
          end else if (rxan_pend) begin
            rxan_pend  <= 1'b0;
            purging    <= 1'b1;
            purged_any <= 1'b0;
            if (rxan_nak) ackd_seq <= rxan_seq;
          end else if (replay_pend) begin
            replay_pend  <= 1'b0;
            replay_timer <= '0;
            if (!rb_empty) begin
              e_tx         <= e_rd;
              stat_replays <= stat_replays + 16'd1;
              if (replay_num == 2'd3) begin
                replay_num  <= 2'd0;
                retrain_req <= 1'b1;
              end else begin
                replay_num <= replay_num + 2'd1;
              end
            end
          end else if (nak_pend || ack_pend) begin
            for (int i = 0; i < 6; i++) dl_buf[i] <= nd[8*i +: 8];
            ack_pend <= 1'b0;
            nak_pend <= 1'b0;
            dl_idx   <= 3'd0;
            tx       <= TX_DLLP;
          end else if (st_init1 || st_init2) begin
            for (int i = 0; i < 6; i++) dl_buf[i] <= nd[8*i +: 8];
            init_rr <= (init_rr == 2'd2) ? 2'd0 : init_rr + 2'd1;
            dl_idx  <= 3'd0;
            tx      <= TX_DLLP;
          end else if (st_active && upd_pend != 3'b000) begin
            for (int i = 0; i < 6; i++) dl_buf[i] <= nd[8*i +: 8];
            upd_pend[upd_sel] <= 1'b0;
            dl_idx <= 3'd0;
            tx     <= TX_DLLP;
          end else if (st_active && e_tx != e_wr) begin
            t_addr  <= ent_start[e_tx[RB_EW-1:0]];
            t_left  <= ent_len[e_tx[RB_EW-1:0]];
            t_first <= 1'b1;
            e_tx    <= e_tx + 1'b1;
            tx      <= TX_TLP;
          end
        end

        TX_DLLP: if (phy_ready) begin
          if (dl_idx == 3'd5) tx <= TX_IDLE;
          dl_idx <= dl_idx + 3'd1;
        end

        TX_TLP: if (phy_ready) begin
          t_first <= 1'b0;
          t_addr  <= t_addr + 1'b1;
          t_left  <= t_left - 1'b1;
          if (t_left == (RB_AW+1)'(1)) begin
            tx <= TX_IDLE;
            // timer (re)starts when a TLP is sent and it is not already running
          end
        end

        default: tx <= TX_IDLE;
      endcase
    end
  end

endmodule
