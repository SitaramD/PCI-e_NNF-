`timescale 1ns/1ps
// =============================================================================
// pcie_dll_rx.sv
// -----------------------------------------------------------------------------
// Data Link Layer - receive side.
//
// DLLPs (from SDP ... END):
//   6 bytes = 4 bytes of content + 16-bit CRC. Good DLLPs are decoded and
//   handed to pcie_dll_top (Ack/Nak -> transmitter, FC -> credit logic).
//   Bad DLLPs are silently dropped (spec: "Bad DLLP" error).
//
// TLPs (from STP ... END):
//   [seq hi][seq lo][ TLP header + data ... ][LCRC0][LCRC1][LCRC2][LCRC3]
//   A 4-byte delay line holds back the last four bytes (the LCRC candidate).
//   Bytes leaving the delay line are fed into the CRC and written to the
//   receive buffer at a *tentative* write pointer. At END:
//     good LCRC & seq == NEXT_RCV_SEQ  -> commit, NEXT_RCV_SEQ++, send Ack
//     good LCRC & seq already received -> discard (duplicate), send Ack
//     bad LCRC / framing error / future seq -> discard, send Nak (once)
//     ended with EDB (nullified)        -> discard silently
//
// Receive buffer read port goes to the Transaction Layer. When the TL finishes
// reading a TLP, its credits are released (rel_*) so the DLL can send UpdateFC.
// =============================================================================
module pcie_dll_rx #(
  parameter int AW = 11                 // receive buffer = 2**AW bytes
) (
  input  logic        clk,
  input  logic        rst,              // also asserted while DL is inactive
  input  logic        tlp_enable,       // accept TLPs (DL_Init2 / DL_Active)

  // ---- from PHY ----
  input  logic        phy_valid,
  input  logic [7:0]  phy_data,
  input  logic        phy_sop,
  input  logic        phy_eop,
  input  logic        phy_dllp,
  input  logic        phy_err,
  input  logic        phy_edb,

  // ---- decoded DLLP (1-cycle pulse) ----
  output logic        dllp_valid,
  output logic [7:0]  dllp_type,        // byte 0 of the DLLP
  output logic [11:0] dllp_seq,         // Ack/Nak sequence number
  output logic [7:0]  dllp_hdr_fc,      // FC DLLPs
  output logic [11:0] dllp_data_fc,

  // ---- Ack/Nak requests to our transmitter ----
  output logic        ack_req,
  output logic        nak_req,
  output logic [11:0] acknak_seq,
  output logic        tlp_received,     // pulse: any TLP ended (for DL_Init2 -> Active)

  // ---- receive buffer read port (to TL) ----
  output logic        tl_valid,
  output logic [7:0]  tl_data,
  output logic        tl_sop,
  output logic        tl_eop,
  input  logic        tl_ready,

  // ---- credit release (1-cycle pulse when the TL has read a whole TLP) ----
  output logic        rel_valid,
  output logic [1:0]  rel_type,
  output logic [11:0] rel_data,

  // ---- statistics ----
  output logic [15:0] stat_bad_tlp,
  output logic [15:0] stat_bad_dllp,
  output logic [15:0] stat_dup_tlp
);
  import pcie_pkg::*;

  // ===========================================================================
  // DLLP receive
  // ===========================================================================
  logic [2:0]  d_cnt;
  logic [7:0]  d_b [4];
  logic [7:0]  d_crc_lo;
  logic [15:0] d_crc;
  logic        d_bad;

  always_ff @(posedge clk) begin
    if (rst) begin
      d_cnt        <= 3'd0;
      d_crc        <= 16'hFFFF;
      d_bad        <= 1'b0;
      d_crc_lo     <= 8'h00;
      dllp_valid   <= 1'b0;
      dllp_type    <= 8'h00;
      dllp_seq     <= 12'h0;
      dllp_hdr_fc  <= 8'h00;
      dllp_data_fc <= 12'h0;
      stat_bad_dllp <= 16'h0;
      for (int i = 0; i < 4; i++) d_b[i] <= 8'h00;
    end else begin
      dllp_valid <= 1'b0;
      if (phy_valid && phy_dllp) begin
        logic [2:0]  cnt;
        logic [15:0] crc;
        logic        bad;
        cnt = phy_sop ? 3'd0 : d_cnt;
        crc = phy_sop ? 16'hFFFF : d_crc;
        bad = phy_sop ? 1'b0 : d_bad;
        if (cnt < 3'd4) begin
          d_b[cnt[1:0]] <= phy_data;
          crc = crc16_byte(crc, phy_data);
        end else if (cnt == 3'd4) begin
          d_crc_lo <= phy_data;
        end
        if (cnt == 3'd7) bad = 1'b1;           // too long
        d_cnt <= (cnt == 3'd7) ? cnt : cnt + 3'd1;
        d_crc <= crc;
        d_bad <= bad;
        if (phy_eop) begin
          if (!bad && !phy_err && cnt == 3'd5 && {phy_data, d_crc_lo} == ~crc) begin
            dllp_valid   <= 1'b1;
            dllp_type    <= d_b[0];
            dllp_seq     <= {d_b[2][3:0], d_b[3]};
            dllp_hdr_fc  <= {d_b[1][5:0], d_b[2][7:6]};
            dllp_data_fc <= {d_b[2][3:0], d_b[3]};
          end else begin
            stat_bad_dllp <= stat_bad_dllp + 16'd1;
          end
        end
      end
    end
  end

  // ===========================================================================
  // TLP receive
  // ===========================================================================
  localparam int DEPTH = 1 << AW;

  logic [9:0]  rbuf [DEPTH];            // {sop, eop, data}
  logic [AW:0] wptr_c, wptr_t, rptr;    // committed / tentative write, read
  logic [7:0]  dly [4];                 // LCRC candidate delay line (dly[0] oldest)
  logic [15:0] t_cnt;                   // bytes received in this TLP
  logic [31:0] t_crc;
  logic [7:0]  t_seq_hi, t_seq_lo;
  logic        t_bad;
  logic [11:0] next_rcv_seq;
  logic        nak_scheduled;

  always_ff @(posedge clk) begin
    if (rst) begin
      wptr_c        <= '0;
      wptr_t        <= '0;
      t_cnt         <= 16'd0;
      t_crc         <= 32'hFFFFFFFF;
      t_seq_hi      <= 8'h00;
      t_seq_lo      <= 8'h00;
      t_bad         <= 1'b0;
      next_rcv_seq  <= 12'd0;
      nak_scheduled <= 1'b0;
      ack_req       <= 1'b0;
      nak_req       <= 1'b0;
      acknak_seq    <= 12'hFFF;
      tlp_received  <= 1'b0;
      stat_bad_tlp  <= 16'h0;
      stat_dup_tlp  <= 16'h0;
      for (int i = 0; i < 4; i++) dly[i] <= 8'h00;
    end else begin
      ack_req      <= 1'b0;
      nak_req      <= 1'b0;
      tlp_received <= 1'b0;

      if (phy_valid && !phy_dllp) begin
        logic [15:0] n;
        logic [31:0] crc;
        logic        bad;
        logic [AW:0] wp;
        logic [7:0]  ob;          // byte leaving the delay line
        logic [7:0]  shi, slo;
        logic [11:0] seq;

        n   = phy_sop ? 16'd0 : t_cnt;
        crc = phy_sop ? 32'hFFFFFFFF : t_crc;
        bad = phy_sop ? 1'b0 : t_bad;
        wp  = phy_sop ? wptr_c : wptr_t;
        shi = t_seq_hi;
        slo = t_seq_lo;
        ob  = dly[0];

        // Byte number n-4 leaves the delay line
        if (n >= 16'd4) begin
          crc = crc32_byte(crc, ob);
          if (n == 16'd4) shi = ob;
          else if (n == 16'd5) slo = ob;
          else begin
            // TLP byte (index n-4 >= 2) goes to the receive buffer
            if (wp - rptr >= (AW+1)'(DEPTH)) begin
              bad = 1'b1;                       // buffer overflow (credit violation)
            end else begin
              rbuf[wp[AW-1:0]] <= {(n == 16'd6), phy_eop, ob};
              wp = wp + 1'b1;
            end
          end
        end
        dly[0] <= dly[1];
        dly[1] <= dly[2];
        dly[2] <= dly[3];
        dly[3] <= phy_data;

        t_cnt    <= (n == 16'hFFFF) ? n : n + 16'd1;
        t_crc    <= crc;
        t_bad    <= bad;
        wptr_t   <= wp;
        t_seq_hi <= shi;
        t_seq_lo <= slo;

        if (phy_eop) begin
          logic good_crc;
          // LCRC bytes are dly[1], dly[2], dly[3] and the current byte
          good_crc = ({phy_data, dly[3], dly[2], dly[1]} == ~crc);
          seq      = {shi[3:0], slo};
          tlp_received <= 1'b1;

          if (phy_edb) begin
            // Nullified TLP: discard, no Nak
            wptr_t <= wptr_c;
          end else if (!tlp_enable) begin
            wptr_t <= wptr_c;
          end else if (bad || phy_err || !good_crc || n < 16'd17) begin
            // Bad TLP: discard, Nak once
            wptr_t       <= wptr_c;
            stat_bad_tlp <= stat_bad_tlp + 16'd1;
            if (!nak_scheduled) begin
              nak_req       <= 1'b1;
              acknak_seq    <= next_rcv_seq - 12'd1;
              nak_scheduled <= 1'b1;
            end
          end else if (seq == next_rcv_seq) begin
            // Good, in-order TLP: commit
            wptr_c        <= wp;
            wptr_t        <= wp;
            next_rcv_seq  <= next_rcv_seq + 12'd1;
            nak_scheduled <= 1'b0;
            ack_req       <= 1'b1;
            acknak_seq    <= seq;
          end else if (12'(next_rcv_seq - 12'd1 - seq) < 12'd2048) begin
            // Duplicate (already received): discard, Ack
            wptr_t       <= wptr_c;
            ack_req      <= 1'b1;
            acknak_seq   <= next_rcv_seq - 12'd1;
            stat_dup_tlp <= stat_dup_tlp + 16'd1;
          end else begin
            // Sequence number from the future: we lost a TLP
            wptr_t <= wptr_c;
            if (!nak_scheduled) begin
              nak_req       <= 1'b1;
              acknak_seq    <= next_rcv_seq - 12'd1;
              nak_scheduled <= 1'b1;
            end
          end
        end
      end
    end
  end

  // ===========================================================================
  // Read port + credit release
  // ===========================================================================
  logic [7:0] r_fmt;
  logic [2:0] r_idx;       // byte index within the header (saturates at 4)
  logic [9:0] r_len;

  assign tl_valid = (rptr != wptr_c);
  assign tl_data  = rbuf[rptr[AW-1:0]][7:0];
  assign tl_sop   = rbuf[rptr[AW-1:0]][9];
  assign tl_eop   = rbuf[rptr[AW-1:0]][8];

  always_ff @(posedge clk) begin
    if (rst) begin
      rptr      <= '0;
      r_fmt     <= 8'h00;
      r_idx     <= 3'd0;
      r_len     <= 10'd0;
      rel_valid <= 1'b0;
      rel_type  <= 2'd0;
      rel_data  <= 12'd0;
    end else begin
      rel_valid <= 1'b0;
      if (tl_valid && tl_ready) begin
        logic [7:0] fmt;
        logic [9:0] len;
        logic [2:0] idx;
        rptr <= rptr + 1'b1;
        fmt = tl_sop ? tl_data : r_fmt;
        idx = tl_sop ? 3'd0 : r_idx;
        len = r_len;
        if (idx == 3'd2) len[9:8] = tl_data[1:0];
        if (idx == 3'd3) len[7:0] = tl_data;
        r_fmt <= fmt;
        r_len <= len;
        r_idx <= (idx == 3'd4) ? idx : idx + 3'd1;
        if (tl_eop) begin
          rel_valid <= 1'b1;
          rel_type  <= tlp_fc_class(fmt);
          rel_data  <= tlp_data_credits(fmt[6], len);
        end
      end
    end
  end

endmodule
