`timescale 1ns/1ps
// =============================================================================
// pcie_phy_rx.sv
// -----------------------------------------------------------------------------
// Physical-layer receive path (logical sub-block), Gen1 x1.
//
//   recovered clock domain (rx_clk):
//     raw 10-bit words -> comma alignment -> 8b/10b decode -> elastic buffer
//   core clock domain (clk):
//     -> de-scrambler -> ordered-set parser (TS1/TS2/SKP) -> packet de-framer
//
// Outputs to the LTSSM: decoded TS1/TS2 (link#, lane#), logical-idle counts.
// Outputs to the DLL  : TLP/DLLP byte stream (STP/SDP ... END stripped).
//
// De-scrambler rules (must mirror pcie_phy_tx):
//   * COM resets the LFSR to 16'hFFFF
//   * SKP does not advance the LFSR
//   * every other symbol advances it by 8 bits
//   * only D symbols outside TS1/TS2 ordered sets are de-scrambled
// =============================================================================
module pcie_phy_rx (
  input  logic       clk,
  input  logic       rst,
  input  logic       rx_clk,
  input  logic [9:0] rx_raw,

  // ---- to LTSSM ----
  output logic       ts_valid,      // pulse: a complete, well-formed TS1/TS2
  output logic       ts_is_ts2,
  output logic [7:0] ts_link,
  output logic       ts_link_pad,
  output logic [7:0] ts_lane,
  output logic       ts_lane_pad,
  output logic       idle_sym,      // pulse: logical idle (D0.0) received
  output logic       nonidle_sym,   // pulse: data symbol that is not idle, outside a packet

  // ---- to DLL ----
  output logic       pkt_valid,
  output logic [7:0] pkt_data,
  output logic       pkt_sop,
  output logic       pkt_eop,
  output logic       pkt_dllp,      // 1 = DLLP (SDP), 0 = TLP (STP)
  output logic       pkt_err,       // on eop: framing / decode error in packet
  output logic       pkt_edb,       // on eop: packet ended with EDB (nullified)

  // ---- status / debug ----
  output logic       rx_err_pulse,  // 8b/10b code or disparity error
  output logic       eb_skp_added,
  output logic       eb_skp_dropped
);
  import pcie_pkg::*;

  // ===========================================================================
  // Recovered clock domain
  // ===========================================================================
  logic [1:0] rrst_sync;
  logic       rrst;
  always_ff @(posedge rx_clk or posedge rst) begin
    if (rst) rrst_sync <= 2'b11;
    else     rrst_sync <= {rrst_sync[0], 1'b0};
  end
  assign rrst = rrst_sync[1];

  logic [9:0] al_sym;
  logic       al_valid, al_realign;

  pcie_comma_align u_align (
    .clk     (rx_clk),
    .rst     (rrst),
    .din     (rx_raw),
    .dout    (al_sym),
    .dvalid  (al_valid),
    .realign (al_realign)
  );

  logic [7:0] dec_d;
  logic       dec_k, dec_cerr, dec_derr, dec_valid;

  pcie_8b10b_dec u_dec (
    .clk      (rx_clk),
    .rst      (rrst),
    .en       (al_valid),
    .din      (al_sym),
    .dout     (dec_d),
    .kout     (dec_k),
    .code_err (dec_cerr),
    .disp_err (dec_derr)
  );

  always_ff @(posedge rx_clk) begin
    if (rrst) dec_valid <= 1'b0;
    else      dec_valid <= al_valid;
  end

  logic       eb_valid;
  logic [9:0] eb_data;
  logic       eb_ovf, eb_unf;

  pcie_elastic_buf #(.AW(5)) u_eb (
    .wclk        (rx_clk),
    .wrst        (rrst),
    .wen         (dec_valid),
    .wdata       ({dec_cerr | dec_derr, dec_k, dec_d}),
    .rclk        (clk),
    .rrst        (rst),
    .rvalid      (eb_valid),
    .rdata       (eb_data),
    .skp_added   (eb_skp_added),
    .skp_dropped (eb_skp_dropped),
    .overflow    (eb_ovf),
    .underflow   (eb_unf)
  );

  // ===========================================================================
  // Core clock domain
  // ===========================================================================
  logic       s_err, s_k;
  logic [7:0] s_d;
  assign s_err = eb_data[9];
  assign s_k   = eb_data[8];
  assign s_d   = eb_data[7:0];

  logic        is_com, is_skp;
  assign is_com = s_k && s_d == K_COM;
  assign is_skp = s_k && s_d == K_SKP;

  // ---- de-scrambler ----
  logic [15:0] lfsr;
  logic [4:0]  os_pos;       // 0 = not in an ordered set, 1..15 = symbol index after COM
  logic        in_ts_sym;    // current symbol belongs to a TS1/TS2 body
  logic [7:0]  ds_d;         // de-scrambled data

  assign in_ts_sym = (os_pos != 5'd0) && !(os_pos == 5'd1 && is_skp);
  assign ds_d      = (!s_k && !in_ts_sym) ? (s_d ^ lfsr_mask(lfsr)) : s_d;

  always_ff @(posedge clk) begin
    if (rst) begin
      lfsr <= 16'hFFFF;
    end else if (eb_valid) begin
      if (is_com)      lfsr <= 16'hFFFF;
      else if (is_skp) lfsr <= lfsr;
      else             lfsr <= lfsr_adv8(lfsr);
    end
  end

  // ---- ordered-set parser ----
  logic       os_ok, os_ts2;
  logic [7:0] os_id;

  always_ff @(posedge clk) begin
    if (rst) begin
      os_pos      <= 5'd0;
      os_ok       <= 1'b0;
      os_ts2      <= 1'b0;
      os_id       <= 8'h00;
      ts_valid    <= 1'b0;
      ts_is_ts2   <= 1'b0;
      ts_link     <= 8'h00;
      ts_link_pad <= 1'b1;
      ts_lane     <= 8'h00;
      ts_lane_pad <= 1'b1;
    end else begin
      ts_valid <= 1'b0;
      if (eb_valid) begin
        if (is_com) begin
          os_pos <= 5'd1;
          os_ok  <= !s_err;
        end else if (os_pos != 5'd0) begin
          case (os_pos)
            5'd1: begin
              if (is_skp) begin
                os_pos <= 5'd0;                 // SKP ordered set: nothing to report
              end else begin
                ts_link     <= s_d;
                ts_link_pad <= s_k && s_d == K_PAD;
                if (s_err || (s_k && s_d != K_PAD)) os_ok <= 1'b0;
                os_pos <= 5'd2;
              end
            end
            5'd2: begin
              ts_lane     <= s_d;
              ts_lane_pad <= s_k && s_d == K_PAD;
              if (s_err || (s_k && s_d != K_PAD)) os_ok <= 1'b0;
              os_pos <= 5'd3;
            end
            5'd3, 5'd4, 5'd5: begin             // N_FTS, rate ID, training control
              if (s_err || s_k) os_ok <= 1'b0;
              os_pos <= os_pos + 5'd1;
            end
            5'd6: begin
              os_id  <= s_d;
              os_ts2 <= (s_d == TS2_ID);
              if (s_err || s_k || (s_d != TS1_ID && s_d != TS2_ID)) os_ok <= 1'b0;
              os_pos <= 5'd7;
            end
            default: begin                      // 7..15: repeat of the identifier
              if (s_err || s_k || s_d != os_id) os_ok <= 1'b0;
              if (os_pos == 5'd15) begin
                os_pos    <= 5'd0;
                ts_valid  <= os_ok && !(s_err || s_k || s_d != os_id);
                ts_is_ts2 <= os_ts2;
              end else begin
                os_pos <= os_pos + 5'd1;
              end
            end
          endcase
        end
      end
    end
  end

  // ---- packet de-framer ----
  // A one-byte holding register lets us mark the last payload byte with eop
  // when the END/EDB symbol arrives.
  logic       in_pkt, cur_dllp, have_held, first_out, pkt_bad;
  logic [7:0] held;

  always_ff @(posedge clk) begin
    if (rst) begin
      in_pkt      <= 1'b0;
      cur_dllp    <= 1'b0;
      have_held   <= 1'b0;
      first_out   <= 1'b0;
      pkt_bad     <= 1'b0;
      held        <= 8'h00;
      pkt_valid   <= 1'b0;
      pkt_data    <= 8'h00;
      pkt_sop     <= 1'b0;
      pkt_eop     <= 1'b0;
      pkt_dllp    <= 1'b0;
      pkt_err     <= 1'b0;
      pkt_edb     <= 1'b0;
      idle_sym    <= 1'b0;
      nonidle_sym <= 1'b0;
    end else begin
      pkt_valid   <= 1'b0;
      pkt_sop     <= 1'b0;
      pkt_eop     <= 1'b0;
      pkt_err     <= 1'b0;
      pkt_edb     <= 1'b0;
      idle_sym    <= 1'b0;
      nonidle_sym <= 1'b0;

      if (eb_valid && (is_com || in_ts_sym || is_skp)) begin
        // Ordered-set symbols. A COM inside a packet is a framing error.
        if (is_com && in_pkt) begin
          if (have_held) begin
            pkt_valid <= 1'b1;
            pkt_data  <= held;
            pkt_sop   <= first_out;
            pkt_eop   <= 1'b1;
            pkt_dllp  <= cur_dllp;
            pkt_err   <= 1'b1;
          end
          in_pkt    <= 1'b0;
          have_held <= 1'b0;
        end
      end else if (eb_valid && s_k) begin
        if (s_d == K_STP || s_d == K_SDP) begin
          if (in_pkt && have_held) begin        // previous packet never ended
            pkt_valid <= 1'b1;
            pkt_data  <= held;
            pkt_sop   <= first_out;
            pkt_eop   <= 1'b1;
            pkt_dllp  <= cur_dllp;
            pkt_err   <= 1'b1;
          end
          in_pkt    <= 1'b1;
          cur_dllp  <= (s_d == K_SDP);
          have_held <= 1'b0;
          first_out <= 1'b1;
          pkt_bad   <= s_err;
        end else if (s_d == K_END || s_d == K_EDB) begin
          if (in_pkt && have_held) begin
            pkt_valid <= 1'b1;
            pkt_data  <= held;
            pkt_sop   <= first_out;
            pkt_eop   <= 1'b1;
            pkt_dllp  <= cur_dllp;
            pkt_err   <= pkt_bad || s_err || (s_d == K_EDB);
            pkt_edb   <= (s_d == K_EDB);
          end
          in_pkt    <= 1'b0;
          have_held <= 1'b0;
        end else if (in_pkt) begin
          pkt_bad <= 1'b1;                      // unexpected K symbol inside a packet
        end
      end else if (eb_valid) begin
        // Data symbol outside an ordered set
        if (in_pkt) begin
          if (have_held) begin
            pkt_valid <= 1'b1;
            pkt_data  <= held;
            pkt_sop   <= first_out;
            pkt_dllp  <= cur_dllp;
            first_out <= 1'b0;
          end
          held      <= ds_d;
          have_held <= 1'b1;
          if (s_err) pkt_bad <= 1'b1;
        end else if (!s_err && ds_d == IDLE_D) begin
          idle_sym <= 1'b1;
        end else begin
          nonidle_sym <= 1'b1;
        end
      end
    end
  end

  // ---- status ----
  always_ff @(posedge clk) begin
    if (rst) rx_err_pulse <= 1'b0;
    else     rx_err_pulse <= eb_valid && s_err;
  end

endmodule
