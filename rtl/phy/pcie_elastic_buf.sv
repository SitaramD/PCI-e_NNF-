`timescale 1ns/1ps
// =============================================================================
// pcie_elastic_buf.sv
// -----------------------------------------------------------------------------
// Elastic buffer for clock compensation (recovered clock -> local core clock).
//
// PCIe allows +/-300 ppm between the two ends of a link. The transmitter sends
// a SKP ordered set (COM SKP SKP SKP) every ~1200 symbols. This buffer keeps
// its fill level near the middle by:
//   * write side: dropping one SKP from a SKP ordered set when it is too full
//   * read side : repeating one SKP of a SKP ordered set when it is too empty
// At least one SKP is always kept, so the receiver still sees COM + SKP.
//
// It is an asynchronous FIFO with Gray-coded pointers and 2-flop
// synchronizers. Entry format: {err, k, data[7:0]}.
// =============================================================================
module pcie_elastic_buf #(
  parameter int AW = 5                // 2**AW entries (32)
) (
  // write side: recovered clock
  input  logic       wclk,
  input  logic       wrst,
  input  logic       wen,
  input  logic [9:0] wdata,           // {err, k, data}
  // read side: core clock
  input  logic       rclk,
  input  logic       rrst,
  output logic       rvalid,
  output logic [9:0] rdata,
  output logic       skp_added,       // debug pulses
  output logic       skp_dropped,
  output logic       overflow,
  output logic       underflow
);
  import pcie_pkg::*;

  localparam int DEPTH = 1 << AW;
  localparam int START = DEPTH / 2;       // start reading at half full
  localparam int HI    = DEPTH / 2 + 4;   // write side drops a SKP above this
  localparam int LO    = DEPTH / 2 - 4;   // read side adds a SKP below this

  logic [9:0] mem [DEPTH];

  function automatic logic [AW:0] bin2gray(input logic [AW:0] b);
    return b ^ (b >> 1);
  endfunction

  function automatic logic [AW:0] gray2bin(input logic [AW:0] g);
    logic [AW:0] b;
    b[AW] = g[AW];
    for (int i = AW - 1; i >= 0; i--) b[i] = b[i+1] ^ g[i];
    return b;
  endfunction

  // ---------------------------------------------------------------------------
  // Write side
  // ---------------------------------------------------------------------------
  logic [AW:0] wptr, wgray, rgray_w1, rgray_w2, rptr_w, wfill;
  logic [AW:0] rptr_r_q, rgray;   // read pointer (binary / Gray), read domain
  logic        w_last_skp, w_dropped_os, w_is_skp, w_is_com, w_drop;

  assign w_is_skp = wdata[8] && wdata[7:0] == K_SKP;
  assign w_is_com = wdata[8] && wdata[7:0] == K_COM;
  assign rptr_w   = gray2bin(rgray_w2);
  assign wfill    = wptr - rptr_w;
  assign w_drop   = wen && w_is_skp && w_last_skp && !w_dropped_os && (wfill > (AW+1)'(HI));

  always_ff @(posedge wclk) begin
    if (wrst) begin
      wptr         <= '0;
      wgray        <= '0;
      rgray_w1     <= '0;
      rgray_w2     <= '0;
      w_last_skp   <= 1'b0;
      w_dropped_os <= 1'b0;
      skp_dropped  <= 1'b0;
      overflow     <= 1'b0;
    end else begin
      rgray_w1    <= rgray;
      rgray_w2    <= rgray_w1;
      skp_dropped <= w_drop;
      overflow    <= 1'b0;
      if (wen) begin
        w_last_skp <= w_is_skp;
        if (w_is_com) w_dropped_os <= 1'b0;
        if (w_drop) begin
          w_dropped_os <= 1'b1;
        end else if (wfill == (AW+1)'(DEPTH)) begin
          overflow <= 1'b1;                     // should never happen
        end else begin
          mem[wptr[AW-1:0]] <= wdata;
          wptr              <= wptr + 1'b1;
          wgray             <= bin2gray(wptr + 1'b1);
        end
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Read side
  // ---------------------------------------------------------------------------
  logic [AW:0] wgray_r1, wgray_r2, wptr_r, rfill;
  logic        started, r_last_skp, r_added_os;

  assign wptr_r = gray2bin(wgray_r2);
  assign rfill  = wptr_r - rptr_r_q;

  always_ff @(posedge rclk) begin
    if (rrst) begin
      rptr_r_q   <= '0;
      rgray      <= '0;
      wgray_r1   <= '0;
      wgray_r2   <= '0;
      started    <= 1'b0;
      r_last_skp <= 1'b0;
      r_added_os <= 1'b0;
      rvalid     <= 1'b0;
      rdata      <= 10'h0;
      skp_added  <= 1'b0;
      underflow  <= 1'b0;
    end else begin
      wgray_r1  <= wgray;
      wgray_r2  <= wgray_r1;
      skp_added <= 1'b0;
      underflow <= 1'b0;
      rvalid    <= 1'b0;
      if (!started) begin
        if (rfill >= (AW+1)'(START)) started <= 1'b1;
      end else if (r_last_skp && !r_added_os && rfill < (AW+1)'(LO)) begin
        // repeat a SKP (do not advance the read pointer)
        rvalid     <= 1'b1;
        rdata      <= {1'b0, 1'b1, K_SKP};
        r_added_os <= 1'b1;
        skp_added  <= 1'b1;
      end else if (rfill == '0) begin
        underflow <= 1'b1;                      // should never happen
      end else begin
        rvalid     <= 1'b1;
        rdata      <= mem[rptr_r_q[AW-1:0]];
        rptr_r_q   <= rptr_r_q + 1'b1;
        rgray      <= bin2gray(rptr_r_q + 1'b1);
        r_last_skp <= mem[rptr_r_q[AW-1:0]][8] && mem[rptr_r_q[AW-1:0]][7:0] == K_SKP;
        if (mem[rptr_r_q[AW-1:0]][8] && mem[rptr_r_q[AW-1:0]][7:0] == K_COM) r_added_os <= 1'b0;
      end
    end
  end

endmodule
