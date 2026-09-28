`timescale 1ns/1ps
// =============================================================================
// pcie_comma_align.sv
// -----------------------------------------------------------------------------
// Symbol (word) alignment. The deserializer hands us 10 bits per recovered
// clock, but the 10-bit boundary is arbitrary. We search a 20-bit window
// (previous word + current word) for the K28.5 (COM) code group in either
// disparity and lock the bit offset where it was found.
//
// Bit order: word[0] is the earliest received bit. Window = {cur, prev}, so
// window[0] is the oldest bit and window[o +: 10] is a candidate symbol that
// starts o bits into the previous word.
//
// K28.5 is a "singular" comma: in a legal 8b/10b stream (without K28.7) its
// pattern cannot appear across symbol boundaries, so re-aligning on every COM
// is safe.
// =============================================================================
module pcie_comma_align (
  input  logic       clk,      // recovered clock
  input  logic       rst,
  input  logic [9:0] din,      // raw word from the deserializer
  output logic [9:0] dout,     // aligned code group
  output logic       dvalid,   // dout valid (alignment has been found)
  output logic       realign   // pulse: COM found at a different bit offset
);

  localparam logic [9:0] COM_RDN = 10'h17C;  // K28.5, RD-
  localparam logic [9:0] COM_RDP = 10'h283;  // K28.5, RD+

  logic [9:0]  prev;
  logic [19:0] win;
  logic [4:0]  off, sel_off, found_off;
  logic        found, locked;

  always_comb begin
    win       = {din, prev};
    found     = 1'b0;
    found_off = 5'd0;
    for (int o = 9; o >= 0; o--) begin
      if (win[o +: 10] == COM_RDN || win[o +: 10] == COM_RDP) begin
        found     = 1'b1;
        found_off = 5'(o);
      end
    end
    sel_off = found ? found_off : off;
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      prev    <= 10'h0;
      off     <= 5'd0;
      locked  <= 1'b0;
      dout    <= 10'h0;
      dvalid  <= 1'b0;
      realign <= 1'b0;
    end else begin
      prev    <= din;
      realign <= found && (!locked || found_off != off);
      if (found) begin
        off    <= found_off;
        locked <= 1'b1;
      end
      dout   <= win[sel_off +: 10];
      dvalid <= locked || found;
    end
  end

endmodule
