`timescale 1ns/1ps
// =============================================================================
// pcie_channel.sv  (simulation model, not synthesizable)
// -----------------------------------------------------------------------------
// One direction of a PCIe lane as seen from the RTL: transmitter's 10-bit
// words -> serial wire -> receiver's deserializer.
//
//   * The receiver's recovered clock is the transmitter's clock (ideal CDR).
//   * The deserializer word boundary is shifted by BIT_OFFSET bits, so the
//     receiver must find symbol alignment itself (comma alignment).
//   * PIPE_DELAY words of latency model the wire + SerDes pipeline.
//   * Bit errors can be injected: while err_inject is high one bit of every
//     transmitted word is flipped.
//   * Electrical idle is modelled as all-zero bits.
// =============================================================================
module pcie_channel #(
  parameter int BIT_OFFSET = 3,         // 0..9
  parameter int PIPE_DELAY = 4
) (
  input  logic       tx_clk,
  input  logic [9:0] tx_sym,
  input  logic       tx_eidle,
  input  logic       err_inject,
  output logic       rx_clk,
  output logic [9:0] rx_raw
);

  logic [9:0] pipe [PIPE_DELAY];
  logic [9:0] prev, cur;
  int         err_bit = 0;

  assign rx_clk = tx_clk;

  always_ff @(posedge tx_clk) begin
    logic [9:0] w;
    w = tx_eidle ? 10'h000 : tx_sym;
    if (err_inject && !tx_eidle) begin
      w[err_bit] = ~w[err_bit];
      err_bit    = (err_bit + 3) % 10;
    end
    pipe[0] <= w;
    for (int i = 1; i < PIPE_DELAY; i++) pipe[i] <= pipe[i-1];
    cur  <= pipe[PIPE_DELAY-1];
    prev <= cur;
  end

  // Bit stream order: prev[0..9] then cur[0..9]. Take 10 bits starting at BIT_OFFSET.
  logic [19:0] win;
  assign win    = {cur, prev};
  assign rx_raw = win[BIT_OFFSET +: 10];

endmodule
