`timescale 1ns/1ps
// =============================================================================
// tb_phy_link.sv - physical layer only: two PHYs train the link to L0
// -----------------------------------------------------------------------------
// A downstream-port PHY and an upstream-port PHY are connected through two
// channel models (different bit offsets) and run on clocks 500 ppm apart.
// Pass = both LTSSMs reach L0, and then stay in L0 for 200 us of logical idle
// (which exercises SKP insertion/removal in the elastic buffers).
// This is the milestone-3 test from the project plan.
// =============================================================================
module tb_phy_link;
  import pcie_pkg::*;

  logic clk_a = 1'b0, clk_b = 1'b0, rst = 1'b1;
  always #2.000 clk_a = ~clk_a;
  always #2.001 clk_b = ~clk_b;

  logic [9:0] a_tx, b_tx, a_rx, b_rx;
  logic       a_ei, b_ei, a_rxclk, b_rxclk;
  logic       a_up, b_up, a_l0, b_l0;
  ltssm_e     sa, sb, pa = LT_DETECT_QUIET, pb = LT_DETECT_QUIET;

  pcie_phy_top #(.DOWNSTREAM(1'b1), .T_DETECT_QUIET(100)) u_a (
    .clk(clk_a), .rst(rst), .tx_sym(a_tx), .tx_eidle(a_ei), .rx_clk(a_rxclk), .rx_raw(a_rx),
    .rx_present(1'b1),
    .dll_tx_valid(1'b0), .dll_tx_data(8'h00), .dll_tx_sop(1'b0), .dll_tx_eop(1'b0),
    .dll_tx_dllp(1'b0), .dll_tx_ready(),
    .dll_rx_valid(), .dll_rx_data(), .dll_rx_sop(), .dll_rx_eop(), .dll_rx_dllp(),
    .dll_rx_err(), .dll_rx_edb(),
    .retrain_req(1'b0), .link_up(a_up), .in_l0(a_l0), .ltssm_state(sa), .rx_err_pulse());

  pcie_phy_top #(.DOWNSTREAM(1'b0), .T_DETECT_QUIET(300)) u_b (
    .clk(clk_b), .rst(rst), .tx_sym(b_tx), .tx_eidle(b_ei), .rx_clk(b_rxclk), .rx_raw(b_rx),
    .rx_present(1'b1),
    .dll_tx_valid(1'b0), .dll_tx_data(8'h00), .dll_tx_sop(1'b0), .dll_tx_eop(1'b0),
    .dll_tx_dllp(1'b0), .dll_tx_ready(),
    .dll_rx_valid(), .dll_rx_data(), .dll_rx_sop(), .dll_rx_eop(), .dll_rx_dllp(),
    .dll_rx_err(), .dll_rx_edb(),
    .retrain_req(1'b0), .link_up(b_up), .in_l0(b_l0), .ltssm_state(sb), .rx_err_pulse());

  pcie_channel #(.BIT_OFFSET(3)) ch_ab (.tx_clk(clk_a), .tx_sym(a_tx), .tx_eidle(a_ei),
                                        .err_inject(1'b0), .rx_clk(b_rxclk), .rx_raw(b_rx));
  pcie_channel #(.BIT_OFFSET(7)) ch_ba (.tx_clk(clk_b), .tx_sym(b_tx), .tx_eidle(b_ei),
                                        .err_inject(1'b0), .rx_clk(a_rxclk), .rx_raw(a_rx));

  always @(posedge clk_a) begin
    if (sa != pa) $display("%t  DS (A): %s", $time, sa.name());
    pa <= sa;
  end
  always @(posedge clk_b) begin
    if (sb != pb) $display("%t  US (B): %s", $time, sb.name());
    pb <= sb;
  end

  int left_l0 = 0;
  always @(posedge clk_a) if (a_up && !a_l0) left_l0++;

  initial begin
    $timeformat(-6, 3, " us", 12);
    #100 rst = 1'b0;
    wait (a_l0 && b_l0);
    $display("%t  both ends in L0", $time);
    #200us;
    if (a_l0 && b_l0 && left_l0 == 0) $display("*** tb_phy_link PASSED ***");
    else                              $display("*** tb_phy_link FAILED ***");
    $finish;
  end

  initial begin
    #2ms;
    $display("*** tb_phy_link FAILED: timeout ***");
    $finish;
  end
endmodule
