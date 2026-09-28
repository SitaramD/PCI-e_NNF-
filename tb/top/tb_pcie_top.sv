`timescale 1ns/1ps
// =============================================================================
// tb_pcie_top.sv - system test: root complex BFM <-> endpoint over a lane
// -----------------------------------------------------------------------------
//
//   +-----------+   pcie_channel (RC->EP, bit offset 3)   +-------------+
//   |  RC BFM   | ---------------------------------------> | pcie_ep_top |--AXI-Lite--> axil_regfile
//   | (PHY+DLL  | <--------------------------------------- |             |
//   |  +tasks)  |   pcie_channel (EP->RC, bit offset 7)    +-------------+
//   +-----------+
//
// The two sides run on different clocks (500 ppm apart) so the elastic
// buffers have to add/drop SKP symbols.
//
// Tests
//   1  link training to L0 and DL_Active on both sides
//   2  config read of Vendor/Device ID
//   3  BAR0 sizing and assignment, Command register
//   4  completer ID captured from CfgWr
//   5  capability list walk (PM -> PCIe), Link Capabilities
//   6  single-DW memory read / write through BAR0
//   7  8-DW burst write + read
//   8  byte enables on write and read (byte count field)
//   9  BAR miss -> Unsupported Request completion
//  10  bit errors EP->RC during a completion  -> Nak + replay
//  11  bit errors RC->EP during a write        -> Nak + replay
//  12  forced retrain (Recovery) and traffic afterwards
//  13  random read/write regression against a reference model
// =============================================================================
module tb_pcie_top;
  import pcie_pkg::*;

  // ---------------------------------------------------------------------------
  // Simulation-friendly timer values (spec values are in the RTL defaults)
  // ---------------------------------------------------------------------------
  localparam int unsigned T_DQ   = 200;
  localparam int unsigned T_2    = 20_000;
  localparam int unsigned T_24   = 100_000;
  localparam int unsigned T_48   = 200_000;
  localparam int unsigned RPL_TO = 3000;
  localparam int          N_RANDOM = 200;

  // ---------------------------------------------------------------------------
  // Clocks and resets
  // ---------------------------------------------------------------------------
  logic ep_clk = 1'b0, rc_clk = 1'b0;
  logic ep_rst = 1'b1, rc_rst = 1'b1;
  always #2.000 ep_clk = ~ep_clk;   // 250.000 MHz
  always #2.001 rc_clk = ~rc_clk;   // 249.875 MHz (500 ppm slower)

  // ---------------------------------------------------------------------------
  // Endpoint + example register file
  // ---------------------------------------------------------------------------
  logic [9:0]  ep_tx_sym, ep_rx_raw, rc_tx_sym, rc_rx_raw;
  logic        ep_tx_eidle, rc_tx_eidle, ep_rx_clk, rc_rx_clk;
  logic        err_rc2ep = 1'b0, err_ep2rc = 1'b0, force_retrain = 1'b0;

  logic [31:0] awaddr, wdata, araddr, rdata;
  logic [3:0]  wstrb;
  logic [1:0]  bresp, rresp;
  logic        awvalid, awready, wvalid, wready, bvalid, bready;
  logic        arvalid, arready, rvalid, rready;

  ltssm_e      ep_ltssm, rc_ltssm;
  logic        ep_link_up, ep_dl_up, rc_link_up, rc_dl_up;
  logic [15:0] ep_cid, ep_bad_tlp, ep_replays;

  pcie_ep_top #(
    .VENDOR_ID      (16'h1234),
    .DEVICE_ID      (16'hABCD),
    .T_DETECT_QUIET (T_DQ),
    .T_2MS          (T_2),
    .T_24MS         (T_24),
    .T_48MS         (T_48),
    .REPLAY_TIMEOUT (RPL_TO)
  ) dut (
    .clk            (ep_clk),
    .rst            (ep_rst),
    .tx_sym         (ep_tx_sym),
    .tx_eidle       (ep_tx_eidle),
    .rx_clk         (ep_rx_clk),
    .rx_raw         (ep_rx_raw),
    .rx_present     (1'b1),
    .m_axil_awaddr  (awaddr),
    .m_axil_awvalid (awvalid),
    .m_axil_awready (awready),
    .m_axil_wdata   (wdata),
    .m_axil_wstrb   (wstrb),
    .m_axil_wvalid  (wvalid),
    .m_axil_wready  (wready),
    .m_axil_bresp   (bresp),
    .m_axil_bvalid  (bvalid),
    .m_axil_bready  (bready),
    .m_axil_araddr  (araddr),
    .m_axil_arvalid (arvalid),
    .m_axil_arready (arready),
    .m_axil_rdata   (rdata),
    .m_axil_rresp   (rresp),
    .m_axil_rvalid  (rvalid),
    .m_axil_rready  (rready),
    .ltssm_state    (ep_ltssm),
    .link_up        (ep_link_up),
    .dl_up          (ep_dl_up),
    .completer_id   (ep_cid),
    .stat_bad_tlp   (ep_bad_tlp),
    .stat_replays   (ep_replays)
  );

  axil_regfile #(.ADDR_BITS(12)) u_regs (
    .clk            (ep_clk),
    .rst            (ep_rst),
    .s_axil_awaddr  (awaddr),
    .s_axil_awvalid (awvalid),
    .s_axil_awready (awready),
    .s_axil_wdata   (wdata),
    .s_axil_wstrb   (wstrb),
    .s_axil_wvalid  (wvalid),
    .s_axil_wready  (wready),
    .s_axil_bresp   (bresp),
    .s_axil_bvalid  (bvalid),
    .s_axil_bready  (bready),
    .s_axil_araddr  (araddr),
    .s_axil_arvalid (arvalid),
    .s_axil_arready (arready),
    .s_axil_rdata   (rdata),
    .s_axil_rresp   (rresp),
    .s_axil_rvalid  (rvalid),
    .s_axil_rready  (rready)
  );

  // ---------------------------------------------------------------------------
  // Root complex model
  // ---------------------------------------------------------------------------
  pcie_rc_bfm #(
    .T_DETECT_QUIET (T_DQ),
    .T_2MS          (T_2),
    .T_24MS         (T_24),
    .T_48MS         (T_48),
    .REPLAY_TIMEOUT (RPL_TO)
  ) rc (
    .clk           (rc_clk),
    .rst           (rc_rst),
    .tx_sym        (rc_tx_sym),
    .tx_eidle      (rc_tx_eidle),
    .rx_clk        (rc_rx_clk),
    .rx_raw        (rc_rx_raw),
    .force_retrain (force_retrain),
    .link_up       (rc_link_up),
    .dl_up         (rc_dl_up),
    .ltssm_state   (rc_ltssm)
  );

  // ---------------------------------------------------------------------------
  // The lane
  // ---------------------------------------------------------------------------
  pcie_channel #(.BIT_OFFSET(3), .PIPE_DELAY(4)) ch_rc2ep (
    .tx_clk     (rc_clk),
    .tx_sym     (rc_tx_sym),
    .tx_eidle   (rc_tx_eidle),
    .err_inject (err_rc2ep),
    .rx_clk     (ep_rx_clk),
    .rx_raw     (ep_rx_raw)
  );

  pcie_channel #(.BIT_OFFSET(7), .PIPE_DELAY(5)) ch_ep2rc (
    .tx_clk     (ep_clk),
    .tx_sym     (ep_tx_sym),
    .tx_eidle   (ep_tx_eidle),
    .err_inject (err_ep2rc),
    .rx_clk     (rc_rx_clk),
    .rx_raw     (rc_rx_raw)
  );

  // ---------------------------------------------------------------------------
  // LTSSM trace
  // ---------------------------------------------------------------------------
  ltssm_e ep_prev = LT_DETECT_QUIET, rc_prev = LT_DETECT_QUIET;
  always @(posedge ep_clk) begin
    if (ep_ltssm != ep_prev) $display("%t   EP LTSSM -> %s", $time, ep_ltssm.name());
    ep_prev <= ep_ltssm;
  end
  always @(posedge rc_clk) begin
    if (rc_ltssm != rc_prev) $display("%t   RC LTSSM -> %s", $time, rc_ltssm.name());
    rc_prev <= rc_ltssm;
  end

  // ---------------------------------------------------------------------------
  // Checking helpers
  // ---------------------------------------------------------------------------
  int errors = 0;
  int checks = 0;

  task automatic check32(input string what, input logic [31:0] got, input logic [31:0] exp);
    checks++;
    if (got !== exp) begin
      errors++;
      $display("%t   FAIL %s: got %08x expected %08x", $time, what, got, exp);
    end
  endtask

  task automatic section(input string s);
    $display("%t ==== %s", $time, s);
  endtask

  // Reference model of the register file (index = byte address / 4)
  logic [31:0] model [1024];

  localparam logic [15:0] EP_BDF   = 16'h0100;     // bus 1, device 0, function 0
  localparam logic [31:0] BAR_BASE = 32'hF000_0000;

  // ---------------------------------------------------------------------------
  // Test sequence
  // ---------------------------------------------------------------------------
  initial begin
    logic [31:0] d;
    logic [2:0]  st;
    int          t;

    $timeformat(-6, 3, " us", 12);
    for (int i = 0; i < 1024; i++) model[i] = 32'h0;
    model[0] = 32'hC0DE_0001;

    #100;
    ep_rst = 1'b0;
    #37;
    rc_rst = 1'b0;

    // ------------------------------------------------------------------ 1
    section("1: link training");
    t = 0;
    while (!(ep_dl_up && rc_dl_up) && t < 200_000) begin
      @(posedge rc_clk);
      t++;
    end
    checks++;
    if (!(ep_dl_up && rc_dl_up)) begin
      errors++;
      $display("FAIL: link did not come up");
      $finish;
    end
    $display("%t   link up: L0 + DL_Active on both ends", $time);

    // ------------------------------------------------------------------ 2
    section("2: config read Vendor/Device ID");
    rc.cfg_rd(EP_BDF, 10'h000, d, st);
    check32("cfg status", 32'(st), 32'(CPL_SC));
    check32("vendor/device ID", d, 32'hABCD_1234);

    // ------------------------------------------------------------------ 3
    section("3: BAR0 sizing and assignment");
    rc.cfg_wr(EP_BDF, 10'h004, 32'hFFFF_FFFF, 4'hF, st);
    check32("cfgwr status", 32'(st), 32'(CPL_SC));
    rc.cfg_rd(EP_BDF, 10'h004, d, st);
    check32("BAR0 size mask (4 KB)", d, 32'hFFFF_F000);
    rc.cfg_wr(EP_BDF, 10'h004, BAR_BASE, 4'hF, st);
    rc.cfg_rd(EP_BDF, 10'h004, d, st);
    check32("BAR0 base", d, BAR_BASE);
    rc.cfg_wr(EP_BDF, 10'h001, 32'h0000_0006, 4'h3, st);   // memory space + bus master
    rc.cfg_rd(EP_BDF, 10'h001, d, st);
    check32("status/command", d, 32'h0010_0006);

    // ------------------------------------------------------------------ 4
    section("4: completer ID captured from CfgWr");
    check32("EP completer ID", {16'h0, ep_cid}, {16'h0, EP_BDF});
    rc.cfg_rd(EP_BDF, 10'h000, d, st);
    check32("completer ID in Cpl", {16'h0, rc.cpl_completer_id(rc.next_tag - 8'd1)}, {16'h0, EP_BDF});

    // ------------------------------------------------------------------ 5
    section("5: capability list");
    rc.cfg_rd(EP_BDF, 10'h00D, d, st);
    check32("cap pointer", d & 32'hFF, 32'h40);
    rc.cfg_rd(EP_BDF, 10'h010, d, st);
    check32("PM cap ID / next", d & 32'hFFFF, 32'h6001);
    rc.cfg_rd(EP_BDF, 10'h018, d, st);
    check32("PCIe cap ID / next / version", d, 32'h0002_0010);
    rc.cfg_rd(EP_BDF, 10'h01B, d, st);
    check32("Link Capabilities (2.5GT/s x1)", d, 32'h0000_0011);
    rc.cfg_rd(EP_BDF, 10'h01C, d, st);
    check32("Link Status (2.5GT/s x1)", d >> 16, 32'h0011);

    // ------------------------------------------------------------------ 6
    section("6: single-DW memory read / write");
    rc.mem_rd(BAR_BASE + 32'h0, 1, 4'hF, 4'h0, st);
    check32("mrd status", 32'(st), 32'(CPL_SC));
    check32("ID register", rc.rd_buf[0], 32'hC0DE_0001);
    rc.wr_buf[0] = 32'h1234_5678;                          // e.g. fair_px @ 0x04
    rc.mem_wr(BAR_BASE + 32'h4, 1, 4'hF, 4'h0);
    model[1] = 32'h1234_5678;
    rc.mem_rd(BAR_BASE + 32'h4, 1, 4'hF, 4'h0, st);
    check32("reg 0x04 readback", rc.rd_buf[0], 32'h1234_5678);

    // ------------------------------------------------------------------ 7
    section("7: 8-DW burst");
    for (int i = 0; i < 8; i++) begin
      rc.wr_buf[i] = 32'hA5A5_0000 + i;
      model[64 + i] = 32'hA5A5_0000 + i;
    end
    rc.mem_wr(BAR_BASE + 32'h100, 8, 4'hF, 4'hF);
    rc.mem_rd(BAR_BASE + 32'h100, 8, 4'hF, 4'hF, st);
    for (int i = 0; i < 8; i++) check32($sformatf("burst dw%0d", i), rc.rd_buf[i], 32'hA5A5_0000 + i);
    check32("byte count 8DW", {20'h0, rc.cpl_byte_count(rc.next_tag - 8'd1)}, 32'd32);

    // ------------------------------------------------------------------ 8
    section("8: byte enables");
    rc.wr_buf[0] = 32'hAABB_CCDD;
    rc.mem_wr(BAR_BASE + 32'h200, 1, 4'hF, 4'h0);
    rc.wr_buf[0] = 32'h1122_3344;
    rc.mem_wr(BAR_BASE + 32'h200, 1, 4'b0101, 4'h0);
    model[128] = 32'hAA22_CC44;
    rc.mem_rd(BAR_BASE + 32'h200, 1, 4'hF, 4'h0, st);
    check32("partial write", rc.rd_buf[0], 32'hAA22_CC44);
    rc.mem_rd(BAR_BASE + 32'h200, 1, 4'b0110, 4'h0, st);
    check32("byte count fbe=0110", {20'h0, rc.cpl_byte_count(rc.next_tag - 8'd1)}, 32'd2);
    rc.mem_rd(BAR_BASE + 32'h100, 3, 4'b1100, 4'b0011, st);
    check32("byte count 3DW fbe=1100 lbe=0011", {20'h0, rc.cpl_byte_count(rc.next_tag - 8'd1)}, 32'd8);

    // ------------------------------------------------------------------ 9
    section("9: BAR miss -> UR");
    rc.mem_rd(32'hE000_0000, 1, 4'hF, 4'h0, st);
    check32("UR status", 32'(st), 32'(CPL_UR));

    // ------------------------------------------------------------------ 10
    section("10: bit errors EP->RC during a completion (Nak + replay)");
    for (int i = 0; i < 32; i++) begin
      rc.wr_buf[i] = $urandom;
      model[256 + i] = rc.wr_buf[i];
    end
    rc.mem_wr(BAR_BASE + 32'h400, 32, 4'hF, 4'hF);
    begin
      int replays_before;
      replays_before = ep_replays;
      fork
        rc.mem_rd(BAR_BASE + 32'h400, 32, 4'hF, 4'hF, st);
        begin
          // wait for the endpoint to start sending a TLP, then corrupt it
          @(posedge ep_clk iff (dut.p_tx_valid && !dut.p_tx_dllp && dut.p_tx_sop));
          repeat (20) @(posedge ep_clk);
          err_ep2rc = 1'b1;
          repeat (2) @(posedge ep_clk);
          err_ep2rc = 1'b0;
        end
      join
      for (int i = 0; i < 32; i++) check32($sformatf("replayed cpl dw%0d", i), rc.rd_buf[i], model[256 + i]);
      checks++;
      if (ep_replays == replays_before) begin
        errors++;
        $display("   FAIL: expected the endpoint to replay");
      end else begin
        $display("%t   endpoint replays: %0d -> %0d", $time, replays_before, ep_replays);
      end
    end

    // ------------------------------------------------------------------ 11
    section("11: bit errors RC->EP during a write (Nak + replay)");
    begin
      int bad_before;
      bad_before = ep_bad_tlp;
      for (int i = 0; i < 16; i++) begin
        rc.wr_buf[i] = 32'h5A00_0000 + (i << 8) + i;
        model[512 + i] = rc.wr_buf[i];
      end
      fork
        rc.mem_wr(BAR_BASE + 32'h800, 16, 4'hF, 4'hF);
        begin
          @(posedge rc_clk iff (rc.p_tx_valid && !rc.p_tx_dllp && rc.p_tx_sop));
          repeat (30) @(posedge rc_clk);
          err_rc2ep = 1'b1;
          @(posedge rc_clk);
          err_rc2ep = 1'b0;
        end
      join
      rc.mem_rd(BAR_BASE + 32'h800, 16, 4'hF, 4'hF, st);
      for (int i = 0; i < 16; i++) check32($sformatf("write after replay dw%0d", i), rc.rd_buf[i], model[512 + i]);
      checks++;
      if (ep_bad_tlp == bad_before) begin
        errors++;
        $display("   FAIL: endpoint did not see the corrupted TLP");
      end else begin
        $display("%t   endpoint bad TLPs: %0d -> %0d", $time, bad_before, ep_bad_tlp);
      end
    end

    // ------------------------------------------------------------------ 12
    section("12: forced retrain (Recovery)");
    @(posedge rc_clk);
    force_retrain = 1'b1;
    @(posedge rc_clk);
    force_retrain = 1'b0;
    t = 0;
    while (rc_ltssm != LT_REC_LOCK && t < 1000) begin @(posedge rc_clk); t++; end
    t = 0;
    while (!(rc_ltssm == LT_L0 && ep_ltssm == LT_L0) && t < 100_000) begin @(posedge rc_clk); t++; end
    checks++;
    if (!(rc_ltssm == LT_L0 && ep_ltssm == LT_L0)) begin
      errors++;
      $display("   FAIL: did not return to L0");
    end
    rc.mem_rd(BAR_BASE + 32'h4, 1, 4'hF, 4'h0, st);
    check32("read after recovery", rc.rd_buf[0], model[1]);

    // ------------------------------------------------------------------ 13
    section($sformatf("13: %0d random reads / writes", N_RANDOM));
    for (int n = 0; n < N_RANDOM; n++) begin
      int          ndw, idx;
      logic [3:0]  fbe, lbe;
      ndw = 1 + ($urandom % 32);
      idx = 1 + ($urandom % (1024 - 32 - 1));      // keep away from the read-only ID register
      if ($urandom % 2) begin
        fbe = 4'hF;
        lbe = 4'hF;
        for (int i = 0; i < ndw; i++) begin
          rc.wr_buf[i]   = $urandom;
          model[idx + i] = rc.wr_buf[i];
        end
        rc.mem_wr(BAR_BASE + 32'(idx * 4), ndw, fbe, lbe);
      end else begin
        rc.mem_rd(BAR_BASE + 32'(idx * 4), ndw, 4'hF, 4'hF, st);
        check32("random rd status", 32'(st), 32'(CPL_SC));
        for (int i = 0; i < ndw; i++)
          check32($sformatf("random rd @%03x", (idx + i) * 4), rc.rd_buf[i], model[idx + i]);
      end
    end

    // ------------------------------------------------------------------ done
    repeat (200) @(posedge rc_clk);
    $display("");
    $display("=====================================================");
    $display("  checks: %0d   errors: %0d", checks, errors);
    $display("  EP: bad TLPs %0d, replays %0d | RC: bad TLPs %0d, replays %0d",
             ep_bad_tlp, ep_replays, rc.stat_bad_tlp, rc.stat_replays);
    if (errors == 0) $display("  *** TEST PASSED ***");
    else             $display("  *** TEST FAILED ***");
    $display("=====================================================");
    $finish;
  end

  // Global watchdog
  initial begin
    #20ms;
    $display("FAIL: global timeout");
    $finish;
  end

endmodule
