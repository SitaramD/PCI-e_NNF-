`timescale 1ns/1ps
// =============================================================================
// tb_crc_scrambler.sv - unit test for the pcie_pkg helper functions
// -----------------------------------------------------------------------------
//  1. Scrambler: 16 bytes of scrambled D0.0 after a COM (LFSR = 16'hFFFF)
//     must equal the sequence published in the PCIe base specification:
//       FF 17 C0 14 B2 E7 02 82 72 6E 28 A6 BE 6D BF 8D
//  2. Scramble -> descramble round trip on random data.
//  3. DLLP CRC-16: complete DLLPs compared with an independent model
//     (Alex Forencich's cocotbext-pcie, core/dllp.py).
//  4. TLP LCRC-32: compared with zlib.crc32 (Ethernet CRC-32 arithmetic).
//
// Expected values were produced by Python (see comments next to each vector).
// =============================================================================
module tb_crc_scrambler;
  import pcie_pkg::*;

  int errors = 0;

  task automatic chk(input string what, input logic [63:0] got, input logic [63:0] exp);
    if (got !== exp) begin
      errors++;
      $display("FAIL %s: got %h expected %h", what, got, exp);
    end else begin
      $display("ok   %s = %h", what, got);
    end
  endtask

  // LCRC of a byte array: returns the 4 bytes as sent, first byte in [7:0]
  function automatic logic [31:0] lcrc(input logic [7:0] b [64], input int n);
    logic [31:0] c;
    c = 32'hFFFF_FFFF;
    for (int i = 0; i < n; i++) c = crc32_byte(c, b[i]);
    return ~c;
  endfunction

  initial begin
    logic [15:0] l, l2;
    logic [7:0]  seq  [16];
    logic [7:0]  expd [16];
    logic [7:0]  t [64];

    // ------------------------------------------------------------ 1 scrambler
    expd = '{8'hFF, 8'h17, 8'hC0, 8'h14, 8'hB2, 8'hE7, 8'h02, 8'h82,
             8'h72, 8'h6E, 8'h28, 8'hA6, 8'hBE, 8'h6D, 8'hBF, 8'h8D};
    l = 16'hFFFF;
    for (int i = 0; i < 16; i++) begin
      seq[i] = 8'h00 ^ lfsr_mask(l);
      l      = lfsr_adv8(l);
      if (seq[i] !== expd[i]) begin
        errors++;
        $display("FAIL scrambler byte %0d: got %02x expected %02x", i, seq[i], expd[i]);
      end
    end
    if (errors == 0) $display("ok   scrambler sequence matches the spec");

    // ------------------------------------------------------------ 2 round trip
    l  = 16'hFFFF;
    l2 = 16'hFFFF;
    for (int i = 0; i < 1000; i++) begin
      logic [7:0] d, s, r;
      d  = $urandom;
      s  = d ^ lfsr_mask(l);
      r  = s ^ lfsr_mask(l2);
      l  = lfsr_adv8(l);
      l2 = lfsr_adv8(l2);
      if (r !== d) begin errors++; $display("FAIL descramble %0d", i); break; end
    end

    // ------------------------------------------------------------ 3 DLLP CRC
    // Dllp.create_ack(0x123).pack_crc()            -> 00 00 01 23 e2 85
    chk("Ack(0x123)",  64'(dllp_pack(DLLP_ACK, 8'h00, 8'h01, 8'h23)), 64'h85e2_2301_0000);
    // Dllp.create_nak(0xFFF).pack_crc()            -> 10 00 0f ff ce cf
    chk("Nak(0xFFF)",  64'(dllp_pack(DLLP_NAK, 8'h00, 8'h0F, 8'hFF)), 64'hcfce_ff0f_0010);
    // InitFC1-P hdr_fc=8 data_fc=64                 -> 40 02 00 40 f3 68
    chk("InitFC1-P",   64'(dllp_pack_fc(DLLP_INITFC1_P, 8'd8, 12'd64)), 64'h68f3_4000_0240);
    // UpdateFC-NP hdr_fc=0xAB data_fc=0x5CD         -> 90 2a c5 cd 9f 0d
    chk("UpdateFC-NP", 64'(dllp_pack_fc(DLLP_UPDFC_N, 8'hAB, 12'h5CD)), 64'h0d9f_cdc5_2a90);

    // ------------------------------------------------------------ 4 LCRC
    // seq 0 + CfgRd0 header: zlib.crc32 -> LE bytes 4f a6 2a ff
    for (int i = 0; i < 64; i++) t[i] = 8'h00;
    t[2] = 8'h04;  t[5] = 8'h01;  t[9] = 8'h0F;  t[10] = 8'h01;
    chk("LCRC CfgRd0", 64'(lcrc(t, 14)), 64'hff2a_a64f);
    // seq 0x123 + bytes 0..43                        -> 86 96 8b 0d
    t[0] = 8'h01;
    t[1] = 8'h23;
    for (int i = 0; i < 44; i++) t[2 + i] = 8'(i);
    chk("LCRC 44B",    64'(lcrc(t, 46)), 64'h0d8b_9686);

    if (errors == 0) $display("*** tb_crc_scrambler PASSED ***");
    else             $display("*** tb_crc_scrambler FAILED: %0d errors ***", errors);
    $finish;
  end
endmodule
