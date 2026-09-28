`timescale 1ns/1ps
// =============================================================================
// pcie_8b10b_dec.sv
// -----------------------------------------------------------------------------
// 8b/10b decoder. Inverse of pcie_8b10b_enc.
//
//   din[9:0] = {j,h,g,f,i,e,d,c,b,a}  (din[0] = first bit received)
//   dout     = HGF_EDCBA, kout = 1 for a control symbol
//   code_err = code group not in the 8b/10b tables
//   disp_err = sub-block disparity inconsistent with running disparity
//
// The running disparity self-synchronises: after any unbalanced sub-block the
// decoder's RD takes the polarity of that sub-block, so a wrong initial RD
// produces at most one disparity error.
// Output is registered (1 cycle latency).
// =============================================================================
module pcie_8b10b_dec (
  input  logic       clk,
  input  logic       rst,
  input  logic       en,
  input  logic [9:0] din,
  output logic [7:0] dout,
  output logic       kout,
  output logic       code_err,
  output logic       disp_err
);

  logic rd;

  // Same RD- tables as the encoder (abcdei / fghj literal order)
  function automatic logic [5:0] tbl6(input logic [4:0] x);
    case (x)
      5'd0 : return 6'b100111;  5'd1 : return 6'b011101;
      5'd2 : return 6'b101101;  5'd3 : return 6'b110001;
      5'd4 : return 6'b110101;  5'd5 : return 6'b101001;
      5'd6 : return 6'b011001;  5'd7 : return 6'b111000;
      5'd8 : return 6'b111001;  5'd9 : return 6'b100101;
      5'd10: return 6'b010101;  5'd11: return 6'b110100;
      5'd12: return 6'b001101;  5'd13: return 6'b101100;
      5'd14: return 6'b011100;  5'd15: return 6'b010111;
      5'd16: return 6'b011011;  5'd17: return 6'b100011;
      5'd18: return 6'b010011;  5'd19: return 6'b110010;
      5'd20: return 6'b001011;  5'd21: return 6'b101010;
      5'd22: return 6'b011010;  5'd23: return 6'b111010;
      5'd24: return 6'b110011;  5'd25: return 6'b100110;
      5'd26: return 6'b010110;  5'd27: return 6'b110110;
      5'd28: return 6'b001110;  5'd29: return 6'b101110;
      5'd30: return 6'b011110;  default: return 6'b101011;
    endcase
  endfunction

  function automatic logic [3:0] tbl4d(input logic [2:0] y);
    case (y)
      3'd0: return 4'b1011;  3'd1: return 4'b1001;
      3'd2: return 4'b0101;  3'd3: return 4'b1100;
      3'd4: return 4'b1101;  3'd5: return 4'b1010;
      3'd6: return 4'b0110;  default: return 4'b1110;
    endcase
  endfunction

  function automatic logic [3:0] tbl4k(input logic [2:0] y);
    case (y)
      3'd0: return 4'b1011;  3'd1: return 4'b0110;
      3'd2: return 4'b1010;  3'd3: return 4'b1100;
      3'd4: return 4'b1101;  3'd5: return 4'b0101;
      3'd6: return 4'b1001;  default: return 4'b0111;
    endcase
  endfunction

  function automatic int pop6(input logic [5:0] v);
    return $countones(v);
  endfunction

  function automatic int pop4(input logic [3:0] v);
    return $countones(v);
  endfunction

  function automatic logic [5:0] rev6(input logic [5:0] v);
    return {v[0], v[1], v[2], v[3], v[4], v[5]};
  endfunction

  function automatic logic [3:0] rev4(input logic [3:0] v);
    return {v[0], v[1], v[2], v[3]};
  endfunction

  logic [5:0] l6;          // 6b sub-block in abcdei literal order
  logic [3:0] l4, n4;      // 4b sub-block in fghj literal order
  logic [4:0] x;
  logic [2:0] y;
  logic       f6, f4, k28, kx7, is_k;
  logic       rd_mid, rd_end, derr;
  int         p6, p4;
  logic [5:0] t6;          // loop temporaries
  logic [3:0] t4;
  logic       cmp6, cmp4;

  always_comb begin
    l6  = rev6(din[5:0]);
    l4  = rev4(din[9:6]);
    x   = 5'd0;
    y   = 3'd0;
    f6  = 1'b0;
    f4  = 1'b0;
    kx7 = 1'b0;
    n4  = 4'b0000;
    t6  = 6'b0;
    t4  = 4'b0;
    cmp6 = 1'b0;
    cmp4 = 1'b0;

    // ---- 6b -> 5b ----
    k28 = (l6 == 6'b001111) || (l6 == 6'b110000);
    if (k28) begin
      x  = 5'd28;
      f6 = 1'b1;
    end else begin
      for (int i = 0; i < 32; i++) begin
        t6   = tbl6(5'(i));
        cmp6 = (pop6(t6) != 3) || (i == 7);
        if (l6 == t6 || (cmp6 && l6 == ~t6)) begin
          x  = 5'(i);
          f6 = 1'b1;
        end
      end
    end

    // ---- 4b -> 3b ----
    if (k28) begin
      // After 001111 the RD is +, so the 4b part was complemented: undo it.
      n4 = (l6 == 6'b001111) ? ~l4 : l4;
      for (int j = 0; j < 8; j++) begin
        if (n4 == tbl4k(3'(j))) begin
          y  = 3'(j);
          f4 = 1'b1;
        end
      end
    end else if ((x == 5'd23 || x == 5'd27 || x == 5'd29 || x == 5'd30) &&
                 (l4 == 4'b0111 || l4 == 4'b1000)) begin
      kx7 = 1'b1;           // K23.7 / K27.7 / K29.7 / K30.7
      y   = 3'd7;
      f4  = 1'b1;
    end else begin
      for (int j = 0; j < 8; j++) begin
        t4   = tbl4d(3'(j));
        cmp4 = (pop4(t4) != 2) || (j == 3);
        if (l4 == t4 || (cmp4 && l4 == ~t4)) begin
          y  = 3'(j);
          f4 = 1'b1;
        end
      end
      if (l4 == 4'b0111 || l4 == 4'b1000) begin  // D.x.A7
        y  = 3'd7;
        f4 = 1'b1;
      end
    end
    is_k = k28 || kx7;

    // ---- running disparity check ----
    p6   = pop6(l6);
    p4   = pop4(l4);
    derr = 1'b0;
    if ((p6 > 3 && rd) || (p6 < 3 && !rd))            derr = 1'b1;
    if (!k28 && l6 == 6'b111000 &&  rd)               derr = 1'b1;
    if (!k28 && l6 == 6'b000111 && !rd)               derr = 1'b1;
    rd_mid = (p6 > 3) ? 1'b1 : (p6 < 3) ? 1'b0 : rd;
    if ((p4 > 2 && rd_mid) || (p4 < 2 && !rd_mid))   derr = 1'b1;
    if (!is_k && l4 == 4'b1100 &&  rd_mid)            derr = 1'b1;
    if (!is_k && l4 == 4'b0011 && !rd_mid)            derr = 1'b1;
    rd_end = (p4 > 2) ? 1'b1 : (p4 < 2) ? 1'b0 : rd_mid;
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      rd       <= 1'b0;
      dout     <= 8'h00;
      kout     <= 1'b0;
      code_err <= 1'b0;
      disp_err <= 1'b0;
    end else if (en) begin
      rd       <= rd_end;
      dout     <= {y, x};
      kout     <= is_k;
      code_err <= !(f6 && f4);
      disp_err <= derr;
    end
  end

endmodule
