`timescale 1ns/1ps
// =============================================================================
// pcie_8b10b_enc.sv
// -----------------------------------------------------------------------------
// 8b/10b encoder (IBM / Widmer-Franaszek), as used by PCIe Gen1/Gen2.
//
//   din[7:0] = HGF_EDCBA, kin = 1 for a control (K) symbol.
//   dout[9:0] = {j,h,g,f,i,e,d,c,b,a};  dout[0] ('a') is sent first on the wire.
//
// The byte is split into a 5-bit part (EDCBA -> 6-bit "abcdei") and a 3-bit
// part (HGF -> 4-bit "fghj"). Each sub-block has an RD- form; the RD+ form is
// its bitwise complement. Running disparity (RD) is updated after each
// sub-block: more ones -> RD+, more zeros -> RD-, balanced -> unchanged.
//
// Valid K symbols: K28.0..K28.7, K23.7, K27.7, K29.7, K30.7 (kerr flags others).
// Output is registered (1 cycle latency).
// =============================================================================
module pcie_8b10b_enc (
  input  logic       clk,
  input  logic       rst,
  input  logic       en,      // encode din/kin this cycle
  input  logic [7:0] din,
  input  logic       kin,
  output logic [9:0] dout,    // registered code group
  output logic       kerr     // registered: kin=1 with an invalid K value
);

  logic rd;  // running disparity: 0 = RD-, 1 = RD+

  // 5b/6b table, RD- column, written as abcdei (a = MSB of the literal)
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
      5'd30: return 6'b011110;  default: return 6'b101011; // 31
    endcase
  endfunction

  // 3b/4b data table, RD- column, written as fghj
  function automatic logic [3:0] tbl4d(input logic [2:0] y);
    case (y)
      3'd0: return 4'b1011;  3'd1: return 4'b1001;
      3'd2: return 4'b0101;  3'd3: return 4'b1100;
      3'd4: return 4'b1101;  3'd5: return 4'b1010;
      3'd6: return 4'b0110;  default: return 4'b1110; // D.x.P7
    endcase
  endfunction

  // 3b/4b control table, RD- column, written as fghj
  function automatic logic [3:0] tbl4k(input logic [2:0] y);
    case (y)
      3'd0: return 4'b1011;  3'd1: return 4'b0110;
      3'd2: return 4'b1010;  3'd3: return 4'b1100;
      3'd4: return 4'b1101;  3'd5: return 4'b0101;
      3'd6: return 4'b1001;  default: return 4'b0111; // K.x.7
    endcase
  endfunction

  function automatic int pop6(input logic [5:0] v);
    return $countones(v);
  endfunction

  function automatic int pop4(input logic [3:0] v);
    return $countones(v);
  endfunction

  // Literal "abcdei" has a at bit 5; the wire order needs a at bit 0.
  function automatic logic [5:0] rev6(input logic [5:0] v);
    return {v[0], v[1], v[2], v[3], v[4], v[5]};
  endfunction

  function automatic logic [3:0] rev4(input logic [3:0] v);
    return {v[0], v[1], v[2], v[3]};
  endfunction

  logic [4:0] x;
  logic [2:0] y;
  logic       k28, kvalid, alt7;
  logic [5:0] c6, s6;
  logic [3:0] c4, s4;
  logic       comp6, comp4, rd_mid, rd_end;
  logic [9:0] code;

  always_comb begin
    x      = din[4:0];
    y      = din[7:5];
    k28    = kin && (x == 5'd28);
    kvalid = k28 || (y == 3'd7 && (x == 5'd23 || x == 5'd27 || x == 5'd29 || x == 5'd30));

    // ---- 6b sub-block ----
    c6     = k28 ? 6'b001111 : tbl6(x);
    // Complement in RD+ for unbalanced codes, and for the balanced D.7 (111000/000111)
    comp6  = (pop6(c6) != 3) || (!kin && x == 5'd7);
    s6     = (rd && comp6) ? ~c6 : c6;
    rd_mid = (pop6(s6) > 3) ? 1'b1 : (pop6(s6) < 3) ? 1'b0 : rd;

    // ---- 4b sub-block ----
    if (kin) begin
      c4    = tbl4k(y);
      comp4 = 1'b1;
      alt7  = 1'b0;
    end else begin
      // D.x.A7 avoids a run of five identical bits across the sub-block boundary
      alt7  = (y == 3'd7) &&
              ((!rd_mid && (x == 5'd17 || x == 5'd18 || x == 5'd20)) ||
               ( rd_mid && (x == 5'd11 || x == 5'd13 || x == 5'd14)));
      c4    = alt7 ? 4'b0111 : tbl4d(y);
      comp4 = (pop4(c4) != 2) || (y == 3'd3);
    end
    s4     = (rd_mid && comp4) ? ~c4 : c4;
    rd_end = (pop4(s4) > 2) ? 1'b1 : (pop4(s4) < 2) ? 1'b0 : rd_mid;

    code   = {rev4(s4), rev6(s6)};
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      rd   <= 1'b0;
      dout <= 10'h0;
      kerr <= 1'b0;
    end else if (en) begin
      rd   <= rd_end;
      dout <= code;
      kerr <= kin && !kvalid;
    end
  end

endmodule
