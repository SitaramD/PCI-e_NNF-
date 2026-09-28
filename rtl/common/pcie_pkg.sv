`timescale 1ns/1ps
// =============================================================================
// pcie_pkg.sv
// -----------------------------------------------------------------------------
// Shared constants, types and helper functions for the PCIe Gen1 x1 stack.
//
//   * Special (K) symbols used by the physical layer
//   * LTSSM -> PHY TX "mode" encoding
//   * DLLP / TLP type codes
//   * Scrambler LFSR, DLLP CRC-16 and TLP LCRC-32 helper functions
//
// Conventions used everywhere in this repository
//   * One symbol (8b data + K flag) per clock at 250 MHz (Gen1 x1 = 2.5 GT/s).
//   * Synchronous, active-high reset.
//   * Byte streams use valid/ready handshakes with sop/eop markers.
// =============================================================================
package pcie_pkg;

  // ---------------------------------------------------------------------------
  // Special symbols (value is the 8-bit "Dx.y" byte, transmitted with K = 1)
  // ---------------------------------------------------------------------------
  localparam logic [7:0] K_COM = 8'hBC;  // K28.5  Comma, starts every ordered set
  localparam logic [7:0] K_STP = 8'hFB;  // K27.7  Start of TLP
  localparam logic [7:0] K_SDP = 8'h5C;  // K28.2  Start of DLLP
  localparam logic [7:0] K_END = 8'hFD;  // K29.7  End of packet
  localparam logic [7:0] K_EDB = 8'hFE;  // K30.7  End of nullified (bad) packet
  localparam logic [7:0] K_PAD = 8'hF7;  // K23.7  PAD (unassigned link / lane number)
  localparam logic [7:0] K_SKP = 8'h1C;  // K28.0  Skip (clock compensation)
  localparam logic [7:0] K_FTS = 8'h3C;  // K28.1  Fast training sequence (unused here)
  localparam logic [7:0] K_IDL = 8'h7C;  // K28.3  Electrical idle (unused here)

  localparam logic [7:0] TS1_ID = 8'h4A; // D10.2  TS1 identifier (symbols 6..15)
  localparam logic [7:0] TS2_ID = 8'h45; // D5.2   TS2 identifier (symbols 6..15)
  localparam logic [7:0] IDLE_D = 8'h00; // D0.0   Logical idle (before scrambling)

  // TS1/TS2 fixed fields we transmit
  localparam logic [7:0] TS_N_FTS   = 8'd255; // N_FTS (L0s not supported, value unused)
  localparam logic [7:0] TS_RATE_ID = 8'h02;  // bit1 = 2.5 GT/s supported
  localparam logic [7:0] TS_TRN_CTL = 8'h00;  // no hot-reset / disable / loopback / no-scramble

  // SKP ordered set interval (spec: 1180..1538 symbol times)
  localparam int SKP_INTERVAL = 1200;

  // ---------------------------------------------------------------------------
  // What the LTSSM asks the PHY transmitter to send
  // ---------------------------------------------------------------------------
  typedef enum logic [2:0] {
    TXM_EIDLE = 3'd0,  // electrical idle (Detect)
    TXM_TS1   = 3'd1,  // continuous TS1 ordered sets
    TXM_TS2   = 3'd2,  // continuous TS2 ordered sets
    TXM_IDLE  = 3'd3,  // logical idle (scrambled D0.0) - Config.Idle / Recovery.Idle
    TXM_L0    = 3'd4   // L0: DLL packets, logical idle when nothing to send
  } tx_mode_e;

  // LTSSM state encoding (exported for debug / waveform viewing)
  typedef enum logic [3:0] {
    LT_DETECT_QUIET  = 4'd0,
    LT_DETECT_ACTIVE = 4'd1,
    LT_POLL_ACTIVE   = 4'd2,
    LT_POLL_CONFIG   = 4'd3,
    LT_CFG_LW_START  = 4'd4,
    LT_CFG_LW_ACCEPT = 4'd5,
    LT_CFG_LN_WAIT   = 4'd6,
    LT_CFG_COMPLETE  = 4'd7,
    LT_CFG_IDLE      = 4'd8,
    LT_L0            = 4'd9,
    LT_REC_LOCK      = 4'd10,
    LT_REC_CFG       = 4'd11,
    LT_REC_IDLE      = 4'd12
  } ltssm_e;

  // ---------------------------------------------------------------------------
  // Flow-control credit types
  // ---------------------------------------------------------------------------
  localparam logic [1:0] FC_P   = 2'd0;   // posted       (MWr, Msg)
  localparam logic [1:0] FC_NP  = 2'd1;   // non-posted   (MRd, IO, Cfg)
  localparam logic [1:0] FC_CPL = 2'd2;   // completions  (Cpl, CplD)

  // ---------------------------------------------------------------------------
  // DLLP type byte (VC0)
  // ---------------------------------------------------------------------------
  localparam logic [7:0] DLLP_ACK       = 8'h00;
  localparam logic [7:0] DLLP_NAK       = 8'h10;
  localparam logic [7:0] DLLP_INITFC1_P = 8'h40;
  localparam logic [7:0] DLLP_INITFC1_N = 8'h50;
  localparam logic [7:0] DLLP_INITFC1_C = 8'h60;
  localparam logic [7:0] DLLP_INITFC2_P = 8'hC0;
  localparam logic [7:0] DLLP_INITFC2_N = 8'hD0;
  localparam logic [7:0] DLLP_INITFC2_C = 8'hE0;
  localparam logic [7:0] DLLP_UPDFC_P   = 8'h80;
  localparam logic [7:0] DLLP_UPDFC_N   = 8'h90;
  localparam logic [7:0] DLLP_UPDFC_C   = 8'hA0;

  // ---------------------------------------------------------------------------
  // TLP header byte 0 = {Fmt[2:0], Type[4:0]}
  // ---------------------------------------------------------------------------
  localparam logic [7:0] TLP_MRD32  = 8'h00;
  localparam logic [7:0] TLP_MRD64  = 8'h20;
  localparam logic [7:0] TLP_MWR32  = 8'h40;
  localparam logic [7:0] TLP_MWR64  = 8'h60;
  localparam logic [7:0] TLP_CFGRD0 = 8'h04;
  localparam logic [7:0] TLP_CFGWR0 = 8'h44;
  localparam logic [7:0] TLP_CPL    = 8'h0A;
  localparam logic [7:0] TLP_CPLD   = 8'h4A;

  // Completion status
  localparam logic [2:0] CPL_SC = 3'b000;  // successful completion
  localparam logic [2:0] CPL_UR = 3'b001;  // unsupported request
  localparam logic [2:0] CPL_CA = 3'b100;  // completer abort

  // ---------------------------------------------------------------------------
  // Scrambler: G(x) = x^16 + x^5 + x^4 + x^3 + 1, seed 16'hFFFF.
  // Galois form, one bit per step. Bit 0 of the byte is scrambled first.
  // ---------------------------------------------------------------------------
  function automatic logic [15:0] lfsr_step(input logic [15:0] s);
    return {s[14:0], 1'b0} ^ (s[15] ? 16'h0039 : 16'h0000);
  endfunction

  // Advance the LFSR by 8 bits (one symbol)
  function automatic logic [15:0] lfsr_adv8(input logic [15:0] s);
    logic [15:0] l;
    l = s;
    for (int i = 0; i < 8; i++) l = lfsr_step(l);
    return l;
  endfunction

  // 8-bit XOR mask produced by the LFSR for the current symbol
  function automatic logic [7:0] lfsr_mask(input logic [15:0] s);
    logic [15:0] l;
    logic [7:0]  m;
    l = s;
    for (int i = 0; i < 8; i++) begin
      m[i] = l[15];
      l    = lfsr_step(l);
    end
    return m;
  endfunction

  // ---------------------------------------------------------------------------
  // DLLP CRC-16: polynomial 0x100B (bit-reflected 0xD008), seed 0xFFFF,
  // processed LSB first. Transmitted value = ~crc, low byte first.
  // ---------------------------------------------------------------------------
  function automatic logic [15:0] crc16_byte(input logic [15:0] crc, input logic [7:0] d);
    logic [15:0] c;
    c = crc ^ {8'h00, d};
    for (int i = 0; i < 8; i++) c = c[0] ? ((c >> 1) ^ 16'hD008) : (c >> 1);
    return c;
  endfunction

  // Complete DLLP: 4 content bytes + CRC. Result[7:0] is the first byte on the
  // wire (b0), result[47:40] the last (CRC high byte).
  function automatic logic [47:0] dllp_pack(input logic [7:0] b0, input logic [7:0] b1,
                                            input logic [7:0] b2, input logic [7:0] b3);
    logic [15:0] c;
    c = 16'hFFFF;
    c = crc16_byte(c, b0);
    c = crc16_byte(c, b1);
    c = crc16_byte(c, b2);
    c = crc16_byte(c, b3);
    c = ~c;
    return {c[15:8], c[7:0], b3, b2, b1, b0};
  endfunction

  // Flow-control DLLP (InitFC1/InitFC2/UpdateFC), scale fields = 0
  function automatic logic [47:0] dllp_pack_fc(input logic [7:0] typ, input logic [7:0] hdr,
                                               input logic [11:0] dat);
    return dllp_pack(typ, {2'b00, hdr[7:2]}, {hdr[1:0], 2'b00, dat[11:8]}, dat[7:0]);
  endfunction

  // ---------------------------------------------------------------------------
  // TLP LCRC-32: polynomial 0x04C11DB7 (bit-reflected 0xEDB88320), seed
  // 0xFFFFFFFF, LSB first, covers {sequence number, TLP}. Transmitted
  // value = ~crc, low byte first (same arithmetic as Ethernet FCS).
  // ---------------------------------------------------------------------------
  function automatic logic [31:0] crc32_byte(input logic [31:0] crc, input logic [7:0] d);
    logic [31:0] c;
    c = crc ^ {24'h0, d};
    for (int i = 0; i < 8; i++) c = c[0] ? ((c >> 1) ^ 32'hEDB88320) : (c >> 1);
    return c;
  endfunction

  // ---------------------------------------------------------------------------
  // TLP classification helpers
  // ---------------------------------------------------------------------------
  // Flow-control class of a TLP from header byte 0
  function automatic logic [1:0] tlp_fc_class(input logic [7:0] fmt_type);
    logic [4:0] t;
    t = fmt_type[4:0];
    if (t == 5'b01010 || t == 5'b01011) return FC_CPL;          // Cpl / CplD / CplLk
    if (t[4:3] == 2'b10)                return FC_P;            // Msg / MsgD
    if ((t == 5'b00000) && fmt_type[6]) return FC_P;            // MWr
    return FC_NP;                                               // MRd, MRdLk, IO, Cfg
  endfunction

  // Data credits (16-byte units) needed for a payload of len_dw DWORDs.
  // A Length field of 0 means 1024 DW.
  function automatic logic [11:0] tlp_data_credits(input logic has_data, input logic [9:0] len_dw);
    logic [10:0] n;
    if (!has_data) return 12'd0;
    n = (len_dw == 10'd0) ? 11'd1024 : {1'b0, len_dw};
    return 12'((n + 11'd3) >> 2);
  endfunction

endpackage
