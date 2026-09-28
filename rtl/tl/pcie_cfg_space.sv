`timescale 1ns/1ps
// =============================================================================
// pcie_cfg_space.sv
// -----------------------------------------------------------------------------
// Type 0 (endpoint) configuration space, 256 bytes (legacy PCI-compatible
// region). Extended space (0x100-0xFFF) reads as zero.
//
//   0x00  Device ID | Vendor ID
//   0x04  Status | Command            Command: mem enable, bus master, ...
//   0x08  Class code | Revision ID
//   0x0C  BIST | Header type | Latency | Cache line size
//   0x10  BAR0  32-bit, non-prefetchable memory, 2**BAR0_BITS bytes
//   0x14..0x24  BAR1..BAR5 = 0 (not implemented)
//   0x2C  Subsystem ID | Subsystem vendor ID
//   0x34  Capabilities pointer = 0x40
//   0x3C  Max_Lat | Min_Gnt | Interrupt pin (0 = none) | Interrupt line
//   0x40  Power Management capability      (next = 0x60)
//   0x44  PMCSR
//   0x60  PCI Express capability           (next = 0x00, end of list)
//   0x64  Device Capabilities   (Max payload 128 B)
//   0x68  Device Status | Device Control
//   0x6C  Link Capabilities     (2.5 GT/s, x1)
//   0x70  Link Status | Link Control
//   0x84  Device Capabilities 2 ... 0x90 Link Control 2
//
// BAR sizing: software writes 0xFFFFFFFF and reads back the size mask,
// because only bits [31:BAR0_BITS] are writable.
// =============================================================================
module pcie_cfg_space #(
  parameter logic [15:0] VENDOR_ID    = 16'h1234,   // placeholder - use your own PCI-SIG ID
  parameter logic [15:0] DEVICE_ID    = 16'h0001,
  parameter logic [7:0]  REVISION_ID  = 8'h01,
  parameter logic [23:0] CLASS_CODE   = 24'h120000, // processing accelerator
  parameter logic [15:0] SUBSYS_VID   = 16'h1234,
  parameter logic [15:0] SUBSYS_ID    = 16'h0001,
  parameter int          BAR0_BITS    = 12          // BAR0 size = 4 KB
) (
  input  logic        clk,
  input  logic        rst,

  input  logic [9:0]  addr,          // DWORD index {ext reg, reg}
  input  logic        wr_en,
  input  logic [31:0] wr_data,
  input  logic [3:0]  wr_be,
  output logic [31:0] rd_data,       // combinational read

  output logic        mem_enable,
  output logic        bus_master,
  output logic [31:0] bar0,
  output logic [2:0]  max_payload,   // Device Control MPS field
  output logic [2:0]  max_read_req
);

  // Writable registers
  logic [15:0] command;
  logic [7:0]  cache_line, int_line;
  logic [31:0] bar0_q;
  logic [1:0]  pm_state;
  logic [15:0] dev_ctl;
  logic [15:0] link_ctl;
  logic [3:0]  link_ctl2_speed;

  localparam logic [31:0] BAR0_MASK = ~((32'd1 << BAR0_BITS) - 32'd1);

  // ---- read mux ----
  always_comb begin
    rd_data = 32'h0;
    if (addr[9:6] == 4'd0) begin
      case (addr[5:0])
        6'h00: rd_data = {DEVICE_ID, VENDOR_ID};
        6'h01: rd_data = {16'h0010, command};                // status: capabilities list
        6'h02: rd_data = {CLASS_CODE, REVISION_ID};
        6'h03: rd_data = {8'h00, 8'h00, 8'h00, cache_line};  // header type 0
        6'h04: rd_data = bar0_q;                             // [3:0] = 0000: 32-bit mem, non-prefetch
        6'h0B: rd_data = {SUBSYS_ID, SUBSYS_VID};
        6'h0D: rd_data = {24'h0, 8'h40};                     // capabilities pointer
        6'h0F: rd_data = {8'h00, 8'h00, 8'h00, int_line};    // interrupt pin = 0
        // Power Management capability @0x40
        6'h10: rd_data = {16'h0003, 8'h60, 8'h01};           // PMC v1.2, next 0x60, ID 01
        6'h11: rd_data = {16'h0000, 12'h000, 1'b1, 1'b0, pm_state}; // No_Soft_Reset = 1
        // PCI Express capability @0x60
        6'h18: rd_data = {16'h0002, 8'h00, 8'h10};           // v2, endpoint, next 0, ID 10h
        6'h19: rd_data = 32'h0000_8000;                      // DevCap: MPS 128B, role-based err
        6'h1A: rd_data = {16'h0000, dev_ctl};                // DevStatus | DevCtl
        6'h1B: rd_data = 32'h0000_0011;                      // LinkCap: 2.5GT/s, x1
        6'h1C: rd_data = {16'h0011, link_ctl};               // LinkStatus: 2.5GT/s x1 | LinkCtl
        6'h21: rd_data = 32'h0000_0000;                      // DevCap2
        6'h22: rd_data = 32'h0000_0000;                      // DevCtl2
        6'h23: rd_data = 32'h0000_0002;                      // LinkCap2: 2.5 GT/s supported
        6'h24: rd_data = {28'h0, link_ctl2_speed};           // LinkCtl2: target speed
        default: rd_data = 32'h0;
      endcase
    end
  end

  // ---- byte-enable write helper ----
  function automatic logic [31:0] merge(input logic [31:0] old, input logic [31:0] nw,
                                        input logic [3:0] be, input logic [31:0] wmask);
    logic [31:0] m;
    m = {{8{be[3]}}, {8{be[2]}}, {8{be[1]}}, {8{be[0]}}} & wmask;
    return (old & ~m) | (nw & m);
  endfunction

  // Writable-bit mask for each register
  logic [31:0] wmask, nv;
  always_comb begin
    case (addr[5:0])
      6'h01:   wmask = 32'h0000_0546;   // Command: mem, bus master, parity, SERR, INTx dis
      6'h03:   wmask = 32'h0000_00FF;   // cache line size
      6'h04:   wmask = BAR0_MASK;       // BAR0 address bits
      6'h0F:   wmask = 32'h0000_00FF;   // interrupt line
      6'h11:   wmask = 32'h0000_0003;   // PMCSR power state
      6'h1A:   wmask = 32'h0000_7CFF;   // Device Control
      6'h1C:   wmask = 32'h0000_00CB;   // Link Control
      6'h24:   wmask = 32'h0000_000F;   // Link Control 2 target speed
      default: wmask = 32'h0;
    endcase
    if (addr[9:6] != 4'd0) wmask = 32'h0;
    nv = merge(rd_data, wr_data, wr_be, wmask);   // new register value
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      command         <= 16'h0000;
      cache_line      <= 8'h00;
      int_line        <= 8'h00;
      bar0_q          <= 32'h0;
      pm_state        <= 2'b00;
      dev_ctl         <= 16'h2810;   // MRRS 512B, no-snoop en, relaxed ordering en, MPS 128B
      link_ctl        <= 16'h0000;
      link_ctl2_speed <= 4'h1;
    end else if (wr_en && addr[9:6] == 4'd0) begin
      case (addr[5:0])
        6'h01: command         <= nv[15:0];
        6'h03: cache_line      <= nv[7:0];
        6'h04: bar0_q          <= nv;
        6'h0F: int_line        <= nv[7:0];
        6'h11: pm_state        <= nv[1:0];
        6'h1A: dev_ctl         <= nv[15:0];
        6'h1C: link_ctl        <= nv[15:0];
        6'h24: link_ctl2_speed <= nv[3:0];
        default: ;
      endcase
    end
  end

  assign mem_enable   = command[1];
  assign bus_master   = command[2];
  assign bar0         = bar0_q;
  assign max_payload  = dev_ctl[7:5];
  assign max_read_req = dev_ctl[14:12];

endmodule
