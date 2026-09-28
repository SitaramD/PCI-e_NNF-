`timescale 1ns/1ps
// =============================================================================
// pcie_ltssm.sv
// -----------------------------------------------------------------------------
// Link Training and Status State Machine - Gen1, x1, reduced feature set.
//
//   Detect.Quiet -> Detect.Active -> Polling.Active -> Polling.Configuration
//   -> Configuration (Linkwidth.Start / Linkwidth.Accept / Lanenum.Wait /
//      Complete / Idle) -> L0 <-> Recovery (RcvrLock / RcvrCfg / Idle)
//
// The same RTL implements both ends of the link:
//   DOWNSTREAM = 1 : downstream port (root port / switch downstream port).
//                    It proposes the link number and assigns lane 0.
//   DOWNSTREAM = 0 : upstream port (endpoint). It echoes what it receives.
//
// Not implemented: Polling.Compliance, L0s, L1, L2, Disabled, Loopback,
// Hot Reset, lane reversal, multi-lane width negotiation, speed change.
// Any timeout returns to Detect.
//
// "Consecutive" counters: rx_cnt counts TS1/TS2 (or idle symbols) that match
// what the current state is waiting for; a received TS that does not match
// resets it to zero.
// =============================================================================
module pcie_ltssm #(
  parameter bit          DOWNSTREAM     = 1'b0,
  parameter logic [7:0]  LINK_NUM       = 8'd0,        // used by a downstream port
  // Timers in core-clock cycles (250 MHz -> 4 ns). Defaults are spec values.
  parameter int unsigned T_DETECT_QUIET = 3_000_000,   // 12 ms
  parameter int unsigned T_2MS          = 500_000,
  parameter int unsigned T_24MS         = 6_000_000,
  parameter int unsigned T_48MS         = 12_000_000,
  parameter int unsigned POLL_TS1_MIN   = 1024         // TS1 to send in Polling.Active
) (
  input  logic                 clk,
  input  logic                 rst,

  input  logic                 rx_present,     // receiver detection result
  input  logic                 retrain_req,    // go to Recovery (DLL replay rollover / user)

  // ---- from PHY RX ----
  input  logic                 rx_ts_valid,
  input  logic                 rx_ts_is_ts2,
  input  logic [7:0]           rx_link,
  input  logic                 rx_link_pad,
  input  logic [7:0]           rx_lane,
  input  logic                 rx_lane_pad,
  input  logic                 rx_idle,
  input  logic                 rx_nonidle,

  // ---- from PHY TX ----
  input  logic                 ts_sent,
  input  logic                 ts_sent_ts2,
  input  logic                 idle_sent,

  // ---- to PHY TX ----
  output pcie_pkg::tx_mode_e   tx_mode,
  output logic [7:0]           tx_link,
  output logic                 tx_link_pad,
  output logic [7:0]           tx_lane,
  output logic                 tx_lane_pad,

  // ---- status ----
  output pcie_pkg::ltssm_e     state,
  output logic                 link_up,        // PCIe "LinkUp": L0 reached, not back in Detect
  output logic                 in_l0,
  output logic [7:0]           link_num
);
  import pcie_pkg::*;

  ltssm_e      nxt;
  logic [31:0] timer;
  logic [11:0] tx_cnt;        // TS sent in this state (saturating)
  logic [4:0]  tx_after;      // TS / idle sent after the first matching reception
  logic        got_rx;        // at least one matching TS / idle received
  logic [4:0]  rx_cnt;        // consecutive matching receptions
  logic [7:0]  link_q, lane_q;

  // ---------------------------------------------------------------------------
  // Does the received TS match what this state is waiting for?
  // ---------------------------------------------------------------------------
  logic ts1, ts2, match;
  assign ts1 = rx_ts_valid && !rx_ts_is_ts2;
  assign ts2 = rx_ts_valid &&  rx_ts_is_ts2;

  always_comb begin
    match = 1'b0;
    case (state)
      LT_POLL_ACTIVE:   match = rx_ts_valid && rx_link_pad && rx_lane_pad;
      LT_POLL_CONFIG:   match = ts2 && rx_link_pad && rx_lane_pad;
      LT_CFG_LW_START:  match = DOWNSTREAM ? (ts1 && !rx_link_pad && rx_link == LINK_NUM)
                                           : (ts1 && !rx_link_pad);
      LT_CFG_LW_ACCEPT: match = ts1 && !rx_link_pad && rx_link == link_q && !rx_lane_pad;
      LT_CFG_LN_WAIT:   match = DOWNSTREAM ? (ts1 && !rx_link_pad && rx_link == link_q &&
                                              !rx_lane_pad && rx_lane == lane_q)
                                           : (ts2 && !rx_link_pad && rx_link == link_q &&
                                              !rx_lane_pad && rx_lane == lane_q);
      LT_CFG_COMPLETE,
      LT_REC_CFG:       match = ts2 && !rx_link_pad && rx_link == link_q &&
                                !rx_lane_pad && rx_lane == lane_q;
      LT_REC_LOCK:      match = rx_ts_valid && !rx_link_pad && rx_link == link_q &&
                                !rx_lane_pad && rx_lane == lane_q;
      default:          match = 1'b0;
    endcase
  end

  // Idle-type states count logical idle symbols instead of TS
  logic idle_state;
  assign idle_state = (state == LT_CFG_IDLE) || (state == LT_REC_IDLE);

  // Transmitted items that count towards "sent after first reception"
  logic tx_tick;
  always_comb begin
    case (state)
      LT_POLL_CONFIG, LT_CFG_COMPLETE, LT_REC_CFG: tx_tick = ts_sent && ts_sent_ts2;
      LT_CFG_IDLE, LT_REC_IDLE:                    tx_tick = idle_sent;
      default:                                     tx_tick = ts_sent && !ts_sent_ts2;
    endcase
  end

  // ---------------------------------------------------------------------------
  // Next state
  // ---------------------------------------------------------------------------
  always_comb begin
    nxt = state;
    case (state)
      LT_DETECT_QUIET:
        if (timer >= T_DETECT_QUIET) nxt = LT_DETECT_ACTIVE;
      LT_DETECT_ACTIVE:
        nxt = rx_present ? LT_POLL_ACTIVE : LT_DETECT_QUIET;
      LT_POLL_ACTIVE:
        if (tx_cnt >= 12'(POLL_TS1_MIN) && rx_cnt >= 5'd8) nxt = LT_POLL_CONFIG;
        else if (timer >= T_24MS)                           nxt = LT_DETECT_QUIET;
      LT_POLL_CONFIG:
        if (rx_cnt >= 5'd8 && tx_after >= 5'd16) nxt = LT_CFG_LW_START;
        else if (timer >= T_48MS)                nxt = LT_DETECT_QUIET;
      LT_CFG_LW_START:
        if (rx_cnt >= 5'd2)        nxt = DOWNSTREAM ? LT_CFG_LN_WAIT : LT_CFG_LW_ACCEPT;
        else if (timer >= T_24MS)  nxt = LT_DETECT_QUIET;
      LT_CFG_LW_ACCEPT:
        if (rx_cnt >= 5'd2)        nxt = LT_CFG_LN_WAIT;
        else if (timer >= T_2MS)   nxt = LT_DETECT_QUIET;
      LT_CFG_LN_WAIT:
        if (rx_cnt >= 5'd2)        nxt = LT_CFG_COMPLETE;
        else if (timer >= T_2MS)   nxt = LT_DETECT_QUIET;
      LT_CFG_COMPLETE:
        if (rx_cnt >= 5'd8 && tx_after >= 5'd16) nxt = LT_CFG_IDLE;
        else if (timer >= T_2MS)                 nxt = LT_DETECT_QUIET;
      LT_CFG_IDLE:
        if (rx_cnt >= 5'd8 && tx_after >= 5'd16) nxt = LT_L0;
        else if (timer >= T_2MS)                 nxt = LT_DETECT_QUIET;
      LT_L0:
        if (rx_ts_valid || retrain_req) nxt = LT_REC_LOCK;
      LT_REC_LOCK:
        if (rx_cnt >= 5'd8)        nxt = LT_REC_CFG;
        else if (timer >= T_24MS)  nxt = LT_DETECT_QUIET;
      LT_REC_CFG:
        if (rx_cnt >= 5'd8 && tx_after >= 5'd16) nxt = LT_REC_IDLE;
        else if (timer >= T_48MS)                nxt = LT_DETECT_QUIET;
      LT_REC_IDLE:
        if (rx_cnt >= 5'd8 && tx_after >= 5'd16) nxt = LT_L0;
        else if (timer >= T_2MS)                 nxt = LT_DETECT_QUIET;
      default:
        nxt = LT_DETECT_QUIET;
    endcase
  end

  // ---------------------------------------------------------------------------
  // State register and counters
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (rst) begin
      state    <= LT_DETECT_QUIET;
      timer    <= '0;
      tx_cnt   <= '0;
      tx_after <= '0;
      got_rx   <= 1'b0;
      rx_cnt   <= '0;
      link_q   <= LINK_NUM;
      lane_q   <= 8'd0;
      link_up  <= 1'b0;
    end else begin
      state <= nxt;

      // Latch link / lane numbers negotiated in Configuration
      if (state == LT_CFG_LW_START && nxt != state) begin
        if (DOWNSTREAM) begin
          link_q <= LINK_NUM;
          lane_q <= 8'd0;
        end else begin
          link_q <= rx_link;
        end
      end
      if (!DOWNSTREAM && state == LT_CFG_LW_ACCEPT && nxt != state) lane_q <= rx_lane;

      if (nxt == LT_L0)           link_up <= 1'b1;
      if (nxt == LT_DETECT_QUIET) link_up <= 1'b0;

      if (nxt != state) begin
        timer    <= '0;
        tx_cnt   <= '0;
        tx_after <= '0;
        got_rx   <= 1'b0;
        rx_cnt   <= '0;
      end else begin
        timer <= timer + 32'd1;
        if (ts_sent && tx_cnt != 12'hFFF) tx_cnt <= tx_cnt + 12'd1;
        if (tx_tick && got_rx && tx_after != 5'h1F) tx_after <= tx_after + 5'd1;

        if (idle_state) begin
          if (rx_idle) begin
            got_rx <= 1'b1;
            if (rx_cnt != 5'h1F) rx_cnt <= rx_cnt + 5'd1;
          end else if (rx_nonidle || rx_ts_valid) begin
            rx_cnt <= '0;
          end
        end else if (rx_ts_valid) begin
          if (match) begin
            got_rx <= 1'b1;
            if (rx_cnt != 5'h1F) rx_cnt <= rx_cnt + 5'd1;
          end else begin
            rx_cnt <= '0;
          end
        end
      end
    end
  end

  // ---------------------------------------------------------------------------
  // What to transmit in each state
  // ---------------------------------------------------------------------------
  always_comb begin
    tx_mode     = TXM_EIDLE;
    tx_link     = link_q;
    tx_link_pad = 1'b1;
    tx_lane     = lane_q;
    tx_lane_pad = 1'b1;
    case (state)
      LT_DETECT_QUIET, LT_DETECT_ACTIVE: tx_mode = TXM_EIDLE;
      LT_POLL_ACTIVE:   tx_mode = TXM_TS1;
      LT_POLL_CONFIG:   tx_mode = TXM_TS2;
      LT_CFG_LW_START: begin
        tx_mode     = TXM_TS1;
        tx_link     = LINK_NUM;
        tx_link_pad = !DOWNSTREAM;              // downstream proposes a link number
      end
      LT_CFG_LW_ACCEPT: begin
        tx_mode     = TXM_TS1;
        tx_link_pad = 1'b0;
      end
      LT_CFG_LN_WAIT, LT_REC_LOCK: begin
        tx_mode     = TXM_TS1;
        tx_link_pad = 1'b0;
        tx_lane_pad = 1'b0;
      end
      LT_CFG_COMPLETE, LT_REC_CFG: begin
        tx_mode     = TXM_TS2;
        tx_link_pad = 1'b0;
        tx_lane_pad = 1'b0;
      end
      LT_CFG_IDLE, LT_REC_IDLE: tx_mode = TXM_IDLE;
      LT_L0:                    tx_mode = TXM_L0;
      default:                  tx_mode = TXM_EIDLE;
    endcase
  end

  assign in_l0    = (state == LT_L0);
  assign link_num = link_q;

endmodule
