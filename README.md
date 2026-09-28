# PCIe Gen1 x1 Endpoint — from scratch in SystemVerilog using Claude Opus 5.5

A PCI Express **Gen1 (2.5 GT/s), x1** endpoint where every layer is written in RTL. No vendor PCIe hard block or IP is used:

| Layer | What is implemented |
|---|---|
| **Physical (logical sub-block)** | 8b/10b encode/decode, comma alignment, elastic buffer with SKP add/drop, scrambler/descrambler, TS1/TS2/SKP ordered sets, STP/SDP/END framing, **LTSSM** (Detect → Polling → Configuration → L0 ↔ Recovery) |
| **Data Link** | DL state machine (Inactive/Init1/Init2/Active), InitFC/UpdateFC flow control, 12-bit sequence numbers, LCRC-32, DLLP CRC-16, Ack/Nak, replay buffer, replay timer, REPLAY_NUM rollover → retrain |
| **Transaction** | Type 0 config space (BAR0, PM + PCIe capabilities), CfgRd0/CfgWr0, MRd/MWr 32/64-bit → **AXI4-Lite master**, completions (SC / UR / CA), byte enables, completer-ID capture |

The same PHY and DLL RTL also builds as a **downstream port**. The testbench uses it to make a root-complex model, so both ends of the link are real RTL.

> Status: **simulation-verified** (XSim / Verilator). It has not been tested against real PCIe silicon. The CRCs and 8b/10b tables are checked against independent reference models (see *Verification*).

---

## Repository layout

```
pcie_gen1_x1/
├── README.md                     <- this file
├── docs/
│   └── architecture.md           <- block diagrams, packet formats, LTSSM, design notes
├── rtl/                          <- synthesizable design
│   ├── common/
│   │   └── pcie_pkg.sv           <- constants, K-codes, scrambler, CRC-16/32, helpers
│   ├── phy/
│   │   ├── pcie_8b10b_enc.sv     <- 8b/10b encoder
│   │   ├── pcie_8b10b_dec.sv     <- 8b/10b decoder (+ code / disparity errors)
│   │   ├── pcie_comma_align.sv   <- K28.5 word alignment
│   │   ├── pcie_elastic_buf.sv   <- clock compensation FIFO (SKP add/drop)
│   │   ├── pcie_phy_tx.sv        <- ordered sets, SKP scheduling, framing, scrambler
│   │   ├── pcie_phy_rx.sv        <- descrambler, TS1/TS2 parser, de-framer
│   │   ├── pcie_ltssm.sv         <- link training state machine
│   │   └── pcie_phy_top.sv       <- PHY wrapper
│   ├── dll/
│   │   ├── pcie_dll_tx.sv        <- replay buffer, seq#, LCRC, Ack/Nak/FC DLLPs
│   │   ├── pcie_dll_rx.sv        <- DLLP/TLP check, Ack/Nak generation, RX buffer
│   │   └── pcie_dll_top.sv       <- DL state machine + flow-control credits
│   ├── tl/
│   │   ├── pcie_cfg_space.sv     <- Type 0 configuration header + capabilities
│   │   └── pcie_tl_ep.sv         <- TLP decode, BAR0 -> AXI4-Lite, completions
│   ├── top/
│   │   └── pcie_ep_top.sv        <- endpoint top: PHY + DLL + TL
│   └── example/
│       └── axil_regfile.sv       <- example AXI4-Lite register file behind BAR0
├── tb/                           <- simulation only
│   ├── models/
│   │   └── pcie_channel.sv       <- lane model: bit offset, latency, bit-error injection
│   ├── rc/
│   │   └── pcie_rc_bfm.sv        <- root complex: RTL PHY+DLL (downstream) + TLP tasks
│   ├── unit/
│   │   ├── tb_8b10b.sv           <- 8b/10b vs golden vectors (generated)
│   │   ├── tb_crc_scrambler.sv   <- scrambler vs spec sequence, CRC-16/LCRC vs references
│   │   └── tb_phy_link.sv        <- two PHYs train to L0
│   └── top/
│       └── tb_pcie_top.sv        <- full system test (13 tests, ~2000 checks)
├── sim/
│   ├── filelist_rtl.f            <- RTL compile order (package first)
│   ├── filelist_tb.f             <- system-test testbench files
│   ├── xsim/
│   │   ├── run_xsim.bat          <- Windows: Vivado XSim flow
│   │   ├── run_xsim.sh           <- Linux:   Vivado XSim flow
│   │   └── files_xsim.f          <- file list used by the XSim scripts
│   └── verilator/
│       └── Makefile              <- Verilator 5 flow (Linux / WSL)
├── scripts/
│   ├── create_vivado_project.tcl <- builds a Vivado GUI project
│   └── gen_8b10b_vectors.py      <- regenerates tb/unit/tb_8b10b.sv
└── .gitignore
```

---

## Running the simulations

### Option A: XSim on Windows (Vivado 2022.2)

Open **Vivado 2022.2 Tcl Shell** (or a `cmd` window after `call D:\Xilinx\Vivado\2022.2\settings64.bat`), then:

```bat
cd <path>\pcie_gen1_x1
sim\xsim\run_xsim.bat                     :: full system test
sim\xsim\run_xsim.bat tb_phy_link         :: PHY link training only
sim\xsim\run_xsim.bat tb_8b10b            :: 8b/10b unit test
sim\xsim\run_xsim.bat tb_crc_scrambler    :: CRC + scrambler unit test
sim\xsim\run_xsim.bat tb_pcie_top gui     :: open waveforms in the XSim GUI
```

Logs and the `xsim.dir` snapshot go to `build\xsim\`.

### Option B: Vivado GUI project

```bat
vivado -mode batch -source scripts/create_vivado_project.tcl
```

Open `build/vivado/pcie_gen1_x1.xpr`, then run **Run Simulation → Run Behavioral Simulation**. To run a different test, change the simulation top in *Simulation Settings*.

### Option C: Verilator (Linux / WSL)

```bash
cd sim/verilator
make            # runs crc, 8b10b, phy and top
make top        # only the system test
```

### Expected output (system test)

```
    0.137 us ==== 1: link training
    0.910 us   EP LTSSM -> LT_POLL_ACTIVE
   66.662 us   EP LTSSM -> LT_POLL_CONFIG
   ...
   70.626 us   EP LTSSM -> LT_L0
   71.021 us   link up: L0 + DL_Active on both ends
   ...
   91.511 us   endpoint replays: 0 -> 1
   94.685 us   endpoint bad TLPs: 0 -> 1
   ...
=====================================================
  checks: 2011   errors: 0
  EP: bad TLPs 1, replays 1 | RC: bad TLPs 1, replays 1
  *** TEST PASSED ***
=====================================================
```

Polling.Active takes about 65 µs because the spec requires at least 1024 TS1 ordered sets.

---

## Verification

| Test | What it proves |
|---|---|
| `tb_8b10b` | Encoder output equals an **independent** 8b/10b implementation (`encdec8b10b` Python package) for all 256 D-codes, all 12 K-codes and 500 random symbols. Decoder round-trips with no errors. Run length ≤ 5 and DC balance hold. Invalid codes are flagged. |
| `tb_crc_scrambler` | Scrambler output equals the published PCIe sequence `FF 17 C0 14 B2 E7 02 82 …`. DLLP CRC-16 equals Alex Forencich's `cocotbext-pcie` model. LCRC-32 equals `zlib.crc32`. |
| `tb_phy_link` | Downstream and upstream PHYs train to L0 over two lanes with different bit offsets and a **500 ppm clock difference**, then stay in L0 (the elastic buffers add and drop SKPs). |
| `tb_pcie_top` | Enumeration (IDs, BAR sizing, Command, capability walk), MRd/MWr through BAR0, bursts, byte enables and Byte Count, UR on BAR miss, **bit errors in both directions → Nak → replay**, forced Recovery, and 200 random reads/writes against a reference model. |

---

## Interfaces of `pcie_ep_top`

| Group | Signals | Notes |
|---|---|---|
| Clock/reset | `clk`, `rst` | 250 MHz, one symbol per clock. Synchronous active-high reset. |
| Serial side | `tx_sym[9:0]`, `tx_eidle`, `rx_clk`, `rx_raw[9:0]`, `rx_present` | 10-bit parallel, bit 0 first on the wire. This is what a GTH/GTY transceiver gives you in raw mode (8b/10b and comma alignment bypassed). |
| BAR0 | `m_axil_*` | AXI4-Lite master, 32-bit, one transaction at a time. BAR0 offset appears on `awaddr`/`araddr`. |
| Status | `ltssm_state`, `link_up`, `dl_up`, `completer_id`, `stat_*` | For debug / ILA. |

Key parameters: `VENDOR_ID`, `DEVICE_ID`, `CLASS_CODE`, `BAR0_BITS` (size = 2^BAR0_BITS bytes). The LTSSM timers default to spec values; testbenches shorten them.

---

## Known limitations (by design, for a first version)

- Gen1 only, x1 only. No speed change, lane reversal or multi-lane deskew.
- No Polling.Compliance, L0s, L1/L2 power states, Hot Reset, Disabled or Loopback. Every timeout goes back to Detect.
- Receiver detection and electrical idle are analog functions; here `rx_present` is an input.
- Endpoint is completer-only: no bus mastering / DMA, no MSI or INTx.
- MRd supports up to 32 DW (128 B, the Max_Payload_Size advertised). Larger reads get Completer Abort.
- No ECRC generation. Received ECRC (TD = 1) is skipped, not checked.
- No AER. Error statistics are exposed as counters.
- `0x1234` is a placeholder Vendor ID. A real product needs a PCI-SIG-assigned ID.

## Roadmap

1. Run on hardware: Alveo U55C GTY in raw mode, Gen1 x1 to a Linux host (PERST#, REFCLK, CDR lock).
2. MSI + bus-master DMA engine (host memory writes of market data).
3. x4 link with lane-to-lane deskew.
4. Gen2 (5 GT/s) speed change through Recovery.

## References

- MindShare, *PCI Express Technology 3.0*, M. Jackson & R. Budruk
- Intel, *PHY Interface for the PCI Express Architecture (PIPE)*
- PCI-SIG, *PCI Express Base Specification* (membership required)
- A. Forencich, `cocotbext-pcie` (reference DLLP model used in `tb_crc_scrambler`)
