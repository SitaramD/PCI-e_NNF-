# =============================================================================
# Verilator flow (Linux / WSL). Needs Verilator >= 5.0 (uses --timing).
#
#   make            run every test
#   make top        system test   (tb_pcie_top)
#   make phy        PHY link test (tb_phy_link)
#   make 8b10b      8b/10b unit test
#   make crc        CRC + scrambler unit test
#   make clean
#
# Run from this directory:  cd sim/verilator && make
# =============================================================================
ROOT    := ../..
BUILD   := $(ROOT)/build/verilator
VFLAGS  := --binary --timing -j 0 -Wno-fatal -Wno-lint -Wno-style -Wno-SYNCASYNCNET
RTL     := $(addprefix $(ROOT)/,$(shell cat $(ROOT)/sim/filelist_rtl.f))
TBCOMMON:= $(ROOT)/tb/models/pcie_channel.sv $(ROOT)/tb/rc/pcie_rc_bfm.sv

.PHONY: all top phy 8b10b crc clean

all: crc 8b10b phy top

top:
	mkdir -p $(BUILD)
	verilator $(VFLAGS) --Mdir $(BUILD)/top $(RTL) $(TBCOMMON) $(ROOT)/tb/top/tb_pcie_top.sv \
	  --top-module tb_pcie_top -o sim
	$(BUILD)/top/sim

phy:
	mkdir -p $(BUILD)
	verilator $(VFLAGS) --Mdir $(BUILD)/phy $(RTL) $(ROOT)/tb/models/pcie_channel.sv \
	  $(ROOT)/tb/unit/tb_phy_link.sv --top-module tb_phy_link -o sim
	$(BUILD)/phy/sim

8b10b:
	mkdir -p $(BUILD)
	verilator $(VFLAGS) --Mdir $(BUILD)/8b10b $(ROOT)/rtl/common/pcie_pkg.sv \
	  $(ROOT)/rtl/phy/pcie_8b10b_enc.sv $(ROOT)/rtl/phy/pcie_8b10b_dec.sv \
	  $(ROOT)/tb/unit/tb_8b10b.sv --top-module tb_8b10b -o sim
	$(BUILD)/8b10b/sim

crc:
	mkdir -p $(BUILD)
	verilator $(VFLAGS) --Mdir $(BUILD)/crc $(ROOT)/rtl/common/pcie_pkg.sv \
	  $(ROOT)/tb/unit/tb_crc_scrambler.sv --top-module tb_crc_scrambler -o sim
	$(BUILD)/crc/sim

clean:
	rm -rf $(BUILD)
