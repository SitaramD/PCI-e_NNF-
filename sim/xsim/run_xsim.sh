#!/usr/bin/env bash
# ============================================================================
#  XSim (Vivado simulator) flow for Linux.
#    ./sim/xsim/run_xsim.sh [tb_pcie_top|tb_8b10b|tb_crc_scrambler|tb_phy_link] [gui]
#  Vivado must be on PATH:  source /tools/Xilinx/Vivado/2022.2/settings64.sh
# ============================================================================
set -e
TOP=${1:-tb_pcie_top}
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
mkdir -p "$ROOT/build/xsim"
cd "$ROOT/build/xsim"

echo "==== xvlog (compile)"
xvlog --sv -f "$ROOT/sim/xsim/files_xsim.f" --log xvlog.log
echo "==== xelab (elaborate $TOP)"
xelab "$TOP" -debug typical -s "${TOP}_snap" --log xelab.log
echo "==== xsim (run)"
if [ "$2" = "gui" ]; then
  xsim "${TOP}_snap" -gui --log xsim.log
else
  xsim "${TOP}_snap" -R --log xsim.log
fi
