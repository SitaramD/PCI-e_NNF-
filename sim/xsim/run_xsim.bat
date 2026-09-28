@echo off
REM ============================================================================
REM  XSim (Vivado simulator) flow for Windows.
REM
REM  Usage (from any folder):
REM     sim\xsim\run_xsim.bat                     system test  (tb_pcie_top)
REM     sim\xsim\run_xsim.bat tb_8b10b            8b/10b unit test
REM     sim\xsim\run_xsim.bat tb_crc_scrambler    CRC + scrambler unit test
REM     sim\xsim\run_xsim.bat tb_phy_link         PHY link-training test
REM     sim\xsim\run_xsim.bat tb_pcie_top gui     open the waveform GUI
REM
REM  Vivado must be on PATH. Either run this from the "Vivado 2022.2 Tcl Shell"
REM  or first run:   call D:\Xilinx\Vivado\2022.2\settings64.bat
REM  (adjust the path to where Vivado is installed).
REM ============================================================================
setlocal
set TOP=%1
if "%TOP%"=="" set TOP=tb_pcie_top

REM Go to the repository root (two levels above this script)
cd /d "%~dp0..\.."
if not exist build\xsim mkdir build\xsim
cd build\xsim

echo ==== xvlog (compile)
call xvlog --sv ^
  -f ..\..\sim\xsim\files_xsim.f ^
  --log xvlog.log
if errorlevel 1 goto :fail

echo ==== xelab (elaborate %TOP%)
call xelab %TOP% -debug typical -s %TOP%_snap --log xelab.log
if errorlevel 1 goto :fail

echo ==== xsim (run)
if /I "%2"=="gui" (
  call xsim %TOP%_snap -gui --log xsim.log
) else (
  call xsim %TOP%_snap -R --log xsim.log
)
if errorlevel 1 goto :fail
goto :eof

:fail
echo **** XSim flow failed - see the .log files in build\xsim
exit /b 1
