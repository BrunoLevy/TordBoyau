# TordBoyau
A pipelined RISC-V processor

![](Images/tinyraytracer.png)

# Instructions
Included Vivado project is configured for an ARTY A35T

- Load project in Vivado, synthesize, create bitstream, send to device
- Included firmware computes a raytracing image and displays it on the TTY using ANSI codes
- Connect to terminal using 1000000 bauds (see `terminal.sh`, adapt to your setup)

Instructions for other boards / Yosys-NextPNR are coming

# Configuration
Several parameters can be configured in `soc.v`:

| Name               | Description                                                                 |
|--------------------|-----------------------------------------------------------------------------|
|`CPU_FREQ`          | Depending on the options, timings will validate around 100-120 MHz          |
|`CONFIG_PC_PREDICT` | Enables `D`-`F` path, used by branch prediction and return address stack    |
|`CONFIG_RAS`        | Enables return address stack                                                |
|`CONFIG_GSHARE`     | GSHARE branch predictor (uses BTFNT if not set)                             |
|`CONFIG_RV32M`      | RV32M instruction set (`MUL`,`DIV`,`REM`).                                  |
|`CONFIG_DEBUG`      | Enables built-in debugger/disassembler (used in simulation)                 |
|`CONFIG_INITIALIZE` | Initializes register file and BHT (required by Icarus and some synth tools) |

# Firmware

Firmware takes the form of two files, `PROGROM.hex` that contains
code, and `DATARAM.hex` that contains variables initialization. The
included firmware computes an image in raytracing and sends it to the
TTY (1000000 bauds). It also measures the average CPI, and a
'raystones' performance score (pixels/s/MHz).

Some precompiled firmwares are available in `PRECOMPILED_FIRMWARE/<arch>/<progname>/PROGROM.hex` and `DATARAM.hex`.
To use one of them, just copy `PROGROM.hex` and `DATARAM.hex` in
`TordBoyau/` (the same directory that contains `soc.v`) and re-synthesize (or launch simulation).

Other firmwares can be compiled, see [learn-fpga, pipeline
tutorial](https://github.com/BrunoLevy/learn-fpga/blob/master/FemtoRV/TUTORIALS/FROM_BLINKER_TO_RISCV/PIPELINE.md)
for more details (`PROGROM.hex` and `DATARAM.hex` are portable between
both projects, just make sure you target the same instruction set
(RV32I or RV32IM). You will need also to remove all the lines of zeroes
after line 1024 in `DATARAM.hex` (the core in `learn-fpga` is configured with
64kB of data ram, and here it is 16kB, which suffices for most examples).

# Performance (RV32I) (A35T/Vivado)

| branch prediction    | CoreMarks/MHz  |  DMips/MHz   | Raystones | LUTs  | FFs   | MaxFreq |
|----------------------|----------------|--------------|-----------|-------|-------|---------|
|     none             |  0.928         |   1.298      |  5.665    | 909   | 517   | 125 MHz |
| static (BTFNT)       |  1.118         |   1.488      |  6.633    | 938   | 516   | 125 MHz |
| static + RAS         |  1.147         |   1.528      |  6.795    | 1040  | 676   | 105 MHz |
| gshare               |  1.124         |   1.562      |  7.186    | 1297  | 547   | 120 MHz |
| gshare + RAS         |  1.153         |   1.606      |  7.375    | 1388  | 711   | 100 MHz |

# Performance (RV32IM) (A35T/Vivado)

| branch prediction    | CoreMarks/MHz  |  DMips/MHz   | Raystones | LUTs  | FFs   | MaxFreq  |
|----------------------|----------------|--------------|-----------|-------|-------|----------|
|     none             | 2.387          |  1.341       | 15.296    | 1368  | 681   | < 80 MHz |
| static (BTFNT)       | 2.763          |  1.545       | 16.097    | 1363  | 680   | < 80 MHz |
| static + RAS         | 2.790          |  1.579       | 16.476    | 1478  | 840   | < 80 MHz |
| gshare               | 2.837          |  1.597       | 17.753    | 1760  | 711   | < 80 MHz |
| gshare + RAS         | 2.866          |  1.634       | 18.215    | 1801  | 875   | < 80 MHz |

- Vivado complains that it fails to meet timings even at 80 MHz, to be investigated...
- However, in practice, it seems to work at 140 MHz with the largest configuration (`gshare + RAS`). CoreMarks
  and Dhrystones both validate correct operation, and RayStones generates the correct image.


# Debugger / disassembler

![](Images/debugger.png)

Simulation can be started using `run_verilator.sh`. If `CONFIG_DEBUG` is set in `soc.v`, then one can see the
content of the pipeline stages, the hazards, register forwarding, branch prediction, return address stack.
It is also possible to create "breakpoints", by defining the `breakpoint` signal in `TordBoyau5.v` (default
breakpoint is on TTY character display). 

# Sequential pipeline

A completely sequential version `TordBoyau5_sequential` is included. It has a state machine that executes each
stage sequentially, without hazard nor data forwarding. It is there to estimate an upper boundary of
what maxfreq one can expect on a given FPGA. On the ARTY, it validates at 150 MHz (still works at 160 MHz).

# Documentation on the design 

- [Course - episode I](https://github.com/BrunoLevy/learn-fpga/blob/master/FemtoRV/TUTORIALS/FROM_BLINKER_TO_RISCV/README.md)
- [Course - episode II](https://github.com/BrunoLevy/learn-fpga/blob/master/FemtoRV/TUTORIALS/FROM_BLINKER_TO_RISCV/PIPELINE.md)

# Next steps / TODO

- Try to validate RV32IM at 140 MHz or so
   - Activating RAS makes maxfreq drop, to be investigated.
   - Activating RV32M makes maxfreq dramatically drop, to be investigated.
   - I don't have a Branch Target Buffer, I'm always computing the branch target, maybe it is not good.
- Write Amaranth glue code for LiteX, so that we can
  [run Doom](https://github.com/BrunoLevy/learn-fpga/tree/master/LiteX/software/Doom) on it.
  Doom already works for the simpler non-pipelined
  [FemtoRV](https://github.com/BrunoLevy/learn-fpga/tree/master/FemtoRV) cores. Here we need to
  adapt LiteX cache and plug it onto PROGROM and DATARAM.
- It seems that alignment logic for load and store plays a role in the critical path.
  A 6 stages pipeline may be more optimal, to be tested.
- Write scripts to synthesize using `yosys` and `nextpnr-xilinx`
- Write scripts for other boards (ULX3S, orange crab, ...)