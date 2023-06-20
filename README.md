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

# Documentation on the design 
- [Course - episode I](https://github.com/BrunoLevy/learn-fpga/blob/master/FemtoRV/TUTORIALS/FROM_BLINKER_TO_RISCV/README.md)
- [Course - episode II](https://github.com/BrunoLevy/learn-fpga/blob/master/FemtoRV/TUTORIALS/FROM_BLINKER_TO_RISCV/PIPELINE.md)

