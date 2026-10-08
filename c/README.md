# ADF5355 no-OS example

adf5355_noos_example.c is a one-shot RFOUTA program using Analog Devices'
current [no-OS ADF5355 driver](https://github.com/analogdevicesinc/no-OS/tree/main/drivers/frequency/adf5355).
The ADI module supplies the device lifecycle and Linux SPI abstraction. The
application supplies a Python-compatible register layer, because the immutable
ADI source has known ADF5355 Register 2, Register 6, and Register 7
differences.

## Raspberry Pi wiring

The program opens /dev/spidev0.0, so use the standard SPI0 connections:

| Raspberry Pi | ADF5355 board |
| --- | --- |
| GPIO11, physical pin 23 | CLK |
| GPIO10, physical pin 19 | DATA |
| GPIO8, physical pin 24 (SPI0 CE0) | LE |
| GND | GND |

Hold the ADF5355 **CE** pin high from the board's 3.3 V rail. Do not use
SPI0 CE0 as CE: it is the register latch-enable (LE) signal. The example
configures MUXOUT for digital lock detect, but does not read it.

## Build

On the Raspberry Pi, with SPI0 enabled:

~~~sh
git clone --depth 1 https://github.com/analogdevicesinc/no-OS ../no-OS
cd c
make
~~~

If the no-OS checkout is elsewhere, supply its path:

~~~sh
make NO_OS_DIR=/path/to/no-OS
~~~

## Run

~~~sh
sudo ./adf5355_noos_example
sudo ./adf5355_noos_example 1000000000
~~~

Use --help to see every default, fixed parameter, and supported option:

~~~sh
./adf5355_noos_example --help
~~~

Normal successful programming is silent. Use --verbose to print the selected
configuration and the exact corrected 32-bit words transmitted to the ADF5355
in the same Register 12 through Register 0 initialization order and Step/13
layout as the Python programs. The ADI Linux backend does not report individual
SPI-transfer timestamps, so the C report prints its start, end, transfer, and
gap fields as n/a.

Use --dry-run to calculate and report those same words without opening SPI:

~~~sh
./adf5355_noos_example --dry-run --rf-output-hz 1000000000 \
  --rf-output-power 2 --muxout digital
~~~

The default is RFOUTA = 2.1 GHz, from the board's 125 MHz reference. It
selects a 3.125 mA charge-pump current and enables fractional-N negative
bleed where the ADF5355 permits it. The default channel spacing is 200 kHz,
matching the Python program. For example:

~~~sh
sudo ./adf5355_noos_example --rf-output-hz 1002500000 \
  --reference-hz 125000000 --charge-pump-current-ua 3125 \
  --channel-spacing-hz 200000 --rf-output-power 2 --muxout digital --verbose
~~~

The compatibility layer chooses R counter, MOD2, and bleed-current values
using the Python program's rules. This C utility does not provide the Python
programs' Raspberry Pi GPIO digital-lock wait or lock-time measurement.

This is a starting point, not a lock/settling-time measurement application.
The physical loop filter must be appropriate for the selected charge-pump
current before using it for RF-quality measurements.
