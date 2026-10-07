# ADF5355 no-OS example

adf5355_noos_example.c is a one-shot RFOUTA program using Analog Devices'
current [no-OS ADF5355 driver](https://github.com/analogdevicesinc/no-OS/tree/main/drivers/frequency/adf5355).
The ADI module performs the register calculations and writes the registers over
SPI; this repository supplies only the Linux userspace application and build
recipe.

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

The default is RFOUTA = 2.1 GHz, from the board's 125 MHz reference. It
selects a 3.125 mA charge-pump current and enables fractional-N negative
bleed. The no-OS driver automatically disables negative bleed for integer-N
frequencies and for PFD frequencies above 100 MHz.

This is a starting point, not a lock/settling-time measurement application.
The physical loop filter must be appropriate for the selected charge-pump
current before using it for RF-quality measurements.
