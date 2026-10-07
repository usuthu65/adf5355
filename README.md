# ADF5355 Raspberry Pi examples

This repository contains Raspberry Pi userspace examples for an ADF5355
wideband synthesizer board labeled "ADF535X EVAL NWDZ V2.0", provided
with a default 125 MHz crystal basd onboard XO.

The schematic is likely similar or identical to 
http://gm8bjf.joomla.com/images/pdf/ADF5355_sch.pdf.

## Layout

- python/ — direct Python SPI programs:
  - adf5355_pi.py: one-shot frequency programming.
  - adf5355_pi_sweep.py: stepped-sweep and digital-lock timing work.
- c/ — a C one-shot RFOUTA example built on the official Analog Devices
  no-OS ADF5355 driver.

## Python

The Python programs use SPI0:

| Raspberry Pi | ADF5355 board |
| --- | --- |
| GPIO11, physical pin 23 | CLK |
| GPIO10, physical pin 19 | DATA |
| GPIO8, physical pin 24 (SPI0 CE0) | LE |
| GND | GND |

For example:

~~~sh
cd python
python3 adf5355_pi.py --verify
sudo python3 adf5355_pi.py --rf-output-hz 2100000000 \
  --reference-hz 125000000 --channel-spacing-hz 50000 \
  --set-rf-output-power 2 --muxout-lock-detect digital --wait-for-lock
~~~

The ADF5355 CE pin is separate from LE. Hold CE high from the board's 3.3 V
rail; do not connect it to SPI0 CE0.

## C

See c/README.md for the no-OS setup, build, and run commands.
