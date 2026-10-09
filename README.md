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

By default, the programs choose the smallest legal Register 4 R counter. To
diagnose loop behavior at a lower PFD frequency, use `--reference-divider R`.
For the board's 125 MHz reference, the normal enabled divide-by-2 path gives
`fPFD = 62.5 MHz / R`; for example, `--reference-divider 2` selects 31.25 MHz.
The scripts recalculate N, FRAC1, FRAC2, MOD2, Register 9, and Register 10 for
the selected PFD.

The ADF5355 CE pin is separate from LE. Hold CE high from the board's 3.3 V
rail; do not connect it to SPI0 CE0.

## C

Install the build prerequisites and enable SPI0:

~~~sh
sudo apt update
sudo apt install -y build-essential git
sudo raspi-config nonint do_spi 0
sudo reboot
~~~

After the Pi has restarted, clone the ADI no-OS repository alongside this
repository, then build the C example:

~~~sh
git clone --depth 1 https://github.com/analogdevicesinc/no-OS ../no-OS
cd c
make
~~~

See c/README.md for the no-OS wiring details and run commands.
