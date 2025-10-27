
# Neon Actuation Controller Repository for ODrive-RaspberryPi Interface

The repository includes an archive of the JTLA jam-tolerant triple redundant ball screw actuator controller programs developed by Sean Murphy at Answer Engineering for Neon Aero.

## Notes on JTLA Program

The JTLA program was designed to be run on a Raspberry Pi with an analog to digital converter (ADC) to read the LVDT signals from the LVDT signal conditioner board. The ADC is manufactured by Waveshare as a "High Precision Analog to Digital Hat" which uses an ADS1263 chip for A to D conversion. The drivers for the AD hat is located in the repository as ADS1263.7z, this should be sent to the Raspberry Pi and set up on the local environment, and the sys.path.append() call should be amended to match the particular environment.


The generic single-axis controller is based on the JTLA control program.

## Goals

 - Use a Raspberry Pi (RPi) to send commands to an ODrive Pro for the November 2025 demonstrator
 - Have a user-friendly GUI to interface with the RPi
 - Have a setup/config GUI that allows the user to configure the motor and actuator parameters (linear/rotary, overall ratio, encoders)
 - Modular software that is capable of being reused in different actuator designs for different purposes


## Repository Architecture

 - Primary workspace contains core main driver functions
 - modules directory stores modular functions and programs which can be added/removed from main feature
 - utils directory stores useful utilities and libraries that make tasks more understandable and readable

SocketCAN / Raspberry Pi Quick Setup
-----------------------------------

This repository contains a Raspberry Pi targeted copy of the BasicController code which defaults to using SocketCAN on Linux (talks to `can0`).

1) Install OS packages on the Pi:

```bash
sudo apt update
sudo apt install -y python3-pip python3-venv can-utils git
```

2) Create and activate a Python virtualenv (recommended) and install Python dependencies:

```bash
cd RaspberryPi
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

3) Configure and bring up `can0` (example for 500k bitrate):

```bash
# If using MCP2515 HAT, ensure dtoverlay is present in /boot/config.txt and SPI is enabled.
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
ip -details link show can0
```

4) Quick functional test with can-utils:

```bash
sudo candump can0
cansend can0 123#1122334455667788   # note: do NOT use the 0x prefix for the CAN id
```

5) Running the controller:

```bash
# run from RaspberryPi/ with virtualenv active
python3 can_runner.py
# or to force GS-USB backend (if you have a USB CAN adapter):
export ODRIVE_CAN_BACKEND=gs_usb
python3 can_runner.py
```

6) Running from Thonny and permissions

- Thonny can run the code, but raw CAN sockets typically require elevated privileges. Options:
	- Run Thonny with sudo (not recommended generally): `sudo thonny`
	- Grant the python3 binary network capabilities so it can open raw sockets without sudo:

```bash
sudo setcap 'cap_net_raw,cap_net_admin+eip' $(which python3)
```

	- Or run the controller from a terminal with sudo: `sudo python3 can_runner.py` (recommended for quick tests).

If you want, I can add a small smoke-test script under `RaspberryPi/tests/` that sends a frame and displays received frames; tell me and I will add it.

