
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

