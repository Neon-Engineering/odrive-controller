# Copilot Instructions for ODrive Controller Codebase

## Overview
This repository contains Python scripts and modules for controlling ODrive motor controllers, with a focus on CAN bus communication and integration with Raspberry Pi and GUI-based workflows. The codebase is modular, supporting both direct USB and CAN interfaces, and is designed for use in demonstrator and actuator control projects.

## Architecture & Key Components
- **BasicController/**: Core scripts for ODrive control, including CAN communication, configuration, and calibration utilities. Contains reusable modules for CAN, logging, position control, and telemetry.
- **RaspberryPi/**: Raspberry Pi-specific adaptations, including setup scripts, actuator configs, and legacy JTLA actuator control programs. Uses SocketCAN (`can0`) for Linux CAN communication.
- **BasicController/GUI/**: PyQt5-based GUI for motor control, telemetry, live plotting, trajectory playback, and configuration management. GUI interacts with core modules for real-time control and monitoring.
- **flat_endpoints.json & config.json**: ODrive configuration and endpoint mapping files, often exported from ODrive's web GUI or firmware docs.
- **modules/** and **utils/**: Contain reusable logic for CAN management, telemetry, error handling, and motor/position control. Follow a modular import pattern.

## Developer Workflows
- **Setup**: Use Python 3.11+ and install dependencies from `requirements.txt` (or `gui_requirements.txt` for GUI). On Raspberry Pi, use a virtualenv and install OS packages as described in `RaspberryPi/README.md`.
- **CAN Communication**: Scripts default to CAN interface. On Linux, ensure `can0` is configured (see `RaspberryPi/README.md`). On Windows, USB-CAN adapters may require libusb workarounds (see `BasicController/README.md`).
- **Configuration**: Use `can_restore_config.py` to load ODrive config files. Configs are in flat JSON format with dot notation keys.
- **Calibration**: Use `can_calibrate.py` for ODrive calibration routines.
- **Testing**: Use `can_simple.py` for basic CAN communication tests. Use `can_runner.py` for CLI-based control, or `mainGUI.py` for GUI-based workflows.
- **Anticogging**: Experimental anticogging routines are available; see `BasicController/README.md` for status and tuning notes.

## Project Conventions
- **Flat JSON Configs**: All ODrive configs use flat, dot-notated JSON keys (e.g., `axis0.config.motor.pole_pairs`).
- **Modular Imports**: Core logic is split into modules (e.g., `can_manager.py`, `position_controller.py`). Import and reuse these for new scripts.
- **GUI/CLI Parity**: Most control features are available via both CLI scripts and the GUI. Keep logic in modules to avoid duplication.
- **CAN Node IDs & Protocols**: Node IDs and CAN protocol settings are set in config files and scripts. Check for correct values before deployment.
- **libusb Backend**: On Windows, specify the path to the libusb DLL if using USB-CAN adapters (see code snippet in `BasicController/README.md`).

## Integration & Extensibility
- **SocketCAN**: On Raspberry Pi/Linux, all CAN communication uses SocketCAN (`can0`).
- **Trajectory Playback**: GUI supports loading and playing back trajectory files (CSV, JSON, TXT).
- **Data Logging**: High-speed CSV logging is available from the GUI.
- **Legacy Support**: `RaspberryPi/JTLA_Archive/` contains legacy actuator control scripts for reference.

## Example: Loading a Config
```python
from modules.can_manager import CANManager
can_mgr = CANManager()
can_mgr.restore_config('config.json')
```

## References
- See `BasicController/README.md` and `RaspberryPi/README.md` for setup, CAN, and calibration workflows.
- See `BasicController/GUI/README.md` for GUI features and usage.
- ODrive documentation: https://docs.odriverobotics.com/
