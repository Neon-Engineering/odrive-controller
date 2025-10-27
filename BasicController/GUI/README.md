# ODrive Control GUI

A comprehensive PyQt5-based graphical interface for controlling ODrive motors via CAN bus.

## Features

### 🎮 Motor Control Tab
- **System Control**: Connect, Arm, Disarm, Emergency Stop
- **Position Control**: Set absolute position, move with velocity, relative moves
- **Velocity Control**: Direct velocity setpoint control
- **Oscillation Control**: Configurable amplitude, frequency, and duration

### 📊 Telemetry Tab
- **Real-time Data Display**:
  - Position (turns)
  - Velocity (turns/s)
  - Motor Temperature (°C)
  - FET Temperature (°C)
  - Bus Voltage (V)
  - Bus Current (A)
  - Iq Measured (A)
  - Torque Estimate (Nm)
- **System Status**: Connection, Armed state, Logging status
- **Auto-refreshing**: Updates at 10Hz for smooth monitoring

### 📈 Live Plot Tab
- **Real-time Visualization**: 
  - Position plot with **Setpoint vs Measured** comparison
  - Velocity plot
  - **Tracking Error Display**: Real-time error with color-coded status
- **Low Priority Design**: 5Hz update rate (won't affect control performance)
- **Ring Buffer**: Stores up to 1000 data points
- **Configurable Time Window**: 5-300 seconds display range
- **Enable/Disable**: Toggle plotting on demand
- **Clear Data**: Reset plot buffers
- **Non-blocking Updates**: Uses matplotlib's `draw_idle()` for smooth performance
- **Error Analysis**: 
  - Green: < 0.001 turns (excellent)
  - Orange: 0.001-0.01 turns (good)
  - Red: > 0.01 turns (poor tracking)

### 🎯 Trajectory Tab
- Load trajectory files (CSV, JSON, TXT formats)
- Playback controls (Play, Pause, Stop)
- Trajectory status display

### ⚙️ Configuration Tab
- Load ODrive configuration from JSON files
- Save configuration to NVM with reboot
- Configuration output display

### 📝 Data Logging Tab
- Start/stop high-speed CSV logging (100Hz)
- Optional custom filenames
- Logging status display

## Installation

### Prerequisites
1. Python 3.11 or higher
2. ODrive with CAN interface
3. Working `can_runner.py` setup

### Install GUI Dependencies
```bash
cd GUI
pip install -r gui_requirements.txt
```

Or install manually:
```bash
pip install PyQt5 qasync matplotlib
```

## Usage

### Starting the GUI
```bash
cd GUI
python mainGUI.py
```

### Basic Workflow
1. **Connect**: Click "Connect" to initialize ODrive system
2. **Arm**: Click "Arm" to enable motor control
3. **Control**: Use any of the control tabs to command the motor
4. **Monitor**: Watch real-time telemetry on the Telemetry tab
5. **Visualize** (Optional): Enable Live Plot for real-time graphing
6. **Disarm**: Click "Disarm" when finished

### Emergency Stop
The red "🛑 STOP" button provides immediate motor stop functionality at any time.

## GUI Layout

```
┌─────────────────────────────────────────────────────┐
│  Motor Control | Telemetry | Live Plot | Trajectory │
│  Configuration | Data Logging                       │
├─────────────────────────────────────────────────────┤
│  [Tab Content Area]                                 │
│                                                     │
│  • Motor Control: Position, velocity, oscillation  │
│  • Telemetry: Real-time sensor data (10Hz)        │
│  • Live Plot: Position/velocity graphs (5Hz)      │
│  • Trajectory: File-based motion                   │
│  • Config: ODrive parameter loading                │
│  • Logging: CSV data capture (100Hz)               │
│                                                     │
├─────────────────────────────────────────────────────┤
│  Console Output (scrolling log)                    │
├─────────────────────────────────────────────────────┤
│  Status Bar                                         │
└─────────────────────────────────────────────────────┘
```

## Controls Reference

### Position Control
- **Target Position**: Absolute position in turns
- **Velocity**: Movement speed in turns/s
- **Set Position**: Jump to position immediately
- **Move to Position**: Smooth movement with velocity limit
- **Relative Move**: Move by offset from current position

### Velocity Control
- **Target Velocity**: Desired velocity in turns/s
- **Set Velocity**: Enable velocity control mode

### Oscillation
- **Amplitude**: ± oscillation range in turns
- **Frequency**: Oscillation rate in Hz
- **Duration**: Total oscillation time in seconds

### Live Plot
- **Enable Live Plot**: Toggle real-time plotting on/off
- **Clear Plot Data**: Reset buffers and clear graphs
- **Time Window**: Visible time range (5-300 seconds)
- **Update Rate**: 5Hz (low priority - won't affect control)
- **Capacity**: 1000-point ring buffer with auto-scaling
- **Tracking Error**: Real-time setpoint vs measured comparison
  - Displays position error in turns and percentage
  - Color-coded status indicator (green/orange/red)

## Technical Details

### Architecture
- **Backend**: Uses `can_runner.py` HighPerformanceODriveSystem
- **Async Integration**: qasync for PyQt5 + asyncio compatibility
- **Thread Safety**: All CAN operations executed asynchronously
- **Update Rates**: 
  - Control: Real-time (100Hz backend)
  - Telemetry Display: 10Hz
  - Live Plot: 5Hz (low priority)
  - CSV Logging: 100Hz

### Communication Flow
```
GUI (PyQt5) → qasync → asyncio → can_runner.py → CAN Bus → ODrive
```

### File Structure
```
GUI/
├── mainGUI.py              # Main GUI application
├── gui_requirements.txt    # Python dependencies
└── README.md              # This file
```

## Troubleshooting

### GUI won't start
- Check PyQt5 installation: `pip install PyQt5`
- Check qasync installation: `pip install qasync`
- Verify Python version ≥ 3.11

### Can't connect to ODrive
- Verify ODrive is powered and connected
- Check CAN interface is available (gs_usb)
- Ensure `can_runner.py` works from command line
- Check node ID is correct (default: 0)

### Telemetry not updating
- Ensure system is armed
- Check ODrive is responding to encoder requests
- Verify CAN bus communication is active

### Controls disabled
- System must be connected first (Connect button)
- System must be armed for motor control (Arm button)
- Check for error messages in console output

## Safety Features

1. **Emergency Stop**: Always available red button
2. **State Verification**: Controls disabled when system not ready
3. **Confirmation Dialogs**: For critical operations (save config)
4. **Error Handling**: Graceful failure with user notifications
5. **Honest Telemetry**: Shows "N/A" for unavailable sensor data

## Advanced Usage

### Custom Trajectory Files
Place trajectory files in any directory and browse to them using the Trajectory tab.

Supported formats:
- **CSV**: `time,position` with headers
- **JSON**: `[{"time": t, "position": p}, ...]`
- **TXT**: Space-separated `time position`

### Configuration Management
1. Export config from ODrive web GUI as JSON
2. Load via Configuration tab
3. Modify parameters as needed
4. Save to NVM to persist across reboots

### Data Analysis
- Log files saved with datetime stamps
- CSV format compatible with Excel, MATLAB, Python
- Includes comprehensive telemetry data (position, velocity, currents, temperatures, torque)

## Integration with can_runner.py

The GUI is a thin wrapper around `can_runner.py` functionality:
- All motor control uses existing backend methods
- Configuration system leverages direct parameter writing
- Trajectory playback uses enhanced trajectory player
- Logging uses high-speed queue-based CSV writer

## Performance Notes

- **Telemetry Updates**: 10Hz GUI refresh (100Hz backend polling)
- **Control Latency**: <10ms typical command response
- **Data Logging**: 100Hz non-blocking CSV writes
- **Memory Usage**: ~100MB typical (PyQt5 + asyncio)

## Future Enhancements

Potential additions:
- Real-time plotting of position/velocity
- Multi-motor control (multiple node IDs)
- Custom control loops
- Advanced trajectory editing
- Parameter tuning interface

## License

Same as parent ODrive control system project.
