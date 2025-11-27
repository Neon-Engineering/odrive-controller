# ODrive Controller Codebase - GitHub Copilot Instructions

## Project Overview
High-performance ODrive motor controller system with CAN bus communication support for Raspberry Pi and Windows platforms.

**Current Version:** 1.2.0  
**Main Branch:** `main`  
**Active Development Branches:** `odrive-rpi`, `node-id`

---

## Architecture

### Core Components

1. **RaspberryPi/can_runner.py** - Main CLI control system
   - Async architecture using Python asyncio
   - Interactive CLI with command menu
   - Auto-discovery and node ID assignment capabilities
   - High-speed logging and telemetry

2. **RaspberryPi/utils/node_discovery.py** - ODrive node discovery (v1.2.0)
   - **Official ODrive enumeration protocol** (CMD 0x06 with RTR frames)
   - Discovers both addressed and unaddressed ODrives
   - Thread executor pattern for non-blocking CAN bus operations
   - Returns tuple: `(addressed_nodes, unaddressed_nodes)`

3. **RaspberryPi/modules/** - Core system modules
   - `simple_can_manager.py` - CAN bus communication wrapper
   - `position_controller.py` - Motion control
   - `telemetry_manager.py` - Real-time data collection
   - `trajectory_player.py` - Trajectory execution

4. **RaspberryPi/GUI/** - PyQt5 graphical interfaces
   - `mainGUI.py` - Production GUI
   - `mainGUI_test.py` - Development GUI with error handler
   - Node selection dialogs for multi-device support

---

## Critical Implementation Details

### ODrive CAN Protocol (Simple Protocol)
- **Arbitration ID Format:** `(node_id << 5) | cmd_id`
- **Node ID Range:** 0-62 (0x00-0x3E)
- **Broadcast Address:** 63 (0x3F)
- **Key Commands:**
  - `0x01` - Heartbeat (automatic, ~10Hz)
  - `0x06` - ADDRESS_CMD (discovery/addressing)
  - `0x09` - Encoder data request (RTR frame)
  - `0x0C` - Set position
  - `0x0D` - Set velocity
  - `0x07` - Set requested state

### Node Discovery Implementation (v1.2.0)

**IMPORTANT:** Discovery uses **thread executor** to prevent event loop blocking:

```python
# Correct pattern - DO NOT use direct bus.recv()
loop = asyncio.get_event_loop()
msg = await loop.run_in_executor(None, lambda: self.bus.recv(timeout=0.01))
```

**Why:** `bus.recv()` is synchronous and blocks the asyncio event loop, causing GUI freezes and hangs.

**Discovery Flow:**
1. Send RTR frame to broadcast address: `arbitration_id = (0x3F << 5) | 0x06`
2. ODrives respond with `[node_id, serial_number (6 bytes), ...]`
3. Unaddressed ODrives respond with `node_id = 0x3F`
4. Method returns: `(addressed_list, unaddressed_list)`

**Timeout Safety:**
- Requested timeout: configurable (default 3.0s)
- Absolute timeout: 2x requested timeout
- Discovery interval: 0.6s between requests

### Node ID Assignment Over CAN

ODrives can be assigned node IDs via CAN without USB connection:

```python
# Assignment message format
data = [new_node_id] + serial_number.to_bytes(6, 'little') + [checksum]
checksum = new_node_id ^ (XOR of all serial_number bytes)
```

**Note:** Assignment is **temporary (RAM only)**. For persistence:
```bash
odrivetool
odrv0.axis0.config.can.node_id = <id>
odrv0.save_configuration()
odrv0.reboot()
```

---

## Async/Await Patterns

### DO ✅
```python
# Use thread executor for blocking I/O
loop = asyncio.get_event_loop()
result = await loop.run_in_executor(None, blocking_function)

# Yield to event loop in loops
await asyncio.sleep(0.01)

# Proper exception handling with async sleep
except Exception:
    await asyncio.sleep(0.01)
    continue
```

### DON'T ❌
```python
# Direct blocking calls in async functions
msg = self.bus.recv(timeout=0.1)  # Blocks event loop!

# Tight loops without yielding
while True:
    # ... intensive work without await
    pass  # GUI will freeze
```

---

## GUI Implementation (PyQt5)

### Critical Patterns

**Async Integration:**
```python
# Use qasync for PyQt5 + asyncio
import qasync

async def async_method(self):
    # Your async code
    pass

# Connect to button
self.button.clicked.connect(lambda: asyncio.ensure_future(self.async_method()))
```

**Node Selection Dialog:**
- Always offer assignment when unaddressed devices found
- Show serial numbers, node IDs, axis states
- Provide "Assign Node ID", "Continue", and "Cancel" options

**Error Handling:**
```python
try:
    nodes, unaddressed = await discovery.enumerate_odrives(timeout=3.0)
except Exception as e:
    self.log_to_console(f"❌ Enumeration failed: {e}")
    QMessageBox.critical(self, "Error", str(e))
    return
```

---

## CLI Commands Reference

```
arm              - Software arm (enable control)
disarm           - Software disarm
pos <value>      - Set position
vel <value>      - Set velocity
move <pos> <vel> - Move to position at velocity
stop             - Emergency stop
scan             - Scan for ODrive nodes
assign           - Assign node ID to unaddressed ODrive (NEW in v1.2.0)
traj load/play   - Trajectory control
log start/stop   - High-speed logging
calibrate        - Run calibration sequence
status           - Show system status
```

---

## Common Issues & Solutions

### Issue: GUI Hangs During Discovery
**Cause:** `bus.recv()` blocking event loop  
**Solution:** Use thread executor pattern (see Node Discovery section)

### Issue: "No ODrives found" with unaddressed devices
**Cause:** Unaddressed devices filtered from main list  
**Solution:** Check `unaddressed` list in returned tuple, prompt for assignment

### Issue: Can't control ODrive after assignment
**Cause:** Assignment is temporary (RAM only)  
**Solution:** Save configuration via USB with odrivetool

### Issue: Import errors in GUI subdirectory
**Solution:** Add parent path to sys.path:
```python
parent_path = str(Path(__file__).parent.parent)
if parent_path not in sys.path:
    sys.path.insert(0, parent_path)
```

---

## Testing Checklist

When making changes, verify:

- [ ] CLI auto-discovery works on startup
- [ ] GUI enumeration doesn't freeze interface
- [ ] Unaddressed ODrives trigger assignment prompt
- [ ] Multi-node selection dialog displays correctly
- [ ] Assignment command works in CLI
- [ ] Assignment button works in GUI
- [ ] Timeout mechanisms prevent infinite hangs
- [ ] Error messages are clear and actionable

---

## Version History

**v1.2.0** (November 2025)
- Added official ODrive enumeration protocol (CMD 0x06)
- Implemented auto node ID discovery for all ODrive states
- Added CAN-based node ID assignment (no USB required)
- Fixed GUI/CLI hanging issues with thread executor pattern
- Added interactive assignment prompts for unaddressed devices
- Created comprehensive multi-device selection dialogs

**v1.1.0** (Previous)
- High-performance CAN control system
- Trajectory playback
- High-speed logging

---

## Development Guidelines

1. **Always use async/await properly** - No blocking calls in async functions
2. **Test on Raspberry Pi** - CAN bus behavior differs from simulation
3. **Handle unaddressed devices** - Don't just fall back to node_id=0
4. **Provide user guidance** - Clear error messages and next steps
5. **Document protocol details** - ODrive's protocol is specific, document assumptions
6. **Use thread executors** - For any blocking I/O in async contexts
7. **Maintain backwards compatibility** - CLI flags and config file formats

---

## File Structure

```
RaspberryPi/
├── can_runner.py           # Main CLI application
├── modules/
│   ├── simple_can_manager.py
│   ├── position_controller.py
│   ├── telemetry_manager.py
│   └── trajectory_player.py
├── utils/
│   ├── node_discovery.py   # v1.2.0 enumeration
│   ├── can_manager.py
│   ├── motor_controller.py
│   └── error_handler.py
└── GUI/
    ├── mainGUI.py          # Production GUI
    └── mainGUI_test.py     # Development GUI
```

---

## Dependencies

- Python 3.7+
- python-can (socketcan for Linux, gs_usb for Windows)
- asyncio
- PyQt5 (for GUI)
- qasync (PyQt5 + asyncio integration)
- struct, time, threading (standard library)

---

## Contact & Resources

- **Official ODrive Docs:** https://docs.odriverobotics.com/
- **CAN Simple Protocol:** https://docs.odriverobotics.com/v/latest/can-protocol.html
- **ODrive Enumeration Example:** ODriveResources/examples/can_enumerate.py

---

*Last Updated: November 26, 2025 - Version 1.2.0 Node Discovery Release*
