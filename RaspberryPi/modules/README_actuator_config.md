# Actuator Configuration Module

## Overview

The `actuator_config.py` module provides a comprehensive configuration management system for the generic single-axis actuator controller. It supports both linear and rotary actuators with full parameter validation and file persistence.

## Features

- **Actuator Types**: Linear (inches/mm) and Rotary (degrees)
- **Comprehensive Parameters**: Motor config, encoder config, mechanical ratios, limits
- **Validation**: Automatic validation of all parameters with helpful error messages
- **Position Conversion**: Built-in utilities for physical ↔ motor revolution conversion
- **Smart Limits**: Automatically uses most conservative velocity limit
- **File Persistence**: Save/load configurations as JSON
- **Multiple Interfaces**: CLI interactive mode, Python module usage, or direct instantiation

## Usage

### 1. Standalone CLI Mode

```bash
python modules/actuator_config.py
```

Interactive prompts guide you through creating or loading configurations.

### 2. Python Module Usage (for mainControl.py)

```python
from modules.actuator_config import ActuatorConfig

# Load existing configuration
config = ActuatorConfig.load_config('my_actuator.json')

# Validate
is_valid, message = config.validate()

# Access parameters
node_id = config.motor.node_id
max_vel = config.get_effective_velocity_limit()

# Position conversion
motor_pos = config.physical_to_motor_revs(1.5, home_offset=10.0)
physical_pos = config.motor_revs_to_physical(12.3, home_offset=10.0)

# Get full config dict
params = config.get_config_dict()
```

### 3. Programmatic Creation

```python
from modules.actuator_config import ActuatorConfig, MotorConfig

config = ActuatorConfig(
    name="My Actuator",
    actuator_type="linear",
    units="inches",
    overall_ratio=0.2,
    position_min=-2.5,
    position_max=2.5,
    motor=MotorConfig(
        node_id=0,
        current_limit=10.0,
        velocity_limit=10.0
    )
)

config.save_config('my_actuator.json')
```

## Configuration Parameters

### System Identification
- `name`: Descriptive name
- `description`: Optional description

### Actuator Configuration
- `actuator_type`: "linear" or "rotary"
- `units`: "inches", "mm" (linear) or "degrees" (rotary)
- `overall_ratio`: Physical units per motor revolution
  - Linear: pitch (e.g., 0.2 in/rev)
  - Rotary: gear_ratio × 360 (e.g., 36.0 deg/rev for 10:1 gearbox)

### Position Limits
- `position_min`: Minimum position (physical units)
- `position_max`: Maximum position (physical units)
- `velocity_limit_physical`: Max velocity (physical units/sec)

### Homing Configuration
- `homing_direction`: 1 (positive) or -1 (negative)
- `homing_velocity`: Speed during homing (physical units/sec)
- `homing_backoff`: Distance to back off from hardstop
- `homing_threshold`: Velocity threshold for hardstop detection (rev/s)
- `home_position`: Logical home position (physical units)

### Motor Configuration (`motor.*`)
- `node_id`: ODrive CAN node ID (0-63)
- `pole_pairs`: Motor pole pairs
- `torque_constant`: Kt [Nm/A] (0 = auto)
- `velocity_constant`: Kv [RPM/V] (0 = auto)
- `current_limit`: Max motor current [A]
- `velocity_limit`: Max motor velocity [rev/s]
- `encoder_cpr`: Encoder counts per revolution

### Load Encoder Configuration (`load_encoder.*`) - Optional
- `enabled`: Use external load encoder (true/false)
- `encoder_type`: "lvdt", "quadrature", "absolute", or "none"
- `counts_per_unit`: Encoder counts per physical unit
- `lvdt_*`: LVDT-specific calibration parameters (if type = "lvdt")

## Validation

The module automatically validates:
- Actuator type and units compatibility
- Positive ratios and limits
- Valid position ranges
- Motor parameters (node_id, pole_pairs, currents, etc.)
- Homing parameters
- LVDT calibration (if enabled)

## Effective Velocity Limiting

The module calculates the most conservative velocity limit:

```
effective_limit = min(velocity_limit_physical, motor.velocity_limit * overall_ratio)
```

This ensures both physical and motor limits are respected.

## Example Configurations

See:
- `example_linear_actuator.json` - Ball screw linear actuator
- `example_rotary_actuator.json` - Geared rotary stage
- `test_config_usage.py` - Programmatic usage examples

## Integration with mainControl.py

The configuration module is designed to be used by `mainControl.py`:

```python
# In mainControl.py
from modules.actuator_config import ActuatorConfig

config = ActuatorConfig.load_config('my_actuator.json')
params = config.get_config_dict()

# Use params to initialize:
# - CAN communication (params['motor']['node_id'])
# - Position conversion (params['overall_ratio'])
# - Safety limits (params['position_min'], params['position_max'])
# - Homing sequence (params['homing_*'])
```

## File Format

Configurations are stored as JSON with nested structure:

```json
{
    "name": "Example Actuator",
    "actuator_type": "linear",
    "units": "inches",
    "overall_ratio": 0.2,
    ...
    "motor": {
        "node_id": 0,
        "pole_pairs": 7,
        ...
    },
    "load_encoder": {
        "enabled": false,
        ...
    }
}
```

## Next Steps

This module will be integrated with:
1. **Encoder Interface Module** - Process encoder signals and provide position feedback
2. **mainControl.py** - Main controller that orchestrates all modules
3. **GUI** - Graphical configuration editor (future enhancement)
