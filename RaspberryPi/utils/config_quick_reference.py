"""
Quick Reference: Common Configuration Operations
"""

from modules.actuator_config import ActuatorConfig, MotorConfig, LoadEncoderConfig

# ============================================================
# LOADING AND VALIDATION
# ============================================================

# Load configuration
config = ActuatorConfig.load_config('example_linear_actuator.json')

# Validate
is_valid, msg = config.validate()
print(f"Valid: {is_valid}, Message: {msg}")

# ============================================================
# ACCESSING PARAMETERS
# ============================================================

# Top-level parameters
print(f"Actuator: {config.name}")
print(f"Type: {config.actuator_type}")
print(f"Units: {config.units}")
print(f"Ratio: {config.overall_ratio}")
print(f"Limits: [{config.position_min}, {config.position_max}]")

# Motor parameters
print(f"Node ID: {config.motor.node_id}")
print(f"Current Limit: {config.motor.current_limit} A")
print(f"Velocity Limit: {config.motor.velocity_limit} rev/s")

# Homing parameters
print(f"Homing direction: {config.homing_direction}")
print(f"Homing velocity: {config.homing_velocity} {config.units}/s")

# ============================================================
# POSITION CONVERSION
# ============================================================

# Setup (after homing)
home_offset = 10.5  # Motor position when homed

# Physical → Motor revolutions
target_physical = 1.5  # inches
target_motor = config.physical_to_motor_revs(target_physical, home_offset)
print(f"{target_physical} {config.units} → {target_motor} rev")

# Motor revolutions → Physical
feedback_motor = 12.0  # rev
feedback_physical = config.motor_revs_to_physical(feedback_motor, home_offset)
print(f"{feedback_motor} rev → {feedback_physical} {config.units}")

# ============================================================
# VELOCITY LIMIT CALCULATION
# ============================================================

# Get effective (most conservative) velocity limit
effective_vel = config.get_effective_velocity_limit()
print(f"Effective velocity limit: {effective_vel} {config.units}/s")

# Check which limit is active
motor_limit = config.motor.velocity_limit * config.overall_ratio
if effective_vel == config.velocity_limit_physical:
    print("Limited by physical velocity")
else:
    print("Limited by motor velocity")

# ============================================================
# CREATING NEW CONFIGURATION
# ============================================================

new_config = ActuatorConfig(
    name="Quick Test",
    actuator_type="linear",
    units="inches",
    overall_ratio=0.2,
    position_min=-2.0,
    position_max=2.0,
    motor=MotorConfig(node_id=0)
)

# Save
new_config.save_config('quick_test.json')

# ============================================================
# EXPORTING FOR CONTROLLER
# ============================================================

# Get full config as dictionary
params = config.get_config_dict()

# Use in controller
node_id = params['motor']['node_id']
ratio = params['overall_ratio']
min_pos = params['position_min']
max_pos = params['position_max']
