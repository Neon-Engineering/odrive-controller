"""
Actuator Configuration Module

Manages configuration for generic single-axis actuator control system.
Supports both linear (inches/mm) and rotary (degrees) actuators with configurable
parameters for motor, encoder, and mechanical properties.

Usage:
    # Standalone CLI mode
    python actuator_config.py

    # Python module usage
    from modules.actuator_config import ActuatorConfig
    config = ActuatorConfig()
    config.load_config('my_actuator.json')
    params = config.get_config_dict()
"""

import json
import os
from typing import Dict, Any, Optional, Tuple
from dataclasses import dataclass, asdict, field


@dataclass
class MotorConfig:
    """Motor-specific configuration parameters"""
    node_id: int = 0                        # ODrive CAN node ID (0-63)
    pole_pairs: int = 7                     # Number of motor pole pairs
    torque_constant: float = 0.0            # Kt [Nm/A] - leave 0 for auto
    velocity_constant: float = 0.0          # Kv [RPM/V] - leave 0 for auto
    current_limit: float = 10.0             # Motor current limit [A]
    velocity_limit: float = 10.0            # Motor velocity limit [rev/s]
    encoder_cpr: int = 8192                 # Encoder counts per revolution


@dataclass
class LoadEncoderConfig:
    """Load-side encoder configuration (optional external encoder)"""
    enabled: bool = False                   # Use external load encoder
    encoder_type: str = "none"              # "lvdt", "quadrature", "absolute", "none"
    counts_per_unit: float = 0.0            # Encoder counts per physical unit
    # LVDT-specific parameters (if encoder_type == "lvdt")
    lvdt_v1: float = 0.0                    # Calibration voltage 1 [V]
    lvdt_x1: float = 0.0                    # Calibration position 1 [units]
    lvdt_v2: float = 0.0                    # Calibration voltage 2 [V]
    lvdt_x2: float = 0.0                    # Calibration position 2 [units]
    lvdt_offset: float = 0.0                # Position offset [units]


@dataclass
class ActuatorConfig:
    """Complete actuator system configuration"""
    
    # === System identification ===
    name: str = "Generic Actuator"
    description: str = ""
    
    # === Actuator type and units ===
    actuator_type: str = "linear"           # "linear" or "rotary"
    units: str = "inches"                   # "inches", "mm" (linear), "degrees" (rotary)
    
    # === Mechanical configuration ===
    overall_ratio: float = 0.2              # Physical units per motor revolution
                                            # Linear: pitch [in/rev] or [mm/rev]
                                            # Rotary: gear_ratio * 360 [deg/rev]
    
    # === Position limits (in physical units) ===
    position_min: float = -2.5              # Minimum position [units]
    position_max: float = 2.5               # Maximum position [units]
    
    # === Velocity limits ===
    velocity_limit_physical: float = 1.0    # Max velocity in physical units/sec
    # Effective velocity limit is min(velocity_limit_physical, motor_velocity_limit * overall_ratio)
    
    # === Homing configuration (optional - not needed for absolute encoders) ===
    homing_required: bool = True            # Whether homing is required
    homing_direction: int = 1               # 1 = positive, -1 = negative
    homing_velocity: float = 0.5            # Homing speed [physical units/sec]
    homing_backoff: float = 0.5             # Distance to back off from hardstop [units]
    homing_threshold: float = 0.05          # Velocity threshold for hardstop detection [rev/s]
    home_position: float = 0.0              # Logical home position [units]
    
    # === Sub-configurations ===
    motor: MotorConfig = field(default_factory=MotorConfig)
    load_encoder: LoadEncoderConfig = field(default_factory=LoadEncoderConfig)
    
    def validate(self) -> Tuple[bool, str]:
        """
        Validate configuration parameters.
        
        Returns:
            (is_valid, error_message)
        """
        # Actuator type validation
        if self.actuator_type not in ["linear", "rotary"]:
            return False, f"Invalid actuator_type: {self.actuator_type}. Must be 'linear' or 'rotary'."
        
        # Units validation
        valid_units = {
            "linear": ["inches", "mm"],
            "rotary": ["degrees"]
        }
        if self.units not in valid_units[self.actuator_type]:
            return False, f"Invalid units '{self.units}' for {self.actuator_type} actuator. Must be {valid_units[self.actuator_type]}."
        
        # Overall ratio validation
        if self.overall_ratio <= 0:
            return False, f"overall_ratio must be positive, got {self.overall_ratio}"
        
        # Position limits validation
        if self.position_min >= self.position_max:
            return False, f"position_min ({self.position_min}) must be less than position_max ({self.position_max})"
        
        # Velocity limits
        if self.velocity_limit_physical <= 0:
            return False, f"velocity_limit_physical must be positive, got {self.velocity_limit_physical}"
        
        # Motor configuration validation
        if not (0 <= self.motor.node_id <= 63):
            return False, f"motor.node_id must be 0-63, got {self.motor.node_id}"
        
        if self.motor.pole_pairs <= 0:
            return False, f"motor.pole_pairs must be positive, got {self.motor.pole_pairs}"
        
        if self.motor.current_limit <= 0:
            return False, f"motor.current_limit must be positive, got {self.motor.current_limit}"
        
        if self.motor.velocity_limit <= 0:
            return False, f"motor.velocity_limit must be positive, got {self.motor.velocity_limit}"
        
        if self.motor.encoder_cpr <= 0:
            return False, f"motor.encoder_cpr must be positive, got {self.motor.encoder_cpr}"
        
        # Homing validation (only if homing is required)
        if self.homing_required:
            if self.homing_direction not in [-1, 1]:
                return False, f"homing_direction must be 1 or -1, got {self.homing_direction}"
            
            if self.homing_velocity <= 0:
                return False, f"homing_velocity must be positive, got {self.homing_velocity}"
            
            if self.homing_backoff < 0:
                return False, f"homing_backoff must be non-negative, got {self.homing_backoff}"
        
        # Load encoder validation
        if self.load_encoder.enabled:
            valid_encoder_types = ["lvdt", "quadrature", "absolute", "none"]
            if self.load_encoder.encoder_type not in valid_encoder_types:
                return False, f"load_encoder.encoder_type must be one of {valid_encoder_types}"
            
            if self.load_encoder.encoder_type == "lvdt":
                if self.load_encoder.lvdt_v1 == self.load_encoder.lvdt_v2:
                    return False, "LVDT calibration voltages (lvdt_v1, lvdt_v2) must be different"
        
        return True, "Configuration is valid"
    
    def get_effective_velocity_limit(self) -> float:
        """
        Calculate effective velocity limit (most conservative).
        
        Returns:
            Effective velocity limit in physical units/sec
        """
        motor_limit_physical = self.motor.velocity_limit * self.overall_ratio
        return min(self.velocity_limit_physical, motor_limit_physical)
    
    def physical_to_motor_revs(self, physical_position: float, home_offset: float = 0.0) -> float:
        """
        Convert physical position to motor revolutions.
        
        Args:
            physical_position: Position in physical units
            home_offset: Motor position at home (from homing sequence)
            
        Returns:
            Motor position in revolutions
        """
        return home_offset + (physical_position - self.home_position) / self.overall_ratio
    
    def motor_revs_to_physical(self, motor_position: float, home_offset: float = 0.0) -> float:
        """
        Convert motor revolutions to physical position.
        
        Args:
            motor_position: Motor position in revolutions
            home_offset: Motor position at home (from homing sequence)
            
        Returns:
            Position in physical units
        """
        return self.home_position + (motor_position - home_offset) * self.overall_ratio
    
    def save_config(self, filepath: str) -> None:
        """
        Save configuration to JSON file.
        
        Args:
            filepath: Path to save configuration file
        """
        config_dict = asdict(self)
        with open(filepath, 'w') as f:
            json.dump(config_dict, f, indent=4)
        print(f"✓ Configuration saved to {filepath}")
    
    @classmethod
    def load_config(cls, filepath: str) -> 'ActuatorConfig':
        """
        Load configuration from JSON file.
        
        Args:
            filepath: Path to configuration file
            
        Returns:
            ActuatorConfig instance
        """
        with open(filepath, 'r') as f:
            config_dict = json.load(f)
        
        # Reconstruct nested dataclasses
        motor_config = MotorConfig(**config_dict.pop('motor'))
        load_encoder_config = LoadEncoderConfig(**config_dict.pop('load_encoder'))
        
        config = cls(**config_dict)
        config.motor = motor_config
        config.load_encoder = load_encoder_config
        
        print(f"✓ Configuration loaded from {filepath}")
        return config
    
    def get_config_dict(self) -> Dict[str, Any]:
        """
        Get configuration as dictionary for use by controller.
        
        Returns:
            Dictionary representation of configuration
        """
        return asdict(self)


# === CLI Interface Functions ===

def create_config_interactive() -> ActuatorConfig:
    """
    Interactive CLI for creating new actuator configuration.
    
    Returns:
        ActuatorConfig instance
    """
    print("\n" + "="*60)
    print("  Actuator Configuration Setup")
    print("="*60 + "\n")
    
    # System identification
    name = input("Actuator name [Generic Actuator]: ").strip() or "Generic Actuator"
    description = input("Description (optional): ").strip()
    
    # Actuator type
    print("\nActuator Type:")
    print("  1. Linear (ball screw, lead screw, linear actuator)")
    print("  2. Rotary (direct drive, geared rotary)")
    actuator_choice = input("Select type [1]: ").strip() or "1"
    actuator_type = "linear" if actuator_choice == "1" else "rotary"
    
    # Units
    if actuator_type == "linear":
        print("\nUnits:")
        print("  1. Inches")
        print("  2. Millimeters")
        units_choice = input("Select units [1]: ").strip() or "1"
        units = "inches" if units_choice == "1" else "mm"
    else:
        units = "degrees"
        print(f"\nUnits: {units} (rotary default)")
    
    # Overall ratio
    print(f"\nOverall Ratio ({units}/motor_revolution):")
    if actuator_type == "linear":
        print("  Example: Ball screw with 0.2\" pitch → 0.2")
    else:
        print("  Example: 10:1 gearbox → 36.0 (360/10)")
    overall_ratio = float(input("Enter overall ratio: "))
    
    # Position limits
    print(f"\nPosition Limits (in {units}):")
    position_min = float(input("Minimum position: "))
    position_max = float(input("Maximum position: "))
    
    # Velocity limits
    print(f"\nVelocity Limit (in {units}/sec):")
    velocity_limit_physical = float(input(f"Maximum velocity [{1.0}]: ") or 1.0)
    
    # Motor configuration
    print("\n" + "-"*60)
    print("  Motor Configuration")
    print("-"*60)
    node_id = int(input("ODrive CAN node ID [0]: ") or 0)
    pole_pairs = int(input("Motor pole pairs [7]: ") or 7)
    current_limit = float(input("Current limit [A] [10.0]: ") or 10.0)
    motor_velocity_limit = float(input("Motor velocity limit [rev/s] [10.0]: ") or 10.0)
    encoder_cpr = int(input("Motor encoder counts/rev [8192]: ") or 8192)
    
    motor = MotorConfig(
        node_id=node_id,
        pole_pairs=pole_pairs,
        current_limit=current_limit,
        velocity_limit=motor_velocity_limit,
        encoder_cpr=encoder_cpr
    )
    
    # Load encoder (optional)
    print("\n" + "-"*60)
    print("  Load Encoder Configuration (Optional)")
    print("-"*60)
    use_load_encoder = input("Use external load encoder? [y/N]: ").strip().lower() == 'y'
    
    if use_load_encoder:
        print("\nEncoder Type:")
        print("  1. LVDT (Linear Variable Differential Transformer) - Absolute")
        print("  2. Quadrature encoder - Incremental")
        print("  3. Absolute encoder - Absolute")
        encoder_choice = input("Select type [1]: ").strip() or "1"
        encoder_types = {"1": "lvdt", "2": "quadrature", "3": "absolute"}
        encoder_type = encoder_types.get(encoder_choice, "lvdt")
        
        load_encoder = LoadEncoderConfig(enabled=True, encoder_type=encoder_type)
        
        if encoder_type == "lvdt":
            print("\nLVDT Calibration (two-point linear calibration):")
            load_encoder.lvdt_v1 = float(input("  Voltage 1 [V]: "))
            load_encoder.lvdt_x1 = float(input(f"  Position 1 [{units}]: "))
            load_encoder.lvdt_v2 = float(input("  Voltage 2 [V]: "))
            load_encoder.lvdt_x2 = float(input(f"  Position 2 [{units}]: "))
            load_encoder.lvdt_offset = float(input(f"  Position offset [{units}] [0.0]: ") or 0.0)
    else:
        load_encoder = LoadEncoderConfig()
    
    # Homing configuration
    print("\n" + "-"*60)
    print("  Homing Configuration")
    print("-"*60)
    
    # Check if absolute encoder is being used
    is_absolute_encoder = (use_load_encoder and encoder_type in ["lvdt", "absolute"])
    
    if is_absolute_encoder:
        print("Note: Absolute position encoder detected (LVDT or absolute encoder).")
        homing_required_input = input("Require homing sequence? [y/N]: ").strip().lower()
        homing_required = homing_required_input == 'y'
        
        if not homing_required:
            print("✓ Homing disabled - absolute encoder will provide position reference.")
            homing_direction = 1
            homing_velocity = 0.5
            homing_backoff = 0.5
            home_position = 0.0
        else:
            print("Homing enabled - will establish reference despite absolute encoder.")
            homing_direction = int(input("Homing direction (1=positive, -1=negative) [1]: ") or 1)
            homing_velocity = float(input(f"Homing velocity [{units}/s] [0.5]: ") or 0.5)
            homing_backoff = float(input(f"Backoff distance from hardstop [{units}] [0.5]: ") or 0.5)
            home_position = float(input(f"Logical home position [{units}] [0.0]: ") or 0.0)
    else:
        # Incremental encoder - homing is required
        print("Note: Incremental encoder - homing sequence required to establish reference.")
        homing_required = True
        homing_direction = int(input("Homing direction (1=positive, -1=negative) [1]: ") or 1)
        homing_velocity = float(input(f"Homing velocity [{units}/s] [0.5]: ") or 0.5)
        homing_backoff = float(input(f"Backoff distance from hardstop [{units}] [0.5]: ") or 0.5)
        home_position = float(input(f"Logical home position [{units}] [0.0]: ") or 0.0)
    
    # Create config
    config = ActuatorConfig(
        name=name,
        description=description,
        actuator_type=actuator_type,
        units=units,
        overall_ratio=overall_ratio,
        position_min=position_min,
        position_max=position_max,
        velocity_limit_physical=velocity_limit_physical,
        homing_required=homing_required,
        homing_direction=homing_direction,
        homing_velocity=homing_velocity,
        homing_backoff=homing_backoff,
        home_position=home_position,
        motor=motor,
        load_encoder=load_encoder
    )
    
    # Validate
    is_valid, message = config.validate()
    if is_valid:
        print(f"\n✓ {message}")
    else:
        print(f"\n✗ Validation Error: {message}")
        print("Please fix the configuration and try again.")
        return None
    
    # Show effective limits
    print(f"\nEffective velocity limit: {config.get_effective_velocity_limit():.3f} {units}/s")
    motor_limit = config.motor.velocity_limit * config.overall_ratio
    if config.velocity_limit_physical < motor_limit:
        print(f"  (Limited by physical velocity limit)")
    else:
        print(f"  (Limited by motor velocity limit)")
    
    return config


def main():
    """Main CLI entry point"""
    print("\n" + "="*60)
    print("  Actuator Configuration Manager")
    print("="*60 + "\n")
    
    print("Options:")
    print("  1. Create new configuration")
    print("  2. Load existing configuration")
    print("  3. Exit")
    
    choice = input("\nSelect option [1]: ").strip() or "1"
    
    if choice == "1":
        config = create_config_interactive()
        if config is None:
            return
        
        # Save option
        save = input("\nSave configuration? [Y/n]: ").strip().lower()
        if save != 'n':
            default_filename = config.name.lower().replace(" ", "_") + "_config.json"
            filepath = input(f"Filename [{default_filename}]: ").strip() or default_filename
            config.save_config(filepath)
    
    elif choice == "2":
        filepath = input("Enter configuration file path: ").strip()
        if not os.path.exists(filepath):
            print(f"✗ File not found: {filepath}")
            return
        
        try:
            config = ActuatorConfig.load_config(filepath)
            is_valid, message = config.validate()
            print(f"\nValidation: {message}")
            
            # Display summary
            print("\n" + "="*60)
            print(f"  {config.name}")
            print("="*60)
            print(f"Type: {config.actuator_type} ({config.units})")
            print(f"Ratio: {config.overall_ratio} {config.units}/rev")
            print(f"Limits: {config.position_min} to {config.position_max} {config.units}")
            print(f"Motor: Node {config.motor.node_id}, {config.motor.pole_pairs} pole pairs")
            print(f"Velocity: {config.get_effective_velocity_limit():.3f} {config.units}/s (effective)")
            print(f"Homing: {'Required' if config.homing_required else 'Not required (absolute encoder)'}")
            if config.load_encoder.enabled:
                print(f"Load Encoder: {config.load_encoder.encoder_type}")
            print("="*60 + "\n")
            
        except Exception as e:
            print(f"✗ Error loading configuration: {e}")
    
    elif choice == "3":
        print("Exiting...")
    else:
        print("Invalid option")


if __name__ == "__main__":
    main()






