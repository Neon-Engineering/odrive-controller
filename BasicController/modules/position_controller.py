#!/usr/bin/env python3
"""
Position Controller Module

High-performance position control with trajectory planning and safety limits.
Supports various position control modes and smooth motion planning.
"""

import asyncio
import threading
import time
import math
from typing import Optional, Callable, Dict, Any
from dataclasses import dataclass
from enum import Enum

class PositionControlMode(Enum):
    """Position control modes"""
    DIRECT = "direct"           # Direct position setpoints
    TRAJECTORY = "trajectory"   # Smooth trajectory following
    RAMP = "ramp"              # Ramped position changes
    SPLINE = "spline"          # Spline interpolation

@dataclass
class PositionTarget:
    """Position target specification"""
    position: float
    velocity_ff: float = 0.0
    acceleration_ff: float = 0.0
    timestamp: float = 0.0

@dataclass
class PositionLimits:
    """Position control limits"""
    min_position: float = -10.0
    max_position: float = 10.0
    max_velocity: float = 5.0
    max_acceleration: float = 20.0

class TrajectoryPlanner:
    """
    High-performance trajectory planner for smooth motion
    """
    
    def __init__(self, limits: PositionLimits):
        self.limits = limits
        
    def plan_trajectory(self, start_pos: float, target_pos: float, 
                       duration: float) -> list[PositionTarget]:
        """Plan smooth trajectory between positions"""
        # Simple trapezoidal velocity profile
        targets = []
        
        # Calculate motion parameters
        distance = target_pos - start_pos
        max_vel = min(abs(distance) / (duration * 0.5), self.limits.max_velocity)
        
        # Acceleration phase
        accel_time = max_vel / self.limits.max_acceleration
        accel_dist = 0.5 * self.limits.max_acceleration * accel_time**2
        
        # Adjust for short moves
        if accel_dist * 2 > abs(distance):
            accel_time = math.sqrt(abs(distance) / self.limits.max_acceleration)
            accel_dist = abs(distance) / 2
            max_vel = self.limits.max_acceleration * accel_time
        
        # Generate trajectory points
        dt = 0.01  # 10ms resolution
        t = 0.0
        
        while t <= duration:
            if t <= accel_time:
                # Acceleration phase
                vel = self.limits.max_acceleration * t
                pos = start_pos + 0.5 * self.limits.max_acceleration * t**2
            elif t <= duration - accel_time:
                # Constant velocity phase
                vel = max_vel
                pos = start_pos + accel_dist + max_vel * (t - accel_time)
            else:
                # Deceleration phase
                remaining_time = duration - t
                vel = self.limits.max_acceleration * remaining_time
                pos = target_pos - 0.5 * self.limits.max_acceleration * remaining_time**2
            
            # Apply direction
            if distance < 0:
                vel = -vel
            
            targets.append(PositionTarget(
                position=pos,
                velocity_ff=vel,
                timestamp=t
            ))
            
            t += dt
        
        # Ensure final position is exact
        targets.append(PositionTarget(
            position=target_pos,
            velocity_ff=0.0,
            timestamp=duration
        ))
        
        return targets

class HighPerformancePositionController:
    """
    High-performance position controller with multiple control modes
    """
    
    def __init__(self, can_manager, telemetry_manager, 
                 limits: Optional[PositionLimits] = None):
        self.can_manager = can_manager
        self.telemetry_manager = telemetry_manager
        self.limits = limits or PositionLimits()
        
        # Control state
        self.active = False
        self.control_mode = PositionControlMode.DIRECT
        self.current_target = PositionTarget(position=0.0)
        
        # Trajectory execution
        self.trajectory_planner = TrajectoryPlanner(self.limits)
        self.active_trajectory = []
        self.trajectory_start_time = 0.0
        self.trajectory_index = 0
        
        # Threading
        self.control_thread = None
        self.running = False
        
        # Performance tracking
        self.control_rate = 100.0  # Hz
        self.control_period = 1.0 / self.control_rate
        self.stats = {
            'commands_sent': 0,
            'control_errors': 0,
            'last_position_error': 0.0,
            'last_update_time': 0.0
        }
        
        # Callbacks
        self.position_callbacks = []
        self.error_callbacks = []
    
    async def start(self, control_rate_hz: float = 100.0):
        """Start position controller"""
        print(f"🎯 Starting Position Controller @ {control_rate_hz} Hz...")
        
        self.control_rate = control_rate_hz
        self.control_period = 1.0 / control_rate_hz
        self.running = True
        
        # Start control thread
        self.control_thread = threading.Thread(
            target=self._position_control_thread,
            daemon=True,
            name="Position-Controller"
        )
        self.control_thread.start()
        
        print("✅ Position Controller started")
    
    def _position_control_thread(self):
        """High-frequency position control thread"""
        next_control_time = time.time()
        
        while self.running:
            current_time = time.time()
            
            if self.active:
                self._execute_position_control(current_time)
            
            # Maintain precise timing
            next_control_time += self.control_period
            sleep_time = next_control_time - time.time()
            
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # Running behind, reset timing
                next_control_time = time.time()
    
    def _execute_position_control(self, current_time: float):
        """Execute position control logic"""
        try:
            # Get current position
            position, velocity = self.telemetry_manager.get_position_velocity()
            
            # Determine target based on control mode
            target = self._get_current_target(current_time)
            
            if target and position is not None:
                # Calculate position error
                position_error = target.position - position
                self.stats['last_position_error'] = position_error
                
                # Send position command directly (thread-safe)
                success = self._send_position_command_direct(target.position, target.velocity_ff)
                
                if success:
                    self.stats['commands_sent'] += 1
                    
                    # Call position callbacks
                    for callback in self.position_callbacks:
                        try:
                            callback(target, position, velocity)
                        except Exception as e:
                            print(f"⚠️ Position callback error: {e}")
                else:
                    self.stats['control_errors'] += 1
            
            self.stats['last_update_time'] = current_time
            
        except Exception as e:
            self.stats['control_errors'] += 1
            
            # Call error callbacks
            for callback in self.error_callbacks:
                try:
                    callback(f"Position control error: {e}")
                except:
                    pass
    
    def _get_current_target(self, current_time: float) -> Optional[PositionTarget]:
        """Get current position target based on control mode"""
        if self.control_mode == PositionControlMode.DIRECT:
            return self.current_target
            
        elif self.control_mode == PositionControlMode.TRAJECTORY:
            return self._get_trajectory_target(current_time)
            
        elif self.control_mode == PositionControlMode.RAMP:
            return self._get_ramped_target(current_time)
            
        return self.current_target
    
    def _get_trajectory_target(self, current_time: float) -> Optional[PositionTarget]:
        """Get target from active trajectory"""
        if not self.active_trajectory:
            return None
        
        # Calculate elapsed time since trajectory start
        elapsed_time = current_time - self.trajectory_start_time
        
        # Find appropriate trajectory point
        while (self.trajectory_index < len(self.active_trajectory) - 1 and
               self.active_trajectory[self.trajectory_index + 1].timestamp <= elapsed_time):
            self.trajectory_index += 1
        
        if self.trajectory_index < len(self.active_trajectory):
            return self.active_trajectory[self.trajectory_index]
        else:
            # Trajectory complete
            self.active_trajectory = []
            self.trajectory_index = 0
            return self.active_trajectory[-1] if self.active_trajectory else None
    
    def _get_ramped_target(self, current_time: float) -> Optional[PositionTarget]:
        """Get ramped position target"""
        # Simple ramped implementation
        # TODO: Implement proper ramping logic
        return self.current_target
    
    def _send_position_command_direct(self, position: float, velocity_ff: float = 0.0) -> bool:
        """Send position command directly (thread-safe)"""
        try:
            # Import here to avoid circular import
            import struct
            from .can_manager import CANMessage
            
            message = CANMessage(
                arbitration_id=(self.can_manager.node_id << 5 | 0x0C),
                data=struct.pack('<ff', position, velocity_ff),
                timestamp=time.time()
            )
            
            self.can_manager.command_queue.put_nowait(message)
            return True
            
        except Exception as e:
            return False
    
    # Public API methods
    async def set_position(self, position: float, velocity_ff: float = 0.0):
        """Set direct position target"""
        # Clamp to limits
        clamped_position = max(self.limits.min_position, 
                              min(self.limits.max_position, position))
        
        if clamped_position != position:
            print(f"⚠️ Position clamped: {position:.3f} -> {clamped_position:.3f}")
        
        self.control_mode = PositionControlMode.DIRECT
        self.current_target = PositionTarget(
            position=clamped_position,
            velocity_ff=velocity_ff,
            timestamp=time.time()
        )
        
        print(f"🎯 Position target set: {clamped_position:.3f} turns")
    
    async def move_to_position(self, target_position: float, duration: float = 1.0):
        """Move to position with smooth trajectory"""
        current_position, _ = self.telemetry_manager.get_position_velocity()
        
        if current_position is None:
            print("❌ Cannot plan trajectory - no current position")
            return False
        
        # Plan trajectory
        trajectory = self.trajectory_planner.plan_trajectory(
            current_position, target_position, duration
        )
        
        if trajectory:
            self.control_mode = PositionControlMode.TRAJECTORY
            self.active_trajectory = trajectory
            self.trajectory_start_time = time.time()
            self.trajectory_index = 0
            
            print(f"🎯 Trajectory started: {current_position:.3f} -> {target_position:.3f} "
                  f"over {duration:.2f}s ({len(trajectory)} points)")
            return True
        
        return False
    
    async def relative_move(self, delta_position: float, duration: float = 1.0):
        """Move relative to current position"""
        current_position, _ = self.telemetry_manager.get_position_velocity()
        
        if current_position is None:
            print("❌ Cannot move relative - no current position")
            return False
        
        target_position = current_position + delta_position
        return await self.move_to_position(target_position, duration)
    
    async def oscillate(self, amplitude: float, frequency: float, duration: float):
        """Create oscillating motion"""
        current_position, _ = self.telemetry_manager.get_position_velocity()
        
        if current_position is None:
            print("❌ Cannot oscillate - no current position")
            return False
        
        # Generate oscillation trajectory
        center_position = current_position
        dt = 0.01  # 10ms resolution
        trajectory = []
        
        t = 0.0
        while t <= duration:
            angle = 2 * math.pi * frequency * t
            position = center_position + amplitude * math.sin(angle)
            velocity = amplitude * 2 * math.pi * frequency * math.cos(angle)
            
            trajectory.append(PositionTarget(
                position=position,
                velocity_ff=velocity,
                timestamp=t
            ))
            
            t += dt
        
        # Set trajectory
        self.control_mode = PositionControlMode.TRAJECTORY
        self.active_trajectory = trajectory
        self.trajectory_start_time = time.time()
        self.trajectory_index = 0
        
        print(f"🌊 Oscillation started: ±{amplitude:.3f} turns @ {frequency:.2f} Hz "
              f"for {duration:.2f}s")
        return True
    
    def activate(self):
        """Activate position control"""
        self.active = True
        print("✅ Position control activated")
    
    def deactivate(self):
        """Deactivate position control"""
        self.active = False
        self.active_trajectory = []
        print("🛑 Position control deactivated")
    
    def is_active(self) -> bool:
        """Check if position control is active"""
        return self.active
    
    def get_current_target(self) -> PositionTarget:
        """Get current position target"""
        return self.current_target
    
    def get_position_error(self) -> float:
        """Get latest position error"""
        return self.stats['last_position_error']
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get position controller statistics"""
        return {
            'active': self.active,
            'control_mode': self.control_mode.value,
            'control_rate': self.control_rate,
            'commands_sent': self.stats['commands_sent'],
            'control_errors': self.stats['control_errors'],
            'position_error': self.stats['last_position_error'],
            'trajectory_active': len(self.active_trajectory) > 0,
            'trajectory_progress': self.trajectory_index / len(self.active_trajectory) 
                                  if self.active_trajectory else 0.0
        }
    
    def register_position_callback(self, callback: Callable):
        """Register callback for position updates"""
        self.position_callbacks.append(callback)
    
    def register_error_callback(self, callback: Callable):
        """Register callback for control errors"""
        self.error_callbacks.append(callback)
    
    async def stop(self):
        """Stop position controller"""
        print("🛑 Stopping Position Controller...")
        
        self.running = False
        self.active = False
        
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=1.0)
            print("   Control thread stopped")
        
        print("✅ Position Controller stopped")

# Factory function
def create_position_controller(can_manager, telemetry_manager, 
                             limits: Optional[PositionLimits] = None) -> HighPerformancePositionController:
    """Create high-performance position controller"""
    return HighPerformancePositionController(
        can_manager=can_manager,
        telemetry_manager=telemetry_manager,
        limits=limits
    )