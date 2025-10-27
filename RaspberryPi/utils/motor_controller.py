#!/usr/bin/env python3
"""
Motor Controller - High-level motor control interface

Provides safe, robust motor control with position/velocity limiting,
error handling, and trajectory generation.
"""

import asyncio
import struct
import time
import math
from dataclasses import dataclass
from typing import Optional, Dict, List, Callable
from enum import IntEnum

class ControlState(IntEnum):
    """ODrive control states"""
    IDLE = 1
    CLOSED_LOOP_CONTROL = 8

class InputMode(IntEnum):
    """ODrive input modes"""
    INACTIVE = 0
    PASSTHROUGH = 1
    VEL_RAMP = 2
    POS_FILTER = 3

@dataclass
class MotorLimits:
    """Motor safety limits"""
    max_position: float = 5.0      # turns
    min_position: float = -5.0     # turns
    max_velocity: float = 2.0      # turns/sec
    max_current: float = 20.0      # amps
    max_temperature: float = 80.0  # celsius

@dataclass
class MotorStatus:
    """Current motor status"""
    position: Optional[float] = None
    velocity: Optional[float] = None
    current: Optional[float] = None
    voltage: Optional[float] = None
    fet_temp: Optional[float] = None
    motor_temp: Optional[float] = None
    axis_state: Optional[int] = None
    error_code: Optional[int] = None
    armed: bool = False

class TrajectoryGenerator:
    """Generate smooth position trajectories with velocity limiting"""
    
    def __init__(self, max_velocity: float = 1.0, max_acceleration: float = 2.0):
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        
    def generate_trajectory(self, start_pos: float, target_pos: float, 
                          current_time: float, dt: float = 0.01) -> List[tuple]:
        """
        Generate a smooth trajectory from start to target position
        
        Returns: List of (time, position, velocity) tuples
        """
        distance = abs(target_pos - start_pos)
        direction = 1.0 if target_pos > start_pos else -1.0
        
        # Calculate trajectory phases
        accel_time = self.max_velocity / self.max_acceleration
        accel_distance = 0.5 * self.max_acceleration * accel_time**2
        
        if distance <= 2 * accel_distance:
            # Triangular profile (no constant velocity phase)
            total_time = 2 * math.sqrt(distance / self.max_acceleration)
            peak_velocity = math.sqrt(distance * self.max_acceleration)
        else:
            # Trapezoidal profile
            const_distance = distance - 2 * accel_distance
            const_time = const_distance / self.max_velocity
            total_time = 2 * accel_time + const_time
            peak_velocity = self.max_velocity
        
        # Generate trajectory points
        trajectory = []
        t = 0.0
        
        while t <= total_time:
            if distance <= 2 * accel_distance:
                # Triangular profile
                half_time = total_time / 2
                if t <= half_time:
                    # Acceleration phase
                    pos = start_pos + direction * 0.5 * self.max_acceleration * t**2
                    vel = direction * self.max_acceleration * t
                else:
                    # Deceleration phase
                    t_decel = t - half_time
                    pos = start_pos + direction * (distance/2 + peak_velocity * t_decel - 0.5 * self.max_acceleration * t_decel**2)
                    vel = direction * (peak_velocity - self.max_acceleration * t_decel)
            else:
                # Trapezoidal profile
                if t <= accel_time:
                    # Acceleration phase
                    pos = start_pos + direction * 0.5 * self.max_acceleration * t**2
                    vel = direction * self.max_acceleration * t
                elif t <= total_time - accel_time:
                    # Constant velocity phase
                    t_const = t - accel_time
                    pos = start_pos + direction * (accel_distance + self.max_velocity * t_const)
                    vel = direction * self.max_velocity
                else:
                    # Deceleration phase
                    t_decel = t - (total_time - accel_time)
                    pos = start_pos + direction * (distance - accel_distance + self.max_velocity * t_decel - 0.5 * self.max_acceleration * t_decel**2)
                    vel = direction * (self.max_velocity - self.max_acceleration * t_decel)
            
            trajectory.append((current_time + t, pos, vel))
            t += dt
        
        # Ensure we end exactly at target
        if trajectory:
            trajectory[-1] = (current_time + total_time, target_pos, 0.0)
        
        return trajectory

class MotorController:
    """
    High-level motor controller with safety features and trajectory generation
    """
    
    def __init__(self, can_manager, node_id: int = 0, limits: Optional[MotorLimits] = None):
        self.can_manager = can_manager
        self.node_id = node_id
        self.limits = limits or MotorLimits()
        
        # Current state
        self.status = MotorStatus()
        self.trajectory_active = False
        self.trajectory_task = None
        
        # Trajectory generator
        self.trajectory_gen = TrajectoryGenerator(
            max_velocity=self.limits.max_velocity * 0.8,  # Safety margin
            max_acceleration=self.limits.max_velocity * 2  # Quick acceleration
        )
        
        # Callbacks
        self.status_callbacks: List[Callable] = []
        self.error_callbacks: List[Callable] = []
        
    def add_status_callback(self, callback: Callable):
        """Add callback for status updates"""
        self.status_callbacks.append(callback)
        
    def add_error_callback(self, callback: Callable):
        """Add callback for error notifications"""
        self.error_callbacks.append(callback)
    
    def _notify_status_callbacks(self):
        """Notify all status callbacks"""
        for callback in self.status_callbacks:
            try:
                callback(self.status)
            except Exception as e:
                print(f"⚠️  Status callback error: {e}")
    
    def _notify_error_callbacks(self, error_msg: str):
        """Notify all error callbacks"""
        for callback in self.error_callbacks:
            try:
                callback(error_msg)
            except Exception as e:
                print(f"⚠️  Error callback error: {e}")
    
    async def arm_motor(self) -> bool:
        """Arm the motor for control"""
        try:
            # Set control mode to CLOSED_LOOP_CONTROL
            response = await self.can_manager.send_message(
                cmd_id=0x07,  # Set_Axis_State
                data=struct.pack('<I', ControlState.CLOSED_LOOP_CONTROL),
                expect_response=False
            )
            
            # Brief delay for state change
            await asyncio.sleep(0.5)  # Increased delay for state transition
            
            # Verify state multiple times if needed
            for attempt in range(3):
                await self.update_status(debug=True)
                print(f"🔍 Arm verification attempt {attempt + 1}: axis_state = {self.status.axis_state}")
                
                if self.status.axis_state == ControlState.CLOSED_LOOP_CONTROL:
                    self.status.armed = True
                    print("✅ Motor armed successfully")
                    self._notify_status_callbacks()
                    return True
                elif attempt < 2:  # Not the last attempt
                    print(f"⏳ Waiting for state transition... (current: {self.status.axis_state})")
                    await asyncio.sleep(0.3)
            
            # Fallback: if heartbeat isn't working but no errors, assume armed
            if self.status.axis_state is None and self.status.error_code == 0:
                print("⚠️  Cannot verify state via heartbeat, but no errors detected")
                print("💡 Assuming motor is armed - try position command")
                self.status.armed = True
                self._notify_status_callbacks()
                return True
            
            print(f"❌ Motor arm failed - Final state: {self.status.axis_state} (expected: {ControlState.CLOSED_LOOP_CONTROL})")
            return False
                
        except Exception as e:
            error_msg = f"Motor arm error: {e}"
            print(f"❌ {error_msg}")
            self._notify_error_callbacks(error_msg)
            return False
    
    async def disarm_motor(self) -> bool:
        """Disarm the motor (set to idle)"""
        try:
            # Cancel any active trajectory
            await self.stop_trajectory()
            
            # Set control mode to IDLE
            response = await self.can_manager.send_message(
                cmd_id=0x07,  # Set_Axis_State
                data=struct.pack('<I', ControlState.IDLE),
                expect_response=False
            )
            
            # Brief delay for state change
            await asyncio.sleep(0.1)
            
            # Update status
            await self.update_status()
            self.status.armed = False
            print("✅ Motor disarmed")
            self._notify_status_callbacks()
            return True
            
        except Exception as e:
            error_msg = f"Motor disarm error: {e}"
            print(f"❌ {error_msg}")
            self._notify_error_callbacks(error_msg)
            return False
    
    async def update_status(self, debug=False) -> bool:
        """Update motor status from CAN messages"""
        try:
            # Get encoder estimates (position, velocity)
            response = await self.can_manager.send_message(
                cmd_id=0x09,  # Get_Encoder_Estimates
                expect_response=True
            )
            
            if response and len(response.data) >= 8:
                self.status.position, self.status.velocity = struct.unpack('<ff', response.data[:8])
                if debug:
                    print(f"   📍 Position: {self.status.position:.3f}, Velocity: {self.status.velocity:.3f}")
            elif debug:
                print("   ⚠️  No encoder response")
            
            # Get temperature readings
            response = await self.can_manager.send_message(
                cmd_id=0x15,  # Get_Temperature
                expect_response=True
            )
            
            if response and len(response.data) >= 8:
                self.status.fet_temp, self.status.motor_temp = struct.unpack('<ff', response.data[:8])
                if debug:
                    print(f"   🌡️  FET: {self.status.fet_temp:.1f}°C, Motor: {self.status.motor_temp:.1f}°C")
            elif debug:
                print("   ⚠️  No temperature response")
            
            # Get axis state (heartbeat)
            response = await self.can_manager.send_message(
                cmd_id=0x01,  # Heartbeat
                expect_response=True
            )
            
            if response and len(response.data) >= 8:
                self.status.axis_state, error_code = struct.unpack('<II', response.data[:8])
                self.status.error_code = error_code
                if debug:
                    print(f"   🚦 Axis State: {self.status.axis_state}, Error: 0x{error_code:04X}")
            elif debug:
                print("   ⚠️  No heartbeat response")
            
            self._notify_status_callbacks()
            return True
            
        except Exception as e:
            error_msg = f"Status update error: {e}"
            print(f"⚠️  {error_msg}")
            self._notify_error_callbacks(error_msg)
            return False
    
    def _check_position_limits(self, position: float) -> bool:
        """Check if position is within limits"""
        return self.limits.min_position <= position <= self.limits.max_position
    
    def _check_velocity_limits(self, velocity: float) -> bool:
        """Check if velocity is within limits"""
        return abs(velocity) <= self.limits.max_velocity
    
    async def set_position(self, position: float, use_trajectory: bool = True, force: bool = False) -> bool:
        """
        Set target position with optional trajectory planning
        
        Args:
            position: Target position in turns
            use_trajectory: Whether to use smooth trajectory (True) or direct command (False)
            force: Force command even if armed flag is False
        """
        if not self.status.armed and not force:
            print("❌ Motor not armed - cannot set position")
            print("💡 If motor is actually armed, use 'status' to check or try 'arm' again")
            return False
        
        if not self._check_position_limits(position):
            error_msg = f"Position {position:.3f} outside limits [{self.limits.min_position:.3f}, {self.limits.max_position:.3f}]"
            print(f"❌ {error_msg}")
            self._notify_error_callbacks(error_msg)
            return False
        
        try:
            if use_trajectory and self.status.position is not None:
                # Use trajectory planning
                await self.move_to_position_smooth(position)
                return True
            else:
                # Direct position command
                await self.can_manager.send_message(
                    cmd_id=0x0C,  # Set_Input_Pos
                    data=struct.pack('<ff', position, 0.0),  # position, velocity_feedforward
                    expect_response=False
                )
                print(f"✅ Position command sent: {position:.3f} turns")
                return True
                
        except Exception as e:
            error_msg = f"Set position error: {e}"
            print(f"❌ {error_msg}")
            self._notify_error_callbacks(error_msg)
            return False
    
    async def move_to_position_smooth(self, target_position: float):
        """Move to position using smooth trajectory"""
        if self.trajectory_active:
            await self.stop_trajectory()
        
        if self.status.position is None:
            await self.update_status()
            if self.status.position is None:
                print("❌ Cannot determine current position for trajectory")
                return False
        
        # Generate trajectory
        trajectory = self.trajectory_gen.generate_trajectory(
            start_pos=self.status.position,
            target_pos=target_position,
            current_time=time.time()
        )
        
        if not trajectory:
            print("❌ Failed to generate trajectory")
            return False
        
        # Execute trajectory
        self.trajectory_active = True
        self.trajectory_task = asyncio.create_task(self._execute_trajectory(trajectory))
        
        print(f"🎯 Starting smooth trajectory to {target_position:.3f} turns ({len(trajectory)} points)")
        
        try:
            await self.trajectory_task
            print("✅ Trajectory completed")
            return True
        except asyncio.CancelledError:
            print("⚠️  Trajectory cancelled")
            return False
        except Exception as e:
            error_msg = f"Trajectory execution error: {e}"
            print(f"❌ {error_msg}")
            self._notify_error_callbacks(error_msg)
            return False
        finally:
            self.trajectory_active = False
            self.trajectory_task = None
    
    async def _execute_trajectory(self, trajectory: List[tuple]):
        """Execute a position trajectory"""
        start_time = time.time()
        
        for i, (target_time, position, velocity) in enumerate(trajectory):
            if not self.trajectory_active:
                break
            
            # Wait for correct timing
            current_time = time.time()
            sleep_time = (start_time + target_time - start_time) - current_time
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)
            
            # Send position command with velocity feedforward
            try:
                await self.can_manager.send_message(
                    cmd_id=0x0C,  # Set_Input_Pos
                    data=struct.pack('<ff', position, velocity),
                    expect_response=False
                )
                
                # Update status periodically
                if i % 10 == 0:  # Every 10th point
                    await self.update_status()
                    
            except Exception as e:
                print(f"⚠️  Trajectory point {i} error: {e}")
                break
    
    async def stop_trajectory(self):
        """Stop any active trajectory"""
        if self.trajectory_active and self.trajectory_task:
            self.trajectory_active = False
            self.trajectory_task.cancel()
            try:
                await self.trajectory_task
            except asyncio.CancelledError:
                pass
            self.trajectory_task = None
    
    async def emergency_stop(self):
        """Emergency stop - immediately disarm motor"""
        print("🚨 EMERGENCY STOP")
        self.trajectory_active = False
        await self.disarm_motor()
    
    async def go_home(self, home_position: float = 0.0) -> bool:
        """Go to home position using smooth trajectory"""
        print(f"🏠 Going to home position: {home_position:.3f} turns")
        return await self.move_to_position_smooth(home_position)
    
    async def jog_relative(self, distance: float) -> bool:
        """Jog relative distance from current position"""
        if self.status.position is None:
            await self.update_status()
            if self.status.position is None:
                print("❌ Cannot determine current position for jog")
                return False
        
        target_position = self.status.position + distance
        print(f"🎮 Jogging {distance:+.3f} turns to {target_position:.3f}")
        return await self.move_to_position_smooth(target_position)
    
    def get_status(self) -> MotorStatus:
        """Get current motor status"""
        return self.status
    
    async def sync_armed_status(self) -> bool:
        """Synchronize armed status with actual axis state"""
        await self.update_status(debug=True)
        
        if self.status.axis_state == ControlState.CLOSED_LOOP_CONTROL:
            if not self.status.armed:
                print("🔧 Detected motor is actually armed, syncing status...")
                self.status.armed = True
                self._notify_status_callbacks()
            return True
        else:
            if self.status.armed:
                print("🔧 Detected motor is not armed, syncing status...")
                self.status.armed = False
                self._notify_status_callbacks()
            return False