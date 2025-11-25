#!/usr/bin/env python3
"""
ODrive Calibration Module

Handles motor and encoder calibration sequences for ODrive controllers.
This module helps resolve MISSING_ESTIMATE errors that can occur after
saving configurations to NVM.

Calibration States (from ODrive documentation):
- AXIS_STATE_IDLE = 1
- AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3
- AXIS_STATE_MOTOR_CALIBRATION = 4
- AXIS_STATE_ENCODER_INDEX_SEARCH = 6
- AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7
- AXIS_STATE_CLOSED_LOOP_CONTROL = 8
"""

import asyncio
import struct
import can
from typing import Optional, Tuple
from utils.can_simple_utils import CanSimpleNode

# ODrive Axis State Constants
AXIS_STATE_UNDEFINED = 0
AXIS_STATE_IDLE = 1
AXIS_STATE_STARTUP_SEQUENCE = 2
AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3
AXIS_STATE_MOTOR_CALIBRATION = 4
AXIS_STATE_ENCODER_INDEX_SEARCH = 6
AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

# Heartbeat message CMD ID
HEARTBEAT_CMD = 0x01


class ODriveCalibration:
    """
    ODrive calibration manager for CAN-based control
    
    Provides methods to calibrate motor and encoder, check calibration status,
    and handle the full calibration sequence via CAN messages.
    """
    
    def __init__(self, can_node: CanSimpleNode, node_id: int = 0):
        """
        Initialize calibration manager
        
        Args:
            can_node: CanSimpleNode instance for CAN communication
            node_id: ODrive node ID (default 0)
        """
        self.node = can_node
        self.node_id = node_id
        self.bus = can_node.bus
    
    async def get_axis_state(self, timeout: float = 1.0) -> Optional[int]:
        """
        Get current axis state from heartbeat message
        
        Args:
            timeout: Timeout in seconds
            
        Returns:
            Current axis state or None if timeout
        """
        try:
            msg = await self.node.await_msg(HEARTBEAT_CMD, timeout=timeout)
            if msg and len(msg.data) >= 8:
                # Heartbeat format: axis_error (4 bytes), axis_state (1 byte), ...
                axis_error, axis_state = struct.unpack('<IB', msg.data[:5])
                return axis_state
            return None
        except asyncio.TimeoutError:
            return None
    
    async def get_axis_error(self, timeout: float = 1.0) -> Optional[int]:
        """
        Get current axis error from heartbeat message
        
        Args:
            timeout: Timeout in seconds
            
        Returns:
            Current axis error code or None if timeout
        """
        try:
            msg = await self.node.await_msg(HEARTBEAT_CMD, timeout=timeout)
            if msg and len(msg.data) >= 4:
                axis_error = struct.unpack('<I', msg.data[:4])[0]
                return axis_error
            return None
        except asyncio.TimeoutError:
            return None
    
    async def wait_for_state(self, target_state: int, timeout: float = 30.0, 
                            poll_interval: float = 0.5) -> Tuple[bool, Optional[int]]:
        """
        Wait for axis to reach target state
        
        Args:
            target_state: Desired axis state
            timeout: Maximum time to wait in seconds
            poll_interval: How often to check state in seconds
            
        Returns:
            Tuple of (success: bool, final_state: Optional[int])
        """
        start_time = asyncio.get_event_loop().time()
        last_state = None
        
        while (asyncio.get_event_loop().time() - start_time) < timeout:
            current_state = await self.get_axis_state(timeout=poll_interval)
            
            if current_state is not None:
                last_state = current_state
                
                if current_state == target_state:
                    return True, current_state
                
                # Check if returned to IDLE (calibration complete or error)
                if target_state != AXIS_STATE_IDLE and current_state == AXIS_STATE_IDLE:
                    # Check for errors
                    error = await self.get_axis_error(timeout=1.0)
                    if error and error != 0:
                        print(f"⚠️ Calibration ended with error code: 0x{error:08X}")
                        return False, current_state
                    # If no error, calibration completed successfully
                    return True, current_state
            
            await asyncio.sleep(poll_interval)
        
        print(f"⏱️ Timeout waiting for state {target_state} (last state: {last_state})")
        return False, last_state
    
    async def run_full_calibration(self, timeout: float = 60.0) -> bool:
        """
        Run full calibration sequence (motor + encoder)
        
        This is the recommended calibration for most setups.
        
        Args:
            timeout: Maximum time to wait for calibration in seconds
            
        Returns:
            True if calibration successful, False otherwise
        """
        print("🔧 Starting full calibration sequence...")
        print("⚠️ WARNING: Motor will spin during calibration!")
        print("   Ensure motor shaft is free to move")
        
        try:
            # Clear any existing errors
            print("🧹 Clearing errors...")
            self.node.clear_errors_msg()
            await asyncio.sleep(0.5)
            
            # Request full calibration sequence
            print("🎯 Requesting FULL_CALIBRATION_SEQUENCE (state 3)...")
            self.node.set_state_msg(AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
            await asyncio.sleep(1.0)
            
            # Monitor calibration progress
            print("⏳ Calibrating (this may take 15-30 seconds)...")
            success, final_state = await self.wait_for_state(
                AXIS_STATE_IDLE, 
                timeout=timeout,
                poll_interval=1.0
            )
            
            if success:
                # Check for errors after calibration
                error = await self.get_axis_error(timeout=2.0)
                if error and error != 0:
                    print(f"❌ Calibration failed with error: 0x{error:08X}")
                    print("💡 Common errors:")
                    print("   - 0x0001: Initializing (wait)")
                    print("   - 0x0040: Motor not connected")
                    print("   - 0x0800: Encoder error")
                    return False
                
                print("✅ Full calibration completed successfully!")
                print("💡 You can now save configuration to make calibration persistent")
                print("   or use pre-calibrated mode (set axis0.config.startup_* flags)")
                return True
            else:
                error = await self.get_axis_error(timeout=2.0)
                print(f"❌ Calibration timed out or failed (final state: {final_state})")
                if error:
                    print(f"   Error code: 0x{error:08X}")
                return False
                
        except Exception as e:
            print(f"❌ Calibration exception: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    async def run_motor_calibration(self, timeout: float = 30.0) -> bool:
        """
        Run motor calibration only
        
        Args:
            timeout: Maximum time to wait for calibration in seconds
            
        Returns:
            True if calibration successful, False otherwise
        """
        print("🔧 Starting motor calibration...")
        print("⚠️ WARNING: Motor will spin during calibration!")
        
        try:
            # Clear errors
            self.node.clear_errors_msg()
            await asyncio.sleep(0.5)
            
            # Request motor calibration
            print("🎯 Requesting MOTOR_CALIBRATION (state 4)...")
            self.node.set_state_msg(AXIS_STATE_MOTOR_CALIBRATION)
            await asyncio.sleep(1.0)
            
            # Wait for completion
            print("⏳ Calibrating motor...")
            success, final_state = await self.wait_for_state(
                AXIS_STATE_IDLE,
                timeout=timeout,
                poll_interval=0.5
            )
            
            if success:
                error = await self.get_axis_error(timeout=2.0)
                if error and error != 0:
                    print(f"❌ Motor calibration failed with error: 0x{error:08X}")
                    return False
                print("✅ Motor calibration completed!")
                return True
            else:
                print("❌ Motor calibration timed out")
                return False
                
        except Exception as e:
            print(f"❌ Motor calibration exception: {e}")
            return False
    
    async def run_encoder_offset_calibration(self, timeout: float = 30.0) -> bool:
        """
        Run encoder offset calibration
        
        Note: Requires motor to be already calibrated
        
        Args:
            timeout: Maximum time to wait for calibration in seconds
            
        Returns:
            True if calibration successful, False otherwise
        """
        print("🔧 Starting encoder offset calibration...")
        print("⚠️ WARNING: Motor will rotate during calibration!")
        
        try:
            # Clear errors
            self.node.clear_errors_msg()
            await asyncio.sleep(0.5)
            
            # Request encoder calibration
            print("🎯 Requesting ENCODER_OFFSET_CALIBRATION (state 7)...")
            self.node.set_state_msg(AXIS_STATE_ENCODER_OFFSET_CALIBRATION)
            await asyncio.sleep(1.0)
            
            # Wait for completion
            print("⏳ Calibrating encoder...")
            success, final_state = await self.wait_for_state(
                AXIS_STATE_IDLE,
                timeout=timeout,
                poll_interval=0.5
            )
            
            if success:
                error = await self.get_axis_error(timeout=2.0)
                if error and error != 0:
                    print(f"❌ Encoder calibration failed with error: 0x{error:08X}")
                    return False
                print("✅ Encoder offset calibration completed!")
                return True
            else:
                print("❌ Encoder calibration timed out")
                return False
                
        except Exception as e:
            print(f"❌ Encoder calibration exception: {e}")
            return False
    
    async def check_calibration_status(self) -> dict:
        """
        Check if motor and encoder are calibrated (via parameters)
        
        Returns:
            Dictionary with calibration status information
        """
        status = {
            'axis_state': None,
            'axis_error': None,
            'motor_calibrated': None,
            'encoder_ready': None
        }
        
        try:
            # Get current state and error
            status['axis_state'] = await self.get_axis_state(timeout=2.0)
            status['axis_error'] = await self.get_axis_error(timeout=2.0)
            
            # Try to read calibration flags (may not be available on all firmware)
            try:
                motor_calibrated = await self.node.read_parameter('axis0.motor.is_calibrated')
                status['motor_calibrated'] = motor_calibrated
            except:
                status['motor_calibrated'] = None
            
            try:
                encoder_ready = await self.node.read_parameter('axis0.encoder.is_ready')
                status['encoder_ready'] = encoder_ready
            except:
                status['encoder_ready'] = None
            
            return status
            
        except Exception as e:
            print(f"⚠️ Could not check calibration status: {e}")
            return status
    
    def print_calibration_status(self, status: dict):
        """Print calibration status in human-readable format"""
        print("\n📊 Calibration Status:")
        print(f"   Axis State: {status.get('axis_state', 'Unknown')}")
        
        error = status.get('axis_error', 0)
        if error and error != 0:
            print(f"   ⚠️ Axis Error: 0x{error:08X}")
        else:
            print(f"   ✅ No Errors")
        
        motor_cal = status.get('motor_calibrated')
        if motor_cal is not None:
            print(f"   Motor Calibrated: {'✅ Yes' if motor_cal else '❌ No'}")
        
        encoder_ready = status.get('encoder_ready')
        if encoder_ready is not None:
            print(f"   Encoder Ready: {'✅ Yes' if encoder_ready else '❌ No'}")


def create_odrive_calibration(can_node: CanSimpleNode, node_id: int = 0) -> ODriveCalibration:
    """
    Factory function to create ODriveCalibration instance
    
    Args:
        can_node: CanSimpleNode instance for CAN communication
        node_id: ODrive node ID (default 0)
        
    Returns:
        ODriveCalibration instance
    """
    return ODriveCalibration(can_node, node_id)
