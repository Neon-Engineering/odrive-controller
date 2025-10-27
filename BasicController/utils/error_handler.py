#!/usr/bin/env python3
"""
Error Handler - Centralized error management and recovery

Provides comprehensive error detection, logging, and recovery procedures
for ODrive motor control systems.
"""

import asyncio
import time
from dataclasses import dataclass
from typing import Dict, List, Callable, Optional, Set
from enum import IntEnum
import struct

class ODriveAxisState(IntEnum):
    """ODrive axis states"""
    UNDEFINED = 0
    IDLE = 1
    STARTUP_SEQUENCE = 2
    FULL_CALIBRATION_SEQUENCE = 3
    MOTOR_CALIBRATION = 4
    ENCODER_INDEX_SEARCH = 6
    ENCODER_OFFSET_CALIBRATION = 7
    CLOSED_LOOP_CONTROL = 8
    LOCKIN_SPIN = 9
    ENCODER_DIR_FIND = 10
    HOMING = 11
    ENCODER_HALL_POLARITY_CALIBRATION = 12
    ENCODER_HALL_PHASE_CALIBRATION = 13

class ODriveErrorCode(IntEnum):
    """ODrive error codes"""
    NONE = 0x00
    INVALID_STATE = 0x01
    WATCHDOG_TIMER_EXPIRED = 0x800
    MIN_ENDSTOP_PRESSED = 0x1000
    MAX_ENDSTOP_PRESSED = 0x2000
    ESTOP_REQUESTED = 0x4000
    HOMING_WITHOUT_ENDSTOP = 0x20000
    OVER_TEMP = 0x40000
    UNKNOWN_POSITION = 0x80000

@dataclass
class ErrorEvent:
    """Error event data"""
    timestamp: float
    error_code: int
    axis_state: int
    description: str
    severity: str  # 'info', 'warning', 'error', 'critical'
    auto_recoverable: bool = False

class ErrorHandler:
    """
    Centralized error detection, logging, and recovery system
    """
    
    def __init__(self, can_manager, node_id: int = 0):
        self.can_manager = can_manager
        self.node_id = node_id
        
        # Error tracking
        self.active_errors: Set[int] = set()
        self.error_history: List[ErrorEvent] = []
        self.max_history = 500
        
        # Recovery state
        self.recovery_in_progress = False
        self.recovery_attempts = 0
        self.max_recovery_attempts = 3
        
        # Callbacks
        self.error_callbacks: List[Callable[[ErrorEvent], None]] = []
        self.recovery_callbacks: List[Callable[[str], None]] = []
        
        # Monitoring
        self.monitoring_active = False
        self.monitoring_task = None
        self.check_interval = 0.5  # seconds
        
        # Error descriptions
        self.error_descriptions = {
            0x00: ("No Error", "info", True),
            0x01: ("Invalid State", "error", True),
            0x02: ("DC Bus Under Voltage", "error", False),
            0x04: ("DC Bus Over Voltage", "error", False),
            0x08: ("Current Measurement Timeout", "error", False),
            0x10: ("Brake Resistor Disarmed", "warning", True),
            0x20: ("Motor Disarmed", "warning", True),
            0x40: ("Motor Failed", "critical", False),
            0x80: ("Sensorless Estimator Error", "error", False),
            0x100: ("Encoder Error", "error", False),
            0x200: ("Controller Error", "error", True),
            0x400: ("POS CTRL ERROR", "error", True),
            0x800: ("Watchdog Timer Expired", "critical", False),
            0x1000: ("Min Endstop Pressed", "warning", True),
            0x2000: ("Max Endstop Pressed", "warning", True),
            0x4000: ("Estop Requested", "critical", False),
            0x8000: ("Spinout Detected", "error", False),
            0x10000: ("Other Spinout Detected", "error", False),
            0x20000: ("Homing Without Endstop", "error", False),
            0x40000: ("Over Temperature", "critical", False),
            0x80000: ("Unknown Position", "error", True),
        }
        
        # State descriptions
        self.state_descriptions = {
            0: "UNDEFINED",
            1: "IDLE",
            2: "STARTUP_SEQUENCE", 
            3: "FULL_CALIBRATION_SEQUENCE",
            4: "MOTOR_CALIBRATION",
            6: "ENCODER_INDEX_SEARCH",
            7: "ENCODER_OFFSET_CALIBRATION",
            8: "CLOSED_LOOP_CONTROL",
            9: "LOCKIN_SPIN",
            10: "ENCODER_DIR_FIND",
            11: "HOMING",
            12: "ENCODER_HALL_POLARITY_CALIBRATION",
            13: "ENCODER_HALL_PHASE_CALIBRATION"
        }
    
    def add_error_callback(self, callback: Callable[[ErrorEvent], None]):
        """Add callback for error events"""
        self.error_callbacks.append(callback)
    
    def add_recovery_callback(self, callback: Callable[[str], None]):
        """Add callback for recovery events"""
        self.recovery_callbacks.append(callback)
    
    def _notify_error_callbacks(self, event: ErrorEvent):
        """Notify all error callbacks"""
        for callback in self.error_callbacks:
            try:
                callback(event)
            except Exception as e:
                print(f"⚠️  Error callback failed: {e}")
    
    def _notify_recovery_callbacks(self, message: str):
        """Notify all recovery callbacks"""
        for callback in self.recovery_callbacks:
            try:
                callback(message)
            except Exception as e:
                print(f"⚠️  Recovery callback failed: {e}")
    
    async def start_monitoring(self):
        """Start background error monitoring"""
        if self.monitoring_active:
            return
        
        self.monitoring_active = True
        self.monitoring_task = asyncio.create_task(self._monitoring_loop())
        print("🛡️  Error monitoring started")
    
    async def stop_monitoring(self):
        """Stop background error monitoring"""
        if not self.monitoring_active:
            return
        
        self.monitoring_active = False
        
        if self.monitoring_task:
            self.monitoring_task.cancel()
            try:
                await self.monitoring_task
            except asyncio.CancelledError:
                pass
        
        print("🛡️  Error monitoring stopped")
    
    async def _monitoring_loop(self):
        """Main error monitoring loop"""
        while self.monitoring_active:
            try:
                await self.check_errors()
                await asyncio.sleep(self.check_interval)
            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"⚠️  Error monitoring exception: {e}")
                await asyncio.sleep(1.0)
    
    async def check_errors(self) -> Optional[ErrorEvent]:
        """Check for current errors and state"""
        try:
            # Get heartbeat (axis state and error)
            response = await self.can_manager.send_message(
                cmd_id=0x01,  # Heartbeat
                expect_response=True
            )
            
            if not response or len(response.data) < 8:
                return None
            
            axis_state, error_code = struct.unpack('<II', response.data[:8])
            
            # Process error if present
            if error_code != 0:
                if error_code not in self.active_errors:
                    # New error detected
                    event = self._create_error_event(error_code, axis_state)
                    self.active_errors.add(error_code)
                    self._log_error_event(event)
                    self._notify_error_callbacks(event)
                    
                    # Attempt recovery if appropriate
                    if event.auto_recoverable and not self.recovery_in_progress:
                        await self._attempt_recovery(event)
                    
                    return event
            else:
                # Clear resolved errors
                if self.active_errors:
                    for error_code in list(self.active_errors):
                        event = ErrorEvent(
                            timestamp=time.time(),
                            error_code=0,
                            axis_state=axis_state,
                            description=f"Error {error_code:04X} resolved",
                            severity="info",
                            auto_recoverable=False
                        )
                        self._log_error_event(event)
                        self._notify_error_callbacks(event)
                    
                    self.active_errors.clear()
                    self.recovery_attempts = 0
            
            return None
            
        except Exception as e:
            print(f"⚠️  Error check failed: {e}")
            return None
    
    def _create_error_event(self, error_code: int, axis_state: int) -> ErrorEvent:
        """Create error event from error code and state"""
        # Decode composite error codes
        active_errors = self._decode_error_flags(error_code)
        
        if len(active_errors) == 1:
            # Single error
            description, severity, auto_recoverable = self.error_descriptions.get(
                active_errors[0], (f"Unknown Error 0x{active_errors[0]:04X}", "error", False)
            )
        elif len(active_errors) > 1:
            # Multiple errors - combine descriptions
            descriptions = []
            severities = []
            recoverable_flags = []
            
            for err_code in active_errors:
                desc, sev, rec = self.error_descriptions.get(
                    err_code, (f"Unknown 0x{err_code:04X}", "error", False)
                )
                descriptions.append(desc)
                severities.append(sev)
                recoverable_flags.append(rec)
            
            description = " + ".join(descriptions)
            # Use highest severity
            severity_priority = {'info': 0, 'warning': 1, 'error': 2, 'critical': 3}
            severity = max(severities, key=lambda s: severity_priority.get(s, 2))
            auto_recoverable = all(recoverable_flags)
        else:
            # No recognizable errors
            description, severity, auto_recoverable = (f"Unknown Error 0x{error_code:04X}", "error", False)
        
        state_desc = self.state_descriptions.get(axis_state, f"Unknown State {axis_state}")
        full_description = f"{description} (State: {state_desc})"
        
        return ErrorEvent(
            timestamp=time.time(),
            error_code=error_code,
            axis_state=axis_state,
            description=full_description,
            severity=severity,
            auto_recoverable=auto_recoverable
        )
    
    def _decode_error_flags(self, error_code: int) -> List[int]:
        """Decode composite error code into individual error flags"""
        active_errors = []
        
        # Check each known error bit
        for err_flag in self.error_descriptions.keys():
            if err_flag != 0 and (error_code & err_flag) == err_flag:
                active_errors.append(err_flag)
        
        return active_errors
    
    def _log_error_event(self, event: ErrorEvent):
        """Log error event to history"""
        self.error_history.append(event)
        
        # Trim history if too long
        if len(self.error_history) > self.max_history:
            self.error_history.pop(0)
        
        # Print to console
        severity_icon = {
            'info': 'ℹ️ ',
            'warning': '⚠️ ',
            'error': '❌',
            'critical': '🚨'
        }.get(event.severity, '❓')
        
        print(f"\n{severity_icon} {event.description}")
    
    async def _attempt_recovery(self, event: ErrorEvent):
        """Attempt automatic error recovery"""
        if self.recovery_in_progress or self.recovery_attempts >= self.max_recovery_attempts:
            return
        
        self.recovery_in_progress = True
        self.recovery_attempts += 1
        
        try:
            recovery_msg = f"Attempting recovery from {event.description} (attempt {self.recovery_attempts})"
            print(f"🔧 {recovery_msg}")
            self._notify_recovery_callbacks(recovery_msg)
            
            # Basic recovery procedure
            success = await self._execute_recovery_procedure(event)
            
            if success:
                recovery_msg = f"Recovery successful for error 0x{event.error_code:04X}"
                print(f"✅ {recovery_msg}")
                self._notify_recovery_callbacks(recovery_msg)
                self.recovery_attempts = 0
            else:
                recovery_msg = f"Recovery failed for error 0x{event.error_code:04X}"
                print(f"❌ {recovery_msg}")
                self._notify_recovery_callbacks(recovery_msg)
        
        except Exception as e:
            recovery_msg = f"Recovery exception: {e}"
            print(f"❌ {recovery_msg}")
            self._notify_recovery_callbacks(recovery_msg)
        
        finally:
            self.recovery_in_progress = False
    
    async def _execute_recovery_procedure(self, event: ErrorEvent) -> bool:
        """Execute recovery procedure based on error type"""
        try:
            if event.error_code == 0x01:  # Invalid State
                # Try to return to IDLE state
                await self.can_manager.send_message(
                    cmd_id=0x07,  # Set_Axis_State
                    data=struct.pack('<I', ODriveAxisState.IDLE),
                    expect_response=False
                )
                await asyncio.sleep(0.5)
                return True
            
            elif event.error_code == 0x20:  # Motor Disarmed
                # Motor disarmed is often expected, just acknowledge
                return True
            
            elif event.error_code == 0x80000:  # Unknown Position
                # Try to re-enable closed loop control
                await asyncio.sleep(1.0)  # Wait a bit
                await self.can_manager.send_message(
                    cmd_id=0x07,  # Set_Axis_State
                    data=struct.pack('<I', ODriveAxisState.CLOSED_LOOP_CONTROL),
                    expect_response=False
                )
                await asyncio.sleep(0.5)
                return True
            
            elif event.error_code in [0x200, 0x400]:  # Controller/Position errors
                # Clear error by going to IDLE then back to control
                await self.can_manager.send_message(
                    cmd_id=0x07,  # Set_Axis_State
                    data=struct.pack('<I', ODriveAxisState.IDLE),
                    expect_response=False
                )
                await asyncio.sleep(0.5)
                
                await self.can_manager.send_message(
                    cmd_id=0x07,  # Set_Axis_State
                    data=struct.pack('<I', ODriveAxisState.CLOSED_LOOP_CONTROL),
                    expect_response=False
                )
                await asyncio.sleep(0.5)
                return True
            
            # For other errors, just wait and see if they clear
            await asyncio.sleep(1.0)
            return False
            
        except Exception as e:
            print(f"Recovery procedure failed: {e}")
            return False
    
    async def clear_errors(self) -> bool:
        """Manually clear all errors"""
        try:
            print("🔧 Clearing errors...")
            
            # Set to IDLE state
            await self.can_manager.send_message(
                cmd_id=0x07,  # Set_Axis_State
                data=struct.pack('<I', ODriveAxisState.IDLE),
                expect_response=False
            )
            
            await asyncio.sleep(1.0)
            
            # Check if errors cleared
            await self.check_errors()
            
            if not self.active_errors:
                print("✅ Errors cleared")
                return True
            else:
                print(f"⚠️  {len(self.active_errors)} error(s) remain active")
                return False
                
        except Exception as e:
            print(f"❌ Failed to clear errors: {e}")
            return False
    
    def get_active_errors(self) -> List[int]:
        """Get list of currently active error codes"""
        return list(self.active_errors)
    
    def get_error_history(self) -> List[ErrorEvent]:
        """Get error history"""
        return self.error_history.copy()
    
    def get_error_description(self, error_code: int) -> str:
        """Get human-readable error description"""
        # Handle composite errors
        active_errors = self._decode_error_flags(error_code)
        
        if len(active_errors) == 1:
            description, _, _ = self.error_descriptions.get(
                active_errors[0], (f"Unknown Error 0x{active_errors[0]:04X}", "error", False)
            )
            return description
        elif len(active_errors) > 1:
            descriptions = []
            for err_code in active_errors:
                desc, _, _ = self.error_descriptions.get(
                    err_code, (f"Unknown 0x{err_code:04X}", "error", False)
                )
                descriptions.append(desc)
            return " + ".join(descriptions)
        else:
            return f"Unknown Error 0x{error_code:04X}"
    
    def get_error_breakdown(self, error_code: int) -> List[tuple]:
        """Get detailed breakdown of composite error code"""
        active_errors = self._decode_error_flags(error_code)
        breakdown = []
        
        for err_code in active_errors:
            description, severity, recoverable = self.error_descriptions.get(
                err_code, (f"Unknown Error 0x{err_code:04X}", "error", False)
            )
            breakdown.append((err_code, description, severity, recoverable))
        
        return breakdown
    
    def get_state_description(self, axis_state: int) -> str:
        """Get human-readable state description"""
        return self.state_descriptions.get(axis_state, f"Unknown State {axis_state}")
    
    def has_critical_errors(self) -> bool:
        """Check if any active errors are critical"""
        for error_code in self.active_errors:
            _, severity, _ = self.error_descriptions.get(error_code, ("", "error", False))
            if severity == "critical":
                return True
        return False
    
    def get_stats(self) -> Dict[str, any]:
        """Get error handler statistics"""
        return {
            'active_errors': len(self.active_errors),
            'total_errors_logged': len(self.error_history),
            'recovery_attempts': self.recovery_attempts,
            'recovery_in_progress': self.recovery_in_progress,
            'monitoring_active': self.monitoring_active,
            'has_critical_errors': self.has_critical_errors()
        }