"""
ODrive CAN Control System - Utilities Package

Modular utilities for robust ODrive CAN communication and control.
"""

__version__ = "1.0.0"
__author__ = "ODrive CAN Control System"

# Import key classes for easy access
from .can_manager import CANManager
from .motor_controller import MotorController, MotorLimits, MotorStatus, ControlState, InputMode
from .telemetry_manager import TelemetryManager, TelemetryData, TelemetryLogger
from .error_handler import ErrorHandler, ErrorEvent, ODriveAxisState, ODriveErrorCode

__all__ = [
    'CANManager',
    'MotorController', 
    'MotorLimits',
    'MotorStatus',
    'ControlState',
    'InputMode',
    'TelemetryManager',
    'TelemetryData', 
    'TelemetryLogger',
    'ErrorHandler',
    'ErrorEvent',
    'ODriveAxisState',
    'ODriveErrorCode'
]