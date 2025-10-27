#!/usr/bin/env python3
"""
Telemetry Manager - Background telemetry collection and monitoring

Provides non-blocking telemetry collection with configurable update rates,
data logging, and real-time display.
"""

import asyncio
import time
from dataclasses import dataclass, field
from typing import Optional, List, Callable, Dict, Any
from datetime import datetime
import csv
import os

@dataclass
class TelemetryData:
    """Telemetry data structure"""
    timestamp: float = field(default_factory=time.time)
    position: Optional[float] = None
    velocity: Optional[float] = None
    current: Optional[float] = None
    voltage: Optional[float] = None
    fet_temp: Optional[float] = None
    motor_temp: Optional[float] = None
    axis_state: Optional[int] = None
    error_code: Optional[int] = None
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for logging"""
        return {
            'timestamp': self.timestamp,
            'datetime': datetime.fromtimestamp(self.timestamp).isoformat(),
            'position': self.position,
            'velocity': self.velocity,
            'current': self.current,
            'voltage': self.voltage,
            'fet_temp': self.fet_temp,
            'motor_temp': self.motor_temp,
            'axis_state': self.axis_state,
            'error_code': self.error_code
        }
    
    def format_display(self) -> str:
        """Format for console display"""
        pos_str = f"{self.position:.3f}" if self.position is not None else "N/A"
        vel_str = f"{self.velocity:.3f}" if self.velocity is not None else "N/A"
        fet_temp_str = f"{self.fet_temp:.1f}°C" if self.fet_temp is not None else "N/A"
        motor_temp_str = f"{self.motor_temp:.1f}°C" if self.motor_temp is not None else "N/A"
        
        return (f"Pos: {pos_str:>8} turns | "
                f"Vel: {vel_str:>8} t/s | "
                f"FET: {fet_temp_str:>6} | "
                f"Motor: {motor_temp_str:>6} | "
                f"State: {self.axis_state or 'N/A'}")

class TelemetryLogger:
    """CSV logger for telemetry data"""
    
    def __init__(self, filename: str, buffer_size: int = 100):
        self.filename = filename
        self.buffer_size = buffer_size
        self.buffer: List[TelemetryData] = []
        self.file_handle = None
        self.csv_writer = None
        self.header_written = False
        
    def start(self):
        """Start logging"""
        try:
            self.file_handle = open(self.filename, 'w', newline='', encoding='utf-8')
            self.csv_writer = csv.DictWriter(
                self.file_handle, 
                fieldnames=['timestamp', 'datetime', 'position', 'velocity', 
                           'current', 'voltage', 'fet_temp', 'motor_temp', 
                           'axis_state', 'error_code']
            )
            self.csv_writer.writeheader()
            self.header_written = True
            print(f"📊 Telemetry logging started: {self.filename}")
        except Exception as e:
            print(f"❌ Failed to start telemetry logging: {e}")
    
    def log(self, data: TelemetryData):
        """Log telemetry data"""
        if not self.csv_writer:
            return
        
        self.buffer.append(data)
        
        # Flush buffer when full
        if len(self.buffer) >= self.buffer_size:
            self.flush()
    
    def flush(self):
        """Flush buffer to file"""
        if not self.csv_writer or not self.buffer:
            return
        
        try:
            for data in self.buffer:
                self.csv_writer.writerow(data.to_dict())
            self.file_handle.flush()
            self.buffer.clear()
        except Exception as e:
            print(f"⚠️  Telemetry logging error: {e}")
    
    def stop(self):
        """Stop logging and cleanup"""
        if self.buffer:
            self.flush()
        
        if self.file_handle:
            self.file_handle.close()
            self.file_handle = None
            self.csv_writer = None
        
        print("📊 Telemetry logging stopped")

class TelemetryManager:
    """
    Background telemetry collection and monitoring system
    """
    
    def __init__(self, motor_controller, update_rate_hz: float = 10.0):
        self.motor_controller = motor_controller
        self.update_rate_hz = update_rate_hz
        self.update_interval = 1.0 / update_rate_hz
        
        # State
        self.running = False
        self.monitoring_task = None
        self.display_enabled = True
        
        # Data storage
        self.current_data = TelemetryData()
        self.history: List[TelemetryData] = []
        self.max_history = 1000  # Keep last 1000 samples
        
        # Logging
        self.logger: Optional[TelemetryLogger] = None
        
        # Callbacks
        self.data_callbacks: List[Callable[[TelemetryData], None]] = []
        self.error_callbacks: List[Callable[[str], None]] = []
        
        # Display control
        self.should_display_func: Optional[Callable[[], bool]] = None
        
    def set_display_condition(self, condition_func: Callable[[], bool]):
        """Set condition function for when to display telemetry"""
        self.should_display_func = condition_func
    
    def enable_logging(self, filename: Optional[str] = None, buffer_size: int = 100):
        """Enable CSV logging"""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"telemetry_{timestamp}.csv"
        
        self.logger = TelemetryLogger(filename, buffer_size)
        self.logger.start()
    
    def disable_logging(self):
        """Disable CSV logging"""
        if self.logger:
            self.logger.stop()
            self.logger = None
    
    def add_data_callback(self, callback: Callable[[TelemetryData], None]):
        """Add callback for new telemetry data"""
        self.data_callbacks.append(callback)
    
    def add_error_callback(self, callback: Callable[[str], None]):
        """Add callback for telemetry errors"""
        self.error_callbacks.append(callback)
    
    def _notify_data_callbacks(self, data: TelemetryData):
        """Notify all data callbacks"""
        for callback in self.data_callbacks:
            try:
                callback(data)
            except Exception as e:
                print(f"⚠️  Telemetry data callback error: {e}")
    
    def _notify_error_callbacks(self, error_msg: str):
        """Notify all error callbacks"""
        for callback in self.error_callbacks:
            try:
                callback(error_msg)
            except Exception as e:
                print(f"⚠️  Telemetry error callback error: {e}")
    
    async def start(self):
        """Start telemetry monitoring"""
        if self.running:
            return
        
        self.running = True
        self.monitoring_task = asyncio.create_task(self._monitoring_loop())
        print(f"📡 Telemetry monitoring started ({self.update_rate_hz} Hz)")
    
    async def stop(self):
        """Stop telemetry monitoring"""
        if not self.running:
            return
        
        self.running = False
        
        if self.monitoring_task:
            self.monitoring_task.cancel()
            try:
                await self.monitoring_task
            except asyncio.CancelledError:
                pass
        
        # Stop logging
        if self.logger:
            self.logger.stop()
        
        print("📡 Telemetry monitoring stopped")
    
    async def _monitoring_loop(self):
        """Main telemetry monitoring loop"""
        while self.running:
            try:
                # Collect telemetry data
                await self._collect_data()
                
                # Display if conditions are met
                if self._should_display():
                    self._display_data()
                
                # Log data
                if self.logger:
                    self.logger.log(self.current_data)
                
                # Notify callbacks
                self._notify_data_callbacks(self.current_data)
                
                # Store in history
                self.history.append(self.current_data)
                if len(self.history) > self.max_history:
                    self.history.pop(0)
                
                # Wait for next update
                await asyncio.sleep(self.update_interval)
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                error_msg = f"Telemetry monitoring error: {e}"
                print(f"⚠️  {error_msg}")
                self._notify_error_callbacks(error_msg)
                await asyncio.sleep(1.0)  # Pause on error
    
    async def _collect_data(self):
        """Collect current telemetry data"""
        # Update motor controller status
        await self.motor_controller.update_status()
        status = self.motor_controller.get_status()
        
        # Create telemetry data
        self.current_data = TelemetryData(
            timestamp=time.time(),
            position=status.position,
            velocity=status.velocity,
            current=status.current,
            voltage=status.voltage,
            fet_temp=status.fet_temp,
            motor_temp=status.motor_temp,
            axis_state=status.axis_state,
            error_code=status.error_code
        )
    
    def _should_display(self) -> bool:
        """Determine if telemetry should be displayed"""
        if not self.display_enabled:
            return False
        
        if self.should_display_func:
            return self.should_display_func()
        
        return True
    
    def _display_data(self):
        """Display current telemetry data"""
        print(f"\r{self.current_data.format_display()}", end='', flush=True)
    
    def get_current_data(self) -> TelemetryData:
        """Get current telemetry data"""
        return self.current_data
    
    def get_history(self) -> List[TelemetryData]:
        """Get telemetry history"""
        return self.history.copy()
    
    def clear_history(self):
        """Clear telemetry history"""
        self.history.clear()
    
    def set_display_enabled(self, enabled: bool):
        """Enable/disable console display"""
        self.display_enabled = enabled
    
    def get_stats(self) -> Dict[str, Any]:
        """Get telemetry statistics"""
        if not self.history:
            return {}
        
        positions = [d.position for d in self.history if d.position is not None]
        velocities = [d.velocity for d in self.history if d.velocity is not None]
        temps = [d.fet_temp for d in self.history if d.fet_temp is not None]
        
        stats = {
            'samples_collected': len(self.history),
            'update_rate_hz': self.update_rate_hz,
            'running': self.running
        }
        
        if positions:
            stats.update({
                'position_min': min(positions),
                'position_max': max(positions),
                'position_current': positions[-1]
            })
        
        if velocities:
            stats.update({
                'velocity_min': min(velocities),
                'velocity_max': max(velocities),
                'velocity_current': velocities[-1]
            })
        
        if temps:
            stats.update({
                'temp_min': min(temps),
                'temp_max': max(temps),
                'temp_current': temps[-1]
            })
        
        return stats