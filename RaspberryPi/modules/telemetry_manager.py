#!/usr/bin/env python3
"""
Telemetry Manager Module

High-speed telemetry collection and processing with parallel data streams.
Supports multiple sensor types and configurable update rates.
"""

import asyncio
import threading
import queue
import time
import struct
from typing import Optional, Dict, Any, Callable, List
from dataclasses import dataclass, field
from collections import deque
import statistics

@dataclass
class TelemetryData:
    """Telemetry data structure"""
    timestamp: float
    position: Optional[float] = None
    velocity: Optional[float] = None
    motor_temp: Optional[float] = None
    fet_temp: Optional[float] = None
    bus_voltage: Optional[float] = None
    bus_current: Optional[float] = None
    motor_current: Optional[float] = None
    axis_state: Optional[int] = None

@dataclass 
class TelemetryStats:
    """Telemetry statistics"""
    update_rate: float = 0.0
    data_points: int = 0
    last_update: float = 0.0
    missing_data_rate: float = 0.0

class HighSpeedTelemetryManager:
    """
    High-speed telemetry manager with concurrent data collection
    """
    
    def __init__(self, can_manager, target_rate_hz: float = 100.0):
        self.can_manager = can_manager
        self.target_rate_hz = target_rate_hz
        self.target_period = 1.0 / target_rate_hz
        
        # Data storage
        self.latest_data = TelemetryData(timestamp=time.time())
        self.data_history = deque(maxlen=1000)  # Keep last 1000 samples
        self.data_lock = threading.RLock()
        
        # Processing threads
        self.running = False
        self.threads = {}
        
        # Statistics
        self.stats = TelemetryStats()
        
        # Data callbacks
        self.data_callbacks = []
        
        # Request tracking
        self.last_encoder_request = 0.0
        self.last_temp_request = 0.0
        self.last_voltage_request = 0.0
        self.last_current_request = 0.0
        
        # Request intervals (staggered for optimal performance)
        self.encoder_interval = 0.01    # 100 Hz
        self.temp_interval = 0.1        # 10 Hz
        self.voltage_interval = 0.05    # 20 Hz
        self.current_interval = 0.02    # 50 Hz
    
    async def start(self):
        """Start high-speed telemetry collection"""
        print(f"📡 Starting High-Speed Telemetry Manager @ {self.target_rate_hz} Hz...")
        
        self.running = True
        
        # Start data collection thread
        self.threads['collector'] = threading.Thread(
            target=self._data_collection_thread,
            daemon=True,
            name="Telemetry-Collector"
        )
        self.threads['collector'].start()
        
        # Start data processing thread
        self.threads['processor'] = threading.Thread(
            target=self._data_processing_thread, 
            daemon=True,
            name="Telemetry-Processor"
        )
        self.threads['processor'].start()
        
        # Start statistics thread
        self.threads['stats'] = threading.Thread(
            target=self._statistics_thread,
            daemon=True,
            name="Telemetry-Stats"
        )
        self.threads['stats'].start()
        
        print("✅ Telemetry Manager started successfully")
    
    def _data_collection_thread(self):
        """High-speed data collection thread"""
        next_collection_time = time.time()
        
        while self.running:
            current_time = time.time()
            
            # Staggered data requests for optimal CAN bus utilization
            self._request_staggered_data(current_time)
            
            # Wait for next collection cycle
            next_collection_time += self.target_period
            sleep_time = next_collection_time - time.time()
            
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # We're running behind, skip the sleep
                next_collection_time = time.time()
    
    def _request_staggered_data(self, current_time: float):
        """Request different data types at staggered intervals"""
        # Request encoder data (highest priority) - direct queue access
        if current_time - self.last_encoder_request >= self.encoder_interval:
            self._request_encoder_data_direct()
            self.last_encoder_request = current_time
        
        # Request temperature data
        if current_time - self.last_temp_request >= self.temp_interval:
            self._request_temperature_data()
            self.last_temp_request = current_time
        
        # Request voltage data
        if current_time - self.last_voltage_request >= self.voltage_interval:
            self._request_voltage_data()
            self.last_voltage_request = current_time
        
        # Request current data
        if current_time - self.last_current_request >= self.current_interval:
            self._request_current_data()
            self.last_current_request = current_time
    
    def _request_encoder_data_direct(self):
        """Request encoder data directly (thread-safe)"""
        try:
            from .can_manager import CANMessage
            
            message = CANMessage(
                arbitration_id=(self.can_manager.node_id << 5 | 0x09),
                data=b'',  # RTR message
                timestamp=time.time()
            )
            self.can_manager.command_queue.put_nowait(message)
        except:
            pass
    
    def _request_temperature_data(self):
        """Request temperature data"""
        try:
            # Import here to avoid circular import
            from .can_manager import CANMessage
            
            # Temperature request (RTR message)
            message = CANMessage(
                arbitration_id=(self.can_manager.node_id << 5 | 0x15),
                data=b'',
                timestamp=time.time()
            )
            self.can_manager.command_queue.put_nowait(message)
        except:
            pass
    
    def _request_voltage_data(self):
        """Request bus voltage data"""
        try:
            from .can_manager import CANMessage
            
            message = CANMessage(
                arbitration_id=(self.can_manager.node_id << 5 | 0x17),
                data=b'',
                timestamp=time.time()
            )
            self.can_manager.command_queue.put_nowait(message)
        except:
            pass
    
    def _request_current_data(self):
        """Request motor current data"""
        try:
            from .can_manager import CANMessage
            
            message = CANMessage(
                arbitration_id=(self.can_manager.node_id << 5 | 0x14),
                data=b'',
                timestamp=time.time()
            )
            self.can_manager.command_queue.put_nowait(message)
        except:
            pass
    
    def _data_processing_thread(self):
        """Process incoming telemetry data"""
        while self.running:
            try:
                # Get telemetry message directly from queue (thread-safe)
                try:
                    message = self.can_manager.telemetry_queue.get(timeout=0.1)
                    if message:
                        self._process_telemetry_message(message)
                        self.can_manager.telemetry_queue.task_done()
                except queue.Empty:
                    continue
                
            except Exception as e:
                time.sleep(0.001)  # Brief pause on error
    
    def _process_telemetry_message(self, message):
        """Process individual telemetry message"""
        # Extract command ID
        cmd_id = message.arbitration_id & 0x1F
        
        with self.data_lock:
            # Update timestamp
            self.latest_data.timestamp = message.timestamp
            
            # Process based on command type
            if cmd_id == 0x09 and len(message.data) >= 8:
                # Encoder data
                pos, vel = struct.unpack('<ff', message.data[:8])
                self.latest_data.position = pos
                self.latest_data.velocity = vel
                
            elif cmd_id == 0x15 and len(message.data) >= 8:
                # Temperature data
                fet_temp, motor_temp = struct.unpack('<ff', message.data[:8])
                self.latest_data.fet_temp = fet_temp
                self.latest_data.motor_temp = motor_temp
                
            elif cmd_id == 0x17 and len(message.data) >= 8:
                # Voltage/current data
                bus_voltage, bus_current = struct.unpack('<ff', message.data[:8])
                self.latest_data.bus_voltage = bus_voltage
                self.latest_data.bus_current = bus_current
                
            elif cmd_id == 0x14 and len(message.data) >= 4:
                # Motor current data
                motor_current = struct.unpack('<f', message.data[:4])[0]
                self.latest_data.motor_current = motor_current
            
            # Add to history
            self.data_history.append(self.latest_data)
            
            # Update statistics
            self.stats.data_points += 1
            
            # Call callbacks
            for callback in self.data_callbacks:
                try:
                    callback(self.latest_data)
                except Exception as e:
                    print(f"⚠️ Telemetry callback error: {e}")
    
    def _statistics_thread(self):
        """Calculate telemetry statistics"""
        last_count = 0
        last_time = time.time()
        
        while self.running:
            time.sleep(1.0)  # Update every second
            
            current_time = time.time()
            current_count = self.stats.data_points
            
            time_delta = current_time - last_time
            count_delta = current_count - last_count
            
            if time_delta > 0:
                self.stats.update_rate = count_delta / time_delta
                self.stats.last_update = current_time
                
                # Calculate missing data rate
                expected_data = self.target_rate_hz * time_delta
                if expected_data > 0:
                    self.stats.missing_data_rate = max(0, (expected_data - count_delta) / expected_data * 100)
            
            last_count = current_count
            last_time = current_time
    
    # Public API methods
    def get_latest_data(self) -> TelemetryData:
        """Get latest telemetry data (thread-safe)"""
        with self.data_lock:
            return TelemetryData(
                timestamp=self.latest_data.timestamp,
                position=self.latest_data.position,
                velocity=self.latest_data.velocity,
                motor_temp=self.latest_data.motor_temp,
                fet_temp=self.latest_data.fet_temp,
                bus_voltage=self.latest_data.bus_voltage,
                bus_current=self.latest_data.bus_current,
                motor_current=self.latest_data.motor_current,
                axis_state=self.latest_data.axis_state
            )
    
    def get_data_history(self, max_samples: int = 100) -> List[TelemetryData]:
        """Get recent telemetry history"""
        with self.data_lock:
            return list(self.data_history)[-max_samples:]
    
    def get_statistics(self) -> TelemetryStats:
        """Get telemetry statistics"""
        return TelemetryStats(
            update_rate=self.stats.update_rate,
            data_points=self.stats.data_points,
            last_update=self.stats.last_update,
            missing_data_rate=self.stats.missing_data_rate
        )
    
    def register_data_callback(self, callback: Callable[[TelemetryData], None]):
        """Register callback for new telemetry data"""
        self.data_callbacks.append(callback)
    
    def get_position_velocity(self) -> tuple[Optional[float], Optional[float]]:
        """Get current position and velocity (fast access)"""
        with self.data_lock:
            return self.latest_data.position, self.latest_data.velocity
    
    def get_temperatures(self) -> tuple[Optional[float], Optional[float]]:
        """Get current temperatures (motor, FET)"""
        with self.data_lock:
            return self.latest_data.motor_temp, self.latest_data.fet_temp
    
    def get_power_data(self) -> tuple[Optional[float], Optional[float], Optional[float]]:
        """Get power-related data (bus voltage, bus current, motor current)"""
        with self.data_lock:
            return (self.latest_data.bus_voltage, 
                   self.latest_data.bus_current, 
                   self.latest_data.motor_current)
    
    def calculate_position_statistics(self, window_size: int = 50) -> Dict[str, float]:
        """Calculate position-based statistics"""
        with self.data_lock:
            recent_data = list(self.data_history)[-window_size:]
            
            if not recent_data:
                return {}
            
            positions = [d.position for d in recent_data if d.position is not None]
            velocities = [d.velocity for d in recent_data if d.velocity is not None]
            
            stats = {}
            
            if positions:
                stats['position_mean'] = statistics.mean(positions)
                stats['position_stdev'] = statistics.stdev(positions) if len(positions) > 1 else 0.0
                stats['position_range'] = max(positions) - min(positions)
            
            if velocities:
                stats['velocity_mean'] = statistics.mean(velocities)
                stats['velocity_stdev'] = statistics.stdev(velocities) if len(velocities) > 1 else 0.0
                stats['velocity_max'] = max(abs(v) for v in velocities)
            
            return stats
    
    async def stop(self):
        """Stop telemetry collection"""
        print("🛑 Stopping Telemetry Manager...")
        
        self.running = False
        
        # Wait for threads to finish
        for name, thread in self.threads.items():
            if thread.is_alive():
                thread.join(timeout=1.0)
                print(f"   {name} thread stopped")
        
        print("✅ Telemetry Manager stopped")

# Factory function
def create_telemetry_manager(can_manager, target_rate_hz: float = 100.0) -> HighSpeedTelemetryManager:
    """Create high-speed telemetry manager"""
    return HighSpeedTelemetryManager(can_manager=can_manager, target_rate_hz=target_rate_hz)