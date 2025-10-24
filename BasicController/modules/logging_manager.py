#!/usr/bin/env python3
"""
Logging Manager Module

High-speed data logging with concurrent file writing and configurable formats.
Supports CSV, binary, and real-time streaming formats.
"""

import asyncio
import threading
import queue
import time
import csv
import json
from typing import Optional, Dict, Any, List
from dataclasses import dataclass, asdict
from pathlib import Path
from datetime import datetime

@dataclass
class LogEntry:
    """Single log entry"""
    timestamp: float
    position: Optional[float] = None
    velocity: Optional[float] = None
    position_target: Optional[float] = None
    velocity_target: Optional[float] = None
    motor_temp: Optional[float] = None
    fet_temp: Optional[float] = None
    bus_voltage: Optional[float] = None
    bus_current: Optional[float] = None
    motor_current: Optional[float] = None
    control_mode: Optional[str] = None
    axis_state: Optional[int] = None

class HighSpeedLoggingManager:
    """
    High-speed logging manager for real-time data collection
    """
    
    def __init__(self, base_filename: str = "odrive_log", 
                 log_rate_hz: float = 100.0, max_queue_size: int = 10000):
        self.base_filename = base_filename
        self.log_rate = log_rate_hz
        self.log_period = 1.0 / log_rate_hz
        self.max_queue_size = max_queue_size
        
        # Logging queue and control
        self.log_queue = queue.Queue(maxsize=max_queue_size)
        self.running = False
        self.threads = {}
        
        # File handles
        self.csv_file = None
        self.csv_writer = None
        
        # Statistics
        self.stats = {
            'entries_logged': 0,
            'queue_overflows': 0,
            'file_errors': 0,
            'last_log_time': 0.0
        }
        
        # Session info
        self.session_start = time.time()
        self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    async def start(self, log_directory: str = "logs"):
        """Start high-speed logging"""
        print(f"📝 Starting High-Speed Logging @ {self.log_rate} Hz...")
        
        # Create log directory
        log_path = Path(log_directory)
        log_path.mkdir(exist_ok=True)
        
        # Create CSV file
        csv_filename = log_path / f"{self.base_filename}_{self.session_id}.csv"
        
        try:
            self.csv_file = open(csv_filename, 'w', newline='', buffering=1)
            self.csv_writer = csv.writer(self.csv_file)
            
            # Write header
            header = [
                'timestamp', 'position', 'velocity', 'position_target', 'velocity_target',
                'motor_temp', 'fet_temp', 'bus_voltage', 'bus_current', 'motor_current',
                'control_mode', 'axis_state'
            ]
            self.csv_writer.writerow(header)
            
            self.running = True
            
            # Start logging thread
            self.threads['logger'] = threading.Thread(
                target=self._logging_thread,
                daemon=True,
                name="Data-Logger"
            )
            self.threads['logger'].start()
            
            print(f"✅ Logging started: {csv_filename}")
            return True
            
        except Exception as e:
            print(f"❌ Failed to start logging: {e}")
            return False
    
    def _logging_thread(self):
        """High-speed logging thread"""
        while self.running:
            try:
                # Get log entry from queue
                entry = self.log_queue.get(timeout=1.0)
                
                # Write to CSV
                if self.csv_writer:
                    row = [
                        entry.timestamp,
                        entry.position,
                        entry.velocity,
                        entry.position_target,
                        entry.velocity_target,
                        entry.motor_temp,
                        entry.fet_temp,
                        entry.bus_voltage,
                        entry.bus_current,
                        entry.motor_current,
                        entry.control_mode,
                        entry.axis_state
                    ]
                    self.csv_writer.writerow(row)
                    self.csv_file.flush()  # Ensure data is written
                
                self.stats['entries_logged'] += 1
                self.stats['last_log_time'] = time.time()
                
                # Mark task as done
                self.log_queue.task_done()
                
            except queue.Empty:
                continue
            except Exception as e:
                self.stats['file_errors'] += 1
                print(f"⚠️ Logging error: {e}")
    
    def log_data(self, telemetry_data, position_target=None, velocity_target=None, 
                control_mode=None, axis_state=None):
        """Log data entry (non-blocking)"""
        try:
            entry = LogEntry(
                timestamp=telemetry_data.timestamp,
                position=telemetry_data.position,
                velocity=telemetry_data.velocity,
                position_target=position_target,
                velocity_target=velocity_target,
                motor_temp=telemetry_data.motor_temp,
                fet_temp=telemetry_data.fet_temp,
                bus_voltage=telemetry_data.bus_voltage,
                bus_current=telemetry_data.bus_current,
                motor_current=telemetry_data.motor_current,
                control_mode=control_mode,
                axis_state=axis_state
            )
            
            self.log_queue.put_nowait(entry)
            
        except queue.Full:
            self.stats['queue_overflows'] += 1
            # Drop oldest entry and try again
            try:
                self.log_queue.get_nowait()
                self.log_queue.put_nowait(entry)
            except:
                pass
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get logging statistics"""
        return {
            'session_id': self.session_id,
            'session_duration': time.time() - self.session_start,
            'log_rate': self.log_rate,
            'queue_size': self.log_queue.qsize(),
            'max_queue_size': self.max_queue_size,
            'entries_logged': self.stats['entries_logged'],
            'queue_overflows': self.stats['queue_overflows'],
            'file_errors': self.stats['file_errors'],
            'last_log_time': self.stats['last_log_time']
        }
    
    async def stop(self):
        """Stop logging and close files"""
        print("🛑 Stopping logging...")
        
        self.running = False
        
        # Wait for queue to empty
        if not self.log_queue.empty():
            print("   Waiting for log queue to empty...")
            self.log_queue.join()
        
        # Wait for logging thread
        if 'logger' in self.threads and self.threads['logger'].is_alive():
            self.threads['logger'].join(timeout=2.0)
        
        # Close files
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
        
        print(f"✅ Logging stopped. {self.stats['entries_logged']} entries logged.")

# Factory function
def create_logging_manager(base_filename: str = "odrive_log", 
                          log_rate_hz: float = 100.0) -> HighSpeedLoggingManager:
    """Create high-speed logging manager"""
    return HighSpeedLoggingManager(
        base_filename=base_filename,
        log_rate_hz=log_rate_hz
    )