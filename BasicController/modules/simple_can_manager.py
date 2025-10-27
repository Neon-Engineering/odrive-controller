#!/usr/bin/env python3
"""
Simple CAN Manager Module

Simplified high-performance CAN communication manager.
Uses direct queue-based communication without complex asyncio/threading integration.
"""

import can
import struct
import threading
import queue
import time
from typing import Optional, Dict, Any
from dataclasses import dataclass

# Import from parent directory
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent))
from utils.can_simple_utils import CanSimpleNode
import libusb_backend_workaround

# Initialize libusb backend
libusb_backend_workaround.find_libusb_backend()

@dataclass
class CANMessage:
    """CAN message data structure"""
    arbitration_id: int
    data: bytes
    timestamp: float
    is_response: bool = False

class SimpleCANManager:
    """
    Simple high-performance CAN manager with thread-safe queues
    """
    
    def __init__(self, node_id: int = 0, bitrate: int = 500000):
        self.node_id = node_id
        self.bitrate = bitrate
        self.bus = None
        self.node = None
        self._node_context = None
        
        # Simple queues
        self.command_queue = queue.Queue(maxsize=1000)
        self.telemetry_queue = queue.Queue(maxsize=1000)
        
        # Threading control
        self.running = False
        self.sender_thread = None
        self.receiver_thread = None
        
        # Performance tracking
        self.stats = {
            'commands_sent': 0,
            'responses_received': 0,
            'errors': 0
        }
    
    async def initialize(self) -> bool:
        """Initialize CAN bus and start threads"""
        try:
            print("🔌 Initializing Simple CAN Manager...")
            
            # Setup CAN bus
            self.bus = can.Bus(
                interface='gs_usb',
                channel='can0',
                index=0,
                bitrate=self.bitrate
            )
            
            # Create node
            self.node = CanSimpleNode(self.bus, self.node_id)
            self._node_context = self.node.__enter__()
            
            # Clear errors
            self.node.clear_errors_msg()
            
            # Start threads
            self.running = True
            self._start_threads()
            
            print("✅ Simple CAN Manager initialized")
            return True
            
        except Exception as e:
            print(f"❌ CAN Manager initialization failed: {e}")
            return False
    
    def _start_threads(self):
        """Start sender and receiver threads"""
        self.sender_thread = threading.Thread(
            target=self._sender_thread,
            daemon=True,
            name="CAN-Sender"
        )
        self.sender_thread.start()
        
        self.receiver_thread = threading.Thread(
            target=self._receiver_thread,
            daemon=True,
            name="CAN-Receiver"
        )
        self.receiver_thread.start()
    
    def _sender_thread(self):
        """Message sender thread"""
        while self.running:
            try:
                message = self.command_queue.get(timeout=0.1)
                
                # Send CAN message
                can_msg = can.Message(
                    arbitration_id=message.arbitration_id,
                    data=message.data,
                    is_extended_id=False,
                    is_remote_frame=(len(message.data) == 0)
                )
                
                # Debug: Log what we're sending (position and velocity commands)
                cmd_type = message.arbitration_id & 0x1F
                if cmd_type == 0x0C:  # Position command
                    if len(message.data) >= 4:
                        position = struct.unpack('<f', message.data[:4])[0]
                        velocity_ff = struct.unpack('<f', message.data[4:8])[0] if len(message.data) >= 8 else 0.0
                        print(f"🔄 CAN TX: Position {position:.3f} turns, vel_ff {velocity_ff:.3f} (ID: 0x{message.arbitration_id:02X})")
                elif cmd_type == 0x0D:  # Velocity command  
                    if len(message.data) >= 4:
                        velocity = struct.unpack('<f', message.data[:4])[0]
                        torque_ff = struct.unpack('<f', message.data[4:8])[0] if len(message.data) >= 8 else 0.0
                        print(f"🔄 CAN TX: Velocity {velocity:.3f} turns/s, torque_ff {torque_ff:.3f} (ID: 0x{message.arbitration_id:02X})")
                
                self.bus.send(can_msg)
                self.stats['commands_sent'] += 1
                self.command_queue.task_done()
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"❌ CAN send error: {e}")
                self.stats['errors'] += 1
    
    def _receiver_thread(self):
        """Message receiver thread"""
        while self.running:
            try:
                can_msg = self.bus.recv(timeout=0.1)
                
                if can_msg:
                    message = CANMessage(
                        arbitration_id=can_msg.arbitration_id,
                        data=can_msg.data,
                        timestamp=time.time(),
                        is_response=True
                    )
                    
                    # Route to telemetry queue
                    cmd_id = can_msg.arbitration_id & 0x1F
                    if cmd_id in [0x09, 0x15, 0x17, 0x14]:  # Sensor data
                        try:
                            self.telemetry_queue.put_nowait(message)
                        except queue.Full:
                            # Remove old data and add new
                            try:
                                self.telemetry_queue.get_nowait()
                                self.telemetry_queue.put_nowait(message)
                            except:
                                pass
                    
                    self.stats['responses_received'] += 1
                    
            except:
                continue
    
    # Simple command methods
    def send_position_command(self, position: float, velocity_ff: float = 0.0) -> bool:
        """Send position command (thread-safe)"""
        try:
            message = CANMessage(
                arbitration_id=(self.node_id << 5 | 0x0C),
                data=struct.pack('<ff', position, velocity_ff),
                timestamp=time.time()
            )
            
            self.command_queue.put_nowait(message)
            return True
            
        except queue.Full:
            return False
    
    def send_velocity_command(self, velocity: float, torque_ff: float = 0.0) -> bool:
        """Send velocity command (thread-safe, like working can_simple.py)"""
        try:
            message = CANMessage(
                arbitration_id=(self.node_id << 5 | 0x0D),
                data=struct.pack('<ff', velocity, torque_ff),
                timestamp=time.time()
            )
            
            self.command_queue.put_nowait(message)
            return True
            
        except queue.Full:
            return False
    
    def request_encoder_data(self) -> bool:
        """Request encoder data (thread-safe)"""
        try:
            message = CANMessage(
                arbitration_id=(self.node_id << 5 | 0x09),
                data=b'',  # RTR message
                timestamp=time.time()
            )
            
            self.command_queue.put_nowait(message)
            return True
            
        except queue.Full:
            return False
    
    def request_voltage_current_data(self) -> bool:
        """Request bus voltage and current data (thread-safe)"""
        try:
            message = CANMessage(
                arbitration_id=(self.node_id << 5 | 0x17),
                data=b'',  # RTR message
                timestamp=time.time()
            )
            
            self.command_queue.put_nowait(message)
            return True
            
        except queue.Full:
            return False
    
    def request_motor_current_data(self) -> bool:
        """Request motor current (Iq) data (thread-safe)"""
        try:
            message = CANMessage(
                arbitration_id=(self.node_id << 5 | 0x14),
                data=b'',  # RTR message
                timestamp=time.time()
            )
            
            self.command_queue.put_nowait(message)
            return True
            
        except queue.Full:
            return False
    
    def request_torque_data(self) -> bool:
        """Request torque data (torque_setpoint and torque_estimate) (thread-safe)"""
        try:
            message = CANMessage(
                arbitration_id=(self.node_id << 5 | 0x1C),
                data=b'',  # RTR message
                timestamp=time.time()
            )
            
            self.command_queue.put_nowait(message)
            return True
            
        except queue.Full:
            return False
    
    def request_temperature_data(self) -> bool:
        """Request temperature data (FET and motor temperature) (thread-safe)"""
        try:
            message = CANMessage(
                arbitration_id=(self.node_id << 5 | 0x15),
                data=b'',  # RTR message
                timestamp=time.time()
            )
            
            self.command_queue.put_nowait(message)
            return True
            
        except queue.Full:
            return False
    
    def get_latest_telemetry(self) -> Optional[CANMessage]:
        """Get latest telemetry (non-blocking)"""
        try:
            return self.telemetry_queue.get_nowait()
        except queue.Empty:
            return None
    
    def get_performance_stats(self) -> Dict[str, Any]:
        """Get performance statistics"""
        return {
            'command_queue_size': self.command_queue.qsize(),
            'telemetry_queue_size': self.telemetry_queue.qsize(),
            'commands_sent': self.stats['commands_sent'],
            'responses_received': self.stats['responses_received'],
            'errors': self.stats['errors']
        }
    
    async def shutdown(self):
        """Shutdown CAN manager"""
        print("🛑 Shutting down Simple CAN Manager...")
        
        self.running = False
        
        # Wait for threads
        if self.sender_thread and self.sender_thread.is_alive():
            self.sender_thread.join(timeout=1.0)
        
        if self.receiver_thread and self.receiver_thread.is_alive():
            self.receiver_thread.join(timeout=1.0)
        
        # Close resources
        if self._node_context:
            self._node_context.__exit__(None, None, None)
        
        if self.bus:
            self.bus.shutdown()
        
        print("✅ Simple CAN Manager shutdown complete")

# Factory function
def create_simple_can_manager(node_id: int = 0) -> SimpleCANManager:
    """Create simple CAN manager"""
    return SimpleCANManager(node_id=node_id)