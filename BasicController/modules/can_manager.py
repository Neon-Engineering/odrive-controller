#!/usr/bin/env python3
"""
CAN Manager Module

High-performance CAN communication manager with optimized message handling.
Supports concurrent read/write operations and message queuing.
"""

import asyncio
import can
import struct
import threading
import queue
import time
from typing import Optional, Callable, Dict, Any
from dataclasses import dataclass
from concurrent.futures import ThreadPoolExecutor

# Import from parent directory
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent))
from can_simple_utils import CanSimpleNode
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

class HighPerformanceCANManager:
    """
    High-performance CAN manager with concurrent message handling
    """
    
    def __init__(self, node_id: int = 0, bitrate: int = 500000):
        self.node_id = node_id
        self.bitrate = bitrate
        self.bus = None
        self.node = None
        self._node_context = None
        
        # High-performance queues
        self.command_queue = queue.Queue(maxsize=1000)
        self.response_queue = queue.Queue(maxsize=1000)
        self.telemetry_queue = queue.Queue(maxsize=1000)
        
        # Threading control
        self.running = False
        self.threads = {}
        self.executor = ThreadPoolExecutor(max_workers=4)
        
        # Performance tracking
        self.stats = {
            'commands_sent': 0,
            'responses_received': 0,
            'errors': 0,
            'last_update': time.time()
        }
        
        # Message callbacks
        self.message_callbacks = {}
    
    async def initialize(self) -> bool:
        """Initialize CAN bus and start background threads"""
        try:
            print("🔌 Initializing High-Performance CAN Manager...")
            
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
            
            # Start background threads
            self.running = True
            self._start_background_threads()
            
            print("✅ CAN Manager initialized successfully")
            return True
            
        except Exception as e:
            print(f"❌ CAN Manager initialization failed: {e}")
            return False
    
    def _start_background_threads(self):
        """Start high-performance background threads"""
        # Message sender thread
        self.threads['sender'] = threading.Thread(
            target=self._message_sender_thread,
            daemon=True,
            name="CAN-Sender"
        )
        self.threads['sender'].start()
        
        # Message receiver thread
        self.threads['receiver'] = threading.Thread(
            target=self._message_receiver_thread,
            daemon=True,
            name="CAN-Receiver"
        )
        self.threads['receiver'].start()
        
        # Performance monitor thread
        self.threads['monitor'] = threading.Thread(
            target=self._performance_monitor_thread,
            daemon=True,
            name="CAN-Monitor"
        )
        self.threads['monitor'].start()
    
    def _message_sender_thread(self):
        """High-performance message sender thread"""
        while self.running:
            try:
                # Get message from queue (blocking with timeout)
                message = self.command_queue.get(timeout=0.1)
                
                # Send message
                can_msg = can.Message(
                    arbitration_id=message.arbitration_id,
                    data=message.data,
                    is_extended_id=False
                )
                
                self.bus.send(can_msg)
                self.stats['commands_sent'] += 1
                
                # Mark task as done
                self.command_queue.task_done()
                
            except queue.Empty:
                continue
            except Exception as e:
                self.stats['errors'] += 1
                print(f"⚠️ CAN send error: {e}")
    
    def _message_receiver_thread(self):
        """High-performance message receiver thread"""
        while self.running:
            try:
                # Receive message (non-blocking)
                can_msg = self.bus.recv(timeout=0.1)
                
                if can_msg:
                    message = CANMessage(
                        arbitration_id=can_msg.arbitration_id,
                        data=can_msg.data,
                        timestamp=time.time(),
                        is_response=True
                    )
                    
                    # Route message to appropriate queue
                    self._route_received_message(message)
                    self.stats['responses_received'] += 1
                    
            except Exception as e:
                # Normal timeout, continue
                continue
    
    def _route_received_message(self, message: CANMessage):
        """Route received messages to appropriate handlers"""
        # Extract node ID and command ID
        node_id = (message.arbitration_id >> 5) & 0x3F
        cmd_id = message.arbitration_id & 0x1F
        
        # Route to telemetry if it's encoder/sensor data
        if cmd_id in [0x09, 0x15, 0x17, 0x14]:  # Encoder, temp, voltage, current
            try:
                self.telemetry_queue.put_nowait(message)
            except queue.Full:
                pass  # Drop old telemetry if queue is full
        else:
            # Route to response queue
            try:
                self.response_queue.put_nowait(message)
            except queue.Full:
                pass  # Drop old responses if queue is full
        
        # Call registered callbacks
        callback_key = f"{node_id}_{cmd_id}"
        if callback_key in self.message_callbacks:
            try:
                self.message_callbacks[callback_key](message)
            except Exception as e:
                print(f"⚠️ Callback error: {e}")
    
    def _performance_monitor_thread(self):
        """Monitor CAN performance"""
        while self.running:
            time.sleep(5.0)  # Update every 5 seconds
            
            current_time = time.time()
            time_delta = current_time - self.stats['last_update']
            
            if time_delta > 0:
                cmd_rate = self.stats['commands_sent'] / time_delta
                resp_rate = self.stats['responses_received'] / time_delta
                
                print(f"📊 CAN Performance: {cmd_rate:.1f} cmd/s, {resp_rate:.1f} resp/s, {self.stats['errors']} errors")
                
                # Reset counters
                self.stats['commands_sent'] = 0
                self.stats['responses_received'] = 0
                self.stats['errors'] = 0
                self.stats['last_update'] = current_time
    
    # High-level command methods
    async def send_position_command(self, position: float, velocity_ff: float = 0.0) -> bool:
        """Send position command (non-blocking)"""
        try:
            message = CANMessage(
                arbitration_id=(self.node_id << 5 | 0x0C),
                data=struct.pack('<ff', position, velocity_ff),
                timestamp=time.time()
            )
            
            self.command_queue.put_nowait(message)
            return True
            
        except queue.Full:
            print("⚠️ Command queue full, dropping command")
            return False
        except Exception as e:
            print(f"❌ Error queueing position command: {e}")
            return False
    
    async def send_velocity_command(self, velocity: float, torque_ff: float = 0.0) -> bool:
        """Send velocity command (non-blocking)"""
        try:
            message = CANMessage(
                arbitration_id=(self.node_id << 5 | 0x0D),
                data=struct.pack('<ff', velocity, torque_ff),
                timestamp=time.time()
            )
            
            self.command_queue.put_nowait(message)
            return True
            
        except queue.Full:
            print("⚠️ Command queue full, dropping command")
            return False
        except Exception as e:
            print(f"❌ Error queueing velocity command: {e}")
            return False
    
    async def send_torque_command(self, torque: float) -> bool:
        """Send torque command (non-blocking)"""
        try:
            message = CANMessage(
                arbitration_id=(self.node_id << 5 | 0x0E),
                data=struct.pack('<f', torque),
                timestamp=time.time()
            )
            
            self.command_queue.put_nowait(message)
            return True
            
        except queue.Full:
            print("⚠️ Command queue full, dropping command")
            return False
        except Exception as e:
            print(f"❌ Error queueing torque command: {e}")
            return False
    
    async def request_encoder_data(self) -> bool:
        """Request encoder data (non-blocking)"""
        try:
            message = CANMessage(
                arbitration_id=(self.node_id << 5 | 0x09),
                data=b'',  # RTR message
                timestamp=time.time()
            )
            
            self.command_queue.put_nowait(message)
            return True
            
        except Exception as e:
            print(f"❌ Error requesting encoder data: {e}")
            return False
    
    def register_message_callback(self, node_id: int, cmd_id: int, callback: Callable):
        """Register callback for specific message types"""
        key = f"{node_id}_{cmd_id}"
        self.message_callbacks[key] = callback
    
    async def get_latest_telemetry(self) -> Optional[CANMessage]:
        """Get latest telemetry message (non-blocking)"""
        try:
            return self.telemetry_queue.get_nowait()
        except queue.Empty:
            return None
    
    async def get_response(self, timeout: float = 0.1) -> Optional[CANMessage]:
        """Get response message with timeout"""
        try:
            return self.response_queue.get(timeout=timeout)
        except queue.Empty:
            return None
    
    def get_performance_stats(self) -> Dict[str, Any]:
        """Get current performance statistics"""
        return {
            'command_queue_size': self.command_queue.qsize(),
            'response_queue_size': self.response_queue.qsize(),
            'telemetry_queue_size': self.telemetry_queue.qsize(),
            'threads_alive': {name: thread.is_alive() for name, thread in self.threads.items()},
            'stats': self.stats.copy()
        }
    
    async def shutdown(self):
        """Shutdown CAN manager gracefully"""
        print("🛑 Shutting down CAN Manager...")
        
        self.running = False
        
        # Wait for threads to finish
        for name, thread in self.threads.items():
            if thread.is_alive():
                thread.join(timeout=1.0)
                print(f"   {name} thread stopped")
        
        # Shutdown executor
        self.executor.shutdown(wait=True)
        
        # Close CAN resources
        if self._node_context:
            self._node_context.__exit__(None, None, None)
        
        if self.bus:
            self.bus.shutdown()
        
        print("✅ CAN Manager shutdown complete")

# Factory function for easy creation
def create_can_manager(node_id: int = 0, bitrate: int = 500000) -> HighPerformanceCANManager:
    """Create and return a high-performance CAN manager"""
    return HighPerformanceCANManager(node_id=node_id, bitrate=bitrate)