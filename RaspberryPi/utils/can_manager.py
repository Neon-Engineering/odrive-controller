#!/usr/bin/env python3
"""
CAN Manager - Core CAN Communication Handler

Provides robust, non-blocking CAN communication with message queuing,
background processing, and automatic recovery.
"""

import asyncio
import can
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional, Dict, Callable, Any
from concurrent.futures import ThreadPoolExecutor

@dataclass
class CANMessage:
    """CAN message with metadata"""
    arbitration_id: int
    data: bytes
    timestamp: float
    is_response: bool = False
    expected_response_id: Optional[int] = None

class CANManager:
    """
    Robust CAN communication manager with background message handling.
    
    Features:
    - Background message processing to prevent blocking
    - Message queuing and response matching
    - Automatic retry and error recovery
    - Thread-safe operation
    """
    
    def __init__(self, interface='socketcan', channel='can0', bitrate=500000, node_id=0):
        self.node_id = node_id
        self.bus = None
        self.running = False
        
        # Message queues
        self.outbound_queue = asyncio.Queue()
        self.response_queue = asyncio.Queue()
        self.pending_responses = {}  # msg_id -> (future, timestamp)
        
        # Background tasks
        self.sender_task = None
        self.receiver_task = None
        
        # Configuration
        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate
        self.response_timeout = 1.0  # seconds
        
        # Statistics
        self.stats = {
            'messages_sent': 0,
            'messages_received': 0,
            'timeouts': 0,
            'errors': 0
        }
        
        # Thread executor for blocking operations
        self.executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="CAN")
        
    async def start(self):
        """Start the CAN manager and background tasks"""
        try:
            # Initialize CAN bus in thread executor to prevent blocking
            self.bus = await asyncio.get_event_loop().run_in_executor(
                self.executor,
                self._init_can_bus
            )
            
            self.running = True
            
            # Start background tasks
            self.sender_task = asyncio.create_task(self._sender_loop())
            self.receiver_task = asyncio.create_task(self._receiver_loop())
            
            print(f"✅ CAN Manager started (Node ID: {self.node_id})")
            return True
            
        except Exception as e:
            print(f"❌ Failed to start CAN Manager: {e}")
            return False
    
    def _init_can_bus(self):
        """Initialize CAN bus (runs in thread executor)"""
        # Allow runtime override via environment variable
        import os, platform
        backend = os.environ.get('ODRIVE_CAN_BACKEND') or self.interface
        if backend is None:
            backend = 'socketcan' if platform.system().lower() == 'linux' else 'gs_usb'

        if backend == 'socketcan':
            # socketcan backend expects the OS to configure bitrate
            return can.Bus(interface='socketcan', channel=self.channel)
        else:
            return can.Bus(interface='gs_usb', channel=self.channel, index=0, bitrate=self.bitrate)
    
    async def stop(self):
        """Stop the CAN manager and cleanup"""
        self.running = False
        
        # Cancel background tasks
        if self.sender_task:
            self.sender_task.cancel()
        if self.receiver_task:
            self.receiver_task.cancel()
        
        # Wait for tasks to complete
        if self.sender_task:
            try:
                await self.sender_task
            except asyncio.CancelledError:
                pass
        if self.receiver_task:
            try:
                await self.receiver_task
            except asyncio.CancelledError:
                pass
        
        # Shutdown CAN bus
        if self.bus:
            await asyncio.get_event_loop().run_in_executor(
                self.executor,
                self.bus.shutdown
            )
        
        # Shutdown executor
        self.executor.shutdown(wait=True)
        print("✅ CAN Manager stopped")
    
    async def send_message(self, cmd_id: int, data: bytes = b'', expect_response: bool = False) -> Optional[can.Message]:
        """
        Send a CAN message with optional response waiting
        
        Args:
            cmd_id: Command ID (will be combined with node_id)
            data: Message data bytes
            expect_response: Whether to wait for a response
            
        Returns:
            Response message if expect_response=True, None otherwise
        """
        arbitration_id = (self.node_id << 5) | cmd_id
        
        msg = CANMessage(
            arbitration_id=arbitration_id,
            data=data,
            timestamp=time.time(),
            is_response=False,
            expected_response_id=arbitration_id if expect_response else None
        )
        
        # Queue message for sending
        await self.outbound_queue.put(msg)
        
        if expect_response:
            # Create future for response
            future = asyncio.Future()
            self.pending_responses[arbitration_id] = (future, time.time())
            
            try:
                # Wait for response with timeout
                response = await asyncio.wait_for(future, timeout=self.response_timeout)
                return response
            except asyncio.TimeoutError:
                self.stats['timeouts'] += 1
                # Clean up pending response
                if arbitration_id in self.pending_responses:
                    del self.pending_responses[arbitration_id]
                return None
        
        return None
    
    async def _sender_loop(self):
        """Background task to send queued messages"""
        while self.running:
            try:
                # Get message from queue
                msg = await asyncio.wait_for(self.outbound_queue.get(), timeout=0.1)
                
                # Send message in thread executor to prevent blocking
                can_msg = can.Message(
                    arbitration_id=msg.arbitration_id,
                    data=msg.data,
                    is_extended_id=False
                )
                
                await asyncio.get_event_loop().run_in_executor(
                    self.executor,
                    self.bus.send,
                    can_msg
                )
                
                self.stats['messages_sent'] += 1
                
            except asyncio.TimeoutError:
                # No messages to send, continue
                continue
            except Exception as e:
                self.stats['errors'] += 1
                print(f"⚠️  CAN send error: {e}")
                await asyncio.sleep(0.1)  # Brief pause on error
    
    async def _receiver_loop(self):
        """Background task to receive messages"""
        while self.running:
            try:
                # Receive message in thread executor to prevent blocking
                msg = await asyncio.get_event_loop().run_in_executor(
                    self.executor,
                    self._recv_message_blocking
                )
                
                if msg:
                    self.stats['messages_received'] += 1
                    
                    # Check if this is a response to a pending request
                    if msg.arbitration_id in self.pending_responses:
                        future, _ = self.pending_responses.pop(msg.arbitration_id)
                        if not future.cancelled():
                            future.set_result(msg)
                    else:
                        # Queue unsolicited message for processing
                        await self.response_queue.put(msg)
                
            except Exception as e:
                self.stats['errors'] += 1
                print(f"⚠️  CAN receive error: {e}")
                await asyncio.sleep(0.1)  # Brief pause on error
    
    def _recv_message_blocking(self, timeout=0.1):
        """Receive a CAN message (blocking call for thread executor)"""
        try:
            return self.bus.recv(timeout=timeout)
        except can.CanTimeoutError:
            return None
        except Exception as e:
            raise e
    
    async def get_unsolicited_message(self, timeout=0.1):
        """Get an unsolicited message from the queue"""
        try:
            return await asyncio.wait_for(self.response_queue.get(), timeout=timeout)
        except asyncio.TimeoutError:
            return None
    
    def get_stats(self):
        """Get communication statistics"""
        return self.stats.copy()
    
    # Cleanup pending responses periodically
    async def _cleanup_pending(self):
        """Clean up expired pending responses"""
        current_time = time.time()
        expired = []
        
        for msg_id, (future, timestamp) in self.pending_responses.items():
            if current_time - timestamp > self.response_timeout:
                expired.append(msg_id)
        
        for msg_id in expired:
            future, _ = self.pending_responses.pop(msg_id)
            if not future.cancelled():
                future.cancel()