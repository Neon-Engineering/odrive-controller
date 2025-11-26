#!/usr/bin/env python3
"""
CAN Node ID Discovery Utility

Scans the CAN bus to discover active ODrive controllers and their node IDs.
Uses heartbeat messages (0x01) to identify active nodes.
"""

import can
import asyncio
import struct
import time
from typing import List, Dict, Optional, Tuple

# Heartbeat message CMD ID
HEARTBEAT_CMD = 0x01

# ODrive Axis State Constants
AXIS_STATE_UNDEFINED = 0
AXIS_STATE_IDLE = 1
AXIS_STATE_STARTUP_SEQUENCE = 2
AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3
AXIS_STATE_MOTOR_CALIBRATION = 4
AXIS_STATE_ENCODER_INDEX_SEARCH = 6
AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7
AXIS_STATE_CLOSED_LOOP_CONTROL = 8


class NodeDiscovery:
    """
    CAN Node ID Discovery
    
    Scans the CAN bus for active ODrive controllers by monitoring heartbeat messages.
    """
    
    def __init__(self, bus: can.Bus):
        """
        Initialize node discovery
        
        Args:
            bus: CAN bus instance
        """
        self.bus = bus
    
    async def discover_nodes(self, timeout: float = 3.0, max_node_id: int = 63) -> List[Dict]:
        """
        Discover active ODrive nodes on the CAN bus
        
        This method passively listens for heartbeat messages from ODrives.
        Heartbeat messages are sent automatically by ODrives at ~10Hz.
        
        Args:
            timeout: How long to listen for heartbeat messages (seconds)
            max_node_id: Maximum node ID to consider (default 63)
            
        Returns:
            List of discovered nodes with their information:
            [
                {
                    'node_id': 0,
                    'axis_state': 1,
                    'axis_error': 0,
                    'last_seen': 1234567890.123
                },
                ...
            ]
        """
        print(f"🔍 Scanning CAN bus for ODrive nodes (timeout: {timeout}s)...")
        print("   Listening for heartbeat messages...")
        
        discovered_nodes = {}
        start_time = time.time()
        
        # Listen for heartbeat messages
        while (time.time() - start_time) < timeout:
            try:
                msg = self.bus.recv(timeout=0.1)
                
                if msg:
                    # Check if this is a heartbeat message
                    cmd_id = msg.arbitration_id & 0x1F
                    node_id = (msg.arbitration_id >> 5) & 0x3F
                    
                    if cmd_id == HEARTBEAT_CMD and node_id <= max_node_id:
                        # Parse heartbeat message
                        if len(msg.data) >= 8:
                            axis_error = struct.unpack('<I', msg.data[:4])[0]
                            axis_state = msg.data[4]
                            
                            # Add or update node info
                            discovered_nodes[node_id] = {
                                'node_id': node_id,
                                'axis_state': axis_state,
                                'axis_error': axis_error,
                                'last_seen': time.time()
                            }
                            
            except can.CanError as e:
                print(f"⚠️ CAN error during discovery: {e}")
                continue
            except Exception:
                continue
        
        # Convert to sorted list
        nodes = sorted(discovered_nodes.values(), key=lambda x: x['node_id'])
        
        if nodes:
            print(f"\n✅ Discovered {len(nodes)} ODrive(s):")
            for node in nodes:
                state_name = self._get_state_name(node['axis_state'])
                error_str = f"error=0x{node['axis_error']:08X}" if node['axis_error'] != 0 else "no errors"
                print(f"   • Node ID {node['node_id']}: {state_name}, {error_str}")
        else:
            print("❌ No ODrive nodes discovered on CAN bus")
            print("💡 Possible issues:")
            print("   - ODrive not powered")
            print("   - CAN bus not connected")
            print("   - Wrong CAN bitrate")
            print("   - ODrive not configured for CAN communication")
        
        return nodes
    
    async def active_scan_nodes(self, timeout_per_node: float = 0.2, max_node_id: int = 63) -> List[Dict]:
        """
        Actively scan for ODrive nodes by probing each node ID
        
        This method actively sends requests to each possible node ID to detect
        ODrives that may not be broadcasting heartbeat messages (e.g., auto-assigned IDs).
        This is more aggressive than passive discovery but guaranteed to find all nodes.
        
        Args:
            timeout_per_node: How long to wait for each node to respond (seconds)
            max_node_id: Maximum node ID to scan (default 63)
            
        Returns:
            List of discovered nodes with their information
        """
        print(f"🔍 Active scanning for ODrive nodes (IDs 0-{max_node_id})...")
        print("   Probing each node ID with encoder requests...")
        
        discovered_nodes = {}
        
        for node_id in range(max_node_id + 1):
            # Send encoder request
            request_msg = can.Message(
                arbitration_id=(node_id << 5 | 0x09),  # 0x09 = GET_ENCODER_ESTIMATES
                data=b'',
                is_extended_id=False,
                is_remote_frame=True
            )
            
            try:
                self.bus.send(request_msg)
            except can.CanError:
                continue
            
            # Wait for response
            start_time = time.time()
            
            while (time.time() - start_time) < timeout_per_node:
                try:
                    msg = self.bus.recv(timeout=0.05)
                    
                    if msg:
                        cmd_id = msg.arbitration_id & 0x1F
                        msg_node_id = (msg.arbitration_id >> 5) & 0x3F
                        
                        # Check for encoder data response
                        if cmd_id == 0x09 and msg_node_id == node_id:
                            # Node responded! Now try to get heartbeat for state info
                            discovered_nodes[node_id] = {
                                'node_id': node_id,
                                'axis_state': None,
                                'axis_error': None,
                                'last_seen': time.time(),
                                'detected_by': 'active_scan'
                            }
                            print(f"   ✅ Node ID {node_id} responded to encoder request")
                            break
                        
                        # Also check for heartbeat messages while waiting
                        elif cmd_id == HEARTBEAT_CMD and msg_node_id == node_id:
                            if len(msg.data) >= 8:
                                axis_error = struct.unpack('<I', msg.data[:4])[0]
                                axis_state = msg.data[4]
                                
                                discovered_nodes[node_id] = {
                                    'node_id': node_id,
                                    'axis_state': axis_state,
                                    'axis_error': axis_error,
                                    'last_seen': time.time(),
                                    'detected_by': 'heartbeat'
                                }
                                print(f"   ✅ Node ID {node_id} sent heartbeat")
                                break
                                
                except:
                    continue
            
            # Small delay between probes to avoid flooding the bus
            await asyncio.sleep(0.01)
        
        # Convert to sorted list
        nodes = sorted(discovered_nodes.values(), key=lambda x: x['node_id'])
        
        if nodes:
            print(f"\n✅ Active scan found {len(nodes)} ODrive(s):")
            for node in nodes:
                if node['axis_state'] is not None:
                    state_name = self._get_state_name(node['axis_state'])
                    error_str = f"error=0x{node['axis_error']:08X}" if node['axis_error'] != 0 else "no errors"
                    print(f"   • Node ID {node['node_id']}: {state_name}, {error_str}")
                else:
                    print(f"   • Node ID {node['node_id']}: Responds to encoder requests")
        else:
            print("❌ No ODrive nodes found during active scan")
        
        return nodes
    
    async def discover_single_node(self, timeout: float = 3.0, use_active_scan: bool = False) -> Optional[int]:
        """
        Discover a single ODrive node (convenience method)
        
        Args:
            timeout: How long to listen for heartbeat messages
            use_active_scan: If True, use active scanning instead of passive listening
            
        Returns:
            Node ID if exactly one node found, None otherwise
        """
        if use_active_scan:
            nodes = await self.active_scan_nodes(timeout_per_node=0.2)
        else:
            nodes = await self.discover_nodes(timeout=timeout)
        
        if len(nodes) == 1:
            print(f"✅ Found single ODrive at node ID {nodes[0]['node_id']}")
            return nodes[0]['node_id']
        elif len(nodes) > 1:
            print(f"⚠️ Multiple ODrives found: {[n['node_id'] for n in nodes]}")
            print("💡 Please specify which node ID to use with --node-id")
            return None
        else:
            return None
    
    async def verify_node(self, node_id: int, timeout: float = 2.0) -> bool:
        """
        Verify that a specific node ID is present and responding
        
        Args:
            node_id: Node ID to verify
            timeout: How long to wait for heartbeat
            
        Returns:
            True if node responds, False otherwise
        """
        print(f"🔍 Verifying node ID {node_id}...")
        
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            try:
                msg = self.bus.recv(timeout=0.1)
                
                if msg:
                    cmd_id = msg.arbitration_id & 0x1F
                    msg_node_id = (msg.arbitration_id >> 5) & 0x3F
                    
                    if cmd_id == HEARTBEAT_CMD and msg_node_id == node_id:
                        print(f"✅ Node ID {node_id} verified and responding")
                        return True
                        
            except:
                continue
        
        print(f"❌ Node ID {node_id} not responding")
        return False
    
    async def request_encoder_and_wait(self, node_id: int, timeout: float = 1.0) -> Optional[Tuple[float, float]]:
        """
        Request encoder data from a specific node and wait for response
        
        This is a more active test to verify the node can respond to requests.
        
        Args:
            node_id: Node ID to test
            timeout: How long to wait for response
            
        Returns:
            Tuple of (position, velocity) if successful, None otherwise
        """
        print(f"📡 Testing encoder communication with node ID {node_id}...")
        
        # Send RTR (Remote Transmission Request) for encoder data
        request_msg = can.Message(
            arbitration_id=(node_id << 5 | 0x09),  # 0x09 = GET_ENCODER_ESTIMATES
            data=b'',
            is_extended_id=False,
            is_remote_frame=True
        )
        
        try:
            self.bus.send(request_msg)
        except can.CanError as e:
            print(f"❌ Failed to send encoder request: {e}")
            return None
        
        # Wait for response
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            try:
                msg = self.bus.recv(timeout=0.1)
                
                if msg:
                    cmd_id = msg.arbitration_id & 0x1F
                    msg_node_id = (msg.arbitration_id >> 5) & 0x3F
                    
                    # Check if this is encoder data from our node
                    if cmd_id == 0x09 and msg_node_id == node_id and len(msg.data) >= 8:
                        position, velocity = struct.unpack('<ff', msg.data[:8])
                        print(f"✅ Encoder data received: pos={position:.3f} turns, vel={velocity:.3f} turns/s")
                        return (position, velocity)
                        
            except:
                continue
        
        print(f"⚠️ No encoder response from node ID {node_id}")
        print("💡 This may indicate:")
        print("   - Motor not calibrated (MISSING_ESTIMATE error)")
        print("   - ODrive in IDLE state")
        print("   - Encoder not configured")
        return None
    
    def _get_state_name(self, state: int) -> str:
        """Convert axis state number to human-readable name"""
        state_names = {
            0: "UNDEFINED",
            1: "IDLE",
            2: "STARTUP_SEQUENCE",
            3: "FULL_CALIBRATION",
            4: "MOTOR_CALIBRATION",
            6: "ENCODER_INDEX_SEARCH",
            7: "ENCODER_OFFSET_CALIBRATION",
            8: "CLOSED_LOOP_CONTROL"
        }
        return state_names.get(state, f"UNKNOWN({state})")


async def discover_odrive_nodes(bus: can.Bus, timeout: float = 3.0) -> List[Dict]:
    """
    Convenience function to discover ODrive nodes
    
    Args:
        bus: CAN bus instance
        timeout: Discovery timeout in seconds
        
    Returns:
        List of discovered nodes
    """
    discovery = NodeDiscovery(bus)
    return await discovery.discover_nodes(timeout=timeout)


async def auto_detect_node_id(bus: can.Bus, timeout: float = 3.0) -> Optional[int]:
    """
    Automatically detect node ID if only one ODrive is present
    
    Args:
        bus: CAN bus instance
        timeout: Discovery timeout in seconds
        
    Returns:
        Node ID if exactly one found, None otherwise
    """
    discovery = NodeDiscovery(bus)
    return await discovery.discover_single_node(timeout=timeout)
