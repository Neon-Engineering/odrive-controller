import asyncio
import can
import struct
import json

ADDRESS_CMD = 0x06
SET_AXIS_STATE_CMD = 0x07
REBOOT_CMD = 0x16
CLEAR_ERRORS_CMD = 0x18
RX_SDO_CMD = 0x04
TX_SDO_CMD = 0x05

REBOOT_ACTION_REBOOT = 0
REBOOT_ACTION_SAVE = 1
REBOOT_ACTION_ERASE = 2

OPCODE_READ = 0x00
OPCODE_WRITE = 0x01

# Load endpoint data for parameter access
try:
    with open('flat_endpoints.json', 'r') as f:
        endpoint_data = json.load(f)
        endpoints = endpoint_data['endpoints']
except FileNotFoundError:
    endpoints = None

# Format lookup for parameter types
format_lookup = {
    'bool': '?',
    'uint8': 'B', 'int8': 'b',
    'uint16': 'H', 'int16': 'h',
    'uint32': 'I', 'int32': 'i',
    'uint64': 'Q', 'int64': 'q',
    'float': 'f'
}

class CanSimpleNode():
    def __init__(self, bus: can.Bus, node_id: int):
        self.bus = bus
        self.node_id = node_id
        self.reader = can.AsyncBufferedReader()

    def __enter__(self):
        self.notifier = can.Notifier(self.bus, [self.reader], loop=asyncio.get_running_loop())
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.notifier.stop()
        pass

    def flush_rx(self):
        while not self.reader.buffer.empty():
            self.reader.buffer.get_nowait()

    def await_msg(self, cmd_id: int, timeout=1.0):
        async def _impl():
            async for msg in self.reader:
                if msg.arbitration_id == (self.node_id << 5 | cmd_id):
                    return msg
        return asyncio.wait_for(_impl(), timeout)

    def clear_errors_msg(self, identify: bool = False):
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5) | CLEAR_ERRORS_CMD,
            data=b'\x01' if identify else b'\x00',
            is_extended_id=False
        ))

    def reboot_msg(self, action: int):
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5) | REBOOT_CMD,
            data=[action],
            is_extended_id=False
        ))

    def set_state_msg(self, state: int):
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | SET_AXIS_STATE_CMD),
            data=struct.pack('<I', state),
            is_extended_id=False
        ))
    
    def set_controller_mode_msg(self, control_mode: int, input_mode: int):
        """Set controller mode and input mode via CAN simple command (0x0B)
        
        Args:
            control_mode: Control mode (0=voltage, 1=torque, 2=velocity, 3=position)
            input_mode: Input mode (1=passthrough, 2=vel_ramp, 3=pos_filter, 5=trap_traj, etc.)
        """
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | 0x0B),
            data=struct.pack('<II', control_mode, input_mode),
            is_extended_id=False
        ))

    async def write_parameter(self, path: str, value):
        """Write a parameter via CAN SDO"""
        if endpoints is None:
            raise RuntimeError("flat_endpoints.json not loaded")
        
        if path not in endpoints:
            raise ValueError(f"Parameter {path} not found in endpoints")
        
        endpoint_id = endpoints[path]['id']
        endpoint_type = endpoints[path]['type']
        
        # Flush RX buffer
        self.flush_rx()
        
        # Pack the value according to its type
        format_char = format_lookup[endpoint_type]
        data = struct.pack('<BHB' + format_char, OPCODE_WRITE, endpoint_id, 0, value)
        
        # Send write command
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | RX_SDO_CMD),
            data=data,
            is_extended_id=False
        ))
        
        # Wait for acknowledgment (increased timeout for better reliability)
        try:
            await self.await_msg(TX_SDO_CMD, timeout=3.0)
        except asyncio.TimeoutError:
            raise RuntimeError(f"Timeout writing parameter {path} (ODrive may not be responding to CAN parameter writes)")

    async def read_parameter(self, path: str):
        """Read a parameter via CAN SDO"""
        if endpoints is None:
            raise RuntimeError("flat_endpoints.json not loaded")
        
        if path not in endpoints:
            raise ValueError(f"Parameter {path} not found in endpoints")
        
        endpoint_id = endpoints[path]['id']
        endpoint_type = endpoints[path]['type']
        
        # Flush RX buffer
        self.flush_rx()
        
        # Send read command
        self.bus.send(can.Message(
            arbitration_id=(self.node_id << 5 | RX_SDO_CMD),
            data=struct.pack('<BHB', OPCODE_READ, endpoint_id, 0),
            is_extended_id=False
        ))
        
        # Wait for response (increased timeout for better reliability)
        try:
            msg = await self.await_msg(TX_SDO_CMD, timeout=3.0)
            format_char = format_lookup[endpoint_type]
            _, _, _, return_value = struct.unpack_from('<BHB' + format_char, msg.data)
            return return_value
        except asyncio.TimeoutError:
            raise RuntimeError(f"Timeout reading parameter {path} (ODrive may not be responding to CAN parameter reads)")

    async def set_control_mode(self, control_mode: int):
        """Set the control mode (0=voltage, 1=torque, 2=velocity, 3=position)"""
        await self.write_parameter('axis0.controller.config.control_mode', control_mode)

    async def set_input_mode(self, input_mode: int):
        """Set the input mode (0=inactive, 1=passthrough, 2=vel_ramp, etc.)"""
        await self.write_parameter('axis0.controller.config.input_mode', input_mode)
