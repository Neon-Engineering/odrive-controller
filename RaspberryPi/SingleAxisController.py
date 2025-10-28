# Import all libraries ----------
import sys								# Accesses system-specific parameters (ex. sys.exit() to quit script)
import time								# Provides time-related functions (ex. time.sleep())
import math								# Provides standard mathematical operations (ex. sin, cos, sqrt)
import struct
from typing import Optional							# Used for converting between Python values and C-style binary data (critical for packing/unpackign CAN messages)
import can								# Provides interfaces for sending/receiving CAN messages
from collections import deque			# Double-ended queue, great for maintaining rolling buffer (ex. real-time plot)



# -----------------------------
# ODrive axis state constants
# -----------------------------
AXIS_STATE_IDLE        = 1
AXIS_STATE_CLOSED_LOOP = 8

# -----------------------------
# Arbitration ID helpers
# -----------------------------
def _id_heartbeat(node_id: int) -> int:
    return (node_id << 5) | 0x01

def _id_set_axis_state(node_id: int) -> int:
    return (node_id << 5) | 0x07

def _id_get_encoder_estimates(node_id: int) -> int:
    return (node_id << 5) | 0x09

def _id_set_input_pos(node_id: int) -> int:
    return (node_id << 5) | 0x0C

def _id_clear_errors(node_id: int) -> int:
    # Simple command: Clear Errors
    return (node_id << 5) | 0x18

def _id_od_write(node_id: int) -> int:
    # Object Dictionary Write (SDO-style)
    return (node_id << 5) | 0x17


# Set up Bus object --------------------------------------
bus = can.interface.Bus("can0", interface="socketcan")
    # can0 is the default for first CAN adapter, and you set up the Pi to automatically set up can0 on startup
    # SocketCan is the backend native to linux kernal
    
# Define global functions --------------------------------
def send_pos(pos_target, velocity_ff, node_id):		# Function to send position commands to ODrive
    vel_ff = velocity_ff		# Velocity feedforward: This will get implemented into an if loop and changed depending on the vel_control boolean value
    torque_ff = 0	# Torque feedforward: We're not using this right now
    data = struct.pack('<fhh', pos_target, vel_ff, torque_ff)
        # '<fhh' sets up the 8 byte string (what ODrive expects)
            # <: Little-endian byte order (LSB first)
            # f: Float (32-bit), 4 bytes, position in revolutions
            # h: Signed short (integer), 2 bytes, vel_ff in mrev/s
            # h: Signed short (integer), 2 bytes, torque_ff in mNm
    arb_id = (node_id << 5) | 0x0C	# Constructs the arbitration ID used for ODrive
        # node_id << 5: Shifts the ID left 5 bits
        # | 0x0C: Sets the message type to "Set Input Position" (0x0C)
        # Follows ODrive CAN protocol
    msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)	# Assembles CAN message
        # Uses arb_id and data from above
        # is_extended_id=False: Standard 11-bit CAN ID
    bus.send(msg)	# Sends message over CAN using the bus object you created earlier
        
def request_pos(node_id):
    init_time = time.time()				# Time when request loop is initiated	
    latest_pos = None					# Default value in case no feedback is received before timeout
    unpack_time = None
    return_time = None					
    arb_id = (node_id << 5) | 0x09		# Same as before, see above for more detail
    msg = can.Message(arbitration_id=arb_id, data=[], is_extended_id=False, is_remote_frame=True)
        # Creates Remote Transmission Request (RTR) frame (reason for last bit in ())
            # Special CAN fram that asks ODrive to respond with data, without including any payload
    bus.send(msg)						# Send request message
    send_time = time.time()					# Record the time when request message is sent
    while time.time() - send_time < .010:	# 10ms window to loop until a CAN message is received
        msg = bus.recv(timeout = .008)	# 8ms window to receive a single message
            # If no message is received within 8ms, bus.recv returns 'None'
        if not msg:						# If msg is returned as 'None', 
            break						# Break out of the entire while loop, no message received
        if msg.arbitration_id == arb_id and not msg.is_remote_frame:	# If message has correct ID and isn't requesting feedback
            response_time = time.time() - send_time	# Time between sending request and obtaining response
            latest_pos, _ = struct.unpack('<ff',msg.data)
                # '<ff' returns 2x 32-bit floats from the 8-byte payload (position, velocity)
                # Stores position as latest_pos
                #' _ ' ignores the velocity feedback
                    # Can assign another value here if you want to use velocity feedback
            unpack_time = time.time() - init_time	# Time from initiating response loop to unpacking feedback
            print(f"Send-to-receive: {response_time: .4f} seconds")	# Print time delay to shell
            print(f"Pack-to-unpack:  {unpack_time: .4f} seconds")	# Print time delay to shell

            break	# Break the while loop after receiving feedback
    return latest_pos#, unpack_time, response_time	# Function call will return the feedback position        

def enter_closed_loop(node_id):
    arb_id_set_state = (node_id << 5) | 0x07
        # 0x07 sets message type to 'Set Axis State'
    msg = can.Message(arbitration_id=arb_id_set_state, data=struct.pack('<I', 8), is_extended_id=False)
    bus.send(msg)
    while True:
        msg = bus.recv(timeout=1.0)
        if msg and msg.arbitration_id == ((node_id << 5) | 0x01):
            _, axis_state, _, _ = struct.unpack('<IBBB', msg.data[:7])
            if axis_state == 8:
                break
            
def exit_closed_loop(node_id):
    arb_id_set_state = (node_id << 5) | 0x07  # 0x07 = Set Axis State
    msg = can.Message(arbitration_id=arb_id_set_state, data=struct.pack('<I', 1), is_extended_id=False)
    bus.send(msg)
    print(f"Sent Set_Axis_State(IDLE) to node {node_id}")
         
def clear_errors(node_id):
    arb_id_clear = (node_id << 5) | 0x018
    msg = can.Message(arbitration_id=arb_id_clear, data=[], is_extended_id=False)
    bus.send(msg)
    print(f"Cleared Error")
    enter_closed_loop(node_id)
    
def write_float_parameter(node_id, index, value, subindex=0x00):
    arb_id = (node_id << 5) | 0x17  # 0x17 = OD Write

    # SDO Write for 32-bit float = 0x2B
    # Format: command, index (2 bytes), subindex, value (4 bytes)
    data = struct.pack('<BHBBf', 0x2B, index, subindex, 0x00, value)

    msg = can.Message(arbitration_id=arb_id, data=data[:8], is_extended_id=False)
    bus.send(msg)
    print(f"Wrote {value} to index {index:#06x}")


# =============================
# Motion commands & feedback
# =============================
def set_input_pos(bus: can.BusABC, node_id: int, position_turns: float, vel_ff: int = 0, torque_ff: int = 0,) -> None:
    """
    Set Input Position (motor turns). Payload format preserved from your code:
        <f32 pos_turns><i16 vel_ff><i16 torque_ff>
    Units for feed-forward must match your ODrive config.
    """
    data = struct.pack("<fhh", position_turns, vel_ff, torque_ff)
    msg = can.Message(arbitration_id=_id_set_input_pos(node_id), data=data, is_extended_id=False,)
    bus.send(msg)

def request_encoder_estimates(bus: can.BusABC,node_id: int,rx_timeout_s: float = 0.010,) -> Optional[Tuple[float, float]]:
    """
    Request encoder estimates via RTR frame. Returns (pos_turns, vel_turns_s) or None on timeout.
    """
    req_id = _id_get_encoder_estimates(node_id)
    rtr = can.Message(
        arbitration_id=req_id,
        data=[],
        is_extended_id=False,
        is_remote_frame=True,
    )
    bus.send(rtr)

    start = time.time()
    while (time.time() - start) < rx_timeout_s:
        msg = bus.recv(timeout=rx_timeout_s)
        if not msg:
            break
        if msg.arbitration_id == req_id and not msg.is_remote_frame and len(msg.data) >= 8:
            pos, vel = struct.unpack("<ff", msg.data[:8])
            return pos, vel
    return None

# Convenience wrapper matching your old `request_pos`
def request_position_only(
    bus: can.BusABC,
    node_id: int,
    rx_timeout_s: float = 0.010,
) -> Optional[float]:
    est = request_encoder_estimates(bus, node_id, rx_timeout_s=rx_timeout_s)
    return est[0] if est else None


# =============================
# Maintenance helpers
# =============================
def clear_errors(bus: can.BusABC, node_id: int) -> None:
    """Send Clear Errors simple command and (optionally) re-enter closed loop yourself afterwards."""
    msg = can.Message(
        arbitration_id=_id_clear_errors(node_id),
        data=[],
        is_extended_id=False,
    )
    bus.send(msg)

def write_float_parameter(
    bus: can.BusABC,
    node_id: int,
    index: int,
    value: float,
    subindex: int = 0x00,
) -> None:
    """
    Write a float parameter to the Object Dictionary (advanced).
    Payload here matches your previous implementation.
    """
    # SDO Write for 32-bit float = 0x2B (per your note)
    data = struct.pack("<BHBBf", 0x2B, index, subindex, 0x00, value)
    msg = can.Message(
        arbitration_id=_id_od_write(node_id),
        data=data[:8],
        is_extended_id=False,
    )
    bus.send(msg)


class Controls(0):

    def activate_state(self):
        enter_closed_loop(node_id_act)	# Enter closed loop if activated

    def deactivate_state(self):
        exit_closed_loop(node_id_act)	# Exit closed loop if deactivated
            
    def activate_vel_ctrl_state(self):
        enter_closed_loop(node_id_act)	# Enter closed loop if activated

    def deactivate_vel_ctrl_state(self)
        exit_closed_loop(node_id_act)	# Exit closed loop if deactivated
            
    def increase_pos(self):
        Current_pos = request_pos(node_id_act)
        New_pos = Current_pos + 1
        velocity_ff = 0
        send_pos(New_pos, velocity_ff, node_id_act) #need to figure out how we want to set velocity control
        
    def decrease_pos(self):
        Current_pos = request_pos(node_id_act)
        New_pos = Current_pos - 1
        velocity_ff = 0
        send_pos(New_pos, velocity_ff, node_id_act) #need to figure out how we want to set velocity control
        
    def handle_clear_error(self):
        clear_errors(node_id_act)
        
    def handle_abort(self):
        exit_closed_loop(node_id_act)

    def close_app(self):
        exit_closed_loop(node_id_act)
        
    def update_status(self):
        Current_pos = request_pos(node_id_act)
        return Current_pos
        

# =============================
# Optional: tiny demo when run directly
# =============================
if __name__ == "__main__":
   
    NODE = 1
    bus = make_bus("can0", "socketcan")

    print("Entering closed loop...")
    ok = enter_closed_loop(bus, NODE, wait=True, timeout_s=3.0)
    print("Closed loop:", ok)

    pos0 = request_position_only(bus, NODE)
    print("Start pos:", pos0)

    print("Commanding +1.0 turn...")
    set_input_pos(bus, NODE, (pos0 or 0.0) + 1.0, vel_ff=0, torque_ff=0)
    time.sleep(0.2)

    pos1 = request_position_only(bus, NODE)
    print("After move pos:", pos1)

    print("Exiting closed loop...")
    exit_closed_loop(bus, NODE, wait=False)
    print("Done.")