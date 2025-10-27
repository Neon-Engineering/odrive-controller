import can
import struct
import time
import math

# === CONFIGURATION ===
node_id = 0  # ODrive CAN Node ID
bus = can.interface.Bus("can0", interface="socketcan")

# === Helper: Send Set_Input_Pos command ===
def send_position_command(position, velocity):
    vel_ff = int(velocity * 1000)  # convert float to int16
    torque_ff = 0
    data = struct.pack('<fhh', position, vel_ff, torque_ff)
    arb_id = (node_id << 5) | 0x0C
    msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
    bus.send(msg)

# === Set Axis State to CLOSED_LOOP_CONTROL ===
print("Requesting CLOSED_LOOP_CONTROL...")
arb_id_set_state = (node_id << 5) | 0x07
msg = can.Message(arbitration_id=arb_id_set_state, data=struct.pack('<I', 8), is_extended_id=False)
bus.send(msg)

# === Wait for Axis State Confirmation ===
while True:
    msg = bus.recv(timeout=1.0)
    if msg and msg.arbitration_id == ((node_id << 5) | 0x01):  # Heartbeat
        _, axis_state, _, _ = struct.unpack('<IBBB', msg.data[:7])
        if axis_state == 8:
            print("Axis is in CLOSED_LOOP_CONTROL.")
            break

# === Sine Sweep Settings ===
amplitude = 3.0   # revolutions
frequency = 0.5   # Hz
duration = 10.0   # seconds
start_time = time.time()

print("Starting sine sweep...")

# === Sine Sweep Loop ===
while (time.time() - start_time) < duration:
    t = time.time() - start_time
    position = amplitude * math.sin(2 * math.pi * frequency * t)
    velocity = 2 * math.pi * frequency * amplitude * math.cos(2 * math.pi * frequency * t)

    send_position_command(position, velocity)

    # Optional: Print feedback if encoder estimates are configured to stream
    msg = bus.recv(timeout=0.05)
    if msg and msg.arbitration_id == ((node_id << 5) | 0x09):  # Get_Encoder_Estimates
        pos, vel = struct.unpack('<ff', msg.data)
        print(f"t={t:.2f}s | Pos={pos:.3f} rev | Vel={vel:.3f} rev/s")

    time.sleep(0.05)  # 20 Hz loop

print("Sine sweep completed.")

