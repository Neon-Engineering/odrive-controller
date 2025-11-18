import os
async def discover_and_assign_node_id(can_interface, can_channel, can_bitrate, preferred_node_id=1, timeout=2.0):
    """
    Scan the CAN bus for ODrives and assign a node ID if needed.
    Returns the discovered or assigned node ID, or None if not found.
    """
    import can
    import time
    print(f"🔍 Scanning CAN bus for ODrives (interface={can_interface}, channel={can_channel}, bitrate={can_bitrate})...")
    bus = can.Bus(interface=can_interface, channel=can_channel, bitrate=can_bitrate, index=0)
    found_ids = set()
    start_time = time.time()
    # Listen for any ODrive heartbeat or status messages
    while time.time() - start_time < timeout:
        msg = bus.recv(timeout=0.1)
        if msg and (msg.arbitration_id >> 5) not in found_ids:
            node_id = (msg.arbitration_id >> 5) & 0x3F
            if node_id != 0:
                found_ids.add(node_id)
                print(f"  - Found ODrive with node ID: {node_id}")
    if found_ids:
        print(f"✅ Discovered ODrive node IDs: {sorted(found_ids)}")
        bus.shutdown()
        return min(found_ids)  # Use the lowest found node ID
    # If no ODrive found, try to assign node ID to a device in unassigned state (node_id=0)
    print("No ODrive node IDs found. Trying to assign node ID to unassigned device...")
    # ODrive CAN protocol: send assignment to node_id=0 (broadcast)
    ASSIGN_NODE_ID_CMD = 0x1B
    msg = can.Message(
        arbitration_id=(0 << 5) | ASSIGN_NODE_ID_CMD,
        data=[preferred_node_id],
        is_extended_id=False
    )
    bus.send(msg)
    print(f"Sent node ID assignment command to node 0, assigning node ID {preferred_node_id}.")
    # Wait for device to come up with new node ID
    time.sleep(0.5)
    start_time = time.time()
    while time.time() - start_time < timeout:
        msg = bus.recv(timeout=0.1)
        if msg:
            node_id = (msg.arbitration_id >> 5) & 0x3F
            if node_id == preferred_node_id:
                print(f"✅ ODrive accepted node ID {preferred_node_id}!")
                bus.shutdown()
                return preferred_node_id
    print("❌ No ODrive responded after node ID assignment.")
    bus.shutdown()
    return None
# Software version tracking
SOFTWARE_VERSION = "1.1.0"
SOFTWARE_NAME = "ODrive High-Performance CAN Control System"



import argparse
import asyncio
import sys
import time
import threading
from pathlib import Path
from datetime import datetime
import json
import can

# Add modules directory to path
sys.path.append(str(Path(__file__).parent / "modules"))


# Import fixed modular components
from modules.simple_can_manager import create_simple_can_manager, SimpleCANManager
from modules.trajectory_player import create_trajectory_player

# Import existing utilities
from utils.can_simple_utils import CanSimpleNode
import libusb_backend_workaround

# Initialize libusb backend
libusb_backend_workaround.find_libusb_backend()

def load_can_settings(config_path="can_settings.json"):
    """Load CAN settings from a JSON config file, or provide defaults."""
    default = {
        "can_interface": "gs_usb",
        "can_channel": "can0",
        "can_bitrate": 1000000,
        "can_node_id": 1
    }
    try:
        with open(config_path, "r") as f:
            user = json.load(f)
        for k in default:
            if k not in user:
                user[k] = default[k]
        return user
    except Exception:
        return default


class SimpleTelemetryManager:
    """
    Aggressive 100Hz Telemetry Manager - Guaranteed Position Updates
    
    AGGRESSIVE polling strategy to prevent dropped encoder messages:
    - Encoder: 100Hz polling (10ms guaranteed) - Critical position/velocity feedback
    - Iq Current: 100Hz polling (offset 5ms) - High-resolution current monitoring
    - Torque: 50Hz polling - Torque setpoint/estimate feedback  
    - Bus Voltage: 25Hz polling - Power system monitoring
    - Temperature: 7Hz polling - Thermal management
    
    Staggered timing prevents CAN bus conflicts while ensuring reliable data.
    Processing at 500Hz for maximum responsiveness to incoming messages.
    """
    
    def __init__(self, can_manager):
        self.can_manager = can_manager
        # Load torque constant from config
        self.torque_constant = 0.0827  # Default from config.json
        try:
            with open('config.json', 'r') as f:
                config = json.load(f)
                self.torque_constant = config.get('axis0.config.motor.torque_constant', 0.0827)
                print(f"📊 Loaded torque constant: {self.torque_constant:.4f} Nm/A")
        except Exception as e:
            print(f"⚠️ Could not load torque constant from config: {e}, using default: {self.torque_constant}")
        self.latest_data = {
            'position': None, 'velocity': None, 'motor_temp': None,
            'fet_temp': None, 'timestamp': time.time(),
            'bus_voltage': None, 'bus_current': None, 'motor_current': None,
            'iq_setpoint': None, 'iq_measured': None, 'torque_setpoint': None, 'torque_estimate': None
        }
        self.running = False
        self.thread = None
        # Encoder update tracking for drop detection
        self.last_encoder_update = 0.0
        self.encoder_drop_warnings = 0
    
    def start(self):
        """Start telemetry collection"""
        print("📡 Starting Simple Telemetry Manager...")
        self.running = True
        self.thread = threading.Thread(target=self._telemetry_thread, daemon=True)
        self.thread.start()
    
    def _telemetry_thread(self):
        """Aggressive 100Hz encoder polling with hybrid optimization"""
        last_encoder_request = 0.0
        last_iq_request = 0.0
        last_other_request = 0.0
        
        while self.running:
            current_time = time.time()
            
            # Process incoming messages at high frequency (200Hz processing)
            message = self.can_manager.get_latest_telemetry()
            if message:
                self._process_message(message)
            
            # AGGRESSIVE 100Hz encoder polling - most critical for position control
            if current_time - last_encoder_request >= 0.01:  # 100Hz encoder
                self.can_manager.request_encoder_data()
                last_encoder_request = current_time
            
            # HIGH-FREQUENCY 100Hz Iq polling - critical for current monitoring  
            if current_time - last_iq_request >= 0.01:  # 100Hz Iq
                # Offset by 5ms to avoid conflicts with encoder requests
                if (current_time - last_encoder_request) >= 0.005:
                    self.can_manager.request_motor_current_data()
                    last_iq_request = current_time
            
            # Lower frequency data (staggered to avoid bus congestion)
            if current_time - last_other_request >= 0.02:  # 50Hz cycle for other data
                cycle_count = int(current_time * 50) % 10
                
                if cycle_count == 0:    # Torque data
                    self.can_manager.request_torque_data()
                elif cycle_count == 3:  # Bus voltage/current
                    self.can_manager.request_voltage_current_data()
                elif cycle_count == 7:  # Temperature (lowest priority)
                    self.can_manager.request_temperature_data()
                
                last_other_request = current_time
            
            time.sleep(0.002)  # 500Hz processing for maximum responsiveness
    
    def _process_message(self, message):
        """Process telemetry message with encoder drop detection"""
        import struct
        current_time = time.time()
        cmd_id = message.arbitration_id & 0x1F
        
        if cmd_id == 0x09 and len(message.data) >= 8:
            pos, vel = struct.unpack('<ff', message.data[:8])
            self.latest_data['position'] = pos
            self.latest_data['velocity'] = vel
            self.latest_data['timestamp'] = message.timestamp
            
            # Track encoder update timing
            if self.last_encoder_update > 0:
                update_gap = current_time - self.last_encoder_update
                if update_gap > 0.025:  # More than 25ms gap (should be ~10ms)
                    self.encoder_drop_warnings += 1
                    if self.encoder_drop_warnings <= 5:  # Limit warning spam
                        print(f"⚠️ Encoder update gap: {update_gap*1000:.1f}ms (expected ~10ms)")
            
            self.last_encoder_update = current_time
        elif cmd_id == 0x15 and len(message.data) >= 8:
            fet_temp, motor_temp = struct.unpack('<ff', message.data[:8])
            self.latest_data['fet_temp'] = fet_temp
            self.latest_data['motor_temp'] = motor_temp
        elif cmd_id == 0x17 and len(message.data) >= 8:
            bus_voltage, bus_current = struct.unpack('<ff', message.data[:8])
            self.latest_data['bus_voltage'] = bus_voltage
            self.latest_data['bus_current'] = bus_current
        elif cmd_id == 0x14 and len(message.data) >= 8:
            # 0x14 returns both Iq setpoint and Iq measured (8 bytes total)
            iq_setpoint, iq_measured = struct.unpack('<ff', message.data[:8])
            self.latest_data['motor_current'] = iq_measured  # Keep backward compatibility
            self.latest_data['iq_setpoint'] = iq_setpoint
            self.latest_data['iq_measured'] = iq_measured
        elif cmd_id == 0x14 and len(message.data) >= 4:
            # Fallback for 4-byte Iq data (just Iq measured)
            iq_measured = struct.unpack('<f', message.data[:4])[0]
            self.latest_data['motor_current'] = iq_measured
            self.latest_data['iq_measured'] = iq_measured
            self.latest_data['iq_setpoint'] = None
        elif cmd_id == 0x1C and len(message.data) >= 8:
            # 0x1C GET_TORQUES: returns torque_setpoint and torque_estimate (8 bytes total)
            torque_setpoint, torque_estimate = struct.unpack('<ff', message.data[:8])
            self.latest_data['torque_setpoint'] = torque_setpoint
            self.latest_data['torque_estimate'] = torque_estimate
    
    def get_position_velocity(self):
        """Get current position and velocity"""
        return self.latest_data['position'], self.latest_data['velocity']
    
    def get_latest_data(self):
        """Get all latest data"""
        return self.latest_data.copy()
    
    def stop(self):
        """Stop telemetry collection"""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)

class HighSpeedLogger:
    """High-speed CSV logger with queue-based non-blocking writes"""
    
    def __init__(self, filename=None):
        import queue
        import csv
        import os
        from datetime import datetime
        
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"odrive_log_{timestamp}.csv"
        
        self.filename = filename
        self.data_queue = queue.Queue(maxsize=10000)  # Large buffer
        self.running = False
        self.thread = None
        self.stats = {'writes': 0, 'drops': 0}
        self.start_time = None  # Will be set when logging starts
        
        # CSV headers - using relative time from start
        self.headers = [
            'time_rel_s', 'system_time', 'position', 'velocity', 
            'fet_temp', 'motor_temp', 'bus_voltage', 'bus_current', 'motor_current',
            'iq_setpoint', 'iq_measured', 'torque_setpoint', 'torque_estimate',
            'target_position', 'trajectory_active'
        ]
    
    @staticmethod
    def create_filename_with_datetime(base_filename):
        """Create filename with datetime suffix, supporting subdirectories"""
        import os
        from datetime import datetime
        
        # Handle subdirectories
        dirname = os.path.dirname(base_filename)
        basename = os.path.basename(base_filename)
        
        # Split filename and extension
        name, ext = os.path.splitext(basename)
        if not ext:
            ext = '.csv'
        
        # Add datetime suffix
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        new_basename = f"{name}_{timestamp}{ext}"
        
        # Combine with directory
        if dirname:
            os.makedirs(dirname, exist_ok=True)  # Create directory if needed
            return os.path.join(dirname, new_basename)
        else:
            return new_basename
    
    def start(self):
        """Start logging thread"""
        import time
        print(f"📝 Starting High-Speed Logger -> {self.filename}")
        self.start_time = time.time()  # Record when logging starts
        self.running = True
        self.thread = threading.Thread(target=self._logging_thread, daemon=True)
        self.thread.start()
    
    def log_data(self, telemetry_data, controller_stats=None):
        """Log data point (non-blocking)"""
        import time
        
        current_time = time.time()
        
        # Calculate relative time from start (in seconds)
        relative_time = current_time - self.start_time if self.start_time else 0.0
        
        # Prepare data row with relative timestamps
        row = {
            'time_rel_s': round(relative_time, 4),  # Relative time from start in seconds
            'system_time': current_time,  # Absolute system time for reference
            'position': telemetry_data.get('position'),
            'velocity': telemetry_data.get('velocity'),
            'fet_temp': telemetry_data.get('fet_temp'),
            'motor_temp': telemetry_data.get('motor_temp'),
            'bus_voltage': telemetry_data.get('bus_voltage'),
            'bus_current': telemetry_data.get('bus_current'),
            'motor_current': telemetry_data.get('motor_current'),
            'iq_setpoint': telemetry_data.get('iq_setpoint'),
            'iq_measured': telemetry_data.get('iq_measured'),
            'torque_setpoint': telemetry_data.get('torque_setpoint'),
            'torque_estimate': telemetry_data.get('torque_estimate'),
            'target_position': controller_stats.get('current_target') if controller_stats else None,
            'trajectory_active': controller_stats.get('trajectory_active') if controller_stats else False
        }
        
        # Non-blocking queue put
        try:
            self.data_queue.put_nowait(row)
        except:
            self.stats['drops'] += 1  # Count dropped messages
    
    def _logging_thread(self):
        """High-speed CSV writing thread"""
        import csv
        import time
        
        try:
            with open(self.filename, 'w', newline='') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=self.headers)
                writer.writeheader()
                
                while self.running or not self.data_queue.empty():
                    try:
                        # Get data with timeout
                        row = self.data_queue.get(timeout=0.1)
                        writer.writerow(row)
                        csvfile.flush()  # Ensure data is written
                        self.stats['writes'] += 1
                    except:
                        continue  # Timeout or empty queue
                        
        except Exception as e:
            print(f"❌ Logging error: {e}")
    
    def get_statistics(self):
        """Get logging statistics"""
        return {
            'filename': self.filename,
            'writes': self.stats['writes'],
            'drops': self.stats['drops'],
            'queue_size': self.data_queue.qsize() if hasattr(self.data_queue, 'qsize') else 0
        }
    
    def stop(self):
        """Stop logging and flush remaining data"""
        print(f"💾 Stopping logger, flushing {self.data_queue.qsize() if hasattr(self.data_queue, 'qsize') else '?'} remaining records...")
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        
        stats = self.get_statistics()
        print(f"📊 Log complete: {stats['writes']} records, {stats['drops']} drops -> {self.filename}")

class SimplePositionController:
    """Advanced position controller with trajectory planning and high-speed operation"""
    
    def __init__(self, can_manager, telemetry_manager):
        self.can_manager = can_manager
        self.telemetry_manager = telemetry_manager
        self.active = False
        self.current_target = 0.0
        self.position_limits = (-1000.0, 1000.0)  # Effectively unlimited range
        self.stats = {'commands_sent': 0, 'last_position_error': 0.0}
        
        # Advanced control features
        self.trajectory_active = False
        self.trajectory_thread = None
        self.trajectory_stop_event = threading.Event()
        self.max_velocity = 10.0  # turns/sec
        self.max_acceleration = 10.0  # turns/sec²
    
    def activate(self):
        """Activate position control"""
        self.active = True
        print("✅ Position control activated")
    
    def deactivate(self):
        """Deactivate position control"""
        self.active = False
        print("🛑 Position control deactivated")
    
    def set_position(self, position: float) -> bool:
        """Set position target"""
        clamped_position = max(self.position_limits[0], min(self.position_limits[1], position))
        if clamped_position != position:
            print(f"⚠️ Position clamped: {position:.3f} -> {clamped_position:.3f}")
        
        # Debug: Show what we're sending
        print(f"📡 Sending position command: {clamped_position:.3f} turns (CAN ID: 0x{(self.can_manager.node_id << 5 | 0x0C):02X})")
        
        success = self.can_manager.send_position_command(clamped_position)
        if success:
            self.current_target = clamped_position
            self.stats['commands_sent'] += 1
            print(f"✅ Position command queued successfully")
        else:
            print(f"❌ Failed to queue position command")
            current_pos, _ = self.telemetry_manager.get_position_velocity()
            if current_pos is not None:
                self.stats['last_position_error'] = clamped_position - current_pos
            return True
        return False
    
    def set_velocity(self, velocity: float) -> bool:
        """Set velocity target (for testing - like working can_simple.py)"""
        clamped_velocity = max(-self.max_velocity, min(self.max_velocity, velocity))
        if clamped_velocity != velocity:
            print(f"⚠️ Velocity clamped: {velocity:.3f} -> {clamped_velocity:.3f}")
        
        # Debug: Show what we're sending
        print(f"📡 Sending velocity command: {clamped_velocity:.3f} turns/s (CAN ID: 0x{(self.can_manager.node_id << 5 | 0x0D):02X})")
        
        success = self.can_manager.send_velocity_command(clamped_velocity)
        if success:
            self.stats['commands_sent'] += 1
            print(f"✅ Velocity command queued successfully")
            return True
        else:
            print(f"❌ Failed to queue velocity command")
            return False
    
    def get_statistics(self):
        """Get controller statistics"""
        return {
            'active': self.active, 'current_target': self.current_target,
            'commands_sent': self.stats['commands_sent'],
            'position_error': self.stats['last_position_error'],
            'trajectory_active': self.trajectory_active
        }
    
    def move_to_position(self, target_position: float, velocity: float = None) -> bool:
        """Move to target position with trajectory planning"""
        if not self.active:
            print("❌ Position control not active")
            return False
            
        velocity = velocity or self.max_velocity
        velocity = min(velocity, self.max_velocity)
        
        target_position = max(self.position_limits[0], min(self.position_limits[1], target_position))
        
        # Stop any existing trajectory
        self._stop_trajectory()
        
        # Start trajectory thread
        self.trajectory_stop_event.clear()
        self.trajectory_thread = threading.Thread(
            target=self._trajectory_thread,
            args=(target_position, velocity),
            daemon=True
        )
        self.trajectory_active = True
        self.trajectory_thread.start()
        print(f"🎯 Moving to {target_position:.3f} turns at {velocity:.2f} turns/sec")
        return True
    
    def relative_move(self, delta_position: float, velocity: float = None) -> bool:
        """Move relative to current position"""
        current_pos, _ = self.telemetry_manager.get_position_velocity()
        if current_pos is None:
            print("❌ Cannot get current position for relative move")
            return False
        
        target = current_pos + delta_position
        return self.move_to_position(target, velocity)
    
    def oscillate(self, amplitude: float, frequency: float, duration: float = None, phase_deg: float = 0.0) -> bool:
        """Oscillate around current position with optional phase shift
        
        Args:
            amplitude: Oscillation amplitude in turns
            frequency: Oscillation frequency in Hz
            duration: Optional duration in seconds (None for continuous)
            phase_deg: Phase angle in degrees (0=sine, 90=cosine, etc.)
        """
        if not self.active:
            print("❌ Position control not active")
            return False
            
        current_pos, _ = self.telemetry_manager.get_position_velocity()
        if current_pos is None:
            print("❌ Cannot get current position for oscillation")
            return False
        
        # Check limits
        center = current_pos
        if (center - amplitude < self.position_limits[0] or 
            center + amplitude > self.position_limits[1]):
            print(f"❌ Oscillation would exceed limits: {self.position_limits}")
            return False
        
        # Stop any existing trajectory
        self._stop_trajectory()
        
        # Start oscillation thread
        self.trajectory_stop_event.clear()
        self.trajectory_thread = threading.Thread(
            target=self._oscillation_thread,
            args=(center, amplitude, frequency, duration, phase_deg),
            daemon=True
        )
        self.trajectory_active = True
        self.trajectory_thread.start()
        
        duration_str = f" for {duration:.1f}s" if duration else " (continuous)"
        phase_str = f", phase {phase_deg:.0f}°" if phase_deg != 0 else ""
        print(f"🌊 Oscillating at {frequency:.2f} Hz, amplitude {amplitude:.3f}{phase_str}{duration_str}")
        return True
    
    def stop_trajectory(self):
        """Stop any active trajectory and allow motor to settle"""
        if self.trajectory_active:
            print("🛑 Stopping trajectory - motor will settle for ~1 second...")
            self._stop_trajectory()
        else:
            print("ℹ️ No active trajectory to stop")
    
    def _stop_trajectory(self):
        """Internal method to stop trajectory and wait for motor settling"""
        if self.trajectory_active:
            self.trajectory_stop_event.set()
            if self.trajectory_thread and self.trajectory_thread.is_alive():
                # Wait longer to allow for the 1-second settling period
                self.trajectory_thread.join(timeout=2.0)
            self.trajectory_active = False
    
    def _trajectory_thread(self, target_position: float, velocity: float):
        """Thread for smooth trajectory execution"""
        import math
        
        current_pos, _ = self.telemetry_manager.get_position_velocity()
        if current_pos is None:
            print("❌ Cannot start trajectory - no position data")
            self.trajectory_active = False
            return
        
        start_position = current_pos
        distance = target_position - start_position
        
        if abs(distance) < 0.001:  # Already at target
            self.trajectory_active = False
            return
        
        # Calculate trajectory timing
        accel_time = velocity / self.max_acceleration
        accel_distance = 0.5 * self.max_acceleration * accel_time ** 2
        
        if abs(distance) <= 2 * accel_distance:
            # Triangular profile (no constant velocity phase)
            peak_velocity = math.sqrt(abs(distance) * self.max_acceleration)
            accel_time = peak_velocity / self.max_acceleration
            total_time = 2 * accel_time
        else:
            # Trapezoidal profile
            const_distance = abs(distance) - 2 * accel_distance
            const_time = const_distance / velocity
            total_time = 2 * accel_time + const_time
        
        start_time = time.time()
        
        while not self.trajectory_stop_event.is_set():
            elapsed = time.time() - start_time
            
            if elapsed >= total_time:
                # Trajectory complete
                self.set_position(target_position)
                break
            
            # Calculate position along trajectory
            if elapsed <= accel_time:
                # Acceleration phase
                s = 0.5 * self.max_acceleration * elapsed ** 2
            elif elapsed <= total_time - accel_time:
                # Constant velocity phase
                s = (accel_distance + velocity * (elapsed - accel_time))
            else:
                # Deceleration phase
                decel_time = elapsed - (total_time - accel_time)
                s = abs(distance) - 0.5 * self.max_acceleration * (accel_time - decel_time) ** 2
            
            # Apply direction
            position = start_position + (s if distance > 0 else -s)
            self.set_position(position)
            
            time.sleep(0.01)  # 100Hz trajectory update
        
        # Position hold phase - maintain final position for motor settling
        final_position = target_position
        if self.trajectory_stop_event.is_set():
            # If stopped early, hold current position
            current_pos, _ = self.telemetry_manager.get_position_velocity()
            if current_pos is not None:
                final_position = current_pos
            print("🛑 Trajectory stopped - holding position for motor settling...")
        else:
            print("✅ Trajectory complete - holding final position for motor settling...")
            
        hold_start = time.time()
        hold_duration = 1.0  # 1 second hold
        
        while time.time() - hold_start < hold_duration:
            # Continue commanding the final position to keep motor stable
            self.set_position(final_position)
            time.sleep(0.01)  # Continue 100Hz position commands during hold
            
            # Allow immediate exit if stop is called again during hold
            if self.trajectory_stop_event.is_set():
                break
        
        print("✅ Motor settled - trajectory complete")
        self.trajectory_active = False
    
    def _oscillation_thread(self, center: float, amplitude: float, frequency: float, duration: float = None, phase_deg: float = 0.0):
        """Thread for oscillation execution with position hold on stop
        
        Args:
            center: Center position in turns (will be adjusted for smooth start)
            amplitude: Oscillation amplitude in turns
            frequency: Oscillation frequency in Hz
            duration: Optional duration in seconds
            phase_deg: Phase angle in degrees (added to angle calculation)
        """
        import math
        
        # Convert phase from degrees to radians
        phase_rad = math.radians(phase_deg)
        
        # Calculate the starting position based on phase angle
        # At t=0, position = center + amplitude * sin(phase_rad)
        starting_offset = amplitude * math.sin(phase_rad)
        
        # Get current motor position
        current_pos, _ = self.telemetry_manager.get_position_velocity()
        
        # Adjust center so that the oscillation starts from the current position
        # This prevents step changes when phase is non-zero
        # If current_pos should equal center + starting_offset, then:
        # center = current_pos - starting_offset
        adjusted_center = current_pos - starting_offset
        
        print(f"🎯 Oscillation adjusted: original center={center:.3f}, adjusted center={adjusted_center:.3f}, starting offset={starting_offset:.3f}")
        
        start_time = time.time()
        last_position = adjusted_center
        
        while not self.trajectory_stop_event.is_set():
            elapsed = time.time() - start_time
            
            if duration and elapsed >= duration:
                break
            
            # Calculate oscillation position with phase shift and adjusted center
            angle = 2 * math.pi * frequency * elapsed + phase_rad
            position = adjusted_center + amplitude * math.sin(angle)
            last_position = position
            
            self.set_position(position)
            time.sleep(0.01)  # 100Hz oscillation update
        
        # Position hold phase - maintain last position for ~1 second to allow motor to settle
        print("🛑 Oscillation stopped - holding position for motor settling...")
        hold_start = time.time()
        hold_duration = 1.0  # 1 second hold
        
        while time.time() - hold_start < hold_duration:
            # Continue commanding the last position to keep motor stable
            self.set_position(last_position)
            time.sleep(0.01)  # Continue 100Hz position commands during hold
            
            # Allow immediate exit if stop is called again
            if self.trajectory_stop_event.is_set():
                break
        
        print("✅ Motor settled - oscillation complete")
        self.trajectory_active = False
    
    def deactivate(self):
        """Deactivate position control and stop trajectories"""
        self._stop_trajectory()
        self.active = False
        print("🛑 Position control deactivated")

class HighPerformanceODriveSystem:
    """
    High-Performance ODrive Control System (Fixed Threading)
    
    Main orchestrator with proper threading and asyncio integration.
    """
    
    def __init__(self, node_id: int = 0):
        self.node_id = node_id
        
        # Core subsystems
        self.can_manager = None
        self.telemetry_manager = None
        self.position_controller = None
        self.logger = None
        self.trajectory_player = None
        
        # System state
        self.initialized = False
        self.running = False
        self.armed = False
        self.logging_active = False
        self.desired_control_mode = "position"  # Default to position mode
    
    async def initialize(self) -> bool:
        """Initialize all subsystems"""
        print(f"🚀 Initializing {SOFTWARE_NAME} v{SOFTWARE_VERSION}")
        print("="*60)
        
        try:
            # 1. Initialize CAN Manager
            print("1️⃣ Initializing CAN communication...")
            self.can_manager = create_simple_can_manager(node_id=self.node_id)
            if not await self.can_manager.initialize():
                return False
            
            # 2. Initialize Telemetry Manager
            print("2️⃣ Initializing telemetry...")
            self.telemetry_manager = SimpleTelemetryManager(self.can_manager)
            self.telemetry_manager.start()
            
            # 3. Initialize Position Controller
            print("3️⃣ Initializing position controller...")
            self.position_controller = SimplePositionController(
                self.can_manager, self.telemetry_manager
            )
            
            # 4. Initialize High-Speed Logger
            print("4️⃣ Initializing high-speed logger...")
            self.logger = HighSpeedLogger()
            
            # 5. Initialize Trajectory Player
            print("5️⃣ Initializing trajectory player...")
            self.trajectory_player = create_trajectory_player(
                self.position_controller, self.telemetry_manager
            )
            
            # Wait for telemetry to start
            await asyncio.sleep(0.5)
            
            self.initialized = True
            self.running = True
            
            print("✅ All subsystems initialized successfully!")
            print("="*60)
            
            return True
            
        except Exception as e:
            print(f"❌ Initialization failed: {e}")
            await self.shutdown()
            return False
    
    async def arm_system(self) -> bool:
        """Arm the ODrive system (assumes pre-configured)"""
        if not self.initialized:
            print("❌ System not initialized")
            return False
        
        print("🔧 Software arming system...")
        print("ℹ️  Motor will remain idle until commanded")
        
        try:
            # Just software safety - no motor state changes
            print("✅ Control commands enabled")
            print("📍 Motor stays in idle state until actual commands")
            
            # Get current position for reference
            print("🔍 Reading current position...")
            self.can_manager.request_encoder_data()
            await asyncio.sleep(0.3)
            
            position, velocity = self.telemetry_manager.get_position_velocity()
            if position is not None:
                print(f"📍 Current position: {position:.3f} turns")
                print(f"📊 Current velocity: {velocity:.3f} turns/s" if velocity is not None else "📊 Velocity: N/A")
            else:
                print("⚠️ No position feedback - check CAN connection")
                return False
            
            # Activate position controller (software only)
            self.position_controller.activate()
            
            self.armed = True
            print("✅ System armed (software safety released)")
            print("💡 Motor will auto-enable during position/velocity commands")
            print("🎮 Commands: 'posmode' + '-p 1.0' or 'velmode' + '-v 0.5'")
            print("🔄 Control modes: 'mode pos' or 'mode vel' (dynamic switching)")
            return True
            
        except Exception as e:
            print(f"❌ Arming failed: {e}")
            return False
    
    async def disarm_system(self):
        """Software disarm - disables control commands (motor returns to idle)"""
        print("🛑 Software disarming system...")
        
        try:
            # Deactivate controllers
            if self.position_controller:
                self.position_controller.deactivate()
            
            # Return motor to idle state
            if self.can_manager and self.can_manager.node:
                print("🛑 Returning motor to idle state...")
                self.can_manager.node.set_state_msg(1)  # IDLE
                await asyncio.sleep(0.2)
            
            self.armed = False
            print("✅ System disarmed (control commands locked)")
            print("💡 Motor returned to idle state")
            
        except Exception as e:
            print(f"⚠️ Disarm error: {e}")
            self.armed = False  # Force disarm for safety
    
    async def enable_motor_for_position_control(self):
        """Enable motor in position control mode (force position mode via CAN command)"""
        try:
            print("🚀 Enabling motor for position control...")
            
            # Clear errors first
            self.can_manager.node.clear_errors_msg()
            await asyncio.sleep(0.1)
            
            # Set control mode to POSITION_CONTROL (3) with PASSTHROUGH input mode (1)
            # Using CAN simple command 0x0B instead of parameter writes
            print("🎯 Setting control mode to POSITION_CONTROL via CAN command 0x0B")
            self.can_manager.node.set_controller_mode_msg(3, 1)  # control_mode=3 (position), input_mode=1 (passthrough)
            await asyncio.sleep(0.1)
            
            # Enter closed-loop control
            self.can_manager.node.set_state_msg(8)  # CLOSED_LOOP_CONTROL
            await asyncio.sleep(0.5)
            
            print("✅ Motor enabled in position control mode")
            return True
            
        except Exception as e:
            print(f"❌ Failed to enable motor for position control: {e}")
            return False
    
    async def enable_motor_for_velocity_control(self):
        """Enable motor in velocity control mode (force velocity mode via CAN command)"""
        try:
            print("🚀 Enabling motor for velocity control...")
            
            # Clear errors first
            self.can_manager.node.clear_errors_msg()
            await asyncio.sleep(0.1)
            
            # Set control mode to VELOCITY_CONTROL (2) with PASSTHROUGH input mode (1)
            # Using CAN simple command 0x0B instead of parameter writes
            print("⚡ Setting control mode to VELOCITY_CONTROL via CAN command 0x0B")
            self.can_manager.node.set_controller_mode_msg(2, 1)  # control_mode=2 (velocity), input_mode=1 (passthrough)
            await asyncio.sleep(0.1)
            
            # Enter closed-loop control
            self.can_manager.node.set_state_msg(8)  # CLOSED_LOOP_CONTROL
            await asyncio.sleep(0.5)
            
            print("✅ Motor enabled in velocity control mode")
            return True
            
        except Exception as e:
            print(f"❌ Failed to enable motor for velocity control: {e}")
            return False
    
    async def disable_motor(self):
        """Return motor to idle state (like stable version)"""
        try:
            if self.can_manager and self.can_manager.node:
                self.can_manager.node.set_state_msg(1)  # IDLE
                await asyncio.sleep(0.2)
                print("🛑 Motor returned to idle")
                
        except Exception as e:
            print(f"⚠️ Error disabling motor: {e}")
    
    def set_position_mode(self):
        """Set desired control mode to position (software only)"""
        self.desired_control_mode = "position"
        print("📍 Control mode set to: POSITION")
        print("💡 Motor will enable in position mode on next command")
    
    def set_velocity_mode(self):
        """Set desired control mode to velocity (software only)"""
        self.desired_control_mode = "velocity"
        print("⚡ Control mode set to: VELOCITY")
        print("� Motor will enable in velocity mode on next command")
    
    def _print_main_menu(self):
        """Print the main command menu"""
        print("\n🎮 Interactive CLI Command Menu")
        print("="*50)
        print("Commands:")
        print("  arm              - Software arm (enable control commands)")
        print("  disarm           - Software disarm (disable control commands)")
        print("  pos <value>      - Set position (auto-enables motor)      [shortcut: -p]")
        print("  vel <value>      - Set velocity (auto-enables motor)      [shortcut: -v]")
        print("  move <pos> <vel> - Move to position at velocity   [shortcut: -m]")
        print("  rel <delta>      - Relative move                  [shortcut: -r]")
        print("  osc <amp> <freq> [time] [phase] - Oscillate with optional phase [shortcut: -o]")
        print("  stop             - Stop motion & return to idle   [shortcut: -s]")
        print("  idle             - Return motor to idle state")
        print("  traj load <file> - Load trajectory file")
        print("  traj play        - Start trajectory playback")
        print("  traj pause       - Pause/resume trajectory")
        print("  traj stop        - Stop trajectory playback")
        print("  traj status      - Show trajectory status")
        print("  log start [file] - Start high-speed logging (optional custom filename)")
        print("  log stop         - Stop logging")
        print("  log status       - Show log statistics")
        print("  config load <file> - Load ODrive configuration from JSON file")
        print("  config save      - Save current configuration to ODrive NVM")
        print("  status           - Show system status")
        print("  stats            - Show performance stats")
        print("  help             - Show this menu                 [shortcut: -h]")
        print("  quit             - Exit program")
        print("-"*50)
        print("Quick shortcuts: -p <val>, -m <pos> <vel>, -r <delta>, -o <amp> <freq>, -s, -h")
        print("="*50)
    
    async def run_interactive_cli(self):
        """Run interactive command-line interface"""
        self._print_main_menu()
        
        self.cli_active = True
        
        while self.cli_active and self.running:
            try:
                # Get user input
                command = await asyncio.get_event_loop().run_in_executor(
                    None, input, "ODrive> "
                )
                
                await self._process_cli_command(command.strip())
                
            except (EOFError, KeyboardInterrupt):
                print("\n👋 Exiting...")
                break
            except Exception as e:
                print(f"❌ CLI error: {e}")
    
    async def _process_cli_command(self, command: str):
        """Process CLI command"""
        if not command:
            return
        
        parts = command.lower().split()
        cmd = parts[0]
        
        try:
            if cmd == "arm":
                await self.arm_system()
                
            elif cmd == "disarm":
                await self.disarm_system()
                
            elif cmd == "mode" and len(parts) >= 2:
                mode = parts[1].lower()
                if mode == "pos" or mode == "position":
                    self.set_position_mode()
                elif mode == "vel" or mode == "velocity":
                    self.set_velocity_mode()
                else:
                    print("Usage: mode pos|vel")
                    print("  pos/position - Set next commands to position control")
                    print("  vel/velocity - Set next commands to velocity control")
                    
            # Mode shortcuts
            elif cmd == "posmode":
                self.set_position_mode()
                
            elif cmd == "velmode":
                self.set_velocity_mode()
                
            elif cmd == "pos" and len(parts) >= 2:
                position = float(parts[1])
                if self.armed:
                    # Auto-enable motor in position mode
                    if await self.enable_motor_for_position_control():
                        success = self.position_controller.set_position(position)
                        if success:
                            print(f"🎯 Position set: {position:.3f} turns")
                            # Return to idle after command execution
                            await asyncio.sleep(1.0)  # Brief hold time
                            await self.disable_motor()
                            print("🔄 Motor returned to idle")
                        else:
                            print("❌ Position command failed")
                    else:
                        print("❌ Could not enable motor for position control")
                else:
                    print("⚠️ System not armed")
            
            # Position shortcut: -p <value>
            elif cmd == "-p" and len(parts) >= 2:
                position = float(parts[1])
                if self.armed:
                    # Auto-enable motor in position mode
                    if await self.enable_motor_for_position_control():
                        success = self.position_controller.set_position(position)
                        if success:
                            print(f"🎯 Position set: {position:.3f} turns")
                            # Return to idle after command execution
                            await asyncio.sleep(1.0)  # Brief hold time
                            await self.disable_motor()
                            print("🔄 Motor returned to idle")
                        else:
                            print("❌ Position command failed")
                    else:
                        print("❌ Could not enable motor for position control")
                else:
                    print("⚠️ System not armed")
                    
            elif cmd == "vel" and len(parts) >= 2:
                velocity = float(parts[1])
                if self.armed:
                    # Auto-enable motor in velocity mode
                    if await self.enable_motor_for_velocity_control():
                        success = self.position_controller.set_velocity(velocity)
                        if success:
                            print(f"⚡ Velocity set: {velocity:.3f} turns/s")
                            print("🔄 Velocity running continuously - use 'stop' or '-s' to stop")
                        else:
                            print("❌ Velocity command failed")
                    else:
                        print("❌ Could not enable motor for velocity control")
                else:
                    print("⚠️ System not armed")
            
            # Velocity shortcut: -v <value>
            elif cmd == "-v" and len(parts) >= 2:
                velocity = float(parts[1])
                if self.armed:
                    # Auto-enable motor in velocity mode
                    if await self.enable_motor_for_velocity_control():
                        success = self.position_controller.set_velocity(velocity)
                        if success:
                            print(f"⚡ Velocity set: {velocity:.3f} turns/s")
                            print("🔄 Velocity running continuously - use 'stop' or '-s' to stop")
                        else:
                            print("❌ Velocity command failed")
                    else:
                        print("❌ Could not enable motor for velocity control")
                else:
                    print("⚠️ System not armed")
                    
            elif cmd == "move" and len(parts) >= 3:
                position = float(parts[1])
                velocity = float(parts[2])
                if self.armed:
                    # Auto-enable motor in position mode for move commands
                    if await self.enable_motor_for_position_control():
                        success = self.position_controller.move_to_position(position, velocity)
                        if success:
                            print(f"🎯 Moving to position {position:.3f} at {velocity:.3f} turns/s")
                            # Wait for trajectory completion, then return to idle
                            await self._wait_for_trajectory_completion()
                            await self.disable_motor()
                            print("🔄 Motor returned to idle")
                        else:
                            print("❌ Move command failed")
                    else:
                        print("❌ Could not enable motor for position control")
                else:
                    print("⚠️ System not armed")
            
            # Move shortcut: -m <pos> <vel>
            elif cmd == "-m" and len(parts) >= 3:
                position = float(parts[1])
                velocity = float(parts[2])
                if self.armed:
                    # Auto-enable motor in position mode for move commands
                    if await self.enable_motor_for_position_control():
                        success = self.position_controller.move_to_position(position, velocity)
                        if success:
                            print(f"🎯 Moving to position {position:.3f} at {velocity:.3f} turns/s")
                            # Wait for trajectory completion, then return to idle
                            await self._wait_for_trajectory_completion()
                            await self.disable_motor()
                            print("🔄 Motor returned to idle")
                        else:
                            print("❌ Move command failed")
                    else:
                        print("❌ Could not enable motor for position control")
                else:
                    print("⚠️ System not armed")
                    
            elif cmd == "rel" and len(parts) >= 2:
                delta = float(parts[1])
                velocity = float(parts[2]) if len(parts) >= 3 else None
                if self.armed:
                    # Auto-enable motor in position mode for relative move commands
                    if await self.enable_motor_for_position_control():
                        success = self.position_controller.relative_move(delta, velocity)
                        if success:
                            vel_str = f" at {velocity:.3f} turns/s" if velocity else ""
                            print(f"🎯 Relative move {delta:+.3f} turns{vel_str}")
                            # Wait for trajectory completion, then return to idle
                            await self._wait_for_trajectory_completion()
                            await self.disable_motor()
                            print("🔄 Motor returned to idle")
                        else:
                            print("❌ Relative move command failed")
                    else:
                        print("❌ Could not enable motor for position control")
                else:
                    print("⚠️ System not armed")
            
            # Relative move shortcut: -r <delta> [vel]
            elif cmd == "-r" and len(parts) >= 2:
                delta = float(parts[1])
                velocity = float(parts[2]) if len(parts) >= 3 else None
                if self.armed:
                    # Auto-enable motor in position mode for relative move commands
                    if await self.enable_motor_for_position_control():
                        success = self.position_controller.relative_move(delta, velocity)
                        if success:
                            vel_str = f" at {velocity:.3f} turns/s" if velocity else ""
                            print(f"🎯 Relative move {delta:+.3f} turns{vel_str}")
                            # Wait for trajectory completion, then return to idle
                            await self._wait_for_trajectory_completion()
                            await self.disable_motor()
                            print("🔄 Motor returned to idle")
                        else:
                            print("❌ Relative move command failed")
                    else:
                        print("❌ Could not enable motor for position control")
                else:
                    print("⚠️ System not armed")
                    
            elif cmd == "osc" and len(parts) >= 3:
                amplitude = float(parts[1])
                frequency = float(parts[2])
                duration = float(parts[3]) if len(parts) >= 4 else None
                phase_deg = float(parts[4]) if len(parts) >= 5 else 0.0
                if self.armed:
                    # Auto-enable motor in position mode for oscillate commands
                    if await self.enable_motor_for_position_control():
                        success = self.position_controller.oscillate(amplitude, frequency, duration, phase_deg)
                        if success:
                            duration_str = f" for {duration:.1f}s" if duration else " (continuous)"
                            phase_str = f", phase {phase_deg:.0f}°" if phase_deg != 0 else ""
                            print(f"🔄 Oscillating ±{amplitude:.3f} at {frequency:.1f} Hz{phase_str}{duration_str}")
                            # Wait for oscillation completion (if finite), then return to idle
                            if duration:  # Only wait if finite duration
                                await self._wait_for_trajectory_completion()
                                await self.disable_motor()
                                print("🔄 Motor returned to idle")
                            else:
                                print("💡 Use 'stop' or '-s' to stop continuous oscillation")
                        else:
                            print("❌ Oscillate command failed")
                    else:
                        print("❌ Could not enable motor for position control")
                else:
                    print("⚠️ System not armed")
            
            # Oscillate shortcut: -o <amp> <freq> [time]
            elif cmd == "-o" and len(parts) >= 3:
                amplitude = float(parts[1])
                frequency = float(parts[2])
                duration = float(parts[3]) if len(parts) >= 4 else None
                if self.armed:
                    # Auto-enable motor in position mode for oscillate commands
                    if await self.enable_motor_for_position_control():
                        success = self.position_controller.oscillate(amplitude, frequency, duration)
                        if success:
                            duration_str = f" for {duration:.1f}s" if duration else " (continuous)"
                            print(f"🔄 Oscillating ±{amplitude:.3f} at {frequency:.1f} Hz{duration_str}")
                            # Wait for oscillation completion (if finite), then return to idle
                            if duration:  # Only wait if finite duration
                                await self._wait_for_trajectory_completion()
                                await self.disable_motor()
                                print("🔄 Motor returned to idle")
                            else:
                                print("💡 Use 'stop' or '-s' to stop continuous oscillation")
                        else:
                            print("❌ Oscillate command failed")
                    else:
                        print("❌ Could not enable motor for position control")
                else:
                    print("⚠️ System not armed")
                    
            elif cmd == "stop":
                if self.armed:
                    # Stop trajectory and velocity control
                    self.position_controller.stop_trajectory()
                    # Set velocity to 0 to stop spinning
                    self.position_controller.set_velocity(0.0)
                    print("🛑 Stopped all motion - velocity set to 0")
                    await asyncio.sleep(0.5)  # Brief pause
                    await self.disable_motor()
                    print("🔄 Motor returned to idle")
                else:
                    print("⚠️ System not armed")
            
            # Stop shortcut: -s
            elif cmd == "-s":
                if self.armed:
                    # Stop trajectory and velocity control
                    self.position_controller.stop_trajectory()
                    # Set velocity to 0 to stop spinning
                    self.position_controller.set_velocity(0.0)
                    print("🛑 Stopped all motion - velocity set to 0")
                    await asyncio.sleep(0.5)  # Brief pause
                    await self.disable_motor()
                    print("🔄 Motor returned to idle")
                else:
                    print("⚠️ System not armed")
                    
            elif cmd == "idle":
                await self.disable_motor()
                print("🛑 Motor returned to idle state")
                    
            elif cmd == "log" and len(parts) >= 2:
                subcmd = parts[1]
                if subcmd == "start":
                    # Support optional filename: log start [filename]
                    filename = None
                    if len(parts) >= 3:
                        filename = " ".join(parts[2:])  # Handle filenames with spaces
                    await self._start_logging(filename)
                elif subcmd == "stop":
                    await self._stop_logging()
                elif subcmd == "status":
                    await self._show_log_status()
                else:
                    print("Usage: log start [filename]|stop|status")
                    print("  filename: Optional custom filename (with datetime suffix added)")
                    print("  Examples: log start mydata.csv")
                    print("           log start logs/experiment.csv")
            
            elif cmd == "traj" and len(parts) >= 2:
                subcmd = parts[1]
                if subcmd == "load" and len(parts) >= 3:
                    filepath = " ".join(parts[2:])  # Handle filenames with spaces
                    await self._load_trajectory(filepath)
                elif subcmd == "play":
                    await self._play_trajectory()
                elif subcmd == "pause":
                    await self._pause_trajectory()
                elif subcmd == "stop":
                    await self._stop_trajectory_playback()
                elif subcmd == "status":
                    await self._show_trajectory_status()
                else:
                    print("Usage: traj load <file>|play|pause|stop|status")
                    
            elif cmd == "status":
                await self._show_status()
                
            elif cmd == "stats":
                await self._show_performance_stats()
                
            elif cmd == "config" and len(parts) >= 2:
                subcommand = parts[1].lower()
                if subcommand == "load" and len(parts) >= 3:
                    config_file = parts[2]
                    await self._load_config_file(config_file)
                elif subcommand == "save":
                    await self._save_config_to_odrive()
                else:
                    print("Usage: config load <file.json> | config save")
                
            elif cmd in ["quit", "exit", "q"]:
                self.cli_active = False
                
            elif cmd == "help":
                self._print_main_menu()
            
            # Help shortcut: -h
            elif cmd == "-h":
                self._print_main_menu()
                
            else:
                print(f"❌ Unknown command: {command}")
                print("Type 'help' or '-h' for available commands")
                
        except ValueError as e:
            print(f"❌ Invalid parameters: {e}")
        except Exception as e:
            print(f"❌ Command error: {e}")
    
    async def _show_status(self):
        """Show current system status"""
        print("\n📊 System Status:")
        print(f"   Initialized: {'✅' if self.initialized else '❌'}")
        print(f"   Armed: {'✅' if self.armed else '❌'}")
        print(f"   Running: {'✅' if self.running else '❌'}")
        
        if self.telemetry_manager:
            data = self.telemetry_manager.get_latest_data()
            pos = data.get('position')
            vel = data.get('velocity')
            temp = data.get('motor_temp')
            fet_temp = data.get('fet_temp')
            bus_voltage = data.get('bus_voltage')
            bus_current = data.get('bus_current')
            iq_setpoint = data.get('iq_setpoint')
            iq_measured = data.get('iq_measured')
            torque_setpoint = data.get('torque_setpoint')
            torque_estimate = data.get('torque_estimate')
            
            print(f"   Position: {pos:.3f} turns" if pos is not None else "   Position: N/A")
            print(f"   Velocity: {vel:.3f} turns/s" if vel is not None else "   Velocity: N/A")
            print(f"   Motor Temp: {temp:.1f}°C" if temp is not None else "   Motor Temp: N/A")
            print(f"   FET Temp: {fet_temp:.1f}°C" if fet_temp is not None else "   FET Temp: N/A")
            print(f"   Bus Voltage: {bus_voltage:.1f}V" if bus_voltage is not None else "   Bus Voltage: N/A")
            print(f"   Bus Current: {bus_current:.2f}A" if bus_current is not None else "   Bus Current: N/A")
            print(f"   Iq Setpoint: {iq_setpoint:.2f}A" if iq_setpoint is not None else "   Iq Setpoint: N/A")
            print(f"   Iq Measured: {iq_measured:.2f}A" if iq_measured is not None else "   Iq Measured: N/A")
            print(f"   Torque Setpoint: {torque_setpoint:.4f}Nm" if torque_setpoint is not None else "   Torque Setpoint: N/A")
            print(f"   Torque Estimate: {torque_estimate:.4f}Nm" if torque_estimate is not None else "   Torque Estimate: N/A")
        
        if self.position_controller:
            stats = self.position_controller.get_statistics()
            print(f"   Target Position: {stats['current_target']:.3f} turns")
            print(f"   Position Error: {stats['position_error']:.3f} turns")
            print(f"   Control Active: {'✅' if stats['active'] else '❌'}")
        
        print()
    
    async def _show_performance_stats(self):
        """Show performance statistics"""
        print("\n📈 Performance Statistics:")
        
        if self.can_manager:
            stats = self.can_manager.get_performance_stats()
            print(f"   Commands Sent: {stats['commands_sent']}")
            print(f"   Responses Received: {stats['responses_received']}")
            print(f"   Command Queue: {stats['command_queue_size']}")
            print(f"   Telemetry Queue: {stats['telemetry_queue_size']}")
            print(f"   Errors: {stats['errors']}")
        
        if self.position_controller:
            stats = self.position_controller.get_statistics()
            print(f"   Position Commands: {stats['commands_sent']}")
        
        print()
    
    async def _start_logging(self, custom_filename=None):
        """Start high-speed logging with optional custom filename"""
        if self.logging_active:
            print("⚠️ Logging already active")
            return
            
        # Create new logger with custom filename if provided
        if custom_filename:
            # Add datetime suffix to custom filename
            filename_with_datetime = HighSpeedLogger.create_filename_with_datetime(custom_filename)
            print(f"📝 Creating custom log file: {filename_with_datetime}")
            self.logger = HighSpeedLogger(filename_with_datetime)
        elif not self.logger:
            print("❌ Logger not initialized")
            return
            
        self.logger.start()
        self.logging_active = True
        
        # Start logging integration thread
        self._logging_integration_task = asyncio.create_task(self._logging_integration_loop())
        print("✅ High-speed logging started at 100Hz (relative timestamps from start)")
        print(f"📊 Columns: time_rel_s (seconds from start), position, velocity, temperatures")
    
    async def _stop_logging(self):
        """Stop logging"""
        if not self.logging_active:
            print("⚠️ Logging not active")
            return
            
        self.logging_active = False
        
        # Stop integration task
        if hasattr(self, '_logging_integration_task'):
            self._logging_integration_task.cancel()
            try:
                await self._logging_integration_task
            except asyncio.CancelledError:
                pass
        
        self.logger.stop()
        print("✅ Logging stopped")
    
    async def _show_log_status(self):
        """Show logging status and statistics"""
        print(f"\n📝 Logging Status:")
        print(f"   Active: {'✅' if self.logging_active else '❌'}")
        
        if self.logger:
            stats = self.logger.get_statistics()
            print(f"   File: {stats['filename']}")
            print(f"   Records written: {stats['writes']}")
            print(f"   Dropped records: {stats['drops']}")
            print(f"   Queue size: {stats['queue_size']}")
        print()
    
    async def _logging_integration_loop(self):
        """Integration loop to feed telemetry data to logger"""
        while self.logging_active:
            try:
                # Get latest telemetry data
                telemetry_data = self.telemetry_manager.get_latest_data()
                controller_stats = self.position_controller.get_statistics()
                
                # Log the data
                self.logger.log_data(telemetry_data, controller_stats)
                
                # Sleep for 100Hz logging rate
                await asyncio.sleep(0.01)
                
            except Exception as e:
                print(f"❌ Logging integration error: {e}")
                await asyncio.sleep(0.1)
    
    async def _wait_for_trajectory_completion(self):
        """Wait for trajectory to complete"""
        print("⏳ Waiting for motion to complete...")
        while self.position_controller.trajectory_active:
            await asyncio.sleep(0.1)
        print("✅ Motion completed")
    
    async def _load_trajectory(self, filepath: str):
        """Load trajectory file"""
        if not self.trajectory_player:
            print("❌ Trajectory player not initialized")
            return
            
        success = self.trajectory_player.load_trajectory(filepath)
        if success:
            print("✅ Trajectory loaded successfully")
        
    async def _play_trajectory(self):
        """Start trajectory playback"""
        if not self.trajectory_player:
            print("❌ Trajectory player not initialized")
            return
            
        if not self.armed:
            print("⚠️ System must be armed to play trajectory")
            return
            
        # Auto-enable motor in position mode for trajectory playback
        if await self.enable_motor_for_position_control():
            success = self.trajectory_player.start_playback()
            if success:
                print("✅ Trajectory playback started")
            else:
                print("❌ Failed to start trajectory playback")
        else:
            print("❌ Could not enable motor for position control")
    
    async def _pause_trajectory(self):
        """Pause/resume trajectory playback"""
        if not self.trajectory_player:
            print("❌ Trajectory player not initialized")
            return
            
        self.trajectory_player.pause_playback()
    
    async def _stop_trajectory_playback(self):
        """Stop trajectory playback"""
        if not self.trajectory_player:
            print("❌ Trajectory player not initialized")
            return
            
        self.trajectory_player.stop_playback()
    
    async def _show_trajectory_status(self):
        """Show trajectory status"""
        if not self.trajectory_player:
            print("❌ Trajectory player not initialized")
            return
            
        status = self.trajectory_player.get_status()
        print(f"\n📈 Trajectory Status:")
        print(f"   📂 Loaded: {'✅' if status['loaded'] else '❌'}")
        if status['loaded_file']:
            print(f"   📄 File: {status['loaded_file']}")
        print(f"   ▶️  Playing: {'✅' if status['playing'] else '❌'}")
        print(f"   ⏸️  Paused: {'✅' if status['paused'] else '❌'}")
        print(f"   🔄 Loop mode: {'✅' if status['loop_mode'] else '❌'}")
        print(f"   🏃 Speed: {status['time_scale']}x")
        print(f"   🛡️ Limits: {status['position_limits'][0]:.2f} to {status['position_limits'][1]:.2f} turns")
        
        if status['stats']:
            stats = status['stats']
            print(f"   📊 Points executed: {stats['points_executed']}")
            print(f"   🔄 Loops completed: {stats['loops_completed']}")
            if stats['max_error'] > 0:
                print(f"   📏 Max error: {stats['max_error']:.4f} turns")
                print(f"   📊 Avg error: {stats['avg_error']:.4f} turns")
        print()
    
    async def _load_config_file(self, config_file: str):
        """Load ODrive configuration from JSON file"""
        import os
        import json
        from utils.can_simple_utils import CanSimpleNode
        
        try:
            # Check if config file exists
            if not os.path.exists(config_file):
                print(f"❌ Config file not found: {config_file}")
                return
            
            # Check if endpoints file exists
            endpoints_file = "flat_endpoints.json"
            if not os.path.exists(endpoints_file):
                print(f"❌ Endpoints file not found: {endpoints_file}")
                print("   Make sure flat_endpoints.json is in the current directory")
                return
            
            print(f"📋 Loading configuration from: {config_file}")
            
            # Load the config file
            with open(config_file, 'r') as f:
                config_data = json.load(f)
            
            # Load endpoints data
            with open(endpoints_file, 'r') as f:
                endpoint_data = json.load(f)
            
            print(f"📡 Found {len(config_data)} configuration parameters")
            
            # Create EndpointAccess for config loading
            from utils.can_restore_config import EndpointAccess, restore_config
            
            # # Use the existing CAN manager's node
            # with CanSimpleNode(bus=self.can_manager.bus, node_id=self.node_id) as config_node:
            #     odrv = EndpointAccess(node=config_node, endpoint_data=endpoint_data)
                
            #     print("🔍 Checking ODrive version compatibility...")
            #     await odrv.version_check()
                
            #     print("⚙️ Applying configuration...")
            #     await restore_config(odrv, config_data)
                
            #     print("✅ Configuration loaded successfully!")
            #     print("💡 Use 'config save' to save to ODrive NVM and reboot")
            # ✅ FIXED: Creates temporary bus for config operations
            # ✅ NEW APPROACH: Use existing CAN infrastructure without bus switching
            print("⚙️ Applying configuration using existing CAN manager...")

            # Load endpoint definitions for parameter writing
            with open("flat_endpoints.json", 'r') as f:
                endpoints_data = json.load(f)['endpoints']

            successful = 0
            failed = 0

            print(f"📝 Writing {len(config_data)} configuration parameters...")

            for param_name, value in config_data.items():
                try:
                    print(f"   Setting {param_name} = {value}")
                    
                    # Write parameter using existing CAN manager infrastructure
                    await self._write_parameter_direct(param_name, value, endpoints_data)
                    successful += 1
                    
                    # Small delay between parameters
                    await asyncio.sleep(0.05)
                    
                except Exception as e:
                    print(f"   ❌ Failed to set {param_name}: {e}")
                    failed += 1

            print(f"✅ Configuration loading complete:")
            print(f"   ✅ Successful: {successful}")
            print(f"   ❌ Failed: {failed}")

            if failed == 0:
                print("🎉 All parameters loaded successfully!")
                print("💡 Use 'config save' to save to ODrive NVM and reboot")
            else:
                print(f"⚠️ {failed} parameters failed to load - check ODrive state and parameter compatibility")

                
        except FileNotFoundError as e:
            print(f"❌ File not found: {e}")
        except json.JSONDecodeError as e:
            print(f"❌ Invalid JSON format: {e}")
        except Exception as e:
            print(f"❌ Config loading failed: {e}")
            import traceback
            traceback.print_exc()
    
    async def _write_parameter_direct(self, param_name: str, value, endpoints_data: dict):
        """Write parameter using existing CAN manager infrastructure"""
        import struct
        
        # Parameter writing constants (from can_restore_config.py)
        OPCODE_WRITE = 0x01
        RX_SDO = 0x04
        
        # Format lookup for different data types
        FORMAT_LOOKUP = {
            'bool': '?',
            'uint8': 'B', 'int8': 'b',
            'uint16': 'H', 'int16': 'h', 
            'uint32': 'I', 'int32': 'i',
            'uint64': 'Q', 'int64': 'q',
            'float': 'f'
        }
        
        if param_name not in endpoints_data:
            raise ValueError(f"Parameter {param_name} not found in endpoints")
        
        endpoint_info = endpoints_data[param_name]
        endpoint_id = endpoint_info['id']
        endpoint_type = endpoint_info['type']
        
        if endpoint_type not in FORMAT_LOOKUP:
            raise ValueError(f"Unsupported parameter type: {endpoint_type}")
        
        format_char = FORMAT_LOOKUP[endpoint_type]
        
        # Pack the write command
        try:
            data = struct.pack('<BHB' + format_char, OPCODE_WRITE, endpoint_id, 0, value)
        except struct.error as e:
            raise ValueError(f"Failed to pack value {value} for parameter {param_name}: {e}")
        
        # Send the write command using existing CAN manager
        cmd_id = self.node_id << 5 | RX_SDO
        await self._send_can_command(cmd_id, data)

    async def _send_can_command(self, cmd_id: int, data: bytes):
        """Send CAN command using existing infrastructure"""
        import can
        
        message = can.Message(
            arbitration_id=cmd_id,
            data=data,
            is_extended_id=False
        )
        
        # Use the existing CAN manager's bus
        self.can_manager.bus.send(message)
        
        # Small delay to ensure message is sent
        await asyncio.sleep(0.01)


    async def _save_config_to_odrive(self):
        """Save current configuration to ODrive NVM and reboot"""
        try:
            from utils.can_simple_utils import CanSimpleNode, REBOOT_ACTION_SAVE
            
            print("💾 Saving configuration to ODrive NVM...")
            
            # Use the existing CAN manager's node for reboot command
            with CanSimpleNode(bus=self.can_manager.bus, node_id=self.node_id) as config_node:
                config_node.reboot_msg(REBOOT_ACTION_SAVE)
                
                print("✅ Configuration saved to NVM")
                print("🔄 ODrive rebooting...")
                print("⚠️ System will need re-initialization after reboot")
                
                # Mark system as needing re-initialization
                self.initialized = False
                self.armed = False
                
        except Exception as e:
            print(f"❌ Config save failed: {e}")
            import traceback
            traceback.print_exc()
    
    async def shutdown(self):
        """Shutdown all subsystems gracefully"""
        print("\n🛑 Shutting down system...")
        
        self.running = False
        
        # Stop trajectory playback first
        if self.trajectory_player:
            self.trajectory_player.stop_playback()
        
        # Stop logging
        if self.logging_active:
            await self._stop_logging()
            await self._stop_logging()
        
        # Disarm system
        await self.disarm_system()
        
        # Stop telemetry
        if self.telemetry_manager:
            self.telemetry_manager.stop()
        
        # Shutdown CAN
        if self.can_manager:
            await self.can_manager.shutdown()
        
        print("✅ System shutdown complete")



async def main():
    """Main entry point"""
    # Load CAN settings from config file
    can_settings = load_can_settings("can_settings.json")
    node_id = can_settings.get("can_node_id", 1)
    can_interface = can_settings.get("can_interface", "gs_usb")
    can_channel = can_settings.get("can_channel", "can0")
    can_bitrate = can_settings.get("can_bitrate", 1000000)

    # Patch SimpleCANManager to use loaded CAN settings
    class PatchedSimpleCANManager(SimpleCANManager):
        async def initialize(self) -> bool:
            try:
                print(f"🔌 Initializing Simple CAN Manager (interface={can_interface}, channel={can_channel}, bitrate={can_bitrate})...")
                self.bus = can.Bus(
                    interface=can_interface,
                    channel=can_channel,
                    index=0,
                    bitrate=can_bitrate
                )
                self.node = CanSimpleNode(self.bus, self.node_id)
                self._node_context = self.node.__enter__()
                self.node.clear_errors_msg()
                self.running = True
                self._start_threads()
                print("✅ Simple CAN Manager initialized")
                return True
            except Exception as e:
                print(f"❌ CAN Manager initialization failed: {e}")
                return False

    # Patch the factory to use our CAN settings
    def create_patched_can_manager(node_id):
        return PatchedSimpleCANManager(node_id=node_id, bitrate=can_bitrate)

    # Patch the global factory used by HighPerformanceODriveSystem
    import modules.simple_can_manager as scm
    scm.create_simple_can_manager = create_patched_can_manager

    # --- Node ID discovery/assignment logic ---
    discovered_node_id = await discover_and_assign_node_id(can_interface, can_channel, can_bitrate, preferred_node_id=node_id)
    if discovered_node_id is not None and discovered_node_id != node_id:
        print(f"Updating can_settings.json with discovered node ID {discovered_node_id}...")
        # Update config file
        config_path = os.path.join(os.path.dirname(__file__), "can_settings.json")
        try:
            with open(config_path, "r") as f:
                config = json.load(f)
            config["can_node_id"] = discovered_node_id
            with open(config_path, "w") as f:
                json.dump(config, f, indent=2)
            node_id = discovered_node_id
        except Exception as e:
            print(f"⚠️ Could not update can_settings.json: {e}")

    # Now run the real system
    try:
        system = HighPerformanceODriveSystem(node_id=node_id)
        success = await system.initialize()
        if not success:
            print("❌ Initialization failed")
            return 1
        await system.run_interactive_cli()
    except KeyboardInterrupt:
        print("\n⚠️ Interrupted by user")
    except Exception as e:
        print(f"❌ System error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        await system.shutdown()
    return 0

if __name__ == "__main__":
    exit(asyncio.run(main()))




