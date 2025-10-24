# Import all libraries ----------
import sys								# Accesses system-specific parameters (ex. sys.exit() to quit script)
import time								# Provides time-related functions (ex. time.sleep())
import math								# Provides standard mathematical operations (ex. sin, cos, sqrt)
import struct							# Used for converting between Python values and C-style binary data (critical for packing/unpackign CAN messages)
import can								# Provides interfaces for sending/receiving CAN messages
from collections import deque			# Double-ended queue, great for maintaining rolling buffer (ex. real-time plot)
from PyQt5 import QtWidgets, QtCore		# GUI framework for python (ex. buttons, sliders, timers, layouts)
#import pyqtgraph as pg					# High-speed real-time plotting library build on PyQt5

# Define constant variables ----------
Pitch_12 = 0.157		# 4mm ball screw pitch
Pitch_23 = 0.197		# 5mm ball screw pitch
Home_HS_Offset = 3.9	# Travel distance from hard stop to home position
node_id_act = 1			# Node id for motor in operation

# Set up Bus object ----------
bus = can.interface.Bus("can0", interface="socketcan")
    # can0 is the default for first CAN adapter, and you set up the Pi to automatically set up can0 on startup
    # SocketCan is the backend native to linux kernal
    
# Define global functions ----------
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
    latest_pos = None					# Default value in case no feedback is received before timeout
    arb_id = (node_id << 5) | 0x09		# Same as before, see above for more detail
    msg = can.Message(arbitration_id=arb_id, data=[], is_extended_id=False, is_remote_frame=True)
        # Creates Remote Transmission Request (RTR) frame (reason for last bit in ())
            # Special CAN fram that asks ODrive to respond with data, without including any payload
    bus.send(msg)						# Send RTR message
    send_time = time.time()					# Record the time when request message is sent
    while time.time() - send_time < .010:	# 10ms window to loop until a CAN message is received
        msg = bus.recv(timeout = .008)	# 8ms window to receive a single message
            # If no message is received within 8ms, bus.recv returns 'None'
        if not msg:						# If msg is returned as 'None', 
            break						# Break out of the entire while loop, no message received
        if msg.arbitration_id == arb_id and not msg.is_remote_frame:	# If message has correct ID and isn't requesting feedback
            response_time = time.time() - send_time	# Time between sending request and obtaining response
            print(f"Feedback received in {response_time: .4f} seconds")	# Print time delay to shell
            latest_pos, _ = struct.unpack('<ff',msg.data)
                # '<ff' returns 2x 32-bit floats from the 8-byte payload (position, velocity)
                # Stores position as latest_pos
                #' _ ' ignores the velocity feedback
                    # Can assign another value here if you want to use velocity feedback
            break	# Break the while loop after receiving feedback
    return latest_pos	# Function call will return the feedback position        

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
            
class ControlPanel(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()	# Initializes the parent QWidget
        self.init_ui()		# Calls another function to set up the windows content
        
    def init_ui(self):
        self.setWindowTitle("Experimenting With GUI")
        
        # Create status window
        self.status_box = QtWidgets.QLabel("Initializing")
        self.status_box.setStyleSheet("background: green; color: black; font-size: 20px;")
        self.status_box.setAlignment(QtCore.Qt.AlignCenter)
        
        # Create timer to automatically update status box
        self.status_timer = QtCore.QTimer()						# Create timer object
        self.status_timer.timeout.connect(self.update_status)	# Define which function to run
        self.status_timer.setSingleShot(False)					# Repeat automatically without being told (I think)
        self.status_timer.start(200)							# Define how often to repeat
        
        # Create buttons to change motor positions
        self.up_button = QtWidgets.QPushButton("UP")
        self.up_button.setMinimumHeight(40)
        self.up_button.clicked.connect(self.increase_pos)
        self.down_button = QtWidgets.QPushButton("DOWN")
        self.down_button.setMinimumHeight(40)
        self.down_button.clicked.connect(self.decrease_pos)
        
        # Create button to enable velocity feedforward control
        self.vel_ctrl_button = QtWidgets.QPushButton("VEL_CTRL")
        self.vel_ctrl_button.setCheckable(True)	# Sets as latching push button
        self.vel_ctrl_button.setMinimumHeight(40)
        
        # Create slider to adjust motor velocity (vel_ff)
        self.vel_slider_label = QtWidgets.QLabel("vel_ff = 10000")
        self.vel_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.vel_slider.setMinimum(10) # Ensures min is greater than 0, so ODrive doesn't default to super fast
        self.vel_slider.setMaximum(10000)
        self.vel_slider.setValue(5000)
        self.vel_slider.valueChanged.connect(self.update_slider_label) # Call method if value changes to update slider label
        
        # Create exit button
        self.exit_button = QtWidgets.QPushButton("Exit")	# Create button and assign label
        self.exit_button.setMinimumHeight(40)				# Set button height
        self.exit_button.clicked.connect(self.close_app)	# Tell button what to do when pressed
        
        # Set layout
        layout = QtWidgets.QVBoxLayout()			# Create layout
        layout.addWidget(self.status_box)			# Add status box
        layout.addWidget(self.up_button)			# Add exit button to layout
        layout.addWidget(self.down_button)			# Add exit button to layout
        layout.addWidget(self.vel_ctrl_button)		# Add exit button to layout
        layout.addWidget(self.vel_slider_label)		# Add velocity ff slider to layout
        layout.addWidget(self.vel_slider)			# Add velocity ff slider to layout
        layout.addWidget(self.exit_button)			# Add exit button to layout
        self.setLayout(layout)						# Assign layout to class object
        self.show()									# Displays the window
        
    def update_slider_label(self):
        value = self.vel_slider.value()
        self.vel_slider_label.setText(f"vel_ff = {value:.2f}")
        
    def increase_pos(self):
        Current_pos = request_pos(node_id_act)
        New_pos = Current_pos + 1
        if self.vel_ctrl_button.isChecked():
            velocity_ff = self.vel_slider.value()
        else:
            velocity_ff = 0
        send_pos(New_pos, velocity_ff, node_id_act)
        
    def decrease_pos(self):
        Current_pos = request_pos(node_id_act)
        New_pos = Current_pos - 1
        if self.vel_ctrl_button.isChecked():
            velocity_ff = self.vel_slider.value() * (-1)
        else:
            velocity_ff = 0
        send_pos(New_pos, velocity_ff, node_id_act)
        
    def close_app(self):
        QtWidgets.QApplication.quit()	# Close application
        
    def update_status(self):
        Current_pos = request_pos(node_id_act)
        self.status_box.setText(f"Current Position: {Current_pos: .2f}")
        
if __name__ == "__main__":
    enter_closed_loop(node_id_act)
    app = QtWidgets.QApplication(sys.argv)
    window = ControlPanel()
    sys.exit(app.exec_())
    