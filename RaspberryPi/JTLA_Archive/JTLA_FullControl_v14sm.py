# === Define libraries ========================================================
import sys
import time
import math
import struct
import can
from collections import deque
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

# === Configuration and global constants ======================================
bus = can.interface.Bus("can0", interface="socketcan")
Pitch_12 = 0.157
Pitch_23 = 0.197

# === Configure everything for reading LVDT outputs ===========================
# Add path and library
sys.path.append('/home/neon1/myenv/lib/python3.11/site-packages/ADS1263')
from ADS1263 import ADS1263
# Settings
VREF = 5.0        # Reference voltage
DIFF_CHANNEL = 0  # Differential input channel
UPDATE_INTERVAL = 100  # Plot update rate in milliseconds (100ms = 10Hz)
WINDOW_SECONDS = 10    # How much history to keep on screen
# Initialize data storage
max_points = int(WINDOW_SECONDS * 1000 / UPDATE_INTERVAL)
time_data = deque(maxlen=max_points)
voltage_data = deque(maxlen=max_points)
# Initialize ADC
ADC = ADS1263()
if (ADC.ADS1263_init_ADC1('ADS1263_400SPS') == -1):
    print("Failed to initialize ADS1263.")
    exit()
ADC.ADS1263_SetMode(1)  # Differential mode

# === Define global functions =================================================
# Function to pack/send position commands to motor controllers
def send_position_command(position, node_id):
    vel_ff = 0
    torque_ff = 0
    data = struct.pack('<fhh', position, vel_ff, torque_ff)
    arb_id = (node_id << 5) | 0x0C
    msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
    bus.send(msg)

# Function to request/unpack position feedback from motor controllers
def request_latest_feedback(node_id):
    latest_pos = None
    arb_id = (node_id << 5) | 0x09
    msg = can.Message(arbitration_id=arb_id, data=[], is_extended_id=False, is_remote_frame=True)
    bus.send(msg)
    start = time.time()
    while time.time() - start < 0.010:
        msg = bus.recv(timeout=0.008)
        if not msg:
            break
        if msg.arbitration_id == arb_id and not msg.is_remote_frame:
            latest_pos, _ = struct.unpack('<ff', msg.data)
    return latest_pos

# Function to send controller into closed loop control (enable PID loop)
def enter_closed_loop(node_id):
    arb_id_set_state = (node_id << 5) | 0x07
    msg = can.Message(arbitration_id=arb_id_set_state, data=struct.pack('<I', 8), is_extended_id=False)
    bus.send(msg)
    while True:
        msg = bus.recv(timeout=1.0)
        if msg and msg.arbitration_id == ((node_id << 5) | 0x01):
            _, axis_state, _, _ = struct.unpack('<IBBB', msg.data[:7])
            if axis_state == 8:
                break
            
# Function to pull controller out of closed loop control (disable PID loop)        
def exit_closed_loop(node_id):
    arb_id_set_state = (node_id << 5) | 0x07  # 0x07 = Set Axis State
    msg = can.Message(arbitration_id=arb_id_set_state, data=struct.pack('<I', 1), is_extended_id=False)
    bus.send(msg)
    print(f"Sent Set_Axis_State(IDLE) to node {node_id}")
    
# === Define all of the motor homing stuff ====================================
class HomingThread(QtCore.QThread):
    homing_done = QtCore.pyqtSignal(list)

    def __init__(self, motor_nodes):
        super().__init__()
        self.motor_nodes = motor_nodes
        self.home_offsets = [0.0, 0.0, 0.0]
        self.interrupted = False	# Initialize boolean as not interrupted

    def run(self):
        def home_motor(index, node_id, direction, pitch):
            step_size = 0.5 * direction
            delay_step = 0.125
            backoff_dist = 3.9 / pitch
            for attempt in range(5):
                pos = request_latest_feedback(node_id)
                if pos is not None:
                    break
                print(f"Motor {index+1} feedback attempt {attempt+1} failed.")
                time.sleep(0.5)
            if pos is None:
                print(f"Motor {index+1} feedback failed after 5 attempts. Defaulting to 0.")
                pos = 0.0
            else:
                print(f"Homing Motor {index+1}... Starting at {pos:.2f}")

            while not self.interrupted:
                send_position_command(pos + step_size, node_id)
                time.sleep(delay_step)
                new_pos = request_latest_feedback(node_id)
                if abs(new_pos - pos) < 0.05:
                    hardstop = new_pos
                    home_pos = hardstop - direction * backoff_dist
                    for j in range(21):		# Use this to slow down how quickly its sent home
                        if self.interrupted:
                            return
                        interp_pos = hardstop + (home_pos - hardstop) * (j / 20)
                        send_position_command(interp_pos, node_id)
                        time.sleep(0.05)
                    time.sleep(2.0)
                    return home_pos
                pos = new_pos

        self.home_offsets[0] = home_motor(0, self.motor_nodes[0], +1, Pitch_12)
        #self.home_offsets[2] = home_motor(2, self.motor_nodes[2], -1, Pitch_23)	Uncomment to home motor 3
        if not self.interrupted:
            self.homing_done.emit(self.home_offsets)

# === Define the control panel for live plotting and GUI ======================
class ControlPanel(QtWidgets.QMainWindow):
    # Function to set up the control panel
    def __init__(self):
        super().__init__()
        self.setWindowTitle("JTLA DEMO")
        self.sine_sweep_active = False
        self.sine_start_time = 0
        self.motor_nodes = [0, 1, 2]
        self.home_offsets = [0.0, 0.0, 0.0]
        self.homing_thread = None

        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QHBoxLayout()
        central_widget.setLayout(main_layout)

        self.plot = pg.PlotWidget(title="Actuator Tip Position")
        self.plot.setYRange(-2.5, 2.5)
        self.plot.showGrid(x=False, y=True, alpha=0.3)
        self.plot.addLegend(offset=(-10, 10))
        self.cmd_curve = self.plot.plot(pen='g', name="Commanded Position")
        self.fb_curve = self.plot.plot(pen='r', name="Actual Position")
        main_layout.addWidget(self.plot, stretch=5)

        control_panel = QtWidgets.QVBoxLayout()
        main_layout.addLayout(control_panel, stretch=1)

        self.status_box = QtWidgets.QLabel("Initializing...")
        self.status_box.setStyleSheet("background: black; color: white; font-size: 20px;")
        self.status_box.setAlignment(QtCore.Qt.AlignCenter)
        control_panel.addWidget(self.status_box)

        label = QtWidgets.QLabel("MOTORS ENABLED")
        label.setAlignment(QtCore.Qt.AlignCenter)
        label.setStyleSheet("""
            font-weight: normal;
            font-size: 16px;
            color: white;
        """)
        control_panel.addWidget(label)
        self.motor_buttons = []
        for i in range(3):
            btn = QtWidgets.QPushButton(f"Motor {i+1}")
            btn.setCheckable(True)
            btn.setChecked(True)
            btn.setMinimumHeight(40)
            btn.toggled.connect(self.update_button_color)
            self.motor_buttons.append(btn)
            btn.setStyleSheet("""
                QPushButton {
                    background-color: green;
                    color: white;
                    font-weight: normal;
                    border-radius: 5px;
                    padding: 10px;
                }
            """)
            control_panel.addWidget(btn)

        label = QtWidgets.QLabel("BEHAVIOR ENABLED")
        label.setAlignment(QtCore.Qt.AlignCenter)
        label.setStyleSheet("""
            font-weight: normal;
            font-size: 16px;
            color: white;
        """)

        control_panel.addWidget(label)
        self.behavior_buttons = []
        behavior_names = ["Manual Pos", "Sine Sweep"]
        self.behavior_group = QtWidgets.QButtonGroup()	# Adds both behavior buttons to a group
        self.behavior_group.setExclusive(True)			# Ensures only 1 button in the group can be active
        for name in behavior_names:
            btn = QtWidgets.QPushButton(name)
            btn.setCheckable(True)
            btn.setChecked(False)
            btn.setMinimumHeight(40)
            btn.setStyleSheet("""
                QPushButton {
                    background-color: darkred;
                    color: white;
                    font-weight: normal;
                    border-radius: 5px;
                    padding: 10px;
                }
            """)
            self.behavior_group.addButton(btn)
            self.behavior_buttons.append(btn)
            control_panel.addWidget(btn)
        self.behavior_group.buttonToggled.connect(self.handle_behavior_toggle)

        label = QtWidgets.QLabel("JUST GONNA SEND IT")
        label.setStyleSheet("""
            font-weight: normal;
            font-size: 16px;
            color: white;
        """)
        label.setAlignment(QtCore.Qt.AlignCenter)
        control_panel.addWidget(label)

        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(400)		# CHECK THIS LATER: WHY SO HIGH?
        self.slider.setValue(200)		# CHECK THIS LATER: WHY SO HIGH?
        control_panel.addWidget(self.slider)

        self.slider_label = QtWidgets.QLabel("Slider Value: 0.00")
        control_panel.addWidget(self.slider_label)
        self.slider.valueChanged.connect(self.update_slider_label)
        self.slider.setStyleSheet("""
            QSlider::groove:horizontal {
                border: 1px solid #bbb;
                background: #ccc;
                height: 15px;
                border-radius: 5px;
            }
            QSlider::sub-page:horizontal {
                background: green;
                height: 10px;
                border-radius: 5px;
            }
            QSlider::add-page:horizontal {
                background: grey;
                height: 10px;
                border-radius: 5px;
            }
            QSlider::handle:horizontal {
                background: white;
                border: 1px solid black;
                width: 30px;
                margin: -10px 0;
                border-radius: 10px;
            }
            """)

        self.send_button = QtWidgets.QPushButton("Activate")
        self.send_button.setCheckable(True)
        self.send_button.setChecked(False)
        self.send_button.setMinimumHeight(40)
        self.send_button.toggled.connect(self.handle_send_toggled)
        self.send_button.setStyleSheet("""
                QPushButton {
                    background-color: darkred;
                    color: white;
                    font-weight: normal;
                    border-radius: 5px;
                    padding: 10px;
                }
            """)
        control_panel.addWidget(self.send_button)

        self.home_button = QtWidgets.QPushButton("Home")
        self.home_button.setMinimumHeight(40)
        self.home_button.clicked.connect(self.handle_home)
        self.home_button.setStyleSheet("""
            QPushButton {
                background-color: darkorange;
                color: white;
                font-size: 15px;
                font-weight: normal;
                border-radius: 5px;
                padding: 10px;
            }
        """)
        control_panel.addWidget(self.home_button)

        self.exit_button = QtWidgets.QPushButton("EXIT")
        self.exit_button.setMinimumHeight(40)
        self.exit_button.clicked.connect(self.exit_application)
        self.exit_button.setStyleSheet("""
            QPushButton {
                background-color: red;
                color: white;
                font-size: 15px;
                font-weight: normal;
                border-radius: 5px;
                padding: 10px;
            }
        """)
        control_panel.addWidget(self.exit_button)

        self.start_time = time.time()
        self.history_sec = 10
        self.max_len = self.history_sec * 20
        self.time_vals = deque(maxlen=self.max_len)
        self.cmd_vals = deque(maxlen=self.max_len)
        self.fb_vals = [deque(maxlen=self.max_len)]
        self.last_cmd = 0.0
        self.last_fb = [0.0, 0.0, 0.0]

        self.plot_timer = QtCore.QTimer()
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.setSingleShot(False)

        self.cmd_timer = QtCore.QTimer()
        self.cmd_timer.timeout.connect(self.update_command)
        self.cmd_timer.setSingleShot(False)

#         self.showFullScreen()			# UNCOMMENT FOR FULL SCREEN CONTROL PANEL
        self.setEnabled(False)
        QtCore.QTimer.singleShot(100, self.start_homing_sequence)

    # Function to calculate motor position targets and send commands
    def update_command(self):
        if self.send_button.isChecked():
            motor_states = [int(btn.isChecked()) for btn in self.motor_buttons]
            motor_active = sum(motor_states)

            if self.behavior_buttons[0].isChecked():
                act_length_target = self.slider.value() / 100.0 - 2.0
                self.last_cmd = act_length_target
            elif self.behavior_buttons[1].isChecked():
                if not self.sine_sweep_active:
                    self.sine_start_time = time.time()
                    self.sine_sweep_active = True
                t = time.time() - self.sine_start_time
                freq = self.slider.value() / 100.0
                amp = 1.0
                act_length_target = amp * math.sin(2 * math.pi * freq * t)
                self.last_cmd = act_length_target
            else:
                act_length_target = 0.0

            if motor_active > 0:
                act_contribute = act_length_target / motor_active
                rev_targets = [
                    self.home_offsets[0] - motor_states[0] * act_contribute / Pitch_12,
                    self.home_offsets[1] - motor_states[1] * act_contribute / (Pitch_12 + Pitch_23),
                    self.home_offsets[2] + motor_states[2] * act_contribute / Pitch_23,
                ]
                for i in range(3):
                    send_position_command(rev_targets[i], self.motor_nodes[i])
        else:
            self.sine_sweep_active = False

        for i, node_id in enumerate(self.motor_nodes):
            pos = request_latest_feedback(node_id)
            if pos is not None:
                self.last_fb[i] = pos
                
    # Function to update the live plot
    def update_plot(self):
        t = time.time() - self.start_time
        self.time_vals.append(t)
        self.cmd_vals.append(self.last_cmd)

        rel_fb = [
            self.last_fb[0] - self.home_offsets[0],
            self.last_fb[1] - self.home_offsets[1],
            self.last_fb[2] - self.home_offsets[2],
        ]
        actual_length = (
            rel_fb[0] * Pitch_12 +
            rel_fb[1] * (Pitch_12 + Pitch_23) -
            rel_fb[2] * Pitch_23
        ) * (-1)
        self.fb_vals[0].append(actual_length)

        self.cmd_curve.setData(self.time_vals, self.cmd_vals)
        self.fb_curve.setData(self.time_vals, self.fb_vals[0])
        self.plot.setXRange(max(0, t - self.history_sec), t)

    # Function to handle styling of activate button
    def handle_send_toggled(self, checked):
        if not checked:
            self.send_button.setStyleSheet("""
                QPushButton {
                    background-color: darkred;
                    color: white;
                    font-weight: normal;
                    border-radius: 5px;
                    padding: 10px;
                }
            """)
            self.sine_sweep_active = False
        else:
            self.send_button.setStyleSheet("""
                QPushButton {
                    background-color: green;
                    color: white;
                    font-weight: normal;
                    border-radius: 5px;
                    padding: 10px;
                }
            """)
            
    # Function to send motors home
    def handle_home(self):
        self.send_button.setChecked(False)
        self.sine_sweep_active = False
        self.last_cmd = 0.0
        for i in range(3):
            send_position_command(self.home_offsets[i], self.motor_nodes[i])
            # Add stepping behavior in here to more slowly move motors to home
    
    # Function to update label above slider
    def update_slider_label(self):
        if self.behavior_buttons[0].isChecked():
            value = self.slider.value() / 100.0 - 2.0
            self.slider_label.setText(f"Position: {value:.2f}")
        elif self.behavior_buttons[1].isChecked():
            value = self.slider.value() / 100.0
            self.slider_label.setText(f"Frequency: {value:.2f} Hz")
    
    # Function to handle what happens when you switch behaviors
    def handle_behavior_toggle(self):
        if self.behavior_buttons[0].isChecked():
            self.slider.setMinimum(0)
            self.slider.setMaximum(400)
            self.slider.setValue(200)
            self.slider_label.setText("Position: 0.00")
            self.behavior_buttons[0].setStyleSheet("""
                QPushButton {
                    background-color: green;
                    color: white;
                    font-weight: normal;
                    border-radius: 5px;
                    padding: 10px;
                }
            """)
            self.behavior_buttons[1].setStyleSheet("""
                QPushButton {
                    background-color: darkred;
                    color: white;
                    font-weight: normal;
                    border-radius: 5px;
                    padding: 10px;
                }
            """)
        elif self.behavior_buttons[1].isChecked():
            self.slider.setMinimum(0)
            self.slider.setMaximum(300)
            self.slider.setValue(100)
            self.slider_label.setText("Frequency: 1.00 Hz")
            self.slider_label.setText("Position: 0.00")
            self.behavior_buttons[0].setStyleSheet("""
                QPushButton {
                    background-color: darkred;
                    color: white;
                    font-weight: normal;
                    border-radius: 5px;
                    padding: 10px;
                }
            """)
            self.behavior_buttons[1].setStyleSheet("""
                QPushButton {
                    background-color: green;
                    color: white;
                    font-weight: normal;
                    border-radius: 5px;
                    padding: 10px;
                }
            """)
            
    # Function to start the homing sequence
    def start_homing_sequence(self):
        self.status_box.setText("Homing motors...")
        self.homing_thread = HomingThread(self.motor_nodes)
        self.homing_thread.homing_done.connect(self.finish_homing_sequence)
        self.homing_thread.start()

    # Function to save home offsets and enable command/plotting timers
    def finish_homing_sequence(self, offsets):
        self.home_offsets = offsets
        self.status_box.setText("Manual control active.")
        self.setEnabled(True)
        self.cmd_timer.start(50)
        self.plot_timer.start(50)

    # Function to perform shut down and exit application
    def exit_application(self):
        if self.homing_thread and self.homing_thread.isRunning():
            self.homing_thread.interrupted = True
            self.homing_thread.wait()
        # Add something here to retract the actuator for travel
        for node in [0, 1, 2]:		# For all 3 controllers
            exit_closed_loop(node)	# Exit closed loop control
        QtWidgets.QApplication.quit()
        
    # Function to update the color of (some) latching pushbuttons
    def update_button_color(self, button, on_color="green", off_color="red"):
        if button.isChecked():
            button.setStyleSheet("""
                QPushButton {
                    background-color: {on_color};
                    color: white;
                    font-weight: normal;
                    border-radius: 5px;
                    padding: 10px;
                }
            """)
        else:
            button.setStyleSheet("""
                QPushButton {
                    background-color: {off_color};
                    color: white;
                    font-weight: normal;
                    border-radius: 5px;
                    padding: 10px;
                }
            """)
        pass
    
    # Function to exit if exc key is pressed
    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Escape:
            self.exit_application()

# === RUN IT! =================================================================
if __name__ == "__main__":					# Checks if script is being run directly, not as a module
#     for node in [0, 1, 2]:				# For all three motor controllers
#         enter_closed_loop(node)			# Enter closed loop control
    app = QtWidgets.QApplication(sys.argv)	# Creates QT application object
    window = ControlPanel()					# Instantiates the GUI class (ControlPanel)
    window.show()							# Displays the GUI window
    sys.exit(app.exec_())					# Enters Qt event loop ot start procevving GUI events
