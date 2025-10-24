# [START OF SCRIPT]
import sys
import time
import math
import struct
import can
from collections import deque
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

# === CONFIGURATION ===
bus = can.interface.Bus("can0", interface="socketcan")
Pitch_12 = 0.157
Pitch_23 = 0.197

class HomingThread(QtCore.QThread):
    homing_done = QtCore.pyqtSignal(list)

    def __init__(self, motor_nodes):
        super().__init__()
        self.motor_nodes = motor_nodes
        self.home_offsets = [0.0, 0.0, 0.0]
        self.interrupted = False

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

def send_position_command(position, node_id):
    vel_ff = 0
    torque_ff = 0
    data = struct.pack('<fhh', position, vel_ff, torque_ff)
    arb_id = (node_id << 5) | 0x0C
    msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
    bus.send(msg)

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

class ControlPanel(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Motor Control Panel + Live Plot")
        self.sine_sweep_active = False
        self.sine_start_time = 0
        self.motor_nodes = [0, 1, 2]
        self.home_offsets = [0.0, 0.0, 0.0]
        self.homing_thread = None

        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QHBoxLayout()
        central_widget.setLayout(main_layout)

        self.plot = pg.PlotWidget(title="Commanded vs Actual Actuator Position")
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
        control_panel.addWidget(label)
        self.motor_buttons = []
        for i in range(3):
            btn = QtWidgets.QPushButton(f"Motor {i+1}")
            btn.setCheckable(True)
            btn.setMinimumHeight(40)
            btn.toggled.connect(self.update_motor_button_color)
            self.motor_buttons.append(btn)
            control_panel.addWidget(btn)

        label = QtWidgets.QLabel("BEHAVIOR ENABLED")
        label.setAlignment(QtCore.Qt.AlignCenter)
        control_panel.addWidget(label)
        self.behavior_buttons = []
        behavior_names = ["Manual Pos", "Sine Sweep"]
        self.behavior_group = QtWidgets.QButtonGroup()
        self.behavior_group.setExclusive(True)
        for name in behavior_names:
            btn = QtWidgets.QPushButton(name)
            btn.setCheckable(True)
            btn.setMinimumHeight(40)
            self.behavior_group.addButton(btn)
            self.behavior_buttons.append(btn)
            control_panel.addWidget(btn)
        self.behavior_group.buttonToggled.connect(self.update_slider_behavior)

        label = QtWidgets.QLabel("OPERATE")
        label.setAlignment(QtCore.Qt.AlignCenter)
        control_panel.addWidget(label)
        self.send_button = QtWidgets.QPushButton("Activate")
        self.send_button.setCheckable(True)
        self.send_button.setMinimumHeight(40)
        self.send_button.toggled.connect(self.handle_send_toggled)
        control_panel.addWidget(self.send_button)

        self.home_button = QtWidgets.QPushButton("Home")
        self.home_button.setMinimumHeight(40)
        self.home_button.clicked.connect(self.handle_home)
        control_panel.addWidget(self.home_button)

        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(400)
        self.slider.setValue(200)
        control_panel.addWidget(self.slider)

        self.slider_label = QtWidgets.QLabel("Slider Value: 0.00")
        control_panel.addWidget(self.slider_label)
        self.slider.valueChanged.connect(self.update_slider_label)

        self.exit_button = QtWidgets.QPushButton("EXIT")
        self.exit_button.setMinimumHeight(40)
        self.exit_button.clicked.connect(self.exit_application)
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

        self.showFullScreen()
        self.setEnabled(False)
        QtCore.QTimer.singleShot(100, self.start_homing_sequence)

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

    def handle_send_toggled(self, checked):
        if not checked:
            self.sine_sweep_active = False

    def handle_home(self):
        self.send_button.setChecked(False)
        self.sine_sweep_active = False
        self.last_cmd = 0.0
        for i in range(3):
            send_position_command(self.home_offsets[i], self.motor_nodes[i])
            # Add stepping behavior in here to more slowly move motors to home

    def update_slider_label(self):
        if self.behavior_buttons[0].isChecked():
            value = self.slider.value() / 100.0 - 2.0
            self.slider_label.setText(f"Position: {value:.2f}")
        elif self.behavior_buttons[1].isChecked():
            value = self.slider.value() / 100.0
            self.slider_label.setText(f"Frequency: {value:.2f} Hz")

    def update_slider_behavior(self):
        if self.behavior_buttons[0].isChecked():
            self.slider.setMinimum(0)
            self.slider.setMaximum(400)
            self.slider.setValue(200)
            self.slider_label.setText("Position: 0.00")
        elif self.behavior_buttons[1].isChecked():
            self.slider.setMinimum(0)
            self.slider.setMaximum(300)
            self.slider.setValue(100)
            self.slider_label.setText("Frequency: 1.00 Hz")

    def start_homing_sequence(self):
        self.status_box.setText("Homing motors...")
        self.homing_thread = HomingThread(self.motor_nodes)
        self.homing_thread.homing_done.connect(self.finish_homing_sequence)
        self.homing_thread.start()

    def finish_homing_sequence(self, offsets):
        self.home_offsets = offsets
        self.status_box.setText("Manual control active.")
        self.setEnabled(True)
        self.cmd_timer.start(50)
        self.plot_timer.start(50)


    def exit_application(self):
        if self.homing_thread and self.homing_thread.isRunning():
            self.homing_thread.interrupted = True
            self.homing_thread.wait()
        QtWidgets.QApplication.quit()
        # Add something in here to retract the actuator for travel

    def update_motor_button_color(self):
        pass

    def keyPressEvent(self, event):
        if event.key() == QtCore.Qt.Key_Escape:
            self.exit_application()

if __name__ == "__main__":
    for node in [0, 1, 2]:
        enter_closed_loop(node)
    app = QtWidgets.QApplication(sys.argv)
    window = ControlPanel()
    window.show()
    sys.exit(app.exec_())
# [END OF SCRIPT]
