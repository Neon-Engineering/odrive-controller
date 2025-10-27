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
            delay_step = 0.25
            backoff_dist = 3.9 / pitch
            pos = request_latest_feedback(node_id)
            if pos is None:
                pos = 0.0
            while not self.interrupted:
                send_position_command(pos + step_size, node_id)
                time.sleep(delay_step)
                new_pos = request_latest_feedback(node_id)
                if new_pos is None:
                    new_pos = pos
                if abs(new_pos - pos) < 0.05:
                    hardstop = new_pos
                    home_pos = hardstop - direction * backoff_dist
                    for j in range(21):
                        if self.interrupted:
                            return
                        interp_pos = hardstop + (home_pos - hardstop) * (j / 20)
                        send_position_command(interp_pos, node_id)
                        time.sleep(0.1)
                    time.sleep(3.0)
                    return home_pos
                pos = new_pos

        self.home_offsets[0] = home_motor(0, self.motor_nodes[0], +1, Pitch_12)
        # Uncomment if motor 3 homing is stable
        # self.home_offsets[2] = home_motor(2, self.motor_nodes[2], -1, Pitch_23)
        if not self.interrupted:
            self.home_offsets[2] = request_latest_feedback(self.motor_nodes[2])
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
    while time.time() - start < 0.014:
        msg = bus.recv(timeout=0.012)
        if msg and msg.arbitration_id == arb_id and not msg.is_remote_frame:
            latest_pos, _ = struct.unpack('<ff', msg.data)
    return latest_pos

def enter_closed_loop(node_id):
    arb_id = (node_id << 5) | 0x07
    msg = can.Message(arbitration_id=arb_id, data=struct.pack('<I', 8), is_extended_id=False)
    bus.send(msg)
    while True:
        msg = bus.recv(timeout=1.0)
        if msg and msg.arbitration_id == ((node_id << 5) | 0x01):
            _, state, _, _ = struct.unpack('<IBBB', msg.data[:7])
            if state == 8:
                break
            
class ControlPanel(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Actuator Control GUI + Dual Plot")

        self.motor_nodes = [0, 1, 2]
        self.home_offsets = [0.0, 0.0, 0.0]
        self.last_fb = [0.0, 0.0, 0.0]
        self.last_cmd = 0.0
        self.sine_sweep_active = False
        self.homing_thread = None

        # === GUI Layout ===
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QHBoxLayout()
        central_widget.setLayout(main_layout)

        # === Left: Dual Plots ===
        plot_column = QtWidgets.QVBoxLayout()
        self.plot_cmd_actual = pg.PlotWidget(title="Commanded vs Actual Position")
        self.plot_cmd_actual.setYRange(-15, 15)
        self.plot_cmd_actual.showGrid(x=False, y=True, alpha=0.3)
        self.cmd_curve = self.plot_cmd_actual.plot(pen='y', name="Commanded")
        self.act_curve = self.plot_cmd_actual.plot(pen='w', name="Actual")

        self.plot_motor_fb = pg.PlotWidget(title="Motor Feedback Positions")
        self.plot_motor_fb.setYRange(-15, 15)
        self.plot_motor_fb.showGrid(x=False, y=True, alpha=0.3)
        self.plot_motor_fb.addLegend()
        self.fb_curves = [
            self.plot_motor_fb.plot(pen=pg.mkPen(color=(255, 105, 180)), name="Motor 1"),
            self.plot_motor_fb.plot(pen=pg.mkPen(color=(255, 165, 0)), name="Motor 2"),
            self.plot_motor_fb.plot(pen=pg.mkPen(color=(0, 255, 0)), name="Motor 3"),
        ]

        plot_column.addWidget(self.plot_cmd_actual)
        plot_column.addWidget(self.plot_motor_fb)
        main_layout.addLayout(plot_column, stretch=5)

        # === Right: GUI Controls ===
        control_panel = QtWidgets.QVBoxLayout()
        main_layout.addLayout(control_panel, stretch=1)

        self.status_box = QtWidgets.QLabel("Initializing...")
        self.status_box.setStyleSheet("background: black; color: white; font-size: 18px;")
        self.status_box.setAlignment(QtCore.Qt.AlignCenter)
        control_panel.addWidget(self.status_box)

        label = QtWidgets.QLabel("MOTORS ACTIVE")
        label.setAlignment(QtCore.Qt.AlignCenter)
        label.setStyleSheet("font-weight: bold;")
        control_panel.addWidget(label)

        self.motor_buttons = []
        for i in range(3):
            btn = QtWidgets.QPushButton(f"Motor {i+1}")
            btn.setCheckable(True)
            btn.setMinimumHeight(50)
            control_panel.addWidget(btn)
            self.motor_buttons.append(btn)

        label = QtWidgets.QLabel("BEHAVIOR ACTIVE")
        label.setAlignment(QtCore.Qt.AlignCenter)
        label.setStyleSheet("font-weight: bold;")
        control_panel.addWidget(label)

        self.behavior_buttons = []
        self.behavior_group = QtWidgets.QButtonGroup()
        for name in ["Manual Pos", "Sine Sweep"]:
            btn = QtWidgets.QPushButton(name)
            btn.setCheckable(True)
            btn.setMinimumHeight(50)
            self.behavior_group.addButton(btn)
            self.behavior_buttons.append(btn)
            control_panel.addWidget(btn)

        label = QtWidgets.QLabel("CONFIGURE")
        label.setAlignment(QtCore.Qt.AlignCenter)
        label.setStyleSheet("font-weight: bold;")
        control_panel.addWidget(label)

        self.send_button = QtWidgets.QPushButton("SEND")
        self.send_button.setCheckable(True)
        self.send_button.setMinimumHeight(50)
        control_panel.addWidget(self.send_button)

        self.home_button = QtWidgets.QPushButton("HOME")
        self.home_button.setMinimumHeight(50)
        control_panel.addWidget(self.home_button)

        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(400)
        self.slider.setValue(200)
        control_panel.addWidget(self.slider)

        self.slider_label = QtWidgets.QLabel("Slider Value: 0.00")
        control_panel.addWidget(self.slider_label)

        self.exit_button = QtWidgets.QPushButton("EXIT")
        self.exit_button.setMinimumHeight(50)
        self.exit_button.clicked.connect(QtWidgets.QApplication.quit)
        control_panel.addWidget(self.exit_button)

        self.slider.valueChanged.connect(self.update_slider_label)
        self.send_button.toggled.connect(self.handle_send_toggled)

        self.time_vals = deque(maxlen=200)
        self.cmd_vals = deque(maxlen=200)
        self.act_vals = deque(maxlen=200)
        self.fb_vals = [deque(maxlen=200) for _ in range(3)]

        self.plot_timer = QtCore.QTimer()
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.start(50)

        self.cmd_timer = QtCore.QTimer()
        self.cmd_timer.timeout.connect(self.update_command)
        self.cmd_timer.start(50)

        self.showFullScreen()
        self.setEnabled(False)
        QtCore.QTimer.singleShot(100, self.start_homing_sequence)

    def update_slider_label(self):
        mode = self.behavior_group.checkedButton()
        if mode and mode.text() == "Sine Sweep":
            val = self.slider.value() / 100.0
            self.slider_label.setText(f"Frequency: {val:.2f} Hz")
        else:
            val = self.slider.value() / 100.0 - 2.0
            self.slider_label.setText(f"Position: {val:.2f}")

    def handle_send_toggled(self, checked):
        self.sine_sweep_active = False if not checked else self.sine_sweep_active

    def start_homing_sequence(self):
        self.status_box.setText("Homing motors...")
        self.homing_thread = HomingThread(self.motor_nodes)
        self.homing_thread.homing_done.connect(self.finish_homing_sequence)
        self.homing_thread.start()

    def finish_homing_sequence(self, offsets):
        self.home_offsets = offsets
        self.status_box.setText("Manual control active.")
        self.setEnabled(True)

    def update_command(self):
        motor_states = [int(btn.isChecked()) for btn in self.motor_buttons]
        motor_active = sum(motor_states)
        if motor_active == 0:
            return

        if self.behavior_buttons[1].isChecked():
            if not self.sine_sweep_active:
                self.sine_start_time = time.time()
                self.sine_sweep_active = True
            t = time.time() - self.sine_start_time
            freq = self.slider.value() / 100.0
            amp = 2.0
            act_length_target = amp * math.sin(2 * math.pi * freq * t)
        else:
            act_length_target = self.slider.value() / 100.0 - 2.0

        self.last_cmd = act_length_target
        act_contribute = act_length_target / motor_active
        rev_targets = [
            self.home_offsets[0] - motor_states[0] * act_contribute / Pitch_12,
            self.home_offsets[1] - motor_states[1] * act_contribute / (Pitch_12 + Pitch_23),
            self.home_offsets[2] + motor_states[2] * act_contribute / Pitch_23,
        ]
        for i in range(3):
            send_position_command(rev_targets[i], self.motor_nodes[i])

        for i, node_id in enumerate(self.motor_nodes):
            pos = request_latest_feedback(node_id)
            if pos is not None:
                self.last_fb[i] = pos

    def update_plot(self):
        t = time.time()
        self.time_vals.append(t)
        self.cmd_vals.append(self.last_cmd)

        # Actual position from relative motor feedback
        m1 = self.last_fb[0] - self.home_offsets[0]
        m2 = self.last_fb[1] - self.home_offsets[1]
        m3 = self.last_fb[2] - self.home_offsets[2]
        actual_pos = (m1 + m2) * Pitch_12 + (m2 + m3) * Pitch_23
        self.act_vals.append(actual_pos)

        self.cmd_curve.setData(list(self.time_vals), list(self.cmd_vals))
        self.act_curve.setData(list(self.time_vals), list(self.act_vals))

        for i in range(3):
            fb_rel = self.last_fb[i] - self.home_offsets[i]
            if i == 2:
                fb_rel *= -1
            self.fb_vals[i].append(fb_rel)
            self.fb_curves[i].setData(list(self.time_vals), list(self.fb_vals[i]))

        self.plot_cmd_actual.setXRange(max(0, t - 10), t)
        self.plot_motor_fb.setXRange(max(0, t - 10), t)

if __name__ == "__main__":
    for node in [0, 1, 2]:
        enter_closed_loop(node)
    app = QtWidgets.QApplication(sys.argv)
    window = ControlPanel()
    window.show()
    sys.exit(app.exec_())
