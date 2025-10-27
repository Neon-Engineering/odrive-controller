import sys
import time
import math
import struct
import can
from collections import deque
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

# === CONFIGURATION ===
node_id = 0
bus = can.interface.Bus("can0", interface="socketcan")

def send_position_command(position):
    vel_ff = 0
    torque_ff = 0
    data = struct.pack('<fhh', position, vel_ff, torque_ff)
    arb_id = (node_id << 5) | 0x0C
    msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
    bus.send(msg)

def request_latest_feedback():
    latest_pos = None

    # Properly send remote frame (no payload, remote request)
    arb_id = (node_id << 5) | 0x09
    msg = can.Message(arbitration_id=arb_id,
                      data=[],
                      is_extended_id=False,
                      is_remote_frame=True)
    bus.send(msg)

    # Drain for up to 2 ms
    start = time.time()
    while time.time() - start < 0.002:
        msg = bus.recv(timeout=0.001)
        if not msg:
            break
        if msg.arbitration_id == arb_id and not msg.is_remote_frame:
            latest_pos, _ = struct.unpack('<ff', msg.data)
    return latest_pos

def enter_closed_loop():
    print("Requesting CLOSED_LOOP_CONTROL...")
    arb_id_set_state = (node_id << 5) | 0x07
    msg = can.Message(arbitration_id=arb_id_set_state, data=struct.pack('<I', 8), is_extended_id=False)
    bus.send(msg)

    while True:
        msg = bus.recv(timeout=1.0)
        if msg and msg.arbitration_id == ((node_id << 5) | 0x01):
            _, axis_state, _, _ = struct.unpack('<IBBB', msg.data[:7])
            if axis_state == 8:
                print("Axis is in CLOSED_LOOP_CONTROL.")
                break

class ControlPanel(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Motor Control Panel + Live Plot")
        self.setGeometry(100, 100, 1400, 700)

        # === State ===
        self.sine_sweep_active = False
        self.sine_start_time = 0

        # === Layout ===
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QHBoxLayout()
        central_widget.setLayout(main_layout)

        # === Plot ===
        self.plot = pg.PlotWidget(title="Commanded vs Feedback Position")
        self.plot.setYRange(-4, 4)
        self.cmd_curve = self.plot.plot(pen='y', name="Commanded")
        self.fb_curve = self.plot.plot(pen='c', name="Feedback")
        self.plot.addLegend(offset=(-10, 10))
        main_layout.addWidget(self.plot, stretch=4)

        # === Control Panel ===
        control_layout = QtWidgets.QVBoxLayout()
        main_layout.addLayout(control_layout, stretch=1)

        self.motor_buttons = []
        for i in range(3):
            btn = QtWidgets.QPushButton(f"Motor {i+1}")
            btn.setCheckable(True)
            btn.setStyleSheet("background-color: red")
            btn.toggled.connect(self.update_motor_button_color)
            self.motor_buttons.append(btn)
            control_layout.addWidget(btn)

        control_layout.addSpacing(20)

        self.behavior_buttons = []
        behavior_names = ["DISCRETE", "SINE SWEEP"]
        self.behavior_group = QtWidgets.QButtonGroup()
        self.behavior_group.setExclusive(True)
        for name in behavior_names:
            btn = QtWidgets.QPushButton(name)
            btn.setCheckable(True)
            btn.setStyleSheet("background-color: lightgray")
            self.behavior_group.addButton(btn)
            self.behavior_buttons.append(btn)
            control_layout.addWidget(btn)

        control_layout.addSpacing(20)

        self.send_button = QtWidgets.QPushButton("ACTIVATE")
        self.send_button.setCheckable(True)
        self.send_button.setStyleSheet("background-color: gold")
        self.send_button.toggled.connect(self.handle_send_toggled)
        control_layout.addWidget(self.send_button)

        self.home_button = QtWidgets.QPushButton("HOME")
        self.home_button.setStyleSheet("background-color: lightblue")
        self.home_button.clicked.connect(self.handle_home)
        control_layout.addWidget(self.home_button)

        control_layout.addSpacing(20)

        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(500)
        self.slider.setValue(250)
        control_layout.addWidget(self.slider)

        self.slider_label = QtWidgets.QLabel("Slider Value: 2.50")
        control_layout.addWidget(self.slider_label)
        self.slider.valueChanged.connect(self.update_slider_label)

        # === Buffers ===
        self.start_time = time.time()
        self.history_sec = 10
        self.max_len = self.history_sec * 20
        self.time_vals = deque(maxlen=self.max_len)
        self.cmd_vals = deque(maxlen=self.max_len)
        self.fb_vals = deque(maxlen=self.max_len)
        self.last_cmd = 0.0
        self.last_fb = 0.0

        # === Timers ===
        self.cmd_timer = QtCore.QTimer()
        self.cmd_timer.timeout.connect(self.update_command)
        self.cmd_timer.start(50)

        self.plot_timer = QtCore.QTimer()
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.start(50)

    def update_motor_button_color(self):
        for btn in self.motor_buttons:
            if btn.isChecked():
                btn.setStyleSheet("background-color: green")
            else:
                btn.setStyleSheet("background-color: red")

    def update_slider_label(self):
        value = self.slider.value() / 100.0
        self.slider_label.setText(f"Slider Value: {value:.2f}")

    def handle_send_toggled(self, checked):
        if not checked:
            self.sine_sweep_active = False

    def handle_home(self):
        self.send_button.setChecked(False)
        self.sine_sweep_active = False
        self.last_cmd = 0.0
        send_position_command(0.0)

    def update_command(self):
        if self.send_button.isChecked():
            if self.behavior_buttons[0].isChecked():
                # Behavior 1: Position control
                position = self.slider.value() / 100.0
                self.last_cmd = position
                send_position_command(position)

            elif self.behavior_buttons[1].isChecked():
                # Behavior 2: Sine sweep
                if not self.sine_sweep_active:
                    self.sine_start_time = time.time()
                    self.sine_sweep_active = True

                t = time.time() - self.sine_start_time
                freq = self.slider.value() / 100.0
                amp = 2.0
                pos_cmd = amp * math.sin(2 * math.pi * freq * t)
                self.last_cmd = pos_cmd
                send_position_command(pos_cmd)
        else:
            self.sine_sweep_active = False

        # Always poll for latest feedback
        pos = request_latest_feedback()
        if pos is not None:
            self.last_fb = pos

    def update_plot(self):
        t = time.time() - self.start_time
        self.time_vals.append(t)
        self.cmd_vals.append(self.last_cmd)
        self.fb_vals.append(self.last_fb)
        self.cmd_curve.setData(list(self.time_vals), list(self.cmd_vals))
        self.fb_curve.setData(list(self.time_vals), list(self.fb_vals))
        self.plot.setXRange(max(0, t - self.history_sec), t)

if __name__ == "__main__":
    enter_closed_loop()
    app = QtWidgets.QApplication(sys.argv)
    window = ControlPanel()
    window.show()
    sys.exit(app.exec_())
