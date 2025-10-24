import sys
import time
import math
import struct
import can
from collections import deque
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

# === CONFIGURATION ===
node_id = 0  # ODrive CAN Node ID
bus = can.interface.Bus("can0", interface="socketcan")

# === CAN COMMAND: Set_Input_Pos ===
def send_position_command(position):
    vel_ff = 0
    torque_ff = 0
    data = struct.pack('<fhh', position, vel_ff, torque_ff)
    arb_id = (node_id << 5) | 0x0C
    msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
    bus.send(msg)

# === Set Axis State to CLOSED_LOOP_CONTROL ===
def enter_closed_loop():
    print("Requesting CLOSED_LOOP_CONTROL...")
    arb_id_set_state = (node_id << 5) | 0x07
    msg = can.Message(arbitration_id=arb_id_set_state, data=struct.pack('<I', 8), is_extended_id=False)
    bus.send(msg)

    # Wait for confirmation via Heartbeat
    while True:
        msg = bus.recv(timeout=1.0)
        if msg and msg.arbitration_id == ((node_id << 5) | 0x01):
            _, axis_state, _, _ = struct.unpack('<IBBB', msg.data[:7])
            if axis_state == 8:
                print("Axis is in CLOSED_LOOP_CONTROL.")
                break

# === GUI with Live Plot ===
class PlotWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ODrive Sine Sweep Live Plot")
        self.setGeometry(100, 100, 1200, 600)

        # Layout
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        layout = QtWidgets.QVBoxLayout()
        central_widget.setLayout(layout)

        # Plot
        self.plot = pg.PlotWidget(title="Commanded vs Feedback Position")
        self.plot.setYRange(-4, 4)
        self.cmd_curve = self.plot.plot(pen='y', name="Commanded")
        self.fb_curve = self.plot.plot(pen='c', name="Feedback")
        self.plot.addLegend(offset=(-10, 10))
        layout.addWidget(self.plot)

        # Time series buffers
        self.start_time = time.time()
        self.history_sec = 10
        self.max_len = self.history_sec * 20
        self.time_vals = deque(maxlen=self.max_len)
        self.cmd_vals = deque(maxlen=self.max_len)
        self.fb_vals = deque(maxlen=self.max_len)
        self.last_cmd = 0.0
        self.last_fb = 0.0

        # Start timers
        self.plot_timer = QtCore.QTimer()
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.start(50)

        self.cmd_timer = QtCore.QTimer()
        self.cmd_timer.timeout.connect(self.sine_sweep_step)
        self.cmd_timer.start(50)

    def sine_sweep_step(self):
        t = time.time() - self.start_time
        pos_cmd = 3.0 * math.sin(2 * math.pi * 0.5 * t)
        self.last_cmd = pos_cmd
        send_position_command(pos_cmd)

        # Read encoder estimates (if streaming)
        msg = bus.recv(timeout=0.01)
        if msg and msg.arbitration_id == ((node_id << 5) | 0x09):
            pos, _ = struct.unpack('<ff', msg.data)
            self.last_fb = pos

    def update_plot(self):
        t = time.time() - self.start_time
        self.time_vals.append(t)
        self.cmd_vals.append(self.last_cmd)
        self.fb_vals.append(self.last_fb)
        self.cmd_curve.setData(list(self.time_vals), list(self.cmd_vals))
        self.fb_curve.setData(list(self.time_vals), list(self.fb_vals))
        self.plot.setXRange(max(0, t - self.history_sec), t)

# === MAIN ===
if __name__ == "__main__":
    enter_closed_loop()
    app = QtWidgets.QApplication(sys.argv)
    window = PlotWindow()
    window.show()
    sys.exit(app.exec_())
