import sys
import time
import math
import struct
import can
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
from collections import deque

class LivePlotSineSweep(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ODrive Sine Sweep Live Plot")
        self.setGeometry(100, 100, 1200, 600)

        # Plot setup
        self.plot = pg.PlotWidget(title="Commanded vs Actual Position")
        self.plot.setYRange(-4, 4)
        self.curve_cmd = self.plot.plot(pen='y', name='Commanded')
        self.curve_act = self.plot.plot(pen='c', name='Actual')
        self.plot.addLegend(offset=(-10, 10))
        self.setCentralWidget(self.plot)

        # Font scaling
        font = QtWidgets.QApplication.font()
        font.setPointSize(font.pointSize() * 2)
        self.plot.getAxis("left").setTickFont(font)
        self.plot.getAxis("bottom").setTickFont(font)
        self.plot.setLabel('left', 'Position (rev)', **{'font-size': f'{font.pointSize()}pt'})
        self.plot.setLabel('bottom', 'Time (s)', **{'font-size': f'{font.pointSize()}pt'})

        # Data buffers
        self.x_vals = deque(maxlen=1000)
        self.cmd_vals = deque(maxlen=1000)
        self.act_vals = deque(maxlen=1000)

        # Timer for updating plot
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(50)

        # CAN + Sweep Params
        self.bus = can.interface.Bus(channel="can0", interface="socketcan")
        self.node_id = 0
        self.feedback_started = False
        self.t0 = None

        self.amplitude = 3.0
        self.frequency = 0.5
        self.duration = 10.0

        self.init_can_and_wait()

    def init_can_and_wait(self):
        print("Clearing CAN buffer...")
        while self.bus.recv(timeout=0):
            pass

        print("Sending Closed Loop Control command...")
        self.send_can((self.node_id << 5) | 0x07, struct.pack('<I', 8))

        print("Waiting for heartbeat...")
        while True:
            msg = self.bus.recv(timeout=1.0)
            if msg and msg.arbitration_id == ((self.node_id << 5) | 0x01):
                _, state, _, _ = struct.unpack('<IBBB', msg.data[:7])
                if state == 8:
                    print("Heartbeat received: Axis in closed loop control.")
                    break

        print("Waiting for encoder feedback...")
        while True:
            msg = self.bus.recv(timeout=1.0)
            if msg and msg.arbitration_id == ((self.node_id << 5) | 0x09):
                print("Encoder feedback received. Starting sine sweep and plot.")
                self.feedback_started = True
                self.t0 = time.time()
                break

    def send_can(self, arb_id, data):
        msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
        self.bus.send(msg)

    def update_plot(self):
        if not self.feedback_started or self.t0 is None:
            return

        t = time.time() - self.t0
        if t > self.duration:
            return

        pos_cmd = self.amplitude * math.sin(2 * math.pi * self.frequency * t)
        vel_ff = 2 * math.pi * self.frequency * self.amplitude * math.cos(2 * math.pi * self.frequency * t)

        # Only send 8 bytes: position + velocity feedforward
        data = struct.pack('<ff', pos_cmd, vel_ff)
        self.send_can((self.node_id << 5) | 0x0C, data)

        # Try reading encoder feedback
        pos_feedback = None
        msg = self.bus.recv(timeout=0.01)
        if msg and msg.arbitration_id == ((self.node_id << 5) | 0x09):
            pos_feedback, _ = struct.unpack('<ff', msg.data)

        # Plotting
        self.x_vals.append(t)
        self.cmd_vals.append(pos_cmd)
        self.act_vals.append(pos_feedback if pos_feedback is not None else 0.0)

        self.curve_cmd.setData(self.x_vals, self.cmd_vals)
        self.curve_act.setData(self.x_vals, self.act_vals)
        self.plot.setXRange(max(0, t - 10), t)

# Run it
app = QtWidgets.QApplication(sys.argv)
window = LivePlotSineSweep()
window.show()
sys.exit(app.exec_())
