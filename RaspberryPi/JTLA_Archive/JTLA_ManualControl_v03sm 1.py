# === Define libraries ===============================================================================
import sys
import time
import struct
import can
from PyQt5 import QtWidgets, QtCore

# === Define class to handle all of the communications between RPi & motor controllers ===============
class MotorController:
    def __init__(self, node_id, bus):
        self.node_id = node_id
        self.bus = bus
        self.current_position = 0.0

    def send_position(self, delta):
        self.current_position += delta
        vel_ff = 0
        torque_ff = 0
        data = struct.pack('<fhh', self.current_position, vel_ff, torque_ff)
        arb_id = (self.node_id << 5) | 0x0C
        msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
        self.bus.send(msg)

    def enter_closed_loop(self):
        arb_id = (self.node_id << 5) | 0x07
        msg = can.Message(arbitration_id=arb_id, data=struct.pack('<I', 8), is_extended_id=False)
        self.bus.send(msg)

    def exit_closed_loop(self):
        arb_id = (self.node_id << 5) | 0x07
        msg = can.Message(arbitration_id=arb_id, data=struct.pack('<I', 1), is_extended_id=False)
        self.bus.send(msg)

    def request_position(self):
        arb_id = (self.node_id << 5) | 0x09
        msg = can.Message(arbitration_id=arb_id, data=[], is_extended_id=False, is_remote_frame=True)
        self.bus.send(msg)
        start = time.time()
        while time.time() - start < 0.010:
            msg = self.bus.recv(timeout=0.008)
            if not msg:
                break
            if msg.arbitration_id == arb_id and not msg.is_remote_frame:
                position, _ = struct.unpack('<ff', msg.data)
                return position
        return None

# === Define class to handle GUI and general motor control =========================================
class ManualControlGUI(QtWidgets.QWidget):
    def __init__(self, motor_controllers):
        super().__init__()
        self.motor_controllers = motor_controllers
        self.position_labels = []  # ⬅️ Store label references
        self.init_ui()

        # Timer for updating motor positions
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_positions)
        self.timer.start(100)  # ms

    def init_ui(self):
        layout = QtWidgets.QVBoxLayout()

        self.loop_button = QtWidgets.QPushButton("Motors Disabled")
        self.loop_button.setCheckable(True)
        self.loop_button.setChecked(False)
        self.loop_button.setMinimumHeight(50)
        self.loop_button.setStyleSheet("""QPushButton {background-color: darkred; color: white; font-weight: bold; font-size: 20px; border-radius: 5px; padding: 10px; }""")
        self.loop_button.clicked.connect(self.toggle_closed_loop)
        layout.addWidget(self.loop_button)

        for motor in self.motor_controllers:
            hbox = QtWidgets.QHBoxLayout()

            label = QtWidgets.QLabel(f"    Motor {motor.node_id + 1}    ")
            label.setStyleSheet("""font-weight: bold; font-size: 24px; color: white;""")
            pos_display = QtWidgets.QLabel("Position: ---")  # ⬅️ Text display for position
            pos_display.setStyleSheet("background: black; color: white; font-weight: bold; font-size: 20px; border-radius: 5px; padding: 10px;")
            btn_forward = QtWidgets.QPushButton("Step +")
            btn_forward.setStyleSheet("""QPushButton {background-color: green; color: white; font-weight: bold; font-size: 20px; border-radius: 5px; padding: 10px; }""")
            btn_backward = QtWidgets.QPushButton("Step -")
            btn_backward.setStyleSheet("""QPushButton {background-color: darkred; color: white; font-weight: bold; font-size: 20px; border-radius: 5px; padding: 10px; }""")

            btn_forward.clicked.connect(lambda _, m=motor: m.send_position(+0.5))
            btn_backward.clicked.connect(lambda _, m=motor: m.send_position(-0.5))

            hbox.addWidget(label)
            hbox.addWidget(pos_display)  # ⬅️ Add display to layout
            hbox.addWidget(btn_backward)
            hbox.addWidget(btn_forward)

            self.position_labels.append((motor, pos_display))  # ⬅️ Save for updates
            layout.addLayout(hbox)

        self.setLayout(layout)
        self.setWindowTitle("JTLA Manual Motor Control")
        self.show()

    def update_positions(self):
        for motor, label in self.position_labels:
            pos = motor.request_position()
            if pos is not None:
                label.setText(f"Position: {pos:.2f}")
            else:
                label.setText("Position: ---")
                
    def toggle_closed_loop(self):
        if self.loop_button.isChecked():
            self.loop_button.setText("Motors Enabled")
            self.loop_button.setStyleSheet("""QPushButton {background-color: green; color: white; font-weight: bold; font-size: 20px; border-radius: 5px; padding: 10px; }""")
            for m in self.motor_controllers:
                m.enter_closed_loop()
        else:
            self.loop_button.setText("Motors Disabled")
            self.loop_button.setStyleSheet("""QPushButton {background-color: darkred; color: white; font-weight: bold; font-size: 20px; border-radius: 5px; padding: 10px; }""")
            for m in self.motor_controllers:
                m.exit_closed_loop()


# === SEND IT ==============================================================================
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)

    bus = can.interface.Bus("can0", interface="socketcan")
    motor_nodes = [0, 1, 2]
    motor_controllers = [MotorController(node_id, bus) for node_id in motor_nodes]

    gui = ManualControlGUI(motor_controllers)
    sys.exit(app.exec_())
