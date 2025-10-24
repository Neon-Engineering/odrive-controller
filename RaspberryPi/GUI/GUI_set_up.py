# Not finished GUI for JTLA DEMO
import sys  
import time
from collections import deque
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg

# === Define the control panel for live plotting and GUI ======================================================================
class ControlPanel(QtWidgets.QMainWindow):
    # Function to set up the control panel
    def __init__(self):
        super().__init__()
        self.setWindowTitle("JTLA DEMO")
        self.sine_sweep_active = False
        self.sine_start_time = 0					
        self.motor_nodes = [0, 1, 2]			# Define ODrive node IDs
        self.home_offsets = [0.0, 0.0, 0.0]		# Initialize motor offsets
        self.data_log = []						# Initialize array to store data for CSV
        self.travel_lim = 2.70					# Allowable travel from HOME (soft stops)
        self.homing_thread = None

        # Main layout ------------------------
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QHBoxLayout()
        central_widget.setLayout(main_layout)

        # Plot title --------------------------------------------
        self.plot = pg.PlotWidget(title="Actuator Tip Position")
        self.plot.setYRange(-2.5, 2.5)
        self.plot.showGrid(x=False, y=True, alpha=0.3)
        self.plot.addLegend(offset=(-10, 10))
        zero_line = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen(color='grey', width=1))	# Thicker line along y=0 on plot for readability
        self.plot.addItem(zero_line)
        self.cmd_curve = self.plot.plot(pen=pg.mkPen('g',width=3), name="Commanded Position")
        self.fb_curve = self.plot.plot(pen=pg.mkPen('r',width=3), name="Actual Position")
        main_layout.addWidget(self.plot, stretch=5)

        control_panel = QtWidgets.QVBoxLayout()
        main_layout.addLayout(control_panel, stretch=1)

        # Status box -----------------------------------------
        self.status_box = QtWidgets.QLabel("Initializing...")
        self.status_box.setStyleSheet("background: black; color: white; font-size: 24px;")
        self.status_box.setAlignment(QtCore.Qt.AlignCenter)
        control_panel.addWidget(self.status_box)

        # Motor title label -----------------------
        label = QtWidgets.QLabel("MOTORS ENABLED")
        label.setAlignment(QtCore.Qt.AlignCenter)
        label.setStyleSheet("""font-weight: bold; font-size: 24px; color: white;""")
        control_panel.addWidget(label)
        
        # Motor buttons ---------------------------------
        self.motor_buttons = []
        for i in range(3):
            btn = QtWidgets.QPushButton(f"Motor {i+1}")
            btn.setCheckable(True)
            btn.setChecked(True)
            btn.setMinimumHeight(40)
            btn.toggled.connect(self.update_button_color)
            self.motor_buttons.append(btn)
            btn.setStyleSheet("""QPushButton {background-color: green; color: white; font-weight: bold; font-size: 20px; border-radius: 5px; padding: 10px; }""")
            control_panel.addWidget(btn)

        # Behavior title label ----------------------
        label = QtWidgets.QLabel("BEHAVIOR ENABLED")
        label.setAlignment(QtCore.Qt.AlignCenter)
        label.setStyleSheet("""font-weight: bold; font-size: 24px; color: white; """)

        # Behavior button group ----------------------
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
            btn.setStyleSheet("""QPushButton {background-color: darkred; color: white; font-weight: bold; font-size: 20px; border-radius: 5px; padding: 10px; }""")
            self.behavior_group.addButton(btn)
            self.behavior_buttons.append(btn)
            control_panel.addWidget(btn)
        self.behavior_group.buttonToggled.connect(self.handle_behavior_toggle)
        
        # 'SEND IT' section label ---------------------
        label = QtWidgets.QLabel("COMMAND")
        label.setStyleSheet("""font-weight: bold; font-size: 24px; color: white;""")
        label.setAlignment(QtCore.Qt.AlignCenter)
        control_panel.addWidget(label)
        
        # Activate (send) button ----------------------------------
        self.send_button = QtWidgets.QPushButton("Activate")
        self.send_button.setCheckable(True)
        self.send_button.setChecked(False)
        self.send_button.setMinimumHeight(40)
        self.send_button.toggled.connect(self.handle_send_toggled)
        self.send_button.setStyleSheet("""QPushButton {background-color: darkred; color: white; font-weight: bold; font-size: 20px; border-radius: 5px; padding: 10px; }""")
        control_panel.addWidget(self.send_button)
        
        # RECORD button
        self.record_button = QtWidgets.QPushButton("Record")
        self.record_button.setCheckable(True)
        self.record_button.setChecked(False)
        self.record_button.setMinimumHeight(40)
        self.record_button.toggled.connect(self.handle_record_toggled)
        self.record_button.setStyleSheet("""QPushButton {background-color: darkred; color: white; font-weight: bold; font-size: 20px; border-radius: 5px; padding: 10px; }""")
#         control_panel.addWidget(self.record_button)		***UNCOMMENT TO RECORD DATA***
        
        # Slider and label ----------------------------------------
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(400)		# CHECK THIS LATER: WHY SO HIGH?
        self.slider.setValue(200)		# CHECK THIS LATER: WHY SO HIGH?
        control_panel.addWidget(self.slider)
        self.slider_label = QtWidgets.QLabel("Slider Value: 0.00")
        control_panel.addWidget(self.slider_label)
        self.slider.valueChanged.connect(self.update_slider_label)
        self.slider.setStyleSheet("""
            QSlider::groove:horizontal {border: 1px solid #bbb; background: #ccc; height: 15px; border-radius: 5px; }
            QSlider::sub-page:horizontal {background: green; height: 10px; border-radius: 5px; }
            QSlider::add-page:horizontal {background: grey; height: 10px; border-radius: 5px; }
            QSlider::handle:horizontal {background: white; border: 1px solid black; width: 30px; margin: -10px 0; border-radius: 10px; }""")
        
        # HOME button -------------------------------------
        self.home_button = QtWidgets.QPushButton("Home")
        self.home_button.setMinimumHeight(40)
        self.home_button.clicked.connect(self.handle_home)
        self.home_button.setStyleSheet("""QPushButton {background-color: darkorange; color: white; font-size: 20px; font-weight: bold; border-radius: 5px; padding: 10px; }""")
        control_panel.addWidget(self.home_button)

        # RESET button
        self.home_button = QtWidgets.QPushButton("Reset")
        self.home_button.setMinimumHeight(40)
        self.home_button.clicked.connect(self.handle_reset)
        self.home_button.setStyleSheet("""QPushButton {background-color: darkorange; color: white; font-size: 20px; font-weight: bold; border-radius: 5px; padding: 10px; }""")
#         control_panel.addWidget(self.home_button)		***UNCOMMENT TO ADD A RESET BUTTON FOR THE CONTROLLERS***

        # EXIT button ------------------------------------------
        self.exit_button = QtWidgets.QPushButton("Exit")
        self.exit_button.setMinimumHeight(40)
        self.exit_button.clicked.connect(self.exit_application)
        self.exit_button.setStyleSheet("""QPushButton {background-color: red; color: white; font-size: 20px; font-weight: bold; border-radius: 5px; padding: 10px; }""")
        control_panel.addWidget(self.exit_button)

        # Create and constrain timer history -------
        self.start_time = time.time()
        self.history_sec = 10
        self.max_len = self.history_sec * 20
        self.time_vals = deque(maxlen=self.max_len)
        self.cmd_vals = deque(maxlen=self.max_len)
        self.fb_vals = [deque(maxlen=self.max_len)]
        
        # Initialize values ----------------
        self.last_cmd = 0.0
        self.last_fb = [0.0, 0.0, 0.0]
        self.rev_targets = [0.0, 0.0, 0.0]

        # Create timers -----------------------------------
        # Plot updating timer ___
        self.plot_timer = QtCore.QTimer()
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.setSingleShot(False)
        # Command updating timer ___
        self.cmd_timer = QtCore.QTimer()
        self.cmd_timer.timeout.connect(self.update_command)
        self.cmd_timer.setSingleShot(False)
        # LVDT tracker timer ___
        self.lvdt_timer = QtCore.QTimer()
        self.lvdt_timer.timeout.connect(read_lvdt)
        self.lvdt_timer.setSingleShot(False)    
        self.lvdt_timer.start(100) # UNCOMMENT TO ENABLE LVDT READINGS

        self.showFullScreen()			# UNCOMMENT FOR FULL SCREEN CONTROL PANEL
        self.setEnabled(False)		# Disable GUI during homing
        QtCore.QTimer.singleShot(100, self.start_homing_sequence)