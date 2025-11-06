"""
ODrive Control GUI
A PyQt5-based graphical interface for controlling ODrive motors via CAN
Uses can_runner.py backend for motor control and telemetry
"""

import sys
import os
import asyncio
from pathlib import Path
from collections import deque
import time
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QPushButton, QLabel, QLineEdit, QTextEdit, QGroupBox, QGridLayout,
    QDoubleSpinBox, QSpinBox, QTabWidget, QFileDialog, QMessageBox,
    QStatusBar, QProgressBar, QCheckBox
)
from PyQt5.QtCore import QTimer, pyqtSignal, QThread, Qt
from PyQt5.QtGui import QFont, QPalette, QColor
import qasync
from qasync import QEventLoop
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

# Add parent directory to path to import can_runner
sys.path.insert(0, str(Path(__file__).parent.parent))
from can_runner import HighPerformanceODriveSystem


class ODriveWorker(QThread):
    """Background thread for running ODrive system"""
    status_update = pyqtSignal(dict)
    log_message = pyqtSignal(str)
    error_message = pyqtSignal(str)
    
    def __init__(self, node_id=0):
        super().__init__()
        self.node_id = node_id
        self.system = None
        self.loop = None
        
    def run(self):
        """Run the asyncio event loop in background thread"""
        try:
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            
            self.system = HighPerformanceODriveSystem(node_id=self.node_id)
            self.loop.run_until_complete(self.system.initialize())
            
            self.log_message.emit("✅ ODrive system initialized")
            
            # Keep loop running
            self.loop.run_forever()
            
        except Exception as e:
            self.error_message.emit(f"System initialization failed: {e}")
    
    def stop(self):
        """Stop the background thread"""
        if self.loop:
            self.loop.call_soon_threadsafe(self.loop.stop)
        self.wait()


class ODriveGUI(QMainWindow):
    """Main GUI window for ODrive control"""
    
    def __init__(self):
        super().__init__()
        self.system = None
        self.telemetry_timer = QTimer()
        self.telemetry_timer.timeout.connect(self.update_telemetry)
        
        # Live plot data buffers (low priority, ring buffer with max 1000 points)
        self.plot_buffer_size = 1000
        self.plot_time_data = deque(maxlen=self.plot_buffer_size)
        self.plot_position_data = deque(maxlen=self.plot_buffer_size)
        self.plot_position_setpoint_data = deque(maxlen=self.plot_buffer_size)
        self.plot_velocity_data = deque(maxlen=self.plot_buffer_size)
        self.plot_start_time = None
        self.plot_enabled = False
        
        # Low priority plot timer (5Hz, half the rate of telemetry)
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.setInterval(200)  # 200ms = 5Hz
        
        self.init_ui()
        
    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle("ODrive Control System v1.1.0")
        self.setGeometry(100, 100, 1200, 800)
        
        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Create tab widget
        tabs = QTabWidget()
        main_layout.addWidget(tabs)
        
        # Add tabs
        tabs.addTab(self.create_control_tab(), "Motor Control")
        tabs.addTab(self.create_telemetry_tab(), "Telemetry")
        tabs.addTab(self.create_plot_tab(), "Live Plot")
        tabs.addTab(self.create_trajectory_tab(), "Trajectory")
        tabs.addTab(self.create_config_tab(), "Configuration")
        tabs.addTab(self.create_logging_tab(), "Data Logging")
        
        # Status bar
        self.statusBar = QStatusBar()
        self.setStatusBar(self.statusBar)
        self.statusBar.showMessage("Disconnected")
        
        
        # Apply styling
        self.apply_styling()
        
    def create_control_tab(self):
        """Create motor control tab"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # System control group
        sys_group = QGroupBox("System Control")
        sys_layout = QHBoxLayout()
        
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self.connect_odrive)
        self.btn_arm = QPushButton("Arm")
        self.btn_arm.clicked.connect(self.arm_system)
        self.btn_arm.setEnabled(False)
        self.btn_disarm = QPushButton("Disarm")
        self.btn_disarm.clicked.connect(self.disarm_system)
        self.btn_disarm.setEnabled(False)
        self.btn_stop = QPushButton("🛑 STOP")
        self.btn_stop.clicked.connect(self.emergency_stop)
        self.btn_stop.setEnabled(False)
        self.btn_stop.setStyleSheet("background-color: #d32f2f; color: white; font-weight: bold;")
        
        sys_layout.addWidget(self.btn_connect)
        sys_layout.addWidget(self.btn_arm)
        sys_layout.addWidget(self.btn_disarm)
        sys_layout.addWidget(self.btn_stop)
        sys_group.setLayout(sys_layout)
        layout.addWidget(sys_group)
        
        # Position control group
        pos_group = QGroupBox("Position Control")
        pos_layout = QGridLayout()
        
        pos_layout.addWidget(QLabel("Target Position (turns):"), 0, 0)
        self.spin_position = QDoubleSpinBox()
        self.spin_position.setRange(-100, 100)
        self.spin_position.setDecimals(3)
        self.spin_position.setSingleStep(0.1)
        pos_layout.addWidget(self.spin_position, 0, 1)
        
        self.btn_set_pos = QPushButton("Set Position")
        self.btn_set_pos.clicked.connect(self.set_position)
        self.btn_set_pos.setEnabled(False)
        pos_layout.addWidget(self.btn_set_pos, 0, 2)
        
        pos_layout.addWidget(QLabel("Velocity (turns/s):"), 1, 0)
        self.spin_velocity = QDoubleSpinBox()
        self.spin_velocity.setRange(0.1, 10.0)
        self.spin_velocity.setDecimals(2)
        self.spin_velocity.setValue(1.0)
        pos_layout.addWidget(self.spin_velocity, 1, 1)
        
        self.btn_move = QPushButton("Move to Position")
        self.btn_move.clicked.connect(self.move_to_position)
        self.btn_move.setEnabled(False)
        pos_layout.addWidget(self.btn_move, 1, 2)
        
        pos_layout.addWidget(QLabel("Relative Move (turns):"), 2, 0)
        self.spin_relative = QDoubleSpinBox()
        self.spin_relative.setRange(-10, 10)
        self.spin_relative.setDecimals(3)
        self.spin_relative.setSingleStep(0.1)
        pos_layout.addWidget(self.spin_relative, 2, 1)
        
        self.btn_relative = QPushButton("Relative Move")
        self.btn_relative.clicked.connect(self.relative_move)
        self.btn_relative.setEnabled(False)
        pos_layout.addWidget(self.btn_relative, 2, 2)
        
        pos_group.setLayout(pos_layout)
        layout.addWidget(pos_group)
        
        # Velocity control group
        vel_group = QGroupBox("Velocity Control")
        vel_layout = QGridLayout()
        
        vel_layout.addWidget(QLabel("Target Velocity (turns/s):"), 0, 0)
        self.spin_vel_target = QDoubleSpinBox()
        self.spin_vel_target.setRange(-10, 10)
        self.spin_vel_target.setDecimals(2)
        self.spin_vel_target.setSingleStep(0.1)
        vel_layout.addWidget(self.spin_vel_target, 0, 1)
        
        self.btn_set_vel = QPushButton("Set Velocity")
        self.btn_set_vel.clicked.connect(self.set_velocity)
        self.btn_set_vel.setEnabled(False)
        vel_layout.addWidget(self.btn_set_vel, 0, 2)
        
        vel_group.setLayout(vel_layout)
        layout.addWidget(vel_group)
        
        # Oscillation control group
        osc_group = QGroupBox("Oscillation Control")
        osc_layout = QGridLayout()
        
        osc_layout.addWidget(QLabel("Amplitude (turns):"), 0, 0)
        self.spin_osc_amp = QDoubleSpinBox()
        self.spin_osc_amp.setRange(0.1, 10.0)
        self.spin_osc_amp.setDecimals(2)
        self.spin_osc_amp.setValue(0.5)
        osc_layout.addWidget(self.spin_osc_amp, 0, 1)
        
        osc_layout.addWidget(QLabel("Frequency (Hz):"), 1, 0)
        self.spin_osc_freq = QDoubleSpinBox()
        self.spin_osc_freq.setRange(0.1, 10.0)
        self.spin_osc_freq.setDecimals(2)
        self.spin_osc_freq.setValue(1.0)
        osc_layout.addWidget(self.spin_osc_freq, 1, 1)
        
        osc_layout.addWidget(QLabel("Duration (s):"), 2, 0)
        self.spin_osc_duration = QSpinBox()
        self.spin_osc_duration.setRange(1, 300)
        self.spin_osc_duration.setValue(10)
        osc_layout.addWidget(self.spin_osc_duration, 2, 1)
        
        osc_layout.addWidget(QLabel("Phase Angle (deg):"), 3, 0)
        self.spin_osc_phase = QDoubleSpinBox()
        self.spin_osc_phase.setRange(0, 360)
        self.spin_osc_phase.setDecimals(0)
        self.spin_osc_phase.setValue(0)
        self.spin_osc_phase.setToolTip("0° = sine wave, 90° = cosine wave")
        osc_layout.addWidget(self.spin_osc_phase, 3, 1)
        
        self.btn_oscillate = QPushButton("Start Oscillation")
        self.btn_oscillate.clicked.connect(self.start_oscillation)
        self.btn_oscillate.setEnabled(False)
        osc_layout.addWidget(self.btn_oscillate, 4, 0, 1, 2)
        
        osc_group.setLayout(osc_layout)
        layout.addWidget(osc_group)
        
        # Console output
        console_group = QGroupBox("Console Output")
        console_layout = QVBoxLayout()
        self.console_output = QTextEdit()
        self.console_output.setReadOnly(True)
        self.console_output.setMaximumHeight(150)
        console_layout.addWidget(self.console_output)
        console_group.setLayout(console_layout)
        layout.addWidget(console_group)
        
        layout.addStretch()
        return widget
    
    def create_telemetry_tab(self):
        """Create telemetry display tab"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # Telemetry display group
        telem_group = QGroupBox("Real-time Telemetry")
        telem_layout = QGridLayout()
        
        # Position
        telem_layout.addWidget(QLabel("Position:"), 0, 0)
        self.lbl_position = QLabel("N/A")
        self.lbl_position.setFont(QFont("Courier New", 12, QFont.Bold))
        telem_layout.addWidget(self.lbl_position, 0, 1)
        telem_layout.addWidget(QLabel("turns"), 0, 2)
        
        # Velocity
        telem_layout.addWidget(QLabel("Velocity:"), 1, 0)
        self.lbl_velocity = QLabel("N/A")
        self.lbl_velocity.setFont(QFont("Courier New", 12, QFont.Bold))
        telem_layout.addWidget(self.lbl_velocity, 1, 1)
        telem_layout.addWidget(QLabel("turns/s"), 1, 2)
        
        # Motor Temperature
        telem_layout.addWidget(QLabel("Motor Temp:"), 2, 0)
        self.lbl_motor_temp = QLabel("N/A")
        self.lbl_motor_temp.setFont(QFont("Courier New", 12, QFont.Bold))
        telem_layout.addWidget(self.lbl_motor_temp, 2, 1)
        telem_layout.addWidget(QLabel("°C"), 2, 2)
        
        # FET Temperature
        telem_layout.addWidget(QLabel("FET Temp:"), 3, 0)
        self.lbl_fet_temp = QLabel("N/A")
        self.lbl_fet_temp.setFont(QFont("Courier New", 12, QFont.Bold))
        telem_layout.addWidget(self.lbl_fet_temp, 3, 1)
        telem_layout.addWidget(QLabel("°C"), 3, 2)
        
        # Bus Voltage
        telem_layout.addWidget(QLabel("Bus Voltage:"), 4, 0)
        self.lbl_bus_voltage = QLabel("N/A")
        self.lbl_bus_voltage.setFont(QFont("Courier New", 12, QFont.Bold))
        telem_layout.addWidget(self.lbl_bus_voltage, 4, 1)
        telem_layout.addWidget(QLabel("V"), 4, 2)
        
        # Bus Current
        telem_layout.addWidget(QLabel("Bus Current:"), 5, 0)
        self.lbl_bus_current = QLabel("N/A")
        self.lbl_bus_current.setFont(QFont("Courier New", 12, QFont.Bold))
        telem_layout.addWidget(self.lbl_bus_current, 5, 1)
        telem_layout.addWidget(QLabel("A"), 5, 2)
        
        # Iq Measured
        telem_layout.addWidget(QLabel("Iq Measured:"), 6, 0)
        self.lbl_iq_measured = QLabel("N/A")
        self.lbl_iq_measured.setFont(QFont("Courier New", 12, QFont.Bold))
        telem_layout.addWidget(self.lbl_iq_measured, 6, 1)
        telem_layout.addWidget(QLabel("A"), 6, 2)
        
        # Torque Estimate
        telem_layout.addWidget(QLabel("Torque Estimate:"), 7, 0)
        self.lbl_torque_estimate = QLabel("N/A")
        self.lbl_torque_estimate.setFont(QFont("Courier New", 12, QFont.Bold))
        telem_layout.addWidget(self.lbl_torque_estimate, 7, 1)
        telem_layout.addWidget(QLabel("Nm"), 7, 2)
        
        telem_group.setLayout(telem_layout)
        layout.addWidget(telem_group)
        
        # Status group
        status_group = QGroupBox("System Status")
        status_layout = QGridLayout()
        
        status_layout.addWidget(QLabel("Connected:"), 0, 0)
        self.lbl_connected = QLabel("❌")
        status_layout.addWidget(self.lbl_connected, 0, 1)
        
        status_layout.addWidget(QLabel("Armed:"), 1, 0)
        self.lbl_armed = QLabel("❌")
        status_layout.addWidget(self.lbl_armed, 1, 1)
        
        status_layout.addWidget(QLabel("Logging:"), 2, 0)
        self.lbl_logging = QLabel("❌")
        status_layout.addWidget(self.lbl_logging, 2, 1)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        layout.addStretch()
        return widget
    
    def create_plot_tab(self):
        """Create live plotting tab (LOW PRIORITY - won't affect control/telemetry)"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # Plot controls
        control_group = QGroupBox("Plot Controls")
        control_layout = QHBoxLayout()
        
        self.chk_enable_plot = QCheckBox("Enable Live Plot")
        self.chk_enable_plot.setChecked(False)
        self.chk_enable_plot.stateChanged.connect(self.toggle_plot)
        control_layout.addWidget(self.chk_enable_plot)
        
        self.btn_clear_plot = QPushButton("Clear Plot Data")
        self.btn_clear_plot.clicked.connect(self.clear_plot_data)
        control_layout.addWidget(self.btn_clear_plot)
        
        control_layout.addStretch()
        
        # Time window control
        control_layout.addWidget(QLabel("Time Window (s):"))
        self.spin_plot_window = QSpinBox()
        self.spin_plot_window.setRange(5, 300)
        self.spin_plot_window.setValue(30)
        self.spin_plot_window.setSuffix(" s")
        control_layout.addWidget(self.spin_plot_window)
        
        control_group.setLayout(control_layout)
        layout.addWidget(control_group)
        
        # Tracking error display
        error_layout = QHBoxLayout()
        self.lbl_tracking_error = QLabel("Tracking Error: N/A")
        self.lbl_tracking_error.setStyleSheet("font-weight: bold; font-size: 11pt;")
        error_layout.addWidget(self.lbl_tracking_error)
        error_layout.addStretch()
        layout.addLayout(error_layout)
        
        # Plot info label
        self.lbl_plot_info = QLabel("⚠️ Plot updates at 5Hz (low priority - won't affect control performance)")
        self.lbl_plot_info.setStyleSheet("color: #666; font-style: italic;")
        layout.addWidget(self.lbl_plot_info)
        
        # Create matplotlib figure and canvas
        self.plot_figure = Figure(figsize=(10, 6), dpi=100)
        self.plot_canvas = FigureCanvas(self.plot_figure)
        layout.addWidget(self.plot_canvas)
        
        # Create subplots
        self.ax_position = self.plot_figure.add_subplot(2, 1, 1)
        self.ax_velocity = self.plot_figure.add_subplot(2, 1, 2)
        
        # Configure plots
        self.ax_position.set_xlabel('Time (s)')
        self.ax_position.set_ylabel('Position (turns)')
        self.ax_position.set_title('Motor Position: Setpoint vs Measured')
        self.ax_position.grid(True, alpha=0.3)
        
        self.ax_velocity.set_xlabel('Time (s)')
        self.ax_velocity.set_ylabel('Velocity (turns/s)')
        self.ax_velocity.set_title('Motor Velocity')
        self.ax_velocity.grid(True, alpha=0.3)
        
        self.plot_figure.tight_layout()
        
        # Initialize plot lines
        self.line_position_setpoint, = self.ax_position.plot([], [], 'g--', linewidth=2.0, label='Setpoint', alpha=0.7)
        self.line_position, = self.ax_position.plot([], [], 'b-', linewidth=1.5, label='Measured')
        self.line_velocity, = self.ax_velocity.plot([], [], 'r-', linewidth=1.5, label='Velocity')
        
        self.ax_position.legend(loc='upper right')
        self.ax_velocity.legend(loc='upper right')
        
        return widget
    
    def create_trajectory_tab(self):
        """Create trajectory control tab"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # Trajectory file group
        traj_group = QGroupBox("Trajectory File")
        traj_layout = QGridLayout()
        
        traj_layout.addWidget(QLabel("File:"), 0, 0)
        self.txt_traj_file = QLineEdit()
        self.txt_traj_file.setReadOnly(True)
        traj_layout.addWidget(self.txt_traj_file, 0, 1)
        
        self.btn_browse_traj = QPushButton("Browse...")
        self.btn_browse_traj.clicked.connect(self.browse_trajectory)
        traj_layout.addWidget(self.btn_browse_traj, 0, 2)
        
        self.btn_load_traj = QPushButton("Load Trajectory")
        self.btn_load_traj.clicked.connect(self.load_trajectory)
        self.btn_load_traj.setEnabled(False)
        traj_layout.addWidget(self.btn_load_traj, 1, 0, 1, 3)
        
        traj_group.setLayout(traj_layout)
        layout.addWidget(traj_group)
        
        # Trajectory control group
        ctrl_group = QGroupBox("Playback Control")
        ctrl_layout = QHBoxLayout()
        
        self.btn_play_traj = QPushButton("▶ Play")
        self.btn_play_traj.clicked.connect(self.play_trajectory)
        self.btn_play_traj.setEnabled(False)
        
        self.btn_pause_traj = QPushButton("⏸ Pause")
        self.btn_pause_traj.clicked.connect(self.pause_trajectory)
        self.btn_pause_traj.setEnabled(False)
        
        self.btn_stop_traj = QPushButton("⏹ Stop")
        self.btn_stop_traj.clicked.connect(self.stop_trajectory)
        self.btn_stop_traj.setEnabled(False)
        
        ctrl_layout.addWidget(self.btn_play_traj)
        ctrl_layout.addWidget(self.btn_pause_traj)
        ctrl_layout.addWidget(self.btn_stop_traj)
        ctrl_group.setLayout(ctrl_layout)
        layout.addWidget(ctrl_group)
        
        # Trajectory status
        status_group = QGroupBox("Trajectory Status")
        status_layout = QVBoxLayout()
        self.txt_traj_status = QTextEdit()
        self.txt_traj_status.setReadOnly(True)
        self.txt_traj_status.setMaximumHeight(200)
        status_layout.addWidget(self.txt_traj_status)
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        layout.addStretch()
        return widget
    
    def create_config_tab(self):
        """Create configuration tab"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # Config file group
        config_group = QGroupBox("Configuration File")
        config_layout = QGridLayout()
        
        config_layout.addWidget(QLabel("File:"), 0, 0)
        self.txt_config_file = QLineEdit()
        self.txt_config_file.setText("config.json")
        config_layout.addWidget(self.txt_config_file, 0, 1)
        
        self.btn_browse_config = QPushButton("Browse...")
        self.btn_browse_config.clicked.connect(self.browse_config)
        config_layout.addWidget(self.btn_browse_config, 0, 2)
        
        self.btn_load_config = QPushButton("Load Configuration")
        self.btn_load_config.clicked.connect(self.load_config)
        self.btn_load_config.setEnabled(False)
        config_layout.addWidget(self.btn_load_config, 1, 0, 1, 3)
        
        self.btn_save_config = QPushButton("Save to ODrive NVM")
        self.btn_save_config.clicked.connect(self.save_config)
        self.btn_save_config.setEnabled(False)
        config_layout.addWidget(self.btn_save_config, 2, 0, 1, 3)
        
        config_group.setLayout(config_layout)
        layout.addWidget(config_group)
        
        # Config output
        output_group = QGroupBox("Configuration Output")
        output_layout = QVBoxLayout()
        self.txt_config_output = QTextEdit()
        self.txt_config_output.setReadOnly(True)
        output_layout.addWidget(self.txt_config_output)
        output_group.setLayout(output_layout)
        layout.addWidget(output_group)
        
        return widget
    
    def create_logging_tab(self):
        """Create data logging tab"""
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # Logging control group
        log_group = QGroupBox("Data Logging Control")
        log_layout = QGridLayout()
        
        log_layout.addWidget(QLabel("Log Filename:"), 0, 0)
        self.txt_log_file = QLineEdit()
        self.txt_log_file.setPlaceholderText("Optional custom filename")
        log_layout.addWidget(self.txt_log_file, 0, 1)
        
        self.btn_start_log = QPushButton("Start Logging")
        self.btn_start_log.clicked.connect(self.start_logging)
        self.btn_start_log.setEnabled(False)
        log_layout.addWidget(self.btn_start_log, 1, 0)
        
        self.btn_stop_log = QPushButton("Stop Logging")
        self.btn_stop_log.clicked.connect(self.stop_logging)
        self.btn_stop_log.setEnabled(False)
        log_layout.addWidget(self.btn_stop_log, 1, 1)
        
        log_group.setLayout(log_layout)
        layout.addWidget(log_group)
        
        # Logging status
        status_group = QGroupBox("Logging Status")
        status_layout = QVBoxLayout()
        self.txt_log_status = QTextEdit()
        self.txt_log_status.setReadOnly(True)
        self.txt_log_status.setMaximumHeight(200)
        status_layout.addWidget(self.txt_log_status)
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        layout.addStretch()
        return widget
    
    def apply_styling(self):
        """Apply custom styling to the GUI"""
        self.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 2px solid #cccccc;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
            QPushButton {
                padding: 8px;
                border-radius: 4px;
                font-size: 10pt;
            }
            QPushButton:enabled {
                background-color: #2196F3;
                color: white;
            }
            QPushButton:disabled {
                background-color: #cccccc;
                color: #666666;
            }
            QPushButton:hover:enabled {
                background-color: #1976D2;
            }
            QSpinBox, QDoubleSpinBox, QLineEdit {
                padding: 5px;
                border: 1px solid #cccccc;
                border-radius: 3px;
            }
        """)
    
    # Connection and initialization
    async def connect_odrive_async(self):
        """Connect to ODrive asynchronously"""
        try:
            self.log_to_console("🔌 Connecting to ODrive...")
            self.system = HighPerformanceODriveSystem(node_id=0)
            
            success = await self.system.initialize()
            
            if success:
                self.log_to_console("✅ Connected to ODrive successfully")
                self.statusBar.showMessage("Connected")
                self.lbl_connected.setText("✅")
                
                # Enable controls
                self.btn_connect.setEnabled(False)
                self.btn_arm.setEnabled(True)
                self.btn_disarm.setEnabled(True)
                self.btn_stop.setEnabled(True)
                self.btn_load_config.setEnabled(True)
                self.btn_save_config.setEnabled(True)
                
                # Start telemetry updates
                self.telemetry_timer.start(100)  # Update every 100ms
            else:
                self.log_to_console("❌ Failed to connect to ODrive")
                QMessageBox.critical(self, "Connection Error", "Failed to initialize ODrive system")
                
        except Exception as e:
            self.log_to_console(f"❌ Connection error: {e}")
            QMessageBox.critical(self, "Connection Error", str(e))
    
    def connect_odrive(self):
        """Connect to ODrive (button handler)"""
        asyncio.ensure_future(self.connect_odrive_async())
    
    async def arm_system_async(self):
        """Arm the system asynchronously"""
        try:
            await self.system.arm_system()
            self.log_to_console("✅ System armed")
            self.lbl_armed.setText("✅")
            self.statusBar.showMessage("Armed")
            
            # Enable motor controls
            self.btn_set_pos.setEnabled(True)
            self.btn_move.setEnabled(True)
            self.btn_relative.setEnabled(True)
            self.btn_set_vel.setEnabled(True)
            self.btn_oscillate.setEnabled(True)
            self.btn_start_log.setEnabled(True)
            self.btn_load_traj.setEnabled(True)
            
        except Exception as e:
            self.log_to_console(f"❌ Arm failed: {e}")
            QMessageBox.warning(self, "Arm Error", str(e))
    
    def arm_system(self):
        """Arm system (button handler)"""
        asyncio.ensure_future(self.arm_system_async())
    
    async def disarm_system_async(self):
        """Disarm the system asynchronously"""
        try:
            await self.system.disarm_system()
            self.log_to_console("🛑 System disarmed")
            self.lbl_armed.setText("❌")
            self.statusBar.showMessage("Disarmed")
            
            # Disable motor controls
            self.btn_set_pos.setEnabled(False)
            self.btn_move.setEnabled(False)
            self.btn_relative.setEnabled(False)
            self.btn_set_vel.setEnabled(False)
            self.btn_oscillate.setEnabled(False)
            
        except Exception as e:
            self.log_to_console(f"❌ Disarm failed: {e}")
    
    def disarm_system(self):
        """Disarm system (button handler)"""
        asyncio.ensure_future(self.disarm_system_async())
    
    async def emergency_stop_async(self):
        """Emergency stop asynchronously"""
        try:
            if self.system and self.system.position_controller:
                self.system.position_controller.stop_trajectory()
            await self.system.disable_motor()
            self.log_to_console("🛑 EMERGENCY STOP")
            
        except Exception as e:
            self.log_to_console(f"❌ Stop failed: {e}")
    
    def emergency_stop(self):
        """Emergency stop (button handler)"""
        asyncio.ensure_future(self.emergency_stop_async())
    
    # Motor control commands
    async def set_position_async(self):
        """Set position asynchronously"""
        try:
            position = self.spin_position.value()
            await self.system.enable_motor_for_position_control()
            
            if self.system.position_controller:
                self.system.position_controller.set_position(position)
                self.log_to_console(f"📍 Position set to {position:.3f} turns")
                
        except Exception as e:
            self.log_to_console(f"❌ Set position failed: {e}")
    
    def set_position(self):
        """Set position (button handler)"""
        asyncio.ensure_future(self.set_position_async())
    
    async def move_to_position_async(self):
        """Move to position with velocity asynchronously"""
        try:
            position = self.spin_position.value()
            velocity = self.spin_velocity.value()
            
            await self.system.enable_motor_for_position_control()
            
            if self.system.position_controller:
                self.system.position_controller.move_to_position(position, velocity)
                self.log_to_console(f"🚀 Moving to {position:.3f} turns at {velocity:.2f} turns/s")
                
        except Exception as e:
            self.log_to_console(f"❌ Move failed: {e}")
    
    def move_to_position(self):
        """Move to position (button handler)"""
        asyncio.ensure_future(self.move_to_position_async())
    
    async def relative_move_async(self):
        """Relative move asynchronously"""
        try:
            delta = self.spin_relative.value()
            velocity = self.spin_velocity.value()
            
            await self.system.enable_motor_for_position_control()
            
            if self.system.position_controller:
                self.system.position_controller.relative_move(delta, velocity)
                self.log_to_console(f"↔️ Relative move {delta:+.3f} turns")
                
        except Exception as e:
            self.log_to_console(f"❌ Relative move failed: {e}")
    
    def relative_move(self):
        """Relative move (button handler)"""
        asyncio.ensure_future(self.relative_move_async())
    
    async def set_velocity_async(self):
        """Set velocity asynchronously"""
        try:
            velocity = self.spin_vel_target.value()
            await self.system.enable_motor_for_velocity_control()
            
            if self.system.position_controller:
                self.system.position_controller.set_velocity(velocity)
                self.log_to_console(f"⚡ Velocity set to {velocity:.2f} turns/s")
                
        except Exception as e:
            self.log_to_console(f"❌ Set velocity failed: {e}")
    
    def set_velocity(self):
        """Set velocity (button handler)"""
        asyncio.ensure_future(self.set_velocity_async())
    
    async def start_oscillation_async(self):
        """Start oscillation asynchronously"""
        try:
            amplitude = self.spin_osc_amp.value()
            frequency = self.spin_osc_freq.value()
            duration = self.spin_osc_duration.value()
            phase_deg = self.spin_osc_phase.value()
            
            await self.system.enable_motor_for_position_control()
            
            if self.system.position_controller:
                self.system.position_controller.oscillate(amplitude, frequency, duration, phase_deg)
                phase_str = f", phase {phase_deg:.0f}°" if phase_deg != 0 else ""
                self.log_to_console(f"🔄 Oscillation: ±{amplitude:.2f} turns @ {frequency:.2f}Hz{phase_str} for {duration}s")
                
        except Exception as e:
            self.log_to_console(f"❌ Oscillation failed: {e}")
    
    def start_oscillation(self):
        """Start oscillation (button handler)"""
        asyncio.ensure_future(self.start_oscillation_async())
    
    # Trajectory commands
    def browse_trajectory(self):
        """Browse for trajectory file"""
        filename, _ = QFileDialog.getOpenFileName(
            self, "Select Trajectory File", "",
            "Trajectory Files (*.csv *.json *.txt);;All Files (*)"
        )
        if filename:
            self.txt_traj_file.setText(filename)
    
    async def load_trajectory_async(self):
        """Load trajectory file asynchronously"""
        try:
            filepath = self.txt_traj_file.text()
            if not filepath:
                QMessageBox.warning(self, "No File", "Please select a trajectory file")
                return
            
            await self.system._load_trajectory(filepath)
            self.log_to_console(f"📂 Trajectory loaded: {filepath}")
            
            self.btn_play_traj.setEnabled(True)
            self.btn_pause_traj.setEnabled(True)
            self.btn_stop_traj.setEnabled(True)
            
        except Exception as e:
            self.log_to_console(f"❌ Load trajectory failed: {e}")
            QMessageBox.warning(self, "Load Error", str(e))
    
    def load_trajectory(self):
        """Load trajectory (button handler)"""
        asyncio.ensure_future(self.load_trajectory_async())
    
    async def play_trajectory_async(self):
        """Play trajectory asynchronously"""
        try:
            await self.system._play_trajectory()
            self.log_to_console("▶️ Trajectory playback started")
            
        except Exception as e:
            self.log_to_console(f"❌ Play trajectory failed: {e}")
    
    def play_trajectory(self):
        """Play trajectory (button handler)"""
        asyncio.ensure_future(self.play_trajectory_async())
    
    async def pause_trajectory_async(self):
        """Pause trajectory asynchronously"""
        try:
            await self.system._pause_trajectory()
            self.log_to_console("⏸️ Trajectory paused")
            
        except Exception as e:
            self.log_to_console(f"❌ Pause trajectory failed: {e}")
    
    def pause_trajectory(self):
        """Pause trajectory (button handler)"""
        asyncio.ensure_future(self.pause_trajectory_async())
    
    async def stop_trajectory_async(self):
        """Stop trajectory asynchronously"""
        try:
            await self.system._stop_trajectory_playback()
            self.log_to_console("⏹️ Trajectory stopped")
            
        except Exception as e:
            self.log_to_console(f"❌ Stop trajectory failed: {e}")
    
    def stop_trajectory(self):
        """Stop trajectory (button handler)"""
        asyncio.ensure_future(self.stop_trajectory_async())
    
    # Configuration commands
    def browse_config(self):
        """Browse for config file"""
        filename, _ = QFileDialog.getOpenFileName(
            self, "Select Configuration File", "",
            "JSON Files (*.json);;All Files (*)"
        )
        if filename:
            self.txt_config_file.setText(filename)
    
    async def load_config_async(self):
        """Load configuration file asynchronously"""
        try:
            config_file = self.txt_config_file.text()
            if not config_file:
                QMessageBox.warning(self, "No File", "Please specify a configuration file")
                return
            
            self.txt_config_output.clear()
            self.txt_config_output.append(f"Loading configuration from: {config_file}")
            
            await self.system._load_config_file(config_file)
            self.log_to_console(f"✅ Configuration loaded from {config_file}")
            
        except Exception as e:
            self.log_to_console(f"❌ Load config failed: {e}")
            QMessageBox.warning(self, "Load Error", str(e))
    
    def load_config(self):
        """Load config (button handler)"""
        asyncio.ensure_future(self.load_config_async())
    
    async def save_config_async(self):
        """Save configuration to ODrive NVM asynchronously"""
        try:
            reply = QMessageBox.question(
                self, "Save Configuration",
                "This will save the current configuration to ODrive NVM and reboot the device.\n\n"
                "The system will need to be re-initialized after reboot.\n\n"
                "Continue?",
                QMessageBox.Yes | QMessageBox.No
            )
            
            if reply == QMessageBox.Yes:
                await self.system._save_config_to_odrive()
                self.log_to_console("💾 Configuration saved to NVM, ODrive rebooting...")
                
        except Exception as e:
            self.log_to_console(f"❌ Save config failed: {e}")
            QMessageBox.warning(self, "Save Error", str(e))
    
    def save_config(self):
        """Save config (button handler)"""
        asyncio.ensure_future(self.save_config_async())
    
    # Logging commands
    async def start_logging_async(self):
        """Start data logging asynchronously"""
        try:
            filename = self.txt_log_file.text() if self.txt_log_file.text() else None
            await self.system._start_logging(filename)
            
            self.log_to_console("📝 Data logging started")
            self.lbl_logging.setText("✅")
            self.btn_start_log.setEnabled(False)
            self.btn_stop_log.setEnabled(True)
            
        except Exception as e:
            self.log_to_console(f"❌ Start logging failed: {e}")
    
    def start_logging(self):
        """Start logging (button handler)"""
        asyncio.ensure_future(self.start_logging_async())
    
    async def stop_logging_async(self):
        """Stop data logging asynchronously"""
        try:
            await self.system._stop_logging()
            
            self.log_to_console("🛑 Data logging stopped")
            self.lbl_logging.setText("❌")
            self.btn_start_log.setEnabled(True)
            self.btn_stop_log.setEnabled(False)
            
        except Exception as e:
            self.log_to_console(f"❌ Stop logging failed: {e}")
    
    def stop_logging(self):
        """Stop logging (button handler)"""
        asyncio.ensure_future(self.stop_logging_async())
    
    # Telemetry update
    def update_telemetry(self):
        """Update telemetry display"""
        if not self.system or not self.system.telemetry_manager:
            return
        
        try:
            data = self.system.telemetry_manager.get_latest_data()
            
            # Update position
            pos = data.get('position')
            self.lbl_position.setText(f"{pos:.3f}" if pos is not None else "N/A")
            
            # Update velocity
            vel = data.get('velocity')
            self.lbl_velocity.setText(f"{vel:.3f}" if vel is not None else "N/A")
            
            # Update motor temperature
            motor_temp = data.get('motor_temp')
            self.lbl_motor_temp.setText(f"{motor_temp:.1f}" if motor_temp is not None else "N/A")
            
            # Update FET temperature
            fet_temp = data.get('fet_temp')
            self.lbl_fet_temp.setText(f"{fet_temp:.1f}" if fet_temp is not None else "N/A")
            
            # Update bus voltage
            bus_voltage = data.get('bus_voltage')
            self.lbl_bus_voltage.setText(f"{bus_voltage:.1f}" if bus_voltage is not None else "N/A")
            
            # Update bus current
            bus_current = data.get('bus_current')
            self.lbl_bus_current.setText(f"{bus_current:.2f}" if bus_current is not None else "N/A")
            
            # Update Iq measured
            iq_measured = data.get('iq_measured')
            self.lbl_iq_measured.setText(f"{iq_measured:.2f}" if iq_measured is not None else "N/A")
            
            # Update torque estimate
            torque_estimate = data.get('torque_estimate')
            self.lbl_torque_estimate.setText(f"{torque_estimate:.3f}" if torque_estimate is not None else "N/A")
            
            # If plotting is enabled, add data to plot buffers (LOW PRIORITY)
            if self.plot_enabled:
                if self.plot_start_time is None:
                    self.plot_start_time = time.time()
                
                current_time = time.time() - self.plot_start_time
                
                # Get target position directly from position controller (not from telemetry)
                target_pos = None
                if self.system and self.system.position_controller:
                    target_pos = self.system.position_controller.current_target
                
                self.plot_time_data.append(current_time)
                self.plot_position_data.append(pos if pos is not None else 0.0)
                self.plot_position_setpoint_data.append(target_pos if target_pos is not None else (pos if pos is not None else 0.0))
                self.plot_velocity_data.append(vel if vel is not None else 0.0)
            
        except Exception as e:
            pass  # Silently ignore telemetry update errors
    
    def update_plot(self):
        """Update live plot (LOW PRIORITY - 5Hz update rate)"""
        if not self.plot_enabled or len(self.plot_time_data) == 0:
            return
        
        try:
            # Convert deques to lists for plotting
            time_data = list(self.plot_time_data)
            pos_data = list(self.plot_position_data)
            pos_setpoint_data = list(self.plot_position_setpoint_data)
            vel_data = list(self.plot_velocity_data)
            
            # Get time window
            time_window = self.spin_plot_window.value()
            current_time = time_data[-1] if time_data else 0
            
            # Filter data to time window
            filtered_indices = [i for i, t in enumerate(time_data) if t >= current_time - time_window]
            if filtered_indices:
                filtered_time = [time_data[i] for i in filtered_indices]
                filtered_pos = [pos_data[i] for i in filtered_indices]
                filtered_pos_setpoint = [pos_setpoint_data[i] for i in filtered_indices]
                filtered_vel = [vel_data[i] for i in filtered_indices]
                
                # Update plot data
                self.line_position_setpoint.set_data(filtered_time, filtered_pos_setpoint)
                self.line_position.set_data(filtered_time, filtered_pos)
                self.line_velocity.set_data(filtered_time, filtered_vel)
                
                # Calculate tracking error (most recent point)
                if filtered_pos and filtered_pos_setpoint:
                    tracking_error = filtered_pos[-1] - filtered_pos_setpoint[-1]
                    error_pct = abs(tracking_error) * 100  # Convert to percent-turns for readability
                    
                    # Color code based on error magnitude
                    if abs(tracking_error) < 0.001:  # < 1 milli-turn
                        color = "green"
                        status = "Excellent"
                    elif abs(tracking_error) < 0.01:  # < 10 milli-turns
                        color = "orange"
                        status = "Good"
                    else:
                        color = "red"
                        status = "Poor"
                    
                    self.lbl_tracking_error.setText(
                        f"Tracking Error: {tracking_error:.4f} turns ({error_pct:.2f}% turn) - {status}"
                    )
                    self.lbl_tracking_error.setStyleSheet(f"font-weight: bold; font-size: 11pt; color: {color};")
                
                # Auto-scale axes
                self.ax_position.relim()
                self.ax_position.autoscale_view(True, True, True)
                self.ax_velocity.relim()
                self.ax_velocity.autoscale_view(True, True, True)
                
                # Redraw canvas (non-blocking)
                self.plot_canvas.draw_idle()
            
        except Exception as e:
            pass  # Silently ignore plot update errors
    
    def toggle_plot(self, state):
        """Enable/disable live plotting"""
        self.plot_enabled = (state == Qt.Checked)
        
        if self.plot_enabled:
            self.plot_start_time = time.time()
            self.plot_timer.start()
            self.lbl_plot_info.setText("✅ Plot active (5Hz update, low priority)")
            self.lbl_plot_info.setStyleSheet("color: green; font-style: italic;")
        else:
            self.plot_timer.stop()
            self.lbl_plot_info.setText("⚠️ Plot paused")
            self.lbl_plot_info.setStyleSheet("color: orange; font-style: italic;")
    
    def clear_plot_data(self):
        """Clear all plot data buffers"""
        self.plot_time_data.clear()
        self.plot_position_data.clear()
        self.plot_position_setpoint_data.clear()
        self.plot_velocity_data.clear()
        self.plot_start_time = None
        
        # Clear plot lines
        self.line_position_setpoint.set_data([], [])
        self.line_position.set_data([], [])
        self.line_velocity.set_data([], [])
        self.plot_canvas.draw_idle()
        
        self.log_to_console("🧹 Plot data cleared")
    
    # Utility methods
    def log_to_console(self, message: str):
        """Add message to console output"""
        self.console_output.append(message)
        # Auto-scroll to bottom
        scrollbar = self.console_output.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
    
    async def cleanup(self):
        """Cleanup on application close"""
        self.telemetry_timer.stop()
        self.plot_timer.stop()
        if self.system:
            await self.system.shutdown()
    
    def closeEvent(self, event):
        """Handle window close event"""
        asyncio.ensure_future(self.cleanup())
        event.accept()


def main():
    """Main application entry point"""
    app = QApplication(sys.argv)
    
    # Create event loop
    loop = QEventLoop(app)
    asyncio.set_event_loop(loop)
    
    # Create and show main window
    window = ODriveGUI()
    window.show()
    
    with loop:
        loop.run_forever()


if __name__ == "__main__":
    main()
