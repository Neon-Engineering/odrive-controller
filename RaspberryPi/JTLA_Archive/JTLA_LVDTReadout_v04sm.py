#!/usr/bin/python
# -*- coding:utf-8 -*-

import sys
import time
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import collections

# === Add ADS1263 driver path ===
sys.path.append('/home/neon1/myenv/lib/python3.11/site-packages/ADS1263')
from ADS1263 import ADS1263

# === Configuration ===
VREF = 5.0                # ADC reference voltage
DIFF_CHANNEL = 0          # Using AIN0–AIN1 differential pair
UPDATE_INTERVAL = 250     # Plot update rate in milliseconds
WINDOW_SECONDS = 10       # Plot window width in seconds

# === Data buffers ===
max_points = int(WINDOW_SECONDS * 1000 / UPDATE_INTERVAL)
time_data = collections.deque(maxlen=max_points)
voltage_data = collections.deque(maxlen=max_points)

# === Initialize ADS1263 ===
ADC = ADS1263()
if ADC.ADS1263_init_ADC1('ADS1263_400SPS') == -1:
    print("Failed to initialize ADS1263.")
    exit()

ADC.ADS1263_SetMode(1)  # Differential mode

# === Hardcoded calibration scaling ===
LVDT_v1 = -2.255	# Bottom end voltage [volt] (measured using JTLA_LVDTReadout_v03sm)
LVDT_x1 = -2.498	# Bottom end position [in] (measured using calipers)
LVDT_v2 = 2.001		# Top end voltage [volt] (measured using JTLA_LVDTReadout_v03sm)
LVDT_x2 = 2.457		# Top end position [in] (measured using calipers)
LVDT_x0 = .130 		# Scaled linear standoff position relative to v=0 position
LVDT_xHome = .188	# Scaled linear home position relative to standoff position

# === ADC read thread ===
def read_adc():
    global start_time
    start_time = time.time()
    while True:
        raw_adc_value = ADC.ADS1263_GetChannalValue(DIFF_CHANNEL)

        # Extract 24-bit signed value from 32-bit raw output
        value_24 = raw_adc_value >> 8
        if value_24 & 0x800000:
            signed_value = value_24 - (1 << 24)
        else:
            signed_value = value_24

        # Print raw voltage
        LVDT_voltage = signed_value * VREF / (2**23)
        print(f"LVDT voltage: {LVDT_voltage}")
        # Print position 
        LVDT_position = ((LVDT_v2-LVDT_v1)/(LVDT_x2-LVDT_x1))*LVDT_voltage + LVDT_x0 + LVDT_xHome
        print(f"LVDT position: {LVDT_position}")

        current_time = time.time() - start_time
        time_data.append(current_time)
        voltage_data.append(LVDT_voltage)
        time.sleep(UPDATE_INTERVAL / 1000.0)

# === Plot setup ===
fig, ax = plt.subplots()
line, = ax.plot([], [], lw=2)
ax.set_title('Live LVDT Voltage Readout')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Voltage (V)')
ax.grid(True)
ax.set_ylim(-3, 3)

def init():
    line.set_data([], [])
    return line,

def update(frame):
    line.set_data(time_data, voltage_data)
    if time_data:
        ax.set_xlim(max(0, time_data[-1] - WINDOW_SECONDS), time_data[-1])
    return line,

# === Start ADC thread ===
adc_thread = threading.Thread(target=read_adc, daemon=True)
adc_thread.start()

# === Start plot animation ===
ani = animation.FuncAnimation(
    fig, update, init_func=init, blit=True,
    interval=UPDATE_INTERVAL, save_count=1000, cache_frame_data=False
)

plt.show()

# === Cleanup on exit ===
ADC.ADS1263_Exit()
