#!/usr/bin/python
# -*- coding:utf-8 -*-

import sys
import time
import threading
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import collections

# --- Add ADS1263 path ---
sys.path.append('/home/neon1/myenv/lib/python3.11/site-packages/ADS1263')
from ADS1263 import ADS1263

# --- User Settings ---
VREF = 5.0        # Reference voltage
DIFF_CHANNEL = 0  # Differential input channel
UPDATE_INTERVAL = 100  # Plot update rate in milliseconds (100ms = 10Hz)
WINDOW_SECONDS = 10    # How much history to keep on screen

# --- Initialize Data Storage ---
max_points = int(WINDOW_SECONDS * 1000 / UPDATE_INTERVAL)
time_data = collections.deque(maxlen=max_points)
voltage_data = collections.deque(maxlen=max_points)

# --- Initialize ADC ---
ADC = ADS1263()
if (ADC.ADS1263_init_ADC1('ADS1263_400SPS') == -1):
    print("Failed to initialize ADS1263.")
    exit()

ADC.ADS1263_SetMode(1)  # Differential mode

# --- Data Reading Thread ---
def read_adc():
    global start_time
    start_time = time.time()
    while True:
        raw_adc_value = ADC.ADS1263_GetChannalValue(DIFF_CHANNEL)
        if (raw_adc_value >> 31) == 1:
            voltage = (raw_adc_value - 2**32) * VREF / (2**31)
        else:
            voltage = raw_adc_value * VREF / (2**31 - 1)
        
        current_time = time.time() - start_time
        time_data.append(current_time)
        voltage_data.append(voltage)
        
        time.sleep(UPDATE_INTERVAL / 1000.0)

# --- Plotting Setup ---
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

# --- Start background ADC reading thread ---
adc_thread = threading.Thread(target=read_adc, daemon=True)
adc_thread.start()

# --- Start live plot animation ---
ani = animation.FuncAnimation(fig, update, init_func=init, blit=True, interval=UPDATE_INTERVAL)
plt.show()

# --- Cleanup on exit ---
ADC.ADS1263_Exit()
