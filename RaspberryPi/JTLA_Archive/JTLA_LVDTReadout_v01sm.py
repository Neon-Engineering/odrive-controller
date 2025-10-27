#!/usr/bin/python
# -*- coding:utf-8 -*-

import sys
import time

sys.path.append('/home/neon1/myenv/lib/python3.11/site-packages/ADS1263')

from ADS1263 import ADS1263


# --- User Settings ---
VREF = 5
DIFF_CHANNEL = 0

try:
    ADC = ADS1263()  # <-- Now clean

    if (ADC.ADS1263_init_ADC1('ADS1263_400SPS') == -1):
        print("Failed to initialize ADS1263.")
        exit()

    ADC.ADS1263_SetMode(1)

    print("Starting LVDT readout...")
    time.sleep(0.5)

    while True:
        raw_adc_value = ADC.ADS1263_GetChannalValue(DIFF_CHANNEL)

        if (raw_adc_value >> 31) == 1:
            voltage = (raw_adc_value - 2**32) * VREF / (2**31)
#         else:
#             voltage = raw_adc_value * VREF / (2**31 - 1)

        print(f"LVDT Channel {DIFF_CHANNEL}: {voltage:.6f} V")
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nKeyboard Interrupt detected, exiting cleanly...")
    ADC.ADS1263_Exit()
    exit()

except Exception as e:
    print(f"An error occurred: {e}")
    try:
        ADC.ADS1263_Exit()
    except:
        pass
    exit()
