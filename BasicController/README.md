
# ODrive Repository

Most of the scripts have been obtained from ODrive's documentation and examples pages.

We are currently testing the CAN communications with the ODrive S1 M83525s kit, through the USB-CAN adapter.

The config.json file was obtained after setting up the ODrive S1 through the web GUI and exported from the "Apply & Calibrate" tab into a json.

flat_endpoints.json was obtained from https://docs.odriverobotics.com/releases/firmware and assumed that the firmware version on the ODrive S1 was 0.6.11-1. If this is not the current firmware, we can always download the correct firmware for it.

odrive_demo_xxxx.py files are meant for direct USB connection and not CAN interface.

# CAN Interface

## libusb Issue Workaround

The libusb issue can be worked around by using this code snippet (change the path by finding your specific libusb dll path and pasting it into the line for backend with a path specified)

    import usb.core
    import usb.backend.libusb1

    backend = usb.backend.libusb1.get_backend(find_library=lambda x: ".\\.venv\\Lib\\site-packages\\libusb\\_platform\\_windows\\x64\\libusb-1.0.dll")
    if backend:
        print("libusb backend found at the specified path.")
    else:
        print("No libusb backend found at the specified path.")

## To-do (2025-10-08)

Run CAN scripts from Python to communicate with the ODrive for control.

1. ~~Run can_restore_config.py to ensure that we can read the config file into the ODrive (current task).~~
2. ~~Run can_calibrate.py to ensure that we are able to run the ODrive calibration process from Python.~~
3.  Run can_simple.py (DONE!)
4. Run anticogging_try.py when complete to try anticogging function. Tuning will likely be necessary.

## Anticogging

We are currently working on using the ODrive experimental anticogging function to minimize the cogging on the M8325s motor supplied with the kit. Once we sufficiently understand how to tune and calibrate the anticogging for this motor we may be able to implement it into future motor controls.

## Control

When we are ready to move into controlling the motor, the link to the ODrive documentation for controls can be found at: https://docs.odriverobotics.com/v/latest/manual/control.html

The ODrive can be controlled using can_runner.py via CLI or GUI/mainGUI.py via GUI.
