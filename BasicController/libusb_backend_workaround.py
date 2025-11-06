import usb.core
import usb.backend.libusb1

def find_libusb_backend():
    backend = usb.backend.libusb1.get_backend(find_library=lambda x: ".\\.venv\\Lib\\site-packages\\libusb\\_platform\\_windows\\x64\\libusb-1.0.dll")
    if backend:
        print("libusb backend found at the specified path.")
    else:
        print("No libusb backend found at the specified path.")

    return backend

if __name__ == "__main__":
    find_libusb_backend()