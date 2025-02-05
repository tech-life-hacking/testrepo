import subprocess
import gpiozero
import time
import argparse

def setup_gpio(boot_pin, reset_pin):
    """Initialize GPIO pins"""
    bootPin = gpiozero.DigitalOutputDevice(pin=boot_pin)
    resetPin = gpiozero.DigitalOutputDevice(pin=reset_pin)
    return bootPin, resetPin

def enter_bootloader(bootPin, resetPin):
    """Enter bootloader mode on STM32"""
    print("Entering bootloader mode...")
    bootPin.on()  # Set BOOT0 pin to High
    time.sleep(0.1)  # Wait for BOOT0 pin to stabilize
    resetPin.off()  # Set RESET pin to Low (reset)
    time.sleep(0.2)  # Hold RESET signal
    resetPin.on()  # Set RESET pin to High (release reset)
    time.sleep(1)  # Wait for bootloader mode to initialize
    print("STM32 is now in bootloader mode.")

def wait_for_device(device, timeout=5):
    """Wait until the device is ready to respond"""
    print(f"Waiting for {device} to be ready...")
    for _ in range(timeout):
        try:
            with open(device, "r"):
                print(f"{device} is ready.")
                return True
        except OSError:
            time.sleep(1)
    print(f"Timeout waiting for {device}.")
    return False

def flash_firmware(bin_file, device, baudrate):
    """Flash the firmware and monitor progress"""
    print("Flashing firmware...")
    command = [
        "stm32flash",
        "-w", bin_file,
        "-v",
        "-g", "0x08000000",
        "-b", str(baudrate),
        device
    ]

    # Run `stm32flash` asynchronously
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    success = False

    try:
        while True:
            # Read progress line by line
            output = process.stdout.readline()
            if output:
                print(output.strip())  # Display output in real-time
                if "Done." in output:
                    success = True
                    break  # Exit the loop on success

            # Exit the loop if the process has finished
            if process.poll() is not None:
                break

        # Check for "Done." in output to confirm success
        if success:
            print("Firmware flashed successfully!")
        else:
            stderr = process.stderr.read()
            print(f"Error during firmware flashing: {stderr.strip()}")

    finally:
        process.stdout.close()
        process.stderr.close()

    return success

def run_application(bootPin, resetPin):
    """Run the application mode on STM32"""
    print("Running application...")
    bootPin.off()  # Set BOOT0 pin to Low
    resetPin.off()  # Set RESET pin to Low (reset)
    time.sleep(0.1)
    resetPin.on()  # Set RESET pin to High (release reset)
    print("STM32 is now running the application.")

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="STM32 Firmware Update Tool")
    parser.add_argument("--bin", required=True, help="Path to the firmware binary file")
    parser.add_argument("--device", required=True, help="UART device (e.g., /dev/ttyAMA0)")
    parser.add_argument("--baudrate", type=int, default=115200, help="Baud rate for stm32flash (default: 115200)")
    parser.add_argument("--boot-pin", type=int, default=17, help="GPIO pin for BOOT0 (default: 17)")
    parser.add_argument("--reset-pin", type=int, default=27, help="GPIO pin for RESET (default: 27)")
    args = parser.parse_args()

    # Initialize GPIO
    bootPin, resetPin = setup_gpio(args.boot_pin, args.reset_pin)

    try:
        # Switch to bootloader mode
        enter_bootloader(bootPin, resetPin)

        # Check if the device is ready
        if not wait_for_device(args.device):
            print("Device not ready. Aborting.")
            return

        # Flash the firmware
        success = flash_firmware(args.bin, args.device, args.baudrate)

        # Only switch to application mode if the update succeeded
        if success:
            print("Firmware update succeeded.")
            run_application(bootPin, resetPin)
        else:
            print("Firmware update failed.")
    finally:
        # Clean up GPIO resources
        bootPin.close()
        resetPin.close()

if __name__ == "__main__":
    main()
