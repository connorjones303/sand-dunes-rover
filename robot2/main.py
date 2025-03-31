import serial
import time

from logger import logger  # Nathan W: Implement logging (logs and prints to console in one statement)

# Pi connects to arduino controller via usb port

# Nathan W: Set up variables as it was done in other files
PORT = '/dev/ttyS0' # '/dev/ttyACM0' is the port on the rasp pi 4 when connecting arduino uno r3 usb port
BAUD_RATE = 9600
TIMEOUT = 1

# Configuring the serial port
def initialize_serial(port=PORT, baudrate=BAUD_RATE, timeout=TIMEOUT): # Nathan W: Put this block of code in a function like other files
    try: # Nathan W: Added a try/catch for serial connection to exit gracefully
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=timeout
        )
        logger.info("UART Controller initialized")
        return ser
    except Exception as e:
        logger.critical(f"Error establishing serial connection: {e}")
        exit(1)

def main():
    ser = serial.Serial()

    try:
        # Nathan W: Changed the control scheme to be more intuitive (Game-like controls)
        print("\nArduino Robot Controller")
        print("-----------------------")
        print("Commands:")
        print("  w - Forward")
        print("  s - Backward")
        print("  a - Left")
        print("  d - Right")
        print("  k - Stop")
        print("  l - Brake")
        print("  q - Quit")

        while True:
            cmd = input("\nEnter command: ").lower()

            if cmd == 'q':
                break

            if cmd in ['w', 's', 'a', 'd', 'k', 'l']:  # Nathan W: Changed the control scheme to be more intuitive (Game-like controls)
                ser.write(cmd.encode())  # Send the character as bytes
                logger.info(f"Sent command: {cmd}")
                time.sleep(0.1)  # Small delay between commands
            else:
                logger.error("Invalid command")

    except KeyboardInterrupt:
        logger.info("\nProgram terminated by user")

    finally:
        ser.close()  # Close the serial port
        logger.info("UART connection closed")

if __name__ == '__main__':
    main()
