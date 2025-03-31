import serial
import time

from robot2.logger import logger # Nathan W: Implement logging (logs and prints to console in one statement)

PORT = '/dev/tty.HC-06'
BAUD_RATE = 9600
TIMEOUT = 1

def initialize_serial(port=PORT, baud_rate=BAUD_RATE, timeout=TIMEOUT): # Nathan W: Put this block of code in a function like other files
    try:
        ser = serial.Serial(port, baud_rate, timeout)
        ser.flush()
        logger.info("Connected")
        return ser
    except Exception as e:
        logger.critical(f"Error opening serial port: {e}")
        exit(1)

def send_message(ser, message):
    try:
        ser.write(message.encode())
        logger.info(f"Sent: {message}")
    except Exception as e:
        logger.critical(f"Error sending message: {e}")
        ser.close()
        exit(1)

def read_response(ser):
    try:
        time.sleep(2)
        response = ser.read_all().decode(errors='replace')
        logger.info(f"Received: {response}")
        return response
    except Exception as e:
        logger.critical(f"Error reading response: {e}")
        ser.close()
        exit(1)

def main(): # Nathan W: Added main method like other test files
    ser = initialize_serial()

    send_message(ser, "test\n")
    read_response(ser)

    ser.close()

if __name__ == "__main__":
    main()
