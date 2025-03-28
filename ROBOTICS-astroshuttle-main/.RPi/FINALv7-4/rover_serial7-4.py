import serial
import time

# SETUP
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600

# OPEN SERIAL PORT
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2) 
    print(f"Serial connection established on {SERIAL_PORT}")
except Exception as e:
    print("Failed to open serial port:", e)
    exit()

def send_command(command):
    """Send a command string to the Arduino."""
    full_command = command.strip() + "\n"
    ser.write(full_command.encode('utf-8'))
    print("Sent:", command)

def read_response():
    """Read and print any available responses from the Arduino."""
    while ser.in_waiting:
        response = ser.readline().decode('utf-8').strip()
        if response:
            print("Arduino:", response)

def main():
    print("Enter a command for the rover (or type 'exit' to quit):")
    while True:
        user_input = input("> ").strip()
        if user_input.lower() == "exit":
            break
        send_command(user_input)
        time.sleep(0.2)
        read_response()

if __name__ == "__main__":
    main()
