import serial
import time
from lidar import record


# Pi connects to arduino controller via usb port

# Configuring the serial port
# '/dev/ttyACM0' is the port on the rasp pi 4 when connecting arduino uno r3 usb port
ser = serial.Serial(
    port='/dev/ttyS0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

print("UART Controller initialized")

try:
    print("\nArduino Robot Controller")
    print("-----------------------")
    print("Commands:")
    print("  f - Forward")
    print("  b - Backward")
    print("  l - Left")
    print("  r - Right")
    print("  s - Stop")
    print("  t - Brake")
    print("  w - Write LiDar Data")
    print("  e - End Writing LiDar")
    print("  q - Quit")
    
    while True:
        cmd = input("\nEnter command: ").lower()
        
        if cmd == 'q':
            break
        
        if cmd in ['f', 'b', 'l', 'r', 's', 't']:
            ser.write(cmd.encode())  # Send the character as bytes
            print(f"Sent command: {cmd}")
            time.sleep(0.1)  # Small delay between commands
        else:
            print("Invalid command")

except KeyboardInterrupt:
    print("\nProgram terminated by user")

finally:
    ser.close()  # Close the serial port
    print("UART connection closed")