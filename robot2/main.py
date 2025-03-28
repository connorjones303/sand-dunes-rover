import serial
import time
from imu import MPU6050


# connect pi to arduino as serial connection
ser = serial.Serial(
    port='/dev/ttyS0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

print("UART Controller initialized")

# init imu sensor
sensor = MPU6050(bus_num=1)
sensor.initialize()

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
            print(sensor.get_sensor_data()) # print sensor data before motor command
            ser.write(cmd.encode())  # Send the character as bytes
            print(f"Sent command: {cmd}")
            time.sleep(0.1)  # Small delay between commands
            print(sensor.get_sensor_data()) # print sensor data after motor command
            time.sleep(0.1)
        else:
            print("Invalid command")

except KeyboardInterrupt:
    print("\nProgram terminated by user")

finally:
    ser.close()  # Close the serial port
    print("UART connection closed")