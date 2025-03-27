#!/usr/bin/env python3
"""
Servo Control Program
This script reads single characters from standard input ('l', 'r', 's')
and controls a servo motor connected to GPIO pin 17 using PWM.
'l' - Turn servo left
'r' - Turn servo right
's' - Center servo (straight)
"""

import RPi.GPIO as GPIO
import sys
import time

# Configuration
SERVO_PIN = 17
FREQUENCY = 50  # Standard frequency for servos is 50Hz

# Servo position constants (duty cycle values)
# Adjust these values based on your specific servo
LEFT_POSITION = 2.5    # Fully left position (typically 0 degrees)
RIGHT_POSITION = 12.5  # Fully right position (typically 180 degrees)
CENTER_POSITION = 7.5  # Center position (typically 90 degrees)

def setup():
    """Setup GPIO and PWM for servo control"""
    # Use BCM pin numbering
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    
    # Initialize PWM
    pwm = GPIO.PWM(SERVO_PIN, FREQUENCY)
    
    # Start with center position
    pwm.start(CENTER_POSITION)
    
    return pwm

def move_servo(pwm, direction):
    """
    Move the servo based on the input direction
    
    Args:
        pwm: PWM object
        direction: 'l' for left, 'r' for right, 's' for straight/center
    """
    if direction.lower() == 'l':
        pwm.ChangeDutyCycle(LEFT_POSITION)
        print("Moving servo LEFT")
    elif direction.lower() == 'r':
        pwm.ChangeDutyCycle(RIGHT_POSITION)
        print("Moving servo RIGHT")
    elif direction.lower() == 's':
        pwm.ChangeDutyCycle(CENTER_POSITION)
        print("Moving servo to CENTER (straight)")
    else:
        # Ignore unrecognized commands
        print(f"Unrecognized command: '{direction}'. Use 'l', 'r', or 's'")

def main():
    """Main program function"""
    try:
        pwm = setup()
        print("Servo Control Ready!")
        print("Enter commands: 'l' (left), 'r' (right), 's' (straight/center)")
        print("Press Ctrl+C to exit")
        
        # Initial delay to allow servo to center
        time.sleep(0.5)
        
        while True:
            # Read a single character from stdin
            char = sys.stdin.read(1)
            
            if char:  # If we actually got a character
                move_servo(pwm, char)
                time.sleep(0.3)  # Small delay to allow servo to move
                
    except KeyboardInterrupt:
        print("\nExiting program")
    finally:
        # Clean up
        if 'pwm' in locals():
            pwm.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    main()