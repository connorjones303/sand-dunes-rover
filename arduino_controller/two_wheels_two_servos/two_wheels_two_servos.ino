#include <SoftPWM.h>
#include <Servo.h>

// Define pins we want to use for PWM, 

const int SERVO_CHANNEL_1  = 7;
const int SERVO_CHANNEL_2 = 6;

// motor channel 1
const int MOTOR_CHANNEL_1_IN1 = 5;
const int MOTOR_CHANNEL_1_IN2 = 4;

// motor channel 2
const int MOTOR_CHANNEL_2_IN1 = 3;
const int MOTOR_CHANNEL_2_IN2 = 2;


// define servo class for using servos
Servo servo_channel_1;
Servo servo_channel_2;

// Speed settings (0-255)
const int DEFAULT_SPEED = 50;
const int TURN_SPEED = 30;

// servo baseline 'straight angle" value
// note: this will change based on how we attach physical motor housing to servo horn
const int SERVO_STRAIGHT = 45;
// servo turn margin angle
const int SERVO_MARGIN = 45;

const byte MOTOR_FADE_RATE = 10;

void setup()
{
  // ==== Pin setup ====
  // Initialize the SoftPWM library for manual PWM assignment (don't need to include servo pins)
  SoftPWMBegin();
  // Set up all motor control pins with SoftPWM
  // Parameters: pin, initial value (0-255), fade rate (optional)
  SoftPWMSet(MOTOR_CHANNEL_1_IN1, 0, MOTOR_FADE_RATE);
  SoftPWMSet(MOTOR_CHANNEL_1_IN2, 0, MOTOR_FADE_RATE);
  
  SoftPWMSet(MOTOR_CHANNEL_2_IN1, 0, MOTOR_FADE_RATE);
  SoftPWMSet(MOTOR_CHANNEL_2_IN2, 0, MOTOR_FADE_RATE);
  
  // Set fade time (optional, can be set to 0 for immediate response)
  SoftPWMSetFadeTime(ALL, 0, 0);
  // ===========================

  // ==== Serial communication setup ====
  // Start UART communication at 9600 baud, 9600 bits per second
  Serial.begin(9600);
  Serial.println("Arduino ready to receive UART commands");
  // ====================================

  // ==== Servo Setup ====
  servo_channel_1.attach(SERVO_CHANNEL_1); // attaches the servo on pin 13 to the servo object
  servo_channel_2.attach(SERVO_CHANNEL_2); // attaches the servo on pin 13 to the servo object
  // initially orient servos to 0 degrees
  servo_channel_1.write(SERVO_STRAIGHT);
  servo_channel_2.write(SERVO_STRAIGHT);
  // =====================

  // Initially stop motors
  stopMotors();
}

void loop()
{

  // Check for incoming UART data
  if (Serial.available() > 0)
  {
    char command = Serial.read(); // Read one character
    executeCommand(command);
    Serial.print("Command executed: ");
    Serial.println(command);
  }
}

// Execute motor commands
void executeCommand(char command)
{
  switch (command)
  {
  case 'f':
    forward();
    break;
  case 'b':
    backward();
    break;
  case 'l':
    left();
    break;
  case 'r':
    right();
    break;
  case 's':
    stopMotors();
    break;
  case 't':
    brakeMotors();
    break;
  default:
    break; // Ignore unknown commands
  }
}

// IN1  IN2  |  Motor State
// ---------------------
// 1    0    |  FORWARD (may be reversed, it's based of current direction, will always spin opposite of REVERSE)
// 0    1    |  REVERSE (may be reversed, it's based of current direction, will always spin opposite of FORWARD)
// 1    1    |  BRAKE
// 0    0    |  OFF (Coast)

// 1 = HIGH/ON (3.3V or 5V), 0 = LOW/OFF (0V or ground)

// Motor control functions

void forward()
{
  // Set servos to straight position (0 degrees)
  servo_channel_1.write(SERVO_STRAIGHT);
  servo_channel_2.write(SERVO_STRAIGHT);
  
  delay(50); // Short delay to allow servos to reach position

  // Motor channel 1 - Forward
  SoftPWMSet(MOTOR_CHANNEL_1_IN1, DEFAULT_SPEED);
  SoftPWMSet(MOTOR_CHANNEL_1_IN2, 0);
  
  // Motor channel 2 - Forward
  SoftPWMSet(MOTOR_CHANNEL_2_IN1, DEFAULT_SPEED);
  SoftPWMSet(MOTOR_CHANNEL_2_IN2, 0);
  
}

void backward()
{
  // Set servos to straight position (0 degrees)
  servo_channel_1.write(SERVO_STRAIGHT);
  servo_channel_2.write(SERVO_STRAIGHT);
  
  delay(50); // Short delay to allow servos to reach position

  // Motor channel 1  - Reverse
  SoftPWMSet(MOTOR_CHANNEL_1_IN1, 0);
  SoftPWMSet(MOTOR_CHANNEL_1_IN2, DEFAULT_SPEED);
  
  // Motor channel 2  - Reverse
  SoftPWMSet(MOTOR_CHANNEL_2_IN1, 0);
  SoftPWMSet(MOTOR_CHANNEL_2_IN2, DEFAULT_SPEED);
  
}
void left()
{
  // Configure servos for left turn
  // Servo channel 1 controls servo 1 (front left) and servo 4 (back right)
  // Servo channel 2 controls servo 2 (front right) and servo 3 (back left)
  
  // Turn servos in appropriate directions using the margin constant
  // For a left turn:
  // - Front wheels point left
  // - Back wheels point left
  servo_channel_1.write(SERVO_STRAIGHT - SERVO_MARGIN); // Front left and back right servos
  servo_channel_2.write(SERVO_STRAIGHT - SERVO_MARGIN); // Front right and back left servos
  
  delay(50); // Short delay to allow servos to reach position

  // Set motors to turn mode (typically at a reduced speed for better control)
  // Motor channel 2  - Forward
  SoftPWMSet(MOTOR_CHANNEL_1_IN1, TURN_SPEED);
  SoftPWMSet(MOTOR_CHANNEL_1_IN2, 0);
  
  // Motor channel 1 - Backwards
  SoftPWMSet(MOTOR_CHANNEL_2_IN1, 0);
  SoftPWMSet(MOTOR_CHANNEL_2_IN2, TURN_SPEED);

}

void right()
{
  // Configure servos for right turn
  // Servo channel 1 controls servo 1 (front left) and servo 4 (back right)
  // Servo channel 2 controls servo 2 (front right) and servo 3 (back left)
  
  // Turn servos in appropriate directions using the margin constant
  // For a right turn:
  // - Front wheels point right
  // - Back wheels point right
  servo_channel_1.write(SERVO_STRAIGHT + SERVO_MARGIN); // Front left and back right servos
  servo_channel_2.write(SERVO_STRAIGHT + SERVO_MARGIN); // Front right and back left servos
  
  delay(50); // Short delay to allow servos to reach position

  // Set motors to turn mode (typically at a reduced speed for better control)
  // Motor channel 1 - Backward
  SoftPWMSet(MOTOR_CHANNEL_2_IN1, TURN_SPEED);
  SoftPWMSet(MOTOR_CHANNEL_2_IN2, 0);
  
  // Motor channel 2  - Forward
  SoftPWMSet(MOTOR_CHANNEL_1_IN1, 0);
  SoftPWMSet(MOTOR_CHANNEL_1_IN2, TURN_SPEED);
}

void stopMotors()
{
  // Reset servos to straight position
  servo_channel_1.write(SERVO_STRAIGHT);
  servo_channel_2.write(SERVO_STRAIGHT);
  
  // Coast stop all motors (set both pins to 0)
  // Motor channel 1 (Front Left)
  SoftPWMSet(MOTOR_CHANNEL_1_IN1, 0);
  SoftPWMSet(MOTOR_CHANNEL_1_IN2, 0);
  
  // Motor channel 2 
  SoftPWMSet(MOTOR_CHANNEL_2_IN1, 0);
  SoftPWMSet(MOTOR_CHANNEL_2_IN2, 0);
  
}

void brakeMotors()
{
  // Reset servos to straight position
  servo_channel_1.write(SERVO_STRAIGHT);
  servo_channel_2.write(SERVO_STRAIGHT);
  
  // Active brake all motors (set both pins to 1)
  // Motor channel 1 
  SoftPWMSet(MOTOR_CHANNEL_1_IN1, 255);
  SoftPWMSet(MOTOR_CHANNEL_1_IN2, 255);
  
  // Motor channel 2
  SoftPWMSet(MOTOR_CHANNEL_2_IN1, 255);
  SoftPWMSet(MOTOR_CHANNEL_2_IN2, 255);
  
  // Short delay for active braking
  delay(100);
  
  // Then set to coast mode (both pins to 0) to avoid overheating the motors
  stopMotors();
}