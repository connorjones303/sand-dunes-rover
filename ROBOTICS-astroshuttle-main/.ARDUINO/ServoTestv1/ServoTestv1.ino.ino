#include <Servo.h>

// -------------------------------
//       PIN ASSIGNMENTS
// -------------------------------
// DC Motor Pins (using PWM)
// Left DC motors (mounted normally)
const int LEFT_MOTOR_F_PIN = 6;  // forward channel for left motors
const int LEFT_MOTOR_R_PIN = 9;  // reverse channel for left motors
// Right DC motors (mounted inverted)
// For the right side, “forward” is achieved by activating the reverse channel.
const int RIGHT_MOTOR_F_PIN = 3; // forward channel for right motors (used for reverse drive)
const int RIGHT_MOTOR_R_PIN = 5; // reverse channel for right motors (used for forward drive)

// Servo Pins (steering)
// Two servos per side are driven as a group.
const int LEFT_SERVO_PIN  = 11;  // left-side servos
const int RIGHT_SERVO_PIN = 10;  // right-side servos

// -------------------------------
//      SPEED & STEERING SETTINGS
// -------------------------------
const int DEFAULT_SPEED = 160;   // normal driving speed (0-255)
const int TURN_SPEED    = 160;   // speed for in-place (skid) turns
const int SLOW_SPEED    = 120;   // reduced speed for differential turning

// Servo parameters (assumes 180° servos)
const int SERVO_CENTER      = 90;  // Center position (can be calibrated)
const int SERVO_TURN_ANGLE  = 30;  // Amount to turn for steering commands

// -------------------------------
//         SERVO OBJECTS
// -------------------------------
Servo leftServo;
Servo rightServo;

// -------------------------------
//          SETUP
// -------------------------------
void setup() {
  Serial.begin(9600);
  
  // Setup motor pins as outputs
  pinMode(LEFT_MOTOR_F_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_R_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_F_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_R_PIN, OUTPUT);
  
  // Attach servos
  leftServo.attach(LEFT_SERVO_PIN);
  rightServo.attach(RIGHT_SERVO_PIN);
  
  // Initialize servos to center
  zeroSteer();
  
  // Ensure motors are stopped
  stopMotors();
  
  // Instruction message
  Serial.println("Rover Control Ready.");
  Serial.println("Commands:");
  Serial.println(" skidL, skidR, steerL, steerR, driveF, driveB, driveL, driveR, zeroSteer, S");
}

// -------------------------------
//           MAIN LOOP
// -------------------------------
void loop() {
  // Read a full command string from Serial (ended by newline)
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Remove whitespace/newline
    
    // Execute based on the command string
    if (command == "skidL") {
      skidL();
    } else if (command == "skidR") {
      skidR();
    } else if (command == "steerL") {
      steerL();
    } else if (command == "steerR") {
      steerR();
    } else if (command == "driveF") {
      driveF();
    } else if (command == "driveB") {
      driveB();
    } else if (command == "driveL") {
      sdriveL();  // Use servo-assisted differential drive left
    } else if (command == "driveR") {
      sdriveR();  // Use servo-assisted differential drive right
    } else if (command == "zeroSteer") {
      zeroSteer();
    } else if (command == "S") {
      stopMotors();
    } else {
      Serial.print("Unknown command: ");
      Serial.println(command);
    }
    
    Serial.print("Executed command: ");
    Serial.println(command);
  }
}

// -------------------------------
//       MOTOR CONTROL FUNCTIONS
// -------------------------------

// Stop all motors
void stopMotors() {
  analogWrite(LEFT_MOTOR_F_PIN, 0);
  analogWrite(LEFT_MOTOR_R_PIN, 0);
  analogWrite(RIGHT_MOTOR_F_PIN, 0);
  analogWrite(RIGHT_MOTOR_R_PIN, 0);
}

// Drive forward (motors only)
// Left motors drive forward: (pin6 = DEFAULT_SPEED, pin9 = 0)
// Right motors drive forward (inverted): (pin3 = 0, pin5 = DEFAULT_SPEED)
void driveF() {
  analogWrite(LEFT_MOTOR_F_PIN, DEFAULT_SPEED);
  analogWrite(LEFT_MOTOR_R_PIN, 0);
  
  analogWrite(RIGHT_MOTOR_F_PIN, 0);
  analogWrite(RIGHT_MOTOR_R_PIN, DEFAULT_SPEED);
}

// Drive backward (motors only)
// Left motors backward: (pin6 = 0, pin9 = DEFAULT_SPEED)
// Right motors backward (inverted): (pin3 = DEFAULT_SPEED, pin5 = 0)
void driveB() {
  analogWrite(LEFT_MOTOR_F_PIN, 0);
  analogWrite(LEFT_MOTOR_R_PIN, DEFAULT_SPEED);
  
  analogWrite(RIGHT_MOTOR_F_PIN, DEFAULT_SPEED);
  analogWrite(RIGHT_MOTOR_R_PIN, 0);
}

// Skid steer turn in place to the left:
// Left motors go backward, right motors go forward.
void skidL() {
  analogWrite(LEFT_MOTOR_F_PIN, 0);
  analogWrite(LEFT_MOTOR_R_PIN, TURN_SPEED);   // left motors backward
  
  analogWrite(RIGHT_MOTOR_F_PIN, 0);
  analogWrite(RIGHT_MOTOR_R_PIN, TURN_SPEED);  // right motors forward (since inverted)
}

// Skid steer turn in place to the right:
// Left motors go forward, right motors go backward.
void skidR() {
  analogWrite(LEFT_MOTOR_F_PIN, DEFAULT_SPEED);
  analogWrite(LEFT_MOTOR_R_PIN, 0);   // left motors forward
  
  analogWrite(RIGHT_MOTOR_F_PIN, DEFAULT_SPEED);
  analogWrite(RIGHT_MOTOR_R_PIN, 0);  // right motors backward (since inverted)
}

// Differential drive functions (motors only)
// DdriveL: for a left turn, left side goes slower than right.
void DdriveL() {
  analogWrite(LEFT_MOTOR_F_PIN, SLOW_SPEED);
  analogWrite(LEFT_MOTOR_R_PIN, 0);
  
  analogWrite(RIGHT_MOTOR_F_PIN, 0);
  analogWrite(RIGHT_MOTOR_R_PIN, DEFAULT_SPEED);
}

// DdriveR: for a right turn, right side goes slower than left.
void DdriveR() {
  analogWrite(LEFT_MOTOR_F_PIN, DEFAULT_SPEED);
  analogWrite(LEFT_MOTOR_R_PIN, 0);
  
  analogWrite(RIGHT_MOTOR_F_PIN, 0);
  analogWrite(RIGHT_MOTOR_R_PIN, SLOW_SPEED);
}

// -------------------------------
//    SERVO-ASSISTED DRIVE FUNCTIONS
// -------------------------------

// sdriveL: Turn left using servos plus a slight differential drive
void sdriveL() {
  // Set servos to steer left:
  // (Assuming that increasing the left servo angle and decreasing the right servo angle turns the wheels left)
  leftServo.write(SERVO_CENTER + SERVO_TURN_ANGLE);
  rightServo.write(SERVO_CENTER - SERVO_TURN_ANGLE);
  
  // Differential drive: left side runs slower than right
  analogWrite(LEFT_MOTOR_F_PIN, SLOW_SPEED);
  analogWrite(LEFT_MOTOR_R_PIN, 0);
  
  analogWrite(RIGHT_MOTOR_F_PIN, 0);
  analogWrite(RIGHT_MOTOR_R_PIN, DEFAULT_SPEED);
}

// sdriveR: Turn right using servos plus a slight differential drive
void sdriveR() {
  // Set servos to steer right:
  leftServo.write(SERVO_CENTER - SERVO_TURN_ANGLE);
  rightServo.write(SERVO_CENTER + SERVO_TURN_ANGLE);
  
  // Differential drive: right side runs slower than left
  analogWrite(LEFT_MOTOR_F_PIN, DEFAULT_SPEED);
  analogWrite(LEFT_MOTOR_R_PIN, 0);
  
  analogWrite(RIGHT_MOTOR_F_PIN, 0);
  analogWrite(RIGHT_MOTOR_R_PIN, SLOW_SPEED);
}

// -------------------------------
//         SERVO STEERING FUNCTIONS
// -------------------------------

// steerL: Turn all servos to steer left (servos only)
void steerL() {
  leftServo.write(SERVO_CENTER + SERVO_TURN_ANGLE);
  rightServo.write(SERVO_CENTER - SERVO_TURN_ANGLE);
}

// steerR: Turn all servos to steer right (servos only)
void steerR() {
  leftServo.write(SERVO_CENTER - SERVO_TURN_ANGLE);
  rightServo.write(SERVO_CENTER + SERVO_TURN_ANGLE);
}

// zeroSteer: Center the servos to the middle of their arc
void zeroSteer() {
  leftServo.write(SERVO_CENTER);
  rightServo.write(SERVO_CENTER);
}
