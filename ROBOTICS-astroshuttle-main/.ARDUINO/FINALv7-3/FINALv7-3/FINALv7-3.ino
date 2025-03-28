#include <Servo.h>
#include <SoftPWM.h>

// ••••••••••••••••••••••••••• DC MOTOR PINS •••••••••••••••••••••••••••
// DC MOTOR - LEFT side
const int LEFT_MOTOR_B_PIN = 6;  // forward channel
const int LEFT_MOTOR_F_PIN = 9;  // reverse channel
// DC MOTOR - RIGHT side
const int RIGHT_MOTOR_B_PIN = 5; // forward channel
const int RIGHT_MOTOR_F_PIN = 3; // reverse channel

// ••••••••••••••••••••••••••• SPEEDS(0–255) •••••••••••••••••••••••••••
const int DEFAULT_SPEED = 160; 
const int TURN_SPEED    = 160;  

// ••••••••••••••••••••••••••• SERVO PINS •••••••••••••••••••••••••••
// Channel A: fR + bL
// Channel B: fL + bR
const int SERVO_FR_PIN = 11;  // fR
const int SERVO_FL_PIN = 10;  // fL
const int SERVO_BL_PIN = 12;  // bL
const int SERVO_BR_PIN = 13;  // bR

// ••••••••••••••••••••••••••• SERVO ANGLES •••••••••••••••••••••••••••
const int SERVO_CENTER       = 90;  // all servos centered when straight
const int SERVO_TURN_ANGLE   = 30;  // standard turn offset for non-pivot commands
const int PIVOT_SERVO_OFFSET = 30;  // offset for pivoting (wheels turn inward)

// ••••••••••••••••••••••••••• SERVO OBJECTS •••••••••••••••••••••••••••
Servo servoA1; // Front Right (Channel A)
Servo servoA2; // Back Left (Channel A)
Servo servoB1; // Front Left (Channel B)
Servo servoB2; // Back Right (Channel B)

// ••••••••••••••••••••••••••••••••••••••••••••••••••••••
//                       SETUP
// ••••••••••••••••••••••••••••••••••••••••••••••••••••••
void setup() {
  Serial.begin(9600);
  pinMode(LEFT_MOTOR_F_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_B_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_F_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_B_PIN, OUTPUT);
  SoftPWMBegin();
  softPwmCreate(LEFT_MOTOR_F_PIN, 0, 255);
  softPwmCreate(LEFT_MOTOR_B_PIN, 0, 255);
  softPwmCreate(RIGHT_MOTOR_F_PIN, 0, 255);
  softPwmCreate(RIGHT_MOTOR_B_PIN, 0, 255);
  servoA1.attach(SERVO_FR_PIN);
  servoB1.attach(SERVO_FL_PIN);
  servoA2.attach(SERVO_BL_PIN);
  servoB2.attach(SERVO_BR_PIN);
  
  steerZ();
  stopMotors();
  
  Serial.println("•••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••");
  Serial.println("•                                       Oberon INITIALIZED                                          •");
  Serial.println("•••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••");
  Serial.println("•                                                                                                   •");
  Serial.println("•    skidL, skidR, driveF, driveB, driveL, driveR, DdriveL, DriveR, pivotL, pivotR, steerL, steerR, steerZ, S   •");
  Serial.println("•                                                                                                   •");
  Serial.println("•••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••");
  Serial.println(" 'C' to show these commands again ");
}

// ••••••••••••••••••••••••••••••••••••••••••••••••••••••
//                   MAIN PROGRAM LOOP
// ••••••••••••••••••••••••••••••••••••••••••••••••••••••
void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "skidL") {
      skidL();
    } else if (command == "skidR") {
      skidR();
    } else if (command == "driveF") {
      driveF();
    } else if (command == "driveB") {
      driveB();
    } else if (command == "driveL") {
      driveL();
    } else if (command == "driveR") {
      driveR();
    } else if (command == "DdriveL") {
      DdriveL();
    } else if (command == "DriveR") {
      DriveR();
    } else if (command == "pivotL") {
      pivotL();
    } else if (command == "pivotR") {
      pivotR();
    } else if (command == "steerL") {
      steerL();
    } else if (command == "steerR") {
      steerR();
    } else if (command == "steerZ") {
      steerZ();
    } else if (command == "S") {
      stopMotors();
    } else if (command == "C") {
      showCommands();
    } else {
      Serial.print("Unknown command: ");
      Serial.println(command);
    }
    
    Serial.print("Executed command: ");
    Serial.println(command);
  }
}
void showCommands() {
  Serial.println("•••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••");
  Serial.println("•    skidL, skidR, driveF, driveB, driveL, driveR, DdriveL, DriveR, pivotL, pivotR, steerL, steerR, steerZ, S, C    •");
  Serial.println("•••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••••");
}
// ••••••••••••••••••••••••••••••••••••••••••••••••••••••
//              MOTOR CONTROL FUNCTIONS
// ••••••••••••••••••••••••••••••••••••••••••••••••••••••
void stopMotors() {
  softPwmWrite(LEFT_MOTOR_F_PIN, 0);
  softPwmWrite(LEFT_MOTOR_B_PIN, 0);
  softPwmWrite(RIGHT_MOTOR_F_PIN, 0);
  softPwmWrite(RIGHT_MOTOR_B_PIN, 0);
}

void driveF() {
  softPwmWrite(LEFT_MOTOR_F_PIN, DEFAULT_SPEED);
  softPwmWrite(LEFT_MOTOR_B_PIN, 0);
  softPwmWrite(RIGHT_MOTOR_F_PIN, DEFAULT_SPEED);
  softPwmWrite(RIGHT_MOTOR_B_PIN, 0);
}

void driveB() {
  softPwmWrite(LEFT_MOTOR_F_PIN, 0);
  softPwmWrite(LEFT_MOTOR_B_PIN, DEFAULT_SPEED);
  softPwmWrite(RIGHT_MOTOR_F_PIN, 0);
  softPwmWrite(RIGHT_MOTOR_B_PIN, DEFAULT_SPEED);
}

void skidL() {
  softPwmWrite(LEFT_MOTOR_F_PIN, 0);
  softPwmWrite(LEFT_MOTOR_B_PIN, TURN_SPEED);
  softPwmWrite(RIGHT_MOTOR_F_PIN, TURN_SPEED);
  softPwmWrite(RIGHT_MOTOR_B_PIN, 0);
}

void skidR() {
  softPwmWrite(LEFT_MOTOR_F_PIN, TURN_SPEED);
  softPwmWrite(LEFT_MOTOR_B_PIN, 0);
  softPwmWrite(RIGHT_MOTOR_F_PIN, 0);
  softPwmWrite(RIGHT_MOTOR_B_PIN, TURN_SPEED);
}

// *********************************************************
//             DC + SERVO FUNCTIONS
// *********************************************************
void driveL() {
  steerL();
  driveF();
}

void driveR() {
  steerR();
  driveF();
}

void DdriveL() {
  steerL();
  softPwmWrite(LEFT_MOTOR_F_PIN, SLOW_SPEED);
  softPwmWrite(LEFT_MOTOR_B_PIN, 0);
  softPwmWrite(RIGHT_MOTOR_F_PIN, DEFAULT_SPEED);
  softPwmWrite(RIGHT_MOTOR_B_PIN, 0);
}

void DriveR() {
  steerR();
  softPwmWrite(LEFT_MOTOR_F_PIN, DEFAULT_SPEED);
  softPwmWrite(LEFT_MOTOR_B_PIN, 0);
  softPwmWrite(RIGHT_MOTOR_F_PIN, SLOW_SPEED);
  softPwmWrite(RIGHT_MOTOR_B_PIN, 0);
}

// ••••••••••••••••••••••••••••••••••••••••••••••••••••••
//                  PIVOT FUNCTIONS
// ••••••••••••••••••••••••••••••••••••••••••••••••••••••
void pivotL() {
  // For pivot left: make servos point inward
  // Channel A (FR & BL) turn left; Channel B (FL & BR) turn right
  servoA1.write(SERVO_CENTER - PIVOT_SERVO_OFFSET);
  servoA2.write(SERVO_CENTER - PIVOT_SERVO_OFFSET);
  servoB1.write(SERVO_CENTER + PIVOT_SERVO_OFFSET);
  servoB2.write(SERVO_CENTER + PIVOT_SERVO_OFFSET);
  
  // Skid-style pivot: left motors reverse, right motors forward
  softPwmWrite(LEFT_MOTOR_F_PIN, 0);
  softPwmWrite(LEFT_MOTOR_B_PIN, TURN_SPEED);
  softPwmWrite(RIGHT_MOTOR_F_PIN, TURN_SPEED);
  softPwmWrite(RIGHT_MOTOR_B_PIN, 0);
}

void pivotR() {
  // For pivot right: make servos point inward in opposite direction
  // Channel A (FR & BL) turn right; Channel B (FL & BR) turn left
  servoA1.write(SERVO_CENTER + PIVOT_SERVO_OFFSET);
  servoA2.write(SERVO_CENTER + PIVOT_SERVO_OFFSET);
  servoB1.write(SERVO_CENTER - PIVOT_SERVO_OFFSET);
  servoB2.write(SERVO_CENTER - PIVOT_SERVO_OFFSET);
  
  // Skid-style pivot: left motors forward, right motors reverse
  softPwmWrite(LEFT_MOTOR_F_PIN, TURN_SPEED);
  softPwmWrite(LEFT_MOTOR_B_PIN, 0);
  softPwmWrite(RIGHT_MOTOR_F_PIN, 0);
  softPwmWrite(RIGHT_MOTOR_B_PIN, TURN_SPEED);
}

// ••••••••••••••••••••••••••••••••••••••••••••••••••••••
//              SERVO CONTROL FUNCTIONS
// ••••••••••••••••••••••••••••••••••••••••••••••••••••••
void steerL() {
  servoA1.write(SERVO_CENTER - SERVO_TURN_ANGLE);
  servoA2.write(SERVO_CENTER - SERVO_TURN_ANGLE);
  servoB1.write(SERVO_CENTER + SERVO_TURN_ANGLE);
  servoB2.write(SERVO_CENTER + SERVO_TURN_ANGLE);
}

void steerR() {
  servoA1.write(SERVO_CENTER + SERVO_TURN_ANGLE);
  servoA2.write(SERVO_CENTER + SERVO_TURN_ANGLE);
  servoB1.write(SERVO_CENTER - SERVO_TURN_ANGLE);
  servoB2.write(SERVO_CENTER - SERVO_TURN_ANGLE);
}

void steerZ() {
  servoA1.write(SERVO_CENTER);
  servoA2.write(SERVO_CENTER);
  servoB1.write(SERVO_CENTER);
  servoB2.write(SERVO_CENTER);
}
