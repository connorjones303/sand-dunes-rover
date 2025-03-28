// === Motor Pins ===
// Motor A (Left)
const int MOTOR_A_IN1 = 3;  // PWM pin
const int MOTOR_A_IN2 = 5;  // PWM pin
// Motor B (Right, mounted in reverse)
const int MOTOR_B_IN1 = 6;  // PWM pin
const int MOTOR_B_IN2 = 9;  // PWM pin

// === Speed Settings ===
const int DEFAULT_SPEED = 160;
const int TURN_SPEED = 120;

void setup() {
  Serial.begin(9600);

  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);

  stopMotors();

  Serial.println("Skid-Steer Mode Ready. Send f/b/l/r/s/t");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    switch (command) {
      case 'f': forward(); break;
      case 'b': backward(); break;
      case 'l': left(); break;
      case 'r': right(); break;
      case 's': stopMotors(); break;
      case 't': brakeMotors(); break;
      default:
        Serial.print("Unknown command: ");
        Serial.println(command);
        return;
    }

    Serial.print("Executed: ");
    Serial.println(command);
  }
}

// === Movement Functions ===

// LEFT motor forward: IN1 PWM, IN2 LOW
// RIGHT motor forward: IN1 LOW, IN2 PWM (because motor is reversed)

void forward() {
  // Left Motor (normal forward)
  analogWrite(MOTOR_A_IN1, DEFAULT_SPEED);
  analogWrite(MOTOR_A_IN2, 0);

  // Right Motor (reversed forward)
  analogWrite(MOTOR_B_IN1, 0);
  analogWrite(MOTOR_B_IN2, DEFAULT_SPEED);
}

void backward() {
  // Left Motor (normal backward)
  analogWrite(MOTOR_A_IN1, 0);
  analogWrite(MOTOR_A_IN2, DEFAULT_SPEED);

  // Right Motor (reversed backward)
  analogWrite(MOTOR_B_IN1, DEFAULT_SPEED);
  analogWrite(MOTOR_B_IN2, 0);
}

void left() {
  // Left Motor backward
  analogWrite(MOTOR_A_IN1, 0);
  analogWrite(MOTOR_A_IN2, TURN_SPEED);

  // Right Motor backward (reversed forward)
  analogWrite(MOTOR_B_IN1, 0);
  analogWrite(MOTOR_B_IN2, TURN_SPEED);
}

void right() {
  // Left Motor forward
  analogWrite(MOTOR_A_IN1, TURN_SPEED);
  analogWrite(MOTOR_A_IN2, 0);

  // Right Motor forward (reversed backward)
  analogWrite(MOTOR_B_IN1, TURN_SPEED);
  analogWrite(MOTOR_B_IN2, 0);
}

void stopMotors() {
  analogWrite(MOTOR_A_IN1, 0);
  analogWrite(MOTOR_A_IN2, 0);
  analogWrite(MOTOR_B_IN1, 0);
  analogWrite(MOTOR_B_IN2, 0);
}

void brakeMotors() {
  // Apply equal voltage to both IN1 and IN2 for braking
  analogWrite(MOTOR_A_IN1, DEFAULT_SPEED);
  analogWrite(MOTOR_A_IN2, DEFAULT_SPEED);
  analogWrite(MOTOR_B_IN1, DEFAULT_SPEED);
  analogWrite(MOTOR_B_IN2, DEFAULT_SPEED);
}
