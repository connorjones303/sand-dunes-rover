// #include <NewPing.h>

// #define TRIGGER_PIN 9    // Ultrasonic sensor trigger pin
// #define ECHO_PIN 10      // Ultrasonic sensor echo pin
// #define MAX_DISTANCE 200 // Maximum sensing distance in cm
// #define MOTOR_A_EN 5
#define MOTOR_A_IN1 5
#define MOTOR_A_IN2 6
// #define MOTOR_B_EN 3
#define MOTOR_B_IN1 10
#define MOTOR_B_IN2 11

// NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

#define DEFAULT_SPEED 80 // (0-255 int value)

void setup()
{

  Serial.begin(9600);

  // pinMode(MOTOR_A_EN, OUTPUT);
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  // pinMode(MOTOR_B_EN, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);

  // Initially stop both motors
  stopMotors();
}

void loop()
{
  // unsigned int distance = sonar.ping_cm();

  // if (distance > 0 && distance <= 30.48)
  // {
  //   turnLeft();
  //   driveForward(30.48); // Drive forward 30.48 cm
  //   straighten();
  // }
  // else
  // {
  //   driveForward(); // Continue driving forward
  // }

  driveForward();
  // Delay for 2000 milliseconds (2 seconds)
  delay(1000);

  turnLeft();

  delay(1000);

  stopMotors();

  delay(1000);
}

void driveForward()
{
  // turn both wheels forward
  analogWrite(MOTOR_A_IN1, DEFAULT_SPEED);
  analogWrite(MOTOR_A_IN2, 0);
  analogWrite(MOTOR_B_IN1, DEFAULT_SPEED);
  analogWrite(MOTOR_B_IN2, 0);

  // analogWrite(MOTOR_A_EN, 255);
  // digitalWrite(MOTOR_A_IN1, HIGH);
  // digitalWrite(MOTOR_A_IN2, LOW);
  // analogWrite(MOTOR_B_EN, 255);
  // digitalWrite(MOTOR_B_IN1, HIGH);
  // digitalWrite(MOTOR_B_IN2, LOW);
}

// void driveForward(float cm)
// {
//   float wheelCircumference = 6.2 * PI; // Circumference of the wheel
//   float rotations = cm / wheelCircumference;
//   float duration = rotations * 1000; // Approximation of driving time

//   driveForward();
//   delay(duration);
//   stopMotors();
// }

void turnLeft()
{
  // turn one wheel forward
  analogWrite(MOTOR_A_IN1, 0);
  analogWrite(MOTOR_A_IN2, DEFAULT_SPEED);

  // turn one wheel backwards
  analogWrite(MOTOR_B_IN1, DEFAULT_SPEED);
  analogWrite(MOTOR_B_IN2, 0);

  // analogWrite(MOTOR_A_EN, 255);
  // digitalWrite(MOTOR_A_IN1, LOW);
  // digitalWrite(MOTOR_A_IN2, HIGH);
  // analogWrite(MOTOR_B_EN, 255);
  // digitalWrite(MOTOR_B_IN1, HIGH);
  // digitalWrite(MOTOR_B_IN2, LOW);
  delay(400); // Adjust delay for a 30-degree turn
  stopMotors();
}

void straighten()
{

  // analogWrite(MOTOR_A_EN, 255);
  // digitalWrite(MOTOR_A_IN1, HIGH);
  // digitalWrite(MOTOR_A_IN2, LOW);
  // analogWrite(MOTOR_B_EN, 255);
  // digitalWrite(MOTOR_B_IN1, HIGH);
  // digitalWrite(MOTOR_B_IN2, LOW);
  // delay(400); // Adjust delay to re-align straight
  // stopMotors();
}

void stopMotors()
{
  analogWrite(MOTOR_A_IN1, 0);
  analogWrite(MOTOR_A_IN2, 0);
  analogWrite(MOTOR_B_IN1, 0);
  analogWrite(MOTOR_B_IN2, 0);

  // analogWrite(MOTOR_A_EN, 0);
  // digitalWrite(MOTOR_A_IN1, LOW);
  // digitalWrite(MOTOR_A_IN2, LOW);
  // analogWrite(MOTOR_B_EN, 0);
  // digitalWrite(MOTOR_B_IN1, LOW);
  // digitalWrite(MOTOR_B_IN2, LOW);
}
