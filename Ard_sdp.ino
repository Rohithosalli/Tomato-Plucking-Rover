#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVO_MIN 150
#define SERVO_MAX 600

// Motor pins
const int motor1Pin1 = 7, motor1Pin2 = 6, enable1Pin = 5;
const int motor2Pin1 = 4, motor2Pin2 = 3, enable2Pin = 2;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(60);

  pinMode(motor1Pin1, OUTPUT); pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT); pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT); pinMode(enable2Pin, OUTPUT);
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.startsWith("forward")) moveRover(true);
    else if (input.startsWith("stop")) stopRover();
    else handleServo(input);
  }
}

void moveRover(bool forward) {
  analogWrite(enable1Pin, 200); analogWrite(enable2Pin, 200);
  digitalWrite(motor1Pin1, forward); digitalWrite(motor1Pin2, !forward);
  digitalWrite(motor2Pin1, forward); digitalWrite(motor2Pin2, !forward);
}

void stopRover() {
  digitalWrite(motor1Pin1, LOW); digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW); digitalWrite(motor2Pin2, LOW);
}

void handleServo(String input) {
  int commaIndex = input.indexOf(',');
  int servo = input.substring(0, commaIndex).toInt();
  int angle = input.substring(commaIndex + 1).toInt();
  int pwmVal = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(servo, 0, pwmVal);
  Serial.println("OK");
}
