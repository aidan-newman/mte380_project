#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 100 // Min pulse length count (≈1 ms)
#define SERVOMAX 570 // Max pulse length count (≈2 ms)

void moveServo(int motor, int angle) {
  angle = constrain(angle, 30, 100);
  int pulselen = map(angle, 0, 180, 100, 570);
  pwm.setPWM(motor, 0, pulselen);
}


void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);
}

void loop() {
  for (float angle = 0; angle <= 2*3.14; angle += 3.14/32) {
    Serial.println(sin(angle));
    Serial.println(map(sin(angle)*100, -100, 100, 30, 100));

    moveServo(0, map(sin(angle)*100, -100, 100, 30, 100));
    moveServo(1, map(sin(angle+3.14/4)*100, -100, 100, 30, 100));
    moveServo(2, map(sin(angle+3.12/2)*100, -100, 100, 30, 100));
    delay(10);
  }
}

