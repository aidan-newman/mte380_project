#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 100
#define SERVOMAX 570
#define NEUTRAL_ANGLE 65

#define MAX_VALUES 3
#define BUF_SIZE 32

char input_buf[BUF_SIZE];
int buf_idx = 0;
int values[MAX_VALUES];

void moveServo(int motor, int angle) {
  angle = constrain(angle, 30, 100);
  int pulselen = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(motor, 0, pulselen);
}

void parseLine() {
  input_buf[buf_idx] = '\0';      // Null-terminate the string
  buf_idx = 0;                    // Reset for next line

  int numValues = 0;
  char *token = strtok(input_buf, ",");

  while (token && numValues < MAX_VALUES) {
    values[numValues++] = atoi(token);
    token = strtok(NULL, ",");
  }

  if (numValues == MAX_VALUES) {
    moveServo(0, values[0]);
    moveServo(1, values[1]);
    moveServo(2, values[2]);
    Serial.println("OK");
  } else {
    Serial.println("BAD");
  }
}

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(50);

  moveServo(0, NEUTRAL_ANGLE);
  moveServo(1, NEUTRAL_ANGLE);
  moveServo(2, NEUTRAL_ANGLE);
}

void loop() {
  while (Serial.available() > 0) {

    char c = Serial.read();

    // Ignore carriage returns
    if (c == '\r') continue;

    // Line complete
    if (c == '\n') {
      parseLine();
      continue;
    }

    // Add to buffer safely
    if (buf_idx < BUF_SIZE - 1) {
      input_buf[buf_idx++] = c;
    } else {
      // Overflow → reset to avoid partial garbage
      buf_idx = 0;
    }
  }
}
