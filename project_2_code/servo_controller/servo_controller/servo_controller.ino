#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 100 // Min pulse length count (≈1 ms)
#define SERVOMAX 570 // Max pulse length count (≈2 ms)
#define NEUTRAL_ANGLE 65
#define MAX_VALUES 3
#define BUF_SIZE 32

int values[MAX_VALUES];
char input_buf[BUF_SIZE];
int buf_idx = 0;

void moveServo(int motor, int angle) {
  angle = constrain(angle, 30, 100);
  int pulselen = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(motor, 0, pulselen);
}

void parseInput(char *buf) {
  int numValues = 0;
  char *token = strtok(buf, ",");
  while (token != NULL && numValues < MAX_VALUES) {
    values[numValues++] = atoi(token);
    token = strtok(NULL, ",");
  }

  // Apply servo movement only if all values received
  if (numValues == MAX_VALUES) {
    moveServo(0, values[0]);
    moveServo(1, values[1]);
    moveServo(2, values[2]);
  }

  // Optional: send ACK back to Python
  Serial.println("OK");
}

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);

  // Initialize servos to neutral
  moveServo(0, NEUTRAL_ANGLE);
  moveServo(1, NEUTRAL_ANGLE);
  moveServo(2, NEUTRAL_ANGLE);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();

    // If newline, parse buffer
    if (c == '\n') {
      input_buf[buf_idx] = '\0';
      parseInput(input_buf);
      buf_idx = 0; // reset buffer index
    } 
    // Otherwise, store character in buffer
    else if (buf_idx < BUF_SIZE - 1) {
      input_buf[buf_idx++] = c;
    }
  }

  // Optional: small delay to avoid saturating CPU
  delay(1);
}
