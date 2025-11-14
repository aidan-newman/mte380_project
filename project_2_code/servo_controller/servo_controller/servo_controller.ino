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
  int angles[3];
  int angle1 = NEUTRAL_ANGLE;
  int angle2 = NEUTRAL_ANGLE;
  int angle3 = NEUTRAL_ANGLE;

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // Read until newline
    numValues = 0;

    // Split by commas
    char buf[input.length() + 1];
    input.toCharArray(buf, sizeof(buf));
    char *token = strtok(buf, ",");
    while (token != NULL && numValues < MAX_VALUES) {
      values[numValues++] = atoi(token);
      token = strtok(NULL, ",");
    }
  }

    // Print what we got
    Serial.print("Received ");
    Serial.print(numValues);
    Serial.println(" values:");
    for (int i = 0; i < numValues; i++) {
      Serial.println(values[i]);
  }


  if(values[0] && values[1] && values[2]) {
    moveServo(0, values[0]);
    moveServo(1, values[1]);
    moveServo(2, values[2]);
  }

  delay(1);
}
