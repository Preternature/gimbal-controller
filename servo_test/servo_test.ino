/*
  Servo Test - New Gimbal
  Base servo: pin 52
  Camera servo: pin 36

  Serial commands:
    'b'         - sweep base servo
    'c'         - sweep camera servo
    'a'         - sweep both
    'n'         - center both
    'B<angle>'  - set base to angle (0-180), e.g. B90
    'C<angle>'  - set camera to angle (0-180), e.g. C90
    'BUS<us>'   - set base via microseconds, e.g. BUS1500
    'CUS<us>'   - set camera via microseconds, e.g. CUS1500
*/

#include <Servo.h>

Servo baseServo;
Servo cameraServo;

const int BASE_PIN   = 52;
const int CAMERA_PIN = 36;

// Pulse width ranges - widen camera range for full travel
const int BASE_MIN_US   = 500;
const int BASE_MAX_US   = 2500;
const int CAM_MIN_US    = 500;
const int CAM_MAX_US    = 2500;

void sweepServo(Servo &s, const char *name) {
    Serial.print("Sweeping ");
    Serial.println(name);
    for (int angle = 0; angle <= 180; angle += 3) {
        s.write(angle);
        delay(30);
    }
    for (int angle = 180; angle >= 0; angle -= 3) {
        s.write(angle);
        delay(30);
    }
    s.write(90);
    Serial.print(name);
    Serial.println(" done");
}

void sweepServoUS(Servo &s, const char *name, int minUs, int maxUs) {
    Serial.print("Sweeping ");
    Serial.print(name);
    Serial.print(" (");
    Serial.print(minUs);
    Serial.print("-");
    Serial.print(maxUs);
    Serial.println("us)");
    for (int us = minUs; us <= maxUs; us += 20) {
        s.writeMicroseconds(us);
        delay(20);
    }
    for (int us = maxUs; us >= minUs; us -= 20) {
        s.writeMicroseconds(us);
        delay(20);
    }
    s.writeMicroseconds((minUs + maxUs) / 2);
    Serial.print(name);
    Serial.println(" done");
}

void setup() {
    Serial.begin(9600);
    Serial.setTimeout(100);

    baseServo.attach(BASE_PIN, BASE_MIN_US, BASE_MAX_US);
    cameraServo.attach(CAMERA_PIN, CAM_MIN_US, CAM_MAX_US);

    baseServo.write(90);
    cameraServo.write(90);
    delay(1000);

    Serial.println("=== Servo Test Ready ===");
    Serial.println("B<angle>  C<angle>  - set by angle (0-180)");
    Serial.println("BUS<us>   CUS<us>   - set by microseconds");
    Serial.println("b / c / a           - sweep  |  n = center");
}

void loop() {
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) return;

        if (line.startsWith("BUS")) {
            int us = constrain(line.substring(3).toInt(), 400, 2600);
            baseServo.writeMicroseconds(us);
            Serial.print("Base -> "); Serial.print(us); Serial.println("us");

        } else if (line.startsWith("CUS")) {
            int us = constrain(line.substring(3).toInt(), 400, 2600);
            cameraServo.writeMicroseconds(us);
            Serial.print("Camera -> "); Serial.print(us); Serial.println("us");

        } else if (line.startsWith("B")) {
            int angle = constrain(line.substring(1).toInt(), 0, 180);
            baseServo.write(angle);
            Serial.print("Base -> "); Serial.print(angle); Serial.println("deg");

        } else if (line.startsWith("C")) {
            int angle = constrain(line.substring(1).toInt(), 0, 180);
            cameraServo.write(angle);
            Serial.print("Camera -> "); Serial.print(angle); Serial.println("deg");

        } else if (line == "b") {
            sweepServoUS(baseServo, "Base", BASE_MIN_US, BASE_MAX_US);

        } else if (line == "c") {
            sweepServoUS(cameraServo, "Camera", CAM_MIN_US, CAM_MAX_US);

        } else if (line == "a") {
            sweepServoUS(baseServo, "Base", BASE_MIN_US, BASE_MAX_US);
            delay(300);
            sweepServoUS(cameraServo, "Camera", CAM_MIN_US, CAM_MAX_US);

        } else if (line == "n") {
            baseServo.writeMicroseconds(1500);
            cameraServo.writeMicroseconds(1500);
            Serial.println("Both centered at 1500us");

        } else if (line == "P") {
            Serial.println("GIMBAL_READY");
        }
    }
}
