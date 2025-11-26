/*
Gimbal Light Controller - Dual Gimbal System
Controls two independent gimbal systems with pendulum motion and lights
Designed for Arduino Mega with multiple servo outputs
*/

#include <Arduino.h>
#include <Servo.h>
#include <math.h>

// --- GIMBAL STRUCTURE ---
struct GimbalSystem {
    // Pin configuration
    uint8_t servo1Pin;
    uint8_t servo2Pin;
    uint8_t lightPin;

    // Servo objects
    Servo servo1;
    Servo servo2;
    bool servosEnabled;

    // Swing parameters - Bottom servo (main lateral swing)
    float swing2Amplitude;
    float swing2Period;
    float swing2Phase;

    // Swing parameters - Top servo (tilt from nadir)
    float swing1Amplitude;
    float swing1Period;
    float swing1Phase;

    // Center positions
    int servo1Center;
    int servo2Center;

    // Damping
    float dampingFactor;
    float dampingRate;

    // Motion control
    bool swingEnabled;
    unsigned long swingStartTime;

    // Light LFO
    bool lightEnabled;
    float lightLFOFreq;
    float lightLFODepth;
    int lightBrightness;
    unsigned long lightStartTime;
};

// Two gimbal systems
GimbalSystem gimbals[2];

// --- EASING FUNCTIONS ---
float pendulumMotion(unsigned long timeMs, float amplitude, float period, float phase) {
    float t = (float)timeMs / period;
    float sine = sin(2.0 * PI * t + phase);
    float sign = (sine >= 0) ? 1.0 : -1.0;
    return amplitude * sign * pow(abs(sine), 0.85);
}

// Initialize a gimbal with default values
void initGimbal(GimbalSystem &g, uint8_t s1Pin, uint8_t s2Pin, uint8_t lPin) {
    g.servo1Pin = s1Pin;
    g.servo2Pin = s2Pin;
    g.lightPin = lPin;
    g.servosEnabled = false;

    g.swing2Amplitude = 90.0;
    g.swing2Period = 3000.0;
    g.swing2Phase = 0.0;

    g.swing1Amplitude = 40.0;
    g.swing1Period = 3000.0;
    g.swing1Phase = 0.0;

    g.servo1Center = 90;
    g.servo2Center = 90;

    g.dampingFactor = 1.0;
    g.dampingRate = 0.0;

    g.swingEnabled = false;
    g.swingStartTime = 0;

    g.lightEnabled = false;
    g.lightLFOFreq = 1.0;
    g.lightLFODepth = 1.0;
    g.lightBrightness = 255;
    g.lightStartTime = 0;
}

// Update swing positions for a gimbal
void updateSwingPositions(GimbalSystem &g) {
    if (!g.swingEnabled || !g.servosEnabled) return;

    unsigned long elapsed = millis() - g.swingStartTime;

    float currentDamping = g.dampingFactor;
    if (g.dampingRate > 0) {
        currentDamping = g.dampingFactor * exp(-g.dampingRate * elapsed);
        if (currentDamping < 0.01) {
            g.swingEnabled = false;
            g.servo1.write(g.servo1Center);
            g.servo2.write(g.servo2Center);
            return;
        }
    }

    float swing2Offset = pendulumMotion(elapsed, g.swing2Amplitude * currentDamping, g.swing2Period, g.swing2Phase);
    int servo2Angle = g.servo2Center + (int)swing2Offset;
    servo2Angle = constrain(servo2Angle, 0, 180);

    float swingPosition = abs(swing2Offset) / (g.swing2Amplitude * currentDamping);
    float swing1Offset = g.swing1Amplitude * swingPosition;
    int servo1Angle = g.servo1Center - (int)swing1Offset;
    servo1Angle = constrain(servo1Angle, 0, 180);

    g.servo1.write(servo1Angle);
    g.servo2.write(servo2Angle);
}

// Update light LFO for a gimbal
void updateLight(GimbalSystem &g) {
    if (!g.lightEnabled) return;

    unsigned long elapsed = millis() - g.lightStartTime;
    float t = (float)elapsed / 1000.0;

    float lfo = 0.5 + 0.5 * sin(2.0 * PI * g.lightLFOFreq * t);
    int minBright = g.lightBrightness * (1.0 - g.lightLFODepth);
    int brightness = minBright + (int)((g.lightBrightness - minBright) * lfo);
    brightness = constrain(brightness, 0, 255);

    analogWrite(g.lightPin, brightness);
}

// Center servos for a gimbal
void centerServos(GimbalSystem &g, uint16_t durationMs = 1000) {
    if (!g.servosEnabled) return;

    g.swingEnabled = false;

    int start1 = g.servo1.read();
    int start2 = g.servo2.read();

    uint16_t steps = durationMs / 20;
    if (steps < 10) steps = 10;
    uint16_t dt = durationMs / steps;

    for (uint16_t i = 0; i <= steps; i++) {
        float t = (float)i / (float)steps;
        float k = 0.5f - 0.5f * cos(PI * t);

        int angle1 = start1 + (int)((g.servo1Center - start1) * k);
        int angle2 = start2 + (int)((g.servo2Center - start2) * k);

        g.servo1.write(angle1);
        g.servo2.write(angle2);
        delay(dt);
    }
}

// Start swing for a gimbal
void startSwing(GimbalSystem &g, int gimbalNum) {
    if (!g.servosEnabled) {
        Serial.print("G");
        Serial.print(gimbalNum);
        Serial.println(":SERVOS_NOT_ENABLED");
        return;
    }

    g.swingStartTime = millis();
    g.dampingFactor = 1.0;
    g.swingEnabled = true;
    Serial.print("G");
    Serial.print(gimbalNum);
    Serial.println(":SWING_STARTED");
}

// Stop swing for a gimbal
void stopSwing(GimbalSystem &g, int gimbalNum) {
    g.swingEnabled = false;
    centerServos(g, 500);
    Serial.print("G");
    Serial.print(gimbalNum);
    Serial.println(":SWING_STOPPED");
}

// Start light for a gimbal
void startLight(GimbalSystem &g, int gimbalNum) {
    g.lightStartTime = millis();
    g.lightEnabled = true;
    Serial.print("G");
    Serial.print(gimbalNum);
    Serial.println(":LIGHT_STARTED");
}

// Stop light for a gimbal
void stopLight(GimbalSystem &g, int gimbalNum) {
    g.lightEnabled = false;
    analogWrite(g.lightPin, 0);
    Serial.print("G");
    Serial.print(gimbalNum);
    Serial.println(":LIGHT_STOPPED");
}

// Process command for a specific gimbal
void processGimbalCommand(GimbalSystem &g, int gimbalNum, String command) {
    if (command.startsWith("SERVO_PINS:")) {
        String values = command.substring(11);
        int comma = values.indexOf(',');

        if (comma > 0) {
            g.servo1Pin = values.substring(0, comma).toInt();
            g.servo2Pin = values.substring(comma + 1).toInt();

            if (g.servo1.attached()) g.servo1.detach();
            if (g.servo2.attached()) g.servo2.detach();

            g.servo1.attach(g.servo1Pin);
            g.servo2.attach(g.servo2Pin);
            g.servosEnabled = true;

            g.servo1.write(g.servo1Center);
            g.servo2.write(g.servo2Center);

            Serial.print("G");
            Serial.print(gimbalNum);
            Serial.println(":SERVO_PINS_OK");
        }
    }
    else if (command.startsWith("SWING1:")) {
        String values = command.substring(7);
        int c1 = values.indexOf(',');
        int c2 = values.indexOf(',', c1 + 1);

        if (c1 > 0 && c2 > 0) {
            g.swing1Amplitude = values.substring(0, c1).toFloat();
            g.swing1Period = values.substring(c1 + 1, c2).toFloat();
            g.swing1Phase = values.substring(c2 + 1).toFloat();
            Serial.print("G");
            Serial.print(gimbalNum);
            Serial.println(":SWING1_OK");
        }
    }
    else if (command.startsWith("SWING2:")) {
        String values = command.substring(7);
        int c1 = values.indexOf(',');
        int c2 = values.indexOf(',', c1 + 1);

        if (c1 > 0 && c2 > 0) {
            g.swing2Amplitude = values.substring(0, c1).toFloat();
            g.swing2Period = values.substring(c1 + 1, c2).toFloat();
            g.swing2Phase = values.substring(c2 + 1).toFloat();
            Serial.print("G");
            Serial.print(gimbalNum);
            Serial.println(":SWING2_OK");
        }
    }
    else if (command.startsWith("CENTER:")) {
        String values = command.substring(7);
        int comma = values.indexOf(',');

        if (comma > 0) {
            g.servo1Center = values.substring(0, comma).toInt();
            g.servo2Center = values.substring(comma + 1).toInt();
            Serial.print("G");
            Serial.print(gimbalNum);
            Serial.println(":CENTER_OK");
        }
    }
    else if (command.startsWith("DAMPING:")) {
        g.dampingRate = command.substring(8).toFloat();
        Serial.print("G");
        Serial.print(gimbalNum);
        Serial.println(":DAMPING_OK");
    }
    else if (command.startsWith("LIGHT_PIN:")) {
        g.lightPin = command.substring(10).toInt();
        pinMode(g.lightPin, OUTPUT);
        Serial.print("G");
        Serial.print(gimbalNum);
        Serial.println(":LIGHT_PIN_OK");
    }
    else if (command.startsWith("LIGHT_LFO:")) {
        String values = command.substring(10);
        int c1 = values.indexOf(',');
        int c2 = values.indexOf(',', c1 + 1);

        if (c1 > 0 && c2 > 0) {
            g.lightLFOFreq = values.substring(0, c1).toFloat();
            g.lightLFODepth = values.substring(c1 + 1, c2).toFloat();
            g.lightBrightness = values.substring(c2 + 1).toInt();
            Serial.print("G");
            Serial.print(gimbalNum);
            Serial.println(":LIGHT_LFO_OK");
        }
    }
    else if (command == "LIGHT_ON") {
        startLight(g, gimbalNum);
    }
    else if (command == "LIGHT_OFF") {
        stopLight(g, gimbalNum);
    }
    else if (command.startsWith("BPM:")) {
        float bpm = command.substring(4).toFloat();
        if (bpm > 0) {
            float periodMs = 60000.0 / bpm;
            g.swing1Period = periodMs;
            g.swing2Period = periodMs * 1.15;
            Serial.print("G");
            Serial.print(gimbalNum);
            Serial.println(":BPM_OK");
        }
    }
    else if (command.startsWith("MOVE1:")) {
        int angle = command.substring(6).toInt();
        angle = map(angle, -360, 360, 0, 180);
        angle = constrain(angle, 0, 180);
        if (g.servosEnabled) {
            g.swingEnabled = false;
            g.servo1.write(angle);
            Serial.print("G");
            Serial.print(gimbalNum);
            Serial.println(":MOVE1_OK");
        }
    }
    else if (command.startsWith("MOVE2:")) {
        int angle = command.substring(6).toInt();
        angle = map(angle, -360, 360, 0, 180);
        angle = constrain(angle, 0, 180);
        if (g.servosEnabled) {
            g.swingEnabled = false;
            g.servo2.write(angle);
            Serial.print("G");
            Serial.print(gimbalNum);
            Serial.println(":MOVE2_OK");
        }
    }
    else if (command == "START") {
        startSwing(g, gimbalNum);
    }
    else if (command == "STOP") {
        stopSwing(g, gimbalNum);
    }
    else if (command == "ALL_ON") {
        startSwing(g, gimbalNum);
        startLight(g, gimbalNum);
    }
    else if (command == "ALL_OFF") {
        stopSwing(g, gimbalNum);
        stopLight(g, gimbalNum);
    }
    else if (command == "RESET") {
        g.swing2Amplitude = 90.0;
        g.swing2Period = 3000.0;
        g.swing2Phase = 0.0;
        g.swing1Amplitude = 20.0;
        g.swing1Period = 3000.0;
        g.swing1Phase = 0.0;
        g.servo1Center = 90;
        g.servo2Center = 90;
        g.dampingRate = 0.0;
        g.dampingFactor = 1.0;
        centerServos(g, 500);
        Serial.print("G");
        Serial.print(gimbalNum);
        Serial.println(":RESET_OK");
    }
    else if (command == "STATUS") {
        Serial.print("G");
        Serial.print(gimbalNum);
        Serial.print(":SERVO_PINS:");
        Serial.print(g.servo1Pin);
        Serial.print(",");
        Serial.println(g.servo2Pin);

        Serial.print("G");
        Serial.print(gimbalNum);
        Serial.print(":SWING_ENABLED:");
        Serial.println(g.swingEnabled ? "YES" : "NO");
    }
    else if (command.startsWith("PRESET:")) {
        String preset = command.substring(7);

        if (preset == "GENTLE") {
            g.swing2Amplitude = 90.0;
            g.swing2Period = 3000.0;
            g.swing1Amplitude = 20.0;
            g.swing1Period = 3000.0;
            g.dampingRate = 0.0;
            Serial.print("G");
            Serial.print(gimbalNum);
            Serial.println(":PRESET_GENTLE_OK");
        }
        else if (preset == "NORMAL") {
            g.swing2Amplitude = 60.0;
            g.swing2Period = 2000.0;
            g.swing1Amplitude = 15.0;
            g.swing1Period = 2000.0;
            g.dampingRate = 0.0;
            Serial.print("G");
            Serial.print(gimbalNum);
            Serial.println(":PRESET_NORMAL_OK");
        }
        else if (preset == "WILD") {
            g.swing2Amplitude = 90.0;
            g.swing2Period = 1500.0;
            g.swing1Amplitude = 30.0;
            g.swing1Period = 1500.0;
            g.dampingRate = 0.0;
            Serial.print("G");
            Serial.print(gimbalNum);
            Serial.println(":PRESET_WILD_OK");
        }
        else if (preset == "DYING") {
            g.swing2Amplitude = 75.0;
            g.swing2Period = 2000.0;
            g.swing1Amplitude = 20.0;
            g.swing1Period = 2000.0;
            g.dampingRate = 0.0003;
            Serial.print("G");
            Serial.print(gimbalNum);
            Serial.println(":PRESET_DYING_OK");
        }
        else if (preset == "CIRCULAR") {
            g.swing2Amplitude = 40.0;
            g.swing2Period = 2000.0;
            g.swing2Phase = 0.0;
            g.swing1Amplitude = 40.0;
            g.swing1Period = 2000.0;
            g.swing1Phase = 1.5708;
            g.dampingRate = 0.0;
            Serial.print("G");
            Serial.print(gimbalNum);
            Serial.println(":PRESET_CIRCULAR_OK");
        }
        else {
            Serial.print("G");
            Serial.print(gimbalNum);
            Serial.println(":UNKNOWN_PRESET");
        }
    }
    else {
        Serial.print("G");
        Serial.print(gimbalNum);
        Serial.println(":UNKNOWN_COMMAND");
    }
}

// Process incoming serial commands
void processCommand(String command) {
    command.trim();

    // Check for gimbal-specific commands (G1: or G2:)
    if (command.startsWith("G1:")) {
        processGimbalCommand(gimbals[0], 1, command.substring(3));
    }
    else if (command.startsWith("G2:")) {
        processGimbalCommand(gimbals[1], 2, command.substring(3));
    }
    // Legacy commands without prefix - apply to gimbal 1
    else {
        processGimbalCommand(gimbals[0], 1, command);
    }
}

void setup() {
    // Initialize gimbals with default pins
    initGimbal(gimbals[0], 33, 45, 9);   // Gimbal 1
    initGimbal(gimbals[1], 35, 47, 10);  // Gimbal 2

    // Set pins to OUTPUT and LOW
    for (int i = 0; i < 2; i++) {
        pinMode(gimbals[i].servo1Pin, OUTPUT);
        pinMode(gimbals[i].servo2Pin, OUTPUT);
        pinMode(gimbals[i].lightPin, OUTPUT);
        digitalWrite(gimbals[i].servo1Pin, LOW);
        digitalWrite(gimbals[i].servo2Pin, LOW);
        analogWrite(gimbals[i].lightPin, 0);
    }

    // Set PWM frequency on pins 9 and 10 to ~3.9kHz
    TCCR2B = (TCCR2B & 0b11111000) | 0x02;

    delay(2000);

    Serial.begin(9600);
    Serial.setTimeout(100);

    // Attach and center servos for both gimbals
    for (int i = 0; i < 2; i++) {
        gimbals[i].servo1.attach(gimbals[i].servo1Pin);
        gimbals[i].servo2.attach(gimbals[i].servo2Pin);
        gimbals[i].servosEnabled = true;
        gimbals[i].servo1.write(gimbals[i].servo1Center);
        gimbals[i].servo2.write(gimbals[i].servo2Center);
    }

    delay(1000);

    Serial.println("DUAL_GIMBAL_CONTROLLER_READY");
    Serial.println("Commands: G1:START | G2:START | G1:STOP | G2:STOP");
    Serial.println("Config: G1:SERVO_PINS:33,45 | G2:SERVO_PINS:35,47");
}

void loop() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        processCommand(command);
    }

    // Update both gimbals
    for (int i = 0; i < 2; i++) {
        updateSwingPositions(gimbals[i]);
        updateLight(gimbals[i]);
    }

    delay(10);
}
