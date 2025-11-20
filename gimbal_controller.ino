/*
Gimbal Light Controller - Double Servo Swinging Light
Creates pendulum-like motion to emulate a swinging light on a rope
Designed for high-mA red light powered by battery
*/

#include <Arduino.h>
#include <Servo.h>
#include <math.h>

// --- PIN CONFIGURATION ---
uint8_t servo1Pin = 26;  // Primary swing axis (pitch) - top gimbal
uint8_t servo2Pin = 48;  // Secondary swing axis (roll) - bottom gimbal
uint8_t lightPin = 9;    // Red light PWM pin

// --- SERVO SETUP ---
Servo servo1;  // Primary pendulum motion
Servo servo2;  // Secondary motion for realistic swing
bool servosEnabled = false;

// --- SWING PARAMETERS ---
// Bottom servo (servo2) - main lateral pendulum swing
float swing2Amplitude = 90.0;    // Degrees from center - the big side-to-side swing
float swing2Period = 3000.0;     // Period in milliseconds
float swing2Phase = 0.0;         // Phase offset in radians

// Top servo (servo1) - tilt from nadir following the swing arc
float swing1Amplitude = 40.0;    // Degrees from nadir - follows swing position
float swing1Period = 3000.0;     // Same period (derived from swing2)
float swing1Phase = 0.0;         // Not directly used - follows swing2 position

// Center positions
int servo1Center = 90;
int servo2Center = 90;

// Damping for wind-down effect
float dampingFactor = 1.0;       // 1.0 = full swing, 0.0 = stopped
float dampingRate = 0.0;         // Per-millisecond decay rate (0 = no damping)

// Motion control
bool swingEnabled = false;
unsigned long swingStartTime = 0;

// --- LIGHT LFO PARAMETERS ---
bool lightEnabled = false;
float lightLFOFreq = 1.0;        // LFO frequency in Hz
float lightLFODepth = 1.0;       // 0.0 to 1.0 (how much brightness varies)
int lightBrightness = 255;       // Base brightness (0-255)
int lightMin = 0;                // Minimum brightness during LFO
unsigned long lightStartTime = 0;

// --- EASING FUNCTIONS ---
// Modified sine wave - faster at extremes, slightly slower through center
float pendulumMotion(unsigned long timeMs, float amplitude, float period, float phase) {
    float t = (float)timeMs / period;
    float sine = sin(2.0 * PI * t + phase);
    // Compress the extremes by reducing the "hang time" at peaks
    // Using a slight power curve to speed through the ends
    float sign = (sine >= 0) ? 1.0 : -1.0;
    return amplitude * sign * pow(abs(sine), 0.85);
}

// Calculate current servo positions based on swing parameters
void updateSwingPositions() {
    if (!swingEnabled || !servosEnabled) return;

    unsigned long elapsed = millis() - swingStartTime;

    // Apply damping if configured
    float currentDamping = dampingFactor;
    if (dampingRate > 0) {
        currentDamping = dampingFactor * exp(-dampingRate * elapsed);
        if (currentDamping < 0.01) {
            // Swing has effectively stopped
            swingEnabled = false;
            servo1.write(servo1Center);
            servo2.write(servo2Center);
            Serial.println("SWING_STOPPED");
            return;
        }
    }

    // Calculate bottom servo position (servo2 - the main pendulum swing, lateral arc)
    float swing2Offset = pendulumMotion(elapsed, swing2Amplitude * currentDamping, swing2Period, swing2Phase);
    int servo2Angle = servo2Center + (int)swing2Offset;
    servo2Angle = constrain(servo2Angle, 0, 180);

    // Calculate top servo position (servo1 - subtle tilt that follows the swing)
    // Top servo stays near nadir (servo1Center) with small movements based on swing position
    // Uses absolute value of swing to create a "dip" at the extremes
    float swingPosition = abs(swing2Offset) / (swing2Amplitude * currentDamping);  // 0 at center, 1 at extremes
    float swing1Offset = swing1Amplitude * swingPosition;  // No extra damping - already in swingPosition
    int servo1Angle = servo1Center - (int)swing1Offset;  // Subtract to tilt UP from nadir
    servo1Angle = constrain(servo1Angle, 0, 180);

    // Write to servos
    servo1.write(servo1Angle);
    servo2.write(servo2Angle);
}

// Move both servos to center position smoothly
void centerServos(uint16_t durationMs = 1000) {
    if (!servosEnabled) return;

    swingEnabled = false;  // Stop any current swing

    // Get current positions (approximate from last write)
    int start1 = servo1.read();
    int start2 = servo2.read();

    uint16_t steps = durationMs / 20;
    if (steps < 10) steps = 10;
    uint16_t dt = durationMs / steps;

    for (uint16_t i = 0; i <= steps; i++) {
        float t = (float)i / (float)steps;
        // Smooth easing
        float k = 0.5f - 0.5f * cos(PI * t);

        int angle1 = start1 + (int)((servo1Center - start1) * k);
        int angle2 = start2 + (int)((servo2Center - start2) * k);

        servo1.write(angle1);
        servo2.write(angle2);
        delay(dt);
    }

    Serial.println("CENTERED");
}

// Start swing motion
void startSwing() {
    if (!servosEnabled) {
        Serial.println("SERVOS_NOT_ENABLED");
        return;
    }

    swingStartTime = millis();
    dampingFactor = 1.0;
    swingEnabled = true;
    Serial.println("SWING_STARTED");
}

// Stop swing motion
void stopSwing() {
    swingEnabled = false;
    centerServos(500);
    Serial.println("SWING_STOPPED");
}

// Update light LFO
void updateLight() {
    if (!lightEnabled) return;

    unsigned long elapsed = millis() - lightStartTime;
    float t = (float)elapsed / 1000.0;  // Time in seconds

    // Calculate LFO value (0 to 1)
    float lfo = 0.5 + 0.5 * sin(2.0 * PI * lightLFOFreq * t);

    // Apply depth: interpolate between min and max brightness
    int minBright = lightBrightness * (1.0 - lightLFODepth);
    int brightness = minBright + (int)((lightBrightness - minBright) * lfo);
    brightness = constrain(brightness, 0, 255);

    analogWrite(lightPin, brightness);
}

// Start light with LFO
void startLight() {
    lightStartTime = millis();
    lightEnabled = true;
    Serial.println("LIGHT_STARTED");
}

// Stop light
void stopLight() {
    lightEnabled = false;
    analogWrite(lightPin, 0);
    Serial.println("LIGHT_STOPPED");
}

// Process incoming serial commands
void processCommand(String command) {
    command.trim();

    if (command.startsWith("SERVO_PINS:")) {
        // SERVO_PINS:9,10
        String values = command.substring(11);
        int comma = values.indexOf(',');

        if (comma > 0) {
            servo1Pin = values.substring(0, comma).toInt();
            servo2Pin = values.substring(comma + 1).toInt();

            // Reattach servos
            if (servo1.attached()) servo1.detach();
            if (servo2.attached()) servo2.detach();

            servo1.attach(servo1Pin);
            servo2.attach(servo2Pin);
            servosEnabled = true;

            servo1.write(servo1Center);
            servo2.write(servo2Center);

            Serial.println("SERVO_PINS_OK");
        }
    }
    else if (command.startsWith("SWING1:")) {
        // SWING1:amplitude,period,phase
        // Example: SWING1:45,2000,0
        String values = command.substring(7);
        int c1 = values.indexOf(',');
        int c2 = values.indexOf(',', c1 + 1);

        if (c1 > 0 && c2 > 0) {
            swing1Amplitude = values.substring(0, c1).toFloat();
            swing1Period = values.substring(c1 + 1, c2).toFloat();
            swing1Phase = values.substring(c2 + 1).toFloat();
            Serial.println("SWING1_OK");
        }
    }
    else if (command.startsWith("SWING2:")) {
        // SWING2:amplitude,period,phase
        String values = command.substring(7);
        int c1 = values.indexOf(',');
        int c2 = values.indexOf(',', c1 + 1);

        if (c1 > 0 && c2 > 0) {
            swing2Amplitude = values.substring(0, c1).toFloat();
            swing2Period = values.substring(c1 + 1, c2).toFloat();
            swing2Phase = values.substring(c2 + 1).toFloat();
            Serial.println("SWING2_OK");
        }
    }
    else if (command.startsWith("CENTER:")) {
        // CENTER:90,90
        String values = command.substring(7);
        int comma = values.indexOf(',');

        if (comma > 0) {
            servo1Center = values.substring(0, comma).toInt();
            servo2Center = values.substring(comma + 1).toInt();
            Serial.println("CENTER_OK");
        }
    }
    else if (command.startsWith("DAMPING:")) {
        // DAMPING:0.001 (decay rate per ms, 0 = no damping)
        dampingRate = command.substring(8).toFloat();
        Serial.println("DAMPING_OK");
    }
    else if (command.startsWith("LIGHT_PIN:")) {
        // LIGHT_PIN:9
        lightPin = command.substring(10).toInt();
        pinMode(lightPin, OUTPUT);
        Serial.println("LIGHT_PIN_OK");
    }
    else if (command.startsWith("LIGHT_LFO:")) {
        // LIGHT_LFO:freq,depth,brightness
        // Example: LIGHT_LFO:2.0,0.5,255
        String values = command.substring(10);
        int c1 = values.indexOf(',');
        int c2 = values.indexOf(',', c1 + 1);

        if (c1 > 0 && c2 > 0) {
            lightLFOFreq = values.substring(0, c1).toFloat();
            lightLFODepth = values.substring(c1 + 1, c2).toFloat();
            lightBrightness = values.substring(c2 + 1).toInt();
            Serial.println("LIGHT_LFO_OK");
        }
    }
    else if (command == "LIGHT_ON") {
        startLight();
    }
    else if (command == "LIGHT_OFF") {
        stopLight();
    }
    else if (command.startsWith("BPM:")) {
        // BPM:30 - sets swing period from BPM (beats per minute)
        float bpm = command.substring(4).toFloat();
        if (bpm > 0) {
            float periodMs = 60000.0 / bpm;  // Convert BPM to period in ms
            swing1Period = periodMs;
            swing2Period = periodMs * 1.15;  // Secondary slightly offset
            Serial.println("BPM_OK");
        }
    }
    else if (command.startsWith("MOVE1:")) {
        // MOVE1:0 - directly move servo 1 to position (for calibration)
        // Accepts -360 to 360, maps to 0-180 for standard servos
        int angle = command.substring(6).toInt();
        angle = map(angle, -360, 360, 0, 180);
        angle = constrain(angle, 0, 180);
        if (servosEnabled) {
            swingEnabled = false;  // Stop swing during calibration
            servo1.write(angle);
            Serial.println("MOVE1_OK");
        }
    }
    else if (command.startsWith("MOVE2:")) {
        // MOVE2:0 - directly move servo 2 to position (for calibration)
        // Accepts -360 to 360, maps to 0-180 for standard servos
        int angle = command.substring(6).toInt();
        angle = map(angle, -360, 360, 0, 180);
        angle = constrain(angle, 0, 180);
        if (servosEnabled) {
            swingEnabled = false;  // Stop swing during calibration
            servo2.write(angle);
            Serial.println("MOVE2_OK");
        }
    }
    else if (command == "START") {
        startSwing();
    }
    else if (command == "STOP") {
        stopSwing();
    }
    else if (command == "ALL_ON") {
        startSwing();
        startLight();
    }
    else if (command == "ALL_OFF") {
        stopSwing();
        stopLight();
    }
    else if (command == "RESET") {
        // Full reset to defaults
        swing2Amplitude = 90.0;   // Bottom servo - main lateral swing
        swing2Period = 3000.0;
        swing2Phase = 0.0;
        swing1Amplitude = 20.0;   // Top servo - tilt from nadir
        swing1Period = 3000.0;
        swing1Phase = 0.0;
        servo1Center = 90;
        servo2Center = 90;
        dampingRate = 0.0;
        dampingFactor = 1.0;
        centerServos(500);
        Serial.println("RESET_OK");
    }
    else if (command == "STATUS") {
        Serial.print("SERVO_PINS:");
        Serial.print(servo1Pin);
        Serial.print(",");
        Serial.println(servo2Pin);

        Serial.print("SWING1: amp=");
        Serial.print(swing1Amplitude);
        Serial.print(" period=");
        Serial.print(swing1Period);
        Serial.print(" phase=");
        Serial.println(swing1Phase);

        Serial.print("SWING2: amp=");
        Serial.print(swing2Amplitude);
        Serial.print(" period=");
        Serial.print(swing2Period);
        Serial.print(" phase=");
        Serial.println(swing2Phase);

        Serial.print("CENTER:");
        Serial.print(servo1Center);
        Serial.print(",");
        Serial.println(servo2Center);

        Serial.print("DAMPING:");
        Serial.println(dampingRate);

        Serial.print("SWING_ENABLED:");
        Serial.println(swingEnabled ? "YES" : "NO");
    }
    else if (command.startsWith("PRESET:")) {
        // Preset swing patterns
        String preset = command.substring(7);

        if (preset == "GENTLE") {
            // Gentle, slow swing
            swing2Amplitude = 90.0;   // Bottom servo - main lateral swing
            swing2Period = 3000.0;
            swing1Amplitude = 20.0;   // Top servo - tilt from nadir
            swing1Period = 3000.0;
            dampingRate = 0.0;
            Serial.println("PRESET_GENTLE_OK");
        }
        else if (preset == "NORMAL") {
            // Normal pendulum swing
            swing2Amplitude = 60.0;   // Bottom servo - main lateral swing
            swing2Period = 2000.0;
            swing1Amplitude = 15.0;   // Top servo - tilt from nadir
            swing1Period = 2000.0;
            dampingRate = 0.0;
            Serial.println("PRESET_NORMAL_OK");
        }
        else if (preset == "WILD") {
            // Wild, erratic swing
            swing2Amplitude = 90.0;   // Bottom servo - main lateral swing
            swing2Period = 1500.0;
            swing1Amplitude = 30.0;   // Top servo - tilt from nadir
            swing1Period = 1500.0;
            dampingRate = 0.0;
            Serial.println("PRESET_WILD_OK");
        }
        else if (preset == "DYING") {
            // Swing that gradually slows down
            swing2Amplitude = 75.0;   // Bottom servo - main lateral swing
            swing2Period = 2000.0;
            swing1Amplitude = 20.0;   // Top servo - tilt from nadir
            swing1Period = 2000.0;
            dampingRate = 0.0003;  // Will take ~15 seconds to stop
            Serial.println("PRESET_DYING_OK");
        }
        else if (preset == "CIRCULAR") {
            // Creates circular motion (equal amplitude, 90 degree phase offset)
            swing2Amplitude = 40.0;
            swing2Period = 2000.0;
            swing2Phase = 0.0;
            swing1Amplitude = 40.0;
            swing1Period = 2000.0;
            swing1Phase = 1.5708;  // PI/2
            dampingRate = 0.0;
            Serial.println("PRESET_CIRCULAR_OK");
        }
        else {
            Serial.println("UNKNOWN_PRESET");
        }
    }
    else {
        Serial.println("UNKNOWN_COMMAND");
    }
}

void setup() {
    // Set servo pins to OUTPUT and LOW to prevent floating on startup
    pinMode(servo1Pin, OUTPUT);
    pinMode(servo2Pin, OUTPUT);
    pinMode(lightPin, OUTPUT);
    digitalWrite(servo1Pin, LOW);
    digitalWrite(servo2Pin, LOW);
    analogWrite(lightPin, 0);

    // Set PWM frequency on pin 9 to ~3.9kHz for Meanwell dimming (needs 2.8-8kHz)
    // Pin 9 on Mega uses Timer2 (affects pins 9 and 10)
    TCCR2B = (TCCR2B & 0b11111000) | 0x02;  // Prescaler = 8 -> ~3.9kHz

    // Wait for power to stabilize
    delay(2000);

    Serial.begin(9600);
    Serial.setTimeout(100);

    // Attach and center servos
    servo1.attach(servo1Pin);
    servo2.attach(servo2Pin);
    servosEnabled = true;

    servo1.write(servo1Center);
    servo2.write(servo2Center);

    // Give servos time to reach position
    delay(1000);

    Serial.println("GIMBAL_CONTROLLER_READY");
    Serial.println("Commands: START | STOP | LIGHT_ON | LIGHT_OFF | ALL_ON | ALL_OFF | STATUS");
    Serial.println("Config: BPM:30 | LIGHT_LFO:freq,depth,brightness | LIGHT_PIN:9");
}

void loop() {
    // Check for serial commands
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        processCommand(command);
    }

    // Update swing positions if enabled
    updateSwingPositions();

    // Update light LFO if enabled
    updateLight();

    // Small delay for stable loop timing
    delay(10);
}
