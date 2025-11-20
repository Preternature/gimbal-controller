# Gimbal Light Controller

Double-servo gimbal that swings a red light to emulate a light swinging on a rope.

## Hardware Setup

### Components
- Arduino Uno (or compatible)
- 2x Servo motors (SG90 or MG996R recommended)
- High-mA red LED/light (powered separately via battery)
- External power supply for servos (recommended for heavy loads)

### Wiring

```
Arduino Pin 9  -> Servo 1 Signal (primary swing - pitch)
Arduino Pin 10 -> Servo 2 Signal (secondary swing - roll)
Arduino GND    -> Servo GND (both)
External 5-6V  -> Servo VCC (both) - use separate power for heavy servos
```

**Important:** For high-torque servos, use external power. Do not power servos directly from Arduino when driving heavy loads.

## Upload

1. Connect Arduino via USB
2. Edit `upload_gimbal.bat` to set correct COM port
3. Run `upload_gimbal.bat`

## Serial Commands (9600 baud)

### Basic Control
- `START` - Begin swing motion
- `STOP` - Stop and center servos
- `RESET` - Reset all parameters to defaults
- `STATUS` - Show current settings

### Configuration
- `SERVO_PINS:9,10` - Set servo pins
- `CENTER:90,90` - Set center positions for both servos
- `SWING1:45,2000,0` - Primary swing: amplitude, period(ms), phase(rad)
- `SWING2:15,2300,1.57` - Secondary swing: amplitude, period(ms), phase(rad)
- `DAMPING:0.0003` - Decay rate (0 = no damping)

### Presets
- `PRESET:GENTLE` - Slow, gentle swing
- `PRESET:NORMAL` - Standard pendulum motion
- `PRESET:WILD` - Fast, erratic swing
- `PRESET:DYING` - Swing that gradually fades out
- `PRESET:CIRCULAR` - Circular motion pattern

## Quick Start

1. Upload sketch
2. Open Serial Monitor at 9600 baud
3. Type `START` and press Enter
4. Use `PRESET:DYING` for realistic swing that fades

## Tuning Tips

- **Slower swing:** Increase period values (e.g., 3000ms)
- **Wider swing:** Increase amplitude (max 90 degrees)
- **More random:** Use different periods for swing1 and swing2
- **Circular motion:** Same amplitude/period, phase offset of PI/2 (1.5708)
