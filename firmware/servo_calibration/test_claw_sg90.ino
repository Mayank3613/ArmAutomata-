/*
 * test_claw_sg90.ino
 * ==================
 * Simple standalone test sketch for the SG90 Claw Servo (Channel 5 on PCA9685).
 * 
 * Hardware setup:
 *   - Arduino A4 (SDA) -> PCA9685 SDA
 *   - Arduino A5 (SCL) -> PCA9685 SCL
 *   - Arduino 5V       -> PCA9685 VCC (Logic Power)
 *   - Arduino GND      -> PCA9685 GND (Common Ground)
 *   - Arduino D4       -> PCA9685 OE (Output Enable)
 *   - Buck Converter   -> PCA9685 V+ & GND (Screw Terminal)
 *   - SG90 Claw Servo  -> PCA9685 Channel 5
 * 
 * Behavior:
 *   Slowly sweeps the claw open and closed continuously so you can test it safely.
 *   Also allows manual control via Serial Monitor (115200 baud):
 *     Type an angle: 0 to 180 (e.g., 30, 90, 120)
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define PCA9685_ADDR  0x40
#define PWM_FREQ      50      // 50 Hz standard servo frequency
#define CLAW_CHANNEL  5       // PCA9685 Channel 5 for SG90 claw

#define OE_PIN        4
#define USE_OE_PIN    1

// Pulse width range for SG90 servo (0° to 180°)
// ~1ms (pulse 150) to ~2ms (pulse 450-500)
#define SERVOMIN      150
#define SERVOMAX      500

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

uint16_t angleToPulse(int angle) {
    angle = constrain(angle, 0, 180);
    return (uint16_t)map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void setup() {
    Serial.begin(115200);

#if USE_OE_PIN
    pinMode(OE_PIN, OUTPUT);
    digitalWrite(OE_PIN, HIGH);   // Keep outputs disabled during boot
#endif

    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(PWM_FREQ);

    // Disable all other channels completely so only channel 5 operates
    for (uint8_t i = 0; i < 16; i++) {
        pwm.setPWM(i, 0, 4096);  // Always-LOW (OFF/limp)
    }

    // Set claw initially to neutral (90°)
    pwm.setPWM(CLAW_CHANNEL, 0, angleToPulse(90));

    delay(200);

#if USE_OE_PIN
    digitalWrite(OE_PIN, LOW);    // Enable outputs
#endif

    Serial.println("\n=== SG90 CLAW TEST READY ===");
    Serial.println("Commands:");
    Serial.println("  Type an angle 0-180 (e.g. '40', '90', '130')");
    Serial.println("  Type 'SWEEP' to start auto-sweeping open/close");
    Serial.println("  Type 'OFF' to release the servo");
}

bool autoSweep = true;
int sweepAngle = 40;
int sweepStep = 2;

void loop() {
    // 1. Check for manual commands from Serial Monitor
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input.equalsIgnoreCase("SWEEP")) {
            autoSweep = true;
            Serial.println("Auto-sweep ENABLED");
        } else if (input.equalsIgnoreCase("OFF")) {
            autoSweep = false;
            pwm.setPWM(CLAW_CHANNEL, 0, 4096);
            Serial.println("Claw servo OFF (limp)");
        } else if (input.length() > 0) {
            int targetAngle = input.toInt();
            autoSweep = false;
            pwm.setPWM(CLAW_CHANNEL, 0, angleToPulse(targetAngle));
            Serial.print("Manual Angle -> ");
            Serial.print(targetAngle);
            Serial.print("° (Pulse: ");
            Serial.print(angleToPulse(targetAngle));
            Serial.println(")");
        }
    }

    // 2. Auto-sweep open and close slowly (from 40° to 120°)
    if (autoSweep) {
        pwm.setPWM(CLAW_CHANNEL, 0, angleToPulse(sweepAngle));
        sweepAngle += sweepStep;

        // Bounce back between 40° (closed) and 120° (open)
        if (sweepAngle <= 40 || sweepAngle >= 120) {
            sweepStep = -sweepStep;
            delay(400); // Pause briefly at ends
        }
        delay(25);      // Speed of sweep motion
    }
}
