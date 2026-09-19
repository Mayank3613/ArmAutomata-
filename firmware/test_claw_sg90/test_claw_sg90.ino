/*
 * test_claw_sg90.ino
 * ==================
 * Manual Pulse Calibration for SG90 Claw (Channel 5 only).
 * 
 * Open Serial Monitor at 115200 baud (set line ending to "Newline" or "Both NL & CR").
 * 
 * Usage:
 *   - Type any pulse number (e.g. 150, 200, 250, 300, 350, 400, 450) and press Enter.
 *   - Type '+' to increase pulse by 10.
 *   - Type '-' to decrease pulse by 10.
 *   - Type 'OFF' to turn off the servo (make it limp).
 *   - Type 'SWEEP' to auto-sweep back and forth.
 * 
 * SG90 typical pulse reference:
 *   150 ~ ~0.7 ms (~0°)
 *   300 ~ ~1.5 ms (~90° neutral)
 *   450 ~ ~2.2 ms (~180°)
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define PCA9685_ADDR 0x40
#define PWM_FREQ     50
#define CLAW_CH      5

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

// Current pulse setting
int currentPulse = 300; // Start at ~midpoint
bool autoSweep = false;
int sweepStep = 5;

void setup() {
    Serial.begin(115200);
    delay(500);

    Wire.begin();
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(PWM_FREQ);

    // Turn off all other channels (0, 1, 2, 3, 4)
    for (uint8_t i = 0; i < 5; i++) {
        pwm.setPWM(i, 0, 4096);
    }
    for (uint8_t i = 6; i < 16; i++) {
        pwm.setPWM(i, 0, 4096);
    }

    // Set claw initially to 300 (~neutral)
    pwm.setPWM(CLAW_CH, 0, currentPulse);

    Serial.println("\n============================================");
    Serial.println("   SG90 CLAW MANUAL PULSE CALIBRATOR (CH 5)");
    Serial.println("============================================");
    Serial.println("Type a pulse value (100 to 550) and press Enter.");
    Serial.println("Or type '+' / '-' to nudge by 10.");
    Serial.println("Or type 'SWEEP' for auto-sweep, 'OFF' to relax servo.");
    Serial.println("--------------------------------------------");
    Serial.print("Current Pulse: ");
    Serial.println(currentPulse);
}

void setClawPulse(int p) {
    currentPulse = constrain(p, 100, 550);
    pwm.setPWM(CLAW_CH, 0, currentPulse);
    Serial.print("-> Channel 5 Pulse set to: ");
    Serial.println(currentPulse);
}

void loop() {
    // Read serial command
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input.length() > 0) {
            if (input.equalsIgnoreCase("SWEEP")) {
                autoSweep = true;
                Serial.println("Auto-sweep mode started! (Type any number to stop)");
            } else if (input.equalsIgnoreCase("OFF")) {
                autoSweep = false;
                pwm.setPWM(CLAW_CH, 0, 4096);
                Serial.println("-> Servo OFF (limp)");
            } else if (input == "+") {
                autoSweep = false;
                setClawPulse(currentPulse + 10);
            } else if (input == "-") {
                autoSweep = false;
                setClawPulse(currentPulse - 10);
            } else {
                // Check if user entered a number
                int val = input.toInt();
                if (val >= 100 && val <= 550) {
                    autoSweep = false;
                    setClawPulse(val);
                } else if (val > 0 && val <= 180) {
                    // User entered angle in degrees (0 to 180)
                    autoSweep = false;
                    int p = map(val, 0, 180, 150, 450);
                    Serial.print("Mapped ");
                    Serial.print(val);
                    Serial.print(" deg to pulse ");
                    Serial.println(p);
                    setClawPulse(p);
                } else {
                    Serial.println("Invalid input. Enter a pulse between 100 and 550.");
                }
            }
        }
    }

    // Auto-sweep mode if enabled
    if (autoSweep) {
        currentPulse += sweepStep;
        if (currentPulse >= 450 || currentPulse <= 150) {
            sweepStep = -sweepStep;
            Serial.print("Turnaround at: ");
            Serial.println(currentPulse);
            delay(300);
        }
        pwm.setPWM(CLAW_CH, 0, currentPulse);
        delay(25);
    }
}
