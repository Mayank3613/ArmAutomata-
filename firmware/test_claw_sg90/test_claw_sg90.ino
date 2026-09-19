/*
 * test_claw_sg90.ino
 * ==================
 * Super Simple Slow-Motion & Step Tester for Claw Servo (Channel 5).
 * 
 * Commands in Serial Monitor (115200 baud):
 *   1  -> Go to 60° (gentle open)
 *   2  -> Go to 90° (center / neutral)
 *   3  -> Go to 120° (gentle close)
 *   +  -> Step +5°
 *   -  -> Step -5°
 *   S  -> Slow crawl sweep (moves 1° at a time so you can inspect gear teeth)
 *   O  -> Turn OFF / Limp (lets you feel gear resistance by hand)
 *   Or type any degree (0 to 180) directly.
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define PCA9685_ADDR 0x40
#define PWM_FREQ     50
#define CLAW_CH      5

// Pulse counts for 0° to 180° on standard micro servo (SG90 / MG90S)
#define SERVOMIN 150  // ~0°
#define SERVOMAX 500  // ~180°

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

int currentAngle = 90;
bool sweeping = false;
int sweepDir = 1;
unsigned long lastSweepTime = 0;

uint16_t angleToPulse(int deg) {
    deg = constrain(deg, 0, 180);
    return (uint16_t)map(deg, 0, 180, SERVOMIN, SERVOMAX);
}

void setClawAngle(int deg) {
    currentAngle = constrain(deg, 0, 180);
    pwm.setPWM(CLAW_CH, 0, angleToPulse(currentAngle));
    Serial.print("Claw Angle: ");
    Serial.print(currentAngle);
    Serial.print(" deg (pulse ");
    Serial.print(angleToPulse(currentAngle));
    Serial.println(")");
}

void setup() {
    Serial.begin(115200);
    delay(500);

    Wire.begin();
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(PWM_FREQ);

    // Disable all other channels (0 to 4 and 6 to 15) so they don't move
    for (uint8_t i = 0; i < 16; i++) {
        if (i != CLAW_CH) {
            pwm.setPWM(i, 0, 4096);
        }
    }

    // Set to 90° center
    setClawAngle(90);

    Serial.println("\n================================================");
    Serial.println("    SUPER SIMPLE CLAW SERVO STEP TESTER");
    Serial.println("================================================");
    Serial.println("Quick Keys (type in top bar and press Enter):");
    Serial.println("  1  -> 60 deg (small open)");
    Serial.println("  2  -> 90 deg (center neutral)");
    Serial.println("  3  -> 120 deg (small close)");
    Serial.println("  +  -> Nudge +5 deg");
    Serial.println("  -  -> Nudge -5 deg");
    Serial.println("  S  -> Slow crawl sweep (1 deg at a time)");
    Serial.println("  O  -> Turn OFF / Limp");
    Serial.println("  Or type any number 0-180");
    Serial.println("================================================");
}

void loop() {
    // Check serial commands
    if (Serial.available() > 0) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        if (cmd.length() > 0) {
            if (cmd == "1") {
                sweeping = false;
                setClawAngle(60);
            } else if (cmd == "2") {
                sweeping = false;
                setClawAngle(90);
            } else if (cmd == "3") {
                sweeping = false;
                setClawAngle(120);
            } else if (cmd == "+") {
                sweeping = false;
                setClawAngle(currentAngle + 5);
            } else if (cmd == "-") {
                sweeping = false;
                setClawAngle(currentAngle - 5);
            } else if (cmd.equalsIgnoreCase("S")) {
                sweeping = !sweeping;
                if (sweeping) {
                    Serial.println("Slow crawl sweep STARTED (type any key to stop)");
                } else {
                    Serial.println("Slow sweep STOPPED");
                }
            } else if (cmd.equalsIgnoreCase("O") || cmd.equalsIgnoreCase("OFF")) {
                sweeping = false;
                pwm.setPWM(CLAW_CH, 0, 4096);
                Serial.println("Servo OFF (limp). You can turn it by hand now.");
            } else {
                int customAngle = cmd.toInt();
                if (cmd == "0" || customAngle > 0) {
                    sweeping = false;
                    setClawAngle(customAngle);
                }
            }
        }
    }

    // Slow sweep: moves 1 degree every 60ms (very slow and smooth)
    if (sweeping && (millis() - lastSweepTime >= 60)) {
        lastSweepTime = millis();
        currentAngle += sweepDir;

        if (currentAngle >= 130) {
            currentAngle = 130;
            sweepDir = -1;
            delay(200);
        } else if (currentAngle <= 50) {
            currentAngle = 50;
            sweepDir = 1;
            delay(200);
        }

        pwm.setPWM(CLAW_CH, 0, angleToPulse(currentAngle));
    }
}
