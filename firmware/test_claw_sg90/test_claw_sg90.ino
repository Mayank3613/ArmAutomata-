/*
 * test_claw_sg90.ino
 * ==================
 * Manual Pulse Calibration for SG90 Claw (Channel 5 only).
 * 
 * Open Serial Monitor at 115200 baud (set line ending to "Newline").
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define PCA9685_ADDR 0x40
#define PWM_FREQ     50
#define CLAW_CH      5

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

int currentPulse = 300; // Start at midpoint

void setClawPulse(int p) {
    currentPulse = constrain(p, 100, 550);
    pwm.setPWM(CLAW_CH, 0, currentPulse);
    Serial.print(">>> Channel 5 Pulse set to: ");
    Serial.println(currentPulse);
}

void setup() {
    Serial.begin(115200);

    // Wait for serial connection to stabilize
    delay(1000);
    while (Serial.available() > 0) {
        Serial.read(); // Flush any USB connection noise
    }

    Wire.begin();
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(PWM_FREQ);

    // Turn off all other channels (0, 1, 2, 3, 4 and 6-15)
    for (uint8_t i = 0; i < 16; i++) {
        if (i != CLAW_CH) {
            pwm.setPWM(i, 0, 4096);
        }
    }

    // Set claw initially to 300
    setClawPulse(currentPulse);

    Serial.println("\n============================================");
    Serial.println("   SG90 CLAW MANUAL PULSE CALIBRATOR (CH 5)");
    Serial.println("============================================");
    Serial.println("Type a pulse (100 to 550) and press Enter.");
    Serial.println("Examples to try: 250, 300, 350, 400");
    Serial.println("Or type '+' to increase by 10, '-' to decrease by 10.");
    Serial.println("--------------------------------------------");
}

void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input.length() == 0) return;

        if (input == "+") {
            setClawPulse(currentPulse + 10);
        } else if (input == "-") {
            setClawPulse(currentPulse - 10);
        } else if (input.equalsIgnoreCase("OFF")) {
            pwm.setPWM(CLAW_CH, 0, 4096);
            Serial.println(">>> Servo OFF (limp)");
        } else {
            int val = input.toInt();
            if (val >= 100 && val <= 550) {
                setClawPulse(val);
            } else {
                Serial.print("Ignored: '");
                Serial.print(input);
                Serial.println("'. Enter a pulse number between 100 and 550.");
            }
        }
    }
}
