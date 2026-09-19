/*
 * test_claw_sg90.ino
 * ==================
 * Dead-simple, bulletproof SG90 Claw test sketch.
 * 
 * - Does NOT use OE pin (relies on PCA9685 built-in pull-down).
 * - Sends pulses to Channel 5 (Claw) AND Channel 0 (just in case it was plugged in Ch 0).
 * - Sweeps back and forth continuously without any Serial blocking.
 * - Prints progress to Serial Monitor so you can see it working.
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define PCA9685_ADDR 0x40
#define PWM_FREQ     50

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

// Standard pulse ranges for SG90 servo
// 150 = ~0.7ms (0°)
// 375 = ~1.5ms (90°)
// 500 = ~2.4ms (180°)
#define PULSE_MIN 150
#define PULSE_MAX 450

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n--- Starting PCA9685 Claw Test ---");

    Wire.begin();
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(PWM_FREQ);

    // Turn off channels 0, 1, 2, 3, 4 (MG996R / MG90S arm joints)
    pwm.setPWM(0, 0, 4096);
    pwm.setPWM(1, 0, 4096);
    pwm.setPWM(2, 0, 4096);
    pwm.setPWM(3, 0, 4096);
    pwm.setPWM(4, 0, 4096);

    Serial.println("PCA9685 initialized successfully!");
    Serial.println("Sweeping Channel 5 (Claw) only...");
}

int pulse = PULSE_MIN;
int step = 5;

void loop() {
    // Send to Channel 5 (Claw only)
    pwm.setPWM(5, 0, pulse);

    pulse += step;
    if (pulse >= PULSE_MAX || pulse <= PULSE_MIN) {
        step = -step;
        Serial.print("Sweep turnaround at pulse: ");
        Serial.println(pulse);
        delay(300); // pause at limits
    }

    delay(20); // smooth motion
}
