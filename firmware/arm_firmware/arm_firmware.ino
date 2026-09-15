/*
 * arm_firmware.ino
 * ================
 * Arduino Uno firmware for a 6-DOF gesture-controlled robotic arm.
 *
 * Motor layout:
 *   Ch 0 — Base Rotation      — MG996R
 *   Ch 1 — Shoulder Extension  — MG996R
 *   Ch 2 — Elbow Extension     — MG996R
 *   Ch 3 — Wrist Rotation      — MG90S
 *   Ch 4 — Wrist Extension     — MG90S
 *   Ch 5 — Claw (2-finger)     — SG90S
 *
 * Receives angle packets over Serial from the Python host:
 *   "base,shoulder,elbow,wrist_rot,wrist_ext,claw\n"
 * Each value is an integer 0-180.
 *
 * Drives servos via a PCA9685 16-channel PWM board over I2C.
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ─── Tunable constants ───────────────────────────────────────────────
// PCA9685 I2C address
#define PCA9685_ADDR 0x40

// PWM pulse counts — tune to your servos [TUNE]
// MG996R and MG90S typically use similar pulse ranges, but tweak per servo
#define SERVOMIN 150   // ~1 ms pulse at 50 Hz
#define SERVOMAX 600   // ~2.4 ms pulse at 50 Hz

// PWM frequency for standard hobby servos
#define PWM_FREQ 50    // Hz

// Channel assignments on the PCA9685
#define CH_BASE       0
#define CH_SHOULDER   1
#define CH_ELBOW      2
#define CH_WRIST_ROT  3
#define CH_WRIST_EXT  4
#define CH_CLAW       5

// Number of joints
#define NUM_JOINTS 6

// Serial baud rate — must match host
#define SERIAL_BAUD 115200

// Serial input buffer size
#define BUF_SIZE 64

// ─── Globals ─────────────────────────────────────────────────────────
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

char buf[BUF_SIZE];
uint8_t bufIdx = 0;

// Channel map (index → PCA9685 channel)
const uint8_t channels[NUM_JOINTS] = {
    CH_BASE, CH_SHOULDER, CH_ELBOW, CH_WRIST_ROT, CH_WRIST_EXT, CH_CLAW
};

// ─── Helpers ─────────────────────────────────────────────────────────

/**
 * Convert an angle (0-180) to a PCA9685 pulse count.
 */
uint16_t angleToPulse(int angle) {
    return (uint16_t)map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

/**
 * Parse the buffer into NUM_JOINTS integers.
 * Returns true on success (exactly NUM_JOINTS comma-separated values).
 * Malformed packets are silently discarded.
 */
bool parsePacket(const char* packet, int* angles) {
    int fieldCount = 0;
    int value = 0;
    bool hasDigits = false;
    bool negative = false;

    for (const char* p = packet; ; p++) {
        char c = *p;

        if (c == '-') {
            negative = true;
            continue;
        }

        if (c >= '0' && c <= '9') {
            value = value * 10 + (c - '0');
            hasDigits = true;
        }

        if (c == ',' || c == '\0') {
            if (!hasDigits) {
                return false;  // empty field
            }
            if (fieldCount >= NUM_JOINTS) {
                return false;  // too many fields
            }
            angles[fieldCount++] = negative ? -value : value;
            value = 0;
            hasDigits = false;
            negative = false;

            if (c == '\0') break;
        }
    }

    return (fieldCount == NUM_JOINTS);
}

// ─── Setup ───────────────────────────────────────────────────────────
void setup() {
    Serial.begin(SERIAL_BAUD);

    pwm.begin();
    pwm.setPWMFreq(PWM_FREQ);

    // Move all servos to neutral (90°)
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
        pwm.setPWM(channels[i], 0, angleToPulse(90));
    }

    Serial.println("ARM_READY");
}

// ─── Loop (non-blocking) ────────────────────────────────────────────
void loop() {
    // Read available serial bytes without blocking
    while (Serial.available() > 0) {
        char c = (char)Serial.read();

        if (c == '\n' || c == '\r') {
            if (bufIdx > 0) {
                buf[bufIdx] = '\0';  // null-terminate

                int angles[NUM_JOINTS];
                if (parsePacket(buf, angles)) {
                    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
                        // Constrain to valid servo range
                        int a = constrain(angles[i], 0, 180);
                        pwm.setPWM(channels[i], 0, angleToPulse(a));
                    }
                }
                // Malformed packets are silently ignored

                bufIdx = 0;  // reset buffer
            }
        } else {
            if (bufIdx < BUF_SIZE - 1) {
                buf[bufIdx++] = c;
            } else {
                // Overflow — discard this line
                bufIdx = 0;
            }
        }
    }

    // No delay() — loop runs as fast as possible
}
