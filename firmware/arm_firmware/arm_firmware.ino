/*
 * arm_firmware.ino
 * ================
 * Arduino Uno firmware for a gesture-controlled robotic arm.
 * WRIST-ONLY MODE: Only drives the 3 positional micro-servos.
 *
 * Motor layout:
 *   Ch 0 — Base Rotation      — MG996R (DISABLED — wrong motor variant)
 *   Ch 1 — Shoulder Extension — MG996R (DISABLED — wrong motor variant)
 *   Ch 2 — Elbow Extension    — MG996R (DISABLED — wrong motor variant)
 *   Ch 3 — Wrist Rotation     — MG90S  (180° Positional)  ← ACTIVE
 *   Ch 4 — Wrist Extension    — MG90S  (180° Positional)  ← ACTIVE
 *   Ch 5 — Claw (2-finger)    — SG90   (180° Positional)  ← ACTIVE
 *
 * Receives packets over Serial from the Python host:
 *   "base_spd,shoulder_spd,elbow_spd,wrist_rot,wrist_ext,claw\n"
 *   - base_spd, shoulder_spd, elbow_spd: IGNORED (always 0, Ch 0-2 never driven)
 *   - wrist_rot, wrist_ext, claw: integers 0 to 180 (positional angles)
 *
 * Drives servos via a PCA9685 16-channel PWM board over I2C.
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ─── Tunable constants ───────────────────────────────────────────────
// PCA9685 I2C address
#define PCA9685_ADDR 0x40

// Per-channel pulse counts (50 Hz PWM, 4096 counts = 20 ms)
// MG90S  (Ch 3, 4):    ~0.50 ms – 2.50 ms (102 – 512, center = 307)
// SG90   (Ch 5):       ~0.50 ms – 2.40 ms (102 – 492, center = 297)
//
// Ch 0-2 (MG996R) are NEVER driven — values kept for array indexing only.
const uint16_t PULSE_MIN[6] = {150, 150, 150, 102, 102, 102};
const uint16_t PULSE_MAX[6] = {464, 464, 464, 512, 512, 492};

// PWM frequency for standard hobby servos
#define PWM_FREQ 50    // Hz

// Channel assignments on the PCA9685
#define CH_BASE       0   // DISABLED — MG996R, do not touch
#define CH_SHOULDER   1   // DISABLED — MG996R, do not touch
#define CH_ELBOW      2   // DISABLED — MG996R, do not touch
#define CH_WRIST_ROT  3   // ACTIVE
#define CH_WRIST_EXT  4   // ACTIVE
#define CH_CLAW       5   // ACTIVE

// Number of joints in the packet (kept at 6 for protocol compatibility)
#define NUM_JOINTS 6

// Serial baud rate — must match host
#define SERIAL_BAUD 115200

// Serial input buffer size
#define BUF_SIZE 64

// PCA9685 OE (Output Enable) pin — connect PCA9685 /OE pin to Arduino pin 4.
// Pulling OE HIGH disables all outputs (servos get no signal = safe).
// Pulling OE LOW enables outputs.
// If you haven't wired this, set USE_OE_PIN to 0 — the code still works,
// but power sequencing becomes critical.
#define USE_OE_PIN  1
#define OE_PIN      4    // Arduino digital pin connected to PCA9685 /OE

// Startup settling time after enabling outputs (ms)
#define STARTUP_DELAY_MS 500

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
 * Convert an angle (0-180) for a given joint index to PCA9685 pulse count.
 * Only used for channels 3, 4, 5 (positional servos).
 */
uint16_t angleToPulse(uint8_t chIdx, int angle) {
    angle = constrain(angle, 0, 180);
    return (uint16_t)map(angle, 0, 180, PULSE_MIN[chIdx], PULSE_MAX[chIdx]);
}

/**
 * Parse the buffer into NUM_JOINTS integers.
 * Returns true on success (exactly NUM_JOINTS comma-separated values).
 * Malformed packets are silently discarded.
 */
bool parsePacket(const char* packet, int* values) {
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
            values[fieldCount++] = negative ? -value : value;
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

#if USE_OE_PIN
    // Immediately pull OE HIGH → disables all PCA9685 outputs.
    // This MUST happen before pwm.begin() so servos never see a
    // garbage PWM signal during initialization.
    pinMode(OE_PIN, OUTPUT);
    digitalWrite(OE_PIN, HIGH);   // outputs OFF
#endif

    // Initialize PCA9685
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);  // trim oscillator for accuracy
    pwm.setPWMFreq(PWM_FREQ);

    // Channels 0, 1, 2 (MG996R): CUT OFF permanently — never send PWM.
    // This ensures the wrong-variant motors receive no signal at all.
    pwm.setPWM(CH_BASE,     0, 4096);  // PWM off
    pwm.setPWM(CH_SHOULDER, 0, 4096);  // PWM off
    pwm.setPWM(CH_ELBOW,    0, 4096);  // PWM off

    // Channels 3, 4, 5 (positional micro servos): boot at neutral 90°.
    for (uint8_t i = 3; i < NUM_JOINTS; i++) {
        pwm.setPWM(channels[i], 0, angleToPulse(i, 90));
    }

    // Turn off unused channels (6-15)
    for (uint8_t i = NUM_JOINTS; i < 16; i++) {
        pwm.setPWM(i, 0, 4096);
    }

    delay(STARTUP_DELAY_MS);   // let PCA9685 registers settle

#if USE_OE_PIN
    // Now enable outputs — MG996R channels are off, positional servos see 90°.
    digitalWrite(OE_PIN, LOW);   // outputs ON
#endif

    Serial.println(F("ARM_READY"));
}

// ─── Loop (non-blocking) ────────────────────────────────────────────
void loop() {
    // Read available serial bytes without blocking
    while (Serial.available() > 0) {
        char c = (char)Serial.read();

        if (c == '\n' || c == '\r') {
            if (bufIdx > 0) {
                buf[bufIdx] = '\0';  // null-terminate

                int values[NUM_JOINTS];
                if (parsePacket(buf, values)) {
                    // Channels 0, 1, 2 (MG996R): COMPLETELY IGNORED.
                    // No matter what values[0..2] contain, we never touch these channels.
                    // They stay permanently off (PWM cut, set in setup()).

                    // Channels 3, 4, 5: Positional micro-servo angles (0 to 180)
                    for (uint8_t i = 3; i < NUM_JOINTS; i++) {
                        int a = constrain(values[i], 0, 180);
                        pwm.setPWM(channels[i], 0, angleToPulse(i, a));
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
