/*
 * arm_firmware.ino
 * ================
 * Arduino Uno firmware for a 6-DOF gesture-controlled robotic arm.
 *
 * Motor layout:
 *   Ch 0 — Base Rotation      — MG996R (360° Continuous)
 *   Ch 1 — Shoulder Extension — MG996R (360° Continuous)
 *   Ch 2 — Elbow Extension    — MG996R (360° Continuous)
 *   Ch 3 — Wrist Rotation     — MG90S  (180° Positional)
 *   Ch 4 — Wrist Extension    — MG90S  (180° Positional)
 *   Ch 5 — Claw (2-finger)    — SG90   (180° Positional)
 *
 * Receives packets over Serial from the Python host:
 *   "base_spd,shoulder_spd,elbow_spd,wrist_rot,wrist_ext,claw\n"
 *   - base_spd, shoulder_spd, elbow_spd: integers -100 to +100 (continuous speed, 0=stop)
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
// 1.0 ms = ~205, 1.5 ms (center / neutral) = ~307, 2.0 ms = ~410
// MG996R (Ch 0, 1, 2): ~0.73 ms – 2.27 ms (150 – 464, center = 307)
// MG90S  (Ch 3, 4):    ~0.50 ms – 2.50 ms (102 – 512, center = 307)
// SG90   (Ch 5):       ~0.50 ms – 2.40 ms (102 – 492, center = 297)
const uint16_t PULSE_MIN[6] = {150, 150, 150, 102, 102, 102};
const uint16_t PULSE_MAX[6] = {464, 464, 464, 512, 512, 492};

// Continuous rotation base motor (MG996R on Ch 0)
// At 50 Hz, 1.5 ms neutral = 307 counts
// Reduced speed by half (throttle deflection halved from +/-157 to +/-78 counts)
#define BASE_STOP_PULSE 307
#define BASE_CW_FULL    229
#define BASE_CCW_FULL   385

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
 * Convert base speed (-100 to +100) to PCA9685 pulse count.
 * 0 = STOP (BASE_STOP_PULSE)
 * >0 = CCW rotation
 * <0 = CW rotation
 */
uint16_t baseSpeedToPulse(int speed) {
    speed = constrain(speed, -100, 100);
    if (speed == 0) {
        return BASE_STOP_PULSE;
    } else if (speed > 0) {
        return (uint16_t)map(speed, 0, 100, BASE_STOP_PULSE, BASE_CCW_FULL);
    } else {
        return (uint16_t)map(speed, -100, 0, BASE_CW_FULL, BASE_STOP_PULSE);
    }
}

/**
 * Convert an angle (0-180) for a given joint index (1-5) to PCA9685 pulse count.
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

    // Channels 0, 1, 2 (continuous MG996R) boot completely stopped (PWM cut off).
    // Channels 3, 4, 5 (positional micro servos) boot at neutral 90°.
    pwm.setPWM(channels[0], 0, 4096);
    pwm.setPWM(channels[1], 0, 4096);
    pwm.setPWM(channels[2], 0, 4096);
    for (uint8_t i = 3; i < NUM_JOINTS; i++) {
        pwm.setPWM(channels[i], 0, angleToPulse(i, 90));
    }
    // Turn off unused channels
    for (uint8_t i = NUM_JOINTS; i < 16; i++) {
        pwm.setPWM(i, 0, 4096);
    }

    delay(STARTUP_DELAY_MS);   // let PCA9685 registers settle

#if USE_OE_PIN
    // Now enable outputs — MG996R motors are stopped, other servos see 90° right away.
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
                    // Channels 0, 1, 2: Continuous MG996R speeds (-100 to +100)
                    for (uint8_t i = 0; i < 3; i++) {
                        int speed = constrain(values[i], -100, 100);
                        if (speed == 0) {
                            // Completely cut PWM signal: stops continuous motor dead, zero creep!
                            pwm.setPWM(channels[i], 0, 4096);
                        } else {
                            pwm.setPWM(channels[i], 0, baseSpeedToPulse(speed));
                        }
                    }

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
