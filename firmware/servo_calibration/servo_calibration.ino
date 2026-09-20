/*
 * servo_calibration.ino
 * =====================
 * Calibration and testing sketch for 6-DOF robotic arm servos.
 * Supports all MG996R motors (Ch 0 Base, Ch 1 Shoulder, Ch 2 Elbow)
 * with half-speed rotation and automatic STOP, plus micro servos (Ch 3, 4, 5).
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define PCA9685_ADDR  0x40
#define PWM_FREQ      50
#define OE_PIN        4
#define USE_OE_PIN    1

// Per-channel pulse counts (50 Hz PWM, 4096 counts = 20 ms)
// 1.0 ms = ~205, 1.5 ms (center / neutral) = ~307, 2.0 ms = ~410
// MG996R (Ch 0, 1, 2): 150 – 464 (center = 307)
// MG90S  (Ch 3, 4):    102 – 512 (center = 307)
// SG90   (Ch 5):       102 – 492 (center = 297)
const uint16_t PULSE_MIN[6] = {150, 150, 150, 102, 102, 102};
const uint16_t PULSE_MAX[6] = {464, 464, 464, 512, 512, 492};

// At 50 Hz, 1.5 ms neutral = 307 counts
// Reduced speed by half (throttle deflection halved from +/-157 to +/-78 counts)
#define BASE_STOP_PULSE 307
#define BASE_CW_FULL    229
#define BASE_CCW_FULL   385

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

char buf[64];
uint8_t bufIdx = 0;

// Interactive FINDSTOP state
bool inFindStopMode = false;
int findStopPulse = BASE_STOP_PULSE;
int findStopChannel = 0;

// ─── Timed Rotation for all MG996R motors (Channels 0, 1, 2) ─────────
float mgCurrentAngle[3] = {0.0, 0.0, 0.0};
float mgSpeedDegPerSec = 120.0;      // Speed: 120.0 deg/sec
float mgPositiveTimeFactor = 1.25;   // Multiplier for positive (CCW) moves (e.g. 1.25 = +25% duration)

bool mgMoving[3] = {false, false, false};
unsigned long mgStartTime[3] = {0, 0, 0};
unsigned long mgDurationMs[3] = {0, 0, 0};
float mgTargetAngle[3] = {0.0, 0.0, 0.0};

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

uint16_t angleToPulse(uint8_t chIdx, int angle) {
    angle = constrain(angle, 0, 180);
    if (chIdx < 6) {
        return (uint16_t)map(angle, 0, 180, PULSE_MIN[chIdx], PULSE_MAX[chIdx]);
    }
    return (uint16_t)map(angle, 0, 180, 150, 464);
}

void startMGMove(uint8_t ch, float targetAngle) {
    if (ch > 2) return;
    targetAngle = constrain(targetAngle, 0.0, 180.0);
    float delta = targetAngle - mgCurrentAngle[ch];
    if (abs(delta) < 1.0) {
        Serial.print(F("MG996R Ch "));
        Serial.print(ch);
        Serial.print(F(" is already at "));
        Serial.print(mgCurrentAngle[ch], 1);
        Serial.println(F(" deg. Use ROTATE <ch> <deg> for relative turn."));
        return;
    }

    int speed = (delta > 0) ? 100 : -100;
    unsigned long durationMs = (unsigned long)((abs(delta) / mgSpeedDegPerSec) * 1000.0);
    if (delta > 0) {
        durationMs = (unsigned long)(durationMs * mgPositiveTimeFactor);
    }

    mgMoving[ch] = true;
    mgStartTime[ch] = millis();
    mgDurationMs[ch] = durationMs;
    mgTargetAngle[ch] = targetAngle;

    pwm.setPWM(ch, 0, baseSpeedToPulse(speed));

    Serial.print(F("Rotating MG996R Ch "));
    Serial.print(ch);
    Serial.print(F(" from "));
    Serial.print(mgCurrentAngle[ch], 1);
    Serial.print(F(" deg to "));
    Serial.print(mgTargetAngle[ch], 1);
    Serial.print(F(" deg (moving "));
    Serial.print(abs(delta), 1);
    Serial.print(F(" deg at "));
    Serial.print(mgSpeedDegPerSec, 1);
    Serial.print(F(" deg/s"));
    if (delta > 0) {
        Serial.print(F(" [pos factor: "));
        Serial.print(mgPositiveTimeFactor, 2);
        Serial.print(F("x]"));
    }
    Serial.print(F(" for "));
    Serial.print(durationMs);
    Serial.println(F(" ms)..."));
}

void startMGRelativeRotate(uint8_t ch, float deltaDeg) {
    if (ch > 2 || abs(deltaDeg) < 1.0) return;
    int speed = (deltaDeg > 0) ? 100 : -100;
    unsigned long durationMs = (unsigned long)((abs(deltaDeg) / mgSpeedDegPerSec) * 1000.0);
    if (deltaDeg > 0) {
        durationMs = (unsigned long)(durationMs * mgPositiveTimeFactor);
    }

    mgMoving[ch] = true;
    mgStartTime[ch] = millis();
    mgDurationMs[ch] = durationMs;
    mgTargetAngle[ch] = mgCurrentAngle[ch] + deltaDeg;

    pwm.setPWM(ch, 0, baseSpeedToPulse(speed));

    Serial.print(F("Rotating MG996R Ch "));
    Serial.print(ch);
    Serial.print(F(" by "));
    Serial.print(deltaDeg, 1);
    Serial.print(F(" deg"));
    if (deltaDeg > 0) {
        Serial.print(F(" [pos factor: "));
        Serial.print(mgPositiveTimeFactor, 2);
        Serial.print(F("x]"));
    }
    Serial.print(F(" (duration: "));
    Serial.print(durationMs);
    Serial.println(F(" ms)..."));
}

void setup() {
    Serial.begin(115200);

#if USE_OE_PIN
    pinMode(OE_PIN, OUTPUT);
    digitalWrite(OE_PIN, HIGH);   // outputs OFF during init
#endif

    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(PWM_FREQ);

    // Start with ALL channels fully OFF — no signal to any servo.
    for (uint8_t i = 0; i < 16; i++) {
        pwm.setPWM(i, 0, 4096);   // always LOW = servo off / limp
    }

#if USE_OE_PIN
    digitalWrite(OE_PIN, LOW);    // outputs ON
#endif

    Serial.println(F("\n=== SERVO CALIBRATION MODE ==="));
    Serial.println(F("Commands for MG996R (Ch 0 Base, Ch 1 Shoulder, Ch 2 Elbow):"));
    Serial.println(F("  DEG <ch> <angle>  - Rotate to target angle (0 to 180) at half speed & STOP"));
    Serial.println(F("  ROTATE <ch> <deg> - Relative rotate by degrees (e.g. ROTATE 1 90) & STOP"));
    Serial.println(F("  SPEED <ch> <val>  - Continuous speed (-100 to 100, 0=STOP)"));
    Serial.println(F("Commands for micro servos:"));
    Serial.println(F("  DEG 3 <angle>     - Wrist Rotation (MG90S, 0-180)"));
    Serial.println(F("  DEG 4 <angle>     - Wrist Extension (MG90S, 0-180)"));
    Serial.println(F("  DEG 5 <angle>     - Claw (SG90, 30-90)"));
    Serial.println(F("General commands:"));
    Serial.println(F("  FINDSTOP <ch>     - Interactive stop pulse calibration"));
    Serial.println(F("  SET <ch> <pulse>  - Set raw pulse count (e.g. SET 0 307)"));
    Serial.println(F("  CENTER <ch>       - Set channel to midpoint (307)"));
    Serial.println(F("  OFF <ch>          - Turn off channel (stop / limp)"));
    Serial.println(F("  ALLOFF            - Turn off all channels"));
    Serial.println(F("READY"));
}

void processFindStopCommand(const char* cmd) {
    if (strcmp(cmd, "+") == 0) {
        findStopPulse += 1;
    } else if (strcmp(cmd, "-") == 0) {
        findStopPulse -= 1;
    } else if (strcmp(cmd, "++") == 0) {
        findStopPulse += 5;
    } else if (strcmp(cmd, "--") == 0) {
        findStopPulse -= 5;
    } else if (strcasecmp(cmd, "EXIT") == 0 || strcasecmp(cmd, "STOP") == 0 || strcasecmp(cmd, "DONE") == 0 || strcasecmp(cmd, "Q") == 0) {
        inFindStopMode = false;
        Serial.print(F("Exited FINDSTOP mode. Final stop pulse for Ch "));
        Serial.print(findStopChannel);
        Serial.print(F(" is: "));
        Serial.println(findStopPulse);
        return;
    } else {
        int val = atoi(cmd);
        if (val > 0) {
            findStopPulse = val;
        } else {
            Serial.println(F("In FINDSTOP mode. Send '+', '-', '++', '--', number, or 'EXIT'."));
            return;
        }
    }

    findStopPulse = constrain(findStopPulse, 100, 600);
    pwm.setPWM(findStopChannel, 0, findStopPulse);
    Serial.print(F(">>> FINDSTOP Ch "));
    Serial.print(findStopChannel);
    Serial.print(F(" -> Pulse: "));
    Serial.println(findStopPulse);
}

void processCommand(const char* cmd) {
    if (inFindStopMode) {
        processFindStopCommand(cmd);
        return;
    }

    int ch, pulse, val, angle;

    if (sscanf(cmd, "FINDSTOP %d", &ch) == 1) {
        findStopChannel = constrain(ch, 0, 15);
        findStopPulse = BASE_STOP_PULSE;
        inFindStopMode = true;
        pwm.setPWM(findStopChannel, 0, findStopPulse);
        Serial.print(F("--- ENTERED FINDSTOP MODE FOR CH "));
        Serial.print(findStopChannel);
        Serial.println(F(" ---"));
        Serial.print(F("Current Pulse: ")); Serial.println(findStopPulse);
        Serial.println(F("Commands: '+' / '-' (+/-1), '++' / '--' (+/-5), <number>, or 'EXIT'"));

    } else if (sscanf(cmd, "SPEED %d %d", &ch, &val) == 2) {
        ch = constrain(ch, 0, 15);
        val = constrain(val, -100, 100);
        if (ch <= 2) mgMoving[ch] = false;
        if (val == 0) {
            pwm.setPWM(ch, 0, 4096);  // Signal cut -> motor stops immediately
            Serial.print(F("Ch ")); Serial.print(ch);
            Serial.println(F(" -> Speed: 0 (STOPPED / PWM OFF)"));
        } else {
            uint16_t p = baseSpeedToPulse(val);
            pwm.setPWM(ch, 0, p);
            Serial.print(F("Ch ")); Serial.print(ch);
            Serial.print(F(" -> Speed: ")); Serial.print(val);
            Serial.print(F(" (pulse ")); Serial.print(p); Serial.println(F(")"));
        }

    } else if (sscanf(cmd, "DEG %d %d", &ch, &angle) == 2) {
        ch = constrain(ch, 0, 15);
        angle = constrain(angle, 0, 180);
        if (ch <= 2) {
            // All MG996R motors (Ch 0 Base, Ch 1 Shoulder, Ch 2 Elbow):
            // Execute timed rotation at half speed and stop dead!
            startMGMove(ch, (float)angle);
        } else {
            // Micro servos (Ch 3, 4, 5)
            uint16_t p = angleToPulse(ch, angle);
            pwm.setPWM(ch, 0, p);
            Serial.print(F("Ch ")); Serial.print(ch);
            Serial.print(F(" -> Angle: ")); Serial.print(angle);
            Serial.print(F(" deg (pulse ")); Serial.print(p); Serial.println(F(")"));
        }

    } else if (sscanf(cmd, "ROTATE %d %d", &ch, &val) == 2) {
        if (ch <= 2) {
            startMGRelativeRotate(ch, (float)val);
        } else {
            Serial.println(F("ROTATE is for MG996R motors (Ch 0, 1, 2). Use DEG <ch> <angle> for Ch 3-5."));
        }

    } else if (sscanf(cmd, "SETANGLE %d %d", &ch, &val) == 2) {
        if (ch <= 2) {
            mgCurrentAngle[ch] = (float)val;
            Serial.print(F("MG996R Ch "));
            Serial.print(ch);
            Serial.print(F(" reference angle set to: "));
            Serial.print(mgCurrentAngle[ch], 1);
            Serial.println(F(" deg."));
        }

    } else if (sscanf(cmd, "SETSPEED %d", &val) == 1) {
        mgSpeedDegPerSec = (float)val;
        Serial.print(F("MG996R speed calibrated to: "));
        Serial.print(mgSpeedDegPerSec, 1);
        Serial.println(F(" deg/sec."));

    } else if (sscanf(cmd, "SETPOSFACTOR %d", &val) == 1 || sscanf(cmd, "SETNEGFACTOR %d", &val) == 1) {
        mgPositiveTimeFactor = (float)val / 100.0;
        Serial.print(F("MG996R positive time factor set to: "));
        Serial.print(mgPositiveTimeFactor, 2);
        Serial.println(F("x"));

    } else if (sscanf(cmd, "SET %d %d", &ch, &pulse) == 2) {
        ch = constrain(ch, 0, 15);
        pulse = constrain(pulse, 80, 650);
        pwm.setPWM(ch, 0, pulse);
        Serial.print(F("Ch ")); Serial.print(ch);
        Serial.print(F(" -> pulse ")); Serial.println(pulse);

    } else if (sscanf(cmd, "CENTER %d", &ch) == 1) {
        ch = constrain(ch, 0, 15);
        pwm.setPWM(ch, 0, 307);
        Serial.print(F("Ch ")); Serial.print(ch);
        Serial.println(F(" -> CENTER (307 / 1.5ms)"));

    } else if (sscanf(cmd, "OFF %d", &ch) == 1) {
        ch = constrain(ch, 0, 15);
        if (ch <= 2) mgMoving[ch] = false;
        pwm.setPWM(ch, 0, 4096);  // always LOW = servo limp/off
        Serial.print(F("Ch ")); Serial.print(ch);
        Serial.println(F(" -> OFF"));

    } else if (sscanf(cmd, "ALL %d", &pulse) == 1) {
        for (uint8_t i = 0; i < 3; i++) mgMoving[i] = false;
        pulse = constrain(pulse, 80, 650);
        for (uint8_t i = 0; i < 16; i++) {
            pwm.setPWM(i, 0, pulse);
        }
        Serial.print(F("ALL -> pulse ")); Serial.println(pulse);

    } else if (strncmp(cmd, "ALLOFF", 6) == 0) {
        for (uint8_t i = 0; i < 3; i++) mgMoving[i] = false;
        for (uint8_t i = 0; i < 16; i++) {
            pwm.setPWM(i, 0, 4096);  // always LOW
        }
        Serial.println(F("ALL -> OFF"));

    } else {
        Serial.print(F("Unknown command: ")); Serial.println(cmd);
    }
}

void loop() {
    // Check if timed rotation for any of the 3 MG996R motors is active and finished
    for (uint8_t i = 0; i < 3; i++) {
        if (mgMoving[i] && (millis() - mgStartTime[i] >= mgDurationMs[i])) {
            mgMoving[i] = false;
            pwm.setPWM(i, 0, 4096);  // CUT SIGNAL -> MOTOR STOPS DEAD!
            mgCurrentAngle[i] = mgTargetAngle[i];
            Serial.print(F(">>> MG996R Ch "));
            Serial.print(i);
            Serial.print(F(" reached "));
            Serial.print(mgCurrentAngle[i], 1);
            Serial.println(F(" deg. Motor STOPPED (PWM OFF)."));
        }
    }

    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (bufIdx > 0) {
                buf[bufIdx] = '\0';
                processCommand(buf);
                bufIdx = 0;
            }
        } else if (bufIdx < 63) {
            buf[bufIdx++] = c;
        }
    }
}
