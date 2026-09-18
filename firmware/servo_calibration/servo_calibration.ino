/*
 * servo_calibration.ino
 * =====================
 * Use this sketch FIRST to find the correct neutral angles for each servo.
 * 
 * Upload this sketch, open Serial Monitor at 115200 baud, then type commands:
 *
 *   SET <channel> <pulse>   — set a specific channel to a raw pulse count
 *   CENTER <channel>        — set channel to pulse 375 (mid-point)
 *   OFF <channel>           — turn off a channel (servo goes limp)
 *   ALL <pulse>             — set all channels to same pulse
 *   ALLOFF                  — turn off all channels
 *
 * Example session:
 *   SET 0 375     → sets base (ch0) to pulse 375
 *   SET 0 300     → moves base toward min
 *   SET 0 450     → moves base toward max
 *   
 * Find the pulse value where each servo is at the position you want as neutral.
 * Write down those values — you'll enter them in config below.
 *
 * Pulse count guide (at 50Hz, 4096 counts = 20ms):
 *   100 = ~0.49ms  (minimum for most servos)
 *   205 = ~1.00ms  
 *   375 = ~1.83ms  (current "90°" — probably wrong for your arm)
 *   410 = ~2.00ms  (center for most standard servos)
 *   500 = ~2.44ms
 *   600 = ~2.93ms  (maximum for most servos)
 *
 * Typical neutral for most hobby servos: pulse ~375–410
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define PCA9685_ADDR  0x40
#define PWM_FREQ      50
#define OE_PIN        4
#define USE_OE_PIN    1

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

char buf[64];
uint8_t bufIdx = 0;

void setup() {
    Serial.begin(115200);

#if USE_OE_PIN
    pinMode(OE_PIN, OUTPUT);
    digitalWrite(OE_PIN, HIGH);   // outputs OFF during init
#endif

    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(PWM_FREQ);

    // Start with all channels OFF (servos limp — safe)
    for (uint8_t i = 0; i < 16; i++) {
        pwm.setPWM(i, 0, 0);
        pwm.setPin(i, 0, true);   // full OFF
    }

#if USE_OE_PIN
    digitalWrite(OE_PIN, LOW);    // outputs ON
#endif

    Serial.println("=== SERVO CALIBRATION MODE ===");
    Serial.println("Commands:");
    Serial.println("  SET <ch> <pulse>  — e.g. SET 0 375");
    Serial.println("  CENTER <ch>       — set to pulse 375");
    Serial.println("  OFF <ch>          — turn off channel");
    Serial.println("  ALL <pulse>       — all channels same pulse");
    Serial.println("  ALLOFF            — turn off all channels");
    Serial.println("Channel map: 0=Base 1=Shoulder 2=Elbow 3=WristRot 4=WristExt 5=Claw");
    Serial.println("Pulse range: 100 (min) to 600 (max). Neutral is usually 370-410.");
    Serial.println("READY");
}

void processCommand(const char* cmd) {
    char token[16];
    int ch, pulse;

    if (sscanf(cmd, "SET %d %d", &ch, &pulse) == 2) {
        ch = constrain(ch, 0, 15);
        pulse = constrain(pulse, 80, 650);
        pwm.setPWM(ch, 0, pulse);
        Serial.print("Ch "); Serial.print(ch);
        Serial.print(" → pulse "); Serial.println(pulse);

    } else if (sscanf(cmd, "CENTER %d", &ch) == 1) {
        ch = constrain(ch, 0, 15);
        pwm.setPWM(ch, 0, 375);
        Serial.print("Ch "); Serial.print(ch);
        Serial.println(" → CENTER (375)");

    } else if (sscanf(cmd, "OFF %d", &ch) == 1) {
        ch = constrain(ch, 0, 15);
        pwm.setPin(ch, 0, true);
        Serial.print("Ch "); Serial.print(ch);
        Serial.println(" → OFF");

    } else if (sscanf(cmd, "ALL %d", &pulse) == 1) {
        pulse = constrain(pulse, 80, 650);
        for (uint8_t i = 0; i < 16; i++) {
            pwm.setPWM(i, 0, pulse);
        }
        Serial.print("ALL → pulse "); Serial.println(pulse);

    } else if (strncmp(cmd, "ALLOFF", 6) == 0) {
        for (uint8_t i = 0; i < 16; i++) {
            pwm.setPin(i, 0, true);
        }
        Serial.println("ALL → OFF");

    } else {
        Serial.print("Unknown command: "); Serial.println(cmd);
    }
}

void loop() {
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
