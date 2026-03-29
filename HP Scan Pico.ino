#include <Adafruit_TinyUSB.h>
#include <Arduino.h>
#include <AccelStepper.h>

#define USB_VID 0x2AD1
#define USB_PID 0x7AB8
Adafruit_USBD_CDC usb_serial;

#define DIR_PIN    2
#define STEP_PIN   5
#define ENABLE_PIN 8
#define SLEEP_PIN  17
#define RESET_PIN  16
#define M0_PIN     19
#define M1_PIN     20
#define M2_PIN     18
#define LED_PIN    25

const int    MICROSTEPS       = 32;
const int    STEPS_PER_REV    = 200;
const float  GEAR_RATIO       = 168.0f / 52.0f;
const float  STEPS_PER_DEGREE = STEPS_PER_REV * MICROSTEPS * GEAR_RATIO / 360.0f;
const float  MAX_SPEED        = STEPS_PER_DEGREE * 60.0f;
const float  ACCELERATION     = STEPS_PER_DEGREE * 120.0f;
const int    BAUDRATE         = 115200;

// ── Debug log ring buffer ─────────────────────────────────────────────
const int MAX_LOG_ENTRIES = 200;
const int MAX_LOG_LENGTH  = 128;

struct LogEntry {
    unsigned long timestamp;
    char message[MAX_LOG_LENGTH];
};

static LogEntry logBuffer[MAX_LOG_ENTRIES];
static int  logHead           = 0;
static int  logCount          = 0;
static bool logBufferOverflow = false;

static void logAdd(const String& msg) {
    logBuffer[logHead].timestamp = millis();
    strncpy(logBuffer[logHead].message, msg.c_str(), MAX_LOG_LENGTH - 1);
    logBuffer[logHead].message[MAX_LOG_LENGTH - 1] = '\0';
    logHead = (logHead + 1) % MAX_LOG_ENTRIES;
    if (logCount < MAX_LOG_ENTRIES) logCount++;
    else logBufferOverflow = true;
}

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
float currentPosition = 0.0f;
bool  moving          = false;
bool  setupComplete   = false;
int   commandCounter  = 0;

// ── Driver helpers ────────────────────────────────────────────────────
void setMicrostepping(int steps) {
    bool m0 = (steps == 2 || steps == 8 || steps == 32);
    bool m1 = (steps == 4 || steps == 8);
    bool m2 = (steps == 16 || steps == 32);
    digitalWrite(M0_PIN, m0);
    digitalWrite(M1_PIN, m1);
    digitalWrite(M2_PIN, m2);
}

void enableDriver() {
    digitalWrite(RESET_PIN, HIGH);
    digitalWrite(SLEEP_PIN, HIGH);
    delayMicroseconds(1700);
    digitalWrite(ENABLE_PIN, LOW);
    logAdd("Driver ENABLED");
}

void disableDriver() {
    digitalWrite(ENABLE_PIN, HIGH);
    logAdd("Driver DISABLED");
}

// ── Motion ────────────────────────────────────────────────────────────
void moveDegrees(float degrees) {
    commandCounter++;
    logAdd("MOVE angle=" + String(degrees, 4) + " cmd=" + String(commandCounter));

    if (fabs(degrees) < 0.001f || moving) {
        logAdd("MOVE SKIP: angle too small or already moving");
        Serial.println("DONE");  // still ack so DLL doesn't hang
        return;
    }

    moving = true;
    enableDriver();
    long steps = (long)(degrees * STEPS_PER_DEGREE);
    logAdd("Steps=" + String(steps) + " dir=" + String((degrees >= 0) ? "POS" : "NEG"));

    stepper.move(steps);

    unsigned long moveStart = millis();
    while (stepper.distanceToGo() != 0) {
        stepper.run();
        if (millis() - moveStart > 30000) {
            logAdd("MOVE TIMEOUT");
            break;
        }
    }

    currentPosition += degrees;
    while (currentPosition >= 360.0f) currentPosition -= 360.0f;
    while (currentPosition <    0.0f) currentPosition += 360.0f;

    moving = false;
    disableDriver();
    logAdd("MOVE DONE pos=" + String(currentPosition, 4));
    Serial.println("DONE");
}

void goHome() {
    commandCounter++;
    logAdd("HOME cmd=" + String(commandCounter));

    if (moving) {
        logAdd("HOME SKIP: already moving");
        Serial.println("HOME");
        return;
    }

    float shortest = currentPosition;
    if (shortest > 180.0f) shortest -= 360.0f;
    moveDegrees(-shortest);
    currentPosition = 0.0f;
    logAdd("HOME DONE");
    Serial.println("HOME");
}

// ── Command parser ────────────────────────────────────────────────────
void processCommand(const String& cmd) {
    logAdd("CMD:[" + cmd + "]");

    if (cmd == "steps_per_degree") {
        Serial.print(STEPS_PER_DEGREE, 4);
        Serial.print("\r\n");
        logAdd("RESP:" + String(STEPS_PER_DEGREE, 4));
        return;
    }
    if (cmd == "version") {
        Serial.print("1.0.0\r\n");
        logAdd("RESP:1.0.0");
        return;
    }
    if (cmd.startsWith("challenge ")) return;
    if (cmd == "P" || cmd == "O") { goHome(); return; }
    if (cmd == "C") {
        moving = false;
        digitalWrite(ENABLE_PIN, HIGH);
        Serial.println("STOPPED");
        logAdd("RESP:STOPPED");
        return;
    }
    if (cmd == "N") { Serial.println("OK"); logAdd("RESP:OK"); return; }
    if (cmd == "BOOT") { reset_usb_boot(0, 0); return; }
    if (cmd == "+")  { moveDegrees( 10.0f); return; }
    if (cmd == "-")  { moveDegrees(-10.0f); return; }

    // Move commands: DLL sends "m+60.0000" or "m-60.0000"
    // substring(1) captures the sign, toFloat() gives the signed value directly.
    // Do NOT negate — just pass the value straight to moveDegrees().
    if (cmd.startsWith("m+") || cmd.startsWith("m-")) {
        float angle = cmd.substring(1).toFloat();  // e.g. "+60.0" → 60.0, "-60.0" → -60.0
        if (angle >  360.0f) angle =  360.0f;
        if (angle < -360.0f) angle = -360.0f;
        moveDegrees(angle);
        return;
    }

    // Debug commands
    if (cmd == "?DEBUG") {
        Serial.println("=== DEBUG START ===");
        Serial.print("Entries:"); Serial.print(logCount);
        Serial.print(" Overflow:"); Serial.println(logBufferOverflow ? "YES" : "NO");
        int startIndex = (logHead - logCount + MAX_LOG_ENTRIES) % MAX_LOG_ENTRIES;
        for (int i = 0; i < logCount; i++) {
            int idx = (startIndex + i) % MAX_LOG_ENTRIES;
            Serial.print("["); Serial.print(logBuffer[idx].timestamp); Serial.print("] ");
            Serial.println(logBuffer[idx].message);
        }
        Serial.println("=== DEBUG END ===");
        return;
    }
    if (cmd == "?CLEAR") {
        logHead = 0; logCount = 0; logBufferOverflow = false;
        Serial.println("OK");
        logAdd("Log cleared");
        return;
    }
    if (cmd == "?STATUS") {
        Serial.print("POS:");   Serial.println(currentPosition, 4);
        Serial.print("MOVING:"); Serial.println(moving ? "1" : "0");
        Serial.print("CMD:");   Serial.println(commandCounter);
        return;
    }

    if (cmd == "1" || cmd == "2" || cmd == "3" || cmd == "4") { Serial.println("OK"); logAdd("RESP:OK"); return; }
    if (cmd == "S" || cmd == "T" || cmd == "R" || cmd == "D" || cmd == "Z" || cmd == "0") { Serial.println("OK"); logAdd("RESP:OK"); return; }

    logAdd("CMD:UNKNOWN[" + cmd + "]");
    Serial.println("OK");  // ack unknowns so DLL never hangs
}

// ── Setup ─────────────────────────────────────────────────────────────
void setup() {
    TinyUSBDevice.setID(USB_VID, USB_PID);
    Serial.begin(BAUDRATE);

    unsigned long t = millis();
    while (!Serial && millis() - t < 5000) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(100);
    }
    digitalWrite(LED_PIN, HIGH);

    pinMode(DIR_PIN,    OUTPUT); pinMode(STEP_PIN,   OUTPUT); pinMode(ENABLE_PIN, OUTPUT);
    pinMode(SLEEP_PIN,  OUTPUT); pinMode(RESET_PIN,  OUTPUT);
    pinMode(M0_PIN,     OUTPUT); pinMode(M1_PIN,     OUTPUT); pinMode(M2_PIN,     OUTPUT);
    pinMode(LED_PIN,    OUTPUT);

    digitalWrite(ENABLE_PIN, HIGH);
    digitalWrite(SLEEP_PIN,  LOW);
    digitalWrite(RESET_PIN,  LOW);
    setMicrostepping(MICROSTEPS);
    stepper.setMaxSpeed(MAX_SPEED);
    stepper.setAcceleration(ACCELERATION);

    delay(500);
    setupComplete = true;
    logAdd("SETUP DONE");
    Serial.println("READY");
}

// ── Loop ──────────────────────────────────────────────────────────────
String inputBuffer = "";

void loop() {
    if (!setupComplete) return;

    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (inputBuffer.length() > 0) {
                String cmd = inputBuffer;
                cmd.trim();
                processCommand(cmd);
                inputBuffer = "";
            }
        } else {
            inputBuffer += c;
        }
    }
}
