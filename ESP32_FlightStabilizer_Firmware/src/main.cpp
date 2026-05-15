// ================================================================
//         ESP32 FLIGHT STABILIZER FIRMWARE — FINAL VERSION
//         Dual-core: Core 1 = flight control, Core 0 = WiFi/GCS
// ================================================================

#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <ESPmDNS.h>
#include <EEPROM.h>
#include <ESP32Servo.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// ================================================================
//                        EEPROM LAYOUT
//  addr  0 : ACCEL_FILTER  (float, 4 bytes)
//  addr  4 : GYRO_FILTER   (float, 4 bytes)
//  addr  8 : COMP_FILTER   (float, 4 bytes)
//  addr 12 : pitchPID.Kp   (float, 4 bytes)
//  addr 16 : pitchPID.Ki   (float, 4 bytes)
//  addr 20 : pitchPID.Kd   (float, 4 bytes)
//  addr 24 : rollPID.Kp    (float, 4 bytes)
//  addr 28 : rollPID.Ki    (float, 4 bytes)
//  addr 32 : rollPID.Kd    (float, 4 bytes)
//  addr 36 : axisConfig    (uint8, 1 byte)  bit0=swap, bit1=pitchSign, bit2=rollSign
//  addr 37 : pidDeadband   (float, 4 bytes)
//  addr 41 : servoDeadband (float, 4 bytes)
//  --- WiFi credentials (new) ---
//  addr 45 : wifi_valid    (uint8, 1 byte)  0xA5 = valid credentials stored
//  addr 46 : wifi_ssid     (char[64])       null-terminated SSID
//  addr 110 : wifi_pass    (char[64])       null-terminated password
//  TOTAL: 174 bytes
// ================================================================
#define EEPROM_SIZE         176
#define ACCEL_FILTER_ADDR    0
#define GYRO_FILTER_ADDR     4
#define COMP_FILTER_ADDR     8
#define PITCH_PID_ADDR      12
#define ROLL_PID_ADDR       24
#define AXIS_CONFIG_ADDR    36
#define PID_DB_ADDR         37
#define SERVO_DB_ADDR       41
// WiFi credential addresses
#define WIFI_VALID_ADDR     45
#define WIFI_SSID_ADDR      46
#define WIFI_PASS_ADDR      110
#define WIFI_FIELD_LEN      64
#define WIFI_VALID_MAGIC    0xA5

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

// ================================================================
//                     MPU6050 CONFIGURATION
// gSensitivity must match register 0x1B:
//   0x00 = ±250°/s  → 131.0
//   0x08 = ±500°/s  → 65.5  ← current
//   0x10 = ±1000°/s → 32.8
//   0x18 = ±2000°/s → 16.4
// ================================================================
#define MPU6050_ADDR  0x68
const float  FREQ          = 50.0f;   // Hz — control loop AND sensor rate
const double G_SENSITIVITY = 65.5;    // must match 0x1B

// ================================================================
//                       HARDWARE PINS
// ================================================================
#define PITCH_IP         15   // PWM input: pitch stick
#define ROLL_IP          16   // PWM input: roll stick
#define YAW_IP           17   // PWM input: yaw stick (read, not yet used)
#define AUTO_PILOT_IP    18   // PWM input: mode switch (>90% = auto)
#define PITCH_SERVO_PIN  25
#define ROLL_SERVO_PIN   26
#define LED_PIN           2

#define MIN_PW       999    // µs — minimum valid PWM pulse
#define MAX_PW      1993    // µs — maximum valid PWM pulse
#define PW_TIMEOUT  25000   // µs — pulseIn timeout

#define SERVO_MIN    45
#define SERVO_MAX   135
#define SERVO_CENTER 90

// ================================================================
//                       WIFI / TCP
// ================================================================
// Default (fallback) credentials — used only if no credentials are
// stored in EEPROM. The user-supplied credentials always take priority.
#define DEFAULT_WIFI_SSID "aju"
#define DEFAULT_WIFI_PASS "@ajujcd@"

// Runtime WiFi credential buffers — loaded from EEPROM or set to defaults
static char wifiSSID[WIFI_FIELD_LEN] = DEFAULT_WIFI_SSID;
static char wifiPass[WIFI_FIELD_LEN] = DEFAULT_WIFI_PASS;

WiFiServer   tcpServer(12345);
#define MAX_CMD_LEN  48

// ================================================================
//                  RUNTIME TUNABLE PARAMETERS
//  All of these can be changed live from GCS and saved to EEPROM.
// ================================================================

// Filter coefficients
float ACCEL_FILTER = 0.3f;
float GYRO_FILTER  = 0.08f;
float COMP_FILTER  = 0.7f;

// PID deadband: errors smaller than this (degrees) are treated as zero.
// Prevents servo hunting from sensor noise at rest.
float PID_DEADBAND   = 0.8f;

// Servo deadband: only command servo if angle change ≥ this (degrees).
// Prevents micro-PWM chatter.
float SERVO_DEADBAND = 1.0f;

// ================================================================
//                  AXIS ORIENTATION (runtime flags)
//
//  These fix the "wrong servo responds" bug without reflashing.
//  Changed via GCS buttons; persisted in EEPROM.
//
//  axisSwap:   false = pitch→pitchServo, roll→rollServo
//              true  = swap the two sensor axes
//  pitchSign:  +1 or -1  (invert if correction goes wrong direction)
//  rollSign:   +1 or -1
// ================================================================
bool  axisSwap  = false;
int   pitchSign = 1;
int   rollSign  = 1;

// ================================================================
//                      SYNCHRONISATION
// ================================================================
SemaphoreHandle_t   configMutex;                                // protects filter + PID config
static portMUX_TYPE imuLock = portMUX_INITIALIZER_UNLOCKED;    // protects 64-bit sensor angles

// ================================================================
//                        SENSOR ANGLES
//  Written by Core 1 exclusively, read by Core 0 for streaming.
//  64-bit doubles are not atomically read/written on Xtensa LX6,
//  so always use imuLock around every access.
// ================================================================
volatile double sensorPitch = 0.0;   // °, nose-up positive
volatile double sensorRoll  = 0.0;   // °, right-tilt positive
volatile double sensorYaw   = 0.0;   // °, gyro-integrated only

// Internal IMU working variables — Core 1 only, no lock needed
static double  gyrX = 0.0, gyrY = 0.0, gyrZ = 0.0;
static double  gyrXoffs = 0.0, gyrYoffs = 0.0, gyrZoffs = 0.0;
static int16_t accX = 0, accY = 0, accZ = 0;
static double  fax = 0.0, fay = 0.0, faz = 0.0;  // filtered accel
static double  fgx = 0.0, fgy = 0.0, fgz = 0.0;  // filtered gyro

// ================================================================
//                     STABILISATION STATE
// ================================================================
float referencePitch  = 0.0f;
float referenceRoll   = 0.0f;
float pitchOffset     = SERVO_CENTER;
float rollOffset      = SERVO_CENTER;
bool  lastAutoState   = false;

static int lastPitchCmd = SERVO_CENTER;   // last angle written to pitch servo
static int lastRollCmd  = SERVO_CENTER;   // last angle written to roll servo

// ================================================================
//                     PWM CACHE  (Core 1 → Core 0)
//  Core 1 reads PWM pulses and stores results here.
//  Core 0 reads these for streaming — no pulseIn on Core 0.
//  int is 32-bit aligned → atomic read on LX6, no lock needed.
// ================================================================
volatile int pwmPct1 = 50, pwmPct2 = 50, pwmPct3 = 50, pwmPct4 = 0;

// ================================================================
//                     STREAMING FLAGS
//  Written and read by Core 0 only inside the client loop.
//  No lock needed.
// ================================================================
volatile bool cubeStreaming = false;
volatile bool pwmStreaming  = false;

static unsigned long lastLoopTime = 0;

// ================================================================
//                         PID STRUCT
// ================================================================
struct PID {
  float Kp, Ki, Kd;
  float integral;
  float prevMeasurement;   // for derivative-on-measurement
  unsigned long lastTime;
  bool  justReset;         // skip bumpless transfer on first tick after reset
};

PID pitchPID = {2.0f, 0.1f, 0.5f, 0.0f, 0.0f, 0, true};
PID rollPID  = {2.0f, 0.1f, 0.5f, 0.0f, 0.0f, 0, true};

Servo pitchServo;
Servo rollServo;

// ================================================================
//                     FUNCTION PROTOTYPES
// ================================================================
void  loadParameters();
void  saveParametersUnlocked();
void  loadWifiCredentials();
void  saveWifiCredentials(const char* ssid, const char* pass);
void  checkSerialConfig();
void  initMPU6050();
void  calibrateMPU6050();
void  updateMPU6050();
void  getAngles(float& pitch, float& roll);
void  updateServoFromMPU();
void  writeServo(Servo& s, int& last, int angle);
float computePID(PID& pid, float setpoint, float meas, float curOut = 0.0f);
void  resetPID(PID& pid, float initMeas = 0.0f);
int   i2cRead(int addr, int reg, uint8_t* buf, int len);
int   i2cWrite(int addr, int reg, uint8_t val);
void  communicationTask(void* pv);

// ================================================================
//                  getAngles — AXIS MAPPING HELPER
//
//  Single place where axisSwap + pitchSign + rollSign are applied.
//  Every part of the code that needs the current angle calls this.
//  Never reads sensorPitch/sensorRoll directly.
// ================================================================
void getAngles(float& pitch, float& roll) {
  double sp, sr;
  portENTER_CRITICAL(&imuLock);
  sp = sensorPitch;
  sr = sensorRoll;
  portEXIT_CRITICAL(&imuLock);

  if (axisSwap) {
    pitch = (float)sr * pitchSign;
    roll  = (float)sp * rollSign;
  } else {
    pitch = (float)sp * pitchSign;
    roll  = (float)sr * rollSign;
  }
}

// ================================================================
//                  WIFI CREDENTIAL STORAGE
// ================================================================

// Load WiFi credentials from EEPROM into wifiSSID / wifiPass buffers.
// Falls back to compile-time defaults if no valid credentials are stored.
void loadWifiCredentials() {
  uint8_t magic = EEPROM.read(WIFI_VALID_ADDR);
  if (magic == WIFI_VALID_MAGIC) {
    // Read stored SSID
    for (int i = 0; i < WIFI_FIELD_LEN; i++) {
      wifiSSID[i] = (char)EEPROM.read(WIFI_SSID_ADDR + i);
    }
    wifiSSID[WIFI_FIELD_LEN - 1] = '\0';  // guarantee null terminator

    // Read stored password
    for (int i = 0; i < WIFI_FIELD_LEN; i++) {
      wifiPass[i] = (char)EEPROM.read(WIFI_PASS_ADDR + i);
    }
    wifiPass[WIFI_FIELD_LEN - 1] = '\0';

    // Sanity-check: SSID must be at least 1 char
    if (strlen(wifiSSID) == 0) {
      Serial.println("[WiFi] EEPROM credentials invalid, using defaults.");
      strncpy(wifiSSID, DEFAULT_WIFI_SSID, WIFI_FIELD_LEN - 1);
      strncpy(wifiPass, DEFAULT_WIFI_PASS, WIFI_FIELD_LEN - 1);
    } else {
      Serial.printf("[WiFi] Loaded SSID from EEPROM: %s\n", wifiSSID);
    }
  } else {
    Serial.println("[WiFi] No credentials in EEPROM, using defaults.");
    strncpy(wifiSSID, DEFAULT_WIFI_SSID, WIFI_FIELD_LEN - 1);
    strncpy(wifiPass, DEFAULT_WIFI_PASS, WIFI_FIELD_LEN - 1);
  }
}

// Persist WiFi credentials to EEPROM.
void saveWifiCredentials(const char* ssid, const char* pass) {
  EEPROM.write(WIFI_VALID_ADDR, WIFI_VALID_MAGIC);

  for (int i = 0; i < WIFI_FIELD_LEN; i++) {
    EEPROM.write(WIFI_SSID_ADDR + i, (i < (int)strlen(ssid)) ? ssid[i] : 0);
  }
  for (int i = 0; i < WIFI_FIELD_LEN; i++) {
    EEPROM.write(WIFI_PASS_ADDR + i, (i < (int)strlen(pass)) ? pass[i] : 0);
  }

  EEPROM.commit();
  Serial.println("[WiFi] Credentials saved to EEPROM.");
}

// ================================================================
//              SERIAL CONFIG MENU
//
//  Called once from setup() before WiFi connects.
//  Detects whether a serial terminal is open (DTR asserted) and, if
//  so, offers the user a short window to enter the config menu.
//  If no key is pressed within SERIAL_WAIT_MS, boot continues
//  normally with no delay or interaction.
//
//  Menu options:
//    1  — Change WiFi credentials (stored permanently in EEPROM)
//    2  — Show current WiFi SSID
//    3  — Clear stored credentials (revert to firmware defaults)
//    0  — Exit / continue boot
//
//  The function is entirely self-contained and does NOT modify any
//  flight-control state. It only reads/writes the WiFi sections of
//  EEPROM and the wifiSSID/wifiPass buffers.
// ================================================================
// ================================================================
//  SERIAL_WAIT_MS — how long the firmware waits for a keypress.
//
//  The GUI shows a 5-second countdown to the user, then sends the
//  choice byte.  The firmware window must be LONGER than the GUI
//  countdown so the choice always arrives in time even on slow PCs.
//  10 s gives comfortable headroom.
// ================================================================
#define SERIAL_WAIT_MS   10000  // ms to wait for keypress (> GUI 5 s countdown)
#define SERIAL_BAUD_WAIT 200    // ms to wait for USB-Serial bridge to enumerate

// ----------------------------------------------------------------
//  serialReadLine  — read one line from Serial
//
//  Blocks until '\n' arrives or timeoutMs elapses with no new char.
//  '\r' bytes are silently discarded (handles both LF and CRLF).
//  Returns true if at least one character was read.
// ----------------------------------------------------------------
static bool serialReadLine(char* buf, int maxLen, unsigned long timeoutMs) {
  int  idx      = 0;
  unsigned long deadline = millis() + timeoutMs;
  memset(buf, 0, maxLen);
  while (millis() < deadline) {
    if (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\r') continue;
      if (c == '\n' || c == '\0') break;
      if (idx < maxLen - 1) buf[idx++] = c;
      deadline = millis() + timeoutMs;  // reset timeout on every received char
    }
  }
  buf[idx] = '\0';
  return (idx > 0);
}

// ----------------------------------------------------------------
//  checkSerialConfig  — boot-time WiFi config menu
//
//  Called once from setup() before WiFi.begin().
//  Silently skipped when no serial terminal is connected.
//
//  The GUI serial monitor detects the "Flight Controller Config"
//  banner and shows the user two buttons within its own 5-second
//  countdown.  When the user clicks a button the GUI sends the
//  corresponding digit.  No human typing speed required.
//
//  Option 1 — Change WiFi credentials
//    GUI sends '1', then waits for each prompt line before sending
//    the value.  Firmware reads SSID, password, confirmation ('y').
//
//  Option 2 — View current SSID + password
//    GUI sends '2'.  Firmware prints credentials and immediately
//    continues boot — no blocking read needed, no Enter required.
//
//  Any other byte / timeout → boot normally.
// ----------------------------------------------------------------
void checkSerialConfig() {
  delay(SERIAL_BAUD_WAIT);
  if (!Serial) return;   // no USB-Serial connection — skip entirely

  // ---- Print banner (GUI watches for "Flight Controller Config") ----
  Serial.println();
  Serial.println("==============================");
  Serial.println("  Flight Controller Config");
  Serial.println("  Waiting for option (10 s)");
  Serial.println("==============================");
  Serial.println("  1 = Change WiFi credentials");
  Serial.println("  2 = View current SSID & password");
  Serial.println("  (timeout = boot normally)");
  Serial.println();

  // Flush any stale RX bytes that arrived before we started listening
  while (Serial.available()) Serial.read();

  // ---- Wait for one choice byte ----
  unsigned long deadline = millis() + SERIAL_WAIT_MS;
  char choice = 0;

  // Print a simple countdown once per second so the log shows activity
  int lastSecond = -1;
  while (millis() < deadline) {
    int remaining = (int)((deadline - millis()) / 1000) + 1;
    if (remaining != lastSecond) {
      Serial.printf("Booting in %d s...\n", remaining);
      lastSecond = remaining;
    }
    if (Serial.available()) {
      choice = (char)Serial.read();
      // Flush everything that arrived with the choice byte (e.g. the trailing
      // '\n' the GUI appends).  Without this flush, serialReadLine() reads the
      // '\n' immediately, sees zero real characters, returns false, and prints
      // "Timeout / empty input" before the user has typed anything.
      delay(10);
      while (Serial.available()) Serial.read();
      break;
    }
    delay(50);
  }

  if (choice == 0) {
    Serial.println("No input — booting normally.");
    return;
  }

  char inputBuf[WIFI_FIELD_LEN];

  switch (choice) {

    // ---- Option 1: Change WiFi credentials ----
    // GUI sends SSID line, password line, then 'y' to confirm.
    // Each serialReadLine() waits up to 30 s — plenty of time for
    // the user to edit fields in the GUI and press Send.
    case '1': {
      Serial.println("--- Change WiFi Credentials ---");
      Serial.println("Enter new SSID (then press Enter):");

      if (!serialReadLine(inputBuf, sizeof(inputBuf), 30000)) {
        Serial.println("Timeout / empty input. Credentials unchanged.");
        break;
      }
      char newSSID[WIFI_FIELD_LEN];
      strncpy(newSSID, inputBuf, WIFI_FIELD_LEN - 1);
      newSSID[WIFI_FIELD_LEN - 1] = '\0';

      Serial.println("Enter new Password (then press Enter):");
      serialReadLine(inputBuf, sizeof(inputBuf), 30000);  // blank = open network
      char newPass[WIFI_FIELD_LEN];
      strncpy(newPass, inputBuf, WIFI_FIELD_LEN - 1);
      newPass[WIFI_FIELD_LEN - 1] = '\0';

      Serial.printf("Confirm: SSID=\"%s\"  Password=\"%s\"\n", newSSID, newPass);
      Serial.println("Save? (y/n):");
      if (!serialReadLine(inputBuf, sizeof(inputBuf), 30000)) {
        Serial.println("Timeout. Credentials unchanged.");
        break;
      }
      if (inputBuf[0] == 'y' || inputBuf[0] == 'Y') {
        saveWifiCredentials(newSSID, newPass);
        strncpy(wifiSSID, newSSID, WIFI_FIELD_LEN - 1);
        strncpy(wifiPass, newPass, WIFI_FIELD_LEN - 1);
        Serial.println("Credentials saved. Continuing boot with new settings.");
      } else {
        Serial.println("Cancelled. Credentials unchanged.");
      }
      break;
    }

    // ---- Option 2: View current SSID & password ----
    // Just print and continue — no blocking read.
    // GUI button label: "1  View SSID" (sends '2' to firmware).
    case '2': {
      uint8_t magic = EEPROM.read(WIFI_VALID_ADDR);
      const char* src = (magic == WIFI_VALID_MAGIC) ? "EEPROM" : "firmware default";
      Serial.printf("Current SSID     (%s): %s\n", src, wifiSSID);
      Serial.printf("Current Password (%s): %s\n", src, wifiPass);
      // No blocking read — GUI does not send Enter for this option
      break;
    }

    default:
      Serial.println("Unrecognised option. Booting normally.");
      break;
  }

  Serial.println("Continuing boot...");
  Serial.println("==============================");
  delay(300);
}

// ================================================================
//                           SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\n=== ESP32 Flight Controller ===");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // --- Synchronisation primitives ---
  configMutex = xSemaphoreCreateMutex();
  if (!configMutex) { Serial.println("FATAL: mutex"); while (true); }

  // --- Hardware ---
  EEPROM.begin(EEPROM_SIZE);
  Wire.begin(21, 22);
  Wire.setClock(400000);  // 400 kHz I2C — 4× faster than default

  // --- Load saved flight config ---
  loadParameters();

  // --- Load WiFi credentials from EEPROM (or use defaults) ---
  loadWifiCredentials();

  // --- Serial config menu (only active when a terminal is connected) ---
  // This must run before WiFi.begin() so any credential change takes effect.
  checkSerialConfig();

  // --- Sensor ---
  initMPU6050();
  calibrateMPU6050();

  // --- PWM inputs ---
  pinMode(PITCH_IP,    INPUT);
  pinMode(ROLL_IP,     INPUT);
  pinMode(YAW_IP,      INPUT);
  pinMode(AUTO_PILOT_IP, INPUT);

  // --- Servos ---
  pitchServo.attach(PITCH_SERVO_PIN);
  rollServo.attach(ROLL_SERVO_PIN);
  pitchServo.write(SERVO_CENTER);
  rollServo.write(SERVO_CENTER);
  lastPitchCmd = SERVO_CENTER;
  lastRollCmd  = SERVO_CENTER;

  // --- Prime complementary filter (50 ticks = 1 s) ---
  Serial.println("Priming IMU...");
  for (int i = 0; i < 50; i++) { updateMPU6050(); delay(20); }

  float p, r;
  getAngles(p, r);
  Serial.printf("IMU ready. pitch=%.2f roll=%.2f\n", p, r);
  Serial.println("Tilt nose UP → pitch+, tilt RIGHT → roll+");
  Serial.println("If wrong: send swapAxes/invertPitch/invertRoll from GCS.");

  // --- WiFi (uses wifiSSID / wifiPass set above) ---
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSSID, wifiPass);
  Serial.printf("WiFi connecting to \"%s\"", wifiSSID);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
    Serial.print('.'); delay(500);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nIP: %s\n", WiFi.localIP().toString().c_str());
    if (MDNS.begin("esp32")) Serial.println("mDNS: esp32.local");
    tcpServer.begin();
    Serial.println("TCP server port 12345");
    xTaskCreatePinnedToCore(communicationTask, "GCS", 12000, NULL, 1, NULL, 0);
    Serial.println("GCS task → Core 0");
    digitalWrite(LED_PIN, HIGH);  // LED on = WiFi connected
  } else {
    Serial.println("\nWiFi failed. Flight-only mode.");
  }
}

// ================================================================
//                      MAIN LOOP  (Core 1)
// ================================================================
void loop() {
  // Rate-limit to FREQ Hz without any blocking delay
  unsigned long now = millis();
  if (now - lastLoopTime < (unsigned long)(1000.0f / FREQ)) return;
  lastLoopTime = now;

  // 1. Update sensor — always, every tick
  updateMPU6050();

  // 2. Read PWM inputs (blocking on Core 1 only — safe)
  uint32_t pw1 = pulseIn(PITCH_IP,      HIGH, PW_TIMEOUT);
  uint32_t pw2 = pulseIn(ROLL_IP,       HIGH, PW_TIMEOUT);
  uint32_t pw3 = pulseIn(YAW_IP,        HIGH, PW_TIMEOUT);
  uint32_t pw4 = pulseIn(AUTO_PILOT_IP, HIGH, PW_TIMEOUT);

  int p1 = constrain(map(pw1, MIN_PW, MAX_PW, 0, 100), 0, 100);
  int p2 = constrain(map(pw2, MIN_PW, MAX_PW, 0, 100), 0, 100);
  int p3 = constrain(map(pw3, MIN_PW, MAX_PW, 0, 100), 0, 100);
  int p4 = constrain(map(pw4, MIN_PW, MAX_PW, 0, 100), 0, 100);

  // Stick deadband — snap ±3% around centre to exactly 50
  if (abs(p1 - 50) < 3) p1 = 50;
  if (abs(p2 - 50) < 3) p2 = 50;
  if (abs(p3 - 50) < 3) p3 = 50;

  // Publish for Core 0 streaming (atomic 32-bit int writes)
  pwmPct1 = p1; pwmPct2 = p2; pwmPct3 = p3; pwmPct4 = p4;

  bool autoActive = (p4 > 90);

  // 3. Mode-transition edge detector
  if (autoActive != lastAutoState) {
    if (autoActive) {
      // Average BIAS_CAL_SAMPLES to absorb sensor mounting offset
      const int SAMPLES = 20;
      double sumP = 0.0, sumR = 0.0;
      for (int i = 0; i < SAMPLES; i++) {
        updateMPU6050();
        float tp, tr;
        getAngles(tp, tr);
        sumP += tp; sumR += tr;
        delay(20);
      }
      referencePitch = (float)(sumP / SAMPLES);
      referenceRoll  = (float)(sumR / SAMPLES);
      pitchOffset    = SERVO_CENTER;
      rollOffset     = SERVO_CENTER;

      float cp, cr;
      getAngles(cp, cr);
      resetPID(pitchPID, cp);
      resetPID(rollPID,  cr);

      Serial.printf("AUTO ON  ref: pitch=%.2f roll=%.2f\n",
                    referencePitch, referenceRoll);
    } else {
      Serial.println("AUTO OFF");
    }
    lastAutoState = autoActive;
  }

  // 4. Apply control
  if (autoActive) {
    bool pitchStickCentred = (abs(p2 - 50) <= 5);
    bool rollStickCentred  = (abs(p1 - 50) <= 5);

    if (pitchStickCentred && rollStickCentred) {
      // Both centred → full PID on both axes
      updateServoFromMPU();
    } else {
      float cp, cr;
      getAngles(cp, cr);

      if (rollStickCentred) {
        // PID on roll only
        float out = computePID(rollPID, referenceRoll, cr, lastRollCmd - rollOffset);
        writeServo(rollServo, lastRollCmd, (int)(rollOffset + out));
      } else {
        resetPID(rollPID, cr);
        writeServo(rollServo, lastRollCmd, map(p1, 0, 100, SERVO_MIN, SERVO_MAX));
      }

      if (pitchStickCentred) {
        // PID on pitch only
        float out = computePID(pitchPID, referencePitch, cp, lastPitchCmd - pitchOffset);
        writeServo(pitchServo, lastPitchCmd, (int)(pitchOffset + out));
      } else {
        resetPID(pitchPID, cp);
        writeServo(pitchServo, lastPitchCmd, map(p2, 0, 100, SERVO_MIN, SERVO_MAX));
      }
    }
  } else {
    // Full manual
    writeServo(rollServo,  lastRollCmd,  map(p1, 0, 100, SERVO_MIN, SERVO_MAX));
    writeServo(pitchServo, lastPitchCmd, map(p2, 0, 100, SERVO_MIN, SERVO_MAX));
  }
}

// ================================================================
//                  SERVO WRITE WITH DEADBAND
// ================================================================
void writeServo(Servo& s, int& last, int angle) {
  angle = constrain(angle, SERVO_MIN, SERVO_MAX);
  if (abs(angle - last) >= (int)SERVO_DEADBAND) {
    s.write(angle);
    last = angle;
  }
}

// ================================================================
//                      PID CONTROLLER
//
//  Derivative-on-measurement: takes d/dt of sensor reading, not error.
//  → Eliminates derivative kick from noise spikes.
//  Error deadband: errors < PID_DEADBAND are zeroed.
//  → Eliminates constant hunting from sensor noise at rest.
//  just_reset flag: skips bumpless transfer on first tick.
//  → Eliminates servo jump on mode switch.
// ================================================================
float computePID(PID& pid, float setpoint, float meas, float curOut) {
  unsigned long now = millis();
  float dt = (now - pid.lastTime) / 1000.0f;
  if (dt <= 0.0f) dt = 1.0f / FREQ;

  float error = setpoint - meas;
  if (fabsf(error) < PID_DEADBAND) error = 0.0f;

  // Bumpless transfer — skipped on first tick after reset
  if (!pid.justReset && pid.Ki != 0.0f) {
    float dM = (meas - pid.prevMeasurement) / dt;
    pid.integral = (curOut - pid.Kp * error + pid.Kd * dM) / pid.Ki;
  }
  pid.justReset = false;

  pid.integral += error * dt;
  pid.integral  = constrain(pid.integral, -50.0f, 50.0f);

  float deriv = -(meas - pid.prevMeasurement) / dt;  // derivative on measurement
  float output = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * deriv;

  pid.prevMeasurement = meas;
  pid.lastTime = now;
  return output;
}

void resetPID(PID& pid, float initMeas) {
  pid.integral        = 0.0f;
  pid.prevMeasurement = initMeas;
  pid.lastTime        = millis();
  pid.justReset       = true;
}

// ================================================================
//                       SERVO CONTROL
// ================================================================
void updateServoFromMPU() {
  float cp, cr;
  getAngles(cp, cr);
  float po = computePID(pitchPID, referencePitch, cp, lastPitchCmd - pitchOffset);
  float ro = computePID(rollPID,  referenceRoll,  cr, lastRollCmd  - rollOffset);
  writeServo(pitchServo, lastPitchCmd, (int)(pitchOffset + po));
  writeServo(rollServo,  lastRollCmd,  (int)(rollOffset  + ro));
}

// ================================================================
//                      MPU6050 FUNCTIONS
// ================================================================
void initMPU6050() {
  Serial.println("Init MPU6050...");
  i2cWrite(MPU6050_ADDR, 0x6B, 0x00);  // wake
  i2cWrite(MPU6050_ADDR, 0x1A, 0x03);  // DLPF 44 Hz
  i2cWrite(MPU6050_ADDR, 0x1B, 0x08);  // gyro ±500°/s
  i2cWrite(MPU6050_ADDR, 0x1C, 0x08);  // accel ±4g
  i2cWrite(MPU6050_ADDR, 0x19, (uint8_t)((1000.0f / FREQ) - 1.0f));
}

void calibrateMPU6050() {
  Serial.println("Calibrating gyro (keep still)...");
  const int N = 500;
  long sx = 0, sy = 0, sz = 0;
  uint8_t d[6];
  for (int i = 0; i < N; i++) {
    if (i2cRead(MPU6050_ADDR, 0x43, d, 6) == 0) {
      sx += (int16_t)((d[0] << 8) | d[1]);
      sy += (int16_t)((d[2] << 8) | d[3]);
      sz += (int16_t)((d[4] << 8) | d[5]);
    }
    delay(2);
  }
  gyrXoffs = (double)sx / N;
  gyrYoffs = (double)sy / N;
  gyrZoffs = (double)sz / N;
  Serial.printf("Gyro offsets: X=%.1f Y=%.1f Z=%.1f\n", gyrXoffs, gyrYoffs, gyrZoffs);
}

void updateMPU6050() {
  static unsigned long prev = 0;
  unsigned long now = millis();
  float dt = (prev == 0) ? (1.0f / FREQ) : (now - prev) / 1000.0f;
  if (dt <= 0.0f) dt = 1.0f / FREQ;
  prev = now;

  uint8_t d[14];
  if (i2cRead(MPU6050_ADDR, 0x3B, d, 14) != 0) return;

  accX = (int16_t)((d[0]  << 8) | d[1]);
  accY = (int16_t)((d[2]  << 8) | d[3]);
  accZ = (int16_t)((d[4]  << 8) | d[5]);
  // d[6..7] = temp, skip
  gyrX = ((int16_t)((d[8]  << 8) | d[9])  - gyrXoffs) / G_SENSITIVITY;
  gyrY = ((int16_t)((d[10] << 8) | d[11]) - gyrYoffs) / G_SENSITIVITY;
  gyrZ = ((int16_t)((d[12] << 8) | d[13]) - gyrZoffs) / G_SENSITIVITY;

  // Low-pass filter accel & gyro
  fax = fax * (1.0 - ACCEL_FILTER) + accX * ACCEL_FILTER;
  fay = fay * (1.0 - ACCEL_FILTER) + accY * ACCEL_FILTER;
  faz = faz * (1.0 - ACCEL_FILTER) + accZ * ACCEL_FILTER;
  fgx = fgx * (1.0 - GYRO_FILTER)  + gyrX * GYRO_FILTER;
  fgy = fgy * (1.0 - GYRO_FILTER)  + gyrY * GYRO_FILTER;
  fgz = fgz * (1.0 - GYRO_FILTER)  + gyrZ * GYRO_FILTER;

  // Accel angles — computed AFTER filter update (not one sample stale)
  // MPU6050 standard orientation (chip flat, dot top-left):
  //   X → RIGHT,  Y → FORWARD,  Z → UP
  // Pitch (nose up/down) uses accX component vs YZ plane
  // Roll  (left/right)   uses accY component vs XZ plane
  double accelPitch = atan2(fax, sqrt(fay * fay + faz * faz)) * 180.0 / M_PI;
  double accelRoll  = atan2(fay, sqrt(fax * fax + faz * faz)) * 180.0 / M_PI;

  // Gyro integration with actual dt
  double np = sensorPitch + fgy * dt;   // Y gyro → pitch rate
  double nr = sensorRoll  + fgx * dt;   // X gyro → roll rate
  double ny = sensorYaw   + fgz * dt;   // Z gyro → yaw rate

  // Complementary filter
  np = np * (1.0 - COMP_FILTER) + accelPitch * COMP_FILTER;
  nr = nr * (1.0 - COMP_FILTER) + accelRoll  * COMP_FILTER;

  // Atomic write (spinlock protects 64-bit double writes)
  portENTER_CRITICAL(&imuLock);
  sensorPitch = np;
  sensorRoll  = nr;
  sensorYaw   = ny;
  portEXIT_CRITICAL(&imuLock);
}

// ================================================================
//                    GCS COMMUNICATION TASK  (Core 0)
//
//  Command reference (all terminated with \n):
//
//  READ:
//    get              → "AF,GF,CF\n"          filter values
//    getPitchPID      → "Kp,Ki,Kd\n"
//    getRollPID       → "Kp,Ki,Kd\n"
//    getStatus        → "pitch,roll,yaw,autoOn,axisSwap,pSign,rSign,pDB,sDB\n"
//
//  WRITE FILTERS:
//    setA{v}          → "OK\n"  ACCEL_FILTER
//    setG{v}          → "OK\n"  GYRO_FILTER
//    setC{v}          → "OK\n"  COMP_FILTER
//
//  WRITE PID:
//    setPitchP{v}     → "OK\n"
//    setPitchI{v}     → "OK\n"
//    setPitchD{v}     → "OK\n"
//    setRollP{v}      → "OK\n"
//    setRollI{v}      → "OK\n"
//    setRollD{v}      → "OK\n"
//
//  WRITE TUNING:
//    setPidDB{v}      → "OK,{v}\n"   PID deadband (degrees)
//    setServoDb{v}    → "OK,{v}\n"   servo deadband (degrees)
//
//  AXIS CONFIG:
//    swapAxes         → "OK,swap={0/1}\n"   toggle axisSwap
//    invertPitch      → "OK,pitchSign={1/-1}\n"
//    invertRoll       → "OK,rollSign={1/-1}\n"
//
//  ACTIONS:
//    calibrate        → "CAL_START\n" ... "CAL_DONE,X,Y,Z\n"
//    save             → "OK\n"
//
//  STREAMING:
//    startPWMStream   → "PWM_STREAM_START\n" then "{p1},{p2},{p3},{p4}\n" @ 5Hz
//    stopPWMStream    → "PWM_STREAM_STOPPED\n"
//    startCubeStream  → "CUBE_STREAM_START\n" then "{roll},{pitch},{yaw}\n" @ 50Hz
//    stopCubeStream   → "CUBE_STREAM_STOPPED\n"
// ================================================================
void communicationTask(void* pv) {
  for (;;) {
    WiFiClient client = tcpServer.available();
    if (!client) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

    Serial.println("[GCS] Client connected.");
    client.setTimeout(5);

    char    cmd[MAX_CMD_LEN + 1];
    char    resp[96];
    bool    streaming_pwm  = false;
    bool    streaming_cube = false;
    unsigned long lastSent = 0;

    while (client.connected()) {

      // --- Incoming command ---
      if (client.available()) {
        size_t n = client.readBytesUntil('\n', cmd, MAX_CMD_LEN);
        cmd[n] = '\0';
        if (n > 0 && cmd[n-1] == '\r') cmd[n-1] = '\0';
        Serial.printf("[GCS] %s\n", cmd);

        if (xSemaphoreTake(configMutex, pdMS_TO_TICKS(30)) == pdTRUE) {

          // ---- READ ----
          if (strcmp(cmd, "get") == 0) {
            snprintf(resp, sizeof(resp), "%.3f,%.3f,%.3f\n",
                     ACCEL_FILTER, GYRO_FILTER, COMP_FILTER);
            client.print(resp);

          } else if (strcmp(cmd, "getPitchPID") == 0) {
            snprintf(resp, sizeof(resp), "%.3f,%.3f,%.3f\n",
                     pitchPID.Kp, pitchPID.Ki, pitchPID.Kd);
            client.print(resp);

          } else if (strcmp(cmd, "getRollPID") == 0) {
            snprintf(resp, sizeof(resp), "%.3f,%.3f,%.3f\n",
                     rollPID.Kp, rollPID.Ki, rollPID.Kd);
            client.print(resp);

          } else if (strcmp(cmd, "getStatus") == 0) {
            double sp, sr, sy;
            portENTER_CRITICAL(&imuLock);
            sp = sensorPitch; sr = sensorRoll; sy = sensorYaw;
            portEXIT_CRITICAL(&imuLock);
            snprintf(resp, sizeof(resp),
                     "%.2f,%.2f,%.2f,%d,%d,%d,%d,%.2f,%.2f\n",
                     (float)sp, (float)sr, (float)sy,
                     (int)lastAutoState,
                     (int)axisSwap, pitchSign, rollSign,
                     PID_DEADBAND, SERVO_DEADBAND);
            client.print(resp);

          // ---- WRITE FILTERS ----
          } else if (strncmp(cmd, "setA", 4) == 0 && cmd[4] != '\0') {
            float v = atof(cmd + 4);
            if (v > 0.0f && v <= 1.0f) { ACCEL_FILTER = v; client.println("OK"); }
            else client.println("ERR:range");

          } else if (strncmp(cmd, "setG", 4) == 0 && cmd[4] != '\0') {
            float v = atof(cmd + 4);
            if (v > 0.0f && v <= 1.0f) { GYRO_FILTER = v; client.println("OK"); }
            else client.println("ERR:range");

          } else if (strncmp(cmd, "setC", 4) == 0 && cmd[4] != '\0') {
            float v = atof(cmd + 4);
            if (v > 0.0f && v <= 1.0f) { COMP_FILTER = v; client.println("OK"); }
            else client.println("ERR:range");

          // ---- WRITE PID ----
          } else if (strncmp(cmd, "setPitchP", 9) == 0 && cmd[9] != '\0') {
            float v = atof(cmd + 9);
            if (v >= 0.0f) { pitchPID.Kp = v; client.println("OK"); }
            else client.println("ERR:negative");

          } else if (strncmp(cmd, "setPitchI", 9) == 0 && cmd[9] != '\0') {
            float v = atof(cmd + 9);
            if (v >= 0.0f) { pitchPID.Ki = v; client.println("OK"); }
            else client.println("ERR:negative");

          } else if (strncmp(cmd, "setPitchD", 9) == 0 && cmd[9] != '\0') {
            float v = atof(cmd + 9);
            if (v >= 0.0f) { pitchPID.Kd = v; client.println("OK"); }
            else client.println("ERR:negative");

          } else if (strncmp(cmd, "setRollP", 8) == 0 && cmd[8] != '\0') {
            float v = atof(cmd + 8);
            if (v >= 0.0f) { rollPID.Kp = v; client.println("OK"); }
            else client.println("ERR:negative");

          } else if (strncmp(cmd, "setRollI", 8) == 0 && cmd[8] != '\0') {
            float v = atof(cmd + 8);
            if (v >= 0.0f) { rollPID.Ki = v; client.println("OK"); }
            else client.println("ERR:negative");

          } else if (strncmp(cmd, "setRollD", 8) == 0 && cmd[8] != '\0') {
            float v = atof(cmd + 8);
            if (v >= 0.0f) { rollPID.Kd = v; client.println("OK"); }
            else client.println("ERR:negative");

          // ---- WRITE TUNING ----
          } else if (strncmp(cmd, "setPidDB", 8) == 0 && cmd[8] != '\0') {
            float v = atof(cmd + 8);
            if (v >= 0.0f && v <= 10.0f) {
              PID_DEADBAND = v;
              snprintf(resp, sizeof(resp), "OK,%.2f\n", v);
              client.print(resp);
            } else client.println("ERR:range");

          } else if (strncmp(cmd, "setServoDb", 10) == 0 && cmd[10] != '\0') {
            float v = atof(cmd + 10);
            if (v >= 0.0f && v <= 10.0f) {
              SERVO_DEADBAND = v;
              snprintf(resp, sizeof(resp), "OK,%.2f\n", v);
              client.print(resp);
            } else client.println("ERR:range");

          // ---- AXIS CONFIG ----
          } else if (strcmp(cmd, "swapAxes") == 0) {
            axisSwap = !axisSwap;
            snprintf(resp, sizeof(resp), "OK,swap=%d\n", (int)axisSwap);
            client.print(resp);
            Serial.printf("Axis swap = %d\n", (int)axisSwap);

          } else if (strcmp(cmd, "invertPitch") == 0) {
            pitchSign = -pitchSign;
            snprintf(resp, sizeof(resp), "OK,pitchSign=%d\n", pitchSign);
            client.print(resp);
            Serial.printf("pitchSign = %d\n", pitchSign);

          } else if (strcmp(cmd, "invertRoll") == 0) {
            rollSign = -rollSign;
            snprintf(resp, sizeof(resp), "OK,rollSign=%d\n", rollSign);
            client.print(resp);
            Serial.printf("rollSign = %d\n", rollSign);

          // ---- CALIBRATE ----
          //  Re-runs gyro bias calibration. Board must be still.
          //  Returns offsets when done.
          } else if (strcmp(cmd, "calibrate") == 0) {
            client.println("CAL_START");
            xSemaphoreGive(configMutex);
            // Run calibration (takes ~1 s, releases mutex first)
            calibrateMPU6050();
            // Re-take mutex to send response
            if (xSemaphoreTake(configMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
              snprintf(resp, sizeof(resp), "CAL_DONE,%.1f,%.1f,%.1f\n",
                       gyrXoffs, gyrYoffs, gyrZoffs);
              client.print(resp);
            }
            // mutex given below at end of block — skip the Give at the end
            goto mutex_already_given;

          // ---- SAVE ----
          } else if (strcmp(cmd, "save") == 0) {
            saveParametersUnlocked();
            client.println("OK");

          // ---- STREAMING ----
          } else if (strcmp(cmd, "startPWMStream") == 0) {
            streaming_cube = false; streaming_pwm = true; lastSent = 0;
            client.println("PWM_STREAM_START");

          } else if (strcmp(cmd, "stopPWMStream") == 0) {
            streaming_pwm = false;
            client.println("PWM_STREAM_STOPPED");

          } else if (strcmp(cmd, "startCubeStream") == 0) {
            streaming_pwm = false; streaming_cube = true; lastSent = 0;
            client.println("CUBE_STREAM_START");

          } else if (strcmp(cmd, "stopCubeStream") == 0) {
            streaming_cube = false;
            client.println("CUBE_STREAM_STOPPED");

          } else {
            client.println("ERR:unknown");
          }

          xSemaphoreGive(configMutex);
          mutex_already_given:;
        } else {
          client.println("ERR:busy");
        }
      }

      // --- PWM streaming (5 Hz) ---
      if (streaming_pwm && millis() - lastSent >= 200) {
        int p1 = pwmPct1, p2 = pwmPct2, p3 = pwmPct3, p4 = pwmPct4;
        if (p4 == 0)
          client.println("No signal");
        else
          client.printf("%d,%d,%d,%d\n", p1, p2, p3, p4);
        lastSent = millis();
      }

      // --- IMU streaming (50 Hz) ---
      //  Format: roll,pitch,yaw  — matches cube_visualizer2.py which does:
      //    gx, gy, gz = data  →  self.ax, self.ay, self.az
      //    glRotatef(self.ay, 1,0,0)  = pitch
      //    glRotatef(self.az, 0,1,0)  = yaw
      //    glRotatef(-self.ax, 0,0,1) = roll
      //  So: first field → roll (ax), second → pitch (ay), third → yaw (az)
      if (streaming_cube && millis() - lastSent >= (unsigned long)(1000.0f / FREQ)) {
        double sp, sr, sy;
        portENTER_CRITICAL(&imuLock);
        sp = sensorPitch; sr = sensorRoll; sy = sensorYaw;
        portEXIT_CRITICAL(&imuLock);

        // Apply axis config for the visualizer too
        float vp, vr;
        if (axisSwap) { vp = (float)sr * pitchSign; vr = (float)sp * rollSign; }
        else          { vp = (float)sp * pitchSign; vr = (float)sr * rollSign; }

        client.printf("%.2f,%.2f,%.2f\n", vr, vp, (float)sy);
        lastSent = millis();
      }

      vTaskDelay(pdMS_TO_TICKS(1));
    }

    client.stop();
    streaming_pwm = false; streaming_cube = false;
    cubeStreaming  = false; pwmStreaming   = false;
    Serial.println("[GCS] Client disconnected.");
  }
}

// ================================================================
//                           EEPROM
// ================================================================
void loadParameters() {
  if (xSemaphoreTake(configMutex, portMAX_DELAY) != pdTRUE) return;

  EEPROM.get(ACCEL_FILTER_ADDR,  ACCEL_FILTER);
  EEPROM.get(GYRO_FILTER_ADDR,   GYRO_FILTER);
  EEPROM.get(COMP_FILTER_ADDR,   COMP_FILTER);
  EEPROM.get(PITCH_PID_ADDR,     pitchPID.Kp);
  EEPROM.get(PITCH_PID_ADDR + 4, pitchPID.Ki);
  EEPROM.get(PITCH_PID_ADDR + 8, pitchPID.Kd);
  EEPROM.get(ROLL_PID_ADDR,      rollPID.Kp);
  EEPROM.get(ROLL_PID_ADDR + 4,  rollPID.Ki);
  EEPROM.get(ROLL_PID_ADDR + 8,  rollPID.Kd);

  uint8_t axCfg = 0;
  EEPROM.get(AXIS_CONFIG_ADDR, axCfg);
  axisSwap  = (axCfg & 0x01) != 0;
  pitchSign = (axCfg & 0x02) ? -1 : 1;
  rollSign  = (axCfg & 0x04) ? -1 : 1;

  EEPROM.get(PID_DB_ADDR,   PID_DEADBAND);
  EEPROM.get(SERVO_DB_ADDR, SERVO_DEADBAND);

  // Validate — NaN happens on blank EEPROM
  if (isnan(ACCEL_FILTER) || ACCEL_FILTER <= 0.0f || ACCEL_FILTER > 1.0f) ACCEL_FILTER = 0.3f;
  if (isnan(GYRO_FILTER)  || GYRO_FILTER  <= 0.0f || GYRO_FILTER  > 1.0f) GYRO_FILTER  = 0.08f;
  if (isnan(COMP_FILTER)  || COMP_FILTER  <= 0.0f || COMP_FILTER  > 1.0f) COMP_FILTER  = 0.7f;
  if (isnan(pitchPID.Kp) || pitchPID.Kp < 0.0f) pitchPID.Kp = 2.0f;
  if (isnan(pitchPID.Ki) || pitchPID.Ki < 0.0f) pitchPID.Ki = 0.1f;
  if (isnan(pitchPID.Kd) || pitchPID.Kd < 0.0f) pitchPID.Kd = 0.5f;
  if (isnan(rollPID.Kp)  || rollPID.Kp  < 0.0f) rollPID.Kp  = 2.0f;
  if (isnan(rollPID.Ki)  || rollPID.Ki  < 0.0f) rollPID.Ki  = 0.1f;
  if (isnan(rollPID.Kd)  || rollPID.Kd  < 0.0f) rollPID.Kd  = 0.5f;
  if (isnan(PID_DEADBAND)   || PID_DEADBAND   < 0.0f) PID_DEADBAND   = 0.8f;
  if (isnan(SERVO_DEADBAND) || SERVO_DEADBAND < 0.0f) SERVO_DEADBAND = 1.0f;

  xSemaphoreGive(configMutex);
}

// Called while configMutex is ALREADY HELD — does not re-take it.
void saveParametersUnlocked() {
  EEPROM.put(ACCEL_FILTER_ADDR,  ACCEL_FILTER);
  EEPROM.put(GYRO_FILTER_ADDR,   GYRO_FILTER);
  EEPROM.put(COMP_FILTER_ADDR,   COMP_FILTER);
  EEPROM.put(PITCH_PID_ADDR,     pitchPID.Kp);
  EEPROM.put(PITCH_PID_ADDR + 4, pitchPID.Ki);
  EEPROM.put(PITCH_PID_ADDR + 8, pitchPID.Kd);
  EEPROM.put(ROLL_PID_ADDR,      rollPID.Kp);
  EEPROM.put(ROLL_PID_ADDR + 4,  rollPID.Ki);
  EEPROM.put(ROLL_PID_ADDR + 8,  rollPID.Kd);

  uint8_t axCfg = 0;
  if (axisSwap)    axCfg |= 0x01;
  if (pitchSign<0) axCfg |= 0x02;
  if (rollSign <0) axCfg |= 0x04;
  EEPROM.put(AXIS_CONFIG_ADDR, axCfg);
  EEPROM.put(PID_DB_ADDR,      PID_DEADBAND);
  EEPROM.put(SERVO_DB_ADDR,    SERVO_DEADBAND);

  EEPROM.commit();
  Serial.println("Parameters saved.");
}

// ================================================================
//                         I2C HELPERS
// ================================================================
int i2cRead(int addr, int reg, uint8_t* buf, int len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  Wire.requestFrom((uint8_t)addr, (size_t)len, (bool)true);
  int i = 0;
  while (Wire.available() && i < len) buf[i++] = Wire.read();
  return (i == len) ? 0 : -1;
}

int i2cWrite(int addr, int reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission(true);
}