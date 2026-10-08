#include <QTRSensors.h>

/*
 * =============================================================================
 *  QTR-8A TEST - Arduino Nano
 * =============================================================================
 *  Sequence:
 *    1. Calibration (~4-5 s, same as line_follower.ino). During this time
 *       slide the robot so EVERY sensor passes over the line and over the
 *       background several times.
 *    2. Wait START_DELAY_MS (2 s).
 *    3. Every PRINT_INTERVAL_MS (100 ms) print to Serial:
 *         - the RAW value of each sensor (0-1023, higher = darker),
 *         - the calibrated value (0-1000) of the two center sensors,
 *         - the estimate: ON LINE / OUTSIDE.
 *
 *  Estimate: the robot is ON LINE when at least one of the two center
 *  sensors (indexes 3 and 4) has a calibrated value >= BLACK_THRESHOLD.
 *  The estimate uses calibrated values, which is why the two center ones are
 *  printed too: raw values alone can't be compared against the threshold.
 *
 *  Serial Monitor: 115200 baud.
 *
 *  What to look for:
 *   - On white background every RAW value should be low and similar between
 *     sensors; over the black line it should be clearly higher. A sensor
 *     that barely changes is dirty, too high above the floor, or faulty.
 *   - Move the line slowly from one side to the other: the high RAW value
 *     should travel across the sensors in order (index 0 -> 7 or 7 -> 0).
 *     If the order jumps, the sensor pins are in the wrong order.
 *   - The estimate should switch to ON LINE only when the line is under the
 *     middle of the bar. If it flickers at the edges, adjust BLACK_THRESHOLD.
 *
 *  Note: D13 is the QTR emitter pin, so the Nano's "L" LED flickers while
 *  the sensors are read. That is expected.
 * =============================================================================
 */

// ================== Line sensor (QTR-8A) ==================
constexpr uint8_t SENSOR_COUNT    = 8;
constexpr uint8_t PIN_QTR_EMITTER = 13;
const uint8_t QTR_PINS[SENSOR_COUNT] = {A7, A6, A5, A4, A3, A2, A1, A0};

// Center sensors used for the on-line estimate (indexes into QTR_PINS).
constexpr uint8_t CENTER_SENSOR_LEFT  = 3;  // A4
constexpr uint8_t CENTER_SENSOR_RIGHT = 4;  // A3

// Calibrated value (0-1000) from which a sensor is considered to see black.
//  - ON LINE when the line is clearly off-center: raise it.
//  - OUTSIDE when the line is right under the center: lower it.
constexpr uint16_t BLACK_THRESHOLD = 500;

// ================== Timing (same as line_follower.ino) ==================
constexpr uint16_t CALIBRATION_SAMPLES         = 400;
constexpr uint16_t CALIBRATION_SAMPLE_DELAY_MS = 10;
constexpr uint16_t START_DELAY_MS              = 2000;
constexpr uint32_t PRINT_INTERVAL_MS           = 100;
constexpr uint32_t SERIAL_BAUD                 = 115200;

// ================== Other hardware (kept safe, not used) ==================
// TB6612FNG standby: LOW keeps both motors off during this test.
constexpr uint8_t PIN_STANDBY = 6;
// D12 is wired to a pushbutton connected to 5V: keep it as INPUT
// (see line_follower.ino for the reason).
constexpr uint8_t PIN_UNUSED_BUTTON = 12;

// ================== State ==================
QTRSensors qtr;
uint16_t rawValues[SENSOR_COUNT];
uint16_t calibratedValues[SENSOR_COUNT];
uint32_t lastPrintTime = 0;

// ================== Function prototypes ==================
void calibrateSensors();
bool isOnLine();
void printReadings(bool onLine);

void setup() {
  pinMode(PIN_STANDBY, OUTPUT);
  digitalWrite(PIN_STANDBY, LOW);
  pinMode(PIN_UNUSED_BUTTON, INPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins(QTR_PINS, SENSOR_COUNT);
  qtr.setEmitterPin(PIN_QTR_EMITTER);

  Serial.begin(SERIAL_BAUD);
  Serial.println(F("=== QTR TEST ==="));
  Serial.println(F("Calibrating: slide every sensor over the line and the background..."));

  calibrateSensors();

  Serial.println(F("Calibration done. Starting in 2 s."));
  delay(START_DELAY_MS);

  Serial.println(F("RAW: s0 s1 s2 s3 s4 s5 s6 s7 | CENTER cal: s3 s4 | ESTIMATE"));
}

void loop() {
  const uint32_t now = millis();
  if (now - lastPrintTime < PRINT_INTERVAL_MS) {
    return;
  }
  lastPrintTime = now;

  qtr.read(rawValues);
  qtr.readCalibrated(calibratedValues);

  printReadings(isOnLine());
}

void calibrateSensors() {
  for (uint16_t i = 0; i < CALIBRATION_SAMPLES; i++) {
    qtr.calibrate();
    delay(CALIBRATION_SAMPLE_DELAY_MS);
  }
}

bool isOnLine() {
  return calibratedValues[CENTER_SENSOR_LEFT] >= BLACK_THRESHOLD ||
         calibratedValues[CENTER_SENSOR_RIGHT] >= BLACK_THRESHOLD;
}

void printReadings(bool onLine) {
  Serial.print(F("RAW:"));
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(' ');
    Serial.print(rawValues[i]);
  }

  Serial.print(F(" | CENTER cal: "));
  Serial.print(calibratedValues[CENTER_SENSOR_LEFT]);
  Serial.print(' ');
  Serial.print(calibratedValues[CENTER_SENSOR_RIGHT]);

  Serial.print(F(" | "));
  Serial.println(onLine ? F("ON LINE") : F("OUTSIDE"));
}
