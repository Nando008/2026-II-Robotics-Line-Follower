/*
 * =============================================================================
 *  MOTOR TEST - Arduino Nano + TB6612FNG
 * =============================================================================
 *  Sequence (each step is preceded by a 5 s wait with the LED off):
 *    1. LED blinking + BOTH motors ramp up from 0 to RAMP_MAX_PWM.
 *    2. LED solid    + LEFT motor ramps up.
 *    3. LED off      + RIGHT motor ramps up.
 *  Each ramp lasts RAMP_DURATION_MS, always forward. Motors stop after each.
 *
 *  Serial Monitor: 115200 baud. Every PWM change is printed with the elapsed
 *  time, so you can match what you see with the PWM value.
 *
 *  What to look for:
 *   - Each wheel turns FORWARD. If one turns backward, flip its
 *     INVERT_*_MOTOR constant (or swap its two motor wires).
 *   - In step 2 only the left wheel moves, in step 3 only the right one.
 *     If it's the other way around, the left/right pins are swapped.
 *   - The PWM at which each wheel STARTS turning: that is the base for
 *     MIN_SPEED in line_follower.ino (use the higher of both + ~10%).
 *     Measure it with the wheels on the floor, carrying the robot's weight,
 *     and hold the robot back once the wheel starts.
 *   - Both wheels should reach a similar speed at the same PWM. A big
 *     difference means the PID will have to compensate a built-in bias.
 *
 *  WARNING: the ramp goes up to full power (255). Lift the robot or hold it
 *  firmly so it does not drive off the table.
 *
 *  Note: D13 also drives the QTR emitters. If the QTR is connected, its IR
 *  LEDs will switch on and off together with the LED. This is harmless.
 * =============================================================================
 */

// ================== Pins ==================
constexpr uint8_t PIN_PWM_LEFT  = 9;
constexpr uint8_t PIN_LEFT_IN1  = 7;
constexpr uint8_t PIN_LEFT_IN2  = 8;

constexpr uint8_t PIN_PWM_RIGHT = 3;
constexpr uint8_t PIN_RIGHT_IN1 = 5;
constexpr uint8_t PIN_RIGHT_IN2 = 4;

constexpr uint8_t PIN_STANDBY   = 6;
constexpr uint8_t PIN_LED       = 13;

// Motor direction correction. Set to true if that motor turns BACKWARD in
// the ramps below (motor wires or IN1/IN2 swapped). Once both wheels turn
// forward here, copy the same values into line_follower.ino.
constexpr bool INVERT_LEFT_MOTOR  = true;   // left motor ran backward in tests
constexpr bool INVERT_RIGHT_MOTOR = false;

// D12 is wired to a pushbutton connected to 5V: keep it as INPUT
// (see line_follower.ino for the reason).
constexpr uint8_t PIN_UNUSED_BUTTON = 12;

// ================== Test settings ==================
constexpr uint32_t SERIAL_BAUD         = 115200;
constexpr uint32_t WAIT_BEFORE_STEP_MS = 5000;
constexpr uint32_t RAMP_DURATION_MS    = 5000;
constexpr uint8_t  RAMP_MAX_PWM        = 255;
constexpr uint32_t BLINK_INTERVAL_MS   = 250;

// false: run the sequence once and stop. true: repeat it forever.
constexpr bool REPEAT_SEQUENCE = false;

// ================== Types ==================
enum class MotorSelection : uint8_t { Both, Left, Right };
enum class LedMode : uint8_t { Off, Solid, Blink };

bool sequenceDone = false;

// ================== Function prototypes ==================
void runStep(const __FlashStringHelper *name, MotorSelection motors, LedMode ledMode);
void rampMotors(MotorSelection motors, LedMode ledMode);
void setSelectedMotorsForward(MotorSelection motors, uint8_t pwm);
void setMotorForward(uint8_t in1Pin, uint8_t in2Pin, uint8_t pwmPin, bool inverted, uint8_t pwm);
void stopMotors();

void setup() {
  pinMode(PIN_PWM_LEFT, OUTPUT);
  pinMode(PIN_LEFT_IN1, OUTPUT);
  pinMode(PIN_LEFT_IN2, OUTPUT);

  pinMode(PIN_PWM_RIGHT, OUTPUT);
  pinMode(PIN_RIGHT_IN1, OUTPUT);
  pinMode(PIN_RIGHT_IN2, OUTPUT);

  pinMode(PIN_STANDBY, OUTPUT);
  digitalWrite(PIN_STANDBY, HIGH);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  pinMode(PIN_UNUSED_BUTTON, INPUT);

  stopMotors();

  Serial.begin(SERIAL_BAUD);
  Serial.println(F("=== MOTOR TEST ==="));
  Serial.println(F("Lift the robot or hold it: the ramp reaches full power."));
}

void loop() {
  if (sequenceDone) {
    return;
  }

  runStep(F("BOTH motors (LED blinking)"), MotorSelection::Both, LedMode::Blink);
  runStep(F("LEFT motor (LED solid)"), MotorSelection::Left, LedMode::Solid);
  runStep(F("RIGHT motor (LED off)"), MotorSelection::Right, LedMode::Off);

  Serial.println(F("\n=== SEQUENCE FINISHED ==="));

  if (!REPEAT_SEQUENCE) {
    sequenceDone = true;
  }
}

void runStep(const __FlashStringHelper *name, MotorSelection motors, LedMode ledMode) {
  Serial.print(F("\nWaiting 5 s before: "));
  Serial.println(name);
  digitalWrite(PIN_LED, LOW);
  delay(WAIT_BEFORE_STEP_MS);

  Serial.print(F("Ramp start: "));
  Serial.println(name);
  rampMotors(motors, ledMode);

  stopMotors();
  digitalWrite(PIN_LED, LOW);
  Serial.println(F("Motors stopped."));
}

void rampMotors(MotorSelection motors, LedMode ledMode) {
  const uint32_t startTime = millis();
  uint32_t lastBlinkTime = startTime;
  bool ledOn = (ledMode != LedMode::Off);
  int lastPwm = -1;

  digitalWrite(PIN_LED, ledOn ? HIGH : LOW);

  while (true) {
    const uint32_t now = millis();
    const uint32_t elapsed = now - startTime;
    if (elapsed >= RAMP_DURATION_MS) {
      break;
    }

    // Linear ramp: 0 at the start, RAMP_MAX_PWM in the last moments.
    uint32_t pwm = elapsed * (static_cast<uint32_t>(RAMP_MAX_PWM) + 1) / RAMP_DURATION_MS;
    if (pwm > RAMP_MAX_PWM) {
      pwm = RAMP_MAX_PWM;
    }

    if (static_cast<int>(pwm) != lastPwm) {
      setSelectedMotorsForward(motors, static_cast<uint8_t>(pwm));
      lastPwm = static_cast<int>(pwm);

      Serial.print(F("PWM="));
      Serial.print(pwm);
      Serial.print(F("  t="));
      Serial.print(elapsed);
      Serial.println(F(" ms"));
    }

    if (ledMode == LedMode::Blink && now - lastBlinkTime >= BLINK_INTERVAL_MS) {
      lastBlinkTime += BLINK_INTERVAL_MS;
      ledOn = !ledOn;
      digitalWrite(PIN_LED, ledOn ? HIGH : LOW);
    }
  }
}

void setSelectedMotorsForward(MotorSelection motors, uint8_t pwm) {
  if (motors == MotorSelection::Both || motors == MotorSelection::Left) {
    setMotorForward(PIN_LEFT_IN1, PIN_LEFT_IN2, PIN_PWM_LEFT, INVERT_LEFT_MOTOR, pwm);
  }
  if (motors == MotorSelection::Both || motors == MotorSelection::Right) {
    setMotorForward(PIN_RIGHT_IN1, PIN_RIGHT_IN2, PIN_PWM_RIGHT, INVERT_RIGHT_MOTOR, pwm);
  }
}

// "Forward" from the robot's point of view: if the motor is inverted, the
// IN1/IN2 levels are swapped so the wheel still turns forward.
void setMotorForward(uint8_t in1Pin, uint8_t in2Pin, uint8_t pwmPin, bool inverted, uint8_t pwm) {
  digitalWrite(in1Pin, inverted ? LOW : HIGH);
  digitalWrite(in2Pin, inverted ? HIGH : LOW);
  analogWrite(pwmPin, pwm);
}

void stopMotors() {
  analogWrite(PIN_PWM_LEFT, 0);
  analogWrite(PIN_PWM_RIGHT, 0);

  digitalWrite(PIN_LEFT_IN1, LOW);
  digitalWrite(PIN_LEFT_IN2, LOW);
  digitalWrite(PIN_RIGHT_IN1, LOW);
  digitalWrite(PIN_RIGHT_IN2, LOW);
}
