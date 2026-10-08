#include <QTRSensors.h>

/*
 * =============================================================================
 *  LINE FOLLOWER - Arduino Nano + TB6612FNG + QTR-8A
 * =============================================================================
 *  Behavior: on power-up the QTR sensors calibrate (slide the robot over the
 *  line by hand during this time), then it waits START_DELAY_MS and starts
 *  following the line. There is no way to stop it except cutting power.
 *
 * -----------------------------------------------------------------------------
 *  HOW TO TUNE (read this before touching any constant)
 * -----------------------------------------------------------------------------
 *  Measuring:
 *   - Serial is not usable while the robot moves, so measure from the outside:
 *       1. Lap time on the same track, with a stopwatch.
 *       2. Number of times it loses the line per lap (you will see it spin in
 *          place: that is recoverLine() running).
 *       3. Wobble on straights. Record with a phone (slow motion if possible).
 *   - Do at least 3 runs per setting and compare averages, not single runs.
 *   - Change ONE constant at a time and write down the value and the result.
 *   - Battery level changes motor speed. Compare runs with similar charge.
 *
 *  Recommended order:
 *   0. Check directions before tuning anything:
 *        - Both wheels turn forward (motor_test.ino, INVERT_*_MOTOR).
 *        - The robot steers TOWARD the line (INVERT_LINE_POSITION). If it
 *          steers away and gets worse the more off-center it is, flip it.
 *      No PID value can fix a wrong direction.
 *   1. Check calibration (see CALIBRATION section below).
 *   2. Find MIN_SPEED for each motor (see SPEEDS section).
 *   3. Tune the PID at a low BASE_SPEED:
 *        a. Set KI = 0 and KD = 0.
 *        b. Raise KP until the robot follows the line but wobbles on
 *           straights. Then lower it ~20-30%.
 *        c. Raise KD until the wobble disappears. If the robot starts to
 *           shake/jitter (fast small movements), KD is too high.
 *        d. Only if the robot consistently runs off-center (always to the
 *           same side, or always late in long curves), add a small KI.
 *           If it starts to swing in slow, wide waves, KI is too high.
 *   4. Raise BASE_SPEED in small steps (~5-10). At each step, if it wobbles
 *      again, raise KD a bit; if it misses curves, raise KP a bit.
 *
 *  Steps: KP and KD in ~20-30% steps; KI in x2 or /2 steps.
 *
 *  Note: KD and KI depend on how fast loop() runs. If you add code that makes
 *  the loop slower or faster, the same KD/KI will behave differently.
 * =============================================================================
 */

// ================== Motor driver (TB6612FNG) ==================
constexpr uint8_t PIN_PWM_LEFT  = 9;
constexpr uint8_t PIN_LEFT_IN1  = 7;
constexpr uint8_t PIN_LEFT_IN2  = 8;

constexpr uint8_t PIN_PWM_RIGHT = 3;
constexpr uint8_t PIN_RIGHT_IN1 = 5;
constexpr uint8_t PIN_RIGHT_IN2 = 4;

constexpr uint8_t PIN_STANDBY   = 6;

// Motor direction correction. Set to true if that motor turns BACKWARD when
// it should go forward (motor wires or IN1/IN2 swapped). Use motor_test.ino
// to check: in its ramps, every wheel must turn forward.
// Keep these values identical in motor_test.ino.
constexpr bool INVERT_LEFT_MOTOR  = false;
constexpr bool INVERT_RIGHT_MOTOR = false;

// D12 is wired to a pushbutton whose other leg goes to 5V. It is not used,
// but it must stay as INPUT: if it were an OUTPUT driven LOW, pressing the
// button would short 5V to ground through the pin.
constexpr uint8_t PIN_UNUSED_BUTTON = 12;

// TODO(pushbutton): Use the pushbutton on D12. NOT IMPLEMENTED.
//   Blockers:
//   1. Wiring: when the button is released, D12 is not connected to anything
//      (floating) and reads HIGH or LOW at random. The Nano has internal
//      pull-ups but no pull-downs. Fix one of:
//        a) Add ~10 kOhm resistor between D12 and GND. Pressed = HIGH.
//        b) Move the button's second leg from 5V to GND and use
//           pinMode(PIN, INPUT_PULLUP). Pressed = LOW. No extra parts.
//   2. Behavior not defined: what the button should do (start calibration,
//      start/stop, recalibrate after stop...).
//   When unblocked: add debouncing and a small state machine
//   (IDLE -> CALIBRATING -> WAITING -> RUNNING -> STOPPED).

// TODO(status LED): Blink an LED during calibration. NOT IMPLEMENTED.
//   Blocker: the LED is wired from the 5V line to GND, so it is always on and
//   the code cannot control it. It must be moved to a free digital pin
//   (pin -> resistor -> LED -> GND). D13 is taken by the QTR emitter.

// TODO(IR): Control the robot with an IR remote. NOT IMPLEMENTED.
//   Blockers:
//   1. There is no IR receiver in the current hardware.
//   2. Timer conflict: on the Nano (ATmega328), the IRremote library uses
//      Timer2 by default. Timer2 also generates the PWM on D3 (right motor)
//      and D11. With IR enabled, analogWrite() on D3 stops working, which is
//      exactly what broke the right motor in the old code.
//      Options (each needs rewiring and checking the IRremote docs):
//        a) Move the right motor PWM to a pin not driven by the timer that
//           IRremote uses. On the Nano: Timer0 -> D5/D6 (currently used by
//           RIGHT_IN1 and STBY), Timer1 -> D9/D10 (D9 is the left PWM).
//        b) Configure IRremote to use another timer, if the library version
//           supports it for this board; that timer's PWM pins are then lost.
//   3. The command codes (0x10/0x11/0x12 in the old code) are specific to one
//      remote; they must be read from the actual remote that will be used.

// ================== Line sensor (QTR-8A) ==================
constexpr uint8_t SENSOR_COUNT    = 8;
constexpr uint8_t PIN_QTR_EMITTER = 13;
const uint8_t QTR_PINS[SENSOR_COUNT] = {A7, A6, A5, A4, A3, A2, A1, A0};

// Highest position readLineBlack() can return: (SENSOR_COUNT - 1) * 1000.
constexpr uint16_t LINE_POSITION_MAX = (SENSOR_COUNT - 1) * 1000;

// Sensor bar orientation. The rest of the code expects:
//   position < LINE_CENTER  ->  line is on the robot's LEFT
//   position > LINE_CENTER  ->  line is on the robot's RIGHT
// If the robot steers AWAY from the line (it gets worse the more off-center
// it is, and it spins the wrong way when it loses the line), flip this.
// It corrects both the PID and recoverLine(), since both use linePosition.
constexpr bool INVERT_LINE_POSITION = true;

// ---- CALIBRATION ----
// Total calibration time ~= CALIBRATION_SAMPLES * (CALIBRATION_SAMPLE_DELAY_MS
// + read time). Currently ~4-5 s. During that time slide the robot so EVERY
// sensor passes over the line and over the background several times.
//  - If the robot behaves differently each time you power it on, calibration
//    is probably incomplete: increase CALIBRATION_SAMPLES or sweep more times.
//  - Checking on the bench (robot still, USB connected, a separate test
//    sketch): after calibrating, print qtr.calibrationOn.minimum[i] and
//    .maximum[i]. Every sensor should have a clear gap between min and max.
//    A sensor with min close to max did not see the line or the background.
//  - Recalibrate if the lighting or the track surface changes.
constexpr uint16_t CALIBRATION_SAMPLES         = 400;
constexpr uint16_t CALIBRATION_SAMPLE_DELAY_MS = 10;

// Time between the end of calibration and the start, to place the robot on
// the line. Raise it if you need more time.
constexpr uint16_t START_DELAY_MS = 1500;

// ================== Line position (0..LINE_POSITION_MAX) ==================
// LINE_CENTER: position reported when the line is under the middle of the
// sensor bar. Do not change unless the bar is mounted off-center.
constexpr int16_t LINE_CENTER = LINE_POSITION_MAX / 2;

// When the position goes past these limits, the robot considers the line
// lost and runs recoverLine() (spins toward the side where it was last seen).
//  - If it spins on curves it could have taken normally: move the limits
//    closer to the ends (e.g. 150 / 6850).
//  - If it runs off the line before reacting: move them inward
//    (e.g. 500 / 6500).
constexpr int16_t LINE_LOST_LOW  = 250;
constexpr int16_t LINE_LOST_HIGH = 6750;

// ================== PID ==================
// The error goes from -3500 to +3500. The correction is limited to
// +-BASE_SPEED, so it saturates quickly:
//  - KP alone saturates when |error| > BASE_SPEED / KP (currently 80 / 0.08
//    = 1000, i.e. one sensor away from the center).
//  - KP: how hard it turns for a given offset. Too low: misses curves.
//    Too high: wobbles on straights.
//  - KD: reacts to how fast the error changes; damps the wobble. Too low:
//    wobble. Too high: jitter, and it amplifies sensor noise.
//  - KI: corrects a persistent offset. Here the integral is the sum of the
//    previous INTEGRAL_WINDOW errors only (short memory), so it acts more
//    like extra "push" on sustained curves than a classic integral.
//    Too high: slow, wide oscillation.
constexpr float   KP = 0.08f;
constexpr float   KI = 0.01f;
constexpr float   KD = 1.8f;
constexpr uint8_t INTEGRAL_WINDOW = 6;

// ================== SPEEDS (PWM duty, 0-255) ==================
// BASE_SPEED: speed of both wheels on a centered line. During line following
// no wheel ever goes above this value (the PID only slows one wheel down).
// Raise it only after the PID is stable at the current value.
constexpr int BASE_SPEED = 80;

// MAX_SPEED: cap applied to every motor command in driveMotor(). Since line
// following never exceeds BASE_SPEED, in practice this cap only affects
// recoverLine(): see the note on the brake speeds below.
constexpr int MAX_SPEED  = 90;

// MIN_SPEED: any non-zero command below this is raised to it, because below
// a certain PWM the motor hums but does not turn.
//  - How to find it: on the floor, with the robot's weight, raise the PWM of
//    each motor from 0 until the wheel starts turning reliably. Use the higher
//    of the two motors' values plus ~10%.
//  - Too low: a wheel stops in curves instead of slowing down.
//    Too high: the PID loses resolution (fewer steps between MIN and BASE).
constexpr int MIN_SPEED  = 50;

// Brake / recovery speeds used by recoverLine().
// NOTE: both are currently capped to MAX_SPEED (90) by driveMotor(), so the
// robot actually recovers at -90 / +90, not -120 / +220. Pending decision.
constexpr int BRAKE_FORWARD_SPEED = 220;
constexpr int BRAKE_REVERSE_SPEED = 120;

// ================== State ==================
QTRSensors qtr;
uint16_t sensorValues[SENSOR_COUNT];

uint16_t linePosition = LINE_CENTER;
int lastError = 0;
int errorHistory[INTEGRAL_WINDOW] = {0};
uint8_t errorHistoryIndex = 0;

// ================== Function prototypes ==================
void calibrateSensors();
uint16_t readLinePosition();
void followLine();
void recoverLine();
int applyMinimumSpeed(int speed);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void driveMotor(uint8_t in1Pin, uint8_t in2Pin, uint8_t pwmPin, bool inverted, int speed);
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

  pinMode(PIN_UNUSED_BUTTON, INPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins(QTR_PINS, SENSOR_COUNT);
  qtr.setEmitterPin(PIN_QTR_EMITTER);

  stopMotors();
  calibrateSensors();
  delay(START_DELAY_MS);
}

void loop() {
  linePosition = readLinePosition();

  if (linePosition <= LINE_LOST_LOW || linePosition >= LINE_LOST_HIGH) {
    recoverLine();
  } else {
    followLine();
  }
}

void calibrateSensors() {
  for (uint16_t i = 0; i < CALIBRATION_SAMPLES; i++) {
    qtr.calibrate();
    delay(CALIBRATION_SAMPLE_DELAY_MS);
  }
}

// Line position from the robot's point of view:
// 0 = far left, LINE_CENTER = centered, LINE_POSITION_MAX = far right.
uint16_t readLinePosition() {
  uint16_t position = qtr.readLineBlack(sensorValues);

  if (INVERT_LINE_POSITION) {
    position = LINE_POSITION_MAX - position;
  }

  return position;
}

void followLine() {
  const int error = static_cast<int>(linePosition) - LINE_CENTER;
  const int derivative = error - lastError;

  // Integral over the previous INTEGRAL_WINDOW errors (current one excluded).
  // errorHistory is a ring buffer: errorHistoryIndex points to the oldest
  // value, which is overwritten by the newest one.
  int integral = 0;
  for (uint8_t i = 0; i < INTEGRAL_WINDOW; i++) {
    integral += errorHistory[i];
  }

  errorHistory[errorHistoryIndex] = error;
  errorHistoryIndex = (errorHistoryIndex + 1) % INTEGRAL_WINDOW;
  lastError = error;

  int correction = static_cast<int>(KP * error + KI * integral + KD * derivative);
  correction = constrain(correction, -BASE_SPEED, BASE_SPEED);

  int leftSpeed = BASE_SPEED;
  int rightSpeed = BASE_SPEED;

  // correction < 0: line on the left  -> slow the left wheel  -> turn left.
  // correction > 0: line on the right -> slow the right wheel -> turn right.
  if (correction < 0) {
    leftSpeed += correction;
  } else {
    rightSpeed -= correction;
  }

  setMotorSpeeds(applyMinimumSpeed(leftSpeed), applyMinimumSpeed(rightSpeed));
}

// Line lost: spin in place toward the side where it was last seen.
void recoverLine() {
  if (linePosition <= LINE_LOST_LOW) {
    setMotorSpeeds(-BRAKE_REVERSE_SPEED, BRAKE_FORWARD_SPEED);  // turn left
  } else {
    setMotorSpeeds(BRAKE_FORWARD_SPEED, -BRAKE_REVERSE_SPEED);  // turn right
  }
}

int applyMinimumSpeed(int speed) {
  if (speed > 0 && speed < MIN_SPEED) return MIN_SPEED;
  if (speed < 0 && speed > -MIN_SPEED) return -MIN_SPEED;
  return speed;
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  driveMotor(PIN_LEFT_IN1, PIN_LEFT_IN2, PIN_PWM_LEFT, INVERT_LEFT_MOTOR, leftSpeed);
  driveMotor(PIN_RIGHT_IN1, PIN_RIGHT_IN2, PIN_PWM_RIGHT, INVERT_RIGHT_MOTOR, rightSpeed);
}

// speed > 0 = forward, speed < 0 = backward, from the robot's point of view.
// The inversion is applied here, so the rest of the code never needs to
// know how each motor is wired.
void driveMotor(uint8_t in1Pin, uint8_t in2Pin, uint8_t pwmPin, bool inverted, int speed) {
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);

  if (inverted) {
    speed = -speed;
  }

  if (speed >= 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    speed = -speed;
  }

  analogWrite(pwmPin, speed);
}

void stopMotors() {
  analogWrite(PIN_PWM_LEFT, 0);
  analogWrite(PIN_PWM_RIGHT, 0);

  digitalWrite(PIN_LEFT_IN1, LOW);
  digitalWrite(PIN_LEFT_IN2, LOW);
  digitalWrite(PIN_RIGHT_IN1, LOW);
  digitalWrite(PIN_RIGHT_IN2, LOW);
}
