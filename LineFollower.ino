/*
  ============================================================================
  4WD LINE FOLLOWING ROBOT — Arduino Uno R3
  ============================================================================
  Hardware:
    - Arduino Uno R3
    - 2-Channel Motor Driver (L298N-type: ENA, IN1, IN2, IN3, IN4, ENB)
    - 4x DC gear motors (wired as 2 logical channels: LEFT pair / RIGHT pair)
    - 2x INDIVIDUAL IR line sensor modules (3-pin digital comparator type,
      each with its own onboard sensitivity trim-pot + status LED),
      mounted side-by-side under the front bumper, roughly track-width apart
    - 3S 18650 Li-ion pack -> motor driver VMS; common GND with Arduino

  Control strategy (2-sensor logic):
    With only two sensors there is no meaningful "weighted position" the
    way there would be with a 5-sensor bar, so this uses the standard,
    battle-tested 2-sensor line-follower decision table instead of PID:

      LEFT sees line | RIGHT sees line | Meaning                | Action
      ---------------|-----------------|------------------------|------------------
      YES            | YES             | line centered/crossing | drive straight
      YES            | NO              | line drifted left      | steer left
      NO             | YES             | line drifted right     | steer right
      NO             | NO              | line lost              | bounded search

    "Steer" is done by differential motor speed (torque steering), not by
    a physical steering mechanism. Steering strength is fixed and tunable
    (STEER_DIFFERENTIAL) rather than proportional, since a 2-sensor system
    has no way to measure HOW far off-center the line is — only which
    side, if any, currently sees it.

    If the line is completely lost (both sensors read "no line"), the
    robot does NOT guess randomly. It pivots toward whichever side the
    line was last seen drifting to, for a bounded search time, then stops
    if the line still isn't found. This is a safety-first choice: an
    unbounded "spin and hope" search can walk the robot off a table.

  NOTE ON RELIABILITY WITH THIS HARDWARE:
    A 2-sensor setup is intentionally simpler and less smooth than a
    5-sensor array, particularly on sharp curves or the loop section of
    your track — it corrects in fixed steps rather than proportionally.
    It is fully workable, but if you find the robot overshooting on
    curves, the easiest hardware upgrade is adding one more identical
    sensor module in the center (see the commented THREE-SENSOR notes
    below computeAction() for how small the code change would be).

  IMPORTANT — CALIBRATE BEFORE RUNNING:
    1. Set LINE_IS_LOW below to match your sensor modules' actual behavior.
       Most comparator IR modules pull the output LOW when they see a dark
       line on a light background — but this varies by board, and each of
       your two modules also has its OWN onboard trim-pot that must be
       physically adjusted (turn until the LED just switches state at the
       line's edge) before the digital output is trustworthy at all.
       Test with Serial output (see DEBUG_SENSORS) to confirm polarity.
    2. Tune BASE_SPEED / STEER_DIFFERENTIAL for your motors and surface.
  ============================================================================
*/

#include <Arduino.h>

// ---------------------------------------------------------------------------
// PIN DEFINITIONS
// ---------------------------------------------------------------------------

// Motor driver control pins (matches wiring table in the write-up)
const uint8_t PIN_ENA = 9;   // Left pair speed (PWM)
const uint8_t PIN_IN1 = 8;   // Left pair direction bit 1
const uint8_t PIN_IN2 = 7;   // Left pair direction bit 2
const uint8_t PIN_IN3 = 5;   // Right pair direction bit 1
const uint8_t PIN_IN4 = 4;   // Right pair direction bit 2
const uint8_t PIN_ENB = 10;  // Right pair speed (PWM)

// IR sensor pins — one individual module on the left, one on the right.
const uint8_t PIN_SENSOR_LEFT  = A0;
const uint8_t PIN_SENSOR_RIGHT = A1;

// ---------------------------------------------------------------------------
// CALIBRATION CONSTANTS — tune these to your hardware
// ---------------------------------------------------------------------------

// Set to true if your sensor module outputs LOW when it sees the dark line.
// Set to false if it outputs HIGH when it sees the dark line.
// (Uncertain? Set DEBUG_SENSORS true, open Serial Monitor, and watch the
//  raw values while you slide the robot's sensor bar over the black line.)
const bool LINE_IS_LOW = true;

// Speed limits. PWM range is hard-clamped to [0,255] everywhere motor
// speed is set, so a bad calculation can never exceed motor driver limits.
const int BASE_SPEED  = 130;  // Cruising speed on a straight line (0-255)
const int MAX_SPEED   = 200;  // Absolute ceiling for any single motor
const int MIN_SPEED   = 60;   // Below this, a "moving" motor tends to stall
const int SEARCH_SPEED = 100; // Speed used while searching for a lost line

// How hard to steer when only one sensor sees the line. This is a fixed
// step (not proportional — a 2-sensor system has no way to measure HOW
// far off-center the line is, only which side sees it). Start small and
// increase if the robot doesn't correct fast enough on curves; decrease
// if it overshoots/zig-zags on straight sections.
const int STEER_DIFFERENTIAL = 60;

// If the line is lost, how long (ms) we allow the bounded search turn
// before giving up and stopping, as a safety fallback.
const unsigned long LOST_LINE_SEARCH_TIMEOUT_MS = 700;

// Set true to print raw sensor + error + PID data to Serial for debugging.
const bool DEBUG_SENSORS = false;

// ---------------------------------------------------------------------------
// INTERNAL STATE
// ---------------------------------------------------------------------------

// Remembers which side the line was last confidently seen on, so the
// lost-line search turns the correct direction instead of guessing.
// +1 = line was last seen toward the right, -1 = toward the left.
int lastKnownDirection = 1;

bool leftSees  = false; // true = left sensor currently detects the line
bool rightSees = false; // true = right sensor currently detects the line

// ---------------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------------

void setup() {
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENB, OUTPUT);

  pinMode(PIN_SENSOR_LEFT, INPUT);
  pinMode(PIN_SENSOR_RIGHT, INPUT);

  stopMotors(); // Always start in a known, stopped state.

  if (DEBUG_SENSORS) {
    Serial.begin(9600);
  }
}

// ---------------------------------------------------------------------------
// MAIN LOOP
// ---------------------------------------------------------------------------

void loop() {
  readSensors();

  if (!leftSees && !rightSees) {
    // Neither sensor sees the line -> run bounded recovery search.
    handleLineLost();
    return;
  }

  // A sensor currently seeing the line is also, by definition, an
  // up-to-date "last known direction" if it disagrees with the other.
  if (leftSees && !rightSees) {
    lastKnownDirection = -1; // line is toward the left
  } else if (rightSees && !leftSees) {
    lastKnownDirection = 1;  // line is toward the right
  }
  // If both see it, lastKnownDirection is left unchanged deliberately —
  // "centered" carries no new directional information for a future search.

  int leftSpeed, rightSpeed;
  computeMotorSpeeds(leftSpeed, rightSpeed);
  driveMotors(leftSpeed, rightSpeed);

  if (DEBUG_SENSORS) {
    Serial.print(F("L=")); Serial.print(leftSees);
    Serial.print(F(" R=")); Serial.print(rightSees);
    Serial.print(F(" -> speedL=")); Serial.print(leftSpeed);
    Serial.print(F(" speedR=")); Serial.println(rightSpeed);
  }
}

// ---------------------------------------------------------------------------
// SENSOR HANDLING
// ---------------------------------------------------------------------------

// Reads both sensors into leftSees/rightSees, normalized so that
// "true" always means "line detected here", regardless of whether the
// hardware is active-high or active-low (set by LINE_IS_LOW above).
void readSensors() {
  int rawLeft  = digitalRead(PIN_SENSOR_LEFT);
  int rawRight = digitalRead(PIN_SENSOR_RIGHT);

  leftSees  = LINE_IS_LOW ? (rawLeft  == LOW) : (rawLeft  == HIGH);
  rightSees = LINE_IS_LOW ? (rawRight == LOW) : (rawRight == HIGH);
}

// ---------------------------------------------------------------------------
// DECISION TABLE (replaces PID for a 2-sensor system)
// ---------------------------------------------------------------------------

// Fills leftSpeed/rightSpeed according to the 2-sensor decision table
// documented at the top of this file. Using out-parameters (rather than
// a struct/return) keeps this readable as plain, easy-to-audit branches —
// deliberately simple since this logic cannot be tested on real hardware
// before deployment.
void computeMotorSpeeds(int &leftSpeed, int &rightSpeed) {
  if (leftSees && rightSees) {
    // Line centered under both sensors (or a wide crossing/junction,
    // such as the loop on this track) -> drive straight.
    leftSpeed  = BASE_SPEED;
    rightSpeed = BASE_SPEED;
  } else if (leftSees && !rightSees) {
    // Line has drifted left -> steer left: slow the left side down,
    // keep the right side at full pace, so the robot arcs back onto it.
    leftSpeed  = BASE_SPEED - STEER_DIFFERENTIAL;
    rightSpeed = BASE_SPEED;
  } else { // (!leftSees && rightSees)
    // Line has drifted right -> steer right, mirror of the above.
    leftSpeed  = BASE_SPEED;
    rightSpeed = BASE_SPEED - STEER_DIFFERENTIAL;
  }

  // NOTE — upgrading to 3 sensors later:
  // If you add a center sensor, this whole function becomes a small
  // lookup over 3 booleans instead of 2 (still no PID needed): both-outer
  // + center all seeing line = straight; center-only = straight; one
  // outer sensor alone = steer that direction; all-off = line lost
  // (same handleLineLost() below still applies unchanged).
}

// ---------------------------------------------------------------------------
// LOST-LINE RECOVERY
// ---------------------------------------------------------------------------

void handleLineLost() {
  static unsigned long searchStartTime = 0;
  static bool searching = false;

  if (!searching) {
    searching = true;
    searchStartTime = millis();
  }

  if (millis() - searchStartTime > LOST_LINE_SEARCH_TIMEOUT_MS) {
    // Safety fallback: give up rather than spinning indefinitely.
    stopMotors();
    if (DEBUG_SENSORS) Serial.println(F("Line lost: search timed out, stopping."));
    return;
  }

  // Pivot in place toward the side the line was last seen drifting to.
  if (lastKnownDirection > 0) {
    driveMotors(SEARCH_SPEED, -SEARCH_SPEED);   // pivot right
  } else {
    driveMotors(-SEARCH_SPEED, SEARCH_SPEED);   // pivot left
  }

  // Reset the "searching" flag the instant either sensor reacquires the
  // line, rather than waiting a full extra loop() cycle to notice.
  readSensors();
  if (leftSees || rightSees) {
    searching = false;
  }
}

// ---------------------------------------------------------------------------
// MOTOR CONTROL
// ---------------------------------------------------------------------------

// Accepts signed speed values (negative = reverse) for each side
// independently, enabling both differential steering and in-place pivots.
// All PWM values are hard-clamped to safe driver limits before being
// written, so no calling code path can ever exceed the motor driver's
// rated PWM range.
void driveMotors(float leftSpeed, float rightSpeed) {
  setMotor(PIN_IN1, PIN_IN2, PIN_ENA, leftSpeed);
  setMotor(PIN_IN3, PIN_IN4, PIN_ENB, rightSpeed);
}

void setMotor(uint8_t pinForward, uint8_t pinReverse, uint8_t pinEnable, float speed) {
  bool forward = speed >= 0;
  int magnitude = (int)fabs(speed);

  // Enforce absolute PWM safety bounds regardless of upstream math.
  magnitude = constrain(magnitude, 0, MAX_SPEED);

  // Avoid the "buzzing but not turning" dead zone near zero PWM, while
  // still allowing a true, deliberate stop (magnitude == 0).
  if (magnitude > 0 && magnitude < MIN_SPEED) {
    magnitude = MIN_SPEED;
  }
  magnitude = constrain(magnitude, 0, 255); // Final hardware-level clamp.

  digitalWrite(pinForward, forward ? HIGH : LOW);
  digitalWrite(pinReverse, forward ? LOW : HIGH);
  analogWrite(pinEnable, magnitude);
}

void stopMotors() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
  analogWrite(PIN_ENA, 0);
  analogWrite(PIN_ENB, 0);
}
