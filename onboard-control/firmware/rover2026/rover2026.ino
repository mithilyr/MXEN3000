// ============================================================
// MXEN3000 S2 2026 - wireless line-following rover
// ============================================================
// One firmware for all four competition runs: Clockwise, Anticlockwise,
// Squiggle and Reverse. The GUI picks the mode and starts/stops the run
// over an XBee link (see protocol.h). All tuning is in config.h.
//
// Track: white line on a black table. Two analog IR sensors sit at the
// front, both over the line when the rover is centred. The difference
// between them gives the position error. Each mode steers with one of three
// controllers (PID, bang-bang or the lecture 8 adaptive one), chosen in
// config.h or from the GUI (key T, also while running). If both sensors see black, the
// rover spins towards the side it last saw the line on, and keeps searching
// until it finds the line again or the GUI sends STOP.
//
// Start / stop: motors sit at neutral until START. Once started, the rover keeps
// running through radio dropouts and lost lines until it gets a STOP command
// (sent only when the operator presses Space in the GUI). Nothing on the rover
// stops a run by itself. A brown-out reset mid-run does not stop it either:
// the run state survives in .noinit RAM and setup() carries the run on.

#include "config.h"
#include "protocol.h"

#if LINK_USE_SOFTWARE_SERIAL
#include <SoftwareSerial.h>
SoftwareSerial softLink(SOFT_SERIAL_RX_PIN, SOFT_SERIAL_TX_PIN);
Stream& xbee = softLink;
#else
Stream& xbee = Serial;
#endif

enum SensorIndex : byte { SENSOR_LEFT = 0, SENSOR_RIGHT = 1, SENSOR_COUNT = 2 };
const byte SENSOR_PINS[SENSOR_COUNT] = {IR_LEFT_PIN, IR_RIGHT_PIN};

// ---- Run state ----
Mode mode = MODE_CLOCKWISE;
ModeTuning liveTuning[MODE_COUNT];  // RAM copy of MODE_TUNING, edited live from the GUI
int baseSpeed = MODE_TUNING[MODE_CLOCKWISE].baseSpeed;
bool running = false;
bool lineLost = false;
bool calibrating = false;

// ---- Sensor state ----
int rawLevel[SENSOR_COUNT] = {0, 0};
int whiteLevel[SENSOR_COUNT] = {DEFAULT_WHITE_LEVEL, DEFAULT_WHITE_LEVEL};
int blackLevel[SENSOR_COUNT] = {DEFAULT_BLACK_LEVEL, DEFAULT_BLACK_LEVEL};
int calibrationMin[SENSOR_COUNT];
int calibrationMax[SENSOR_COUNT];
float normLevel[SENSOR_COUNT] = {0.0f, 0.0f};  // 0 = white, 1 = black
bool sensorWhite[SENSOR_COUNT] = {false, false};  // Steady 1/0: true = on the white line

// position > 0: right sensor is on black, so the line has moved to the left.
float position = 0.0f;
float lastPosition = 0.0f;
int lastTurn = TURN_CLOCKWISE;
// Bang-bang: the sensor that was last on white while the other was black. With
// both on black the rover turns towards this side.
byte lastWhiteSide = SENSOR_RIGHT;
bool bangRealign = false;  // Back on white after both-black: keep turning until aligned

// ---- PID state ----
float integral = 0.0f;                      // Sum of position x seconds
float filteredDerivative = 0.0f;            // Position change per second, low-passed

// ---- Adaptive follower state (lecture 8 variables A, B and C) ----
bool correcting = false;                    // A sensor is on the line right now
unsigned long lastCorrectionEndMs = 0;      // A: when the last correction ended
int lastCorrectionDir = TURN_CLOCKWISE;     // B: direction of the last correction
float correctionRate = 0.0f;                // C: turn rate of the last correction, %

// ---- Outputs ----
byte dacLeft = DAC_NEUTRAL;
byte dacRight = DAC_NEUTRAL;
int8_t lastLeftPercent = 0;                 // Last wheel commands, -100..100 %
int8_t lastRightPercent = 0;

// ---- Backtrack history: ring buffer of recent wheel commands ----
int8_t historyLeft[BACKTRACK_SAMPLES];
int8_t historyRight[BACKTRACK_SAMPLES];
byte historyHead = 0;                       // Next slot to write
byte historyCount = 0;
unsigned long lastHistoryMs = 0;
unsigned long nextBacktrackMs = 0;

// ---- Corner search: line lost straight from both sensors on white ----
bool lineWasCentred = false;                // Both sensors were on white last step
unsigned long lastSideMs = 0;               // Last time one sensor alone saw the line
bool cornerSearch = false;                  // This loss is a corner: sweep, no backtrack
unsigned long searchStartMs = 0;
int searchDir = TURN_CLOCKWISE;             // Direction of the latest search spin
unsigned long cornerFoundMs = 0;            // When a corner search last found the line

// ---- Debug telemetry (see protocol.h MSG_DEBUG) ----
byte controlState = STATE_IDLE;
float pTerm = 0.0f;                         // PID terms of the last step, turn %
float iTerm = 0.0f;
float dTerm = 0.0f;
bool saturatedNow = false;                  // Steering clamped at maxTurn this step
byte lostEvents = 0;                        // Times the line was lost this run
byte calibrationResult = CAL_DEFAULTS;
byte debugSeq = 0;
// Since the last debug frame:
unsigned int windowSteps = 0;
unsigned int windowSaturated = 0;
float windowMaxError = 0.0f;                // Largest |position|
unsigned long windowStepMaxUs = 0;          // Longest control step
unsigned long windowPeriodMaxUs = 0;        // Longest time between control steps
unsigned long lastStepUs = 0;

// ---- Run state kept through a reset ----
// A brown-out (motors pulling the supply down) resets the Nano mid-run. The C
// startup code leaves .noinit RAM alone, so this copy of the run state, taken
// every control step, survives the reset and setup() carries the run on with
// no PC involved. A power-on fills RAM with noise, so the check fails and the
// rover starts stopped as usual. buildId stops a fresh upload resuming a run.
struct ResumeState {
  uint32_t buildId;
  Mode mode;
  bool running;
  int baseSpeed;
  int lastTurn;
  int8_t lastLeftPercent;       // Wheel commands at the reset, so a turn carries on
  int8_t lastRightPercent;
  byte lastWhiteSide;
  bool bangRealign;
  int whiteLevel[SENSOR_COUNT];
  int blackLevel[SENSOR_COUNT];
  byte calibrationResult;
  ModeTuning tuning[MODE_COUNT];
  uint16_t check;               // resumeCheck() of everything above
};
ResumeState resumeState __attribute__((section(".noinit")));
uint32_t thisBuild = 0;
bool resumedAfterReset = false;  // This run was carried on through a reset

// ---- Timers ----
unsigned long runStartMs = 0;               // For the soft start
unsigned long lastControlMs = 0;
unsigned long lastTelemetryMs = 0;
unsigned long lastTuningMs = 0;
bool tuningDirty = false;  // A tuning command arrived: echo the values at once
unsigned long calibrationStartMs = 0;

// ============================================================
// Motor output
// ============================================================
void writeDac(const byte pins[], byte value) {
  for (byte bit = 0; bit < DAC_BITS; bit++) {
    digitalWrite(pins[bit], bitRead(value, bit) ? HIGH : LOW);
  }
}

byte speedToDac(int speedPercent, int motorDir) {
  speedPercent = constrain(speedPercent, -SPEED_MAX, SPEED_MAX);
  return DAC_NEUTRAL + (long)motorDir * speedPercent * DAC_SPAN / SPEED_MAX;
}

// Writes the last wheel commands to the DACs, scaled down during the soft start.
void writeWheels() {
  long scale = SPEED_MAX;
  unsigned long sinceStart = millis() - runStartMs;
  if (running && sinceStart < SOFT_START_MS) scale = SPEED_MAX * sinceStart / SOFT_START_MS;
  dacLeft = speedToDac(lastLeftPercent * scale / SPEED_MAX, LEFT_MOTOR_DIR);
  dacRight = speedToDac(lastRightPercent * scale / SPEED_MAX, RIGHT_MOTOR_DIR);
  writeDac(DAC_LEFT_PINS, dacLeft);
  writeDac(DAC_RIGHT_PINS, dacRight);
}

// Speeds are -100..100 % of full speed, positive = forwards for that wheel.
void setWheelSpeeds(int leftPercent, int rightPercent) {
  lastLeftPercent = constrain(leftPercent, -SPEED_MAX, SPEED_MAX);
  lastRightPercent = constrain(rightPercent, -SPEED_MAX, SPEED_MAX);
  writeWheels();
}

void holdNeutral() {
  setWheelSpeeds(0, 0);
}

// ============================================================
// Sensors
// ============================================================
int readSensor(byte pin) {
  long total = 0;
  for (byte i = 0; i < SENSOR_SAMPLES; i++) {
    total += analogRead(pin);
  }
  int average = total / SENSOR_SAMPLES;
  return WHITE_READS_HIGH ? ADC_MAX - average : average;
}

float normalise(int raw, int white, int black) {
  float level = (float)(raw - white) / (black - white);
  return constrain(level, 0.0f, 1.0f);
}

void updateSensors() {
  for (byte s = 0; s < SENSOR_COUNT; s++) {
    rawLevel[s] = readSensor(SENSOR_PINS[s]);
    normLevel[s] = normalise(rawLevel[s], whiteLevel[s], blackLevel[s]);
    if (normLevel[s] < WHITE_BELOW) sensorWhite[s] = true;
    else if (normLevel[s] > BLACK_ABOVE) sensorWhite[s] = false;
  }
  lastPosition = position;
  position = normLevel[SENSOR_RIGHT] - normLevel[SENSOR_LEFT];
}

// ============================================================
// Calibration: sweep the rover across the line by hand while it runs.
// Assumes white reads lower than black (true for last year's sensors).
// ============================================================
void startCalibration() {
  if (running) return;
  holdNeutral();
  calibrating = true;
  calibrationStartMs = millis();
  for (byte s = 0; s < SENSOR_COUNT; s++) {
    calibrationMin[s] = rawLevel[s];
    calibrationMax[s] = rawLevel[s];
  }
}

void updateCalibration(unsigned long now) {
  for (byte s = 0; s < SENSOR_COUNT; s++) {
    calibrationMin[s] = min(calibrationMin[s], rawLevel[s]);
    calibrationMax[s] = max(calibrationMax[s], rawLevel[s]);
  }
  if (now - calibrationStartMs < CALIBRATION_MS) return;

  calibrating = false;
  tuningDirty = true;               // Sends the calibration frame at once
  for (byte s = 0; s < SENSOR_COUNT; s++) {
    if (calibrationMax[s] - calibrationMin[s] < MIN_CALIBRATION_SPAN) {
      calibrationResult = CAL_REJECTED;
      return;
    }
  }
  for (byte s = 0; s < SENSOR_COUNT; s++) {
    whiteLevel[s] = calibrationMin[s];
    blackLevel[s] = calibrationMax[s];
  }
  calibrationResult = CAL_ACCEPTED;
}

// ============================================================
// Run control
// ============================================================
void clearHistory() {
  historyHead = 0;
  historyCount = 0;
}

// The GUI resends START until telemetry shows the rover running, so a START
// that arrives mid-run is ignored instead of restarting the run.
void startRun() {
  if (calibrating || running) return;
  running = true;
  runStartMs = millis();
  clearHistory();
  lineLost = false;
  lineWasCentred = false;
  lastTurn = liveTuning[mode].defaultTurn;
  // Until one sensor has seen white alone, both-black turns the default way.
  lastWhiteSide = lastTurn == TURN_CLOCKWISE ? SENSOR_RIGHT : SENSOR_LEFT;
  bangRealign = false;
  lastPosition = position;
  integral = 0.0f;
  filteredDerivative = 0.0f;
  correcting = false;
  lastCorrectionEndMs = millis();
  lastCorrectionDir = lastTurn;
  correctionRate = 0.0f;
  lostEvents = 0;
}

void stopRun() {
  running = false;
  resumedAfterReset = false;
  holdNeutral();
}

// ============================================================
// Carrying a run on through a reset (see ResumeState)
// ============================================================
uint32_t buildId() {
  const char* text = __DATE__ __TIME__;
  uint32_t hash = 2166136261UL;             // FNV-1a
  while (*text) hash = (hash ^ (byte)*text++) * 16777619UL;
  return hash;
}

// Fletcher-style sums that just wrap (no modulo: a divide per byte would cost
// several ms every control step on the AVR). Random power-on RAM still fails it.
uint16_t resumeCheck() {
  const byte* bytes = (const byte*)&resumeState;
  uint16_t sum1 = 0, sum2 = 0;
  for (size_t i = 0; i < offsetof(ResumeState, check); i++) {
    sum1 += bytes[i];
    sum2 += sum1;
  }
  return sum2 ^ (sum1 << 8);
}

void saveResumeState() {
  resumeState.buildId = thisBuild;
  resumeState.mode = mode;
  resumeState.running = running;
  resumeState.baseSpeed = baseSpeed;
  resumeState.lastTurn = lastTurn;
  resumeState.lastLeftPercent = lastLeftPercent;
  resumeState.lastRightPercent = lastRightPercent;
  resumeState.lastWhiteSide = lastWhiteSide;
  resumeState.bangRealign = bangRealign;
  for (byte s = 0; s < SENSOR_COUNT; s++) {
    resumeState.whiteLevel[s] = whiteLevel[s];
    resumeState.blackLevel[s] = blackLevel[s];
  }
  resumeState.calibrationResult = calibrationResult;
  memcpy(resumeState.tuning, liveTuning, sizeof(liveTuning));
  resumeState.check = resumeCheck();
}

// Called once in setup(), after liveTuning holds the config.h defaults.
void resumeAfterReset() {
  if (resumeState.buildId != thisBuild || resumeState.check != resumeCheck() ||
      resumeState.mode >= MODE_COUNT) {
    return;                                 // Power-on or new upload: start stopped
  }
  mode = resumeState.mode;
  baseSpeed = resumeState.baseSpeed;
  for (byte s = 0; s < SENSOR_COUNT; s++) {
    whiteLevel[s] = resumeState.whiteLevel[s];
    blackLevel[s] = resumeState.blackLevel[s];
  }
  calibrationResult = resumeState.calibrationResult;
  memcpy(liveTuning, resumeState.tuning, sizeof(liveTuning));
  if (resumeState.running) {
    startRun();
    lastTurn = resumeState.lastTurn;
    lastWhiteSide = resumeState.lastWhiteSide == SENSOR_LEFT ? SENSOR_LEFT : SENSOR_RIGHT;
    bangRealign = resumeState.bangRealign;
    setWheelSpeeds(resumeState.lastLeftPercent, resumeState.lastRightPercent);  // Soft-started
    resumedAfterReset = true;
  }
}

void setMode(byte newMode) {
  if (running || newMode >= MODE_COUNT) return;
  mode = (Mode)newMode;
  baseSpeed = liveTuning[mode].baseSpeed;
}

// Allowed while running too, so the GUI's T key always works. The new
// controller starts from a clean state, not the old one's memory.
void setController(byte newController) {
  if (newController >= CONTROLLER_COUNT) return;
  liveTuning[mode].controller = newController;
  integral = 0.0f;
  filteredDerivative = 0.0f;
  correcting = false;
  lastCorrectionEndMs = millis();
  correctionRate = 0.0f;
}

// ============================================================
// Line follower (spin search when lost, then one of three controllers)
// ============================================================
// Corner search spin direction: lastTurn for SWEEP_MS, then back and forth
// across the heading the line was lost on, 2 x SWEEP_MS each way.
int sweepDirection(unsigned long elapsedMs) {
  if (elapsedMs < SWEEP_MS) return lastTurn;
  bool firstReturn = ((elapsedMs - SWEEP_MS) / (2 * SWEEP_MS)) % 2 == 0;
  return firstReturn ? -lastTurn : lastTurn;
}

// While lineMissing, first retraces the last wheel commands in reverse, then
// spins towards the side the line was last seen on, until the line is found or
// the run is stopped. Returns true when it is handling a lost line, so the
// caller must not steer.
// Exception, a sharp corner: both sensors go from white to black at once, or
// one goes black only a moment (CORNER_WINDOW_MS) before the other.
// Backtracking would only find the line before the corner and drive into it
// again, so the rover stays put and sweeps, starting towards lastTurn (the
// side whose sensor stayed white longest, when there is one).
bool searchIfLineLost(bool lineMissing, unsigned long now, const ModeTuning& tuning) {
  if (!lineMissing) {
    if (lineLost && cornerSearch) cornerFoundMs = now;
    lineLost = false;
    return false;
  }
  if (!lineLost) {
    lineLost = true;
    if (lostEvents < DATA_MAX) lostEvents++;
    nextBacktrackMs = now;
    searchStartMs = now;
    cornerSearch = lineWasCentred || now - lastSideMs < CORNER_WINDOW_MS;
    if (cornerSearch) clearHistory();
  }
  if (historyCount > 0) {
    // Backtrack: undo the newest recorded command, one sample at a time.
    controlState = STATE_BACKTRACK;
    if (now >= nextBacktrackMs) {
      nextBacktrackMs = now + BACKTRACK_SAMPLE_MS;
      historyHead = (historyHead + BACKTRACK_SAMPLES - 1) % BACKTRACK_SAMPLES;
      historyCount--;
      setWheelSpeeds(-historyLeft[historyHead], -historyRight[historyHead]);
    }
    return true;
  }
  controlState = cornerSearch ? STATE_CORNER_SWEEP : STATE_SPIN;
  searchDir = cornerSearch ? sweepDirection(now - searchStartMs) : lastTurn;
  int spin = searchDir * tuning.searchSpeed;
  setWheelSpeeds(spin, -spin);
  return true;
}

// Starts a new correction towards dir (lecture 8). The turn rate is
// ADAPT_GAIN / (time since the last correction). Soon after a correction on
// the same side, the earlier rate is added, so the turn gets sharper.
void startCorrection(int dir, unsigned long now, const ModeTuning& tuning) {
  float gapSeconds = max((now - lastCorrectionEndMs) / 1000.0f, MIN_CORRECTION_GAP_S);
  float rate = tuning.adaptGain / gapSeconds;
  if (dir == lastCorrectionDir && gapSeconds < CORRECTION_MEMORY_S) {
    rate += correctionRate;
  }
  correctionRate = constrain(rate, MIN_CORRECTION_TURN, tuning.maxTurn);
  lastCorrectionDir = dir;
  correcting = true;
}

// Adaptive (lecture 8): each sensor is either on the white line or off it.
void updateAdaptive(unsigned long now) {
  const ModeTuning& tuning = liveTuning[mode];

  bool leftOnLine = sensorWhite[SENSOR_LEFT];
  bool rightOnLine = sensorWhite[SENSOR_RIGHT];
  bool bothOff = !leftOnLine && !rightOnLine;

  if (searchIfLineLost(bothOff && !SENSORS_STRADDLE_LINE, now, tuning)) return;

  int forward = tuning.travelDir * baseSpeed;
  if (leftOnLine == rightOnLine) {
    // Both on or both off: go straight, and remember when the correction ended.
    if (correcting) {
      correcting = false;
      lastCorrectionEndMs = now;
    }
    controlState = STATE_FOLLOW;
    setWheelSpeeds(forward, forward);
    return;
  }

  // The line is under one sensor only, so steer towards that side.
  int dir = leftOnLine ? TURN_ANTICLOCKWISE : TURN_CLOCKWISE;
  lastTurn = dir;
  if (!correcting || dir != lastCorrectionDir) {
    startCorrection(dir, now, tuning);
  } else {
    const float periodSeconds = CONTROL_PERIOD_MS / 1000.0f;
    correctionRate += tuning.adaptRamp * periodSeconds;
    correctionRate = min(correctionRate, (float)tuning.maxTurn);
  }
  controlState = STATE_ADAPT_TURN;
  saturatedNow = correctionRate >= tuning.maxTurn;

  // turn > 0 rotates the rover clockwise.
  float turn = tuning.steerSign * dir * correctionRate;
  setWheelSpeeds(forward + turn, forward - turn);
}

// Shared by PID and bang-bang. Handles a lost line (both sensors on black)
// and remembers which side the line was last seen on. Returns true when the
// lost-line search has taken over, so the caller must not steer.
bool analogLineLost(unsigned long now, const ModeTuning& tuning) {
  bool bothOnBlack = normLevel[SENSOR_LEFT] > LOST_LEVEL &&
                     normLevel[SENSOR_RIGHT] > LOST_LEVEL;
  if (searchIfLineLost(bothOnBlack, now, tuning)) return true;

  lineWasCentred = normLevel[SENSOR_LEFT] < WHITE_BELOW && normLevel[SENSOR_RIGHT] < WHITE_BELOW;
  if (position > SIDE_MEMORY_LEVEL) {
    lastTurn = TURN_ANTICLOCKWISE;
    lastSideMs = now;
  } else if (position < -SIDE_MEMORY_LEVEL) {
    lastTurn = TURN_CLOCKWISE;
    lastSideMs = now;
  }
  return false;
}

// turn > 0 rotates the rover clockwise. Right (clockwise) turns are scaled,
// limit included, by RIGHT_TURN_GAIN to even out a rover that turns right weaker.
void driveWithTurn(const ModeTuning& tuning, float turn) {
  const float rightGain = RIGHT_TURN_GAIN[mode];
  if (turn > 0.0f) turn *= rightGain;
  float wanted = turn;
  turn = constrain(turn, -tuning.maxTurn, tuning.maxTurn * rightGain);
  saturatedNow = turn != wanted;
  int forward = tuning.travelDir * baseSpeed;
  setWheelSpeeds(forward + turn, forward - turn);
}

// Turns a position reading into the rover's real sideways offset from the line
// centre, using the sensor geometry in config.h, scaled so +-1 = SENSOR_RANGE_CM.
// position climbs twice as fast while both sensors see the line (inner range)
// as once one is fully white (outer range); this straightens that out.
float lineOffset(float pos) {
  const float innerEnd = 1.0f - 2.0f * SENSOR_GAP_CM / SENSOR_SPOT_CM;  // position where the inner range ends
  float size = fabs(pos);
  float cm = size <= innerEnd
                 ? size * SENSOR_SPOT_CM / 2.0f
                 : (SENSOR_SPOT_CM / 2.0f - SENSOR_GAP_CM) + (size - innerEnd) * SENSOR_SPOT_CM;
  return copysign(cm / SENSOR_RANGE_CM, pos);
}

// PID on the sideways offset, worked out from the two normalised readings.
void updatePid(unsigned long now) {
  const ModeTuning& tuning = liveTuning[mode];
  if (analogLineLost(now, tuning)) {
    integral = 0.0f;              // The error while searching means nothing
    filteredDerivative = 0.0f;
    return;
  }

  // Corner hold: just after a corner search finds the line, both sensors read
  // white (position 0), so plain PID would drive straight off the bend again
  // and the rover would stutter round it in small spins. Instead it keeps
  // arcing the way it was spinning until one sensor alone sees the line.
  bool bothWhite = normLevel[SENSOR_LEFT] < WHITE_BELOW && normLevel[SENSOR_RIGHT] < WHITE_BELOW;
  if (bothWhite && now - cornerFoundMs < CORNER_HOLD_MS[mode]) {
    controlState = STATE_CORNER_HOLD;
    driveWithTurn(tuning, searchDir * CORNER_HOLD_TURN * tuning.maxTurn);
    return;
  }
  controlState = STATE_FOLLOW;

  const float periodSeconds = CONTROL_PERIOD_MS / 1000.0f;
  float error = lineOffset(position);
  float derivative = (error - lineOffset(lastPosition)) / periodSeconds;
  filteredDerivative += D_FILTER_ALPHA * (derivative - filteredDerivative);

  // Anti-windup: the integral term on its own can never ask for more than
  // maxTurn, so it cannot keep growing while the steering is clamped.
  if (tuning.ki > 0.0f) {
    float integralLimit = tuning.maxTurn / tuning.ki;
    integral = constrain(integral + error * periodSeconds, -integralLimit, integralLimit);
  } else {
    integral = 0.0f;
  }

  // A positive error (line to the left) needs an anticlockwise
  // correction, hence the minus sign.
  pTerm = -tuning.steerSign * tuning.kp * error;
  iTerm = -tuning.steerSign * tuning.ki * integral;
  dTerm = -tuning.steerSign * tuning.kd * filteredDerivative;
  driveWithTurn(tuning, pTerm + iTerm + dTerm);
}

// Electrical PWM duty (100 = full forward, 50 = stopped, 0 = full reverse) to
// a wheel speed, -100..100 %.
int dutyToSpeed(int duty) {
  return (constrain(duty, 0, 100) - 50) * 2;
}

// Bang-bang: each wheel runs at its own PWM duty (pwmStraight*). The sensors
// sit over the two edges of the white line, so a sensor reading black means
// the rover has drifted towards that side and the line is on the other side.
// The wheel on the OPPOSITE side then drops to its pwmCorrect* duty, which
// steers back onto the line. Between WHITE_BELOW and BLACK_ABOVE a sensor
// counts as still on the line. steerSign = -1 swaps the sides if this steers
// the wrong way on test. With both sensors black the rover turns towards the
// sensor that last saw white (lastWhiteSide), exactly as if only the other
// sensor were black. When that turn brings both sensors back onto white it
// keeps turning (realign) until exactly one sensor sees black, i.e. the rover
// is lined up with an edge. Going straight there instead jumped the stopped
// wheel 0 -> full and shot the rover across a line it met at an angle.
void updateBangBang(unsigned long now) {
  const ModeTuning& tuning = liveTuning[mode];
  lineLost = false;

  bool leftBlack = normLevel[SENSOR_LEFT] > BLACK_ABOVE;
  bool rightBlack = normLevel[SENSOR_RIGHT] > BLACK_ABOVE;
  if (leftBlack != rightBlack) {
    lastWhiteSide = leftBlack ? SENSOR_RIGHT : SENSOR_LEFT;
    bangRealign = false;              // Aligned with an edge: normal bang-bang again
  }
  bool bothBlack = leftBlack && rightBlack;
  if (bothBlack) bangRealign = true;
  bool turnToLastWhite = bothBlack || bangRealign;
  if (turnToLastWhite) {
    // Turn towards the last white sensor: treat the other one as the black one.
    leftBlack = lastWhiteSide == SENSOR_RIGHT;
    rightBlack = !leftBlack;
  }
  if (tuning.steerSign < 0) {
    bool swap = leftBlack;
    leftBlack = rightBlack;
    rightBlack = swap;
  }
  if (bothBlack) {
    controlState = STATE_BANG_BOTH_BLACK;
  } else if (bangRealign) {
    controlState = STATE_BANG_REALIGN;
  } else if (rightBlack != leftBlack) {
    controlState = rightBlack ? STATE_BANG_LEFT : STATE_BANG_RIGHT;  // Slowed wheel's side
  } else {
    controlState = STATE_BANG_STRAIGHT;
  }
  setWheelSpeeds(dutyToSpeed(rightBlack ? tuning.pwmCorrectLeft : tuning.pwmStraightLeft),
                 dutyToSpeed(leftBlack ? tuning.pwmCorrectRight : tuning.pwmStraightRight));
}

// Remembers the wheel commands given while the line is being followed.
void recordHistory(unsigned long now) {
  if (now - lastHistoryMs < BACKTRACK_SAMPLE_MS) return;
  lastHistoryMs = now;
  historyLeft[historyHead] = lastLeftPercent;
  historyRight[historyHead] = lastRightPercent;
  historyHead = (historyHead + 1) % BACKTRACK_SAMPLES;
  if (historyCount < BACKTRACK_SAMPLES) historyCount++;
}

void updateLineFollower(unsigned long now) {
  switch (liveTuning[mode].controller) {
    case CONTROLLER_BANG_BANG:
      updateBangBang(now);
      break;
    case CONTROLLER_ADAPTIVE:
      updateAdaptive(now);
      break;
    default:
      updatePid(now);
      break;
  }
  if (running && !lineLost) recordHistory(now);
}

// ============================================================
// XBee link
// ============================================================
void handleCommand(byte command, byte data) {
  switch (command) {
    case CMD_PING:
      break;
    case CMD_START:
      startRun();
      break;
    case CMD_STOP:
      stopRun();
      break;
    case CMD_SET_MODE:
      setMode(data);
      break;
    case CMD_SET_SPEED:
      baseSpeed = min((int)data, SPEED_MAX);
      liveTuning[mode].baseSpeed = baseSpeed;  // Remembered when the mode is reselected
      break;
    case CMD_SET_ADAPT_GAIN:
      liveTuning[mode].adaptGain = data / GAIN_SCALE;
      break;
    case CMD_SET_ADAPT_RAMP:
      liveTuning[mode].adaptRamp = data;
      break;
    case CMD_SET_MAX_TURN:
      liveTuning[mode].maxTurn = min((int)data, SPEED_MAX);
      break;
    case CMD_SET_KP:
      liveTuning[mode].kp = data;
      break;
    case CMD_SET_KD:
      liveTuning[mode].kd = data / KD_SCALE;
      break;
    case CMD_SET_KI:
      liveTuning[mode].ki = data;
      break;
    case CMD_SET_BANG_LEVEL:
      liveTuning[mode].bangLevel = min(data / BANG_LEVEL_SCALE, 1.0f);
      break;
    case CMD_SET_CONTROLLER:
      setController(data);
      break;
    case CMD_RESET_INTEGRAL:
      integral = 0.0f;
      break;
    case CMD_CALIBRATE:
      startCalibration();
      break;
    case CMD_SET_PWM_STRAIGHT_LEFT:
      liveTuning[mode].pwmStraightLeft = min((int)data, 100);
      break;
    case CMD_SET_PWM_STRAIGHT_RIGHT:
      liveTuning[mode].pwmStraightRight = min((int)data, 100);
      break;
    case CMD_SET_PWM_CORRECT_LEFT:
      liveTuning[mode].pwmCorrectLeft = min((int)data, 100);
      break;
    case CMD_SET_PWM_CORRECT_RIGHT:
      liveTuning[mode].pwmCorrectRight = min((int)data, 100);
      break;
  }
}

// Collects [START][command][data][checksum]. Command and data bytes are
// never FRAME_START, so seeing one there means we joined mid-frame.
void pollLink() {
  static byte frame[COMMAND_FRAME_LENGTH];
  static byte received = 0;
  const byte checksumIndex = COMMAND_FRAME_LENGTH - 1;

  while (xbee.available() > 0) {
    byte value = xbee.read();
    if (value == FRAME_START && received < checksumIndex) {
      received = 0;
    }
    if (received == 0 && value != FRAME_START) continue;

    frame[received++] = value;
    if (received < COMMAND_FRAME_LENGTH) continue;

    received = 0;
    byte checksum = frame[0] + frame[1] + frame[2];
    if (checksum == frame[checksumIndex]) {
      handleCommand(frame[1], frame[2]);
      byte command = frame[1];
      if (command == CMD_SET_MODE || command == CMD_SET_SPEED ||
          (command >= CMD_SET_ADAPT_GAIN && command <= CMD_SET_CONTROLLER) ||
          (command >= CMD_SET_PWM_STRAIGHT_LEFT && command <= CMD_SET_PWM_CORRECT_RIGHT)) {
        tuningDirty = true;
      }
    }
  }
}

byte toDataByte(long value) {
  return constrain(value, 0, DATA_MAX);
}

// Fills in the last byte of the frame with the checksum and sends it.
void sendFrame(byte frame[], byte length) {
  byte checksum = 0;
  for (byte i = 0; i < length - 1; i++) {
    checksum += frame[i];
  }
  frame[length - 1] = checksum;
  xbee.write(frame, length);
}

void sendTelemetry() {
  const int ADC_TO_BYTE_SHIFT = 2;  // 10-bit ADC -> 8-bit
  byte status = mode << STATUS_MODE_SHIFT;
  if (running) status |= STATUS_RUNNING;
  if (lineLost) status |= STATUS_LINE_LOST;
  if (calibrating) status |= STATUS_CALIBRATING;

  byte frame[TELEMETRY_FRAME_LENGTH] = {
    FRAME_START,
    MSG_TELEMETRY,
    toDataByte(rawLevel[SENSOR_LEFT] >> ADC_TO_BYTE_SHIFT),
    toDataByte(rawLevel[SENSOR_RIGHT] >> ADC_TO_BYTE_SHIFT),
    toDataByte(dacLeft),
    toDataByte(dacRight),
    toDataByte(lround((position + 1.0f) * DATA_MAX / 2)),  // -1..1 -> 0..254
    toDataByte(baseSpeed),
    status,
    (byte)((sensorWhite[SENSOR_LEFT] ? SENSOR_LEFT_WHITE : 0) |
           (sensorWhite[SENSOR_RIGHT] ? SENSOR_RIGHT_WHITE : 0)),
    0
  };
  sendFrame(frame, TELEMETRY_FRAME_LENGTH);
}

byte toSignedByte(float value) {
  return toDataByte(lround(value) + SIGNED_OFFSET);
}

byte tenthsOfMs(unsigned long us) {
  return toDataByte((us + 50) / 100);
}

// The controller's insides (protocol.h MSG_DEBUG). Clears the per-window
// figures, so each frame covers the control steps since the one before.
void sendDebug() {
  byte saturatedPercent = windowSteps ? 100UL * windowSaturated / windowSteps : 0;
  byte frame[DEBUG_FRAME_LENGTH] = {
    FRAME_START,
    MSG_DEBUG,
    debugSeq,
    controlState,
    toSignedByte(pTerm),
    toSignedByte(iTerm),
    toSignedByte(dTerm),
    toSignedByte((lastLeftPercent - lastRightPercent) / 2.0f),  // Turn the wheels give, > 0 = clockwise
    toDataByte(lround(correctionRate)),
    saturatedPercent,
    toDataByte(lround(windowMaxError * 100)),
    toDataByte(windowSteps),
    tenthsOfMs(windowStepMaxUs),
    tenthsOfMs(windowPeriodMaxUs),
    lostEvents,
    (byte)((saturatedNow ? DEBUG_SATURATED : 0) | (resumedAfterReset ? DEBUG_RESUMED : 0)),
    0
  };
  sendFrame(frame, DEBUG_FRAME_LENGTH);
  debugSeq = (debugSeq + 1) % SEQ_COUNT;
  windowSteps = 0;
  windowSaturated = 0;
  windowMaxError = 0.0f;
  windowStepMaxUs = 0;
  windowPeriodMaxUs = 0;
}

// The white / black levels the rover normalises with, and how the last
// calibration went.
void sendCalibration() {
  const int ADC_TO_BYTE_SHIFT = 2;
  byte frame[CALIBRATION_FRAME_LENGTH] = {
    FRAME_START,
    MSG_CALIBRATION,
    toDataByte(whiteLevel[SENSOR_LEFT] >> ADC_TO_BYTE_SHIFT),
    toDataByte(whiteLevel[SENSOR_RIGHT] >> ADC_TO_BYTE_SHIFT),
    toDataByte(blackLevel[SENSOR_LEFT] >> ADC_TO_BYTE_SHIFT),
    toDataByte(blackLevel[SENSOR_RIGHT] >> ADC_TO_BYTE_SHIFT),
    calibrationResult,
    0
  };
  sendFrame(frame, CALIBRATION_FRAME_LENGTH);
}

// Reports the tuning values the rover is using, so the GUI shows what the
// rover really has, not what was last requested.
void sendTuning() {
  const ModeTuning& tuning = liveTuning[mode];
  byte frame[TUNING_FRAME_LENGTH] = {
    FRAME_START,
    MSG_TUNING,
    mode,
    tuning.controller,
    toDataByte(lround(tuning.adaptGain * GAIN_SCALE)),
    toDataByte(lround(tuning.adaptRamp)),
    toDataByte(tuning.maxTurn),
    toDataByte(lround(tuning.kp)),
    toDataByte(lround(tuning.kd * KD_SCALE)),
    toDataByte(lround(tuning.ki)),
    toDataByte(lround(tuning.bangLevel * BANG_LEVEL_SCALE)),
    toDataByte(tuning.pwmStraightLeft),
    toDataByte(tuning.pwmStraightRight),
    toDataByte(tuning.pwmCorrectLeft),
    toDataByte(tuning.pwmCorrectRight),
    0
  };
  sendFrame(frame, TUNING_FRAME_LENGTH);
}

// ============================================================
// Status LED: solid = running
// ============================================================
// (Unavailable when SoftwareSerial uses D13.)
void updateStatusLed(unsigned long now) {
  digitalWrite(STATUS_LED_PIN, running ? HIGH : LOW);
}

// ============================================================
// Setup and main loop
// ============================================================
void setup() {
  for (byte m = 0; m < MODE_COUNT; m++) {
    liveTuning[m] = MODE_TUNING[m];
  }
  for (byte bit = 0; bit < DAC_BITS; bit++) {
    pinMode(DAC_LEFT_PINS[bit], OUTPUT);
    pinMode(DAC_RIGHT_PINS[bit], OUTPUT);
  }
  holdNeutral();
  thisBuild = buildId();
  resumeAfterReset();

#if LINK_USE_SOFTWARE_SERIAL
  softLink.begin(LINK_BAUD);
#else
  Serial.begin(LINK_BAUD);
  pinMode(STATUS_LED_PIN, OUTPUT);
#endif
}

void loop() {
  unsigned long now = millis();
  pollLink();

  if (now - lastControlMs >= CONTROL_PERIOD_MS) {
    lastControlMs = now;
    unsigned long stepStartUs = micros();
    if (lastStepUs != 0) {
      windowPeriodMaxUs = max(windowPeriodMaxUs, stepStartUs - lastStepUs);
    }
    lastStepUs = stepStartUs;
    pTerm = iTerm = dTerm = 0.0f;
    saturatedNow = false;

    updateSensors();
    if (calibrating) {
      controlState = STATE_CALIBRATING;
      updateCalibration(now);
    } else if (running) {
      updateLineFollower(now);
      // Bang-bang can keep its last command without a new setWheelSpeeds, so the
      // soft-start ramp is written here too.
      if (now - runStartMs <= SOFT_START_MS + CONTROL_PERIOD_MS) writeWheels();
    } else {
      controlState = STATE_IDLE;
    }
    saveResumeState();

    windowStepMaxUs = max(windowStepMaxUs, micros() - stepStartUs);
    windowSteps++;
    if (saturatedNow) windowSaturated++;
    windowMaxError = max(windowMaxError, fabs(position));
  }

  if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = now;
    sendDebug();          // First, so the GUI logs it with the telemetry that follows
    sendTelemetry();
  }

  if (tuningDirty || now - lastTuningMs >= TUNING_PERIOD_MS) {
    tuningDirty = false;
    lastTuningMs = now;
    sendTuning();
    sendCalibration();
  }

#if !LINK_USE_SOFTWARE_SERIAL
  updateStatusLed(now);
#endif
}
