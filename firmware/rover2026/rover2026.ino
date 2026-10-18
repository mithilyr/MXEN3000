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
// rover spins towards the side it last saw the line on, and stops if it
// cannot find it again.
//
// Safety: motors sit at neutral until START, and the rover stops on its
// own if the GUI's heartbeat is lost for LINK_TIMEOUT_MS.

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
bool linkFailsafe = false;
bool calibrating = false;
bool sensorTest = false;     // Controller runs, motors stay off (GUI bench test)
bool driveSuppressed = false;  // Set only while the sensor test runs the controller

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

// ---- Timers ----
unsigned long lastPacketMs = 0;
unsigned long lastControlMs = 0;
unsigned long lastTelemetryMs = 0;
unsigned long lastTuningMs = 0;
bool tuningDirty = false;  // A tuning command arrived: echo the values at once
unsigned long lostSinceMs = 0;
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

// Speeds are -100..100 % of full speed, positive = forwards for that wheel.
void setWheelSpeeds(int leftPercent, int rightPercent) {
  lastLeftPercent = constrain(leftPercent, -SPEED_MAX, SPEED_MAX);
  lastRightPercent = constrain(rightPercent, -SPEED_MAX, SPEED_MAX);
  dacLeft = speedToDac(leftPercent, LEFT_MOTOR_DIR);
  dacRight = speedToDac(rightPercent, RIGHT_MOTOR_DIR);
  if (driveSuppressed) return;  // Sensor test: report the values, leave the pins alone
  writeDac(DAC_LEFT_PINS, dacLeft);
  writeDac(DAC_RIGHT_PINS, dacRight);
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
  sensorTest = false;
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
  for (byte s = 0; s < SENSOR_COUNT; s++) {
    if (calibrationMax[s] - calibrationMin[s] < MIN_CALIBRATION_SPAN) return;
  }
  for (byte s = 0; s < SENSOR_COUNT; s++) {
    whiteLevel[s] = calibrationMin[s];
    blackLevel[s] = calibrationMax[s];
  }
}

// ============================================================
// Run control
// ============================================================
void clearHistory() {
  historyHead = 0;
  historyCount = 0;
}

void startRun() {
  if (calibrating) return;
  sensorTest = false;
  running = true;
  clearHistory();
  lineLost = false;
  linkFailsafe = false;
  lastTurn = liveTuning[mode].defaultTurn;
  lastPosition = position;
  integral = 0.0f;
  filteredDerivative = 0.0f;
  correcting = false;
  lastCorrectionEndMs = millis();
  lastCorrectionDir = lastTurn;
  correctionRate = 0.0f;
}

void stopRun() {
  running = false;
  holdNeutral();
}

// Sensor test: the controller runs on the live sensors but the motors stay
// at neutral, so hovering the sensors over black and white shows the PWM the
// rover would drive. Only while stopped.
void setSensorTest(bool on) {
  if (running || calibrating) return;
  sensorTest = on;
  lineLost = false;
  clearHistory();
  integral = 0.0f;
  filteredDerivative = 0.0f;
  correcting = false;
  lastCorrectionEndMs = millis();
  correctionRate = 0.0f;
  lastTurn = liveTuning[mode].defaultTurn;
  holdNeutral();
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
// While lineMissing, first retraces the last wheel commands in reverse, then
// spins towards the side the line was last seen on, and stops the run if
// that takes longer than LOST_TIMEOUT_MS. Returns true when it is handling a
// lost line, so the caller must not steer.
bool searchIfLineLost(bool lineMissing, unsigned long now, const ModeTuning& tuning) {
  if (!lineMissing) {
    lineLost = false;
    return false;
  }
  if (!lineLost) {
    lineLost = true;
    lostSinceMs = now;
    nextBacktrackMs = now;
  }
  if (now - lostSinceMs > LOST_TIMEOUT_MS) {
    stopRun();
    return true;
  }
  if (historyCount > 0) {
    // Backtrack: undo the newest recorded command, one sample at a time.
    if (now >= nextBacktrackMs) {
      nextBacktrackMs = now + BACKTRACK_SAMPLE_MS;
      historyHead = (historyHead + BACKTRACK_SAMPLES - 1) % BACKTRACK_SAMPLES;
      historyCount--;
      setWheelSpeeds(-historyLeft[historyHead], -historyRight[historyHead]);
    }
    return true;
  }
  int spin = lastTurn * tuning.searchSpeed;
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

  if (position > SIDE_MEMORY_LEVEL) {
    lastTurn = TURN_ANTICLOCKWISE;
  } else if (position < -SIDE_MEMORY_LEVEL) {
    lastTurn = TURN_CLOCKWISE;
  }
  return false;
}

// turn > 0 rotates the rover clockwise.
void driveWithTurn(const ModeTuning& tuning, float turn) {
  turn = constrain(turn, -tuning.maxTurn, tuning.maxTurn);
  int forward = tuning.travelDir * baseSpeed;
  setWheelSpeeds(forward + turn, forward - turn);
}

// PID on the difference of the two normalised readings.
void updatePid(unsigned long now) {
  const ModeTuning& tuning = liveTuning[mode];
  if (analogLineLost(now, tuning)) {
    integral = 0.0f;              // The error while searching means nothing
    filteredDerivative = 0.0f;
    return;
  }

  const float periodSeconds = CONTROL_PERIOD_MS / 1000.0f;
  float derivative = (position - lastPosition) / periodSeconds;
  filteredDerivative += D_FILTER_ALPHA * (derivative - filteredDerivative);

  // Anti-windup: the integral term on its own can never ask for more than
  // maxTurn, so it cannot keep growing while the steering is clamped.
  if (tuning.ki > 0.0f) {
    float integralLimit = tuning.maxTurn / tuning.ki;
    integral = constrain(integral + position * periodSeconds, -integralLimit, integralLimit);
  } else {
    integral = 0.0f;
  }

  // A positive position (line to the left) needs an anticlockwise
  // correction, hence the minus sign.
  float turn = -tuning.steerSign * (tuning.kp * position + tuning.ki * integral +
                                    tuning.kd * filteredDerivative);
  driveWithTurn(tuning, turn);
}

// Bang-bang: straight while the line is near the centre, full maxTurn
// towards it once |position| passes bangLevel.
void updateBangBang(unsigned long now) {
  const ModeTuning& tuning = liveTuning[mode];
  if (analogLineLost(now, tuning)) return;

  float turn = 0.0f;
  if (position > tuning.bangLevel) {
    turn = -tuning.maxTurn;
  } else if (position < -tuning.bangLevel) {
    turn = tuning.maxTurn;
  }
  driveWithTurn(tuning, tuning.steerSign * turn);
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
  if ((running || sensorTest) && !lineLost) recordHistory(now);
}

// ============================================================
// XBee link
// ============================================================
byte dacFromData(byte data) {
  return data == DATA_MAX ? DAC_FULL_SCALE : data;
}

void handleCommand(byte command, byte data) {
  bool benchTestAllowed = !running && !calibrating && !sensorTest;
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
    case CMD_SET_SENSOR_TEST:
      setSensorTest(data != 0);
      break;
    case CMD_CALIBRATE:
      startCalibration();
      break;
    case CMD_SET_DAC_LEFT:
      if (benchTestAllowed) {
        dacLeft = dacFromData(data);
        writeDac(DAC_LEFT_PINS, dacLeft);
      }
      break;
    case CMD_SET_DAC_RIGHT:
      if (benchTestAllowed) {
        dacRight = dacFromData(data);
        writeDac(DAC_RIGHT_PINS, dacRight);
      }
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
      lastPacketMs = millis();
      handleCommand(frame[1], frame[2]);
      byte command = frame[1];
      if (command == CMD_SET_MODE || command == CMD_SET_SPEED ||
          (command >= CMD_SET_ADAPT_GAIN && command <= CMD_SET_CONTROLLER)) {
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
  if (linkFailsafe) status |= STATUS_LINK_FAILSAFE;
  if (sensorTest) status |= STATUS_SENSOR_TEST;

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
    0
  };
  sendFrame(frame, TUNING_FRAME_LENGTH);
}

// ============================================================
// Status LED: solid = running, fast blink = link failsafe tripped
// ============================================================
// (Unavailable when SoftwareSerial uses D13.)
void updateStatusLed(unsigned long now) {
  bool ledOn = running;
  if (linkFailsafe) {
    ledOn = (now / FAILSAFE_BLINK_MS) % 2;
  }
  digitalWrite(STATUS_LED_PIN, ledOn ? HIGH : LOW);
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

  if (running && now - lastPacketMs > LINK_TIMEOUT_MS) {
    stopRun();
    linkFailsafe = true;
  }

  if (now - lastControlMs >= CONTROL_PERIOD_MS) {
    lastControlMs = now;
    updateSensors();
    if (calibrating) {
      updateCalibration(now);
    } else if (running) {
      updateLineFollower(now);
    } else if (sensorTest) {
      driveSuppressed = true;
      updateLineFollower(now);
      driveSuppressed = false;
    }
  }

  if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = now;
    sendTelemetry();
  }

  if (tuningDirty || now - lastTuningMs >= TUNING_PERIOD_MS) {
    tuningDirty = false;
    lastTuningMs = now;
    sendTuning();
  }

#if !LINK_USE_SOFTWARE_SERIAL
  updateStatusLed(now);
#endif
}
