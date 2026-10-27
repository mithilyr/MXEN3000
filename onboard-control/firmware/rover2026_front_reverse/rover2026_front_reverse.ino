// ============================================================
// MXEN3000 S2 2026 - wireless line-following rover
// ============================================================
// FRONT + REVERSE, BANG-BANG ONLY - ../rover2026_last_working_24.10.2026/
// (commit 138dc74) with the Clockwise and Anticlockwise modes and the PID and
// adaptive controllers removed. Pairs with gui/rover_gui_front_reverse.py
// (RoverControl_front_reverse.exe).
//
// The GUI picks the mode (Front or Reverse) and starts/stops the run over
// an XBee link (see protocol.h). All tuning is in config.h.
//
// Track: white line on a black table. Two analog IR sensors sit at the
// front, over the two edges of the line. Bang-bang steering: each wheel runs
// at its own PWM duty, and the wheel opposite a sensor that reads black drops
// to its correction duty, which steers back onto the line.
//
// Safety: motors sit at neutral until START. Once started, the rover keeps
// running through radio dropouts until it gets a STOP command.

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
Mode mode = MODE_FRONT;
byte slot = 0;                       // mode - MODE_FIRST: index into MODE_TUNING
ModeTuning liveTuning[MODE_COUNT];   // RAM copy of MODE_TUNING, edited live from the GUI
int baseSpeed = MODE_TUNING[0].baseSpeed;
bool running = false;
unsigned long runStartMs = 0;        // For the soft start (SOFT_START_MS)
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

// ---- Outputs ----
byte dacLeft = DAC_NEUTRAL;
byte dacRight = DAC_NEUTRAL;
int8_t lastLeftPercent = 0;                 // Last wheel commands, -100..100 %
int8_t lastRightPercent = 0;

// ---- Debug telemetry (see protocol.h MSG_DEBUG) ----
byte controlState = STATE_IDLE;
byte calibrationResult = CAL_DEFAULTS;
byte debugSeq = 0;
// Since the last debug frame:
unsigned int windowSteps = 0;
float windowMaxError = 0.0f;                // Largest |position|
unsigned long windowStepMaxUs = 0;          // Longest control step
unsigned long windowPeriodMaxUs = 0;        // Longest time between control steps
unsigned long lastStepUs = 0;

// ---- Timers ----
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
void startRun() {
  if (calibrating) return;
  running = true;
  runStartMs = millis();
}

void stopRun() {
  running = false;
  holdNeutral();
}

// Only Front and Reverse exist; any other mode number is ignored.
void setMode(byte newMode) {
  if (running || newMode < MODE_FIRST || newMode >= MODE_FIRST + MODE_COUNT) return;
  mode = (Mode)newMode;
  slot = newMode - MODE_FIRST;
  baseSpeed = liveTuning[slot].baseSpeed;
}

// ============================================================
// Bang-bang line follower
// ============================================================
// Electrical PWM duty (100 = full forward, 50 = stopped, 0 = full reverse) to
// a wheel speed, -100..100 %.
int dutyToSpeed(int duty) {
  return (constrain(duty, 0, 100) - 50) * 2;
}

// Each wheel runs at its own PWM duty (pwmStraight*). The sensors sit over the
// two edges of the white line, so a sensor reading black means the rover has
// drifted towards that side and the line is on the other side. The wheel on
// the OPPOSITE side then drops to its pwmCorrect* duty, which steers back onto
// the line. Between WHITE_BELOW and BLACK_ABOVE a sensor counts as still on
// the line. steerSign = -1 swaps the sides if this steers the wrong way on
// test. There is no lost-line search: with both sensors black the rover keeps
// its last wheel command (or the centred levels at the start of a run, before
// it has made one) until a sensor sees the line again.
void updateBangBang() {
  const ModeTuning& tuning = liveTuning[slot];

  bool leftBlack = normLevel[SENSOR_LEFT] > BLACK_ABOVE;
  bool rightBlack = normLevel[SENSOR_RIGHT] > BLACK_ABOVE;
  if (leftBlack && rightBlack) controlState = STATE_BANG_BOTH_BLACK;
  if (leftBlack && rightBlack && (lastLeftPercent != 0 || lastRightPercent != 0)) return;
  if (tuning.steerSign < 0) {
    bool swap = leftBlack;
    leftBlack = rightBlack;
    rightBlack = swap;
  }
  if (rightBlack != leftBlack) {
    controlState = rightBlack ? STATE_BANG_LEFT : STATE_BANG_RIGHT;  // Slowed wheel's side
  } else if (!rightBlack) {
    controlState = STATE_BANG_STRAIGHT;
  }
  setWheelSpeeds(dutyToSpeed(rightBlack ? tuning.pwmCorrectLeft : tuning.pwmStraightLeft),
                 dutyToSpeed(leftBlack ? tuning.pwmCorrectRight : tuning.pwmStraightRight));
}

// ============================================================
// XBee link
// ============================================================
// The PID and adaptive commands (gains, max turn, controller, integral reset)
// fall through and are ignored.
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
      liveTuning[slot].baseSpeed = baseSpeed;  // Remembered when the mode is reselected
      break;
    case CMD_CALIBRATE:
      startCalibration();
      break;
    case CMD_SET_PWM_STRAIGHT_LEFT:
      liveTuning[slot].pwmStraightLeft = min((int)data, 100);
      break;
    case CMD_SET_PWM_STRAIGHT_RIGHT:
      liveTuning[slot].pwmStraightRight = min((int)data, 100);
      break;
    case CMD_SET_PWM_CORRECT_LEFT:
      liveTuning[slot].pwmCorrectLeft = min((int)data, 100);
      break;
    case CMD_SET_PWM_CORRECT_RIGHT:
      liveTuning[slot].pwmCorrectRight = min((int)data, 100);
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
// The PID / adaptive fields of the frame are sent as zero.
void sendDebug() {
  byte frame[DEBUG_FRAME_LENGTH] = {
    FRAME_START,
    MSG_DEBUG,
    debugSeq,
    controlState,
    toSignedByte(0),                // P term
    toSignedByte(0),                // I term
    toSignedByte(0),                // D term
    toSignedByte((lastLeftPercent - lastRightPercent) / 2.0f),  // Turn the wheels give, > 0 = clockwise
    0,                              // Adaptive turn rate
    0,                              // % of steps saturated
    toDataByte(lround(windowMaxError * 100)),
    toDataByte(windowSteps),
    tenthsOfMs(windowStepMaxUs),
    tenthsOfMs(windowPeriodMaxUs),
    0,                              // Line-lost events
    0,                              // Flags
    0
  };
  sendFrame(frame, DEBUG_FRAME_LENGTH);
  debugSeq = (debugSeq + 1) % SEQ_COUNT;
  windowSteps = 0;
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
// rover really has, not what was last requested. The PID / adaptive fields
// of the frame are sent as zero.
void sendTuning() {
  const ModeTuning& tuning = liveTuning[slot];
  byte frame[TUNING_FRAME_LENGTH] = {
    FRAME_START,
    MSG_TUNING,
    mode,
    CONTROLLER_BANG_BANG,
    0,                              // Adaptive gain
    0,                              // Adaptive ramp
    0,                              // Max turn
    0,                              // kp
    0,                              // kd
    0,                              // ki
    0,                              // Bang level
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
void updateStatusLed() {
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

    updateSensors();
    if (calibrating) {
      controlState = STATE_CALIBRATING;
      updateCalibration(now);
    } else if (running) {
      updateBangBang();
      // Bang-bang can keep its last command without a new setWheelSpeeds, so the
      // soft-start ramp is written here too.
      if (now - runStartMs <= SOFT_START_MS + CONTROL_PERIOD_MS) writeWheels();
    } else {
      controlState = STATE_IDLE;
    }

    windowStepMaxUs = max(windowStepMaxUs, micros() - stepStartUs);
    windowSteps++;
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
  updateStatusLed();
#endif
}
