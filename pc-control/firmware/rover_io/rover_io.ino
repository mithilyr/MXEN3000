// ============================================================
// MXEN3000 S2 2026 - rover I/O firmware (all processing on the PC)
// ============================================================
// The Nano does no control work. Every SENSOR_PERIOD_MS it sends the two IR
// readings to the PC, and it writes whatever DAC codes the PC sends back.
// pc-control/gui/pc_brain.py makes every steering and speed decision.
//
// Safety: the motors start at neutral, and go back to neutral if no motor
// frame arrives for LINK_TIMEOUT_MS (radio dropout or GUI closed).
//
// Frames (checksum = sum of the earlier bytes & 0xFF; data bytes <= DATA_MAX,
// so only a frame start or a checksum can be 0xFF):
//   Rover -> PC, 9 bytes: [START][MSG_SENSORS][irLeft][irRight][dacLeft][dacRight][seq][ackSeq][checksum]
//   PC -> rover, 6 bytes: [START][CMD_MOTORS][dacLeft][dacRight][seq][checksum]
// ackSeq echoes the seq of the last motor frame applied (NO_SEQ before the
// first), so the PC can measure the round trip. Keep these values identical
// to pc-control/gui/io_protocol.py.
//
// Upload: DIP switch S6 off (it connects the XBee to D0/D1), upload, S6 on.

// ---- Link: XBee on hardware serial D0/D1 ----
const unsigned long LINK_BAUD = 38400;      // BD = 5, per the XBee Pairing Guide
const unsigned long SENSOR_PERIOD_MS = 25;  // 40 Hz, one control step on the PC
const unsigned long LINK_TIMEOUT_MS = 300;  // No motor frame for this long = neutral

// ---- Frames ----
const byte FRAME_START = 0xFF;
const byte DATA_MAX = 0xFE;
const byte MSG_SENSORS = 0x30;
const byte CMD_MOTORS = 0x31;
const byte SENSOR_FRAME_LENGTH = 9;
const byte MOTOR_FRAME_LENGTH = 6;
const byte SEQ_COUNT = 254;                 // seq runs 0..253
const byte NO_SEQ = DATA_MAX;

// ---- R-2R DACs, pins listed LSB first (same wiring as onboard-control) ----
const byte DAC_BITS = 8;
const byte DAC_LEFT_PINS[DAC_BITS] = {9, 8, 7, 6, 5, 4, 3, 2};
const byte DAC_RIGHT_PINS[DAC_BITS] = {A2, A3, A4, A5, A1, A0, 11, 10};
const byte DAC_NEUTRAL = 128;
const byte DAC_FULL_SCALE = 255;            // Sent as DATA_MAX, since 255 is FRAME_START

// ---- IR sensors ----
const byte IR_LEFT_PIN = A6;
const byte IR_RIGHT_PIN = A7;
const byte SENSOR_SAMPLES = 4;              // Averaged against motor noise
const byte ADC_TO_BYTE_SHIFT = 2;           // 10-bit ADC -> one data byte

byte dacLeft = 0;                           // 0 forces the first setDacs() to write
byte dacRight = 0;
byte sensorSeq = 0;
byte ackSeq = NO_SEQ;
unsigned long lastMotorFrameMs = 0;
unsigned long lastSensorMs = 0;

// ============================================================
// Motors
// ============================================================
void writeDac(const byte pins[], byte value) {
  for (byte bit = 0; bit < DAC_BITS; bit++) {
    digitalWrite(pins[bit], bitRead(value, bit));
  }
}

// Only touches the pins when a code changes.
void setDacs(byte left, byte right) {
  if (left != dacLeft) writeDac(DAC_LEFT_PINS, left);
  if (right != dacRight) writeDac(DAC_RIGHT_PINS, right);
  dacLeft = left;
  dacRight = right;
}

byte dataToDac(byte data) {
  return data == DATA_MAX ? DAC_FULL_SCALE : data;
}

// ============================================================
// Sensors
// ============================================================
// The first conversion after the ADC switches pin can still hold the other
// pin's voltage, so it is thrown away before averaging.
byte readSensor(byte pin) {
  analogRead(pin);
  int total = 0;
  for (byte i = 0; i < SENSOR_SAMPLES; i++) {
    total += analogRead(pin);
  }
  return min((total / SENSOR_SAMPLES) >> ADC_TO_BYTE_SHIFT, DATA_MAX);
}

// ============================================================
// Radio
// ============================================================
byte checksum(const byte frame[], byte length) {
  byte sum = 0;
  for (byte i = 0; i < length; i++) {
    sum += frame[i];
  }
  return sum;
}

// Collects motor frames byte by byte. A FRAME_START before the checksum
// position always restarts the frame, so a lost byte costs one frame only.
void pollLink(unsigned long now) {
  static byte frame[MOTOR_FRAME_LENGTH];
  static byte received = 0;
  const byte checksumIndex = MOTOR_FRAME_LENGTH - 1;

  while (Serial.available() > 0) {
    byte value = Serial.read();
    if (value == FRAME_START && received < checksumIndex) received = 0;
    if (received == 0 && value != FRAME_START) continue;

    frame[received++] = value;
    if (received < MOTOR_FRAME_LENGTH) continue;
    received = 0;

    if (frame[1] == CMD_MOTORS && checksum(frame, checksumIndex) == frame[checksumIndex]) {
      setDacs(dataToDac(frame[2]), dataToDac(frame[3]));
      ackSeq = frame[4];
      lastMotorFrameMs = now;
    }
  }
}

void sendSensors() {
  byte frame[SENSOR_FRAME_LENGTH] = {
    FRAME_START, MSG_SENSORS,
    readSensor(IR_LEFT_PIN), readSensor(IR_RIGHT_PIN),
    (byte)min(dacLeft, DATA_MAX), (byte)min(dacRight, DATA_MAX),
    sensorSeq, ackSeq, 0
  };
  frame[SENSOR_FRAME_LENGTH - 1] = checksum(frame, SENSOR_FRAME_LENGTH - 1);
  Serial.write(frame, SENSOR_FRAME_LENGTH);
  sensorSeq = (sensorSeq + 1) % SEQ_COUNT;
}

// ============================================================
// Main
// ============================================================
void setup() {
  for (byte bit = 0; bit < DAC_BITS; bit++) {
    pinMode(DAC_LEFT_PINS[bit], OUTPUT);
    pinMode(DAC_RIGHT_PINS[bit], OUTPUT);
  }
  setDacs(DAC_NEUTRAL, DAC_NEUTRAL);
  Serial.begin(LINK_BAUD);
}

void loop() {
  unsigned long now = millis();
  pollLink(now);

  if (now - lastMotorFrameMs > LINK_TIMEOUT_MS) {
    setDacs(DAC_NEUTRAL, DAC_NEUTRAL);      // No-op once already neutral
  }

  if (now - lastSensorMs >= SENSOR_PERIOD_MS) {
    lastSensorMs = now;
    sendSensors();
  }
}
