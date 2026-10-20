// ============================================================
// MXEN3000 S2 2026 - rover I/O firmware (all processing on the PC)
// ============================================================
// The Nano does no control work. It only:
//   1. reads the two IR sensors and sends the raw readings to the PC,
//   2. writes the two DAC codes the PC sends back to the R-2R DACs.
// The PC (pc-control/gui/pc_brain.py) decides every duty cycle: line
// following, calibration, lost-line search and tuning all run there.
//
// Safety: motors sit at neutral at power-up. If no motor frame arrives for
// LINK_TIMEOUT_MS (radio dropout, GUI closed), the motors go back to neutral,
// because nothing on the rover can steer by itself.
//
// Frames (checksum = sum of all previous bytes & 0xFF, data bytes <= DATA_MAX):
//   Rover -> PC, 9 bytes: [START][MSG_SENSORS][irLeft][irRight][dacLeft][dacRight]
//                         [seq][ackSeq][checksum]
//   PC -> rover, 6 bytes: [START][CMD_MOTORS][dacLeft][dacRight][seq][checksum]
// seq counts 0..SEQ_COUNT-1 on each side. ackSeq echoes the seq of the last
// motor frame applied (NO_SEQ before the first), so the PC can measure the
// radio round trip and count lost frames. The Nano only copies the number.
// Keep these values identical to pc-control/gui/io_protocol.py.

// ---- Link ----
// 0 = XBee on hardware serial D0/D1 (PCB connector J10 via DIP switch S6;
//     turn S6 off while uploading over USB). 1 = XBee hand-wired to D12/D13.
#define LINK_USE_SOFTWARE_SERIAL 0
const unsigned long LINK_BAUD = 38400;      // BD = 5, per the XBee Pairing Guide
const byte SOFT_SERIAL_RX_PIN = 12;         // Wire to XBee DOUT
const byte SOFT_SERIAL_TX_PIN = 13;         // Wire to XBee DIN

// One sensor frame = one control step on the PC. 25 ms = 40 Hz. Try 20 ms
// only if the GUI's LINK panel shows no lost frames at 25 ms.
const unsigned long SENSOR_PERIOD_MS = 25;
const unsigned long LINK_TIMEOUT_MS = 300;  // No motor frame for this long = neutral

// ---- Frames ----
const byte FRAME_START = 0xFF;
const byte DATA_MAX = 0xFE;
const byte MSG_SENSORS = 0x30;
const byte CMD_MOTORS = 0x31;
const byte SENSOR_FRAME_LENGTH = 9;
const byte MOTOR_FRAME_LENGTH = 6;
const byte SEQ_COUNT = 254;                 // seq runs 0..253
const byte NO_SEQ = DATA_MAX;               // No motor frame applied yet

// ---- R-2R DACs (LSB first), same wiring as the onboard-control config.h ----
const byte DAC_BITS = 8;
const byte DAC_LEFT_PINS[DAC_BITS] = {9, 8, 7, 6, 5, 4, 3, 2};
const byte DAC_RIGHT_PINS[DAC_BITS] = {A2, A3, A4, A5, A1, A0, 11, 10};
const byte DAC_NEUTRAL = 128;
const byte DAC_FULL_SCALE = 255;            // Sent as DATA_MAX, since 255 is FRAME_START

// ---- IR sensors ----
const byte IR_LEFT_PIN = A6;
const byte IR_RIGHT_PIN = A7;
const byte ADC_TO_BYTE_SHIFT = 2;           // 10-bit ADC -> 8-bit data byte

#if LINK_USE_SOFTWARE_SERIAL
#include <SoftwareSerial.h>
SoftwareSerial softLink(SOFT_SERIAL_RX_PIN, SOFT_SERIAL_TX_PIN);
Stream& xbee = softLink;
#else
Stream& xbee = Serial;
#endif

byte dacLeft = DAC_NEUTRAL;
byte dacRight = DAC_NEUTRAL;
unsigned long lastMotorFrameMs = 0;
unsigned long lastSensorMs = 0;
byte sensorSeq = 0;
byte ackSeq = NO_SEQ;

void writeDac(const byte pins[], byte value) {
  for (byte bit = 0; bit < DAC_BITS; bit++) {
    digitalWrite(pins[bit], bitRead(value, bit) ? HIGH : LOW);
  }
}

void setDacs(byte left, byte right) {
  dacLeft = left;
  dacRight = right;
  writeDac(DAC_LEFT_PINS, dacLeft);
  writeDac(DAC_RIGHT_PINS, dacRight);
}

// DATA_MAX on the link stands for the DAC's full scale (255).
byte dataToDac(byte data) {
  return data == DATA_MAX ? DAC_FULL_SCALE : data;
}

// Keeps a value off FRAME_START, so the PC can always re-synchronise.
byte toDataByte(int value) {
  return constrain(value, 0, DATA_MAX);
}

// Collects [START][CMD_MOTORS][dacLeft][dacRight][seq][checksum].
void pollLink(unsigned long now) {
  static byte frame[MOTOR_FRAME_LENGTH];
  static byte received = 0;
  const byte checksumIndex = MOTOR_FRAME_LENGTH - 1;

  while (xbee.available() > 0) {
    byte value = xbee.read();
    if (value == FRAME_START && received < checksumIndex) {
      received = 0;
    }
    if (received == 0 && value != FRAME_START) continue;

    frame[received++] = value;
    if (received < MOTOR_FRAME_LENGTH) continue;

    received = 0;
    byte checksum = 0;
    for (byte i = 0; i < checksumIndex; i++) {
      checksum += frame[i];
    }
    if (checksum == frame[checksumIndex] && frame[1] == CMD_MOTORS) {
      setDacs(dataToDac(frame[2]), dataToDac(frame[3]));
      ackSeq = frame[4];
      lastMotorFrameMs = now;
    }
  }
}

void sendSensors() {
  byte frame[SENSOR_FRAME_LENGTH] = {
    FRAME_START,
    MSG_SENSORS,
    toDataByte(analogRead(IR_LEFT_PIN) >> ADC_TO_BYTE_SHIFT),
    toDataByte(analogRead(IR_RIGHT_PIN) >> ADC_TO_BYTE_SHIFT),
    toDataByte(dacLeft),
    toDataByte(dacRight),
    sensorSeq,
    ackSeq,
    0
  };
  sensorSeq = (sensorSeq + 1) % SEQ_COUNT;
  byte checksum = 0;
  for (byte i = 0; i < SENSOR_FRAME_LENGTH - 1; i++) {
    checksum += frame[i];
  }
  frame[SENSOR_FRAME_LENGTH - 1] = checksum;
  xbee.write(frame, SENSOR_FRAME_LENGTH);
}

void setup() {
  for (byte bit = 0; bit < DAC_BITS; bit++) {
    pinMode(DAC_LEFT_PINS[bit], OUTPUT);
    pinMode(DAC_RIGHT_PINS[bit], OUTPUT);
  }
  setDacs(DAC_NEUTRAL, DAC_NEUTRAL);
#if LINK_USE_SOFTWARE_SERIAL
  softLink.begin(LINK_BAUD);
#else
  Serial.begin(LINK_BAUD);
#endif
}

void loop() {
  unsigned long now = millis();
  pollLink(now);

  if (now - lastMotorFrameMs > LINK_TIMEOUT_MS &&
      (dacLeft != DAC_NEUTRAL || dacRight != DAC_NEUTRAL)) {
    setDacs(DAC_NEUTRAL, DAC_NEUTRAL);
  }

  if (now - lastSensorMs >= SENSOR_PERIOD_MS) {
    lastSensorMs = now;
    sendSensors();
  }
}
