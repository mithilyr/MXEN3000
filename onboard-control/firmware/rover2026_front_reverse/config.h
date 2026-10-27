// ============================================================
// config.h - pins, hardware constants and per-mode tuning
// ============================================================
// FRONT + REVERSE - ../rover2026_last_working_24.10.2026/ (commit 138dc74)
// with the Clockwise and Anticlockwise modes and the PID and adaptive
// controllers removed: bang-bang only. Front and Reverse wheel levels
// unchanged.
// Every number you are expected to tune on the track lives here.
#pragma once

#include <Arduino.h>
#include "protocol.h"

// ---- XBee link ----
// 0 = XBee on hardware serial D0 (RX) / D1 (TX). This is how the PCB is
//     wired (connector J10 through DIP switch S6). Turn S6 off while
//     uploading over USB, then back on.
// 1 = XBee hand-wired to the SoftwareSerial pins below (bypassing J10). The
//     status LED is then disabled because D13 is used by the link.
#define LINK_USE_SOFTWARE_SERIAL 0

constexpr unsigned long LINK_BAUD = 38400;  // BD = 5, per the XBee Pairing Guide
constexpr byte SOFT_SERIAL_RX_PIN = 12;     // Wire to XBee DOUT
constexpr byte SOFT_SERIAL_TX_PIN = 13;     // Wire to XBee DIN

// Fewer, larger gaps between frames: each frame is one radio packet, and the
// Zigbee link handles a few dozen a second at best. 10 Hz is plenty for the GUI.
constexpr unsigned long TELEMETRY_PERIOD_MS = 100;
// The tuning echo is also sent straight after any tuning command, so this is
// only a slow refresh in case a frame was lost.
constexpr unsigned long TUNING_PERIOD_MS = 1000;

// ---- Status LED (built-in, D13) ----
constexpr byte STATUS_LED_PIN = LED_BUILTIN;

// ---- R-2R DACs (LSB first) ----
constexpr byte DAC_BITS = 8;
// Right (first board) was read from the PCB schematics (docs/pcb details/sch2.pdf,
// sch3.pdf and labs/lab4-final-vehicle/PCB_Circuits_Diagrams (1).pdf; the
// DAC0800 input B1 is the MSB, B8 the LSB): B1..B8 = D10 D11 A0 A1 A5 A4 A3 A2.
// Left (second board) was changed on 2026-10-12 from the schematic order
// (D2 D3 D4 D5 D9 D8 D7 D6, LSB first) to consecutive D9..D2, LSB first.
constexpr byte DAC_LEFT_PINS[DAC_BITS] = {9, 8, 7, 6, 5, 4, 3, 2};
constexpr byte DAC_RIGHT_PINS[DAC_BITS] = {A2, A3, A4, A5, A1, A0, 11, 10};

// DAC value that holds a motor still, and the swing either side of it.
constexpr int DAC_NEUTRAL = 128;
constexpr int DAC_SPAN = 127;

// +1 if a DAC value above DAC_NEUTRAL drives that wheel forwards, -1 if
// that wheel is wired or mounted the other way round.
constexpr int LEFT_MOTOR_DIR = 1;
constexpr int RIGHT_MOTOR_DIR = 1;

// ---- IR line sensors ----
constexpr byte IR_LEFT_PIN = A6;
constexpr byte IR_RIGHT_PIN = A7;
constexpr byte SENSOR_SAMPLES = 4;          // analogRead averaging
constexpr int ADC_MAX = 1023;               // 10-bit ADC full scale

// The PCB reads the OPB-704 sensors as analog voltages on A6/A7 (10k
// pull-ups, no comparator), mounted at the front over the line's two edges.

// Set true if a sensor over the white line reads HIGH (the comparator wiring
// decides whether light or dark is a 1). The firmware wants white = low.
constexpr bool WHITE_READS_HIGH = false;

// Default raw ADC levels (0-1023) used until a calibration is run.
// From last year's measurements: white line ~10-100, black table > 600.
constexpr int DEFAULT_WHITE_LEVEL = 60;
constexpr int DEFAULT_BLACK_LEVEL = 720;

constexpr unsigned long CALIBRATION_MS = 3000;
constexpr int MIN_CALIBRATION_SPAN = 100;   // Reject a calibration flatter than this

// ---- Bang-bang line follower ----
constexpr unsigned long CONTROL_PERIOD_MS = 5;

// Each sensor is normalised to 0.0 (white) .. 1.0 (black), and also gets a
// steady 1/0 reading: 1 = white line, 0 = black table. It goes to 1 below
// WHITE_BELOW and back to 0 above BLACK_ABOVE; in between it keeps its last
// value, so noise or a half-covered sensor cannot make it flicker. Bang-bang
// counts a sensor as on black above BLACK_ABOVE.
constexpr float WHITE_BELOW = 0.35f;
constexpr float BLACK_ABOVE = 0.65f;

constexpr int SPEED_MAX = 100;              // Wheel speeds are % of DAC_SPAN

// Soft start: for this long after START the wheel commands are scaled up from
// 0 to full. Both motors jumping from stop to 80 % at once pulled the supply
// down far enough to reset the Nano (25-Oct logs: debug seq restarted ~0.2 s
// after START). 0 = off.
constexpr unsigned long SOFT_START_MS = 300;

struct ModeTuning {
  int baseSpeed;        // % shown in the GUI and remembered per mode; bang-bang drives from the levels below
  int steerSign;        // Flip to -1 if this mode steers the wrong way on test
  // Wheel levels, electrical PWM duty %: 100 forward max, 50 stopped, 0 reverse max.
  int pwmStraightLeft;  // Left wheel with the line centred
  int pwmStraightRight;
  int pwmCorrectLeft;   // Left wheel while it is the slow side of a correction
  int pwmCorrectRight;
};

// strL/strR: left/right duty with the line centred. corL/corR: left/right duty
// while that wheel is the slow side of a correction. Reverse uses duties below
// 50 % so both wheels run backwards.
// Starting values only. The GUI can change them live (they are copied to RAM
// at power-up, so a reset brings these back). Tune each mode on the track,
// then copy the best values from the GUI back into this table.
constexpr ModeTuning MODE_TUNING[MODE_COUNT] = {
  // base  steer  strL strR corL corR
  {  45,   1,     65,  65,  50,  50 },  // Front
  {  25,   1,     35,  35,  50,  50 },  // Reverse
};
