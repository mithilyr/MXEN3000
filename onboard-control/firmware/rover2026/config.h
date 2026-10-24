// ============================================================
// config.h - pins, hardware constants and per-mode tuning
// ============================================================
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

// Sensor geometry and type. The PCB reads the OPB-704 sensors as analog
// voltages on A6/A7 (10k pull-ups, no comparator), and the sensors are mounted
// at the front over a 3 cm wide white line. Measured on the rover (21-Oct-2026):
// 2.7 cm apart (centre to centre), 1.7 cm from the ground to the sensor face.
// The OPB-704 is focused at 0.38 cm (datasheet), so at 1.7 cm each sensor sees
// a blurred spot, SENSOR_SPOT_CM wide, not a point. A centred rover has each
// sensor SENSOR_GAP_CM inside a line edge, so both read part grey. Sliding
// sideways by e cm:
//   |e| < spot/2 - gap   both sensors change, position = right - left moves
//                        fast and in a straight line (inner range)
//   up to spot/2 + gap   one sensor is fully white, only the other changes, so
//                        position moves at half that rate (outer range), to +-1
//   further              position stays at +-1, then both go black (line lost)
// The PID undoes that kink with the geometry below (lineOffset() in the .ino),
// so its error is the real sideways offset: +-1 = SENSOR_RANGE_CM.
constexpr float SENSOR_SPACING_CM = 2.7f;
constexpr float SENSOR_HEIGHT_CM = 1.7f;
constexpr float LINE_WIDTH_CM = 3.0f;
// Spot width per cm of height. 0.73 assumes a +-20 degree viewing cone (not in
// the datasheet). To measure it: slide the rover sideways by hand and note the
// offset e where position first reaches +-1; then spot = 2 * (e - gap).
constexpr float SENSOR_SPREAD = 0.73f;
constexpr float SENSOR_SPOT_CM = SENSOR_HEIGHT_CM * SENSOR_SPREAD;      // ~1.24 cm
constexpr float SENSOR_EDGE_GAP = (LINE_WIDTH_CM - SENSOR_SPACING_CM) / 2.0f;
constexpr float SENSOR_GAP_CM = SENSOR_EDGE_GAP < 0 ? -SENSOR_EDGE_GAP : SENSOR_EDGE_GAP;  // 0.15 cm
constexpr float SENSOR_RANGE_CM = SENSOR_SPOT_CM / 2.0f + SENSOR_GAP_CM;  // Offset at position +-1, ~0.77 cm
static_assert(SENSOR_SPOT_CM > 2.0f * SENSOR_GAP_CM, "spot narrower than the edge gap: no inner range");
// Each mode picks its controller in MODE_TUNING below, and the GUI can change
// it at any time with T (see the Controller enum in protocol.h):
//   CONTROLLER_PID       PID on the difference of the two analog readings.
//   CONTROLLER_BANG_BANG Straight inside +-bangLevel, full maxTurn outside it.
//   CONTROLLER_ADAPTIVE  Lecture 8 "time since last correction" controller on
//                        1/0 sensors. It needs the sensors wider than the line
//                        (about 4-5 cm) so both are off the line when centred.

// Set true if a sensor over the white line reads HIGH (the comparator wiring
// decides whether light or dark is a 1). The firmware wants white = low.
constexpr bool WHITE_READS_HIGH = false;

// Adaptive controller only. true = the sensors sit either side of the line
// (lecture: both off the line means "go straight", so a lost line cannot be
// detected). false = both sensors sit over the line (both on = straight,
// both off = line lost).
constexpr bool SENSORS_STRADDLE_LINE = true;

// Default raw ADC levels (0-1023) used until a calibration is run.
// From last year's measurements: white line ~10-100, black table > 600.
constexpr int DEFAULT_WHITE_LEVEL = 60;
constexpr int DEFAULT_BLACK_LEVEL = 720;

constexpr unsigned long CALIBRATION_MS = 3000;
constexpr int MIN_CALIBRATION_SPAN = 100;   // Reject a calibration flatter than this

// ---- Line-follow controller ----
constexpr unsigned long CONTROL_PERIOD_MS = 5;

// Each sensor is normalised to 0.0 (white) .. 1.0 (black). Both above
// LOST_LEVEL means the line is between neither sensor any more.
constexpr float LOST_LEVEL = 0.75f;
// |position| above this records which side the line was last seen on.
constexpr float SIDE_MEMORY_LEVEL = 0.2f;
// Each sensor also gets a steady 1/0 reading: 1 = white line, 0 = black table.
// It goes to 1 below WHITE_BELOW and back to 0 above BLACK_ABOVE; in between it
// keeps its last value, so noise or a half-covered sensor cannot make it flicker.
constexpr float WHITE_BELOW = 0.35f;
constexpr float BLACK_ABOVE = 0.65f;

// PID only, one per mode (Clockwise, Anticlockwise, Squiggle, Reverse): right
// (clockwise) turns are multiplied by this, and so is their maxTurn limit.
// 1.0 = symmetric. Raise it for a mode that turns left easily but struggles
// right; 1.3 = 30 % more steering to the right. The bench test
// (DAC_Lab_Inputs.xlsx) shows both motor channels within 1 % duty of each
// other, so the left/right difference comes from the motors, wheels or sensors.
// All four modes use the Squiggle value (24-Oct).
constexpr float RIGHT_TURN_GAIN[MODE_COUNT] = {1.5f, 1.5f, 1.5f, 1.5f};

// PID: the derivative is low-pass filtered, because a raw difference of two
// sensor readings 5 ms apart is mostly noise. Each period the filtered value
// moves this fraction of the way to the new raw value (1.0 = no filter,
// 0.2 = roughly the average of the last 5 periods, about 25 ms).
constexpr float D_FILTER_ALPHA = 0.2f;

// Adaptive controller (lecture 8, "A 2-sensor control algorithm"). Each time
// a sensor reaches the line, the turn rate is ADAPT_GAIN divided by the time
// since the last correction. A same-side correction soon after the last one
// adds to the previous rate. While one correction lasts, the rate also ramps up.
constexpr float MIN_CORRECTION_GAP_S = 0.02f;  // Floor on that time, avoids a divide-by-zero spike
constexpr float CORRECTION_MEMORY_S = 1.0f;    // Older corrections are forgotten
constexpr int MIN_CORRECTION_TURN = 15;        // Shallowest turn, % of DAC_SPAN

// Backtracking: when the line is lost the rover first replays its last wheel
// commands in reverse (newest first) to retrace its path, then falls back to
// the spin search. History = BACKTRACK_SAMPLES x BACKTRACK_SAMPLE_MS (0.6 s).
constexpr unsigned long BACKTRACK_SAMPLE_MS = 20;
constexpr byte BACKTRACK_SAMPLES = 30;
// Corner search (both sensors white, then both black at once, e.g. a 90 degree
// bend): no backtrack; spin towards lastTurn for SWEEP_MS, then sweep back and
// forth 2 x SWEEP_MS each way at searchSpeed. Aim for about 60 degrees per
// SWEEP_MS: if the rover spins past the new line's angle, lower it.
constexpr unsigned long SWEEP_MS = 350;
// A loss within this long of one sensor alone seeing the line (e.g. left goes
// black, then right a moment later) also counts as a corner.
constexpr unsigned long CORNER_WINDOW_MS = 150;
// Corner hold, PID only, one per mode (Clockwise, Anticlockwise, Squiggle,
// Reverse): for this long after a corner search finds the line, while both
// sensors read white, the rover arcs the way it was spinning at
// CORNER_HOLD_TURN x maxTurn instead of driving straight. 0 = off.
// Longer or stronger = bigger turns out of each bend.
// All four modes use the Squiggle value (24-Oct).
constexpr unsigned long CORNER_HOLD_MS[MODE_COUNT] = {250, 250, 250, 250};
constexpr float CORNER_HOLD_TURN = 0.6f;

constexpr int SPEED_MAX = 100;              // Speeds are % of DAC_SPAN

// Turn direction convention: +1 = clockwise (to the right), -1 = anticlockwise.
constexpr int TURN_CLOCKWISE = 1;
constexpr int TURN_ANTICLOCKWISE = -1;

struct ModeTuning {
  int travelDir;        // +1 forwards, -1 backwards
  int defaultTurn;      // Search direction before the line has been seen off-centre
  byte controller;      // Controller enum (protocol.h), changeable from the GUI with T
  int baseSpeed;        // % of DAC_SPAN, changeable live from the GUI
  int searchSpeed;      // % of DAC_SPAN while spinning to find a lost line
  float kp;             // PID: % turn per unit of position error
  float ki;             // PID: % turn per (unit of position error x second)
  float kd;             // PID: % turn per (unit of position error per second)
  float adaptGain;      // Adaptive: turn % x seconds, divided by time since last correction
  float adaptRamp;      // Adaptive: extra turn % per second while a correction lasts
  int maxTurn;          // Clamp on the steering term (bang-bang: the turn used), % of DAC_SPAN
  float bangLevel;      // Bang-bang: |position| above this turns at maxTurn, 0-1
  int steerSign;        // Flip to -1 if this mode steers the wrong way on test
  // Bang-bang wheel levels, electrical PWM duty %: 100 forward max, 50 stopped, 0 reverse max.
  int pwmStraightLeft;  // Left wheel with the line centred
  int pwmStraightRight;
  int pwmCorrectLeft;   // Left wheel while it is the slow side of a correction
  int pwmCorrectRight;
};

// pwm* columns (bang-bang only): left/right duty with the line centred, then left/right
// duty while that wheel is the slow side of a correction.
// Starting values only. The GUI can change them live (they are copied to RAM
// at power-up, so a reset brings these back). Tune each mode on the stadium
// track, then copy the best values from the GUI back into this table.
// Every mode starts on bang-bang (24-Oct); PID and adaptive stay available
// from the GUI with T. ki starts at 0 (plain PD). Add it only if the rover runs off-centre on the
// long curves.
constexpr ModeTuning MODE_TUNING[MODE_COUNT] = {
  // travel  defaultTurn          controller       base  search  kp     ki    kd    gain   ramp   maxTurn  bang   steer  strL strR corL corR
  {  1,      TURN_CLOCKWISE,      CONTROLLER_BANG_BANG, 45,   30,     80.0f, 0.0f, 1.5f, 10.0f, 60.0f, 70,      0.3f,  1,     90,  90,  55,  55 },  // Clockwise: exact copy of Squiggle (24-Oct)
  {  1,      TURN_CLOCKWISE,      CONTROLLER_BANG_BANG, 45,   30,     80.0f, 0.0f, 1.5f, 10.0f, 60.0f, 70,      0.3f,  1,     90,  90,  55,  55 },  // Anticlockwise: exact copy of Squiggle (24-Oct)
  {  1,      TURN_CLOCKWISE,      CONTROLLER_BANG_BANG, 45,   30,     80.0f, 0.0f, 1.5f, 10.0f, 60.0f, 70,      0.3f,  1,     90,  90,  55,  55 },  // Squiggle
  { -1,      TURN_ANTICLOCKWISE,  CONTROLLER_BANG_BANG, 45,   30,     80.0f, 0.0f, 1.5f, 10.0f, 60.0f, 70,      0.3f, -1,     30,  30,  45,  45 },  // Reverse: Squiggle mirrored - drives backwards, steering swapped left/right, own reverse PWM levels (24-Oct)
};
