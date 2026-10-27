// ============================================================
// protocol.h - XBee link protocol shared with gui/protocol.py
// ============================================================
// Keep every value in this file identical to gui/protocol.py.
//
// The XBees run in transparent (AT) mode, so the link behaves like a
// plain serial cable. Every frame starts with FRAME_START and ends with
// an 8-bit checksum: (sum of all previous bytes in the frame) & 0xFF.
// Data bytes never use FRAME_START (they are clamped to DATA_MAX), so a
// receiver can re-synchronise by waiting for the next FRAME_START.
//
// PC -> rover, 4 bytes:  [START][command][data][checksum]
// Rover -> PC, 11 bytes: [START][TELEMETRY][irLeft][irRight][dacLeft]
//                        [dacRight][position][speed][status][sensors][checksum]
// Rover -> PC, 12 bytes: [START][TUNING][mode][controller][gain x10][ramp]
//                        [maxTurn][kp][kd x10][ki][bangLevel x100][checksum]
// Rover -> PC, 17 bytes: [START][DEBUG][seq][state][pTerm][iTerm][dTerm][turn]
//                        [adaptRate][saturated %][maxError x100][steps]
//                        [stepMax x0.1ms][periodMax x0.1ms][lostEvents][flags][checksum]
// Rover -> PC, 8 bytes:  [START][CALIBRATION][whiteL][whiteR][blackL][blackR]
//                        [result][checksum]
#pragma once

#include <Arduino.h>

constexpr byte FRAME_START = 0xFF;
constexpr byte DATA_MAX = 0xFE;

// ---- Commands (PC -> rover) ----
constexpr byte CMD_PING = 0x01;           // Heartbeat, data ignored
constexpr byte CMD_START = 0x02;          // Start driving in the current mode
constexpr byte CMD_STOP = 0x03;           // Stop and hold motors at neutral
constexpr byte CMD_SET_MODE = 0x04;       // data = Mode (only while stopped)
constexpr byte CMD_SET_SPEED = 0x05;      // data = base speed, 0-100 %
constexpr byte CMD_CALIBRATE = 0x06;      // Record sensor white/black levels
// A data byte stops at DATA_MAX, so on a telemetry DAC field DATA_MAX stands
// for DAC_FULL_SCALE (255 = +15 V at the motor).
constexpr byte DAC_FULL_SCALE = 0xFF;

// Live tuning of the current mode (allowed while running). data is the
// value in the unit named here, so the GUI and firmware never disagree.
constexpr byte CMD_SET_ADAPT_GAIN = 0x09; // data = digital turn gain x GAIN_SCALE
constexpr byte CMD_SET_ADAPT_RAMP = 0x0A; // data = digital ramp, % turn per second
constexpr byte CMD_SET_MAX_TURN = 0x0B;   // data = steering clamp, 0-100 %
constexpr byte CMD_SET_KP = 0x0C;         // data = analog kp
constexpr byte CMD_SET_KD = 0x0D;         // data = analog kd x KD_SCALE
constexpr byte CMD_SET_KI = 0x0E;         // data = analog ki
constexpr byte CMD_SET_BANG_LEVEL = 0x0F; // data = bang-bang level x BANG_LEVEL_SCALE
constexpr byte CMD_SET_CONTROLLER = 0x10; // data = Controller (also while running)
constexpr byte CMD_RESET_INTEGRAL = 0x11; // Clear the PID integral, data ignored
// Bang-bang wheel levels, as electrical PWM duty %: 100 = full forward, 50 = stopped,
// 0 = full reverse. data = duty % (0-100), for the current mode.
constexpr byte CMD_SET_PWM_STRAIGHT_LEFT = 0x12;  // Left wheel when the line is centred
constexpr byte CMD_SET_PWM_STRAIGHT_RIGHT = 0x13; // Right wheel when the line is centred
constexpr byte CMD_SET_PWM_CORRECT_LEFT = 0x14;   // Left wheel while it is the slow side of a correction
constexpr byte CMD_SET_PWM_CORRECT_RIGHT = 0x15;  // Right wheel while it is the slow side of a correction
constexpr float GAIN_SCALE = 10.0f;
constexpr float KD_SCALE = 10.0f;
constexpr float BANG_LEVEL_SCALE = 100.0f;

constexpr byte COMMAND_FRAME_LENGTH = 4;

// ---- Telemetry (rover -> PC) ----
constexpr byte MSG_TELEMETRY = 0x20;
constexpr byte TELEMETRY_FRAME_LENGTH = 11;
constexpr byte MSG_TUNING = 0x21;         // The values the rover is using right now
constexpr byte TUNING_FRAME_LENGTH = 16;

// Status byte bit layout
constexpr byte STATUS_RUNNING = 0x01;
constexpr byte STATUS_LINE_LOST = 0x02;
constexpr byte STATUS_CALIBRATING = 0x04;
constexpr byte STATUS_MODE_SHIFT = 4;     // Mode lives in bits 4-6

// Sensors byte bit layout: 1 = sensor is over the white line, 0 = over black
constexpr byte SENSOR_LEFT_WHITE = 0x01;
constexpr byte SENSOR_RIGHT_WHITE = 0x02;

// ---- Debug telemetry (rover -> PC), sent just before each telemetry frame ----
// The controller's insides for tuning. stepMax, periodMax, maxError, saturated %
// and steps cover EVERY control step since the previous debug frame, so nothing
// between the 10 Hz frames is missed. Signed values (the turn terms, % of
// full speed, > 0 = clockwise) are sent as value + SIGNED_OFFSET.
constexpr byte MSG_DEBUG = 0x22;
constexpr byte DEBUG_FRAME_LENGTH = 17;
constexpr int SIGNED_OFFSET = 127;
constexpr byte SEQ_COUNT = 254;           // seq runs 0..253, so lost frames can be counted
constexpr byte DEBUG_SATURATED = 0x01;    // Flags: the steering hit maxTurn this step

// What the controller is doing (debug frame "state")
enum ControlState : byte {
  STATE_IDLE = 0,
  STATE_CALIBRATING = 1,
  STATE_FOLLOW = 2,           // PID tracking, or adaptive driving straight
  STATE_BACKTRACK = 3,        // Line lost: retracing the last wheel commands
  STATE_SPIN = 4,             // Line lost: spinning towards the last side seen
  STATE_CORNER_SWEEP = 5,     // Line lost at a corner: sweeping in place
  STATE_CORNER_HOLD = 6,      // PID: arcing round a corner just found
  STATE_ADAPT_TURN = 7,       // Adaptive: correcting
  STATE_BANG_STRAIGHT = 8,
  STATE_BANG_LEFT = 9,        // Bang-bang: left wheel slowed (turning left)
  STATE_BANG_RIGHT = 10,      // Bang-bang: right wheel slowed (turning right)
  STATE_BANG_BOTH_BLACK = 11
};

// ---- Calibration levels (rover -> PC), sent with every tuning frame ----
// Raw ADC levels >> 2 the rover uses for white and black on each side.
constexpr byte MSG_CALIBRATION = 0x23;
constexpr byte CALIBRATION_FRAME_LENGTH = 8;
enum CalibrationResult : byte {
  CAL_DEFAULTS = 0,           // Never calibrated: config.h levels
  CAL_ACCEPTED = 1,
  CAL_REJECTED = 2            // Last sweep's span < MIN_CALIBRATION_SPAN: levels unchanged
};

// ---- Driving modes ----
// Front and Reverse only. They keep their old values (2, 3) so the mode bits
// in telemetry and the tuning frame stay the same as the full four-mode
// firmware. Per-mode tables in config.h are indexed by mode - MODE_FIRST.
enum Mode : byte {
  MODE_FRONT = 2,
  MODE_REVERSE = 3
};
constexpr byte MODE_FIRST = MODE_FRONT;
constexpr byte MODE_COUNT = 2;

// ---- Steering controller ----
// Bang-bang only. It keeps its old value (1) for the tuning frame. The PID and
// adaptive commands above stay defined to match gui/protocol.py, but this
// firmware ignores them.
enum Controller : byte {
  CONTROLLER_BANG_BANG = 1
};
