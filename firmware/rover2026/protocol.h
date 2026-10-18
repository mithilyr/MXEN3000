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
constexpr byte CMD_SET_DAC_LEFT = 0x07;   // Bench test: DAC value (stopped only)
constexpr byte CMD_SET_DAC_RIGHT = 0x08;  // Bench test: DAC value (stopped only)
// A data byte stops at DATA_MAX, so on a DAC command or a telemetry DAC field
// DATA_MAX stands for DAC_FULL_SCALE (255 = +15 V at the motor).
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
constexpr byte CMD_SET_SENSOR_TEST = 0x12; // data = 1: motors stay off but the controller runs and reports its PWM; 0: off (stopped only)
constexpr float GAIN_SCALE = 10.0f;
constexpr float KD_SCALE = 10.0f;
constexpr float BANG_LEVEL_SCALE = 100.0f;

constexpr byte COMMAND_FRAME_LENGTH = 4;

// ---- Telemetry (rover -> PC) ----
constexpr byte MSG_TELEMETRY = 0x20;
constexpr byte TELEMETRY_FRAME_LENGTH = 11;
constexpr byte MSG_TUNING = 0x21;         // The values the rover is using right now
constexpr byte TUNING_FRAME_LENGTH = 12;

// Status byte bit layout
constexpr byte STATUS_RUNNING = 0x01;
constexpr byte STATUS_LINE_LOST = 0x02;
constexpr byte STATUS_CALIBRATING = 0x04;
constexpr byte STATUS_LINK_FAILSAFE = 0x08;
constexpr byte STATUS_SENSOR_TEST = 0x80;  // Bit 7, above the mode bits
constexpr byte STATUS_MODE_SHIFT = 4;     // Mode lives in bits 4-6

// Sensors byte bit layout: 1 = sensor is over the white line, 0 = over black
constexpr byte SENSOR_LEFT_WHITE = 0x01;
constexpr byte SENSOR_RIGHT_WHITE = 0x02;

// ---- Driving modes ----
enum Mode : byte {
  MODE_CLOCKWISE = 0,
  MODE_ANTICLOCKWISE = 1,
  MODE_SQUIGGLE = 2,
  MODE_REVERSE = 3,
  MODE_COUNT = 4
};

// ---- Steering controllers (one per mode, see config.h) ----
enum Controller : byte {
  CONTROLLER_PID = 0,
  CONTROLLER_BANG_BANG = 1,
  CONTROLLER_ADAPTIVE = 2,
  CONTROLLER_COUNT = 3
};
