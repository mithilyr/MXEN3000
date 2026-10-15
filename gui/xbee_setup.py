"""Pairs two XBee S1 (802.15.4) radios from the GUI, so XCTU is not needed.

Uses Digi's documented AT command mode: guard time of silence, "+++", wait
for "OK", then one "AT<cmd><value>\\r" per setting, "ATWR" to save to flash
and "ATCN" to leave. Each XBee is programmed in turn through the PC's USB
adapter: the PC one as MY=0 / DL=1, the rover one as MY=1 / DL=0, both on
the same channel and PAN ID (the table in ../README.md "XBee setup").

Kept free of Tkinter so test_xbee_setup.py can drive it with a fake port.
"""

import random
import time

import serial

ROLE_PC = "pc"
ROLE_ROVER = "rover"
ROLE_NAMES = {ROLE_PC: "PC", ROLE_ROVER: "rover"}
ROLE_ADDRESSES = {ROLE_PC: (0, 1), ROLE_ROVER: (1, 0)}   # role: (MY, DL)

LINK_BAUD_CODE = 3              # BD = 3 is 9600 baud, the rover's LINK_BAUD
TRANSPARENT_MODE = 0            # AP = 0
# Baud rates to try when looking for the XBee, link rate first.
SEARCH_BAUDS = (9600, 115200, 57600, 38400, 19200, 4800, 2400, 1200)
CHANNEL_MIN = 0x0B              # XBee S1. XBee-PRO S1 only allows 0x0C-0x17.
CHANNEL_MAX = 0x1A
PAN_MAX = 0xFFFF
FACTORY_PAN = 0x3332            # Every team's XBee starts on this, so avoid it

GUARD_TIME_S = 1.1              # Factory GT is 1 s of silence around "+++"
OK_TIMEOUT_S = 2.0              # "OK" comes one guard time after "+++"
COMMAND_TIMEOUT_S = 1.0
FLASH_WRITE_TIMEOUT_S = 3.0     # ATWR can take a moment
READ_POLL_S = 0.01

# Read back after writing, to prove the settings stuck.
CHECK_COMMANDS = ("CH", "ID", "MY", "DH", "DL", "BD", "AP")
INFO_COMMANDS = ("VR", "HV", "SH", "SL") + CHECK_COMMANDS


class XBeeError(Exception):
    pass


def parse_hex(text, name, low, high):
    """'2026' -> 0x2026. XBee AT values are always hex."""
    cleaned = text.strip().upper().removeprefix("0X")
    try:
        value = int(cleaned, 16)
    except ValueError:
        raise XBeeError(f"{name} must be hex (0-9, A-F), not '{text}'.") from None
    if not low <= value <= high:
        raise XBeeError(f"{name} must be between {low:X} and {high:X} (hex).")
    return value


def parse_pan(text):
    return parse_hex(text, "PAN ID", 0, PAN_MAX)


def parse_channel(text):
    return parse_hex(text, "Channel", CHANNEL_MIN, CHANNEL_MAX)


def random_pan(rng=random):
    """A PAN ID unlikely to clash with another team in the lab."""
    while True:
        pan = rng.randint(0x0100, 0xFFFE)
        if pan != FACTORY_PAN:
            return pan


def role_settings(role, pan, channel):
    """The (command, value) pairs that make an XBee the PC or rover end."""
    my_address, destination = ROLE_ADDRESSES[role]
    return [("CH", channel), ("ID", pan), ("MY", my_address), ("DH", 0),
            ("DL", destination), ("BD", LINK_BAUD_CODE), ("AP", TRANSPARENT_MODE)]


class XBeeAT:
    """AT command mode session on one serial port.

    `opener(device, baud)` returns an open pyserial-like port; tests pass a fake.
    `log(text)` receives progress lines. `sleep` and `clock` are swappable so
    the tests don't wait for real guard times.
    """

    def __init__(self, device, log=print, opener=None, sleep=time.sleep, clock=time.monotonic):
        self.device = device
        self.log = log
        self.opener = opener or (lambda dev, baud: serial.Serial(dev, baud, timeout=0))
        self.sleep = sleep
        self.clock = clock
        self.port = None
        self.baud = None

    # ---- low level ----
    def _read_line(self, timeout):
        """Bytes up to the XBee's '\\r', or None on timeout."""
        line = bytearray()
        deadline = self.clock() + timeout
        while self.clock() < deadline:
            data = self.port.read(1)
            if not data:
                self.sleep(READ_POLL_S)
                continue
            if data == b"\r":
                return line.decode("ascii", errors="replace").strip()
            line += data
        return None

    def _try_enter(self, baud):
        self.port = self.opener(self.device, baud)
        self.port.reset_input_buffer()
        self.sleep(GUARD_TIME_S)
        self.port.write(b"+++")
        reply = self._read_line(OK_TIMEOUT_S)
        if reply is not None and reply.endswith("OK"):
            self.baud = baud
            return True
        self.port.close()
        self.port = None
        return False

    # ---- public ----
    def enter(self, bauds=SEARCH_BAUDS):
        """Find the XBee's baud rate and put it in command mode."""
        for baud in bauds:
            self.log(f"Looking for an XBee at {baud} baud...")
            try:
                if self._try_enter(baud):
                    self.log(f"XBee answered at {baud} baud.")
                    return
            except serial.SerialException as error:
                raise XBeeError(f"Could not open {self.device}: {error}") from None
        raise XBeeError(
            "No XBee answered. Check it is seated the right way round in the USB "
            "adapter, the GUI is not connected to this port, and XCTU is closed.")

    def command(self, name, value=None, timeout=COMMAND_TIMEOUT_S):
        """Send ATxx (read) or ATxx<hex> (write); return the XBee's reply."""
        text = f"AT{name}" + ("" if value is None else f"{value:X}")
        self.port.write(text.encode("ascii") + b"\r")
        reply = self._read_line(timeout)
        if reply is None:
            raise XBeeError(f"No reply to {text}. The XBee left command mode.")
        return reply

    def set(self, name, value):
        reply = self.command(name, value)
        if reply != "OK":
            raise XBeeError(_refused_message(name, value, reply))

    def read(self, names):
        return {name: self.command(name) for name in names}

    def close(self):
        if self.port is None:
            return
        try:
            self.command("CN")      # Leave command mode; a new BD takes effect here
        except (XBeeError, serial.SerialException):
            pass
        self.port.close()
        self.port = None


def _refused_message(name, value, reply):
    message = f"XBee refused AT{name}{value:X} (replied '{reply}')."
    if name == "CH":
        message += " XBee-PRO S1 only allows channels C to 17: pick one in that range."
    if name in ("CH", "MY"):
        message += (" ZigBee (S2/S2C/XBee3 Zigbee) firmware also refuses it. The rover "
                    "needs 802.15.4 firmware: flash it once with XCTU, then pair here.")
    return message


def read_info(device, log=print, **session_options):
    """Return the current settings of the XBee on `device`."""
    session = XBeeAT(device, log, **session_options)
    session.enter()
    try:
        return session.read(INFO_COMMANDS)
    finally:
        session.close()


def program(device, role, pan, channel, log=print, **session_options):
    """Write the role's settings to flash and check them. Returns the read-back."""
    wanted = role_settings(role, pan, channel)
    session = XBeeAT(device, log, **session_options)
    session.enter()
    try:
        firmware = session.command("VR")
        log(f"Firmware version {firmware}.")
        for name, value in wanted:
            log(f"  AT{name} {value:X}")
            session.set(name, value)
        log("Saving to flash (ATWR)...")
        reply = session.command("WR", timeout=FLASH_WRITE_TIMEOUT_S)
        if reply != "OK":
            raise XBeeError(f"XBee could not save its settings (ATWR replied '{reply}').")
        actual = session.read(CHECK_COMMANDS + ("SH", "SL"))
    finally:
        session.close()
    wrong = [name for name, value in wanted if _hex_value(actual[name]) != value]
    if wrong:
        raise XBeeError("Settings did not stick: " +
                        ", ".join(f"{name}={actual[name]}" for name in wrong))
    log(f"Done: this XBee is now the {ROLE_NAMES[role]} end "
        f"(CH {channel:X}, PAN {pan:X}, MY {ROLE_ADDRESSES[role][0]}, "
        f"DL {ROLE_ADDRESSES[role][1]}, 9600 baud).")
    return actual


def _hex_value(text):
    try:
        return int(text, 16)
    except ValueError:
        return None


def check_pair(pc_settings, rover_settings):
    """Problems that would stop two read-back setting dicts from talking."""
    problems = []
    if (pc_settings.get("SH"), pc_settings.get("SL")) == \
            (rover_settings.get("SH"), rover_settings.get("SL")) and "SL" in pc_settings:
        problems.append("Both were the same XBee. Swap in the other one and program it.")
    for name, label in (("CH", "channel"), ("ID", "PAN ID"), ("BD", "baud rate")):
        if _hex_value(pc_settings[name]) != _hex_value(rover_settings[name]):
            problems.append(f"Different {label}: PC {pc_settings[name]}, "
                            f"rover {rover_settings[name]}.")
    if _hex_value(pc_settings["DL"]) != _hex_value(rover_settings["MY"]):
        problems.append("PC DL does not match the rover's MY.")
    if _hex_value(rover_settings["DL"]) != _hex_value(pc_settings["MY"]):
        problems.append("Rover DL does not match the PC's MY.")
    return problems
