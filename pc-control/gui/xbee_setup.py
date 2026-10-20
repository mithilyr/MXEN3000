"""Pairs two XBee S2C radios from the GUI, following the unit's
"Xbee Pairing Guide.pdf" (../../../Xbee Pairing Guide.pdf) without XCTU.

The guide's XCTU steps, as AT commands (Digi's documented command mode:
guard time of silence, "+++", wait for "OK", one "AT<cmd><value>\\r" per
setting, then one "ATWR" to save everything at once, step 17):

    step 11/19  ID  same random PAN ID on both
    step 12     CE  0
    step 13     SH / SL read and remembered (the module's serial number)
    step 14     NI  a name: "PC" or "ROVER"
    step 15     BD  5 (38400 baud)
    step 16     AP  0 (transparent mode)
    step 20/22  DH / DL = the other module's SH / SL

So it takes three passes through the USB adapter, like the guide: PC XBee,
rover XBee (gets the PC's serial), PC XBee again (gets the rover's serial).

Kept free of Tkinter so test_xbee_setup.py can drive it with a fake port.
"""

import random
import time

import serial

ROLE_PC = "pc"
ROLE_ROVER = "rover"
ROLE_NAMES = {ROLE_PC: "PC", ROLE_ROVER: "rover"}
NODE_NAMES = {ROLE_PC: "PC", ROLE_ROVER: "ROVER"}       # NI, guide step 14

LINK_BAUD = 38400               # Guide step 15; must match LINK_BAUD in config.h
LINK_BAUD_CODE = 5              # BD = 5 is 38400 baud
TRANSPARENT_MODE = 0            # AP = 0, guide step 16
COORDINATOR_OFF = 0             # CE = 0, guide step 12
# Baud rates to try when looking for the XBee: pre-configured (guide step 6),
# then factory default, then the rest.
SEARCH_BAUDS = (38400, 9600, 115200, 57600, 19200, 4800, 2400, 1200)
PAN_MIN = 1                     # ID = 0 makes a ZigBee module pick any PAN
PAN_MAX = 0xFFFF                # The guide's random IDs are this size (e.g. 1002)

GUARD_TIME_S = 1.1              # Factory GT is 1 s of silence around "+++"
OK_TIMEOUT_S = 2.0              # "OK" comes one guard time after "+++"
COMMAND_TIMEOUT_S = 1.0
FLASH_WRITE_TIMEOUT_S = 3.0     # ATWR can take a moment
READ_POLL_S = 0.01

# Read back after writing, to prove the settings stuck.
CHECK_COMMANDS = ("ID", "CE", "NI", "BD", "AP", "DH", "DL")
# ZS only exists on Zigbee firmware (802.15.4 firmware answers ERROR).
IDENTITY_COMMANDS = ("VR", "SH", "SL", "ZS")
# Radio quality, read only (never written): CH channel, PL power level
# (4 = highest), PM boost mode (1 = on), SM sleep (0 = always awake), DB signal
# strength of the last packet received (-dBm, hex; lower = stronger).
RADIO_COMMANDS = ("CH", "PL", "PM", "SM", "DB")
INFO_COMMANDS = ("VR", "HV", "SH", "SL", "ZS") + CHECK_COMMANDS + RADIO_COMMANDS


class XBeeError(Exception):
    pass


def parse_pan(text):
    """'1002' -> 0x1002. XBee AT values are always hex."""
    cleaned = text.strip().upper().removeprefix("0X")
    try:
        value = int(cleaned, 16)
    except ValueError:
        raise XBeeError(f"PAN ID must be hex (0-9, A-F), not '{text}'.") from None
    if not PAN_MIN <= value <= PAN_MAX:
        raise XBeeError(f"PAN ID must be between {PAN_MIN:X} and {PAN_MAX:X} (hex).")
    return value


def random_pan(rng=random):
    """Guide step 11: a random PAN ID, so another team's XBees can't join."""
    return rng.randint(0x0100, PAN_MAX)


def serial_number(settings):
    """(SH, SL) as ints from a read-back dict."""
    return int(settings["SH"], 16), int(settings["SL"], 16)


def is_zigbee(settings):
    """Zigbee firmware (like the guide's 'ZIGBEE TH Reg') needs one coordinator
    (CE = 1) to form the network; two CE = 0 routers never link. 802.15.4
    firmware talks peer to peer with CE = 0 on both, as the guide sets it."""
    return settings.get("ZS", "ERROR") != "ERROR"


def open_port(device, baud):
    """Open without pulsing DTR/RTS, so an Arduino used as a USB pass-through
    (guide step 2) is not reset. Its bootloader would drive TX (D1) for about a
    second and talk over the XBee's reply to +++."""
    port = serial.Serial(baudrate=baud, timeout=0)
    port.port = device
    port.dtr = False
    port.rts = False
    port.open()
    return port


def format_serial(serial_pair):
    return f"{serial_pair[0]:X} {serial_pair[1]:08X}"


def role_settings(role, pan, peer=None, coordinator=False):
    """The (command, value) pairs for guide steps 11-16, plus 20/22 when the
    other module's (SH, SL) is known. Values are ints, except NI (text)."""
    settings = [("ID", pan), ("CE", 1 if coordinator else COORDINATOR_OFF),
                ("NI", NODE_NAMES[role]), ("BD", LINK_BAUD_CODE), ("AP", TRANSPARENT_MODE)]
    if peer is not None:
        settings += [("DH", peer[0]), ("DL", peer[1])]
    return settings


def _value_text(value):
    return value if isinstance(value, str) else f"{value:X}"


class XBeeAT:
    """AT command mode session on one serial port.

    `opener(device, baud)` returns an open pyserial-like port; tests pass a fake.
    `log(text)` receives progress lines. `sleep` and `clock` are swappable so
    the tests don't wait for real guard times.
    """

    def __init__(self, device, log=print, opener=None, sleep=time.sleep, clock=time.monotonic):
        self.device = device
        self.log = log
        self.opener = opener or open_port
        self.sleep = sleep
        self.clock = clock
        self.port = None
        self.baud = None

    # ---- low level ----
    def _read_line(self, timeout):
        """Text up to the XBee's '\\r', or None on timeout."""
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
        """Find the XBee's baud rate and put it in command mode (guide step 9)."""
        for baud in bauds:
            self.log(f"Sending +++ at {baud} baud...")
            try:
                if self._try_enter(baud):
                    self.log(f"OK received at {baud} baud.")
                    return
            except serial.SerialException as error:
                raise XBeeError(f"Could not open {self.device}: {error}") from None
        raise XBeeError(
            "No XBee answered +++. On the V0 shield: switch on 'USB' and an empty sketch "
            "uploaded to the Arduino (guide steps 1-2). Also close XCTU and Disconnect "
            "the GUI, so nothing else holds the port.")

    def command(self, name, value=None, timeout=COMMAND_TIMEOUT_S):
        """Send ATxx (read) or ATxx<value> (write); return the XBee's reply."""
        text = f"AT{name}" + ("" if value is None else _value_text(value))
        self.port.write(text.encode("ascii") + b"\r")
        reply = self._read_line(timeout)
        if reply is None:
            raise XBeeError(f"No reply to {text}. The XBee left command mode.")
        return reply

    def set(self, name, value):
        reply = self.command(name, value)
        if reply != "OK":
            raise XBeeError(f"XBee refused AT{name}{_value_text(value)} (replied '{reply}').")

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


def read_info(device, log=print, **session_options):
    """Return the current settings of the XBee on `device`."""
    session = XBeeAT(device, log, **session_options)
    session.enter()
    try:
        return session.read(INFO_COMMANDS)
    finally:
        session.close()


def program(device, role, pan, peer=None, coordinator=False, log=print, **session_options):
    """Guide steps 9-17 (and 20/22 when `peer`, the other module's (SH, SL), is
    known). Refuses to program the peer itself. Returns the read-back settings."""
    wanted = role_settings(role, pan, peer, coordinator)
    session = XBeeAT(device, log, **session_options)
    session.enter()
    try:
        identity = session.read(IDENTITY_COMMANDS)
        own_serial = serial_number(identity)
        firmware = "Zigbee" if is_zigbee(identity) else "802.15.4"
        log(f"Firmware {identity['VR']} ({firmware}), serial number (SH SL) "
            f"{format_serial(own_serial)}.")
        if peer is not None and own_serial == tuple(peer):
            other = ROLE_NAMES[ROLE_ROVER if role == ROLE_PC else ROLE_PC]
            raise XBeeError(f"This is the {other} XBee. Swap in the "
                            f"{ROLE_NAMES[role]} XBee and try again.")
        for name, value in wanted:
            log(f"  AT{name} {_value_text(value)}")
            session.set(name, value)
        log("Writing all parameters at once (ATWR)...")
        reply = session.command("WR", timeout=FLASH_WRITE_TIMEOUT_S)
        if reply != "OK":
            raise XBeeError(f"XBee could not save its settings (ATWR replied '{reply}').")
        actual = session.read(CHECK_COMMANDS)
        actual.update(identity)
    finally:
        session.close()
    wrong = [name for name, value in wanted if not _matches(actual[name], value)]
    if wrong:
        raise XBeeError("Settings did not stick: " +
                        ", ".join(f"{name}={actual[name]}" for name in wrong))
    target = f", sends to {format_serial(peer)}" if peer is not None else \
        " (no destination yet: program the other XBee next)"
    log(f"Done: {ROLE_NAMES[role]} XBee, PAN {pan:X}, {LINK_BAUD} baud{target}.")
    if is_zigbee(actual) and role == ROLE_PC and not coordinator:
        log("Note: Zigbee firmware needs one coordinator. Program the PC XBee once more "
            "with 'PC XBee as coordinator' ticked.")
    return actual


def _matches(reply, value):
    if isinstance(value, str):
        return reply == value
    try:
        return int(reply, 16) == value
    except ValueError:
        return False


def check_pair(pc_settings, rover_settings):
    """Problems that would stop two read-back setting dicts from talking."""
    if serial_number(pc_settings) == serial_number(rover_settings):
        return ["Both were the same XBee. Swap in the other one and program it."]
    problems = []
    for name, label in (("ID", "PAN ID"), ("BD", "baud rate"), ("AP", "AP mode")):
        if not _matches(pc_settings[name], int(rover_settings[name], 16)):
            problems.append(f"Different {label}: PC {pc_settings[name]}, "
                            f"rover {rover_settings[name]}.")
    for sender, receiver, label in ((pc_settings, rover_settings, "PC"),
                                    (rover_settings, pc_settings, "Rover")):
        destination = (int(sender["DH"], 16), int(sender["DL"], 16))
        if destination != serial_number(receiver):
            problems.append(f"{label} DH/DL does not point at the other XBee yet.")
    if is_zigbee(pc_settings) and is_zigbee(rover_settings) and             1 not in (_hex_or_none(pc_settings["CE"]), _hex_or_none(rover_settings["CE"])):
        problems.append("PC XBee must be the coordinator (CE = 1): Zigbee firmware "
                        "needs one, or the two never link.")
    return problems


def _hex_or_none(text):
    try:
        return int(text, 16)
    except ValueError:
        return None
