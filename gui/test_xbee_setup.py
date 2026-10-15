"""Checks for xbee_setup.py against simulated XBee S2C modules. Run: python test_xbee_setup.py"""

import random
import unittest

import xbee_setup as x

TEXT_SETTINGS = {"NI"}


class FakeXBee:
    """Answers AT command mode like an XBee S2C at one fixed baud rate."""

    def __init__(self, serial_low=0x4231905E, baud=9600, refuse=()):
        self.baud = baud
        self.refuse = set(refuse)
        self.flash = {"ID": 0, "CE": 0, "NI": " ", "BD": 3, "AP": 0, "DH": 0, "DL": 0xFFFF,
                      "VR": 0x4061, "HV": 0x2D55, "SH": 0x13A200, "SL": serial_low}
        self.settings = dict(self.flash)
        self.command_mode = False
        self.opened_at = []

    def open(self, _device, baud):
        self.opened_at.append(baud)
        return FakePort(self, baud == self.baud)

    def handle(self, line):
        if not self.command_mode:
            return b""
        name, value = line[2:4], line[4:]
        if name == "CN":
            self.command_mode = False
            self.baud = (1200 << self.settings["BD"]) if self.settings["BD"] <= 7 else self.baud
            return b"OK\r"
        if name == "WR":
            self.flash = dict(self.settings)
            return b"OK\r"
        if value:
            if name in self.refuse:
                return b"ERROR\r"
            self.settings[name] = value if name in TEXT_SETTINGS else int(value, 16)
            return b"OK\r"
        setting = self.settings[name]
        return (setting if name in TEXT_SETTINGS else f"{setting:X}").encode() + b"\r"


class FakePort:
    def __init__(self, xbee, baud_matches):
        self.xbee = xbee
        self.baud_matches = baud_matches
        self.pending = bytearray()
        self.out = bytearray()

    def reset_input_buffer(self):
        self.out.clear()

    def write(self, data):
        if not self.baud_matches:
            return
        if data == b"+++":
            self.xbee.command_mode = True
            self.out += b"OK\r"
            return
        self.pending += data
        while b"\r" in self.pending:
            line, _, rest = bytes(self.pending).partition(b"\r")
            self.pending = bytearray(rest)
            self.out += self.xbee.handle(line.decode())

    def read(self, size):
        data = bytes(self.out[:size])
        del self.out[:size]
        return data

    def close(self):
        pass


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def sleep(self, seconds):
        self.now += seconds

    def __call__(self):
        self.now += 0.001
        return self.now


def options(xbee):
    clock = FakeClock()
    return {"opener": xbee.open, "sleep": clock.sleep, "clock": clock, "log": lambda _t: None}


def pair(pc, rover, pan=0x1002):
    """The guide's three passes: PC, rover (gets PC serial), PC again (gets rover serial)."""
    first = x.program("COM9", x.ROLE_PC, pan, **options(pc))
    rover_result = x.program("COM9", x.ROLE_ROVER, pan, peer=x.serial_number(first),
                             **options(rover))
    pc_result = x.program("COM9", x.ROLE_PC, pan, peer=x.serial_number(rover_result),
                          **options(pc))
    return pc_result, rover_result


class XBeeSetupTests(unittest.TestCase):
    def test_first_pass_matches_guide_steps_11_to_16(self):
        xbee = FakeXBee()
        x.program("COM9", x.ROLE_PC, 0x1002, **options(xbee))
        for name, value in (("ID", 0x1002), ("CE", 0), ("NI", "PC"), ("BD", 5), ("AP", 0)):
            self.assertEqual(xbee.flash[name], value, name)
        self.assertEqual((xbee.flash["DH"], xbee.flash["DL"]), (0, 0xFFFF))  # Not set yet
        self.assertFalse(xbee.command_mode)

    def test_three_passes_point_each_at_the_other(self):
        pc, rover = FakeXBee(0x4231905E), FakeXBee(0x41B7C2D0)
        pc_result, rover_result = pair(pc, rover)
        self.assertEqual((rover.flash["DH"], rover.flash["DL"]), (0x13A200, 0x4231905E))
        self.assertEqual((pc.flash["DH"], pc.flash["DL"]), (0x13A200, 0x41B7C2D0))
        self.assertEqual(rover.flash["NI"], "ROVER")
        self.assertEqual(x.check_pair(pc_result, rover_result), [])

    def test_after_two_passes_pc_not_pointing_yet(self):
        pc, rover = FakeXBee(0x4231905E), FakeXBee(0x41B7C2D0)
        first = x.program("COM9", x.ROLE_PC, 0x1002, **options(pc))
        rover_result = x.program("COM9", x.ROLE_ROVER, 0x1002, peer=x.serial_number(first),
                                 **options(rover))
        problems = x.check_pair(first, rover_result)
        self.assertEqual(len(problems), 1)
        self.assertIn("PC DH/DL", problems[0])

    def test_finds_preconfigured_38400_first(self):
        xbee = FakeXBee(baud=38400)
        x.read_info("COM9", **options(xbee))
        self.assertEqual(xbee.opened_at, [38400])

    def test_factory_9600_moves_to_38400(self):
        xbee = FakeXBee(baud=9600)
        x.program("COM9", x.ROLE_PC, 0x1002, **options(xbee))
        self.assertEqual(xbee.opened_at[:2], [38400, 9600])
        self.assertEqual(xbee.baud, 38400)
        x.read_info("COM9", **options(xbee))        # Found again at the new rate
        self.assertEqual(xbee.opened_at[-1], 38400)

    def test_refuses_to_program_the_peer(self):
        rover = FakeXBee(0x41B7C2D0)
        with self.assertRaisesRegex(x.XBeeError, "rover XBee"):
            x.program("COM9", x.ROLE_PC, 0x1002, peer=(0x13A200, 0x41B7C2D0), **options(rover))
        self.assertEqual(rover.flash["ID"], 0)      # Nothing written

    def test_same_xbee_twice(self):
        xbee = FakeXBee()
        pc = x.program("COM9", x.ROLE_PC, 0x1002, **options(xbee))
        rover = x.program("COM9", x.ROLE_ROVER, 0x1002, **options(xbee))
        self.assertRegex(x.check_pair(pc, rover)[0], "same XBee")

    def test_refused_setting_saves_nothing(self):
        xbee = FakeXBee(refuse={"CE"})
        with self.assertRaisesRegex(x.XBeeError, "ATCE0"):
            x.program("COM9", x.ROLE_PC, 0x1002, **options(xbee))
        self.assertEqual(xbee.flash["ID"], 0)

    def test_no_xbee(self):
        with self.assertRaisesRegex(x.XBeeError, "V0 shield"):
            x.read_info("COM9", **options(FakeXBee(baud=1)))

    def test_parsing(self):
        self.assertEqual(x.parse_pan("0x1002"), 0x1002)
        for bad in ("G1", "", "0", "10000"):
            with self.assertRaises(x.XBeeError):
                x.parse_pan(bad)

    def test_random_pan_in_range(self):
        rng = random.Random(1)
        for _ in range(200):
            self.assertTrue(x.PAN_MIN <= x.random_pan(rng) <= x.PAN_MAX)


if __name__ == "__main__":
    unittest.main()
