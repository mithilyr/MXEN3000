"""Checks for xbee_setup.py against a simulated XBee. Run: python test_xbee_setup.py"""

import random
import unittest

import xbee_setup as x


class FakeXBee:
    """Answers AT command mode like an XBee S1 at one fixed baud rate."""

    def __init__(self, baud=9600, refuse=()):
        self.baud = baud
        self.refuse = set(refuse)
        self.flash = {"CH": 0xC, "ID": 0x3332, "MY": 0, "DH": 0, "DL": 0, "BD": 3, "AP": 0,
                      "VR": 0x10EF, "HV": 0x1744, "SH": 0x13A200, "SL": 0x4012ABCD}
        self.settings = dict(self.flash)
        self.command_mode = False
        self.written = []
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
            return b"OK\r"
        if name == "WR":
            self.flash = dict(self.settings)
            return b"OK\r"
        if value:
            if name in self.refuse:
                return b"ERROR\r"
            self.settings[name] = int(value, 16)
            self.written.append((name, int(value, 16)))
            return b"OK\r"
        return f"{self.settings[name]:X}\r".encode()


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


def session_options(xbee):
    clock = FakeClock()
    return {"opener": xbee.open, "sleep": clock.sleep, "clock": clock}


class XBeeSetupTests(unittest.TestCase):
    def test_program_pc(self):
        xbee = FakeXBee()
        x.program("COM9", x.ROLE_PC, 0x2026, 0xC, log=lambda _t: None, **session_options(xbee))
        for name, value in (("ID", 0x2026), ("CH", 0xC), ("MY", 0), ("DL", 1), ("BD", 3)):
            self.assertEqual(xbee.flash[name], value, name)
        self.assertFalse(xbee.command_mode)

    def test_program_rover(self):
        xbee = FakeXBee()
        x.program("COM9", x.ROLE_ROVER, 0x2026, 0xC, log=lambda _t: None, **session_options(xbee))
        self.assertEqual((xbee.flash["MY"], xbee.flash["DL"]), (1, 0))

    def test_finds_other_baud_and_resets_to_9600(self):
        xbee = FakeXBee(baud=115200)
        x.program("COM9", x.ROLE_PC, 0x2026, 0xC, log=lambda _t: None, **session_options(xbee))
        self.assertEqual(xbee.opened_at[:2], [9600, 115200])
        self.assertEqual(xbee.flash["BD"], 3)

    def test_no_xbee(self):
        xbee = FakeXBee(baud=1)
        with self.assertRaises(x.XBeeError):
            x.read_info("COM9", log=lambda _t: None, **session_options(xbee))

    def test_zigbee_firmware_explained(self):
        xbee = FakeXBee(refuse={"CH", "MY"})
        with self.assertRaisesRegex(x.XBeeError, "802.15.4"):
            x.program("COM9", x.ROLE_PC, 0x2026, 0xC, log=lambda _t: None,
                      **session_options(xbee))
        self.assertNotEqual(xbee.flash["ID"], 0x2026)   # Nothing saved

    def test_read_info(self):
        info = x.read_info("COM9", log=lambda _t: None, **session_options(FakeXBee()))
        self.assertEqual(info["ID"], "3332")
        self.assertEqual(info["VR"], "10EF")

    def test_check_pair(self):
        pc = {"CH": "C", "ID": "2026", "MY": "0", "DL": "1", "BD": "3"}
        rover = {"CH": "C", "ID": "2026", "MY": "1", "DL": "0", "BD": "3"}
        self.assertEqual(x.check_pair(pc, rover), [])
        self.assertEqual(len(x.check_pair(pc, dict(rover, ID="2027", MY="0"))), 2)

    def test_check_pair_same_xbee_twice(self):
        xbee = FakeXBee()
        options = session_options(xbee)
        pc = x.program("COM9", x.ROLE_PC, 0x2026, 0xC, log=lambda _t: None, **options)
        rover = x.program("COM9", x.ROLE_ROVER, 0x2026, 0xC, log=lambda _t: None, **options)
        self.assertRegex(x.check_pair(pc, rover)[0], "same XBee")

    def test_parsing(self):
        self.assertEqual(x.parse_pan("0x2026"), 0x2026)
        self.assertEqual(x.parse_channel("c"), 0xC)
        for bad in ("G1", "", "10000"):
            with self.assertRaises(x.XBeeError):
                x.parse_pan(bad)
        with self.assertRaises(x.XBeeError):
            x.parse_channel("1B")

    def test_random_pan_avoids_factory(self):
        rng = random.Random(1)
        for _ in range(200):
            self.assertNotEqual(x.random_pan(rng), x.FACTORY_PAN)


if __name__ == "__main__":
    unittest.main()
