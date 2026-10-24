"""The GUI keeps the session's mode when the rover comes back in another one."""
import unittest
from types import SimpleNamespace

import protocol as p
import rover_gui as gui


class FakeLink:
    def __init__(self):
        self.sent = []

    def send(self, command, data=0):
        self.sent.append((command, data))
        return True


class FakeVar:
    def __init__(self, value):
        self.value = value

    def get(self):
        return self.value


def make_gui(selected):
    return SimpleNamespace(selected_mode=FakeVar(selected), link=FakeLink(), mode_sent=0.0)


def telemetry(mode, running=False):
    return SimpleNamespace(mode=mode, running=running)


class KeepSessionModeTest(unittest.TestCase):
    def test_rover_reset_to_clockwise_gets_session_mode_back(self):
        app = make_gui(p.MODE_SQUIGGLE)
        gui.RoverGUI._keep_session_mode(app, telemetry(p.MODE_CLOCKWISE))
        self.assertEqual(app.link.sent, [(p.CMD_SET_MODE, p.MODE_SQUIGGLE)])
        self.assertEqual(app.selected_mode.get(), p.MODE_SQUIGGLE)

    def test_resend_is_rate_limited(self):
        app = make_gui(p.MODE_SQUIGGLE)
        gui.RoverGUI._keep_session_mode(app, telemetry(p.MODE_CLOCKWISE))
        gui.RoverGUI._keep_session_mode(app, telemetry(p.MODE_CLOCKWISE))
        self.assertEqual(len(app.link.sent), 1)

    def test_nothing_sent_when_modes_match_or_running(self):
        app = make_gui(p.MODE_SQUIGGLE)
        gui.RoverGUI._keep_session_mode(app, telemetry(p.MODE_SQUIGGLE))
        gui.RoverGUI._keep_session_mode(app, telemetry(p.MODE_CLOCKWISE, running=True))
        self.assertEqual(app.link.sent, [])


if __name__ == "__main__":
    unittest.main()
