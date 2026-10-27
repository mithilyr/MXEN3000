"""A T change is resent until the rover echoes it, then given up on."""
import unittest
from types import SimpleNamespace
from unittest import mock

import protocol as p
import rover_gui as gui


class FakeLink:
    def __init__(self):
        self.sent = []

    def send(self, command, data=0):
        self.sent.append((command, data))
        return True


class FakeLabel:
    def __init__(self):
        self.text = ""

    def config(self, text):
        self.text = text


def make_gui(rover_controller, target, asked=100.0, sent=100.0):
    return SimpleNamespace(tuning=SimpleNamespace(controller=rover_controller),
                           controller_target=target, controller_asked=asked,
                           controller_sent=sent, link=FakeLink(), tuning_status=FakeLabel())


def keep_at(app, now):
    with mock.patch.object(gui.time, "monotonic", return_value=now):
        gui.RoverGUI._keep_controller(app)


class KeepControllerTest(unittest.TestCase):
    def test_lost_command_is_resent(self):
        app = make_gui(p.CONTROLLER_BANG_BANG, p.CONTROLLER_PID)
        keep_at(app, 100.5)
        self.assertEqual(app.link.sent, [(p.CMD_SET_CONTROLLER, p.CONTROLLER_PID)])
        self.assertEqual(app.controller_target, p.CONTROLLER_PID)

    def test_resend_is_rate_limited(self):
        app = make_gui(p.CONTROLLER_BANG_BANG, p.CONTROLLER_PID)
        keep_at(app, 100.1)
        self.assertEqual(app.link.sent, [])

    def test_echo_ends_the_resends(self):
        app = make_gui(p.CONTROLLER_PID, p.CONTROLLER_PID)
        keep_at(app, 100.5)
        self.assertIsNone(app.controller_target)
        self.assertEqual(app.link.sent, [])
        self.assertEqual(app.tuning_status.text, "Controller: PID")

    def test_gives_up_and_says_so(self):
        app = make_gui(p.CONTROLLER_BANG_BANG, p.CONTROLLER_PID)
        keep_at(app, 100.0 + gui.CONTROLLER_GIVE_UP_S + 0.1)
        self.assertIsNone(app.controller_target)
        self.assertEqual(app.link.sent, [])
        self.assertIn("did not take PID", app.tuning_status.text)


if __name__ == "__main__":
    unittest.main()
