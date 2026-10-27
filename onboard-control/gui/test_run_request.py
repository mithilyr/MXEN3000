"""Space START / STOP is resent until the rover confirms it; nothing else stops it."""
import os
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

    def config(self, text, **_options):
        self.text = text


def make_gui(command, rover_running, alive=True, asked=100.0):
    app = SimpleNamespace(
        run_request={"command": command, "asked": asked, "sent": asked},
        telemetry=SimpleNamespace(running=rover_running), link=FakeLink(),
        note_hint=FakeLabel(), cancelled=False)
    app.link_alive = lambda: alive
    app._cancel_run = lambda: setattr(app, "cancelled", True)
    return app


def keep_at(app, now):
    with mock.patch.object(gui.time, "monotonic", return_value=now):
        gui.RoverGUI._keep_run_request(app)


class KeepRunRequestTest(unittest.TestCase):
    def test_lost_start_is_resent(self):
        app = make_gui(p.CMD_START, rover_running=False)
        keep_at(app, 100.3)
        self.assertEqual(app.link.sent, [(p.CMD_START, 0)])

    def test_lost_stop_is_resent_even_without_telemetry(self):
        app = make_gui(p.CMD_STOP, rover_running=True, alive=False)
        keep_at(app, 100.3)
        self.assertEqual(app.link.sent, [(p.CMD_STOP, 0)])

    def test_confirmation_ends_the_resends(self):
        app = make_gui(p.CMD_START, rover_running=True)
        keep_at(app, 100.3)
        self.assertIsNone(app.run_request)
        self.assertEqual(app.link.sent, [])

    def test_unconfirmed_start_gives_up_and_cancels_the_run(self):
        app = make_gui(p.CMD_START, rover_running=False)
        keep_at(app, 100.0 + gui.RUN_GIVE_UP_S + 0.1)
        self.assertIsNone(app.run_request)
        self.assertTrue(app.cancelled)
        self.assertIn("did not confirm START", app.note_hint.text)

    def test_no_request_sends_nothing(self):
        app = make_gui(p.CMD_START, rover_running=False)
        app.run_request = None
        keep_at(app, 105.0)
        self.assertEqual(app.link.sent, [])


class OnlySpaceStopsTest(unittest.TestCase):
    def test_no_stop_outside_space(self):
        """CMD_STOP is sent from one line only, in stop_run (the Space path)."""
        folder = os.path.dirname(gui.__file__)
        lines = []
        for name in os.listdir(folder):
            if name.endswith(".py") and not name.startswith("test_"):
                with open(os.path.join(folder, name), encoding="utf-8") as source:
                    lines += [line.strip() for line in source if "p.CMD_STOP" in line]
        self.assertEqual(lines, ["self._request_run(p.CMD_STOP)"])


if __name__ == "__main__":
    unittest.main()
