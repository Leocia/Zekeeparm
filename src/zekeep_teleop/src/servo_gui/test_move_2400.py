"""Run with PYTHONPATH=src python3 -m servo_gui.test_move_2400; no hardware."""

import unittest
from unittest.mock import Mock, PropertyMock, call, patch

from .bus_worker import BusWorker


class Move2400Test(unittest.TestCase):
    def test_move_and_guards(self):
        worker = BusWorker(simulate=True)
        try:
            self.assertEqual(worker.move_to_2400(2, 2000, "MOVE 002 TO 2400")["error"]["code"], "not_connected")
            self.assertTrue(worker.open_port("模拟串口", 115200)["ok"])
            bus = worker._bus
            with patch.object(bus, "command", wraps=bus.command) as commands:
                for sid, duration in [(1, 2000), (True, 2000), (4, 2000), (2, True),
                                      (2, 499), (2, 10000), (2, 2000.5), (2, "2000")]:
                    self.assertEqual(worker.move_to_2400(sid, duration, "MOVE 002 TO 2400")["error"]["code"], "invalid_request")
                self.assertEqual(worker.move_to_2400(2, 2000, "wrong")["error"]["code"], "confirm_mismatch")
                self.assertFalse(commands.called)
                worker.start_reading(1)
                self.assertEqual(worker.move_to_2400(2, 2000, "MOVE 002 TO 2400")["error"]["code"], "reading_active")
                worker.stop_reading(1)
                commands.reset_mock()
                for sid, duration in [(2, 2000), (3, 500), (2, 9999)]:
                    result = worker.move_to_2400(sid, duration, f"MOVE {sid:03d} TO 2400")
                    self.assertTrue(result["ok"], result)
                    self.assertEqual(result["after"], 2400)
                    self.assertTrue(result["torque_restored"])
                    self.assertEqual(commands.call_args_list[-2].args[0], f"#{sid:03d}PULR!")
                    self.assertEqual(commands.call_args_list[-1].args[0], f"#{sid:03d}P2400T{duration:04d}!")
                self.assertNotIn("#002PULK!", [call.args[0] for call in commands.call_args_list])
                self.assertEqual(bus.pwms[1], 1460)
                self.assertTrue(worker.release_torque(2)["ok"])
                self.assertEqual(commands.call_args.args[0], "#002PULK!")
                self.assertEqual(worker.release_torque(1)["error"]["code"], "invalid_request")
                self.assertEqual(worker.calibrate_zero(2, "CALIBRATE 002 TO 2400")["error"]["code"], "manual_only")
                # Exercise real-mode timing against the same fake bus; no serial port is opened.
                with patch.object(worker, "_simulate", False), patch.object(worker, "_command_wait", return_value="#OK!"), patch("servo_gui.bus_worker.time.sleep") as sleep:
                    self.assertTrue(worker.move_to_2400(2, 2000, "MOVE 002 TO 2400")["ok"])
                    sleep.assert_called_once_with(2.1)
        finally:
            worker.shutdown()

    def test_delayed_fragmented_torque_ack(self):
        worker = BusWorker(simulate=True)
        try:
            worker.open_port("模拟串口", 115200)
            bus = worker._bus
            serial_port = Mock()
            send = bus.command
            # The acknowledgement arrives after two polling intervals (>30 ms), in two chunks.
            type(serial_port).in_waiting = PropertyMock(side_effect=[0, 0, 2, 2, 0, 0, 2, 2])
            serial_port.read.side_effect = [b"#O", b"K!", b"#O", b"K!"]
            with patch.object(bus, "serial", serial_port, create=True), \
                    patch.object(bus, "terminator", "\r\n", create=True), \
                    patch.object(worker, "_simulate", False), \
                    patch("servo_gui.bus_worker.time.sleep") as sleep, \
                    patch.object(bus, "command", side_effect=lambda command: "" if command.endswith("PULR!") else send(command)) as commands:
                result = worker.move_to_2400(3, 5000, "MOVE 003 TO 2400")
                self.assertTrue(result["ok"], result)
                commands.assert_called_once_with("#003P2400T5000!")
                self.assertIn(call(5.1), sleep.call_args_list)
                self.assertTrue(worker.release_torque(3)["ok"])
                self.assertEqual(serial_port.write.call_args_list,
                                 [call(b"#003PULR!\r\n"), call(b"#003PULK!\r\n")])
        finally:
            worker.shutdown()

    def test_failed_confirmation_and_readback(self):
        worker = BusWorker(simulate=True)
        try:
            worker.open_port("模拟串口", 115200)
            bus = worker._bus
            with patch.object(bus, "command", return_value="") as commands:
                self.assertEqual(worker.move_to_2400(2, 2000, "MOVE 002 TO 2400")["error"]["code"], "torque_restore_failed")
                self.assertEqual(commands.call_args_list, [call("#002PULR!"), call("#002PVER!")])
                self.assertEqual(worker.release_torque(2)["error"]["code"], "torque_release_failed")
            with patch.object(bus, "read_pwm", side_effect=[2360, 2300]):
                result = worker.move_to_2400(2, 2000, "MOVE 002 TO 2400")
                self.assertEqual(result["error"]["code"], "pwm_out_of_tolerance")
                self.assertEqual(result["after"], 2300)
                self.assertTrue(result["torque_restored"])
            with patch.object(bus, "read_pwm", return_value=None), patch.object(bus, "command") as commands:
                self.assertEqual(worker.move_to_2400(3, 2000, "MOVE 003 TO 2400")["error"]["code"], "no_response")
                self.assertFalse(commands.called)
        finally:
            worker.shutdown()

    def test_verified_firmware_without_torque_ack(self):
        worker = BusWorker(simulate=True)
        try:
            worker.open_port("模拟串口", 115200)
            bus = worker._bus
            send = bus.command
            def command(message):
                if message == "#003PULR!":
                    return ""
                if message == "#003PVER!":
                    return "#003@ ZL-ZServo_AD_CBM V2.1.16STG!\r\n"
                return send(message)
            with patch.object(bus, "command", side_effect=command):
                result = worker.move_to_2400(3, 5000, "MOVE 003 TO 2400")
                self.assertTrue(result["ok"], result)
                self.assertEqual(result["after"], 2400)
                self.assertFalse(result["torque_acknowledged"])
                self.assertIsNone(result["torque_restored"])
                for after, expected in [(2125, "pwm_out_of_tolerance"), (None, "pwm_out_of_tolerance")]:
                    with patch.object(bus, "read_pwm", side_effect=[2046, 2046, after]):
                        self.assertEqual(worker.move_to_2400(3, 5000, "MOVE 003 TO 2400")["error"]["code"], expected)
                with patch.object(bus, "read_pwm", side_effect=[2046, None]):
                    self.assertEqual(worker.move_to_2400(3, 5000, "MOVE 003 TO 2400")["error"]["code"], "torque_restore_failed")
            for response in ["", "#003@ ZL-ZServo_AD_CBM V2.1.17STG!", "#ERROR!", "#002@ ZL-ZServo_AD_CBM V2.1.16STG!"]:
                with patch.object(worker, "_command_wait", side_effect=["", response]), patch.object(bus, "command") as commands:
                    self.assertEqual(worker.move_to_2400(3, 5000, "MOVE 003 TO 2400")["error"]["code"], "torque_restore_failed")
                    self.assertFalse(commands.called)
        finally:
            worker.shutdown()


if __name__ == "__main__":
    unittest.main()
