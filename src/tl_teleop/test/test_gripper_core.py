"""Offline serial protocol and asynchronous input tests; no hardware access."""

import threading
import unittest
from unittest.mock import patch

from tl_teleop_gripper.claw import CheckedClaw, vendor
from tl_teleop_gripper.controller import GripperController


class FakeClock:
    def __init__(self):
        self.now = 10.

    def __call__(self):
        return self.now


class FakeClaw:
    def __init__(self, hook=lambda operation: None):
        self.hook = hook
        self.operations = []
        self.motion_allowed = False
        self.state = dict(position=0, torque=10, is_done=True, calibrated=True)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        pass

    def get_state(self):
        self.hook('read')
        return dict(self.state)

    def set_force(self, value):
        self.operations.append(('force', value))
        self.hook('force')

    def set_speed(self, value, save):
        if save:
            raise AssertionError('Persistent setting requested')
        self.operations.append(('speed', value))
        self.hook('speed')

    def move(self, value, wait):
        if wait or not self.motion_allowed:
            raise AssertionError('Unexpected blocking or disabled movement')
        self.operations.append(('move', value))
        self.hook('move')


class GripperInputTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.controller = GripperController('/fake', clock=self.clock)
        self.controller._running = True
        self.claw = FakeClaw()

    def cycle(self):
        self.controller._service_cycle(self.claw)

    def test_engagement_with_released_trigger_does_not_open_existing_load(self):
        self.controller.update_input(0., True)
        self.cycle()
        self.assertEqual(self.claw.operations, [])
        self.assertTrue(self.controller.snapshot()['ready'])

    def test_trigger_closes_then_opens_without_repeating_same_target(self):
        self.controller.update_input(1., True)
        self.cycle()
        self.assertEqual(self.claw.operations, [('force', 10), ('speed', 20), ('move', 0)])
        self.controller.update_input(1., True)
        self.cycle()
        self.controller.update_input(0., True)
        self.cycle()
        self.controller.update_input(0., True)
        self.cycle()
        self.assertEqual(self.claw.operations[-1], ('move', 100))
        self.assertEqual(self.controller.snapshot()['commands_sent'], 2)

    def test_releasing_grip_never_opens_and_cancels_pending_target(self):
        self.controller.update_input(1., True)
        self.controller.update_input(0., False)
        self.cycle()
        self.assertEqual(self.claw.operations, [])
        self.controller.update_input(0., True)
        self.cycle()
        self.assertEqual(self.claw.operations, [])

    def test_stale_input_during_state_read_is_not_executed(self):
        self.controller.update_input(1., True)
        self.claw.hook = lambda operation: setattr(self.clock, 'now', self.clock.now + .21)
        self.cycle()
        self.assertEqual(self.claw.operations, [])

    def test_source_age_is_preserved_instead_of_freshened_by_callback(self):
        self.controller.update_input(1., True, age_s=.19)
        self.clock.now += .02
        self.cycle()
        self.assertEqual(self.claw.operations, [])

    def test_input_expiring_during_force_configuration_blocks_later_writes(self):
        self.controller.update_input(1., True)
        self.claw.hook = lambda operation: setattr(self.clock, 'now', self.clock.now + (.21 if operation == 'force' else 0.))
        self.cycle()
        self.assertEqual(self.claw.operations, [('force', 10)])
        self.assertFalse(self.claw.motion_allowed)

    def test_grip_release_during_speed_configuration_blocks_movement(self):
        self.controller.update_input(1., True)
        self.claw.hook = lambda operation: self.controller.update_input(0., False) if operation == 'speed' else None
        self.cycle()
        self.assertEqual(self.claw.operations, [('force', 10), ('speed', 20)])

    def test_newest_trigger_intent_wins_after_slow_configuration(self):
        self.controller.update_input(1., True)
        self.claw.hook = lambda operation: self.controller.update_input(0., True) if operation == 'speed' else None
        self.cycle()
        self.assertEqual(self.claw.operations[-1], ('move', 100))

    def test_invalid_input_cancels_pending_command(self):
        for trigger, age in [(float('nan'), 0.), (2., 0.), (0., float('nan')), (0., -.01), (0., .2)]:
            self.controller.update_input(1., True)
            self.controller.update_input(trigger, True, age)
            self.cycle()
            self.assertEqual(self.claw.operations, [])

    def test_uncalibrated_worker_fault_latches_without_writes_or_auto_homing(self):
        claw = FakeClaw()
        claw.state['calibrated'] = False
        controller = GripperController('/fake', claw_factory=lambda **_: claw)
        controller.update_input(1., True)
        controller.start()
        controller._thread.join(timeout=1.)
        self.assertFalse(controller.snapshot()['ready'])
        self.assertIn('not calibrated', controller.snapshot()['error'])
        self.assertFalse(controller.snapshot()['state']['calibrated'])
        self.assertEqual(claw.operations, [])
        controller.update_input(1., True)
        self.assertIsNone(controller._current_goal())
        with self.assertRaises(RuntimeError):
            controller.start()
        controller.close()

    def test_disconnection_fault_latches(self):
        def disconnected(**_):
            raise OSError('serial disconnected')
        controller = GripperController('/fake', claw_factory=disconnected)
        controller.start()
        controller._thread.join(timeout=1.)
        self.assertEqual(controller.snapshot()['error'], 'serial disconnected')
        self.assertFalse(controller.snapshot()['ready'])
        controller.close()

    def test_input_callback_remains_responsive_while_serial_read_blocks(self):
        reading, resume, updated = threading.Event(), threading.Event(), threading.Event()

        def blocking_read(operation):
            if operation == 'read':
                reading.set()
                if not resume.wait(1.):
                    raise TimeoutError('test did not resume')

        claw = FakeClaw(blocking_read)
        controller = GripperController('/fake', claw_factory=lambda **_: claw)
        controller.start()
        try:
            self.assertTrue(reading.wait(1.))
            def callback():
                controller.update_input(0., False)
                updated.set()
            callback_thread = threading.Thread(target=callback)
            callback_thread.start()
            self.assertTrue(updated.wait(.2), 'input callback blocked on serial I/O')
            callback_thread.join(timeout=1.)
        finally:
            resume.set()
            controller.close()
        self.assertEqual(claw.operations, [])

    def test_stale_feedback_does_not_advertise_ready(self):
        self.cycle()
        self.clock.now += 1.
        self.assertFalse(self.controller.snapshot()['ready'])

    def test_close_cancels_intent_before_worker_resumes_from_blocking_read(self):
        reading, resume = threading.Event(), threading.Event()

        def blocking_read(operation):
            if operation == 'read':
                reading.set()
                if not resume.wait(1.):
                    raise TimeoutError('test did not resume')

        claw = FakeClaw(blocking_read)
        controller = GripperController('/fake', claw_factory=lambda **_: claw)
        controller.update_input(1., True)
        controller.start()
        self.assertTrue(reading.wait(1.))
        closer = threading.Thread(target=controller.close)
        closer.start()
        self.assertTrue(controller._stop.wait(1.))
        resume.set()
        closer.join(timeout=1.)
        self.assertFalse(closer.is_alive())
        self.assertFalse(controller.snapshot()['running'])
        self.assertFalse(controller.snapshot()['ready'])
        self.assertEqual(claw.operations, [])


class FakeSerial:
    def __init__(self, response):
        self.response = response
        self.writes = []

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def flush(self):
        pass

    def read(self, _length):
        return self.response


class GripperProtocolTests(unittest.TestCase):
    def setUp(self):
        self.claw = object.__new__(CheckedClaw)
        self.claw._slave_id = 1
        self.claw._debug = False
        self.claw.motion_allowed = False
        self.claw.packet_logger = None

    def response(self, body):
        return body + self.claw._crc16(body)

    def test_guard_blocks_motion_homing_and_persistent_writes_before_serial(self):
        with patch.object(vendor.LebaiClaw, '_send') as transport:
            with self.assertRaises(PermissionError):
                self.claw.move(0, wait=False)
            self.claw.motion_allowed = True
            for call in (lambda: self.claw.init(10, 20), lambda: self.claw.set_speed(20, save=True)):
                with self.assertRaises(PermissionError):
                    call()
            for call in (lambda: self.claw.set_force(21), lambda: self.claw.set_speed(31, save=False),
                         lambda: self.claw.move(50, wait=False)):
                with self.assertRaises(ValueError):
                    call()
            transport.assert_not_called()

    def test_write_ack_address_register_and_count_are_verified(self):
        self.claw.motion_allowed = True
        for body in (bytes.fromhex('02 10 9c 40 00 01'), bytes.fromhex('01 10 9c 41 00 01'),
                     bytes.fromhex('01 10 9c 40 00 02')):
            self.claw._ser = FakeSerial(self.response(body))
            with patch.object(vendor.time, 'sleep'), self.assertRaises(ValueError):
                self.claw.move(0, wait=False)

    def test_crc_error_and_truncated_response_are_rejected(self):
        good = self.response(bytes.fromhex('01 03 02 00 64'))
        for response, error_type in ((good[:-1] + bytes([good[-1] ^ 1]), ValueError), (good[:3], TimeoutError)):
            self.claw._ser = FakeSerial(response)
            with patch.object(vendor.time, 'sleep'), self.assertRaises(error_type):
                self.claw.get_position()

    def test_valid_feedback_and_motion_use_original_sdk_wire_protocol(self):
        self.claw._ser = FakeSerial(self.response(bytes.fromhex('01 03 02 00 64')))
        with patch.object(vendor.time, 'sleep'):
            self.assertEqual(self.claw.get_position(), 100)
        self.assertEqual(self.claw._ser.writes[0], self.response(bytes.fromhex('01 03 9c 45 00 01')))
        self.claw.motion_allowed = True
        self.claw._ser = FakeSerial(self.response(bytes.fromhex('01 10 9c 40 00 01')))
        with patch.object(vendor.time, 'sleep'):
            self.claw.move(0, wait=False)
        self.assertEqual(self.claw._ser.writes[0], self.response(bytes.fromhex('01 10 9c 40 00 01 02 00 00')))


if __name__ == '__main__':
    unittest.main()
