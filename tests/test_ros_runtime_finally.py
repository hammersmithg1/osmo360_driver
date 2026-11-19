"""Final worker cleanup preserves native ownership and closes public Camera."""

import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from osmo360_camera import Camera
from osmo360_driver.runtime import OsmoCamera


class RuntimeFinallyTests(unittest.TestCase):
    def owner(self, *, managed=False, receipt=None, uncertain=False):
        owner = OsmoCamera.__new__(OsmoCamera)
        owner.stopping = threading.Event()
        owner.stopping.set()  # Exercise real run.finally without worker timing.
        owner.usb_charging = False if managed else None
        owner._power_lease = (SimpleNamespace(receipt=receipt, uncertain=uncertain)
                              if managed else None)
        owner._close_power_error = None
        owner.power = dict(requested=owner.usb_charging, state='prepared',
                           ready=True, receipt='original policy preserved')
        owner.error = ''
        owner.on_error = Mock()
        self.events = []
        owner.close_stream = Mock(side_effect=lambda: self.events.append('stream_close'))
        owner.restore_power = Mock(side_effect=lambda: self.events.append('power_restore'))
        owner.camera = SimpleNamespace(close=Mock(side_effect=lambda: self.events.append('camera_close')))
        return owner

    def fail(self, stage, error):
        def failure():
            self.events.append(stage)
            raise error
        return failure

    def test_deferred_stream_cleanup_still_attempts_public_camera_close_without_restore(self):
        owner = self.owner()
        error = RuntimeError('Retained reset uncertain; explicit recovery required')
        owner.close_stream.side_effect = self.fail('stream_close', error)
        owner.run()
        self.assertEqual(self.events, ['stream_close', 'camera_close'])
        owner.close_stream.assert_called_once_with()
        owner.restore_power.assert_not_called()
        owner.camera.close.assert_called_once_with()
        owner.on_error.assert_called_once_with(str(error))
        self.assertIsNone(owner._close_power_error)

    def test_secondary_camera_failure_cannot_mask_original_or_restore_unknown_policy(self):
        owner = self.owner(managed=True, receipt={'original': [0, 0, 1000000]})
        original = RuntimeError('stream cleanup deferred')
        secondary = RuntimeError('USB control busy; local close deferred')
        owner.close_stream.side_effect = self.fail('stream_close', original)
        owner.camera.close.side_effect = self.fail('camera_close', secondary)
        owner.run()
        self.assertEqual(self.events, ['stream_close', 'camera_close'])
        owner.restore_power.assert_not_called()
        self.assertIs(owner._close_power_error, original)
        self.assertEqual(owner.error, str(original))
        owner.on_error.assert_called_once_with(str(original))
        self.assertTrue(any(str(secondary) in note for note in original.__notes__))
        self.assertFalse(owner.power['ready'])
        self.assertEqual(owner.power['state'], 'unknown')
        self.assertEqual(owner.power['receipt'], 'original policy preserved')

    def test_real_public_camera_close_retires_local_transport_but_retains_uncertain_owner(self):
        owner = self.owner()
        error = RuntimeError('Retained reset uncertain')
        stream = SimpleNamespace(close=Mock(side_effect=error), cleanup_safe=False)
        camera = Camera.__new__(Camera)
        camera._usb_stream = stream
        camera._control_lock = threading.RLock()
        camera._transport = SimpleNamespace(close=Mock(
            side_effect=lambda: self.events.append('local_transport_close')))
        camera.close = Mock(wraps=camera.close)
        owner.camera = camera
        owner.close_stream.side_effect = self.fail('stream_close', error)
        owner.run()
        self.assertEqual(self.events, ['stream_close', 'local_transport_close'])
        owner.restore_power.assert_not_called()
        camera.close.assert_called_once_with()
        camera._transport.close.assert_called_once_with()
        self.assertIs(camera._usb_stream, stream)
        self.assertEqual(owner.error, str(error))

    def test_restore_failure_still_closes_camera_and_preserves_uncertain_lease(self):
        owner = self.owner(managed=True, uncertain=True)
        error = RuntimeError('Native charging restore result unknown')
        owner.restore_power.side_effect = self.fail('power_restore', error)
        owner.run()
        self.assertEqual(self.events, ['stream_close', 'power_restore', 'camera_close'])
        owner.restore_power.assert_called_once_with()
        owner.camera.close.assert_called_once_with()
        self.assertIs(owner._close_power_error, error)
        self.assertEqual(owner.power['state'], 'unknown')
        self.assertFalse(owner.power['ready'])

    def test_success_keeps_stream_restore_camera_order_and_reports_no_error(self):
        owner = self.owner(managed=True, receipt={'original': [0, 0, 1000000]})
        owner.run()
        self.assertEqual(self.events, ['stream_close', 'power_restore', 'camera_close'])
        owner.close_stream.assert_called_once_with()
        owner.restore_power.assert_called_once_with()
        owner.camera.close.assert_called_once_with()
        owner.on_error.assert_not_called()
        self.assertIsNone(owner._close_power_error)

    def test_camera_close_failure_is_primary_when_stream_and_restore_succeeded(self):
        owner = self.owner(managed=True, receipt={'original': [0, 0, 1000000]})
        error = RuntimeError('Local USB reader did not join')
        owner.camera.close.side_effect = self.fail('camera_close', error)
        owner.run()
        self.assertEqual(self.events, ['stream_close', 'power_restore', 'camera_close'])
        self.assertIs(owner._close_power_error, error)
        owner.on_error.assert_called_once_with(str(error))

    def test_never_created_camera_needs_no_public_close(self):
        owner = self.owner()
        owner.camera = None
        owner.run()
        self.assertEqual(self.events, ['stream_close', 'power_restore'])
        owner.on_error.assert_not_called()

    def test_base_exception_still_closes_camera_then_propagates_without_restore(self):
        owner = self.owner()
        error = KeyboardInterrupt()
        owner.close_stream.side_effect = self.fail('stream_close', error)
        with self.assertRaises(KeyboardInterrupt) as caught:
            owner.run()
        self.assertIs(caught.exception, error)
        self.assertEqual(self.events, ['stream_close', 'camera_close'])
        owner.camera.close.assert_called_once_with()
        owner.restore_power.assert_not_called()


if __name__ == '__main__':
    unittest.main()
