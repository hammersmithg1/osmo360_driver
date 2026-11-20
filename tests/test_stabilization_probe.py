"""A candidate parameter GET must never become an EIS-off claim or a SET."""

import json
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from osmo360_camera import Camera
from osmo360_camera.controls import CameraReplyError, ControlDecodeError
from osmo360_driver.stabilization_probe import (
    decode_candidate_reply, probe_stabilization_candidate,
)


class StabilizationProbeTests(unittest.TestCase):
    def camera(self, payload):
        transport = SimpleNamespace(request=Mock(return_value=SimpleNamespace(payload=payload)))
        return Camera(transport=transport), transport

    def assert_unverified(self, report):
        self.assertFalse(report['model_semantics_verified'])
        self.assertFalse(report['video_effect_verified'])
        self.assertFalse(report['onboard_stabilization_disabled_verified'])
        self.assertEqual(json.loads(json.dumps(report)), report)

    def assert_one_get(self, transport):
        transport.request.assert_called_once_with(
            2, 0x8e, bytes.fromhex('00010800'), receiver=1, timeout=2.0)

    def test_valid_zero_is_only_an_unverified_candidate_off_label(self):
        camera, transport = self.camera(bytes.fromhex('00000108000100'))
        report = probe_stabilization_candidate(camera)
        self.assertTrue(report['wire_reply_valid'])
        self.assertEqual(report['parameter_value'], 0)
        self.assertEqual(report['candidate_action6_label'], 'Off')
        self.assertEqual(report['response_hex'], '00000108000100')
        self.assert_unverified(report)
        self.assert_one_get(transport)

    def test_other_known_modes_and_unknown_values_remain_unverified(self):
        for value, label in [(1, 'RockSteady'), (2, 'HorizonSteady'),
                             (3, 'RockSteady+'), (4, 'HorizonBalancing'), (255, None)]:
            with self.subTest(value=value):
                camera, transport = self.camera(bytes.fromhex('000001080001') + bytes([value]))
                report = probe_stabilization_candidate(camera)
                self.assertTrue(report['wire_reply_valid'])
                self.assertEqual(report['parameter_value'], value)
                self.assertEqual(report['candidate_action6_label'], label)
                self.assert_unverified(report)
                self.assert_one_get(transport)

    def test_malformed_or_other_parameter_replies_never_mean_off(self):
        replies = ['', '00', '000001080001', '0000010800010000',
                   '00010108000100', '00000208000100', '0000013f000100',
                   '00000108000200', '00000100080100']
        for raw in replies:
            with self.subTest(reply=raw):
                payload = bytes.fromhex(raw)
                with self.assertRaises(ControlDecodeError):
                    decode_candidate_reply(payload)
                camera, transport = self.camera(payload)
                report = probe_stabilization_candidate(camera)
                self.assertFalse(report['wire_reply_valid'])
                self.assertIsNone(report['parameter_value'])
                self.assertIsNone(report['candidate_action6_label'])
                self.assertEqual(report['response_hex'], raw)
                self.assertEqual(report['error']['type'], 'ControlDecodeError')
                self.assert_unverified(report)
                self.assert_one_get(transport)

    def test_camera_rejection_is_preserved_and_not_retried(self):
        with self.assertRaises(CameraReplyError):
            decode_candidate_reply(b'\xdf')
        camera, transport = self.camera(b'\xdf')
        report = probe_stabilization_candidate(camera)
        self.assertEqual(report['camera_status'], 0xdf)
        self.assertEqual(report['error']['type'], 'CameraReplyError')
        self.assertFalse(report['wire_reply_valid'])
        self.assert_unverified(report)
        self.assert_one_get(transport)

    def test_transport_failure_never_reconnects_or_retries(self):
        camera, transport = self.camera(b'')
        transport.request.side_effect = TimeoutError('No matching reply')
        report = probe_stabilization_candidate(camera)
        self.assertEqual(report['error']['type'], 'TimeoutError')
        self.assertIsNone(report['response_hex'])
        self.assert_unverified(report)
        self.assert_one_get(transport)

    def test_invalid_timeout_cannot_send_any_request(self):
        camera, transport = self.camera(b'')
        for value in [0, -1, float('inf'), float('nan'), True, '2']:
            with self.subTest(timeout=value), self.assertRaises((TypeError, ValueError)):
                probe_stabilization_candidate(camera, timeout=value)
        transport.request.assert_not_called()


if __name__ == '__main__':
    unittest.main()
