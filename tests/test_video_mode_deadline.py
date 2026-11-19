"""Startup deadlines block forward SET even after nested C1 GETs are slow.

Real Camera facade/typed setters and fake transport prove both absence of the
forward mutation and absence of a spurious rollback when nothing was delivered.
An already accepted SET still completes its original readback/rollback.
"""
from unittest import TestCase
from unittest.mock import patch

from osmo360_camera.camera import Camera, SettingApplyError
from osmo360_camera import controls as c
from osmo360_camera._video_mode import ensure_video_mode
from test_camera_api import FakeTransport, FakeClock


class VideoModeDeadlineTests(TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.clock_patch = patch('osmo360_camera.camera.time', self.clock)
        self.clock_patch.start()
        self.addCleanup(self.clock_patch.stop)
        self.transport = FakeTransport()
        self.camera = Camera(transport=self.transport).connect()
        self.profile = 5
        self.lens = c.LensMode.SINGLE_LENS
        self.transport.handlers[(2, int(c.CameraCommand.GET_CAMERA_INFO))] = self.profile_get
        self.transport.handlers[(2, int(c.CameraCommand.SET_MODE_PROFILE))] = self.profile_set
        self.transport.handlers[(2, int(c.CameraCommand.PARAM_OPTION))] = self.lens_option

    def guard(self):
        if self.clock.now >= 10:
            raise TimeoutError('startup deadline')

    def profile_get(self, payload):
        self.assertEqual(payload, c.encode_profile_query())
        return bytes.fromhex('00ff016142c0000700') + bytes((self.profile,))

    def profile_set(self, payload):
        self.profile = payload[0]
        return b'\0'

    def lens_option(self, payload):
        if payload == c.encode_lens_mode_query():
            return bytes.fromhex('0000013f0001') + bytes((self.lens,))
        self.lens = c.LensMode(payload[-1])
        return b'\0'

    def mutations(self):
        return [row for row in self.transport.requests
                if row[1] == int(c.CameraCommand.SET_MODE_PROFILE)
                or row[1] == int(c.CameraCommand.PARAM_OPTION) and row[2] != c.encode_lens_mode_query()]

    def test_expired_before_public_setters_does_not_issue_even_gets(self):
        self.clock.now = 10
        for operation, value in ((self.camera.set_mode_profile, c.ModeProfile.VIDEO_PANO),
                                  (self.camera.set_lens_mode, c.LensMode.PANORAMIC)):
            with self.subTest(operation=operation.__name__), self.assertRaises(TimeoutError):
                operation(value, guard=self.guard)
        self.assertEqual(self.transport.requests, [])

    def test_slow_record0_get_inside_lens_setter_blocks_all_forward_and_rollback_sets(self):
        original = self.transport.handlers[(2, 0x70)]
        def slow(payload):
            value = original(payload)
            self.clock.now = 11
            return value
        self.transport.handlers[(2, 0x70)] = slow
        with self.assertRaisesRegex(TimeoutError, 'startup deadline'):
            self.camera.set_lens_mode(c.LensMode.PANORAMIC, guard=self.guard)
        self.assertEqual([row[1] for row in self.transport.requests], [0x70])
        self.assertEqual(self.mutations(), [])

    def test_slow_previous_lens_get_inside_apply_blocks_forward_and_rollback_sets(self):
        def slow(payload):
            value = self.lens_option(payload)
            self.clock.now = 11
            return value
        self.transport.handlers[(2, int(c.CameraCommand.PARAM_OPTION))] = slow
        with self.assertRaisesRegex(TimeoutError, 'startup deadline'):
            self.camera.set_lens_mode(c.LensMode.PANORAMIC, guard=self.guard)
        self.assertEqual([row[1] for row in self.transport.requests], [0x70, int(c.CameraCommand.PARAM_OPTION)])
        self.assertEqual(self.mutations(), [])
        self.assertIs(self.lens, c.LensMode.SINGLE_LENS)

    def test_slow_previous_profile_get_inside_apply_blocks_forward_and_rollback_sets(self):
        def slow(payload):
            value = self.profile_get(payload)
            self.clock.now = 11
            return value
        self.transport.handlers[(2, int(c.CameraCommand.GET_CAMERA_INFO))] = slow
        with self.assertRaisesRegex(TimeoutError, 'startup deadline'):
            self.camera.set_mode_profile(c.ModeProfile.VIDEO_PANO, guard=self.guard)
        self.assertEqual([row[1] for row in self.transport.requests], [int(c.CameraCommand.GET_CAMERA_INFO)])
        self.assertEqual(self.mutations(), [])
        self.assertEqual(self.profile, 5)

    def test_slow_top_level_photo_get_expires_before_any_setter_is_called(self):
        self.lens = c.LensMode.PANORAMIC
        def slow(payload):
            value = self.profile_get(payload)
            self.clock.now = 11
            return value
        self.transport.handlers[(2, int(c.CameraCommand.GET_CAMERA_INFO))] = slow
        with self.assertRaisesRegex(TimeoutError, 'startup deadline'):
            self.camera.ensure_video_mode(guard=self.guard)
        self.assertEqual(self.mutations(), [])

    def test_slow_final_record0_before_profile_change_blocks_forward_set(self):
        self.lens = c.LensMode.PANORAMIC
        original = self.transport.handlers[(2, 0x70)]
        calls = 0
        def late(payload):
            nonlocal calls
            value = original(payload)
            calls += 1
            if calls == 2:
                self.clock.now = 11
            return value
        self.transport.handlers[(2, 0x70)] = late
        with self.assertRaisesRegex(TimeoutError, 'startup deadline'):
            ensure_video_mode(self.camera, guard=self.guard)
        self.assertEqual(calls, 2)
        self.assertEqual(self.mutations(), [])

    def test_accepted_profile_set_completes_readback_even_if_deadline_passed(self):
        def accepted(payload):
            self.clock.now = 11
            return self.profile_set(payload)
        self.transport.handlers[(2, int(c.CameraCommand.SET_MODE_PROFILE))] = accepted
        result = self.camera.set_mode_profile(c.ModeProfile.VIDEO_PANO, guard=self.guard)
        self.assertEqual(result.profile, int(c.ModeProfile.VIDEO_PANO))
        self.assertEqual([row[2] for row in self.mutations()], [bytes((int(c.ModeProfile.VIDEO_PANO),))])

    def test_accepted_but_unverified_set_still_rolls_back_after_deadline(self):
        def ignored(payload):
            self.clock.now = max(self.clock.now, 11)
            # ACK without changing the profile; readback must fail and rollback
            # remains part of the original accepted transaction after deadline.
            return b'\0'
        self.transport.handlers[(2, int(c.CameraCommand.SET_MODE_PROFILE))] = ignored
        with self.assertRaises(SettingApplyError) as error:
            self.camera.set_mode_profile(c.ModeProfile.VIDEO_PANO, guard=self.guard)
        self.assertTrue(error.exception.restored)
        self.assertEqual([row[2] for row in self.mutations()], [bytes((56,)), bytes((5,))])
