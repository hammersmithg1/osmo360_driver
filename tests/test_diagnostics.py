"""SDK-only diagnostic orchestration with fakes; no camera/ROS/process probes."""

import base64
import json
import os
import re
import shlex
import tempfile
import threading
from collections import deque
from dataclasses import dataclass
from pathlib import Path
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch


from osmo360_driver import diagnostics as check

class FakeStream:
    def __init__(self):
        self.events = []
        self.state = 'NEW'
        self.imu_reading = False
        self.video_missing = self.imu_missing = False
        self.video_error = None
        self.error_state = None
        self.native_timestamp = 123
        self.video = self.imu = 0
        self.report = {'errors': []}

    @property
    def stats(self):
        return {'state': self.state, 'preview': {'video': {'published_frames': self.video}},
                'imu': {'received': self.imu}, 'report': dict(self.report)}

    def start(self):
        self.events.append('start'); self.state = 'AWAKE'

    def read_video(self, timeout):
        time.sleep(.001)
        if self.video_error:
            if self.error_state is not None:
                self.state = self.error_state
            raise self.video_error
        if self.video_missing:
            raise TimeoutError()
        self.video += 1
        events = self.events
        native_timestamp = self.native_timestamp
        class Frame:
            timestamp_ns = native_timestamp
            @property
            def bgr(self):
                events.append('bgr')
                return object()
        return Frame()

    def read_imu(self, timeout):
        self.imu_reading = True
        try:
            time.sleep(.002)
            if self.imu_missing:
                raise TimeoutError()
            self.imu += 1
            return SimpleNamespace(timestamp_ns=123, gyro=(0, 0, 0), accel=(0, 0, 9.8))
        finally:
            self.imu_reading = False

    def sleep(self):
        if self.imu_reading:
            raise AssertionError('lifecycle raced IMU reader')
        self.events.append('sleep'); self.state = 'SLEEP'

    def wake(self):
        self.events.append('wake'); self.state = 'AWAKE'

    def close(self):
        if self.imu_reading:
            raise AssertionError('close raced IMU reader')
        self.events.append('close'); self.state = 'DETACHED'
        self.report['last_imu_stop'] = {'restored': True, 'paired_samples': self.imu}
        return dict(self.report)


class FakeCamera:
    def __init__(self, stream):
        self.stream = stream
        self.options = None
        self.closed = False
        self.queries = []
        self.mode_error = None

    def connect(self, timeout):
        if timeout is not None:
            raise AssertionError('SDK must own service readiness')

    def usb_stream(self, **options):
        self.options = options
        return self.stream

    def close(self):
        self.closed = True

    def state(self, timeout):
        if self.stream.imu_reading or self.closed:
            raise AssertionError('GET raced reader or happened after close')
        if timeout != 2:
            raise AssertionError('unbounded failure query')
        self.queries.append('state')
        @dataclass
        class State:
            recording_state: int = 0
        return State()

    def system_mode(self, timeout):
        self.queries.append('system_mode')
        if self.mode_error:
            raise self.mode_error
        return {'state': 'standard_state', 'scene': 'liveview'}

    def mode_profile(self):
        self.queries.append('mode_profile')
        return {'profile': 56}

    def lens_mode(self):
        self.queries.append('lens_mode')
        return 1


class CheckOsmoTests(unittest.TestCase):
    def test_durations_reject_nonfinite_zero_and_negative(self):
        for value in ('nan', 'inf', '-1', '0'):
            with self.subTest(value=value), self.assertRaises(Exception):
                check.positive_seconds(value)
        self.assertEqual(check.positive_seconds('0.01'), .01)
        args = check.parser().parse_args([])
        self.assertEqual((args.seconds, args.cycles, args.cycle_seconds), (150, 3, 8))

    def test_joint_capture_and_cycles_join_readers_then_preserve_final_cleanup(self):
        stream = FakeStream(); camera = FakeCamera(stream); output = []
        args = check.parser().parse_args(['--seconds', '.02', '--cycles', '2', '--cycle-seconds', '.02', '--sleep-seconds', '.001'])
        result = check.run_check(args, lambda: camera, lambda event, **fields: output.append((event, fields)), lambda: {})
        self.assertEqual(result, 0)
        self.assertEqual(camera.options, {'fast_sleep': True, 'retain_ecm': True,
            'queue_size': 8, 'imu_options': {'queue_size': 32768, 'read_buffer_bytes': 65536}})
        self.assertEqual(output[0][1]['imu_read_buffer_bytes'], 65536)
        self.assertEqual(stream.events.count('sleep'), 2)
        self.assertEqual(stream.events.count('wake'), 2)
        self.assertGreater(stream.events.count('bgr'), 0)
        self.assertEqual(stream.events[-1], 'close')
        self.assertTrue(camera.closed)
        final = output[-1][1]
        self.assertTrue(final['ok'])
        self.assertEqual(len(final['captures']), 3)
        self.assertEqual(final['after_cleanup_stats']['report']['last_imu_stop']['paired_samples'], stream.imu)

    def test_source_failure_returns_error_but_still_closes_and_reports(self):
        stream = FakeStream(); stream.video_error = RuntimeError('source failed')
        camera = FakeCamera(stream); output = []
        args = check.parser().parse_args(['--seconds', '.03', '--cycles', '0'])
        result = check.run_check(args, lambda: camera, lambda event, **fields: output.append((event, fields)), lambda: {})
        self.assertEqual(result, 1)
        self.assertTrue(camera.closed)
        self.assertIn('close', stream.events)
        self.assertIn('source failed', str(output[-1][1]['errors']))
        self.assertIn('last_imu_stop', output[-1][1]['cleanup']['stream_close'])
        self.assertEqual(camera.queries, ['state', 'system_mode', 'mode_profile', 'lens_mode'])
        self.assertEqual(output[-1][1]['failure_camera_state']['state'], {'recording_state': 0})

    def test_optional_video_native_clock_is_reported_without_changing_sdk_contract(self):
        stream = FakeStream(); stream.native_timestamp = None
        camera = FakeCamera(stream); output = []
        args = check.parser().parse_args(['--seconds', '.02', '--cycles', '0'])
        result = check.run_check(args, lambda: camera, lambda event, **fields: output.append((event, fields)), lambda: {})
        self.assertEqual(result, 0)
        counts = output[-1][1]['captures'][0]['counts']
        self.assertGreater(counts['video'], 0)
        self.assertEqual(counts['video_without_native_clock'], counts['video'])

    def test_real_no_data_timeout_stops_both_readers_without_lifecycle(self):
        for source in ('video', 'imu'):
            stream = FakeStream(); setattr(stream, source + '_missing', True)
            with self.subTest(source=source), self.assertRaisesRegex(RuntimeError, 'No SDK ' + source.upper() if source == 'imu' else 'No SDK video'):
                check.capture(stream, .2, 0, lambda *a, **k: None, lambda: {}, video_timeout=.02, imu_timeout=.02)
            self.assertFalse(stream.imu_reading)
            self.assertNotIn('close', stream.events)

    def test_buffer_ab_option_is_validated_and_reaches_only_sdk_imu_options(self):
        for value in ('0', '1023', '1025', '65537', 'inf', '1.5'):
            with self.subTest(value=value), self.assertRaises(Exception):
                check.imu_read_buffer_bytes(value)
        for value in ('1024', '16384', '65536'):
            self.assertEqual(check.imu_read_buffer_bytes(value), int(value))
        stream = FakeStream(); camera = FakeCamera(stream); output = []
        args = check.parser().parse_args(['--seconds', '.02', '--cycles', '0', '--imu-read-buffer-bytes', '16384'])
        self.assertEqual(check.run_check(args, lambda: camera,
            lambda event, **fields: output.append((event, fields)), lambda: {}), 0)
        self.assertEqual(camera.options['imu_options'], {'queue_size': 32768, 'read_buffer_bytes': 16384})
        self.assertEqual(output[0][1]['imu_read_buffer_bytes'], 16384)

    def test_failure_queries_preserve_primary_error_and_skip_unknown_sdk_state(self):
        for state in ('AWAKE', 'RECOVERY_REQUIRED'):
            with self.subTest(state=state):
                stream = FakeStream(); stream.video_error = RuntimeError('original capture failure')
                stream.error_state = state
                camera = FakeCamera(stream); camera.mode_error = TimeoutError('mode GET timeout')
                output = []
                args = check.parser().parse_args(['--seconds', '.02', '--cycles', '0'])
                result = check.run_check(args, lambda: camera,
                    lambda event, **fields: output.append((event, fields)), lambda: {})
                self.assertEqual(result, 1)
                final = output[-1][1]
                self.assertIn('original capture failure', str(final['errors']))
                if state == 'AWAKE':
                    self.assertIn('mode GET timeout', str(final['failure_camera_state']))
                    self.assertEqual(camera.queries[-1], 'lens_mode')
                else:
                    self.assertEqual(camera.queries, [])
                    self.assertIsNone(final['failure_camera_state'])


class FakeInspectionControl:
    def __init__(self, camera):
        self.pending_task = self.pending_command = None
        self._unknown_task = False
        self.commands, self.exports = [], {}
        self.resolve_calls = 0
        self.resolved = False
        self.bootstrap = SimpleNamespace(transport=camera._transport,
            session=SimpleNamespace(quarantined=False), pull_bytes=self.pull)

    def resolve_pending(self, timeout):
        self.resolve_calls += 1
        self.resolved = True

    def _run(self, command, timeout_ms, pre_send):
        if not self.resolved:
            raise AssertionError('native run did not resolve the same controller')
        self.resolved = False
        pre_send()
        self.commands.append(command)
        if len(command) > 500 or not command.isascii() or timeout_ms != 5000:
            raise AssertionError('unbounded native inspection command')
        path = re.search(r'/blackbox/export_tmp/osmo_inspect_[0-9a-f]{12}\.json', command)
        if path is not None:
            path = path[0]
            if 'os.open(' in command:
                data = b'cameraCaptureService IPC waiting\npassword=SYNTHETIC_PRIVATE_VALUE\n'
                self.exports[path] = json.dumps({'e': 0, 'n': len(data),
                    'b': base64.b64encode(data).decode(), 'i': [42, 17]}).encode()
            elif 'os.unlink(p)' in command:
                if '[s.st_dev,s.st_ino]==[42, 17]' not in command:
                    raise AssertionError('cleanup lacks exact inode proof')
                del self.exports[path]

    def pull(self, path, max_bytes, timeout):
        if max_bytes != 16384 or timeout != 10:
            raise AssertionError('unbounded inspection pull')
        return self.exports[path]


class InspectionTests(unittest.TestCase):
    def camera(self):
        camera = FakeCamera(FakeStream())
        camera._control_lock = threading.RLock()
        camera._transport = SimpleNamespace(connected=True, _thread=object(),
                                             _device_identity={'bus': 1, 'address': 2})
        return camera

    def test_inspection_reuses_one_guarded_control_and_never_creates_a_stream(self):
        camera = self.camera(); control = FakeInspectionControl(camera); output = []
        camera.mode_error = TimeoutError('ordinary service not ready')
        factories = []
        def factory(value, timeout):
            factories.append((value, timeout))
            return control
        result = check.run_inspection(lambda: camera, factory,
            lambda event, **fields: output.append((event, fields)))
        self.assertEqual(result, 0)
        self.assertEqual(factories, [(camera, 15)])
        self.assertEqual(camera.stream.events, [])
        self.assertIsNone(camera.options)
        self.assertTrue(camera.closed)
        self.assertFalse(control.exports)
        self.assertGreaterEqual(control.resolve_calls, len(control.commands))
        final = output[-1][1]
        self.assertEqual(set(final['evidence']), set(check.INSPECTION_JOBS))
        self.assertIn('ordinary service not ready', str(final['native_camera_state']))
        serialized = json.dumps(output)
        self.assertNotIn('SYNTHETIC_PRIVATE_VALUE', serialized)
        self.assertNotIn('Y2FtZXJhQ2FwdHVyZVNlcnZpY2U', serialized)
        self.assertIn('[REDACTED LOG LINE]', serialized)

    def test_unknown_or_quarantined_control_refuses_all_native_runs(self):
        for unknown in (False, True):
            with self.subTest(unknown=unknown):
                camera = self.camera(); control = FakeInspectionControl(camera); output = []
                if unknown:
                    control._unknown_task = True
                else:
                    control.bootstrap.session.quarantined = True
                result = check.run_inspection(lambda: camera, lambda *a, **k: control,
                    lambda event, **fields: output.append((event, fields)))
                self.assertEqual(result, 1)
                self.assertEqual(control.commands, [])
                self.assertEqual(camera.queries, [])
                self.assertTrue(camera.closed)

    def test_fixed_export_program_is_bounded_and_creates_exclusive_private_file(self):
        native_path = '/blackbox/export_tmp/osmo_inspect_aabbccddeeff.json'
        for job in check.INSPECTION_JOBS:
            command = check.inspection_export_command(job, native_path)
            self.assertLessEqual(len(command), 500)
            self.assertTrue(command.isascii())
        with self.assertRaises(KeyError):
            check.inspection_export_command('arbitrary caller command', native_path)
        with self.assertRaises(ValueError):
            check.inspection_export_command('logcat', '/tmp/caller-path')
        command = check.inspection_export_command('logcat', native_path)
        program = shlex.split(command)[2]
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / 'export.json'
            original_open = os.open
            def camera_open(path, flags, mode):
                self.assertEqual((path, flags, mode), (native_path, 193, 384))
                # Translate only Linux open flags for an offline macOS test.
                return original_open(target, os.O_WRONLY | os.O_CREAT | os.O_EXCL, mode)
            data = b'cameraCaptureService ready\n'
            with patch.object(os, 'open', side_effect=camera_open), patch('subprocess.run',
                    return_value=SimpleNamespace(stdout=data, returncode=0)) as run:
                exec(compile(program, 'fixed-camera-inspection', 'exec'), {})
            run.assert_called_once_with(check.INSPECTION_JOBS['logcat'], stdout=-1, stderr=-2, timeout=2)
            self.assertEqual(target.stat().st_mode & 0o777, 0o600)
            value, inode = check.decode_inspection_export(target.read_bytes())
            self.assertEqual(inode, [target.stat().st_dev, target.stat().st_ino])
            self.assertIn('cameraCaptureService ready', value['text'])

    def test_truncated_first_line_is_omitted_before_redaction(self):
        data = b'SYNTHETIC_VALUE_WITHOUT_KEYWORD_PREFIX\ncamera healthy\n'
        # Emulate a long native tail whose first line lost its keyword prefix.
        tail = b'x' * (8000 - len(data)) + data
        raw = json.dumps({'e': 0, 'n': 9000, 'b': base64.b64encode(tail).decode(), 'i': [1, 2]}).encode()
        value, _ = check.decode_inspection_export(raw)
        self.assertEqual(value['text'], 'camera healthy')
        self.assertTrue(value['truncated'])

    def test_usb_evidence_contains_only_bounded_unmatched_get_prefixes(self):
        messages = []
        for i in range(100):
            message = SimpleNamespace(command=(2, 0x70), cmd_set=2, cmd_id=0x70,
                sender=1, receiver=10, seq=i, flags=0x80,
                payload=bytes(16) + b'SYNTHETIC_PAYLOAD_TAIL')
            messages.append((i, message))
        messages.append((101, SimpleNamespace(command=(0, 0xD1), payload=b'PRIVATE_OTHER_COMMAND')))
        transport = SimpleNamespace(received_bytes=12345, failure=None,
            read_failure_status=None, events=deque(messages, maxlen=256))
        evidence = check.usb_transport_diagnostics(transport)
        self.assertEqual(evidence['received_bytes'], 12345)
        self.assertEqual(len(evidence['unmatched_get_events']), 32)
        self.assertEqual(evidence['unmatched_get_events'][-1]['sequence'], 99)
        for event in evidence['unmatched_get_events']:
            self.assertEqual(event['command'], '02/70')
            self.assertEqual(event['status_prefix_hex'], '00' * 16)
        self.assertNotIn('PRIVATE_OTHER_COMMAND', json.dumps(evidence))
        self.assertNotIn(b'SYNTHETIC_PAYLOAD_TAIL'.hex(), json.dumps(evidence))

    def test_app_presence_uses_only_typed_readonly_system_query(self):
        import sys
        codec = SimpleNamespace(query_payload=lambda: bytes.fromhex('1100320001'),
            decode_query_reply=lambda payload: SimpleNamespace(action=0x55, count=0,
                                                               own_records=(), all_apps_absent=True))
        calls = []
        def request(*args, **options):
            calls.append((args, options))
            return SimpleNamespace(payload=bytes.fromhex('1200550000'))
        camera = SimpleNamespace(_transport=SimpleNamespace(request=request))
        with patch.dict(sys.modules, {'osmo360_camera': SimpleNamespace(_app=codec)}):
            value = check.inspect_app_presence(camera)
        self.assertEqual(calls, [((0, 0x88, bytes.fromhex('1100320001')), {'receiver': 0x28, 'timeout': 2})])
        self.assertEqual(value['own_records'], 0)
        self.assertTrue(value['all_apps_absent'])
