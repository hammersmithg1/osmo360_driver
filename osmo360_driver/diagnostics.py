#!/usr/bin/env python3
"""Pure SDK capture/lifecycle baseline. No ROS, JPEG, recording or recovery.

Run as ``python3 -m osmo360_driver.diagnostics`` with Linux USB permissions. Every stdout line is JSON; a failed capture or cleanup exits nonzero.
"""

import argparse
import base64
from dataclasses import asdict, is_dataclass
import json
import math
import os
import re
import secrets
import shlex
import shutil
import subprocess
import threading
import time


class ReaderShutdownError(RuntimeError):
    """A live reader forbids issuing another SDK lifecycle operation."""


def positive_seconds(value):
    try:
        number = float(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError('expected positive finite seconds') from error
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('expected positive finite seconds')
    return number


def nonnegative_integer(value):
    try:
        number = int(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError('expected a nonnegative integer') from error
    if number < 0:
        raise argparse.ArgumentTypeError('expected a nonnegative integer')
    return number


def imu_read_buffer_bytes(value):
    try:
        number = int(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError('expected 1024..65536 bytes, multiple of 1024') from error
    if not 1024 <= number <= 65536 or number % 1024:
        raise argparse.ArgumentTypeError('expected 1024..65536 bytes, multiple of 1024')
    return number


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('--seconds', type=positive_seconds, default=150.0)
    result.add_argument('--cycles', type=nonnegative_integer, default=3)
    result.add_argument('--cycle-seconds', type=positive_seconds, default=8.0)
    result.add_argument('--sleep-seconds', type=positive_seconds, default=1.0)
    result.add_argument('--imu-read-buffer-bytes', type=imu_read_buffer_bytes, default=65536,
                        help='A/B host USB read size: 1024..65536, multiple of 1024 (default: 65536)')
    result.add_argument('--inspect-only', action='store_true',
                        help='Read camera OS diagnostics only; no capture or lifecycle transitions')
    result.add_argument('--standard-sleep', action='store_true',
                        help='Compare the SDK default sleep path without retained ECM')
    return result


def error_details(error):
    chain, seen = [], set()
    while error is not None and id(error) not in seen:
        seen.add(id(error))
        chain.append({'type': type(error).__name__, 'message': str(error)})
        error = error.__cause__ if error.__cause__ is not None else (
            None if error.__suppress_context__ else error.__context__)
    return chain


def emit(event, **fields):
    print(json.dumps(dict(event=event, monotonic_s=time.monotonic(), **fields),
                     ensure_ascii=False, allow_nan=False, default=repr), flush=True)


def host_health():
    result = {'load_average': list(os.getloadavg())}
    command = shutil.which('vcgencmd')
    if command is None:
        result['power'] = {'available': False}
        return result
    power = result['power'] = {'available': True}
    for argument in ('get_throttled', 'pmic_read_adc'):
        try:
            value = subprocess.run([command, argument], capture_output=True, text=True, timeout=1)
            if value.returncode:
                power[argument + '_error'] = value.stderr.strip()[:200] or value.stdout.strip()[:200]
            elif argument == 'get_throttled':
                power['get_throttled'] = value.stdout.strip()
            else:
                match = re.search(r'EXT5V_V\s+volt\(\d+\)=([0-9.]+)V', value.stdout)
                power['ext5v_v'] = float(match[1]) if match else None
        except (OSError, subprocess.TimeoutExpired) as error:
            power[argument + '_error'] = str(error)[:200]
    return result


def datalink_health(stream):
    """Read the current SDK's own private link; never change or monkeypatch it."""
    core = getattr(stream, '_core', stream)
    preview = getattr(core, '_preview', None)
    video = getattr(preview, '_stream', None)
    link = getattr(video, '_link', None)
    if link is None:
        return None
    now = time.monotonic()
    def age(name):
        stamp = getattr(link, name, None)
        return now - stamp if isinstance(stamp, (int, float)) and stamp > 0 else None
    result = {'last_video_age_s': age('last_video'), 'last_data_ack_age_s': age('_last_ack')}
    for name in ('_acks_seen', '_acked_cursor', '_link_seq', '_data_seq', '_data_count',
                 '_message_seq', '_transmit_state', 'resent'):
        if hasattr(link, name):
            result[name.lstrip('_')] = getattr(link, name)
    result['pending_data'] = len(getattr(link, '_unacked', ()))
    result['network'] = dict(getattr(link, 'stats', {}))
    return result


def safe_stats(stream):
    if stream is None:
        return None
    try:
        return stream.stats
    except Exception as error:
        return {'stats_error': error_details(error)}


def stats_subset(stats):
    if not stats:
        return stats
    if 'stats_error' in stats:
        return stats
    preview = stats.get('preview') or {}
    video = preview.get('video') or {}
    keys = ('encoded_units', 'decoded_frames', 'published_frames', 'dropped_frames',
            'missing_timestamps', 'metadata_errors', 'queued_frames', 'stopped',
            'error', 'error_cause', 'network', 'latest_video')
    return {'state': stats.get('state'), 'control_verified': stats.get('control_verified'),
            'video': {key: video.get(key) for key in keys}, 'lease': preview.get('lease'),
            'imu': stats.get('imu'), 'errors': (stats.get('report') or {}).get('errors')}


def failure_camera_state(camera):
    """Best-effort ordinary GETs after capture failure, never diagnostic START."""
    result = {}
    for name, options in (('state', {'timeout': 2}), ('system_mode', {'timeout': 2}),
                          ('mode_profile', {}), ('lens_mode', {})):
        try:
            value = getattr(camera, name)(**options)
            result[name] = asdict(value) if is_dataclass(value) else value
        except Exception as error:
            result[name + '_error'] = error_details(error)
    result['usb_transport'] = usb_transport_diagnostics(getattr(camera, '_transport', None))
    return result


def usb_transport_diagnostics(transport):
    if transport is None:
        return None
    allowed = {(2, 0x70), (0, 0x44), (2, 0xEB), (2, 0x8E)}
    result = {'received_bytes': getattr(transport, 'received_bytes', None),
              'failure': str(transport.failure) if getattr(transport, 'failure', None) else None,
              'read_failure_status': getattr(transport, 'read_failure_status', None),
              'unmatched_get_events': []}
    try:
        events = getattr(transport, 'events', ())
        snapshot = list(events.copy()) if hasattr(events, 'copy') else list(events)
        for received, message in reversed(snapshot):
            if message.command not in allowed:
                continue
            payload = message.payload
            result['unmatched_get_events'].append({
                'received_monotonic_ns': received,
                'command': f'{message.cmd_set:02x}/{message.cmd_id:02x}',
                'source': message.sender, 'target': message.receiver,
                'sequence': message.seq, 'flags': message.flags,
                'payload_bytes': len(payload), 'status_prefix_hex': payload[:16].hex(),
            })
            if len(result['unmatched_get_events']) == 32:
                break
        result['unmatched_get_events'].reverse()
    except Exception as error:
        result['events_error'] = error_details(error)
    return result


def inspect_app_presence(camera):
    """Typed QUERY only; never register or unregister an APP."""
    from osmo360_camera import _app
    reply = camera._transport.request(0, 0x88, _app.query_payload(), receiver=0x28, timeout=2)
    value = _app.decode_query_reply(reply.payload)
    return {'action': value.action, 'registered_records': value.count,
            'own_records': len(value.own_records), 'all_apps_absent': value.all_apps_absent}


INSPECTION_JOBS = {
    'uptime_load': ['cat', '/proc/uptime', '/proc/loadavg'],
    'processes': ['sh', '-c', 'ps -A -o pid,stat,comm | grep -Ei "dcam|camera|thermal|ipc|duss|media|service"'],
    'logcat': ['logcat', '-d', '-v', 'threadtime', '-t', '2000'],
    'logcat_warnings': ['logcat', '-d', '-v', 'threadtime', '-t', '2000', '*:W'],
    'dmesg': ['dmesg'],
    'camera_warnings': ['logcat', '-d', '-v', 'threadtime', '-t', '20000', 'DUSS51:W', 'DUSS40:F', '*:S'],
    'camera_threads': ['sh', '-c', 'ps -T -p "$(pidof dji_camera3)" -o tid,stat,comm,wchan'],
    'storage': ['df', '-h'],
    'log_files': ['sh', '-c', 'ls -ltr /blackbox/camera/* /blackbox/system/* | tail -40'],
}
INSPECTION_DIRECTORY = '/blackbox/export_tmp'
_SENSITIVE_LINE = re.compile(r'password|passwd|psk|token|secret|credential', re.IGNORECASE)
_WIRELESS_LINE = re.compile(r'wi-?fi|wlan|wpa|ssid|bssid', re.IGNORECASE)


def redact_inspection_text(text):
    lines, redacted = [], 0
    for line in text.splitlines():
        if _SENSITIVE_LINE.search(line) or _WIRELESS_LINE.search(line):
            lines.append('[REDACTED LOG LINE]')
            redacted += 1
        else:
            lines.append(line)
    return '\n'.join(lines), redacted


def _inspection_command(program):
    command = 'python3 -c ' + shlex.quote(program)
    if not command.isascii() or len(command) > 500 or '\0' in command:
        raise ValueError('Inspection command exceeds fixed 500-byte ASCII boundary')
    return command


def inspection_export_command(job, path):
    # Only names from this fixed table are accepted, never a caller's command.
    args = INSPECTION_JOBS[job]
    if re.fullmatch(r'/blackbox/export_tmp/osmo_inspect_[0-9a-f]{12}\.json', path) is None:
        raise ValueError('Unexpected inspection export path')
    # Linux camera flags 193 = O_WRONLY|O_CREAT|O_EXCL, mode384 = 0600.
    # Combined stdout/stderr is bounded AFTER the two-second child finishes.
    program = ('import os,json,base64,subprocess as s\ntry:r=s.run(' + json.dumps(args) +
               ',stdout=-1,stderr=-2,timeout=2);b=r.stdout;e=r.returncode\n'
               'except Exception as x:b=getattr(x,"stdout",b"")or b"";e=repr(x)\n'
               'f=os.fdopen(os.open(' + json.dumps(path) + ',193,384),"w");i=os.fstat(f.fileno());'
               'json.dump(dict(e=e,n=len(b),b=base64.b64encode(b[-8000:]).decode(),'
               'i=[i.st_dev,i.st_ino]),f);f.close()')
    return _inspection_command(program)


def _parse_inspection_export(raw):
    if not isinstance(raw, bytes) or not 0 < len(raw) <= 16384:
        raise ValueError('Bounded inspection JSON export required')
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError('Duplicate inspection report key')
            result[key] = value
        return result
    value = json.loads(raw, object_pairs_hook=unique)
    if (not isinstance(value, dict) or set(value) != {'e', 'n', 'b', 'i'}
            or type(value['n']) is not int or value['n'] < 0
            or not isinstance(value['b'], str)
            or type(value['i']) is not list or len(value['i']) != 2
            or any(type(item) is not int or item <= 0 for item in value['i'])
            or type(value['e']) not in (int, str)):
        raise ValueError('Invalid inspection report schema')
    return value


def decode_inspection_export(raw):
    value = _parse_inspection_export(raw)
    data = base64.b64decode(value['b'], validate=True)
    if len(data) != min(8000, value['n']):
        raise ValueError('Inspection tail length disagrees with export')
    text = data.decode('utf-8', errors='replace')
    if value['n'] > 8000:
        # The first partial line could have lost its sensitive-keyword prefix.
        text = text.split('\n', 1)[1] if '\n' in text else '[TRUNCATED LOG LINE OMITTED]'
    text, count = redact_inspection_text(text)
    status = value['e']
    if isinstance(status, str):
        status = redact_inspection_text(status)[0]
    return {'status': status, 'text': text, 'output_bytes': value['n'],
            'truncated': value['n'] > 8000, 'redacted_lines': count}, value['i']


def _redact_inspection_value(value):
    if isinstance(value, str):
        return redact_inspection_text(value)[0]
    if isinstance(value, dict):
        return {key: _redact_inspection_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_redact_inspection_value(item) for item in value]
    return value


def run_inspection(camera_factory, control_factory, output=emit):
    """Observe OS logs over one owned AMT control; no stream/reset/recovery."""
    camera = control = None
    evidence, failures, cleanup = {}, [], {}
    native = None
    began = time.monotonic()
    try:
        output('inspect_started')
        camera = camera_factory()
        camera.connect(timeout=None)
        with camera._control_lock:
            control = control_factory(camera, timeout=15)
            transport = camera._transport
            reader = transport._thread
            identity = dict(transport._device_identity)

            def guard(resolve=True):
                if (camera._transport is not transport or not transport.connected
                        or transport._thread is not reader
                        or transport._device_identity != identity):
                    raise RuntimeError('Inspection control generation changed; no new task')
                if resolve:
                    control.resolve_pending(timeout=3)
                session = control.bootstrap.session
                if (control.bootstrap.transport is not transport
                        or getattr(control, 'pending_command', None) is not None
                        or getattr(control, 'pending_task', None) is not None
                        or getattr(control, '_unknown_task', False)
                        or getattr(session, 'quarantined', False)):
                    raise RuntimeError('Unknown inspection task/transition; no new task')

            def run(command):
                guard()
                return control._run(command, timeout_ms=5000, pre_send=lambda: guard(False))

            program = ('import os,stat;p="/blackbox/export_tmp";'
                       'os.makedirs(p,mode=448,exist_ok=True);s=os.lstat(p);'
                       'assert stat.S_ISDIR(s.st_mode)and s.st_uid==0 and '
                       's.st_mode&511==448 and os.path.realpath(p)==p')
            run(_inspection_command(program))
            for name in INSPECTION_JOBS:
                path = INSPECTION_DIRECTORY + '/osmo_inspect_' + secrets.token_hex(6) + '.json'
                run(inspection_export_command(name, path))
                guard()
                raw = control.bootstrap.pull_bytes(path, max_bytes=16384, timeout=10)
                inode = _parse_inspection_export(raw)['i']
                try:
                    value, _ = decode_inspection_export(raw)
                    # Never print or store the original base64 in host output.
                    evidence[name] = value
                finally:
                    program = ('import os;p=' + json.dumps(path) + ';s=os.lstat(p);'
                               'assert [s.st_dev,s.st_ino]==' + repr(inode) + ';os.unlink(p)')
                    run(_inspection_command(program))
                    cleanup[name] = 'matching inode removed'
                output('inspect_stage_finished', stage=name, status=value['status'])
            native = failure_camera_state(camera)
            try:
                native['app_presence'] = inspect_app_presence(camera)
            except Exception as error:
                native['app_presence_error'] = error_details(error)
            native['usb_transport'] = usb_transport_diagnostics(transport)
            native = _redact_inspection_value(native)
    except BaseException as error:
        # Error details can include native command failure text too.
        details = error_details(error)
        for item in details:
            item['message'] = redact_inspection_text(item['message'])[0]
        failures.append(details)
    finally:
        if camera is not None:
            try:
                camera.close()
                cleanup['camera_close'] = 'closed owned handle'
            except BaseException as error:
                cleanup['camera_close_error'] = _redact_inspection_value(error_details(error))
        ok = not failures and not any(key.endswith('_error') for key in cleanup)
        output('inspect_finished', ok=ok, elapsed_s=time.monotonic() - began,
               evidence=evidence, native_camera_state=native, errors=failures, cleanup=cleanup)
    return 0 if ok else 1


def capture(stream, seconds, cycle, output, health_probe, *, reports=None, video_timeout=5, imu_timeout=3):
    stop = threading.Event()
    lock = threading.Lock()
    errors = []
    counts = {'video': 0, 'video_without_native_clock': 0, 'imu': 0}
    began = time.monotonic()

    def consume_imu():
        last = time.monotonic()
        try:
            while not stop.is_set():
                try:
                    sample = stream.read_imu(timeout=0.1)
                except TimeoutError:
                    if time.monotonic() - last >= imu_timeout:
                        raise RuntimeError(f'No SDK IMU samples for {imu_timeout:g} seconds')
                    continue
                # Exercise the same public values used by the ROS integration.
                _ = sample.timestamp_ns, sample.gyro, sample.accel
                last = time.monotonic()
                with lock:
                    counts['imu'] += 1
        except BaseException as error:
            errors.append(error)
            stop.set()

    def sample_status():
        next_sample = time.monotonic()
        while not stop.is_set():
            try:
                with lock:
                    snapshot = dict(counts)
                output('sample', cycle=cycle, elapsed_s=time.monotonic() - began,
                       counts=snapshot, sdk=stats_subset(safe_stats(stream)),
                       datalink=datalink_health(stream), host=health_probe())
            except Exception as error:
                output('sample_error', cycle=cycle, error=error_details(error))
            next_sample += 1
            stop.wait(max(0, next_sample - time.monotonic()))

    imu = threading.Thread(target=consume_imu, name='SDK baseline IMU', daemon=True)
    sampler = threading.Thread(target=sample_status, name='SDK baseline stats', daemon=True)
    output('capture_started', cycle=cycle, seconds=seconds)
    imu.start()
    sampler.start()
    last_video = time.monotonic()
    try:
        while time.monotonic() - began < seconds and not stop.is_set():
            try:
                frame = stream.read_video(timeout=0.2)
            except TimeoutError:
                if time.monotonic() - last_video >= video_timeout:
                    raise RuntimeError(f'No SDK video frames for {video_timeout:g} seconds')
                continue
            last_video = time.monotonic()
            _ = frame.bgr  # Decode/BGR conversion, without ROS or JPEG compression.
            with lock:
                counts['video'] += 1
                counts['video_without_native_clock'] += frame.timestamp_ns is None
        if errors:
            raise errors[0]
    finally:
        stop.set()
        imu.join(6)
        sampler.join(6)
        if imu.is_alive() or sampler.is_alive():
            # Lifecycle must not race an SDK reader or stats lock owner.
            raise ReaderShutdownError('SDK baseline readers did not stop; lifecycle is unsafe')
        report = dict(cycle=cycle, elapsed_s=time.monotonic() - began,
                      counts=dict(counts), sdk=stats_subset(safe_stats(stream)))
        if reports is not None:
            reports.append(report)
        output('capture_finished', **report)
    if errors:
        raise errors[0]
    if not counts['video'] or not counts['imu']:
        raise RuntimeError('Capture ended without joint video and IMU')
    return dict(counts)


def run_check(args, camera_factory, output=emit, health_probe=host_health):
    camera = stream = None
    before = after = None
    failures, cleanup = [], {}
    native_failure = None
    captures = []
    readers_stopped = True
    began = time.monotonic()

    def stage(name, operation):
        at = time.monotonic()
        output('stage_started', stage=name)
        try:
            result = operation()
        except BaseException as error:
            output('stage_failed', stage=name, elapsed_s=time.monotonic() - at,
                   error=error_details(error))
            raise
        output('stage_finished', stage=name, elapsed_s=time.monotonic() - at)
        return result

    try:
        output('check_started', seconds=args.seconds, cycles=args.cycles,
               cycle_seconds=args.cycle_seconds, sleep_seconds=args.sleep_seconds,
               video_queue=8, imu_queue=32768, imu_read_buffer_bytes=args.imu_read_buffer_bytes,
               fast_sleep=not args.standard_sleep, retain_ecm=not args.standard_sleep)
        camera = camera_factory()
        stage('connect', lambda: camera.connect(timeout=None))
        stream = camera.usb_stream(fast_sleep=not args.standard_sleep, retain_ecm=not args.standard_sleep, queue_size=8,
                                   imu_options={'queue_size': 32768,
                                                'read_buffer_bytes': args.imu_read_buffer_bytes})
        stage('start', stream.start)
        capture(stream, args.seconds, 0, output, health_probe, reports=captures)
        for cycle in range(1, args.cycles + 1):
            stage(f'sleep_{cycle}', stream.sleep)
            time.sleep(args.sleep_seconds)
            stage(f'wake_{cycle}', stream.wake)
            capture(stream, args.cycle_seconds, cycle, output, health_probe, reports=captures)
    except BaseException as error:
        readers_stopped = not isinstance(error, ReaderShutdownError)
        failures.append(error_details(error))
    finally:
        before = safe_stats(stream) if readers_stopped else {'stats_deferred': 'reader may own SDK lock'}
        if failures and readers_stopped and stream is not None and stream.state == 'AWAKE':
            # Readers have exited, but the failed native session is still here.
            # Never probe a transition/unknown recovery state or replace the
            # primary capture exception with an optional diagnostic failure.
            try:
                native_failure = failure_camera_state(camera)
                output('failure_camera_state', **native_failure)
            except Exception as error:
                native_failure = {'diagnostic_error': error_details(error)}
        if not readers_stopped:
            cleanup['readers_error'] = 'Live reader: no competing close or recovery was attempted'
        if stream is not None and readers_stopped:
            try:
                cleanup['stream_close'] = stage('stream_close', stream.close)
            except BaseException as error:
                cleanup['stream_close_error'] = error_details(error)
        if camera is not None and readers_stopped:
            try:
                cleanup['camera_close'] = stage('camera_close', camera.close)
            except BaseException as error:
                cleanup['camera_close_error'] = error_details(error)
        after = safe_stats(stream) if readers_stopped else None
        ok = not failures and not any(key.endswith('_error') for key in cleanup)
        output('check_finished', ok=ok, elapsed_s=time.monotonic() - began,
               captures=captures, errors=failures, cleanup=cleanup,
               failure_camera_state=native_failure,
               before_cleanup_stats=before, after_cleanup_stats=after)
    return 0 if ok else 1


def main(argv=None):
    args = parser().parse_args(argv)
    from osmo360_camera import Camera
    if args.inspect_only:
        from osmo360_camera._helper_install import _controller
        return run_inspection(Camera, _controller)
    return run_check(args, Camera)


if __name__ == '__main__':
    raise SystemExit(main())
