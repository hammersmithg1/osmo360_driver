"""Single owner for USB and lifecycle. Video and 8 kHz IMU have separate readers.

Commands are queued: start/reboot may take tens of seconds, so an HTTP/ROS
request must not wait for USB. Read status()['busy'/'error'] for the outcome.
No device is opened until an explicit start/wake/refresh command.
"""

from dataclasses import asdict, is_dataclass
import queue
import threading
import time

from osmo360_camera import (
    Camera, CameraReplyError, ColorMode, ExposureMode, ExposureModeSetting, FrameRate, IsoSetting,
    ShutterSpeed, ShutterValue, WhiteBalance, WhiteBalanceMode,
)
from osmo360_camera.transport import CameraDisconnected, RequestTimeout
from .clock import CameraClock
from .jpeg import PREVIEW_WIDTH, PREVIEW_HEIGHT, PREVIEW_QUALITY, PreviewEncoder
from osmo360_camera._usb_charging import USBChargingError, attached_camera_count


def _ignore_video(*_):
    pass


def json_value(value):
    """Raw calibration contains bytes; preserve them as explicitly named hex."""
    if is_dataclass(value):
        return json_value(asdict(value))
    if isinstance(value, bytes):
        return {'encoding': 'hex', 'data': value.hex()}
    if isinstance(value, dict):
        return {str(k): json_value(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_value(v) for v in value]
    return value


def error_chain(error):
    """Bounded diagnostic causes; never replace the exception being reported."""
    result, seen = [], set()
    while error is not None and id(error) not in seen and len(result) < 8:
        seen.add(id(error))
        try:
            message = str(error)[:1000]
        except Exception:
            message = type(error).__name__
        result.append(dict(type=type(error).__name__, message=message))
        error = error.__cause__ if error.__cause__ is not None else (
            None if error.__suppress_context__ else error.__context__)
    return result


def usb_capture_error(stats):
    if 'stats_error' in stats:
        return None  # Cached counters cannot prove the current USB failure.
    imu = stats.get('imu') or {}
    current = imu.get('collector_live') if isinstance(imu, dict) else None
    if isinstance(current, dict) and current.get('fatal_read_status') == -4:
        return 'USB-соединение с Osmo потеряно (LIBUSB_ERROR_NO_DEVICE)'
    return None


def validate_settings(value):
    if not isinstance(value, dict) or not value:
        raise ValueError('Нужен непустой объект настроек Osmo 360')
    allowed = {'exposure_mode', 'iso', 'shutter_denominator', 'white_balance', 'color', 'frame_rate'}
    if set(value) - allowed:
        raise ValueError('Неизвестные настройки Osmo 360: ' + ', '.join(sorted(set(value) - allowed)))
    for key, number in value.items():
        if type(number) is not int:
            raise ValueError(f'{key}: нужно целое число')
        if key == 'exposure_mode' and number not in (0, 1, 4):
            raise ValueError('Экспозиция: 0 — авто, 1 — Program, 4 — вручную')
        if key == 'iso' and number not in (0, 50, 100, 200, 400, 800, 1600, 3200, 6400, 12800):
            raise ValueError('ISO: авто (0), 50…12800 из списка')
        if key == 'shutter_denominator' and not 1 <= number <= 8000:
            raise ValueError('Выдержка: знаменатель 1…8000')
        if key == 'white_balance' and number != 0 and not (2000 <= number <= 10000 and number % 100 == 0):
            raise ValueError('Баланс белого: авто (0) или 2000…10000 K, шаг 100 K')
        if key == 'color' and number not in (0, 60, 61, 63):
            raise ValueError('Неизвестный цветовой профиль')
        if key == 'frame_rate' and number not in (int(FrameRate.FPS25), int(FrameRate.FPS30), int(FrameRate.FPS30_TRUE)):
            raise ValueError('Поддерживаются настройки съёмки 25, 29,97 и 30 FPS')
    return dict(value)


class OsmoCamera:
    def __init__(self, on_video=None, on_imu=None, on_error=None, *, on_unstamped_video=None,
                 on_video_lazy=None, camera_factory=Camera, host_offset_ns=None,
                 usb_charging=None, before_power_restore=None):
        if usb_charging is not None and type(usb_charging) is not bool:
            raise ValueError('usb_charging must be bool or None')
        if before_power_restore is not None and not callable(before_power_restore):
            raise ValueError('before_power_restore must be callable or None')
        self.usb_charging = usb_charging
        self.before_power_restore = before_power_restore
        self._power_lease = None
        self._power_rebind_allowed = False
        self._close_power_error = None
        self.power = dict(requested=usb_charging, state='unmanaged' if usb_charging is None else 'unprepared',
                          ready=usb_charging is None, error='', physical_proof='unknown',
                          policy_readback_is_cache=True)
        # The lazy callback replaces both legacy video callbacks, receiving
        # (frame, full_jpeg_factory, stamp). Call the factory synchronously only
        # if an archival/ROS consumer needs the unchanged full-resolution JPEG.
        self.on_video_lazy = on_video_lazy
        self.on_video = on_video or _ignore_video
        self.on_imu = on_imu or (lambda *_: None)
        self.on_error = on_error or (lambda *_: None)
        self.on_unstamped_video = on_unstamped_video or _ignore_video
        self.camera_factory = camera_factory
        self.camera = self.stream = None
        self.clock = CameraClock(host_offset_ns)
        self.host_offset_ns = self.clock.host_offset_ns
        self.condition = threading.Condition(threading.RLock())
        self.control_lock = threading.RLock()
        self.stopping = threading.Event()
        self.reader_stop = threading.Event()
        self.commands = queue.Queue(maxsize=1)
        self.readers = []
        self.frame = None
        self.sequence = self.frames = self.imu_samples = 0
        self.timestamp_missing_frames = 0
        self.frame_times = []
        self.imu_at = None
        self.enabled = False
        self.busy = self.error = self.capture_error = ''
        self._capture_failure = None
        self._close_failure_sequence = 0
        self.command_started_at = None
        self.preparation = {}
        self.helpers_ready = False
        self._verified_helpers = None
        self._helpers_previous_boot_id = None
        self.state = 'disconnected'
        self.settings = {}
        self.settings_error = ''
        self.calibration = {}
        self.capture_stats = {}
        self.stats_at = 0
        self.worker = threading.Thread(target=self.run, name='Osmo control', daemon=True)
        self.worker.start()

    def command(self, action, settings=None):
        if action not in ('start', 'stop', 'sleep', 'wake', 'reboot', 'configure', 'refresh', 'prepare_power'):
            raise ValueError('Неизвестная команда Osmo 360')
        if action == 'configure':
            settings = validate_settings(settings)
        with self.control_lock, self.condition:
            if self.stopping.is_set():
                raise ValueError('Драйвер закрывается')
            if self.busy:
                raise ValueError('Дождись завершения команды Osmo 360: ' + self.busy)
            self.busy, self.error = action, ''
            self.command_started_at = time.monotonic()
            self.commands.put_nowait((action, settings))
        return self.status()

    def status(self):
        with self.condition:
            now = time.monotonic()
            age = now - self.frame[2] if self.frame else None
            times = self.frame_times
            fps = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 and times[-1] > times[0] else 0
            return dict(enabled=self.enabled, state=self.state, busy=self.busy, error=self.error,
                        command_elapsed_s=round(now - self.command_started_at, 1) if self.busy and self.command_started_at is not None else None,
                        preparation=dict(self.preparation),
                        power=json_value(self.power),
                        frame_age_s=age, capture_fps=round(fps, 1) if age is not None and age < 2 else 0,
                        frames=self.frames, imu_samples=self.imu_samples,
                        timestamp_missing_frames=self.timestamp_missing_frames,
                        imu_age_s=now - self.imu_at if self.imu_at else None,
                        mode={'width': 2048, 'height': 1024, 'fps': 30},
                        preview={'width': PREVIEW_WIDTH, 'height': PREVIEW_HEIGHT,
                                 'jpeg_quality': PREVIEW_QUALITY},
                        projection='dual_fisheye', settings=dict(self.settings), settings_error=self.settings_error,
                        clock=self.clock.status(), calibration=self.calibration,
                        stats=self.capture_stats)

    def latest(self, after=-1, timeout=0):
        with self.condition:
            if timeout:
                self.condition.wait_for(lambda: self.sequence > after or not self.enabled or self.stopping.is_set(), timeout)
            if not self.enabled or not self.frame or self.sequence <= after or time.monotonic() - self.frame[2] > 1:
                return None
            return self.sequence, self.frame

    def hold_power(self, state='preparing'):
        if getattr(self, 'usb_charging', None) is None:
            return
        # Publish the closed gate before synchronously releasing the webcam.
        # A parent timer must not reopen it while the callback/SET is running.
        with self.condition:
            self.power = dict(self.power, ready=False, state=state, error='')
        try:
            if self.before_power_restore is not None:
                self.before_power_restore()
        except Exception as error:
            self.power = dict(self.power, ready=False, state='error', error=str(error))
            raise

    def prepare_power(self, *, verified_rebind=False):
        if getattr(self, 'usb_charging', None) is None:
            return self.power
        self.hold_power()
        try:
            count = attached_camera_count()
            if count == 0:
                if self._power_lease is not None and self._power_lease.receipt is not None:
                    raise USBChargingError('Previously owned Osmo disappeared; original charging policy is unknown')
                self.power = dict(self.power, state='absent', ready=True, error='',
                                  evidence={'complete_usb_inventory': True, 'matching_cameras': 0})
                return self.power
            if count != 1:
                raise USBChargingError('USB charging requires exactly one supported Osmo camera')
            self.require_session_state()
            self.connect(wait_ready=False)
            if self._power_lease is None:
                self._power_lease = self.camera.usb_charging(self.usb_charging)
            self.power = self._power_lease.prepare(
                verified_rebind=verified_rebind or self._power_rebind_allowed)
            self._power_rebind_allowed = False
            if self.power.get('ready') is not True:
                raise USBChargingError('USB battery charging policy is not ready', self.power)
            return self.power
        except Exception as error:
            detail = getattr(error, 'report', None) or self.power
            self.power = dict(detail, requested=self.usb_charging, ready=False,
                              state=detail.get('state', 'error'), error=str(error))
            raise

    def recheck_power(self, *, sleeping=False):
        if self.usb_charging is None:
            return
        if self._power_lease is None:
            raise USBChargingError('USB charging policy has no prepared lease')
        try:
            # SDK start/wake returns data first. Readers keep draining while
            # ordinary control/diagnostic readiness is verified; no SET here.
            control = self.stream._core.control
            if sleeping:
                previous = control.last
                if (self.stream.state != 'SLEEP' or type(previous) is not dict
                        or previous.get('state') != 'SLEEP' or not previous.get('command_id')):
                    raise USBChargingError('SDK has no completed native Sleep receipt for charging recheck')
                proof = control.wait_sleep_snapshot(previous['command_id'], deadline=time.monotonic()+20)
                if type(proof) is not dict or proof.get('state') != 'SLEEP':
                    raise USBChargingError('SDK has not verified current native Sleep for charging recheck')
            else:
                proof = self.stream.verify_control(timeout=20)
                if type(proof) is not dict or proof.get('state') != 'AWAKE':
                    raise USBChargingError('SDK has not verified current AWAKE control for charging recheck')
            self.power = self._power_lease.recheck(control=control, verified_rebind=True)
            if self.power.get('ready') is not True:
                raise USBChargingError('USB charging changed during the verified USB transition', self.power)
            self.power.setdefault('evidence', {})['verified_control_state'] = proof['state']
        except Exception as error:
            detail = getattr(error, 'report', None) or self.power
            self.power = dict(detail, requested=self.usb_charging, ready=False, error=str(error))
            raise

    def restore_power(self):
        if self._power_lease is None:
            return
        self.hold_power('restoring')
        try:
            report = self._power_lease.restore(verified_rebind=self._power_rebind_allowed)
            # A verified original restore ends our requested power policy. It
            # must not reopen a webcam merely because native restoration is OK.
            self.power = dict(report, ready=False)
        except Exception as error:
            detail = getattr(error, 'report', None) or self.power
            self.power = dict(detail, requested=self.usb_charging, ready=False, error=str(error))
            raise

    def connect(self, *, wait_ready=True):
        if self.camera is None:
            self.camera = self.camera_factory()
        if not self.camera.connected:
            self.state = 'connecting'
            self.helpers_ready = False
            self.camera.connect(timeout=30 if wait_ready else None)

    def prepare_helpers(self):
        if self.helpers_ready:
            return
        self.state = 'checking_helpers'
        previous = getattr(self, '_helpers_previous_boot_id', None)
        if previous is None:
            try:
                # An existing sleeping owner can answer AMT while ordinary
                # services sleep; let usb_stream discover/wake that owner.
                self.camera.state(timeout=1)
                self.camera.system_mode(timeout=1)
            except (RequestTimeout, CameraDisconnected):
                return
            except CameraReplyError as exc:
                if exc.status in (0xE0, 0xE3):
                    return
                raise
        # After our explicit reboot the operation is verify-only. Do not
        # silently skip its required current-boot proof on an early GET.
        self.state = 'preparing_helpers'
        try:
            report = self.camera.install_helpers(timeout=180,
                **({} if previous is None else {'previous_boot_id': previous}))
        except Exception as exc:
            self.preparation = json_value(getattr(exc, 'report', {}))
            raise
        self.preparation = {key: report.get(key) for key in
                            ('installed', 'already_installed', 'autostart_hydration_verified', 'reboot_attempted',
                             'profile', 'boot_id_before', 'boot_id', 'hydration_wait', 'verification_only')}
        if not self.preparation['installed'] or not self.preparation['autostart_hydration_verified']:
            raise RuntimeError('SDK не подтвердил установку и автозапуск служебных скриптов Osmo')
        self.helpers_ready = True
        if report.get('boot_id') and report.get('profile'):
            self._verified_helpers = dict(boot_id=report['boot_id'], profile=report['profile'])
        self._helpers_previous_boot_id = None

    def read_settings(self, *, strict=True):
        cam = self.camera
        if self.stream is not None and self.stream.state == 'SLEEP':
            raise ValueError('Сначала разбуди камеру')
        try:
            field = 'USB-управление'
            if self.stream is not None:
                if self.stream.state != 'AWAKE':
                    raise RuntimeError('SDK не подтвердил активную USB-сессию')
                # Data-first Wake deliberately precedes C1 control readiness.
                # Keep readers running while the SDK pins the new generation
                # and resolves its exact Wake/task proof before ordinary GETs.
                proof = self.stream.verify_control(timeout=20)
                if type(proof) is not dict or proof.get('state') != 'AWAKE':
                    raise RuntimeError('SDK не подтвердил готовность USB-управления')
            field = 'экспозицию'
            exposure = int(cam.exposure_mode().mode)
            field = 'ISO'
            iso = cam.iso()
            field = 'баланс белого'
            wb = cam.white_balance()
            field = 'выдержку'
            try:
                shutter = cam.shutter_setting()
            except CameraReplyError as exc:
                # GET_SHUTTER returns DF when no manual shutter was set in Auto/P.
                # This observed reply is not a failed stream or a requested SET.
                if exc.status != 0xDF or exposure not in (ExposureMode.AUTO, ExposureMode.PROGRAM):
                    raise
                shutter = None
            field = 'цветовой профиль'
            color = int(cam.color().mode)
            field = 'частоту съёмки'
            frame_rate = int(cam.video_format().frame_rate)
        except Exception as exc:
            # Never expose a stale or guessed snapshot as verified settings.
            # Metadata GET failures do not invalidate already healthy data.
            self.settings = {}
            self.settings_error = (f'Не удалось подтвердить USB-управление: {exc}' if field == 'USB-управление'
                                   else f'Не удалось прочитать {field}: {exc}')
            if strict:
                raise RuntimeError(self.settings_error) from exc
            return False
        self.settings = dict(exposure_mode=exposure,
                             iso=0 if iso.automatic else iso.iso,
                             white_balance=0 if wb.mode == WhiteBalanceMode.AUTO else wb.kelvin,
                             color=color, frame_rate=frame_rate,
                             shutter_denominator=shutter.shutter.value if shutter and shutter.shutter.reciprocal else 0)
        self.settings_error = ''
        return True

    def require_session_state(self):
        if self.stream is not None and self.stream.state not in ('AWAKE', 'SLEEP'):
            raise RuntimeError('SDK не подтвердил состояние USB-сессии (' + self.stream.state +
                               '). Выключи и включи только Osmo 360, затем перезапусти драйвер')

    def stop_readers(self):
        self.reader_stop.set()
        for reader in self.readers:
            reader.join(timeout=6)
        if any(reader.is_alive() for reader in self.readers):
            raise RuntimeError('Читатель Osmo не завершился; переход состояния запрещён')
        self.readers = []
        self.enabled = False
        self.frame = None
        with self.condition:
            self.condition.notify_all()
        if self.stream is not None:
            self.capture_stats = json_value(self.stream.stats)

    def start_readers(self):
        self.reader_stop.clear()
        self.capture_error = ''
        self._capture_failure = None
        self.clock = CameraClock(self.host_offset_ns)
        self.frame_times = []
        self.imu_at = None
        self.stats_at = 0
        self.enabled = True
        self.state = 'streaming'
        self.readers = [threading.Thread(target=self.video_loop, name='Osmo video', daemon=True),
                        threading.Thread(target=self.imu_loop, name='Osmo IMU', daemon=True)]
        for reader in self.readers:
            reader.start()

    def close_stream(self):
        try:
            if self.stream is not None:
                self.hold_power('usb_transition')
            self.stop_readers()
            if self.stream is not None:
                report = self.stream.close()
        except Exception as error:
            # stop_readers captured pre-close AWAKE state. Freeze the actual
            # failed cleanup and final collector cause instead, including a
            # failure reading those pre-close stats before cleanup can begin.
            current = self.failure_stats(error)
            current['failure'] = dict(capture=getattr(self, '_capture_failure', None),
                                      close=error_chain(error))
            self.capture_stats = current
            self._close_failure_sequence = getattr(self, '_close_failure_sequence', 0) + 1
            raise
        if self.stream is not None:
            # Preserve the last data counters alongside the verified cleanup
            # result, including the collector's final DIAG/transport report.
            self.capture_stats = dict(self.capture_stats, state=self.stream.state, report=json_value(report))
            if getattr(self, '_capture_failure', None) is not None:
                self.capture_stats['failure'] = dict(capture=self._capture_failure)
            # Camera.close clears SDK ownership only after verified cleanup.
            self.camera.close()
            self.stream = None
            self._power_rebind_allowed = True

    def failure_stats(self, error=None):
        """Only existing SDK/local reports; no query, reconnect or lifecycle."""
        current = dict(getattr(self, 'capture_stats', {}))
        if self.stream is not None:
            try:
                value = json_value(self.stream.stats)
                if isinstance(value, dict):
                    current = value
            except Exception as diagnostic_error:
                current['stats_error'] = error_chain(diagnostic_error)
        report = getattr(error, 'report', None)
        if isinstance(report, dict):
            current['report'] = json_value(report)
        try:
            state = getattr(self.stream, 'state', None)
        except Exception as diagnostic_error:
            current['state_error'] = error_chain(diagnostic_error)
            state = None
        if isinstance(state, str):
            current['state'] = state
        elif isinstance(report, dict) and isinstance(report.get('state'), str):
            current['state'] = report['state']
        return current

    def activate(self):
        self.require_session_state()
        # The resumable SDK owns readiness, standby discovery and wake.
        if self.enabled:
            return
        if self.stream is None:
            self.prepare_power()
            self.connect(wait_ready=False)
            self.prepare_helpers()
            if self.usb_charging is not None and self.preparation.get('reboot_attempted'):
                self.prepare_power(verified_rebind=True)
            self.hold_power('usb_transition')
            self.state = 'starting'
            self.stream = self.camera.usb_stream(fast_sleep=True, retain_ecm=True, queue_size=8,
                                                 imu_options={'queue_size': 32768})
            self.stream.start()
        elif self.stream.state == 'SLEEP':
            self.hold_power('usb_transition')
            self.connect(wait_ready=False)
            self.state = 'waking'
            self.stream.wake()
        # Drain video and 8 kHz IMU immediately. Optional camera GETs stay on
        # the owner thread, but must neither block readers nor fail capture.
        self.start_readers()
        if self.usb_charging is not None:
            self.state = 'checking_power'
            try:
                self.recheck_power()
            except Exception as error:
                try:
                    self.close_stream()
                except Exception as cleanup:
                    if hasattr(error, 'add_note'):
                        error.add_note('Capture cleanup after charging verification failure: ' + str(cleanup))
                raise
        self.state = 'reading_settings'
        self.read_settings(strict=False)
        if self.enabled:
            self.state = 'streaming'

    def apply_settings(self, values):
        self.require_session_state()
        self.hold_power('usb_transition')
        restart = self.enabled
        self.close_stream()
        self.connect()
        try:
            for key, value in values.items():
                if key == 'exposure_mode':
                    self.camera.set_exposure_mode(ExposureModeSetting(ExposureMode(value)))
                elif key == 'iso':
                    self.camera.set_iso(IsoSetting(0) if value == 0 else value)
                elif key == 'white_balance':
                    self.camera.set_white_balance(WhiteBalance(WhiteBalanceMode.AUTO, 50) if value == 0 else value)
                elif key == 'shutter_denominator':
                    self.camera.set_shutter(ShutterSpeed(1, ShutterValue(value, True)))
                elif key == 'color':
                    self.camera.set_color(ColorMode(value))
                elif key == 'frame_rate':
                    self.camera.set_frame_rate(FrameRate(value))
        except Exception:
            # SDK setters verify their own readback/rollback. An unrelated
            # optional GET must not replace the original SET failure.
            self.read_settings(strict=False)
            raise
        self.read_settings(strict=False)
        if restart:
            self.activate()
        else:
            self.prepare_power()
            self.state = 'idle'

    def execute(self, action, settings):
        if action == 'prepare_power':
            self.prepare_power()
        elif action in ('start', 'wake'):
            self.activate()
        elif action in ('stop', 'sleep'):
            self.require_session_state()
            if self.stream is None:
                if action == 'stop':
                    self.state = 'idle' if self.camera and self.camera.connected else 'disconnected'
                    return
                self.activate()
            self.hold_power('sleeping')
            self.state = 'sleeping_camera'
            self.stop_readers()
            if self.stream.state == 'AWAKE':
                self.stream.sleep()
            if self.usb_charging is not None:
                self.recheck_power(sleeping=True)
            self.state = 'sleeping'
        elif action == 'reboot':
            self.require_session_state()
            self.hold_power('usb_transition')
            verified_helpers = getattr(self, '_verified_helpers', None)
            if self.stream is not None and self.stream.state == 'SLEEP':
                self.state = 'waking'
                self.stream.wake()
            self.close_stream()
            self.connect()
            self.state = 'rebooting'
            self.helpers_ready = False
            self._helpers_previous_boot_id = (verified_helpers or {}).get('boot_id')
            self.preparation = dict(installed=verified_helpers is not None,
                already_installed=verified_helpers is not None, autostart_hydration_verified=False,
                reboot_attempted=False, boot_id_before=self._helpers_previous_boot_id,
                profile=(verified_helpers or {}).get('profile'))
            self.camera.reboot(timeout=90)
            if self.usb_charging is not None:
                self.prepare_power(verified_rebind=True)
            self.clock = CameraClock(self.host_offset_ns)
            self.read_settings(strict=False)
            self.state = 'idle'
        elif action == 'configure':
            self.apply_settings(settings)
        elif action == 'refresh':
            self.require_session_state()
            if self.state == 'sleeping' or self.stream is not None and self.stream.state == 'SLEEP':
                raise ValueError('Сначала разбуди камеру')
            if self.stream is None:
                self.connect()
            self.read_settings(strict=False)
            if not self.enabled:
                self.state = 'idle'

    def run(self):
        try:
            while not self.stopping.is_set():
                try:
                    action, settings = self.commands.get(timeout=0.1)
                except queue.Empty:
                    if self.capture_error:
                        error, self.capture_error = self.capture_error, ''
                        self.error = error
                        try:
                            self.close_stream()
                            if self.usb_charging is not None:
                                self.prepare_power(verified_rebind=True)
                        except Exception as exc:
                            self.error += '; cleanup: ' + str(exc)
                        self.state = 'error'
                    continue
                close_failure_sequence = self._close_failure_sequence
                try:
                    self.execute(action, settings)
                except Exception as exc:
                    self.error = str(exc)
                    cause = getattr(exc, 'last_error', None) or exc.__cause__
                    if cause is not None and str(cause) not in self.error:
                        self.error += ': ' + str(cause)
                    if self.stream is not None and self._close_failure_sequence == close_failure_sequence:
                        try:
                            self.capture_stats = json_value(self.stream.stats)
                        except Exception:
                            pass  # Preserve the command's original failure.
                    self.state = 'error'
                    self.on_error(self.error)
                finally:
                    with self.condition:
                        self.busy = ''
                        self.command_started_at = None
        finally:
            cleanup_error = None
            try:
                try:
                    self.close_stream()
                    self.restore_power()
                except Exception as exc:
                    cleanup_error = exc
            finally:
                # Public Camera.close retains uncertain stream ownership while
                # safely retiring local USB control. A deferred stream cleanup
                # must not leave its libusb reader alive at interpreter shutdown.
                try:
                    if self.camera is not None:
                        self.camera.close()
                except Exception as exc:
                    if cleanup_error is None:
                        cleanup_error = exc
                    elif hasattr(cleanup_error, 'add_note'):
                        cleanup_error.add_note('Local Camera close: ' + error_chain(exc)[0]['message'])
            if cleanup_error is not None:
                self.error = str(cleanup_error)
                if (self.usb_charging is not None and self._power_lease is not None
                        and (self._power_lease.receipt is not None or getattr(self._power_lease, 'uncertain', False))):
                    self._close_power_error = cleanup_error
                    self.power = dict(self.power, ready=False, state='unknown',
                                      error='Original charging policy restore is incomplete: ' + str(cleanup_error))
                self.on_error(self.error)

    def failed(self, error):
        current = self.failure_stats()
        self.capture_error = usb_capture_error(current) or str(error)
        self._capture_failure = dict(message=self.capture_error, causes=error_chain(error))
        current['failure'] = dict(capture=self._capture_failure)
        self.capture_stats = current
        self.reader_stop.set()
        self.enabled = False
        self.on_error(self.capture_error)

    def video_loop(self):
        try:
            preview_encoder = PreviewEncoder()
            last = time.monotonic()
            while not self.reader_stop.is_set():
                try:
                    frame = self.stream.read_video(timeout=0.2)
                except TimeoutError:
                    if time.monotonic() - last > 5:
                        raise RuntimeError('Osmo 360 не передаёт видео более 5 секунд')
                    continue
                native_timestamp = frame.timestamp_ns is not None
                if native_timestamp:
                    stamp = self.clock.stamp(frame.timestamp_ns, frame.host_receive_monotonic_ns, 'video')
                else:
                    # SDK explicitly permits pictures without a camera clock.
                    # Keep the raw AU for decoder continuity and the picture for
                    # live viewing. Arrival time is tagged by the separate sink;
                    # it must never enter the synchronized image topic.
                    stamp = frame.host_receive_monotonic_ns + self.host_offset_ns
                    self.timestamp_missing_frames += 1
                jpegs = preview_encoder.for_frame(frame)
                # Before web consumers; raw_h264 is preserved by the recorder.
                if self.on_video_lazy is not None:
                    self.on_video_lazy(frame, jpegs.full, stamp)
                else:
                    callback = self.on_video if native_timestamp else self.on_unstamped_video
                    if callback is not _ignore_video:
                        callback(frame, jpegs.full(), stamp)
                jpeg = jpegs.preview()
                calibration = {k: v for k, v in frame.metadata.items() if k.startswith('calibration_')}
                if calibration:
                    self.calibration = json_value(calibration)
                last = time.monotonic()
                # SDK stats include deep lifecycle reports. Snapshot once per
                # second outside the frame condition, not on every video frame.
                if last - self.stats_at >= 1:
                    self.capture_stats = json_value(self.stream.stats)
                    self.stats_at = last
                with self.condition:
                    self.frame = (jpeg, stamp, last)
                    self.sequence += 1
                    self.frames += 1
                    self.frame_times = [at for at in self.frame_times if last - at < 2] + [last]
                    self.condition.notify_all()
        except Exception as exc:
            if not self.reader_stop.is_set():
                self.failed(exc)

    def imu_loop(self):
        try:
            last = time.monotonic()
            while not self.reader_stop.is_set():
                try:
                    sample = self.stream.read_imu(timeout=0.1)
                except TimeoutError:
                    if time.monotonic() - last > 3:
                        raise RuntimeError('Osmo 360 не передаёт IMU более 3 секунд')
                    continue
                stamp = self.clock.stamp(sample.timestamp_ns, sample.host_receive_monotonic_ns, 'imu')
                self.on_imu(sample, stamp)
                last = self.imu_at = time.monotonic()
                self.imu_samples += 1
        except Exception as exc:
            if not self.reader_stop.is_set():
                self.failed(exc)

    def close(self):
        self.stopping.set()
        self.reader_stop.set()
        with self.condition:
            self.condition.notify_all()
        self.worker.join(timeout=120)
        if self.worker.is_alive():
            raise RuntimeError('Osmo cleanup не завершён; проверь USB перед повторным запуском')
        if self._close_power_error is not None:
            raise USBChargingError('Original USB battery charging policy restoration failed: '
                                   + str(self._close_power_error), self.power) from self._close_power_error
