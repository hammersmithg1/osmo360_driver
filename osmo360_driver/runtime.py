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
    Camera, ColorMode, ExposureMode, ExposureModeSetting, FrameRate, IsoSetting,
    ShutterSpeed, ShutterValue, WhiteBalance, WhiteBalanceMode,
)
from .clock import CameraClock


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


def validate_settings(value):
    if not isinstance(value, dict) or not value:
        raise ValueError('Нужен непустой объект настроек Osmo 360')
    allowed = {'exposure_mode', 'iso', 'shutter_denominator', 'white_balance', 'color', 'frame_rate'}
    if set(value) - allowed:
        raise ValueError('Неизвестные настройки Osmo 360: ' + ', '.join(sorted(set(value) - allowed)))
    for key, number in value.items():
        if type(number) is not int:
            raise ValueError(f'{key}: нужно целое число')
        if key == 'exposure_mode' and number not in (0, 4):
            raise ValueError('Экспозиция: 0 — авто, 4 — вручную')
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
    def __init__(self, on_video=None, on_imu=None, on_error=None, *, camera_factory=Camera, host_offset_ns=None):
        self.on_video = on_video or (lambda *_: None)
        self.on_imu = on_imu or (lambda *_: None)
        self.on_error = on_error or (lambda *_: None)
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
        self.frame_times = []
        self.imu_at = None
        self.enabled = False
        self.busy = self.error = self.capture_error = ''
        self.state = 'disconnected'
        self.settings = {}
        self.calibration = {}
        self.capture_stats = {}
        self.worker = threading.Thread(target=self.run, name='Osmo control', daemon=True)
        self.worker.start()

    def command(self, action, settings=None):
        if action not in ('start', 'stop', 'sleep', 'wake', 'reboot', 'configure', 'refresh'):
            raise ValueError('Неизвестная команда Osmo 360')
        if action == 'configure':
            settings = validate_settings(settings)
        with self.control_lock, self.condition:
            if self.stopping.is_set():
                raise ValueError('Драйвер закрывается')
            if self.busy:
                raise ValueError('Дождись завершения команды Osmo 360: ' + self.busy)
            self.busy, self.error = action, ''
            self.commands.put_nowait((action, settings))
        return self.status()

    def status(self):
        with self.condition:
            now = time.monotonic()
            age = now - self.frame[2] if self.frame else None
            times = self.frame_times
            fps = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 and times[-1] > times[0] else 0
            return dict(enabled=self.enabled, state=self.state, busy=self.busy, error=self.error,
                        frame_age_s=age, capture_fps=round(fps, 1) if age is not None and age < 2 else 0,
                        frames=self.frames, imu_samples=self.imu_samples,
                        imu_age_s=now - self.imu_at if self.imu_at else None,
                        mode={'width': 2048, 'height': 1024, 'fps': 30},
                        projection='dual_fisheye', settings=dict(self.settings),
                        clock=self.clock.status(), calibration=self.calibration,
                        stats=self.capture_stats)

    def latest(self, after=-1, timeout=0):
        with self.condition:
            if timeout:
                self.condition.wait_for(lambda: self.sequence > after or not self.enabled or self.stopping.is_set(), timeout)
            if not self.enabled or not self.frame or self.sequence <= after or time.monotonic() - self.frame[2] > 1:
                return None
            return self.sequence, self.frame

    def connect(self):
        if self.camera is None:
            self.camera = self.camera_factory()
        if not self.camera.connected:
            self.state = 'connecting'
            self.camera.connect(timeout=30)

    def read_settings(self):
        cam = self.camera
        iso, wb, shutter = cam.iso(), cam.white_balance(), cam.shutter_setting()
        self.settings = dict(exposure_mode=int(cam.exposure_mode().mode),
                             iso=0 if iso.automatic else iso.iso,
                             white_balance=0 if wb.mode == WhiteBalanceMode.AUTO else wb.kelvin,
                             color=int(cam.color().mode), frame_rate=int(cam.video_format().frame_rate),
                             shutter_denominator=shutter.shutter.value if shutter.shutter.reciprocal else 0)

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
        self.clock = CameraClock(self.host_offset_ns)
        self.frame_times = []
        self.imu_at = None
        self.enabled = True
        self.state = 'streaming'
        self.readers = [threading.Thread(target=self.video_loop, name='Osmo video', daemon=True),
                        threading.Thread(target=self.imu_loop, name='Osmo IMU', daemon=True)]
        for reader in self.readers:
            reader.start()

    def close_stream(self):
        self.stop_readers()
        if self.stream is not None:
            self.stream.close()
            # Camera.close clears SDK ownership only after verified cleanup.
            self.camera.close()
            self.stream = None

    def activate(self):
        self.connect()
        if self.enabled:
            return
        if self.stream is None:
            self.state = 'starting'
            self.stream = self.camera.usb_stream(fast_sleep=True, retain_ecm=True, queue_size=8,
                                                 imu_options={'queue_size': 32768})
            self.stream.start()
        elif self.stream.state == 'SLEEP':
            self.stream.wake()
        elif self.stream.state != 'AWAKE':
            raise RuntimeError('Неопределённое состояние USB-сессии; перезапусти драйвер')
        # Settings queries belong to the owner thread, before data readers.
        self.read_settings()
        self.start_readers()

    def apply_settings(self, values):
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
        finally:
            # Read actual values even if a later SET failed; never report the
            # requested group as an atomic transaction when firmware is not one.
            self.read_settings()
        if restart:
            self.activate()
        else:
            self.state = 'idle'

    def execute(self, action, settings):
        if action in ('start', 'wake'):
            self.activate()
        elif action == 'stop':
            self.close_stream()
            self.state = 'idle' if self.camera else 'disconnected'
        elif action == 'sleep':
            if self.stream is None:
                self.activate()
            self.stop_readers()
            if self.stream.state == 'AWAKE':
                self.stream.sleep()
            self.state = 'sleeping'
        elif action == 'reboot':
            self.close_stream()
            self.connect()
            self.state = 'rebooting'
            self.camera.reboot(timeout=90)
            self.clock = CameraClock(self.host_offset_ns)
            self.read_settings()
            self.state = 'idle'
        elif action == 'configure':
            self.apply_settings(settings)
        elif action == 'refresh':
            if self.state == 'sleeping':
                raise ValueError('Сначала разбуди камеру')
            self.connect()
            self.read_settings()
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
                        except Exception as exc:
                            self.error += '; cleanup: ' + str(exc)
                        self.state = 'error'
                    continue
                try:
                    self.execute(action, settings)
                except Exception as exc:
                    self.error = str(exc)
                    self.state = 'error'
                    self.on_error(self.error)
                finally:
                    with self.condition:
                        self.busy = ''
        finally:
            try:
                self.close_stream()
                if self.camera is not None:
                    self.camera.close()
            except Exception as exc:
                self.error = str(exc)
                self.on_error(self.error)

    def failed(self, error):
        self.capture_error = str(error)
        self.reader_stop.set()
        self.enabled = False
        self.on_error(self.capture_error)

    def video_loop(self):
        try:
            import cv2
            last = time.monotonic()
            while not self.reader_stop.is_set():
                try:
                    frame = self.stream.read_video(timeout=0.2)
                except TimeoutError:
                    if time.monotonic() - last > 5:
                        raise RuntimeError('Osmo 360 не передаёт видео более 5 секунд')
                    continue
                stamp = self.clock.stamp(frame.timestamp_ns, frame.host_receive_monotonic_ns, 'video')
                ok, encoded = cv2.imencode('.jpg', frame.bgr, [cv2.IMWRITE_JPEG_QUALITY, 92])
                if not ok:
                    raise RuntimeError('Не удалось подготовить JPEG Osmo')
                jpeg = encoded.tobytes()
                # Before web consumers; raw_h264 is preserved by the recorder.
                self.on_video(frame, jpeg, stamp)
                calibration = {k: v for k, v in frame.metadata.items() if k.startswith('calibration_')}
                if calibration:
                    self.calibration = json_value(calibration)
                last = time.monotonic()
                with self.condition:
                    self.frame = (jpeg, stamp, last)
                    self.sequence += 1
                    self.frames += 1
                    self.frame_times = [at for at in self.frame_times if last - at < 2] + [last]
                    self.capture_stats = json_value(self.stream.stats)
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
