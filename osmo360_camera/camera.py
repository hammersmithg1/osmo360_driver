"""Небольшой фасад штатного USB-управления Osmo 360.

Аппаратно проверены настройки/температуры, raw IMU и live preview по USB.
Ни открытие Camera, ни чтение параметров не меняют AMT-авторизацию.
"""

from dataclasses import dataclass, replace
from functools import wraps
import math
import threading
import time

from . import controls as c
from .imu import IMUStream
from .power import SystemMode, build_system_mode_request, decode_system_mode_response
from .temperature import (
    build_temperature_request,
    decode_temperature_response,
    DEFAULT_SENSORS,
)
from .transport import USBError, USBTransport


class CameraError(RuntimeError):
    pass


class SettingApplyError(CameraError):
    def __init__(self, message, *, previous, actual=None, restored=None):
        self.previous, self.actual, self.restored = previous, actual, restored
        super().__init__(message)


class RebootError(CameraError):
    """Warm reboot was not verified; the power command is never retried.

    state/system_mode are the last observations and may predate the command;
    stage and disconnect_observed distinguish them from completed readiness.
    """

    def __init__(
        self,
        message,
        *,
        stage,
        receipt,
        receipt_error,
        disconnect_observed,
        disconnect_evidence,
        elapsed,
        last_error,
        state,
        system_mode,
        read_status=None,
        last_inventory_error=None,
    ):
        self.stage, self.receipt, self.receipt_error = stage, receipt, receipt_error
        self.disconnect_observed, self.elapsed = disconnect_observed, elapsed
        self.disconnect_evidence = disconnect_evidence
        self.last_error, self.state, self.system_mode = last_error, state, system_mode
        self.read_status, self.last_inventory_error = read_status, last_inventory_error
        super().__init__(message)


def _serialized(method):
    """Одна транзакция SET/readback/rollback или lifecycle за раз."""

    @wraps(method)
    def call(self, *args, **kwargs):
        with self._control_lock:
            return method(self, *args, **kwargs)

    return call


@dataclass(frozen=True)
class CameraState:
    raw: int
    work_mode: int

    @property
    def recording_state(self):
        """0 idle, 1 starting, 2 recording, 3 stopping."""
        return self.raw >> 6 & 3

    @property
    def recording(self):
        return self.recording_state == 2

    @property
    def usb_connected(self):
        return bool(self.raw & 2)


@dataclass(frozen=True)
class RebootResult:
    state: CameraState
    system_mode: SystemMode
    receipt: bytes | None
    reconnect_elapsed: float
    receipt_error: str | None = None
    disconnect_evidence: dict | None = None


class Camera:
    """``with Camera() as cam: print(cam.temperatures())``.

    Управляющие запросы занимают только interface4. IMU открывает interface0
    отдельным проверенным сборщиком. USB preview временно добавляет ECM и
    восстанавливает исходную USB-конфигурацию при закрытии.
    """

    def __init__(self, *, transport=None):
        self._transport = transport if transport is not None else USBTransport()
        self._imu = None
        self._usb_preview = None
        self._usb_stream = None
        self._record_owned = False
        self._control_lock = threading.RLock()

    @property
    def connected(self):
        return self._transport.connected

    @property
    def link_speed(self):
        return self._transport.link_speed

    @_serialized
    def connect(self, timeout=None):
        """Open immediately, or wait for USB and read-only service readiness.

        With ``timeout=None`` this preserves the original open-only behavior.
        A finite positive timeout waits for valid state/system-mode replies;
        it neither changes settings nor requires idle/liveview. USB failures
        and the observed boot-service statusE0 are retried; other camera errors
        and malformed replies fail immediately. A live handle is retained
        across request timeouts.

        Failure closes a connection opened by this call, but leaves a
        connection that was already open at entry under its caller's control.
        The timeout error retains the last retryable failure as ``last_error``.
        Native USB calls may overrun the budget; late replies are not success.
        """
        if timeout is None:
            self._transport.open()
            return self
        if (
            type(timeout) not in (int, float)
            or not math.isfinite(timeout)
            or timeout <= 0
        ):
            raise ValueError("connect timeout must be finite and positive, or None")
        deadline = time.monotonic() + timeout
        already_open, opened_here = self._transport.connected, False
        last_error = None

        def remaining():
            value = deadline - time.monotonic()
            if value <= 0:
                error = CameraError(
                    "Camera connection timed out before service readiness"
                )
                error.last_error = last_error
                raise error from last_error
            return value

        try:
            while True:
                remaining()
                try:
                    if not self._transport.connected:
                        self._transport.open()
                        opened_here = not already_open
                    self.state(timeout=min(2, remaining()))
                    self.system_mode(timeout=min(2, remaining()))
                    remaining()
                    return self
                except USBError as error:
                    last_error = error
                except c.CameraReplyError as error:
                    # Hardware after reboot can enumerate before its service
                    # is initialized. E0 is the only observed boot rejection.
                    if error.status != 0xE0:
                        raise
                    last_error = error
                time.sleep(min(0.2, remaining()))
        except BaseException as error:
            if opened_here:
                try:
                    self._transport.close()
                except Exception as cleanup_error:
                    error.camera_cleanup_error = cleanup_error
                    if hasattr(error, "add_note"):
                        error.add_note(
                            "Camera connect cleanup failed: " + str(cleanup_error)
                        )
            raise

    def _get(self, command, payload=b"", *, receiver=1, command_set=2, timeout=None):
        options = {} if timeout is None else {"timeout": timeout}
        return self._transport.request(
            command_set, int(command), payload, receiver=receiver, **options
        ).payload

    def state(self, *, timeout=None):
        payload = self._get(0x70, timeout=timeout)
        if not payload:
            raise c.ControlDecodeError("Пустой ответ состояния камеры")
        if payload[0]:
            raise c.CameraReplyError(payload[0], payload)
        if len(payload) != 6:
            raise c.ControlDecodeError(
                "GET_SYSTEM_STATE должен содержать status+u32+u8"
            )
        return CameraState(int.from_bytes(payload[1:5], "little"), payload[5])

    def mode_profile(self):
        return c.decode_mode_profile(
            self._get(c.CameraCommand.GET_CAMERA_INFO, c.encode_profile_query())
        )

    def lens_mode(self) -> c.LensMode:
        """Read Single Lens / panoramic mode through typed parameter003F."""
        return c.decode_lens_mode(
            self._get(c.CameraCommand.PARAM_OPTION, c.encode_lens_mode_query())
        )

    def capture_parameters(self):
        return c.decode_capture_parameters(
            self._get(c.CameraCommand.GET_CAMERA_INFO, c.encode_camera_info_query(6001))
        )

    def video_format(self):
        return c.decode_video_format(self._get(c.CameraCommand.GET_VIDEO_FORMAT))

    def exposure_mode(self):
        return c.decode_exposure_mode(self._get(c.CameraCommand.GET_EXPOSURE_MODE))

    def white_balance(self):
        return c.decode_white_balance(self._get(c.CameraCommand.GET_WHITE_BALANCE))

    def color(self):
        return c.decode_color(self._get(c.CameraCommand.GET_COLOR))

    def iso(self):
        return c.decode_iso(self._get(c.CameraCommand.GET_ISO))

    def shutter_setting(self):
        """Установленная выдержка. В Program камера может вернуть DF (не задана).

        Текущая выдержка из camera status доступна в capture_parameters().shutter.
        """
        return c.decode_shutter(self._get(c.CameraCommand.GET_SHUTTER))

    def temperatures(self, sensors=DEFAULT_SENSORS):
        names = tuple(sensors)
        payload = self._get(
            0x44, build_temperature_request(names), receiver=0x28, command_set=0
        )
        return {
            r.sensor: r.celsius for r in decode_temperature_response(payload, names)
        }

    def system_mode(self, *, timeout=None):
        """Логические state/scene; не доказательство выдачи кадров или IMU."""
        payload = self._get(
            0x44,
            build_system_mode_request(),
            receiver=0x28,
            command_set=0,
            timeout=timeout,
        )
        return decode_system_mode_response(payload)

    @_serialized
    def reboot(self, timeout=90) -> RebootResult:
        """Verified stock warm reboot; never a cold power-on/off operation.

        Requires Record0, stops the owned IMU collector with verified DIAG-mask
        restoration, and sends system00/44 ``8206`` exactly once with ACK_NOW.
        Receipt01 is diagnostic evidence, not a completed SET acknowledgment.
        Success requires actual USB disappearance (NO_DEVICE or successful
        descriptor inventory proving the old device absent), reconnection and
        Record0/standard_state-liveview readback. A lost receipt can still
        succeed through that evidence; errors retain the receipt and phase.

        ``timeout`` budgets the operation, including IMU cleanup. Native USB
        calls and cleanup joins can finish beyond that budget; subsequent
        checks prevent a late power command or late success. Reconnection and
        readiness reads may be retried; the power command is never retried.
        An IMU stream is not automatically restarted after reboot.
        """
        if (
            type(timeout) not in (int, float)
            or not math.isfinite(timeout)
            or timeout <= 0
        ):
            raise ValueError("reboot timeout must be finite and positive")
        stream = getattr(self, "_usb_stream", None)
        if stream is not None and stream.owns_camera_resources:
            raise CameraError(
                "Сначала закройте USB stream; для его recovery используйте stream.recover()"
            )
        started = time.monotonic()
        deadline = started + timeout
        stage, receipt, receipt_error = "preflight", None, None
        disconnected, last_error, state, mode = False, None, None, None
        disconnect_evidence = None
        sent_at = None

        def failure(message, error=None):
            detail = error or last_error or getattr(self._transport, "failure", None)
            return RebootError(
                message,
                stage=stage,
                receipt=receipt,
                receipt_error=receipt_error,
                disconnect_observed=disconnected,
                disconnect_evidence=disconnect_evidence,
                elapsed=time.monotonic() - started,
                last_error=detail,
                state=state,
                system_mode=mode,
                read_status=getattr(self._transport, "read_failure_status", None),
                last_inventory_error=getattr(
                    self._transport, "last_inventory_error", None
                ),
            )

        def remaining():
            value = deadline - time.monotonic()
            if value <= 0:
                raise failure(f"Warm reboot timed out during {stage}")
            return value

        try:
            if self._usb_preview is not None:
                stage = "preview_stop"
                self._usb_preview.stop()
                remaining()
                stage = "preflight"
            state = self.state(timeout=min(2, remaining()))
            if state.recording_state != 0:
                raise failure("Warm reboot requires idle Record0")
            if self._imu is not None:
                stage = "imu_stop"
                imu = self._imu
                report = imu.stop(timeout=remaining())
                if (report is None and imu.pid is not None) or (
                    report is not None
                    and (
                        report.get("restored") is not True
                        or report.get("quiescent") is not True
                        or report.get("errors")
                    )
                ):
                    raise failure(
                        "Owned IMU mask restoration and DIAG quiescence were not verified"
                    )
                self._imu = None
            # Recheck after IMU cleanup; another camera controller may have
            # started recording while the collector restored its mask.
            stage = "preflight"
            state = self.state(timeout=min(2, remaining()))
            if state.recording_state != 0:
                raise failure("Camera left Record0 before warm reboot")

            stage = "receipt"
            receipt_timeout = min(2, remaining())
            sent_at = time.monotonic()
            try:
                receipt = self._transport.request(
                    0,
                    0x44,
                    b"\x82\x06",
                    receiver=0x28,
                    timeout=receipt_timeout,
                    ack_type=1,
                ).payload
            except USBError as error:
                # The camera can disconnect before its receipt reaches us.
                # Retain the error and require independent reboot evidence.
                receipt_error = f"{type(error).__name__}: {error}"
                last_error = error

            stage = "disconnect"
            if not self._transport.wait_disconnected(remaining()):
                raise failure("Warm reboot did not produce an observed USB disconnect")
            disconnected = True
            disconnect_evidence = getattr(self._transport, "disconnect_evidence", None)
            stage = "close"
            self._transport.close()  # Only after independently verified disappearance.

            stage = "reconnect"
            while True:
                remaining()
                try:
                    if not self._transport.connected:
                        self._transport.open()
                    state = self.state(timeout=min(2, remaining()))
                    mode = self.system_mode(timeout=min(2, remaining()))
                    if (
                        state.recording_state == 0
                        and mode.state == "standard_state"
                        and mode.scene == "liveview"
                    ):
                        remaining()
                        self._record_owned = False
                        return RebootResult(
                            state,
                            mode,
                            receipt,
                            time.monotonic() - sent_at,
                            receipt_error,
                            disconnect_evidence,
                        )
                    last_error = CameraError(
                        "Camera is connected but idle/liveview is not ready"
                    )
                except (USBError, c.ControlDecodeError, c.CameraReplyError) as error:
                    last_error = error
                time.sleep(min(0.2, remaining()))
        except RebootError:
            raise
        except Exception as error:
            raise failure(
                f"Warm reboot failed during {stage}: {error}", error
            ) from error
        except BaseException as error:
            if hasattr(error, "add_note"):
                error.add_note(
                    f"Warm reboot interrupted during {stage}; "
                    f"command attempted={sent_at is not None}; "
                    f"USB disconnect observed={disconnected}; no command retry"
                )
            raise

    @_serialized
    def imu(self, **options):
        """Вернуть контекст raw gyro/accel; вызов сам по себе сбор не запускает."""
        if self._imu is not None:
            raise CameraError("Эта Camera уже создала IMUStream; сначала закройте её")
        self._imu = IMUStream(**options)
        return self._imu

    @_serialized
    def usb_preview(self, **options):
        """Контекст кадров 360° по кабелю, без записи и переключения Wi-Fi.

        Запускайте preview перед ``cam.imu()``: вход/выход меняют USB-режим.
        ``decode=True`` требует optional requirements-video.txt.
        """
        from .usb_preview import USBPreview

        if self._usb_preview is not None:
            raise CameraError(
                "Эта Camera уже владеет USB preview; сначала закройте его"
            )
        self._usb_preview = USBPreview(self, **options)
        return self._usb_preview

    @_serialized
    def usb_stream(
        self,
        *,
        decode=True,
        queue_size=1,
        imu_options=None,
        timeout=60,
        fast_sleep=False,
        retain_ecm=False,
    ):
        """Контекст USB-видео + raw IMU с методами ``sleep()`` / ``wake()``.

        Камера должна быть включена, с аккумулятором и в Record0. Первая
        подготовка загружает проверенные RAM-компоненты; последующие циклы
        используют их повторно. Закрытие контекста сохраняет штатный сон,
        если ``sleep()`` уже завершён, и не перезагружает камеру.
        ``fast_sleep=True`` возвращает подтверждение сна до завершения
        фоновой подготовки USB; ``wake()`` и ``close()`` дожидаются её.
        Экспериментальный Linux-путь ``retain_ecm=True`` вместе с
        ``fast_sleep=True`` сохраняет USB-канал во сне и восстанавливает
        его непосредственно при пробуждении.
        """
        if getattr(self, "_usb_stream", None) is not None:
            raise CameraError("Эта Camera уже владеет USB stream; сначала закройте его")
        if self._usb_preview is not None or self._imu is not None:
            raise CameraError("USB stream требует Camera без другого preview или IMU")
        from ._stream_resume import build_resumable_stream

        self._usb_stream = build_resumable_stream(
            self,
            decode=decode,
            queue_size=queue_size,
            imu_options=imu_options,
            timeout=timeout,
            fast_sleep=fast_sleep,
            retain_ecm=retain_ecm,
        )
        return self._usb_stream

    def install_helpers(self, *, timeout=180):
        """Установить постоянный код и проверить автозагрузку после reboot.

        Вызывать один раз вне активного потока. Состояние сессии и параметры
        камеры не сохраняются в пакете; после каждого boot они создаются заново.
        Повторный вызов для проверенной установки не перезагружает камеру.
        """
        from ._helper_lifecycle import install

        return install(self, timeout=timeout)

    def remove_helpers(self, *, timeout=180):
        """Удалить наши компоненты и проверить штатное состояние после reboot.

        Останавливает принадлежащие этой Camera потоки, возвращает исходный
        загрузочный скрипт и очищает наш код. Настройки/медиа/калибровки не удаляет.
        """
        from ._helper_lifecycle import remove

        return remove(self, timeout=timeout)

    @_serialized
    def _apply(
        self,
        command,
        value,
        getter,
        encoder,
        *,
        allow_missing_previous=False,
        equivalent=None,
    ):
        matches = equivalent or (lambda actual, expected: actual == expected)
        previous = None
        try:
            previous = getter()
        except c.CameraReplyError as exc:
            if not allow_missing_previous or exc.status != 0xDF:
                raise
        encoded = encoder(value)  # validate before any camera write
        actual = None
        attempted = False
        try:
            attempted = True
            c.decode_set_ack(self._get(command, encoded))
            # Some camera changes apply asynchronously. Require a real readback.
            end = time.monotonic() + 2
            while True:
                actual = getter()
                if matches(actual, value):
                    return actual
                if time.monotonic() >= end:
                    raise CameraError(
                        f"ACK получен, но readback отличается: {actual!r}"
                    )
                time.sleep(0.05)
        except BaseException as exc:
            restored = None
            if attempted and previous is not None:
                try:
                    c.decode_set_ack(self._get(command, encoder(previous)))
                    restored = matches(getter(), previous)
                except Exception:
                    restored = False
            if not isinstance(exc, Exception):
                if hasattr(exc, "add_note"):
                    exc.add_note(
                        f"Восстановление настройки после прерывания: {restored}"
                    )
                raise
            raise SettingApplyError(
                str(exc), previous=previous, actual=actual, restored=restored
            ) from exc

    def set_video_format(self, value: c.VideoFormat):
        return self._apply(
            c.CameraCommand.SET_VIDEO_FORMAT,
            value,
            self.video_format,
            c.encode_video_format,
        )

    @_serialized
    def set_frame_rate(self, value: c.FrameRate):
        """Использует явный FPS enum: например FPS30 и FPS30_TRUE различаются."""
        if not isinstance(value, c.FrameRate):
            raise TypeError(
                "Нужен FrameRate, чтобы не спутать wire code и физические fps"
            )
        return self.set_video_format(
            replace(self.video_format(), frame_rate=int(value))
        )

    def set_mode_profile(self, value: c.ModeProfileSetting | c.ModeProfile):
        if isinstance(value, c.ModeProfile):
            value = c.ModeProfileSetting(int(value))
        return self._apply(
            c.CameraCommand.SET_MODE_PROFILE,
            value,
            self.mode_profile,
            c.encode_mode_profile,
        )

    @_serialized
    def set_lens_mode(self, value: c.LensMode) -> c.LensMode:
        """Switch Single Lens / panoramic mode while the camera is Record0.

        The camera automatically changes its shooting profile on this switch.
        For video, explicitly call set_mode_profile(ModeProfile.VIDEO_NORMAL)
        after SINGLE_LENS, or set_mode_profile(ModeProfile.VIDEO_PANO) after
        PANORAMIC. The automatic profile may be a photo profile.
        Read mode_profile()/video_format() afterwards; this method does not
        promise to retain either setting. On failure rollback verifies only
        the previous lens mode, not restoration of the old profile/format.
        """
        if not isinstance(value, c.LensMode):
            raise TypeError("Нужен LensMode.SINGLE_LENS или LensMode.PANORAMIC")
        if self.state().recording_state != 0:
            raise CameraError("Переключение объективов требует Record0")
        return self._apply(
            c.CameraCommand.PARAM_OPTION, value, self.lens_mode, c.encode_lens_mode
        )

    def set_exposure_mode(self, value: c.ExposureModeSetting):
        return self._apply(
            c.CameraCommand.SET_EXPOSURE_MODE,
            value,
            self.exposure_mode,
            c.encode_exposure_mode,
        )

    @_serialized
    def set_white_balance(self, value: c.WhiteBalance | int):
        """Verify mode and, for MANUAL6 only, the requested temperature.

        AUTO/other modes retain a saved manual temperature in native GET;
        that stored value and the legacy/extended layout do not reject a mode
        change. Return the actual readback, including its saved temperature.
        """
        # An integer is Kelvin; use the firmware's current legacy/extended layout.
        if type(value) is int:
            value = replace(
                c.WhiteBalance.manual(value), extended=self.white_balance().extended
            )
        return self._apply(
            c.CameraCommand.SET_WHITE_BALANCE,
            value,
            self.white_balance,
            c.encode_white_balance,
            equivalent=lambda actual, expected: (
                actual.mode == expected.mode
                and (
                    expected.mode != c.WhiteBalanceMode.MANUAL
                    or actual.temperature_code == expected.temperature_code
                )
            ),
        )

    def set_color(self, value: c.ColorSetting | c.ColorMode):
        if isinstance(value, c.ColorMode):
            value = c.ColorSetting(int(value))
        return self._apply(c.CameraCommand.SET_COLOR, value, self.color, c.encode_color)

    def set_iso(self, value: c.IsoSetting | int):
        if type(value) is int:
            value = c.IsoSetting.manual(value)
        return self._apply(c.CameraCommand.SET_ISO, value, self.iso, c.encode_iso)

    def set_shutter(self, value: c.ShutterSpeed):
        def matches(actual, expected):
            # Legacy SET may be returned in V2 GET layout. Only compare V2
            # metadata when the caller explicitly requested those fields.
            return (
                actual.mode == expected.mode
                and actual.shutter.seconds == expected.shutter.seconds
                and (
                    expected.shutter_type is None
                    or (actual.shutter_type, actual.parameter_flags)
                    == (expected.shutter_type, expected.parameter_flags)
                )
            )

        return self._apply(
            c.CameraCommand.SET_SHUTTER,
            value,
            self.shutter_setting,
            c.encode_shutter,
            allow_missing_previous=True,
            equivalent=matches,
        )

    def _wait_record_state(self, expected, timeout=5):
        end = time.monotonic() + timeout
        while True:
            state = self.state()
            if state.recording_state == expected:
                return state
            if time.monotonic() >= end:
                raise CameraError(
                    f"Ожидался Record{expected}, получен Record{state.recording_state}"
                )
            time.sleep(0.05)

    @_serialized
    def start_recording(self):
        if self.state().recording_state != 0:
            raise CameraError("Камера уже снимает или меняет состояние записи")
        self._record_owned = True
        try:
            c.decode_set_ack(self._get(2, b"\1"))
            return self._wait_record_state(2)
        except BaseException as error:
            try:
                self.stop_recording()
            except Exception as cleanup_error:
                if hasattr(error, "add_note"):
                    error.add_note(
                        f"STOP после ошибки START не подтверждён: {cleanup_error}"
                    )
            raise

    @_serialized
    def stop_recording(self):
        try:
            state = self.state()
        except Exception:
            # A lost state reply must not prevent STOP after a delivered START.
            state = None
        if state is None or state.recording_state != 0:
            c.decode_set_ack(self._get(2, b"\0"))
            state = self._wait_record_state(0)
        self._record_owned = False
        return state

    def close(self):
        # Session lifecycle takes its own lock before using camera control.
        # Do not hold the control lock while asking it to close: a concurrent
        # sleep/wake may still need that lock to finish its native operation.
        stream = getattr(self, "_usb_stream", None)
        if stream is not None:
            failure = None
            try:
                stream.close()
            except Exception as error:
                failure = error
            if not stream.cleanup_safe:
                # An unconfirmed prepared apply may run after this call.
                # Ordinary preview.stop() could race it before armed exists.
                # Preserve the session owner and only release local control.
                try:
                    # A preparation worker may still own camera control after
                    # its bounded join timed out. Do not wait on that lock or
                    # close a transport underneath its in-flight transaction.
                    # The session remains owned and close() can be retried.
                    acquired = self._control_lock.acquire(blocking=False)
                    if not acquired:
                        raise CameraError(
                            "USB control is still owned by the stream; local close deferred"
                        )
                    try:
                        self._transport.close()
                    finally:
                        self._control_lock.release()
                except Exception as error:
                    if failure is None:
                        failure = error
                    elif hasattr(failure, "add_note"):
                        failure.add_note("Local USB close: " + str(error))
                if failure is not None:
                    raise failure
                raise CameraError(
                    "USB stream cleanup is uncertain; its owner is retained"
                )
            self._usb_stream = None
            try:
                self._close_resources()
            except Exception as error:
                if failure is None:
                    raise
                if hasattr(failure, "add_note"):
                    failure.add_note("Camera resource cleanup: " + str(error))
            if failure is not None:
                raise failure
            return
        self._close_resources()

    @_serialized
    def _close_resources(self):
        errors = []
        if self._record_owned:
            try:
                self.stop_recording()
            except Exception as exc:
                errors.append(f"STOP: {exc}")
        if self._usb_preview is not None:
            try:
                self._usb_preview.stop()
            except Exception as exc:
                errors.append(f"Preview: {exc}")
        if self._imu is not None:
            try:
                imu = self._imu
                report = imu.stop()
                if (report is None and imu.pid is not None) or (
                    report is not None
                    and (
                        report.get("restored") is not True
                        or report.get("quiescent") is not True
                        or report.get("errors")
                    )
                ):
                    raise CameraError(
                        "Owned IMU mask restoration and DIAG quiescence were not verified"
                    )
                self._imu = None
            except Exception as exc:
                errors.append(f"IMU: {exc}")
        try:
            self._transport.close()
        except Exception as exc:
            errors.append(f"USB: {exc}")
        if errors:
            raise CameraError("; ".join(errors))

    def __enter__(self):
        return self.connect()

    def __exit__(self, exc_type, exc, tb):
        try:
            self.close()
        except Exception as cleanup_error:
            if exc is None:
                raise
            # Keep the original structured failure and disconnect evidence.
            # Python3.10 has no add_note, so retain
            # the actual cleanup exception as accessible diagnostics as well.
            exc.camera_cleanup_error = cleanup_error
            add_note = getattr(exc, "add_note", None)
            if callable(add_note):
                add_note("Camera context cleanup failed: " + str(cleanup_error))
        return False
