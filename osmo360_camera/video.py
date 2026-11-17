"""Network preview only; USB setup and the camera lease belong to the facade.

Every encoded access unit reaches the H.264 decoder. Only decoded pictures
are dropped from the bounded consumer queue. PyAV is imported on start only
when decode=True; NumPy is needed only for the lazy .bgr conversion.
"""

from __future__ import annotations

from collections import deque
from copy import deepcopy
from dataclasses import dataclass, field
from fractions import Fraction
import hashlib
import math
import re
import threading
import time

from .vendor.datalink import Datalink, poke
from .vendor.preview_metadata import nals, records_from_video


class VideoStreamError(RuntimeError):
    """Preview network, metadata, decoder or shutdown failure."""


@dataclass
class VideoFrame:
    timestamp_ns: int | None
    host_receive_monotonic_ns: int
    image: object | None
    raw_h264: bytes
    metadata: dict = field(default_factory=dict)
    _bgr: object = field(default=None, init=False, repr=False)

    @property
    def bgr(self):
        """Lazy BGR ndarray; timestamp always remains the camera's raw clock."""
        if self.image is None:
            raise VideoStreamError("BGR requires a stream started with decode=True")
        if self._bgr is None:
            self._bgr = self.image.to_ndarray(format="bgr24")
        return self._bgr


_START = re.compile(rb"\x00\x00\x00?\x01")
_TIME_BASE = Fraction(1, 1_000_000_000)


def _metadata_diagnostic(raw, error):
    """Bounded host-side evidence only; never accepts or repairs a bad AU."""
    result = dict(
        error_type=type(error).__name__,
        error_message=str(error),
        access_unit_bytes=len(raw),
        access_unit_sha256=hashlib.sha256(raw).hexdigest(),
    )
    timestamp_sei = []
    scanned = 0
    for offset, nal in nals(raw):
        if not nal or nal[0] & 31 != 6:
            continue
        if scanned == 8:
            result["sei_scan_truncated"] = True
            break
        scanned += 1
        entry = dict(
            nal_offset=offset,
            nal_bytes=len(nal),
            nal_sha256=hashlib.sha256(nal).hexdigest(),
            nal_hex=nal[:8192].hex(),
            truncated=len(nal) > 8192,
        )
        # Each NAL is parsed independently only after the normal parser failed.
        # This locates the actual malformed SEI without retaining image slices.
        if len(nal) > 8192:
            entry["parse_skipped"] = "diagnostic size limit"
            timestamp_sei.append(entry)
            continue
        try:
            records = list(records_from_video(b"\0\0\0\1" + nal))
        except ValueError as cause:
            entry["parser_error"] = str(cause)
            result.update(offending_sei=entry, sei_nals_scanned=scanned)
            return result
        clocks = [r["frame_clock_ns_raw"] for r in records if "frame_clock_ns_raw" in r]
        if clocks and len(timestamp_sei) < 4:
            timestamp_sei.append(dict(entry, frame_clocks_ns_raw=clocks))
    # All individual SEIs can be valid while their AU clocks conflict.
    result.update(timestamp_sei=timestamp_sei[:4], sei_nals_scanned=scanned)
    return result


def _missing_timestamp_diagnostic(raw, records, received):
    """Bounded metadata evidence; no image slices or guessed camera clocks."""
    result = dict(
        received_monotonic_ns=received, access_unit_bytes=len(raw),
        access_unit_sha256=hashlib.sha256(raw).hexdigest(),
        nal_types=[], f5_sei=[], metadata_records=[],
    )
    for offset, nal in nals(raw):
        if len(result['nal_types']) == 64:
            result['nal_scan_truncated'] = True
            break
        if not nal:
            continue
        kind = nal[0] & 31
        result['nal_types'].append(kind)
        if kind == 6 and len(nal) > 1 and nal[1] == 245:
            if len(result['f5_sei']) == 2:
                result['f5_scan_truncated'] = True
                continue
            result['f5_sei'].append(dict(
                nal_offset=offset, nal_bytes=len(nal),
                nal_sha256=hashlib.sha256(nal).hexdigest(),
                nal_hex=nal[:8192].hex(), truncated=len(nal) > 8192,
            ))
    fields = ('motion_version', 'motion_declared_body_length', 'motion_body_length',
              'motion_decode_error', 'frame_clock_available', 'header_hex')
    for record in records[:4]:
        entry = {key: record[key] for key in fields if key in record}
        raw_tlv = record.get('raw_tlv_hex', '')
        entry.update(raw_tlv_hex=raw_tlv[:8192], raw_tlv_truncated=len(raw_tlv) > 8192)
        result['metadata_records'].append(entry)
    if len(records) > 4:
        result['metadata_records_truncated'] = True
    result['reason'] = ('no_f5_sei' if not records else
                        'frame_clock_unavailable' if any(r.get('frame_clock_available') is False for r in records) else
                        'unsupported_motion_version' if any(r.get('motion_decode_error') == 'unsupported_version' for r in records) else
                        'no_frame_clock_in_sei')
    return result


def _decoder_bytes(data):
    """Remove whole F5 SEI NALs only, preserving every other Annex-B byte.

    DJI counts escaped SEI size differently from FFmpeg. The original AU is
    retained on VideoFrame; removing its timestamp NAL avoids decoder warnings.
    """
    starts = list(_START.finditer(data))
    if not starts:
        return data
    result = bytearray(data[: starts[0].start()])
    for index, start in enumerate(starts):
        end = starts[index + 1].start() if index + 1 < len(starts) else len(data)
        nal = data[start.end() : end]
        kind, pos = 0, 1
        if nal and nal[0] & 31 == 6:
            while pos < len(nal) and nal[pos] == 255:
                kind += 255
                pos += 1
            if pos < len(nal):
                kind += nal[pos]
                if kind == 245:
                    continue
        result += data[start.start() : end]
    return bytes(result)


class VideoStream:
    """Asynchronous Osmo Action-style preview on an already configured link.

    start() returns immediately. read(timeout) waits for a frame, raises
    TimeoutError on timeout, VideoStreamError on failure, or StopIteration
    after a clean stop and queue drain. Instances are single-use. This class
    never configures a network interface, USB function, lease or recording.
    The facade can supply local_address/interface to bind both UDP and TCP;
    an unavailable or denied interface binding fails instead of using a route.
    pause()/resume() retain the datalink and decoder; they do not request
    camera Sleep/Wake or change USB/network/lease ownership.
    """

    def __init__(
        self,
        host="192.168.253.1",
        decode=True,
        queue_size=1,
        *,
        local_address=None,
        interface=None,
    ):
        if not isinstance(host, str) or not host or "\0" in host:
            raise ValueError("host must be a nonempty address")
        if type(decode) is not bool:
            raise ValueError("decode must be a bool")
        if type(queue_size) is not int or not 1 <= queue_size <= 256:
            raise ValueError("queue_size must be an integer in1..256")
        self.host, self.decode, self.queue_size = host, decode, queue_size
        self.local_address, self.interface = local_address, interface
        self._condition = threading.Condition()
        self._pause_lock = threading.Lock()
        self._publication_paused = False
        self._publication_epoch = 0
        self._queue = deque()
        self._pending = {}
        self._unstamped = deque()
        self._calibration = None
        self._started = self._closed = self._stop_requested = self._done = False
        self._thread = self._link = self._decoder = self._av = self._error = None
        self._metadata_error = None
        self._missing_timestamp = self._first_missing_timestamp = None
        self._counts = dict(
            encoded_units=0,
            encoded_bytes=0,
            decoded_frames=0,
            published_frames=0,
            dropped_frames=0,
            missing_timestamps=0,
            metadata_errors=0,
            suppressed_frames=0,
            paused_queue_discards=0,
        )

    @property
    def calibration(self):
        """Copy of the first raw calibration_* SEI fields, or None if unseen.

        Retained independently of image queue drops. These are versioned raw
        fields, not an asserted optical model or camera-to-IMU transform.
        """
        with self._condition:
            return deepcopy(self._calibration)

    @property
    def stats(self):
        with self._condition:
            cause = self._error.__cause__ if self._error else None
            error_cause = (type(cause).__name__ + ": " + str(cause)) if cause else None
            if error_cause is None and self._metadata_error:
                error_cause = (
                    self._metadata_error["error_type"]
                    + ": "
                    + self._metadata_error["error_message"]
                )
            return dict(
                self._counts,
                queued_frames=len(self._queue),
                paused=self._publication_paused,
                started=self._started,
                stopped=self._done,
                error=str(self._error) if self._error else None,
                error_cause=error_cause,
                metadata_error=deepcopy(self._metadata_error),
                missing_timestamp=deepcopy(self._missing_timestamp),
                first_missing_timestamp=deepcopy(self._first_missing_timestamp),
                network=dict(self._link.stats) if self._link else {},
            )

    def start(self):
        with self._condition:
            if self._closed:
                raise VideoStreamError("A stopped VideoStream cannot be restarted")
            if self._started:
                return self
            if self.decode:
                try:
                    import av
                except ImportError as error:
                    raise VideoStreamError(
                        "decode=True requires the optional PyAV package"
                    ) from error
                self._av = av
                self._decoder = av.CodecContext.create("h264", "r")
                self._decoder.thread_count = 1
            try:
                self._link = Datalink(
                    host=self.host,
                    on_video=self._on_video,
                    liveview="action",
                    local_address=self.local_address,
                    interface=self.interface,
                )
            except (OSError, ValueError) as error:
                raise VideoStreamError(
                    "Cannot bind preview socket: " + str(error)
                ) from error
            self._thread = threading.Thread(
                target=self._run, name="osmo-video", daemon=True
            )
            self._started = True
            self._thread.start()
        return self

    def _run(self):
        try:
            poke(self.host, local_address=self.local_address, interface=self.interface)
            if not self._stop_requested:
                self._link.run()
            if not self._stop_requested:
                raise VideoStreamError("Preview datalink stopped unexpectedly")
        except BaseException as error:
            if not (self._stop_requested and isinstance(error, OSError)):
                self._error = error
        finally:
            try:
                self._link.stop()
            except BaseException as error:
                if self._error is None:
                    self._error = error
            finally:
                with self._condition:
                    self._done = True
                    self._condition.notify_all()

    def _publish(self, frame, *, epoch=None):
        with self._condition:
            if self._stop_requested:
                return
            if self._publication_paused or (
                epoch is not None and epoch != self._publication_epoch
            ):
                self._counts["suppressed_frames"] += 1
                return
            if len(self._queue) == self.queue_size:
                self._queue.popleft()
                self._counts["dropped_frames"] += 1
            self._queue.append(frame)
            self._counts["published_frames"] += 1
            self._condition.notify_all()

    def _on_video(self, data, first_packet):
        # This is completion of one reassembled AU, not its first packet or
        # exposure time. Preserve the camera clock separately for SLAM.
        received = time.monotonic_ns()
        raw = bytes(data)
        with self._condition:
            epoch = -1 if self._publication_paused else self._publication_epoch
            self._counts["encoded_units"] += 1
            self._counts["encoded_bytes"] += len(raw)
        try:
            records = list(records_from_video(raw))
            stamps = [
                r["frame_clock_ns_raw"] for r in records if "frame_clock_ns_raw" in r
            ]
            if len(stamps) > 1 and len(set(stamps)) != 1:
                raise ValueError("Multiple frame timestamps in one access unit")
            stamp = stamps[0] if stamps else None
        except ValueError as error:
            try:
                diagnostic = _metadata_diagnostic(raw, error)
                diagnostic["received_monotonic_ns"] = received
            except Exception as diagnostic_error:
                # Diagnostics must preserve the original parser failure.
                diagnostic = dict(
                    error_type=type(error).__name__,
                    error_message=str(error),
                    diagnostic_error=repr(diagnostic_error),
                )
            with self._condition:
                self._counts["metadata_errors"] += 1
                self._metadata_error = diagnostic
            raise VideoStreamError(
                "Invalid preview timestamp metadata: " + str(error)
            ) from error
        frame_metadata = {}
        for record in records:
            calibration = {
                key: value
                for key, value in record.items()
                if key.startswith("calibration_")
            }
            if calibration:
                with self._condition:
                    if self._calibration is None:
                        self._calibration = deepcopy(calibration)
            # Keep actual raw clocks, version/length information, header and
            # opaque TLV bytes. Fused quaternion arrays are deliberately absent.
            frame_metadata.update(
                {key: value for key, value in record.items() if key != "quaternions"}
            )
        if stamp is None:
            diagnostic = _missing_timestamp_diagnostic(raw, records, received)
            if self._link is not None:
                diagnostic['network'] = {key: self._link.stats[key] for key in
                    ('truncated_pictures', 'video_packets', 'video_bytes') if key in self._link.stats}
            with self._condition:
                self._counts["missing_timestamps"] += 1
                self._missing_timestamp = diagnostic
                if self._first_missing_timestamp is None:
                    self._first_missing_timestamp = deepcopy(diagnostic)
        if not self.decode:
            self._publish(
                VideoFrame(stamp, received, None, raw, frame_metadata), epoch=epoch
            )
            return
        metadata = (stamp, received, raw, frame_metadata, epoch)
        if stamp is not None:
            self._pending.setdefault(stamp, deque()).append(metadata)
        else:
            self._unstamped.append(metadata)
        packet = self._av.Packet(_decoder_bytes(raw))
        packet.pts, packet.time_base = stamp, _TIME_BASE
        for image in self._decoder.decode(packet):
            stamp_out = (
                int(image.pts * image.time_base * 1_000_000_000)
                if image.pts is not None
                else None
            )
            if stamp_out is None:
                if not self._unstamped:
                    raise VideoStreamError(
                        "Decoded frame lacks its access-unit timestamp"
                    )
                meta = self._unstamped.popleft()
            else:
                entries = self._pending.get(stamp_out)
                if not entries:
                    raise VideoStreamError("Decoded PTS does not match its access unit")
                meta = entries.popleft()
                if not entries:
                    del self._pending[stamp_out]
            with self._condition:
                self._counts["decoded_frames"] += 1
            self._publish(
                VideoFrame(meta[0], meta[1], image, meta[2], meta[3]), epoch=meta[4]
            )
        if sum(map(len, self._pending.values())) + len(self._unstamped) > 64:
            raise VideoStreamError(
                "Decoder retained more than64 access units without matching frames"
            )

    def pause(self, timeout=2, *, keep_answer_requests=True):
        """Suppress publication and drain active datalink DATA, retaining both.

        Return True only after the datalink pause barrier. False is not a
        permission to sleep: resume(trigger_encoder=False) before continuing.
        All arriving AUs still reach the decoder. Queued pictures and outputs
        of pre-pause/paused AUs cannot become a first frame after resume.
        """
        if (
            type(timeout) not in (int, float)
            or not math.isfinite(timeout)
            or timeout <= 0
        ):
            raise ValueError("pause timeout must be finite and positive")
        if type(keep_answer_requests) is not bool:
            raise ValueError("keep_answer_requests must be boolean")
        deadline = time.monotonic() + timeout
        if not self._pause_lock.acquire(timeout=timeout):
            return False
        try:
            with self._condition:
                if not self._started or self._closed or self._done:
                    raise VideoStreamError("Pause requires a running VideoStream")
                self._publication_paused = True
                self._publication_epoch += 1
                self._counts["paused_queue_discards"] += len(self._queue)
                self._queue.clear()
                link = self._link
            remaining = deadline - time.monotonic()
            return remaining > 0 and link.pause(
                remaining, keep_answer_requests=keep_answer_requests
            )
        finally:
            self._pause_lock.release()

    def resume(self, *, trigger_encoder=True):
        """After native wake, resume the same link/decoder without bootstrap.

        Publication freshness is an AU-entry epoch, not an exposure-time or
        hardware-wake claim. Callers must check native timestamps and camera
        state; a transport backlog is not turned into a new capture timestamp.
        """
        if type(trigger_encoder) is not bool:
            raise ValueError("trigger_encoder must be boolean")
        with self._pause_lock:
            with self._condition:
                if not self._started or self._closed or self._done:
                    raise VideoStreamError("Resume requires a running VideoStream")
                if self._publication_paused:
                    self._publication_epoch += 1
                    self._counts["paused_queue_discards"] += len(self._queue)
                    self._queue.clear()
                    self._publication_paused = False
                link = self._link
            try:
                result = link.resume(trigger_encoder=trigger_encoder)
            except BaseException:
                with self._condition:
                    self._publication_paused = True
                    self._publication_epoch += 1
                    self._counts["paused_queue_discards"] += len(self._queue)
                    self._queue.clear()
                raise
            with self._condition:
                self._condition.notify_all()
            return result

    def read(self, timeout=None):
        if timeout is not None and (not math.isfinite(timeout) or timeout < 0):
            raise ValueError("timeout must be finite and nonnegative, or None")
        with self._condition:
            if not self._started:
                raise VideoStreamError("Start the VideoStream before reading")
            deadline = time.monotonic() + timeout if timeout is not None else math.inf
            while True:
                if self._queue:
                    return self._queue.popleft()
                if self._done:
                    if self._error:
                        raise VideoStreamError(str(self._error)) from self._error
                    raise StopIteration
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("No video frame before timeout")
                self._condition.wait(None if math.isinf(remaining) else remaining)

    def stop(self, timeout=5):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("stop timeout must be finite and positive")
        with self._condition:
            self._closed = self._stop_requested = True
            link, thread = self._link, self._thread
            if not self._started:
                self._done = True
                return self.stats
        try:
            link.stop()
        except Exception as error:
            if self._error is None:
                self._error = error
        if thread is not threading.current_thread():
            thread.join(timeout)
        if thread.is_alive():
            raise VideoStreamError("Video worker did not stop before timeout")
        if self._error:
            raise VideoStreamError(str(self._error)) from self._error
        return self.stats

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc, tb):
        try:
            self.stop()
        except VideoStreamError as error:
            if exc is None:
                raise
            exc.add_note("Video shutdown failed: " + str(error))
        return False

    def __iter__(self):
        return self

    def __next__(self):
        return self.read()
