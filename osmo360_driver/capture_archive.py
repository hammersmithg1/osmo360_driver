"""Bounded raw H.264 preroll and atomic archival admission, without ROS or disk.

Only a GOP beginning with an IDR is reusable. If it grows beyond the memory
budget, begin() requests a fresh IDR instead of retaining unbounded video.
The sink must enqueue immutable data promptly; it must never encode or write
files on this producer thread. Preview consumers are deliberately unrelated.
"""

from collections import deque
import math
import threading
import time


class ArchiveBeginCancelled(RuntimeError):
    """A stopped/reset archive boundary cannot admit a replacement epoch."""


class ArchiveVideo:
    def __init__(self, sink, *, max_bytes=8 * 1024 * 1024):
        if not callable(sink) or type(max_bytes) is not int or max_bytes <= 0:
            raise ValueError('A callable sink and positive byte budget are required')
        self.sink, self.max_bytes = sink, max_bytes
        self.condition = threading.Condition(threading.RLock())
        self.cache = deque()
        self.cache_bytes = 0
        self.valid = self.active = False
        self.error = ''
        self.units = self.sessions = 0
        self.preroll_units = 0
        self.first_native_ns = self.first_stamp_ns = None
        self.codec_headers_hex = ''
        self._boundary_generation = 0
        self._pending_begin = None

    def append(self, frame, stamp):
        with self.condition:
            if frame.metadata.get('h264_keyframe'):
                self.cache.clear()
                self.cache_bytes = 0
                self.valid = bool(frame.metadata.get('h264_codec_headers_complete'))
            if self.valid:
                size = len(frame.raw_h264)
                if self.cache_bytes + size > self.max_bytes or len(self.cache) >= 256:
                    self.cache.clear()
                    self.cache_bytes = 0
                    self.valid = False
                else:
                    self.cache.append((frame, stamp))
                    self.cache_bytes += size
            if self.active:
                try:
                    self.sink(frame, stamp)
                    self.units += 1
                except Exception as error:
                    # Make sink failure explicit and propagate it to producer
                    # diagnostics; never silently claim this AU was archived.
                    self.error = str(error)
                    self.active = False
                    raise
            self.condition.notify_all()

    def begin(self, request_keyframe, *, timeout=5):
        if type(timeout) not in (int, float) or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('Archive boundary timeout must be positive')
        deadline = time.monotonic() + timeout
        with self.condition:
            if self.active:
                raise ValueError('Raw archive is already active')
            if self._pending_begin is not None:
                raise ValueError('Raw archive is already waiting for its start boundary')
            generation = self._boundary_generation
            token = self._pending_begin = object()
            needs_keyframe = not self.valid or not self.cache

        def check_boundary():
            if self._boundary_generation != generation or self._pending_begin is not token:
                raise ArchiveBeginCancelled('Archive start was cancelled by end/reset; capture boundary changed')

        try:
            if needs_keyframe:
                request_keyframe()
            with self.condition:
                while True:
                    check_boundary()
                    if self.valid and self.cache:
                        break
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError('No decodable H.264 IDR boundary before archive deadline')
                    self.condition.wait(remaining)
                # Holding the same lock as append makes preroll and subsequent
                # live delivery ordered, with no duplicates or admission gap.
                self.error = ''
                self.units = 0
                self.preroll_units = len(self.cache)
                self.first_native_ns = self.cache[0][0].timestamp_ns
                self.first_stamp_ns = self.cache[0][1]
                self.codec_headers_hex = self.cache[0][0].metadata['h264_codec_headers_hex']
                try:
                    for frame, stamp in self.cache:
                        check_boundary()
                        self.sink(frame, stamp)
                        check_boundary()
                        self.units += 1
                except Exception as error:
                    if self._boundary_generation == generation:
                        self.error = str(error)
                    raise
                self.active = True
                self.sessions += 1
                self._pending_begin = None
                return self.status()
        finally:
            with self.condition:
                if self._pending_begin is token:
                    self._pending_begin = None
                self.condition.notify_all()

    def reset(self):
        """Call before a new capture epoch; never reuse pre-reboot GOP bytes."""
        with self.condition:
            self._boundary_generation += 1
            self._pending_begin = None
            self.active = self.valid = False
            self.cache.clear()
            self.cache_bytes = self.units = self.preroll_units = 0
            self.first_native_ns = self.first_stamp_ns = None
            self.codec_headers_hex = self.error = ''
            self.condition.notify_all()

    def end(self):
        with self.condition:
            self._boundary_generation += 1
            self._pending_begin = None
            self.active = False
            self.condition.notify_all()
            return self.status()

    def status(self):
        with self.condition:
            return dict(format='h264_annex_b', active=self.active,
                        starting=self._pending_begin is not None, error=self.error,
                        units=self.units, preroll_units=self.preroll_units,
                        first_native_ns=self.first_native_ns, first_stamp_ns=self.first_stamp_ns,
                        codec_headers_hex=self.codec_headers_hex, preroll_frames=self.preroll_units,
                        independently_decodable=bool(self.first_stamp_ns is not None and self.codec_headers_hex),
                        cached_units=len(self.cache), cached_bytes=self.cache_bytes,
                        cache_limit_bytes=self.max_bytes, decodable_boundary=self.valid,
                        sessions=self.sessions)
