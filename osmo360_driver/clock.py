"""One fixed camera→host offset per stream epoch, shared by video and IMU.

Arrival time is not exposure time. This is an initial clock alignment, not a
claim of hardware synchronization. Both original clocks remain in timing topics.
"""

import threading
import time
from uuid import uuid4


class CameraClock:
    def __init__(self, host_offset_ns=None):
        self.lock = threading.Lock()
        self.epoch = uuid4().hex
        # Fixed host monotonic→Unix mapping prevents NTP steps inside a stream.
        self.host_offset_ns = time.time_ns() - time.monotonic_ns() if host_offset_ns is None else host_offset_ns
        self.offset_ns = None
        self.last = {}

    def stamp(self, native_ns, received_ns, kind):
        if type(native_ns) is not int or native_ns < 0:
            raise ValueError('Camera sample has no native timestamp')
        with self.lock:
            previous = self.last.get(kind)
            if previous is not None and native_ns <= previous:
                raise ValueError('Camera clock went backwards or repeated; restart the stream')
            if self.offset_ns is None:
                self.offset_ns = received_ns + self.host_offset_ns - native_ns
            self.last[kind] = native_ns
            return native_ns + self.offset_ns

    def status(self):
        with self.lock:
            return dict(epoch=self.epoch, offset_ns=self.offset_ns,
                        host_monotonic_to_unix_ns=self.host_offset_ns,
                        method='fixed_first_arrival_offset', hardware_synchronized=False,
                        camera_imu_extrinsics_calibrated=False)
