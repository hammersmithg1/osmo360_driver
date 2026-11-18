"""Bounded newest-sample queue for the private IMU receiver.

Capacity and eviction counts are samples, never transport datagrams. Producers
never wait for consumers. No callback or external lock is taken while holding
this queue's condition.
"""

from collections import deque
import queue
import threading
import time


class RecentSamples:
    def __init__(self, maxsize):
        if type(maxsize) is not int or maxsize < 1:
            raise ValueError('Positive sample capacity required')
        self.maxsize = maxsize
        self._values = deque(maxlen=maxsize)
        self._condition = threading.Condition()

    def put_many_recent(self, samples):
        """Atomically retain the newest values and return actual evictions."""
        samples = tuple(samples)
        if not samples:
            return 0
        with self._condition:
            dropped = max(0, len(self._values) + len(samples) - self.maxsize)
            self._values.extend(samples)
            self._condition.notify(min(len(samples), self.maxsize))
            return dropped

    def put_nowait(self, sample):
        """Ordinary non-evicting queue operation for private queue users."""
        with self._condition:
            if len(self._values) == self.maxsize:
                raise queue.Full
            self._values.append(sample)
            self._condition.notify()

    def get(self, block=True, timeout=None):
        with self._condition:
            if not block:
                if not self._values:
                    raise queue.Empty
            elif timeout is None:
                while not self._values:
                    self._condition.wait()
            elif timeout < 0:
                raise ValueError('timeout must be non-negative')
            else:
                deadline = time.monotonic() + timeout
                while not self._values:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise queue.Empty
                    self._condition.wait(remaining)
            return self._values.popleft()

    def get_nowait(self):
        return self.get(block=False)

    def qsize(self):
        with self._condition:
            return len(self._values)

    def empty(self):
        return self.qsize() == 0
