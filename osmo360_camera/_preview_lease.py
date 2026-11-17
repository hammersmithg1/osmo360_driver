"""Host heartbeat for the camera's temporary, RAM-only USB video session."""

import math
import socket
import struct
import threading
import time
from .vendor.util import bind_socket

MAGIC = b"O360LSE1"
REQUEST = struct.Struct("!8s16sQB")
REPLY = struct.Struct("!8s16sQddB")


class PreviewLeaseError(RuntimeError):
    pass


class PreviewLease:
    """One session, one UDP socket; no host network configuration or USB I/O."""

    def __init__(self, config):
        self.camera_ip = config["camera_ip"]
        self.host_ip = config["host_ip"]
        self.port = config["port"]
        self.interface = config.get("interface")
        self.nonce = bytes.fromhex(config["nonce"])
        if len(self.nonce) != 16 or not 49152 <= self.port <= 65535:
            raise ValueError("Invalid generated lease configuration")
        self.interval = 1.0
        self._socket = None
        self._sequence = 0
        self._thread = None
        self._started = False
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self.last_ack_monotonic = None
        self.last_ack = None
        self.error = None
        self.stats = dict(sent=0, acknowledged=0, timeouts=0, invalid_replies=0)

    @property
    def healthy(self):
        return (
            self.last_ack_monotonic is not None
            and time.monotonic() - self.last_ack_monotonic < 4
            and not self._stop.is_set()
        )

    def _exchange(self, opcode=0):
        with self._lock:
            self._sequence += 1
            sequence = self._sequence
            self._socket.sendto(
                REQUEST.pack(MAGIC, self.nonce, sequence, opcode),
                (self.camera_ip, self.port),
            )
            self.stats["sent"] += 1
            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline:
                self._socket.settimeout(max(0.001, deadline - time.monotonic()))
                try:
                    data, source = self._socket.recvfrom(1024)
                except socket.timeout:
                    break
                received = time.monotonic()
                if received >= deadline:
                    break
                if source != (self.camera_ip, self.port) or len(data) != REPLY.size:
                    self.stats["invalid_replies"] += 1
                    continue
                magic, nonce, seq, now, expires, state = REPLY.unpack(data)
                if (
                    magic != MAGIC
                    or nonce != self.nonce
                    or seq != sequence
                    or state != (2 if opcode else 1)
                    or not all(math.isfinite(v) for v in (now, expires))
                    or abs((expires - now) - (0 if opcode else 10)) > 0.000001
                ):
                    self.stats["invalid_replies"] += 1
                    continue
                self.last_ack_monotonic = received
                self.last_ack = dict(
                    sequence=seq, camera_monotonic=now, expires=expires, state=state
                )
                self.stats["acknowledged"] += 1
                return self.last_ack
            self.stats["timeouts"] += 1
            raise PreviewLeaseError("Camera USB preview heartbeat was not acknowledged")

    def start(self, timeout=12):
        if self._started or self._stop.is_set():
            raise PreviewLeaseError("PreviewLease is single-use")
        self._started = True
        sock = self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            bind_socket(sock, self.host_ip, self.interface)
            deadline = time.monotonic() + timeout
            while True:
                try:
                    self._exchange()
                    break
                except PreviewLeaseError:
                    if time.monotonic() >= deadline:
                        raise
                    time.sleep(0.2)
            self._thread = threading.Thread(
                target=self._run, name="osmo-preview-heartbeat", daemon=True
            )
            self._thread.start()
            return self
        except BaseException:
            sock.close()
            self._socket = None
            raise

    def _run(self):
        while not self._stop.wait(self.interval):
            try:
                self._exchange()
                self.error = None
            except (OSError, PreviewLeaseError) as error:
                self.error = str(error)

    def renew(self):
        """Obtain one fresh data-only ACK on this existing owned socket."""
        if not self._started or self._stop.is_set() or self._socket is None:
            raise PreviewLeaseError("Renew requires an active owned lease")
        return self._exchange()

    def stop(self, *, close_camera=True):
        """Stop owned IMU/video before this call: close triggers USB rebind."""
        self._stop.set()
        if self._thread:
            self._thread.join(2)
            if self._thread.is_alive():
                raise PreviewLeaseError("Heartbeat thread did not stop")
        if self._socket:
            try:
                if close_camera:
                    return self._exchange(1)
            finally:
                self._socket.close()
                self._socket = None

    def __enter__(self):
        return self.start()

    def __exit__(self, *_):
        self.stop()
