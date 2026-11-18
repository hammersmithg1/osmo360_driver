"""Private proof for stock Sleep after capture stops, with USB still owned.

The active USB lease is deliberately retained. This proof does not claim that
the original USB configuration has already been restored. No device access
occurs here; the coordinator supplies its observed stop/readback facts.
"""

from dataclasses import dataclass
import math
import re
from . import _app


def _collector_closed(imu):
    # IMUStream deliberately retains its completed Popen (and PID) for stats.
    # Completion is the reaped process plus closed local workers/socket.
    process = imu._process
    code = process.poll() if process is not None else None
    report = imu.report
    return (
        type(code) is int
        and code == 0
        and type(imu.stats.get("collector_returncode")) is int
        and imu.stats["collector_returncode"] == 0
        and type(report) is dict
        and report.get("restored") is True
        and report.get("quiescent") is True
        and not report.get("errors")
        and imu._closed is True
        and imu._receiver_done.is_set()
        and all(not thread.is_alive() for thread in imu._threads)
        and imu._socket.fileno() == -1
    )


@dataclass(frozen=True)
class LocalSleepEntryProof:
    camera: object
    preview: object
    stream: object
    lease: object
    closed_imu: object
    verified_at: float
    recording_state: int
    activation_state: int
    battery_present: bool
    imu_mask_restored: bool
    collector_returncode: int
    video_thread_stopped: bool
    datalink_socket_closed: bool
    lease_owned: bool
    lease_healthy: bool
    lease_ack_at: float
    lease_nonce: str
    lease_work: str
    query: bytes
    app_before: _app.QueryReply
    app_after: _app.QueryReply

    def validate(self, now):
        if (
            type(self.verified_at) not in (int, float)
            or not math.isfinite(self.verified_at)
            or not 0 <= now - self.verified_at <= 5
            or type(self.recording_state) is not int
            or self.recording_state != 0
            or type(self.activation_state) is not int
            or self.activation_state != 1
            or self.battery_present is not True
            or self.imu_mask_restored is not True
            or type(self.collector_returncode) is not int
            or self.collector_returncode != 0
            or self.video_thread_stopped is not True
            or self.datalink_socket_closed is not True
            or self.lease_owned is not True
            or self.lease_healthy is not True
            or type(self.lease_ack_at) not in (int, float)
            or not math.isfinite(self.lease_ack_at)
            or not 0 <= now - self.lease_ack_at < 4
            or not isinstance(self.lease_nonce, str)
            or not re.fullmatch("[0-9a-f]{32}", self.lease_nonce)
            or not isinstance(self.lease_work, str)
            or not re.fullmatch(
                "/dev/shm/osmo-ncm-[A-Za-z0-9_-]{1,32}", self.lease_work
            )
            or self.query != _app.query_payload()
            or not isinstance(self.app_before, _app.QueryReply)
            or not isinstance(self.app_after, _app.QueryReply)
            or len(self.app_before.own_records) != 1
            or self.app_after.own_records
        ):
            raise ValueError(
                "Fresh closed producers, original mask and owned live lease required"
            )
        plan = self.preview._plan
        ack = self.lease.last_ack_monotonic
        if (
            self.camera._usb_preview is not self.preview
            or self.camera._imu is not None
            or self.preview._stream is not self.stream
            or self.preview._lease is not self.lease
            or plan["work_dir"] != self.lease_work
            or plan["lease"]["nonce"] != self.lease_nonce
            or self.lease.nonce.hex() != self.lease_nonce
            or not self.lease.healthy
            or type(ack) not in (int, float)
            or not math.isfinite(ack)
            or not self.lease_ack_at <= ack <= now
            or not now - ack < 4
            or not _collector_closed(self.closed_imu)
            or self.stream._thread.is_alive()
            or not self.stream._link.stopped.is_set()
            or self.stream._link._sock.fileno() != -1
        ):
            raise ValueError(
                "Owned capture/lease identities changed before native Sleep"
            )


def submit_local_sleep(control, proof):
    """One fixed native input; the unchanged controller repeats validation.

    Its _submit checks the owner AWAKE state, then validates this proof after
    the status transaction and immediately before the one possible START.
    """
    if type(proof) is not LocalSleepEntryProof:
        raise TypeError("Exact local-producer entry proof required")
    if (
        control.lock is not proof.camera._control_lock
        or control.bootstrap.transport is not proof.camera._transport
    ):
        raise ValueError(
            "Proof and controller must share the caller-owned transport/lock"
        )
    with control.lock:
        proof.validate(control.clock())
        return control._submit("sleep", proof)
