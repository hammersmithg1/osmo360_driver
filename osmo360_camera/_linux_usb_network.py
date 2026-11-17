"""Session-owned static IPv4 on a new unmanaged Linux USB ECM interface.

No persistent profile, DHCP client, DNS, default-route or Wi-Fi change. The
address has no expiry while the session is alive; normal cleanup deletes only
owned state, and camera-side heartbeat rollback disconnects the ephemeral NIC
when the host disappears. An unexpected/stale interface is never adopted.

The initial supported Linux setup is systemd-networkd with the newly enumerated
ECM link unmanaged. Requires host permission for the scoped ip commands; this
module never invokes sudo, grants capabilities, or changes a network manager.
Import/construction does no I/O.
"""

from dataclasses import asdict, dataclass
import math
from pathlib import Path
import re
import subprocess
import time

from . import _linux_network as network

ADDRESS = "192.168.253.2/30"
BROADCAST = "192.168.253.3"


class LinuxUSBNetworkError(network.NetworkGuardError):
    pass


class USBGenerationVanished(LinuxUSBNetworkError):
    """A failed intermediate generation is fully gone; fresh handover allowed."""

    def __init__(self, proof):
        super().__init__(
            "Intermediate USB generation vanished with no remaining owned state"
        )
        self.proof = proof


class _MutationTargetVanished(LinuxUSBNetworkError):
    pass


@dataclass(frozen=True)
class Identity:
    name: str
    ifindex: int
    mac: str
    net_path: str
    net_inode: tuple
    usb_path: str
    usb_inode: tuple
    busnum: int
    devnum: int


class LinuxUSBNetworkGuard:
    """Single-use owned alias; the underlying read-only checks stay intact.

    Dependency injection is for fake sysfs/command tests. The default paths and
    callbacks refer to the real Linux host; import/construction does no I/O.
    """

    def __init__(
        self,
        *,
        capture=network.capture,
        run=subprocess.run,
        net_root=Path("/sys/class/net"),
        usb_root=Path("/sys/bus/usb/devices"),
        physical_root=Path("/sys/devices"),
        state_root=Path("/run/systemd/netif/links"),
        clock=time.monotonic,
        sleep=time.sleep,
    ):
        self.capture, self.run = capture, run
        self.net_root, self.usb_root = Path(net_root), Path(usb_root)
        self.physical_root, self.state_root = Path(physical_root), Path(state_root)
        self.clock, self.sleep = clock, sleep
        self.baseline = self.identity = None
        self.original_up = self.up_intent = self.cleaned = False
        self.add_rejected = False
        self.attempts = set()
        self._reset_origin = self._reset_deadline = None
        self.report = {
            "platform": "linux",
            "address_mode": "owned-session-static",
            "events": [],
            "restored": False,
        }

    @staticmethod
    def _inode(path):
        value = path.stat()
        return value.st_dev, value.st_ino

    def _identity(self, name, mac):
        if not re.fullmatch(r"[A-Za-z0-9_.-]{1,15}", name):
            raise LinuxUSBNetworkError("Unsafe interface name")
        entry = self.net_root / name
        net = entry.resolve(strict=True)
        root = self.physical_root.resolve(strict=True)
        if not net.is_relative_to(root):
            raise LinuxUSBNetworkError("Network device is outside physical sysfs")
        if (entry / "address").read_text().strip().lower() != mac:
            raise LinuxUSBNetworkError("USB host MAC changed")
        index = int((entry / "ifindex").read_text())
        if index <= 0:
            raise LinuxUSBNetworkError("Invalid interface index")
        device = (entry / "device").resolve(strict=True)
        if not device.is_relative_to(root):
            raise LinuxUSBNetworkError(
                "Network device parent is outside physical sysfs"
            )
        usb = next(
            (
                path
                for path in (device, *device.parents)
                if (path / "idVendor").is_file() and (path / "idProduct").is_file()
            ),
            None,
        )
        if usb is None or (
            (usb / "idVendor").read_text().strip().lower(),
            (usb / "idProduct").read_text().strip().lower(),
        ) != ("2ca3", "0025"):
            raise LinuxUSBNetworkError("Interface is not a child of camera 2ca3:0025")
        if (usb / "bDeviceClass").read_text().strip().lower() == "09":
            raise LinuxUSBNetworkError("Camera ancestor cannot be a USB hub")
        matches = []
        for item in self.usb_root.iterdir():
            if ":" in item.name:
                continue
            try:
                pair = (
                    (item / "idVendor").read_text().strip().lower(),
                    (item / "idProduct").read_text().strip().lower(),
                )
                if pair == ("2ca3", "0025"):
                    matches.append(item.resolve(strict=True))
            except (OSError, UnicodeError):
                continue
        if matches != [usb]:
            raise LinuxUSBNetworkError(
                "Expected exactly one matching camera USB ancestor"
            )
        bus, address = (
            int((usb / "busnum").read_text()),
            int((usb / "devnum").read_text()),
        )
        if bus <= 0 or address <= 0:
            raise LinuxUSBNetworkError("Invalid USB bus/device address")
        identity = Identity(
            name,
            index,
            mac,
            str(net),
            self._inode(net),
            str(usb),
            self._inode(usb),
            bus,
            address,
        )
        old = self._reset_origin
        if old is not None and (
            identity.mac != old.mac
            or identity.usb_path != old.usb_path
            or identity.busnum != old.busnum
            or identity.devnum == old.devnum
            or identity.ifindex == old.ifindex
        ):
            raise LinuxUSBNetworkError(
                "Not a new camera USB generation on the original physical port"
            )
        return identity

    @staticmethod
    def _interface_indexes(snapshot):
        indexes = [
            row.get("ifindex") for row in network._json(snapshot, network.ADDR_COMMAND)
        ]
        if any(type(index) is not int or index <= 0 for index in indexes) or len(
            set(indexes)
        ) != len(indexes):
            raise LinuxUSBNetworkError("Incomplete Linux interface index inventory")
        return indexes

    def _old_index_absent(self, old, snapshot=None):
        """A failed/incomplete read is never evidence that the old NIC vanished."""
        try:
            snapshot = self.capture() if snapshot is None else snapshot
            network._parse(snapshot)  # Successful address/route/DNS capture.
            indexes = self._interface_indexes(snapshot)
        except (
            OSError,
            network.NetworkGuardError,
            KeyError,
            TypeError,
            ValueError,
        ) as error:
            raise LinuxUSBNetworkError(
                "Cannot prove complete Linux interface inventory"
            ) from error
        present = old.ifindex in indexes
        sysfs_indexes = []
        try:
            # No skipped entries, including unreadable/disappearing interfaces.
            for entry in self.net_root.iterdir():
                index = int((entry / "ifindex").read_text())
                if index <= 0:
                    raise ValueError("Invalid sysfs ifindex")
                sysfs_indexes.append(index)
        except (OSError, ValueError) as error:
            raise LinuxUSBNetworkError(
                "Cannot prove complete sysfs interface inventory"
            ) from error
        if len(set(sysfs_indexes)) != len(sysfs_indexes):
            raise LinuxUSBNetworkError("Duplicate sysfs interface index")
        absent = not present and old.ifindex not in sysfs_indexes
        return (
            absent,
            dict(
                old_ifindex=old.ifindex,
                ip_interfaces=len(indexes),
                sysfs_interfaces=len(sysfs_indexes),
                complete=True,
                absent=absent,
            ),
        )

    def _networkd_state(self, index):
        values = {}
        for line in (self.state_root / str(index)).read_text().splitlines():
            if not line or line.startswith("#") or "=" not in line:
                continue
            key, value = line.split("=", 1)
            if key in values:
                raise LinuxUSBNetworkError("Duplicate networkd state field")
            values[key] = value
        return values.get("ADMIN_STATE")

    def preflight(self):
        if self.baseline is not None:
            raise LinuxUSBNetworkError("Linux USB network guard is single-use")
        baseline = self.capture()
        result = network.evaluate(baseline)
        if not result["ok"]:
            raise LinuxUSBNetworkError("; ".join(result["errors"]))
        self.baseline = baseline
        return baseline

    def _snapshot(self):
        snapshot = self.capture(network.CAMERA)
        return snapshot, network._parse(snapshot, network.CAMERA)

    def _current(self, observation=None):
        """Return None only when the original NIC is truly gone, not renamed."""
        identity = self.identity
        snapshot, state = self._snapshot() if observation is None else observation
        row = state["interfaces"].get(identity.name)
        if row is None:
            raw = network._json(snapshot, network.ADDR_COMMAND)
            if any(
                item.get("ifindex") == identity.ifindex
                or str(item.get("address", "")).lower() == identity.mac
                for item in raw
            ):
                raise LinuxUSBNetworkError(
                    "Owned interface was renamed or replaced; refusing cleanup"
                )
            return None
        if self._identity(identity.name, identity.mac) != identity:
            raise LinuxUSBNetworkError(
                "Owned interface identity changed; refusing writes"
            )
        if [
            name
            for name, item in state["interfaces"].items()
            if item["mac"] == identity.mac
        ] != [identity.name]:
            raise LinuxUSBNetworkError("Owned USB MAC is no longer unique")
        if self._networkd_state(identity.ifindex) != "unmanaged":
            raise LinuxUSBNetworkError("Owned interface is no longer unmanaged")
        own = [{"address": network.HOST, "network": str(network.NETWORK), "prefix": 30}]
        if row["ipv4"] not in ([], own):
            raise LinuxUSBNetworkError(
                "Foreign IPv4 state on owned interface; refusing writes"
            )
        pending_linkdown = False
        for route in state["routes"]:
            if route.get("dev") != identity.name:
                continue
            if network._own_route(route, identity.name):
                continue
            # Carrier can lag administrative UP. Only this exact state flag may
            # be ignored for ownership, never for the final readiness decision.
            if route.get("flags") == ["linkdown"] and network._own_route(
                dict(route, flags=[]), identity.name
            ):
                pending_linkdown = True
                continue
            raise LinuxUSBNetworkError(
                "Foreign route on owned interface; refusing writes"
            )
        raw = next(
            item
            for item in network._json(snapshot, network.ADDR_COMMAND)
            if item["ifname"] == identity.name
        )
        if raw.get("ifindex") != identity.ifindex or raw.get("master"):
            raise LinuxUSBNetworkError("Interface index/master changed")
        if row["ipv4"] and any(
            item.get("broadcast") != BROADCAST
            for item in raw["addr_info"]
            if item.get("family") == "inet"
        ):
            raise LinuxUSBNetworkError("Owned address broadcast changed")
        return dict(row, pending_linkdown=pending_linkdown)

    def _unchanged_other_state(self, state):
        before = network._parse(self.baseline)
        if (
            network._defaults(before) != network._defaults(state)
            or before["dns"] != state["dns"]
        ):
            raise LinuxUSBNetworkError("Baseline default routes or DNS changed")
        for name, row in before["interfaces"].items():
            now = state["interfaces"].get(name)
            if (
                now is None
                or now["link_address"] != row["link_address"]
                or now["ipv4"] != row["ipv4"]
            ):
                raise LinuxUSBNetworkError("Baseline interface changed: " + name)
        # The owned interface/routes are checked in _current; no other overlap
        # may appear while a carrier transition is pending.
        other = dict(
            state,
            interfaces={
                key: value
                for key, value in state["interfaces"].items()
                if key != self.identity.name
            },
            routes=[
                row for row in state["routes"] if row.get("dev") != self.identity.name
            ],
        )
        if network._overlaps(other):
            raise LinuxUSBNetworkError("Foreign subnet overlap appeared")

    def _command(self, key, argv):
        def vanished(message):
            if key in ("address_add", "link_up") and self._reset_origin is not None:
                raise _MutationTargetVanished(message)
            raise LinuxUSBNetworkError(message)

        current = self._current()  # Pin immediately before each scoped mutation.
        if current is None:
            vanished("Original interface disappeared before mutation")
        if key in ("address_add", "link_up"):
            _, state = self._snapshot()
            self._unchanged_other_state(state)
        if key in self.attempts:
            raise LinuxUSBNetworkError(
                "Refusing to replay an attempted network mutation: " + key
            )
        # Capturing baseline state can block; pin again after it, immediately
        # before marking the one attempt. Pre-existing exact values are not ours.
        current = self._current()
        if current is None:
            vanished("Original interface disappeared before mutation")
        if key in ("address_add", "link_up") and self._reset_origin is not None:
            snapshot, state = self._snapshot()
            current = self._current((snapshot, state))
            absent, proof = self._old_index_absent(self._reset_origin, snapshot)
            if current is None:
                vanished("Original interface disappeared before mutation")
            if not absent:
                raise LinuxUSBNetworkError(
                    "Old USB interface is still present; refusing handover writes"
                )
            # Complete inventory may block while another USB generation takes
            # the same name. Repin that candidate AFTER the inventory, not just
            # before it, and use its fresh address/UP/default/DNS state.
            snapshot, state = self._snapshot()
            indexes = self._interface_indexes(snapshot)
            current = self._current((snapshot, state))
            if current is None:
                vanished("Original interface disappeared before mutation")
            if self._reset_origin.ifindex in indexes:
                raise LinuxUSBNetworkError(
                    "USB generation changed after absence inventory"
                )
            self._unchanged_other_state(state)
            if self.clock() >= self._reset_deadline:
                raise LinuxUSBNetworkError(
                    "USB generation handover deadline before mutation"
                )
            self.report["old_index_absence"] = proof
        if key == "address_add" and current["ipv4"]:
            raise LinuxUSBNetworkError(
                "IPv4 appeared before address add; it is not owned"
            )
        if key == "link_up" and "UP" in current["flags"]:
            raise LinuxUSBNetworkError(
                "Administrative UP appeared before link up; it is not owned"
            )
        self.attempts.add(key)
        if key == "link_up":
            self.up_intent = True
        event = {"operation": key, "argv": list(argv), "attempted": True}
        self.report["events"].append(event)
        try:
            result = self.run(argv, capture_output=True, text=True, timeout=5)
            event.update(returncode=result.returncode, stderr=result.stderr[-1000:])
            if result.returncode:
                if key == "address_add":
                    self.add_rejected = True
                if key == "link_up":
                    self.up_intent = False
                raise LinuxUSBNetworkError("Scoped ip command failed: " + key)
        except BaseException as error:
            event["error"] = type(error).__name__ + ": " + str(error)
            raise

    def wait_ready(self, baseline, host_mac, *, timeout=45):
        if (
            type(timeout) not in (int, float)
            or not math.isfinite(timeout)
            or timeout < 0
        ):
            raise ValueError("timeout must be finite and nonnegative")
        if baseline is not self.baseline or self.identity is not None:
            raise LinuxUSBNetworkError("Unexpected baseline or repeated wait_ready")
        deadline = self.clock() + timeout
        try:
            while True:
                snapshot, state = self._snapshot()
                result = network.evaluate(baseline, after=snapshot, host_mac=host_mac)
                if result["fatal_errors"]:
                    raise LinuxUSBNetworkError("; ".join(result["fatal_errors"]))
                name = result.get("ncm_interface")
                if name is not None:
                    row = state["interfaces"][name]
                    if row["ipv4"]:
                        raise LinuxUSBNetworkError(
                            "Owned USB alias requires empty IPv4"
                        )
                    try:
                        identity = self._identity(name, host_mac.lower())
                    except FileNotFoundError as error:
                        # udev may rename usb0 after the ip snapshot. Nothing
                        # is owned yet: retry discovery, never a pinned write.
                        if self.identity is not None or self.attempts:
                            raise
                        self.report["initial_identity_retries"] = (
                            self.report.get("initial_identity_retries", 0) + 1
                        )
                        if self.clock() >= deadline:
                            raise LinuxUSBNetworkError(
                                "Initial USB identity did not settle before deadline"
                            ) from error
                        self.sleep(min(0.1, max(0, deadline - self.clock())))
                        continue
                    try:
                        setup = self._networkd_state(identity.ifindex)
                    except FileNotFoundError:
                        setup = None
                    if setup == "unmanaged":
                        self.identity = identity
                        self.original_up = "UP" in row["flags"]
                        self.report["identity"] = asdict(identity)
                        self.report["original_up"] = self.original_up
                        self._current()
                        break
                    if setup not in (None, "pending", "initialized"):
                        raise LinuxUSBNetworkError(
                            "New USB NIC is handled by networkd: " + str(setup)
                        )
                if self.clock() >= deadline:
                    raise LinuxUSBNetworkError(
                        "New unmanaged USB ECM interface did not become ready"
                    )
                self.sleep(0.1)
            self._command(
                "address_add",
                [
                    "ip",
                    "-4",
                    "address",
                    "add",
                    ADDRESS,
                    "broadcast",
                    BROADCAST,
                    "dev",
                    name,
                    "valid_lft",
                    "forever",
                    "preferred_lft",
                    "forever",
                ],
            )
            current = self._current()
            if current is None or not current["ipv4"]:
                raise LinuxUSBNetworkError("Temporary address add was not observed")
            if not self.original_up:
                self._command("link_up", ["ip", "link", "set", "dev", name, "up"])
            while True:
                snapshot, state = self._snapshot()
                current = self._current((snapshot, state))
                if current is None:
                    raise LinuxUSBNetworkError(
                        "Original USB interface disappeared before readiness"
                    )
                self._unchanged_other_state(state)
                if current["pending_linkdown"]:
                    self.report["pending_linkdown_observations"] = (
                        self.report.get("pending_linkdown_observations", 0) + 1
                    )
                    if self.clock() >= deadline:
                        raise LinuxUSBNetworkError(
                            "USB carrier remained linkdown before deadline"
                        )
                    self.sleep(0.1)
                    continue
                # Actual, unmodified snapshot: pending ownership is never ready.
                result = network.evaluate(baseline, after=snapshot, host_mac=host_mac)
                if result["ok"]:
                    result["owned_session_static"] = True
                    self.report["ready"] = result
                    return result
                if result["fatal_errors"] or self.clock() >= deadline:
                    raise LinuxUSBNetworkError("; ".join(result["errors"]))
                self.sleep(0.1)
        except BaseException as error:
            try:
                self.cleanup_owned()
            except Exception as cleanup:
                self.report["cleanup_error"] = str(cleanup)
                if hasattr(error, "add_note"):
                    error.add_note("Temporary USB address cleanup: " + str(cleanup))
            raise

    def rebind_after_usb_reset(self, timeout=15):
        """Return ``(new_guard, ready_report)`` after a pinned USB generation change.

        The original baseline object is shared, never recaptured or replaced.
        Only an absent old ifindex and a new unique/unmanaged camera NIC on the
        same USB path/bus are accepted. This does not rebind sockets, restart
        streams, or claim sample continuity. The caller adopts the returned
        guard only after success; failure retains this guard and its error.
        """
        if (
            type(timeout) not in (int, float)
            or not math.isfinite(timeout)
            or timeout <= 0
        ):
            raise ValueError("timeout must be finite and positive")
        if (
            self.baseline is None
            or self.identity is None
            or self.cleaned
            or self.report.get("ready", {}).get("ok") is not True
        ):
            raise LinuxUSBNetworkError(
                "An active ready USB generation is required for handover"
            )
        old, deadline = self.identity, self.clock() + timeout
        event = dict(old=asdict(old), completed=False)
        self.report.setdefault("rebind_attempts", []).append(event)
        replacement = None
        ready_completed = False
        try:
            while True:
                absent, proof = self._old_index_absent(old)
                if self.clock() >= deadline:
                    raise LinuxUSBNetworkError(
                        "Old USB generation did not disappear before deadline"
                    )
                if absent:
                    event["old_index_absence"] = proof
                    break
                self.sleep(min(0.1, max(0, deadline - self.clock())))
            replacement = LinuxUSBNetworkGuard(
                capture=self.capture,
                run=self.run,
                net_root=self.net_root,
                usb_root=self.usb_root,
                physical_root=self.physical_root,
                state_root=self.state_root,
                clock=self.clock,
                sleep=self.sleep,
            )
            replacement.baseline = self.baseline
            replacement._reset_origin, replacement._reset_deadline = old, deadline
            replacement.wait_ready(
                self.baseline, old.mac, timeout=max(0, deadline - self.clock())
            )
            ready_completed = True
            snapshot, state = replacement._snapshot()
            current = replacement._current((snapshot, state))
            absent, proof = replacement._old_index_absent(old, snapshot)
            ready = network.evaluate(self.baseline, after=snapshot, host_mac=old.mac)
            if (
                current is None
                or not absent
                or not ready["ok"]
                or self.clock() >= deadline
            ):
                raise LinuxUSBNetworkError(
                    "New USB generation readiness/absence deadline was not verified"
                )
            ready["owned_session_static"] = True
            replacement.report["ready"] = ready
            replacement.report["old_index_absence"] = proof
            event.update(
                completed=True,
                new=asdict(replacement.identity),
                old_index_absence=proof,
            )
            replacement.report["rebind_from"] = asdict(old)
            # Retire only after complete new readiness. Never invoke old cleanup
            # against a reused name or the expected same-MAC replacement.
            self.cleaned = self.report["restored"] = True
            self.report["cleanup"] = (
                "Old USB ifindex vanished; no old-generation host write"
            )
            return replacement, ready
        except BaseException as error:
            # wait_ready already performs its one cleanup attempt on failure.
            # Only a later final-proof failure needs an additional cleanup.
            if replacement is not None:
                if ready_completed:
                    try:
                        replacement.cleanup_owned()
                    except Exception as cleanup:
                        event["cleanup_error"] = str(cleanup)
                        if hasattr(error, "add_note"):
                            error.add_note(
                                "New USB generation cleanup: " + str(cleanup)
                            )
                event["new_guard_report"] = replacement.report
            event["error"] = type(error).__name__ + ": " + str(error)
            if isinstance(error, _MutationTargetVanished) and replacement is not None:
                proof = self._vanished_handover(replacement, old, deadline)
                if proof is not None:
                    event["vanished_generation"] = proof
                    raise USBGenerationVanished(proof) from error
            raise

    def _vanished_handover(self, candidate, old, deadline):
        """RO reconciliation only; never excuse an uncertain ip operation."""
        events = candidate.report["events"]
        if (
            candidate.identity is None
            or not candidate.cleaned
            or candidate.report.get("restored") is not True
            or candidate.report.get("cleanup_error")
            or candidate.up_intent
            or candidate.add_rejected
        ):
            return None
        if candidate.attempts:
            if (
                candidate.attempts != {"address_add"}
                or len(events) != 1
                or events[0].get("operation") != "address_add"
                or type(events[0].get("returncode")) is not int
                or events[0]["returncode"] != 0
                or events[0].get("error")
            ):
                return None
        elif events:
            return None
        try:
            snapshot = self.capture()
            restored = network.evaluate(self.baseline, after=snapshot, cleanup=True)
            parent_gone, parent_proof = self._old_index_absent(old, snapshot)
            candidate_gone, candidate_proof = candidate._old_index_absent(
                candidate.identity, snapshot
            )
            if (
                not restored["ok"]
                or not parent_gone
                or not candidate_gone
                or self.clock() >= deadline
            ):
                return None
        except (OSError, network.NetworkGuardError, KeyError, TypeError, ValueError):
            return None
        return dict(
            parent=parent_proof,
            candidate=candidate_proof,
            no_usb_alias_or_overlap=True,
            baseline_unchanged=True,
            completed_address_add=bool(candidate.attempts),
        )

    def cleanup_owned(self):
        if self.cleaned or self.identity is None:
            return self.report
        row = self._current()
        if row is None:
            self.report["cleanup"] = "Original USB NIC disappeared; no host write"
            self.cleaned = self.report["restored"] = True
            return self.report
        errors = []
        if row["ipv4"]:
            if "address_add" not in self.attempts or self.add_rejected:
                raise LinuxUSBNetworkError(
                    "Address exists without an owned add; refusing cleanup"
                )
            try:
                self._command(
                    "address_delete",
                    ["ip", "-4", "address", "del", ADDRESS, "dev", self.identity.name],
                )
            except BaseException as error:
                errors.append(str(error))
        row = self._current()
        if (
            row is not None
            and self.up_intent
            and not self.original_up
            and "UP" in row["flags"]
        ):
            try:
                self._command(
                    "link_down",
                    ["ip", "link", "set", "dev", self.identity.name, "down"],
                )
            except BaseException as error:
                errors.append(str(error))
        row = self._current()
        if row is not None and (
            row["ipv4"]
            or (self.up_intent and not self.original_up and "UP" in row["flags"])
        ):
            errors.append("Owned temporary host state remains")
        self.report["cleanup_errors"] = errors
        if errors:
            raise LinuxUSBNetworkError("; ".join(errors))
        self.cleaned = self.report["restored"] = True
        return self.report

    def check_restored(self, baseline):
        if baseline is not self.baseline:
            raise LinuxUSBNetworkError("Unexpected cleanup baseline")
        self.cleanup_owned()
        result = network.evaluate(baseline, after=self.capture(), cleanup=True)
        if not result["ok"]:
            raise LinuxUSBNetworkError("; ".join(result["errors"]))
        return result
