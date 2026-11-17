"""Host control of the explicitly installed, immutable camera helper cache.

Persistent files contain source bytes only. Every owner, nonce, plan and lease
still belongs to fresh RAM and the current camera boot. Import/construction is
pure; all device calls use an already-owned controller and its task resolver.
"""

import base64
from copy import deepcopy
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import posixpath
import re
import secrets
import shlex
import time
import zlib

from ._standby_control import StandbyControl, SOURCES as OWNER_SOURCES
from ._prepared_stream import SOURCES as PREPARED_SOURCES
from ._retained_stream import SOURCES as RETAINED_SOURCES
from ._usb_bootstrap import USBBootstrap
from ._amt import D1Session, wait_ready

ROOT = "/data/osmo360-sdk"
INSTALLER = "helper_install.py"
BOOT_HOOK = "/etc/init.post_boot_product_post.sh"
STOCK_HOOK_SHA = "483881c9dd76d249f82fb1a7264aa0aa3249b23bf75b9a5241c797eb65a2042d"
ROLE = "/sys/devices/platform/soc/a600000.ssusb/mode"
DEVICE = Path(__file__).with_name("_device")
GROUPS = dict(
    owner=frozenset(OWNER_SOURCES),
    prepared=frozenset(PREPARED_SOURCES),
    retained=frozenset(RETAINED_SOURCES),
)
Q = shlex.quote


class HelperInstallError(RuntimeError):
    def __init__(self, message, report=None):
        self.report = deepcopy(report or {})
        super().__init__(message)


def _bounded(command):
    if not command.isascii() or not 0 < len(command) <= 500 or "\0" in command:
        raise ValueError("Fixed helper command must fit500 ASCII bytes")
    return command


def _timeout(value):
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError("Finite positive helper operation timeout required")
    return value


def _json(raw):
    if type(raw) is not bytes or not 0 < len(raw) <= 16384:
        raise HelperInstallError("Bounded helper report required")

    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise HelperInstallError("Duplicate helper report key")
            result[key] = value
        return result

    return json.loads(
        raw,
        object_pairs_hook=unique,
        parse_constant=lambda value: (_ for _ in ()).throw(
            HelperInstallError("Nonfinite helper report")
        ),
    )


def package_profile(*, sources=None, installer=None):
    """Pure, deterministic profile plus a bounded exact-five-source bundle."""
    expected = dict(OWNER_SOURCES, **RETAINED_SOURCES)
    sources = (
        {name: (DEVICE / name).read_bytes() for name in expected}
        if sources is None
        else sources
    )
    installer = (DEVICE / INSTALLER).read_bytes() if installer is None else installer
    if type(sources) is not dict or set(sources) != set(expected):
        raise ValueError("Exact five installed source filenames required")
    if (
        type(installer) is not bytes
        or not 0 < len(installer) <= 98304
        or not installer.isascii()
        or b"\0" in installer
    ):
        raise ValueError("Bounded packaged installer source required")
    compile(installer, INSTALLER, "exec")
    files = {}
    for name, data in sources.items():
        if (
            type(data) is not bytes
            or not data
            or not data.isascii()
            or b"\0" in data
            or hashlib.sha256(data).hexdigest() != expected[name]
        ):
            raise ValueError("Packaged helper source hash mismatch: " + name)
        files[name] = dict(sha256=expected[name], size=len(data))
    manifest = dict(
        schema=1, installer_sha256=hashlib.sha256(installer).hexdigest(), files=files
    )
    encoded = json.dumps(manifest, sort_keys=True, separators=(",", ":")).encode(
        "ascii"
    )
    manifest["profile"] = hashlib.sha256(encoded).hexdigest()
    plain = json.dumps(
        dict(
            schema=1,
            files={name: data.decode("ascii") for name, data in sources.items()},
        ),
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
    ).encode("ascii")
    packed = zlib.compress(plain, 9)
    if len(plain) > 262144 or len(packed) > 65536:
        raise ValueError("Persistent source bundle exceeds reviewed bounds")
    request = dict(
        manifest,
        packed=dict(
            bytes=len(packed),
            sha256=hashlib.sha256(packed).hexdigest(),
            plain_bytes=len(plain),
            plain_sha256=hashlib.sha256(plain).hexdigest(),
        ),
        autostart=True,
    )
    return dict(
        manifest=manifest,
        sources=dict(sources),
        installer=installer,
        packed=packed,
        request=json.dumps(request, sort_keys=True, separators=(",", ":")).encode(
            "ascii"
        ),
    )


def _identity(value):
    return (
        type(value) is dict
        and set(value) == {"device", "inode"}
        and all(type(value[key]) is int and value[key] > 0 for key in value)
    )


def _boot_id(value):
    return (
        isinstance(value, str)
        and re.fullmatch("[0-9a-f]{8}(?:-[0-9a-f]{4}){3}-[0-9a-f]{12}", value)
        is not None
    )


def _decode(raw, *, operation, nonce, manifest=None):
    value = _json(raw)
    if (
        type(value) is not dict
        or type(value.get("schema")) is not int
        or value["schema"] != 1
        or value.get("operation") != operation
        or value.get("nonce") != nonce
        or not _identity(value.get("export_identity"))
        or value.get("partial")
        or value.get("error")
        or type(value.get("installed")) is not bool
    ):
        raise HelperInstallError(
            "Uncorrelated, incomplete or failed helper operation", value
        )
    if operation != "probe" and not _boot_id(value.get("boot_id")):
        raise HelperInstallError("Current camera boot identity is missing", value)
    if value["installed"]:
        actual = value.get("manifest")
        if (
            manifest is None
            or type(actual) is not dict
            or type(actual.get("schema")) is not int
            or actual != manifest
            or value.get("profile") != manifest["profile"]
        ):
            raise HelperInstallError(
                "Installed helper profile differs from this SDK", value
            )
        autostart = value.get("autostart")
        if (
            type(autostart) is not dict
            or autostart.get("verified") is not True
            or autostart.get("enabled") is not True
            or autostart.get("hook") != BOOT_HOOK
            or autostart.get("backup_sha256") != STOCK_HOOK_SHA
        ):
            raise HelperInstallError("Installed boot hook was not verified", value)
    elif value.get("profile") is not None:
        raise HelperInstallError("Absent installation cannot name a profile", value)
    return value


class _Calls:
    def __init__(
        self, control, *, timeout, guard=None, clock=time.monotonic, deadline=None
    ):
        _timeout(timeout)
        self.control, self.clock, self.guard = control, clock, guard
        self.deadline = clock() + timeout if deadline is None else deadline

    def check(self):
        try:
            if self.guard is not None:
                self.guard()
            if self.clock() >= self.deadline:
                raise TimeoutError("Helper operation deadline; no START retry")
        except BaseException as error:
            error.helper_control = self.control
            raise

    def run(self, command):
        self.check()
        remaining_ms = int((self.deadline - self.clock()) * 1000)
        if remaining_ms < 100:
            raise TimeoutError("No helper task budget remaining")
        try:
            value = self.control._run(
                _bounded(command),
                timeout_ms=min(10000, remaining_ms),
                pre_send=self.check,
            )
        except BaseException as error:
            error.helper_control = self.control
            raise
        self.check()
        return value

    def exports(self):
        program = (
            "from pathlib import Path;import os,stat;p=Path('/blackbox/export_tmp');"
            "assert p.parent.resolve()==p.parent;p.mkdir(mode=448,exist_ok=True);s=p.lstat();"
            "assert p.resolve()==p and stat.S_ISDIR(s.st_mode) and s.st_uid==0 and stat.S_IMODE(s.st_mode)==448"
        )
        self.run("python3 -c " + Q(program))

    def pull(self, output, *, operation, nonce, manifest=None):
        self.check()
        try:
            raw = self.control.bootstrap.pull_bytes(
                output, max_bytes=16384, timeout=min(10, self.deadline - self.clock())
            )
        except BaseException as error:
            error.helper_control = self.control
            raise
        self.check()
        value = _decode(raw, operation=operation, nonce=nonce, manifest=manifest)
        identity = value["export_identity"]
        program = (
            "import os,sys;p=sys.argv[1];s=os.lstat(p);assert (s.st_dev,s.st_ino)=="
            + repr((identity["device"], identity["inode"]))
            + ";os.unlink(p)"
        )
        self.run("python3 -c " + Q(program) + " " + Q(output))
        return value

    def invoke(self, script, operation, *, manifest, extra="", nonce=None):
        nonce = secrets.token_hex(6) if nonce is None else nonce
        if not isinstance(nonce, str) or not re.fullmatch("[0-9a-f]{12}", nonce):
            raise ValueError("Fresh12-hex helper operation nonce required")
        output = "/blackbox/export_tmp/osmo_helpers_" + nonce + ".json"
        parent, name = script.rsplit("/", 1)
        self.run(
            "cd "
            + Q(parent)
            + ' && [ "$(sha256sum '
            + Q(name)
            + ' | cut -d" " -f1)" = '
            + manifest["installer_sha256"]
            + " ] && python3 -B "
            + Q(name)
            + " "
            + operation
            + " --nonce "
            + nonce
            + extra
        )
        return self.pull(output, operation=operation, nonce=nonce, manifest=manifest)

    def upload(self, path, raw):
        self.run("set -C && : > " + Q(path))
        for at in range(0, len(raw), 240):
            piece = base64.b64encode(raw[at : at + 240]).decode("ascii")
            self.run("printf %s " + piece + " | base64 -d >> " + Q(path))
        self.run(
            '[ "$(sha256sum '
            + Q(path)
            + ' | cut -d" " -f1)" = '
            + hashlib.sha256(raw).hexdigest()
            + " ]"
        )


class _Budget:
    """One deadline starts before readiness/discovery and never resets."""

    def __init__(self, timeout, control, guard):
        self.clock = control.clock if control is not None else time.monotonic
        self.deadline, self.guard, self.control = (
            self.clock() + _timeout(timeout),
            guard,
            control,
        )

    def check(self):
        try:
            if self.guard is not None:
                self.guard()
            if self.clock() >= self.deadline:
                raise TimeoutError("Helper lifecycle deadline; no further START")
        except BaseException as error:
            if self.control is not None:
                error.helper_control = self.control
            raise

    def remaining(self):
        self.check()
        return self.deadline - self.clock()

    def calls(self, camera, control):
        self.check()
        if control is None:
            control = _controller(camera, self.remaining(), guard=self.check)
        self.control = control
        self.check()
        return _Calls(
            control,
            timeout=self.remaining(),
            guard=self.check,
            clock=self.clock,
            deadline=self.deadline,
        )


class InstalledSources:
    """Optional ordinary-stream capability; only pure absence falls back."""

    def __init__(self, profile=None, *, timeout=15):
        self.package = package_profile() if profile is None else deepcopy(profile)
        self.manifest, self.timeout = self.package["manifest"], _timeout(timeout)
        self.last_report = None
        self._present = None

    def status(self, control, *, guard=None, deadline=None, clock=None):
        calls = _Calls(
            control,
            timeout=self.timeout,
            guard=guard,
            clock=control.clock if clock is None else clock,
            deadline=deadline,
        )
        calls.exports()
        nonce = secrets.token_hex(6)
        output = "/blackbox/export_tmp/osmo_helpers_" + nonce + ".json"
        # This probe never executes bytes from an unknown/partial installation.
        program = (
            "from pathlib import Path;import json,os,sys;p=Path(sys.argv[1]);"
            'f=open(sys.argv[2],"x");s=os.fstat(f.fileno());json.dump({"schema":1,"operation":"probe",'
            '"nonce":sys.argv[3],"installed":False,"profile":None,"root_exists":p.exists() or p.is_symlink(),'
            '"export_identity":{"device":s.st_dev,"inode":s.st_ino}},f);f.close()'
        )
        calls.run(
            "python3 -c " + Q(program) + " " + Q(ROOT) + " " + Q(output) + " " + nonce
        )
        probe = calls.pull(output, operation="probe", nonce=nonce)
        if type(probe.get("root_exists")) is not bool:
            raise HelperInstallError("Malformed persistent-root observation", probe)
        if not probe["root_exists"]:
            self.last_report = dict(probe)
            self._present = False
            return probe
        script = ROOT + "/versions/" + self.manifest["profile"] + "/" + INSTALLER
        self.last_report = calls.invoke(script, "status", manifest=self.manifest)
        if not self.last_report["installed"]:
            raise HelperInstallError(
                "Persistent root exists but installation is incomplete",
                self.last_report,
            )
        self._present = True
        return self.last_report

    def __call__(self, control, files, work, *, guard=None):
        groups = [group for group, names in GROUPS.items() if set(files) == names]
        if len(groups) != 1 or any(
            type(raw) is not bytes
            or hashlib.sha256(raw).hexdigest() != self.manifest["files"][name]["sha256"]
            for name, raw in files.items()
        ):
            raise ValueError("Exact installed source group required")
        group = groups[0]
        patterns = dict(
            owner=r"/dev/shm/osmo-usb-standby-[0-9a-f]{12}",
            prepared=r"/dev/shm/osmo-prepared-assets-[0-9a-f]{12}",
            retained=r"/dev/shm/osmo-usb-standby-[0-9a-f]{12}/retained-assets-"
            + RETAINED_SOURCES["retained_sleep_060.py"][:12],
        )
        if not isinstance(work, str) or not re.fullmatch(patterns[group], work):
            raise ValueError("Exact fresh RAM source-group destination required")
        # Presence is only a hint. The installed stage command independently
        # verifies the full immutable profile on every copy into fresh RAM.
        if self._present is None:
            self.status(control, guard=guard)
        if not self._present:
            return False
        calls = _Calls(control, timeout=self.timeout, guard=guard, clock=control.clock)
        script = ROOT + "/versions/" + self.manifest["profile"] + "/" + INSTALLER
        result = calls.invoke(
            script,
            "stage",
            manifest=self.manifest,
            extra=" --profile "
            + self.manifest["profile"]
            + " --group "
            + group
            + " --dest "
            + Q(work),
        )
        expected = {name: self.manifest["files"][name] for name in files}
        if (
            result.get("staged") is not True
            or result.get("group") != group
            or result.get("dest") != work
            or result.get("files") != expected
        ):
            raise HelperInstallError("Fresh RAM source-copy proof mismatch", result)
        self.last_report = result
        return True


def _controller(camera, timeout, *, guard=None):
    deadline = time.monotonic() + timeout

    def check():
        if guard is not None:
            guard()
        if time.monotonic() >= deadline:
            raise TimeoutError("Helper readiness deadline")

    check()
    camera.connect()
    check()
    wait_ready(camera._transport, timeout=min(60, deadline - time.monotonic()))
    check()
    bootstrap = USBBootstrap(
        camera._transport, session=D1Session(camera._transport, reuse_owned_id=True)
    )
    return StandbyControl(
        bootstrap, source_provider=lambda: {}, control_lock=camera._control_lock
    )


def _bootstrap_installer(calls, package):
    nonce = secrets.token_hex(6)
    work = "/dev/shm/osmo-helper-install-" + nonce
    calls.run(
        '[ "$(readlink -f /dev/shm)" = /dev/shm ] && '
        '[ "$(awk \'$2=="/dev/shm" {print $3}\' /proc/mounts)" = tmpfs ] && umask 077 && mkdir '
        + Q(work)
    )
    calls.upload(work + "/" + INSTALLER, package["installer"])
    return work, nonce


def _existing_owner(control, *, calls):
    from ._stream_resume import _discover, _status_at

    class Bootstrap:
        def pull_bytes(self, path, **options):
            calls.check()
            value = control.bootstrap.pull_bytes(
                path, timeout=min(10, calls.deadline - calls.clock()), **options
            )
            calls.check()
            return value

    class ScopedControl:
        bootstrap = Bootstrap()

        def _run(self, command):
            return calls.run(command)

    scoped = ScopedControl()
    work = _discover(scoped)
    if work is None:
        return None
    snapshot = _status_at(scoped, work)
    if snapshot["state"] not in ("AWAKE", "SLEEP"):
        raise HelperInstallError(
            "Owner transition pending; no helper installation/removal", snapshot
        )
    return dict(
        work=work,
        nonce=snapshot["nonce"],
        owner=snapshot["owner"],
        service=snapshot["service"],
    )


def install(camera, *, timeout=180, control=None, guard=None):
    """Explicit permanent installation; caller serializes/guards Camera owners."""
    budget = _Budget(timeout, control, guard)
    package = package_profile()
    with camera._control_lock:
        calls = budget.calls(camera, control)
        control = calls.control
        calls.exports()
        owner = _existing_owner(control, calls=calls)
        current = InstalledSources(package, timeout=budget.remaining()).status(
            control, guard=budget.check, deadline=budget.deadline, clock=budget.clock
        )
        if current["installed"]:
            budget.check()
            return dict(
                current,
                already_installed=True,
                previous_boot_id=current["boot_id"],
                owner_receipt=owner,
                _control=control,
            )
        work, nonce = _bootstrap_installer(calls, package)
        calls.upload(work + "/request.json", package["request"])
        calls.upload(work + "/bundle.z", package["packed"])
        result = calls.invoke(
            work + "/" + INSTALLER,
            "install",
            manifest=package["manifest"],
            extra=" --work " + Q(work),
            nonce=nonce,
        )
        if not result["installed"]:
            raise HelperInstallError("Permanent installation was not verified", result)
        result["bootstrap_work"] = work
        result["previous_boot_id"] = result["boot_id"]
        result.update(owner_receipt=owner, _control=control)
        budget.check()
        return result


def remove(camera, *, timeout=90, control=None, guard=None):
    """Remove persistent files/hook; the facade explicitly verifies reboot next."""
    budget = _Budget(timeout, control, guard)
    package = package_profile()
    with camera._control_lock:
        calls = budget.calls(camera, control)
        control = calls.control
        owner = _existing_owner(control, calls=calls)
        calls.exports()
        bootstrap_work, nonce = _bootstrap_installer(calls, package)
        result = calls.invoke(
            bootstrap_work + "/" + INSTALLER, "remove", manifest=package["manifest"]
        )
        if (
            result.get("removed") is not True
            or result.get("reboot_required") is not True
            or result["installed"]
        ):
            raise HelperInstallError("Persistent removal was not verified", result)
        result.update(
            requires_reboot=True,
            previous_boot_id=result["boot_id"],
            owner_receipt=owner,
            bootstrap_work=bootstrap_work,
            _control=control,
        )
        budget.check()
        return result


def verify_installed(
    camera, expected_profile, previous_boot_id, *, timeout=30, control=None, guard=None
):
    """Observe a new boot and the actual hydrate-only boot cache, without input."""
    budget = _Budget(timeout, control, guard)
    if not _boot_id(previous_boot_id):
        raise ValueError("Previous camera boot identity required")
    package = package_profile()
    if expected_profile != package["manifest"]["profile"]:
        raise ValueError("Exact current packaged helper profile required")
    with camera._control_lock:
        calls = budget.calls(camera, control)
        control, deadline = calls.control, budget.deadline
        while True:
            remaining = budget.remaining()
            if remaining <= 0:
                raise TimeoutError("New-boot helper hydration was not verified")
            value = InstalledSources(package, timeout=remaining).status(
                control, guard=budget.check, deadline=deadline, clock=budget.clock
            )
            cache = value.get("boot_cache", {})
            if (
                value.get("boot_id") != previous_boot_id
                and value["installed"]
                and cache.get("hydrated") is True
                and cache.get("boot_id") == value["boot_id"]
                and cache.get("profile") == expected_profile
                and cache.get("mode") == "bytes-only"
                and cache.get("work")
                == "/dev/shm/osmo360-sdk-cache-" + expected_profile[:12]
            ):
                budget.check()
                return dict(value, autostart_hydration_verified=True)
            if not value["installed"]:
                raise HelperInstallError(
                    "Installation vanished during the verified reboot", value
                )
            control.sleep(min(0.05, max(0, deadline - budget.clock())))


def verify_removed(camera, previous_boot_id, *, timeout=20, control=None, guard=None):
    """Read-only absence proof after the facade observed a genuine reboot."""
    budget = _Budget(timeout, control, guard)
    if not _boot_id(previous_boot_id):
        raise ValueError("Previous camera boot identity required")
    with camera._control_lock:
        calls = budget.calls(camera, control)
        control = calls.control
        calls.exports()
        nonce = secrets.token_hex(6)
        output = "/blackbox/export_tmp/osmo_helpers_" + nonce + ".json"
        program = (
            "import os,json,glob,hashlib,sys;from pathlib import Path as P;"
            "v=[P('/proc/sys/kernel/random/boot_id').read_text().strip(),"
            "os.path.lexists('/data/osmo360-sdk'),glob.glob('/dev/shm/osmo*'),"
            "hashlib.sha256(P('/etc/init.post_boot_product_post.sh').read_bytes()).hexdigest()];"
            "f=open(sys.argv[1],'x');s=os.fstat(f.fileno());json.dump([v,s.st_dev,s.st_ino],f);f.close()"
        )
        calls.run("python3 -c " + Q(program) + " " + Q(output))
        calls.check()
        raw = control.bootstrap.pull_bytes(
            output, max_bytes=16384, timeout=min(10, budget.remaining())
        )
        calls.check()
        value = _json(raw)
        if (
            type(value) is not list
            or len(value) != 3
            or type(value[0]) is not list
            or len(value[0]) != 4
            or any(type(n) is not int or n <= 0 for n in value[1:])
        ):
            raise HelperInstallError("Malformed removal absence report")
        boot, root_exists, ram, hook_sha = value[0]
        if (
            not _boot_id(boot)
            or boot == previous_boot_id
            or root_exists is not False
            or ram != []
            or hook_sha != STOCK_HOOK_SHA
        ):
            raise HelperInstallError(
                "New boot/persistent-root/RAM/stock-hook absence not proven",
                dict(observed=value[0]),
            )
        predicate = (
            "from pathlib import Path as P;p="
            + repr(ROLE)
            + ";assert not any(x.split()[4]==p for x in open('/proc/self/mountinfo'));"
            "assert P(p).read_text().strip()=='peripheral'"
        )
        calls.run("python3 -c " + Q(predicate))
        try:
            runtime = control.bootstrap.query_runtime(budget=budget.check)
        except BaseException as error:
            error.helper_control = control
            raise
        budget.check()
        # Keep the canonical native five-function/SSP validation independent
        # of a caller-provided bootstrap mock or an earlier cached state.
        from ._usb_plan import _validate_runtime

        _validate_runtime(runtime)
        from ._usb_bootstrap import GADGET, FUNCTIONS

        canonical_links = len(runtime.original_links) == len(FUNCTIONS) and all(
            isinstance(raw, str)
            and posixpath.normpath(posixpath.join(GADGET, "configs/c.1", raw))
            == GADGET + "/functions/" + name
            for raw, name in zip(runtime.original_links, FUNCTIONS)
        )
        if (
            not canonical_links
            or runtime.vendor != "0x2ca3"
            or runtime.product != "0x0025"
            or runtime.configuration != "BULK"
        ):
            raise HelperInstallError(
                "Canonical native five-function USB was not restored"
            )
        cleanup = (
            "import os,sys;p=sys.argv[1];s=os.lstat(p);assert (s.st_dev,s.st_ino)=="
            + repr(tuple(value[1:]))
            + ";os.unlink(p)"
        )
        calls.run("python3 -c " + Q(cleanup) + " " + Q(output))
        budget.check()
        return dict(
            removed=True,
            installed=False,
            profile=None,
            boot_id=boot,
            root_absent=True,
            ram_absent=True,
            stock_hook_sha256=hook_sha,
            role_unshadowed=True,
            diagnostic_cleaned=True,
            native_runtime=asdict(runtime),
        )
