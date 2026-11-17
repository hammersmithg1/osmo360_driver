"""Internal, explicit USB-overlay bootstrap through an already-owned transport.

No USB handle is opened here. The caller owns the authorized D1 session and the
camera lifecycle. Only the small runtime report is exported to the stock file
server's diagnostic directory; that directory is not claimed to be RAM. The
collector, upload, and ownership record live in the verified /dev/shm tmpfs.
"""

from dataclasses import dataclass
import hashlib
import ipaddress
import json
import math
import posixpath
import re
import secrets
import shlex
import threading
import time


GADGET = "/sys/kernel/config/usb_gadget/g1"
FUNCTIONS = ("ffs.diag", "ffs.bulk", "ffs.bulk1", "ffs.bulk5", "ffs.bulk4")
NATIVE_LINKS = tuple("../../functions/" + name for name in FUNCTIONS)
PLACEHOLDER = "(unnamed net_device)"
MAX_REPORT_BYTES = 16384


class BootstrapError(RuntimeError):
    def __init__(self, message, *, cleanup_error=None):
        self.cleanup_error = cleanup_error
        super().__init__(message)


def _mac(value):
    if (
        not isinstance(value, str)
        or not re.fullmatch(r"(?:[0-9a-f]{2}:){5}[0-9a-f]{2}", value)
        or int(value[:2], 16) & 1
        or value == "00:00:00:00:00:00"
    ):
        raise BootstrapError("Expected an exact nonzero unicast runtime MAC")
    return value


@dataclass(frozen=True)
class USBRuntime:
    udc: str
    vendor: str
    product: str
    configuration: str
    original_links: tuple
    original_max_speed: str
    dev_addr: str
    host_addr: str
    net_if: str | None
    original_flags: str | None
    original_netdev_absent: bool
    initial_ifname: str
    camera_ipv4: tuple
    ram_dir: str = "/dev/shm"
    function_name: str = "ecm.ecm"

    def overlay_values(self):
        """Arguments for the reviewed overlay spec; speed is retained separately."""
        return dict(
            udc=self.udc,
            net_if=self.net_if,
            original_flags=self.original_flags,
            ram_dir=self.ram_dir,
            vendor=self.vendor,
            product=self.product,
            configuration=self.configuration,
            original_links=self.original_links,
            original_ipv4=(),
            function_name=self.function_name,
            original_netdev_absent=self.original_netdev_absent,
            initial_ifname=self.initial_ifname,
            dev_addr=self.dev_addr,
            host_addr=self.host_addr,
        )


def decode_runtime(value):
    """Fail closed unless this is the observed native BULK/ECM baseline."""
    try:
        if (
            not isinstance(value, dict)
            or type(value.get("schema")) is not int
            or value["schema"] != 1
        ):
            raise BootstrapError("Unknown runtime report schema")
        if type(value.get("uid")) is not int or value["uid"] != 0:
            raise BootstrapError("Runtime collector did not run as root")
        if (
            value.get("ram_realpath") != "/dev/shm"
            or value.get("ram_filesystem") != "tmpfs"
        ):
            raise BootstrapError(
                "Collector directory is not the canonical /dev/shm tmpfs"
            )
        udc = value["udc"]
        if not isinstance(udc, str) or not re.fullmatch(r"[A-Za-z0-9_.:-]{1,100}", udc):
            raise BootstrapError("Invalid runtime UDC")
        if (value["vendor"], value["product"], value["configuration"]) != (
            "0x2ca3",
            "0x0025",
            "BULK",
        ):
            raise BootstrapError("Only shooting-mode 2ca3:0025 BULK is supported")
        links = value["links"]
        if not isinstance(links, dict) or set(links) != {f"f{i}" for i in range(1, 6)}:
            raise BootstrapError("Native five USB function links changed")
        original_links = tuple(links[f"f{i}"] for i in range(1, 6))
        for raw, function in zip(original_links, FUNCTIONS):
            if (
                not isinstance(raw, str)
                or not raw.isascii()
                or "\n" in raw
                or "\0" in raw
                or posixpath.normpath(posixpath.join(GADGET, "configs/c.1", raw))
                != GADGET + "/functions/" + function
            ):
                raise BootstrapError("Native five USB function links changed")
        if value["max_speed"] != "super-speed-plus":
            raise BootstrapError("Unexpected original USB max_speed")
        if value.get("tools") != {
            name: True
            for name in ("python3", "ip", "setsid", "base64", "sha256sum", "readlink")
        }:
            raise BootstrapError("Required stock bootstrap tools are unavailable")
        dev, host = _mac(value["dev_addr"]), _mac(value["host_addr"])
        if dev == host:
            raise BootstrapError("ECM host/device MACs are identical")
        matches = value["mac_matches"]
        if not isinstance(matches, list) or len(matches) > 1:
            raise BootstrapError("Ambiguous ECM network device MAC")
        reported = value["ifname"]
        if not matches:
            if reported != PLACEHOLDER:
                raise BootstrapError(
                    "Absent ECM netdev needs the exact native placeholder"
                )
            interface, flags, absent = None, None, True
        else:
            item = matches[0]
            interface, flags = item["name"], item["flags"]
            if not isinstance(interface, str) or not re.fullmatch(
                r"[A-Za-z0-9_.-]{1,15}", interface
            ):
                raise BootstrapError("Invalid retained ECM interface name")
            if reported != interface or flags != "0x1002":
                raise BootstrapError(
                    "Retained ECM must match its ifname and native DOWN flags"
                )
            absent = False
        rows = value["ipv4"]
        if not isinstance(rows, list):
            raise BootstrapError("Missing camera IPv4 inventory")
        ipv4 = []
        for row in rows:
            name, cidr = row["interface"], row["cidr"]
            if not isinstance(name, str) or not re.fullmatch(
                r"[A-Za-z0-9_.:-]{1,15}", name
            ):
                raise BootstrapError("Invalid camera IPv4 interface")
            parsed = ipaddress.IPv4Interface(cidr)
            if interface is not None and name == interface:
                raise BootstrapError("Retained ECM already has IPv4")
            if parsed.network.overlaps(ipaddress.IPv4Network("192.168.253.0/30")):
                raise BootstrapError("USB test /30 overlaps a camera IPv4 network")
            ipv4.append((name, str(parsed)))
        return USBRuntime(
            udc,
            value["vendor"],
            value["product"],
            value["configuration"],
            original_links,
            value["max_speed"],
            dev,
            host,
            interface,
            flags,
            absent,
            reported,
            tuple(ipv4),
        )
    except BootstrapError:
        raise
    except (KeyError, TypeError, ValueError) as error:
        raise BootstrapError("Malformed runtime report: " + str(error)) from error


# Uploaded as data, never executed by importing this host module. The fixed
# collector only reads the native USB baseline and writes one owned JSON report.
COLLECTOR = r"""import json,os,re,shutil,subprocess,sys
from pathlib import Path
mode,work_text=sys.argv[1:]
if not re.fullmatch(r'/dev/shm/osmo-ncm-query[0-9a-f]{12}',work_text):raise ValueError('work path')
work=Path(work_text);nonce=work.name[-12:]
if os.getuid()!=0 or os.path.realpath('/dev/shm')!='/dev/shm':raise ValueError('root/tmpfs')
mounts=[x.split() for x in Path('/proc/mounts').read_text().splitlines()]
if [x[2]for x in mounts if x[1]=='/dev/shm']!=['tmpfs']:raise ValueError('tmpfs')
if work.is_symlink() or not work.is_dir():raise ValueError('work directory')
output=Path('/blackbox/export_tmp/osmo_probe_'+nonce+'.json')
owner=work/'export.owner'
if mode=='cleanup':
 if owner.exists():
  if owner.is_symlink():raise ValueError('owner symlink')
  expected=json.loads(owner.read_text())
  try:st=output.lstat()
  except FileNotFoundError:st=None
  if st is not None:
   if output.is_symlink() or [st.st_dev,st.st_ino]!=expected:raise ValueError('export ownership')
   output.unlink()
  owner.unlink()
 (work/'query.py').unlink()
 work.rmdir()
elif mode=='query':
 def read(path):
  b=Path(path).read_bytes().rstrip(b'\0\r\n')
  if b'\0'in b:raise ValueError('internal NUL')
  return b.decode('ascii')
 g=Path('/sys/kernel/config/usb_gadget/g1');c=g/'configs/c.1';f=g/'functions/ecm.ecm'
 if not f.is_dir() or f.is_symlink():raise ValueError('existing ECM function required')
 links={p.name:os.readlink(p)for p in c.iterdir()if p.is_symlink()}
 dev=read(f/'dev_addr').lower();host=read(f/'host_addr').lower()
 matches=[]
 for p in Path('/sys/class/net').iterdir():
  if read(p/'address').lower()==dev:matches.append({'name':p.name,'flags':read(p/'flags')})
 result=subprocess.run(['ip','-o','-4','addr','show'],capture_output=True,text=True,timeout=2,check=True)
 ipv4=[]
 for line in result.stdout.splitlines():
  match=re.match(r'^\d+:\s+(\S+)\s+inet\s+(\S+)',line)
  if not match:raise ValueError('ip address row')
  ipv4.append({'interface':match[1].split('@')[0],'cidr':match[2]})
 data={'schema':1,'uid':os.getuid(),'ram_realpath':os.path.realpath('/dev/shm'),'ram_filesystem':'tmpfs',
       'udc':read(g/'UDC'),'vendor':read(g/'idVendor'),'product':read(g/'idProduct'),
       'configuration':read(c/'strings/0x409/configuration'),'links':links,'max_speed':read(g/'max_speed'),
       'dev_addr':dev,'host_addr':host,'ifname':read(f/'ifname'),'mac_matches':matches,'ipv4':ipv4,
       'tools':{n:bool(shutil.which(n))for n in ('python3','ip','setsid','base64','sha256sum','readlink')}}
 if os.path.realpath('/blackbox')!='/blackbox' or not Path('/blackbox').is_dir():raise ValueError('blackbox root')
 try:output.parent.mkdir(mode=0o700)
 except FileExistsError:pass
 if os.path.realpath(output.parent)!=str(output.parent) or not output.parent.is_dir():raise ValueError('export directory')
 fd=os.open(output,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
 with os.fdopen(fd,'w')as stream:
  st=os.fstat(stream.fileno())
  with owner.open('x')as record:json.dump([st.st_dev,st.st_ino],record)
  json.dump(data,stream,separators=(',',':'))
else:raise ValueError('mode')
"""


class USBBootstrap:
    """Internal adapter; caller serializes other file transfers and owns reconnect."""

    def __init__(self, transport, *, session):
        if getattr(session, "transport", None) is not transport:
            raise ValueError("D1 session must use the same owned USB transport")
        self.transport, self.session = transport, session
        self._lock = threading.RLock()
        self.last_report = None

    def run_checked(self, command, *, timeout_ms=3000, pre_send=None):
        if (
            not isinstance(command, str)
            or not command.isascii()
            or "\0" in command
            or not 1 <= len(command) <= 500
        ):
            raise ValueError("Reviewed D1 commands must be 1..500 NUL-free ASCII bytes")
        result = (
            self.session.run(command, timeout_ms=timeout_ms, pre_send=pre_send)
            if pre_send is not None
            else self.session.run(command, timeout_ms=timeout_ms)
        )
        if (
            not isinstance(result, dict)
            or result.get("completed") is not True
            or result.get("success") is not True
            or result.get("result") != 0
        ):
            raise BootstrapError("D1 diagnostic command did not complete successfully")
        return result

    def pull_bytes(self, path, *, max_bytes=MAX_REPORT_BYTES, timeout=10):
        """Use stock op08/read/MD5 on the existing reader; retain unrelated events."""
        if (
            type(max_bytes) is not int
            or not 1 <= max_bytes <= MAX_REPORT_BYTES
            or isinstance(timeout, bool)
            or not isinstance(timeout, (int, float))
            or not math.isfinite(timeout)
            or not 0 < timeout <= 60
        ):
            raise ValueError("Bounded pull requires 1..16384 bytes and 0<timeout<=60")
        # Existing pure state machine; packaging can move it to a private module.
        from ._file_pull import PullReceiver

        with self._lock:
            pull = PullReceiver(
                path, sender=self.transport.sender, receiver=0x28, max_bytes=max_bytes
            )
            started = time.monotonic_ns()
            deadline = time.monotonic() + timeout
            reply = self.transport.request(
                0, 0x2A, pull.request_payload(), receiver=0x28, timeout=min(2, timeout)
            )
            if not reply.payload or reply.payload[0]:
                raise BootstrapError("Stock file server refused diagnostic pull")
            while time.monotonic() < deadline:
                consumed = False
                for event in list(self.transport.events):
                    stamp, message = event
                    if (
                        stamp < started
                        or message.command != (0, 0x2A)
                        or message.is_reply
                        or message.sender != 0x28
                        or message.receiver != self.transport.sender
                    ):
                        continue
                    try:
                        self.transport.events.remove(event)
                    except ValueError:
                        continue
                    consumed = True
                    for answer in pull.feed(message):
                        self.transport._write(answer)
                    if pull.error:
                        raise BootstrapError("Diagnostic pull failed: " + pull.error)
                    if pull.done:
                        if not pull.checksum_verified:
                            raise BootstrapError("Diagnostic pull lacks verified MD5")
                        if time.monotonic() >= deadline:
                            raise BootstrapError(
                                "Diagnostic pull completed after deadline"
                            )
                        return bytes(pull.data)
                if not self.transport.connected:
                    raise BootstrapError(
                        "USB transport disconnected during diagnostic pull"
                    )
                if not consumed:
                    time.sleep(min(0.005, max(0, deadline - time.monotonic())))
            raise BootstrapError("Diagnostic pull deadline")

    def query_runtime(self, *, budget=None):
        """Read actual USB values, pull the tiny report, delete only owned files."""
        import base64

        if budget is not None and not callable(budget):
            raise TypeError("Optional runtime budget must be callable")

        def check():
            if budget is not None:
                budget()

        def run(command, **options):
            check()
            value = (
                self.run_checked(command, pre_send=check, **options)
                if budget is not None
                else self.run_checked(command, **options)
            )
            check()
            return value

        with self._lock:
            nonce = secrets.token_hex(6)
            work = "/dev/shm/osmo-ncm-query" + nonce
            script, output = (
                work + "/query.py",
                "/blackbox/export_tmp/osmo_probe_" + nonce + ".json",
            )
            q = shlex.quote
            report = dict(
                work_dir=work,
                export_path=output,
                export_storage="stock blackbox diagnostic directory",
                cleanup_complete=False,
            )
            self.last_report = report
            created = ready = False
            error = runtime = None
            try:
                check()
                # Record the conclusive mkdir receipt before a late post-check
                # can fail; cleanup must retain ownership of that RAM path.
                creation_options = {"pre_send": check} if budget is not None else {}
                self.run_checked(
                    '[ "$(readlink -f /dev/shm)" = /dev/shm ] && '
                    '[ "$(awk \'$2=="/dev/shm" {print $3}\' /proc/mounts)" = tmpfs ] && '
                    f"umask 077 && mkdir {q(work)} && : > {q(script)}",
                    **creation_options,
                )
                created = True
                check()
                data = COLLECTOR.encode("ascii")
                for offset in range(0, len(data), 210):
                    chunk = base64.b64encode(data[offset : offset + 210]).decode(
                        "ascii"
                    )
                    run(f"printf %s {q(chunk)} | base64 -d >> {q(script)}")
                digest = hashlib.sha256(data).hexdigest()
                run(f'[ "$(sha256sum {q(script)} | cut -d" " -f1)" = {digest} ]')
                ready = True
                run(f"python3 {q(script)} query {q(work)}", timeout_ms=5000)
                check()
                raw = self.pull_bytes(output)
                check()
                report["bytes"] = len(raw)
                runtime = decode_runtime(json.loads(raw))
            except BaseException as exc:
                error = exc
            finally:
                if created:
                    try:
                        if ready:
                            run(f"python3 {q(script)} cleanup {q(work)}")
                        else:
                            run(
                                f"test ! -L {q(script)} && rm -- {q(script)} && rmdir {q(work)}"
                            )
                        report["cleanup_complete"] = True
                    except BaseException as cleanup:
                        report["cleanup_error"] = f"{type(cleanup).__name__}: {cleanup}"
                        if budget is not None:
                            report["cleanup_deferred"] = True
            if error is not None:
                if report.get("cleanup_error"):
                    raise BootstrapError(
                        str(error), cleanup_error=report["cleanup_error"]
                    ) from error
                raise error
            if not report["cleanup_complete"]:
                raise BootstrapError(
                    "Could not verify diagnostic file cleanup",
                    cleanup_error=report.get("cleanup_error"),
                )
            check()
            return runtime
