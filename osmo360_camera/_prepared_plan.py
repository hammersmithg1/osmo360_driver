"""Build a fresh USB preview plan for the prepared wake path.

fast=True is ONLY for the045 camera watcher, which launches apply.sh after the
resident native QS wake GET0 and completed real EXIT. That detached launch has
no AMT USB reply to protect with the canonical2s grace. All other callers keep
the canonical plan. This module never opens USB, runs scripts or changes files.
"""

import hashlib
import shlex

from ._usb_bundle import build_bundle
from ._usb_plan import build_usb_plan


GRACE = """# The calling AMT RPC must finish before its own USB link is rebound.
sleep 2
"""

WAIT_SOURCE = """import os,sys,time
def wait_ready(paths, clock=time.monotonic, sleep=time.sleep, stat=os.stat):
    deadline = clock() + 5.0
    while True:
        def nonempty(path):
            try:
                return stat(path).st_size > 0
            except OSError:
                return False
        ready = all(nonempty(path) for path in paths)
        remaining = deadline - clock()
        if remaining <= 0:
            return False
        if ready:
            return True
        sleep(min(0.05, remaining))
"""


def wait_command(exit_code, *names):
    # No fractional shell sleep dependency. Only the waiting primitive changes;
    # the canonical shell's file/PID/kill checks immediately below remain final.
    paths = " ".join('"$W/' + name + '"' for name in names)
    return (
        "python3 - "
        + paths
        + " <<'OSMO046_READY' || exit "
        + str(exit_code)
        + "\n"
        + WAIT_SOURCE
        + "raise SystemExit(0 if wait_ready(sys.argv[1:]) else 1)\nOSMO046_READY\n"
    )


def old_wait(name):
    return (
        "attempt=0\n"
        f'while [ ! -s "$W/{name}" ] && [ "$attempt" -lt 5 ]; do\n'
        "  sleep 1\n"
        "  attempt=$((attempt + 1))\n"
        "done\n"
    )


def patch_apply(source):
    """Fail closed on any canonical marker drift, including duplicates."""
    replacements = [
        (
            GRACE,
            "# Prepared045 wake/EXIT completed before detached apply; no AMT reply grace.\n",
        ),
        (old_wait("watchdog.ready"), wait_command(72, "watchdog.ready")),
        (old_wait("lease.receiver.ready"), wait_command(93, "lease.receiver.ready")),
        (old_wait("dhcp.ready"), wait_command(85, "dhcp.ready", "dhcp.pid")),
    ]
    for old, _ in replacements:
        if source.count(old) != 1:
            raise RuntimeError(
                "Canonical046 apply marker changed/multiplied: " + old.splitlines()[0]
            )
    for old, new in replacements:
        source = source.replace(old, new, 1)
    return source


def build_prepared_fast_plan(runtime, tag=None, nonce=None, port=None, *, fast=False):
    """Return a fresh plan; default is byte-identical to build_usb_plan.

    Explicit fast=True changes only apply.sh and the representation derived
    from its bytes: seven SHA/length records, compressed bundle/upload/ready
    marker. Setup90s, lease10s, rollback, lifecycle and ownership are untouched.
    A lost apply intent/launch is still single-use and must never be retried.
    """
    if type(fast) is not bool:
        raise TypeError("fast must be bool")
    plan = build_usb_plan(runtime, tag=tag, nonce=nonce, port=port)
    if not fast:
        return plan
    files = plan["scripts"]
    files["apply.sh"] = patch_apply(files["apply.sh"])
    bundle = build_bundle(files, plan["work_dir"])
    commands = bundle.pop("commands")
    bundle.pop("files")
    plan.update(
        commands=commands,
        bundle=bundle,
        sha256={
            name: hashlib.sha256(source.encode("ascii")).hexdigest()
            for name, source in files.items()
        },
        byte_lengths={
            name: len(source.encode("ascii")) for name, source in files.items()
        },
        verify_command="sha256sum "
        + " ".join(shlex.quote(plan["work_dir"] + "/" + name) for name in files),
    )
    if any(
        not command.isascii() or len(command.encode("ascii")) > 500
        for command in commands
    ):
        raise ValueError("046 upload command exceeds reviewed500-byte bound")
    return plan
