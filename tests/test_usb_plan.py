"""Package-local USB plans, reconstructed uploads and isolated netdev state."""

import ast
import base64
from contextlib import redirect_stdout
from dataclasses import replace
import hashlib
import io
from pathlib import Path
import shlex
import subprocess
import tempfile
import unittest

from osmo360_camera._usb_bootstrap import USBRuntime, FUNCTIONS
from osmo360_camera._usb_plan import build_usb_plan
from osmo360_camera._device.script_bundle import decode_bundle


def runtime(*, retained=False):
    return USBRuntime(
        "a600000.dwc3",
        "0x2ca3",
        "0x0025",
        "BULK",
        tuple("../../../../usb_gadget/g1/functions/" + name for name in FUNCTIONS),
        "super-speed-plus",
        "aa:e4:f8:1c:10:86",
        "96:03:ae:cc:f8:37",
        "usb0" if retained else None,
        "0x1002" if retained else None,
        not retained,
        "usb0" if retained else "(unnamed net_device)",
        (("wlan0", "192.168.1.2/24"),),
    )


def configuration(source):
    for node in ast.parse(source).body:
        if isinstance(node, ast.Assign) and any(
            isinstance(t, ast.Name) and t.id == "CONFIG" for t in node.targets
        ):
            return ast.literal_eval(node.value)
    raise AssertionError("Generated device helper lacks its CONFIG")


class USBPlanTests(unittest.TestCase):
    def build(self, value=None, **options):
        return build_usb_plan(
            value or runtime(),
            tag=options.pop("tag", "offline-test"),
            nonce=options.pop("nonce", "0123456789abcdef" * 2),
            port=options.pop("port", 55000),
            **options,
        )

    def test_absent_and_retained_plans_have_exact_runtime_and_lease_values(self):
        for retained in (False, True):
            with self.subTest(retained=retained):
                plan = self.build(runtime(retained=retained))
                self.assertEqual(
                    set(plan["scripts"]),
                    {
                        "apply.sh",
                        "rollback.sh",
                        "watchdog.sh",
                        "netdev.py",
                        "lifecycle.py",
                        "ecm_dhcp.py",
                        "lease.py",
                    },
                )
                self.assertEqual(plan["original_netdev_absent"], not retained)
                net = configuration(plan["scripts"]["netdev.py"])
                self.assertEqual(net["original_netdev_absent"], not retained)
                self.assertEqual(net["original_net_if"], "usb0" if retained else None)
                self.assertEqual(net["original_flags"], "0x1002" if retained else None)
                self.assertEqual(net["host_mac"], "96:03:ae:cc:f8:37")
                lease = configuration(plan["scripts"]["lease.py"])
                self.assertEqual(
                    (lease["lease_seconds"], lease["setup_seconds"]), (10, 90)
                )
                self.assertEqual(lease["dev_mac"], "aa:e4:f8:1c:10:86")
                self.assertEqual(lease["nonce"], "0123456789abcdef" * 2)
                self.assertEqual(lease["port"], 55000)
                self.assertEqual(plan["lease"]["request_bytes"], 33)
                self.assertEqual(plan["lease"]["reply_bytes"], 49)
                self.assertIn(
                    "96:03:ae:cc:f8:37 </dev/null", plan["scripts"]["apply.sh"]
                )
                self.assertIn(
                    "while alive and not (work / 'lease.closed').exists():",
                    plan["scripts"]["ecm_dhcp.py"],
                )
                self.assertNotIn(
                    "time.monotonic() + 110", plan["scripts"]["ecm_dhcp.py"]
                )

    def test_dynamic_udc_macs_and_retained_interface_are_not_hardcoded(self):
        value = replace(
            runtime(retained=True),
            udc="fe800000.dwc3",
            net_if="usb7",
            initial_ifname="usb7",
            dev_addr="02:11:22:33:44:55",
            host_addr="02:11:22:33:44:56",
        )
        plan = self.build(value)
        sources = "\n".join(plan["scripts"].values())
        for old in ("a600000.dwc3", "aa:e4:f8:1c:10:86", "96:03:ae:cc:f8:37", "usb0"):
            self.assertNotIn(old, sources)
        self.assertIn("fe800000.dwc3", sources)
        self.assertIn("02:11:22:33:44:56 </dev/null", plan["scripts"]["apply.sh"])
        self.assertEqual(
            configuration(plan["scripts"]["netdev.py"])["original_net_if"], "usb7"
        )

    def test_every_upload_reconstructs_the_final_hashed_scripts(self):
        plan = self.build(tag="x" * 32)
        rebuilt = {name: bytearray() for name in ("unpack.py", "scripts.z")}
        for command in plan["commands"]:
            self.assertTrue(command.isascii())
            self.assertLessEqual(len(command.encode()), 500)
            if command.startswith("printf %s "):
                words = shlex.split(command)
                self.assertEqual(words[3:7], ["|", "base64", "-d", ">>"])
                name = Path(words[7]).name
                self.assertEqual(str(Path(words[7]).parent), plan["work_dir"])
                rebuilt[name].extend(base64.b64decode(words[2], validate=True))
        bundle = plan["bundle"]
        self.assertEqual(
            hashlib.sha256(rebuilt["unpack.py"]).hexdigest(), bundle["loader_sha256"]
        )
        decoded = decode_bundle(
            bytes(rebuilt["scripts.z"]),
            bundle["compressed_bytes"],
            bundle["compressed_sha256"],
            bundle["plain_bytes"],
            bundle["plain_sha256"],
        )
        for name, source in plan["scripts"].items():
            self.assertEqual(decoded[name], source.encode(), name)
            self.assertEqual(
                plan["sha256"][name], hashlib.sha256(decoded[name]).hexdigest()
            )
            self.assertEqual(plan["byte_lengths"][name], len(decoded[name]))
            if name.endswith(".sh"):
                subprocess.run(["/bin/sh", "-n"], input=source, text=True, check=True)
            else:
                compile(source, name, "exec")
        self.assertIn("bundle.ready", plan["commands"][-1])

    def test_apply_and_rollback_preserve_reviewed_order(self):
        plan = self.build()
        apply, rollback = plan["scripts"]["apply.sh"], plan["scripts"]["rollback.sh"]
        operations = [
            'lifecycle.py" claim',
            "watchdog.ready",
            "usb.touched",
            "printf '\\n' > \"$G/UDC\"",
            "printf high-speed",
            "f6.owned",
            'ln -s "$F" "$C/f6"',
            'printf \'%s\' "$U" > "$G/UDC"',
            'netdev.py" discover',
            "ip addr add",
            'lease.py" receiver',
            'ecm_dhcp.py"',
        ]
        offsets = [apply.index(item) for item in operations]
        self.assertEqual(offsets, sorted(offsets))
        operations = [
            'lifecycle.py" quiesce',
            'lease.py" close',
            'netdev.py" stop-dhcp',
            'netdev.py" cleanup',
            "printf '\\n' > \"$G/UDC\" || failed=35",
            "printf %s super-speed-plus",
            'rm "$C/f6"',
            'printf \'%s\' "$U" > "$G/UDC"',
            'netdev.py" verify',
        ]
        offsets = [rollback.index(item) for item in operations]
        self.assertEqual(offsets, sorted(offsets))
        self.assertIn(
            'exec python3 "$W/lease.py" watchdog', plan["scripts"]["watchdog.sh"]
        )
        for index, name in enumerate(FUNCTIONS, 1):
            self.assertIn("../../../../usb_gadget/g1/functions/" + name, apply)
            self.assertNotIn(f'rm "$C/f{index}"', apply + rollback)

    def test_invalid_runtime_or_lease_inputs_are_rejected(self):
        for value in (
            replace(runtime(), original_max_speed="high-speed"),
            replace(runtime(retained=True), original_flags="0x1003"),
            replace(runtime(), original_links=runtime().original_links[:-1]),
            replace(runtime(), host_addr=runtime().dev_addr),
            replace(runtime(), camera_ipv4=(("wlan0", "192.168.253.1/30"),)),
        ):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.build(value)
        for options in (
            dict(tag="bad;tag"),
            dict(nonce="00"),
            dict(port=True),
            dict(port=80),
        ):
            with self.subTest(options=options), self.assertRaises(ValueError):
                self.build(**options)

    def test_generated_netdev_preflight_and_discovery_on_fake_tree(self):
        for retained in (False, True):
            with (
                self.subTest(retained=retained),
                tempfile.TemporaryDirectory() as folder,
            ):
                plan = self.build(runtime(retained=retained))
                namespace = {"__name__": "isolated_netdev_helper"}
                exec(
                    compile(plan["scripts"]["netdev.py"], "netdev.py", "exec"),
                    namespace,
                )
                root = Path(folder)
                function, net, work = root / "ecm", root / "net", root / "work"
                for directory in (function, net, work):
                    directory.mkdir()
                (function / "dev_addr").write_text("aa:e4:f8:1c:10:86\0\n")
                (function / "host_addr").write_text("96:03:ae:cc:f8:37\0\n")
                (function / "ifname").write_text(
                    "usb0" if retained else "(unnamed net_device)"
                )
                namespace["CONFIG"].update(
                    function=str(function), sys_net=str(net), work=str(work)
                )
                ip_calls = []

                def ip(*args):
                    ip_calls.append(args)
                    return ""

                namespace["ip"] = ip

                def add_interface():
                    interface = net / "usb0"
                    interface.mkdir()
                    (interface / "address").write_text("aa:e4:f8:1c:10:86\n")
                    (interface / "flags").write_text("0x1002\n")

                if retained:
                    add_interface()
                with redirect_stdout(io.StringIO()):
                    namespace["preflight"]()
                self.assertFalse((work / "ifname").exists())
                self.assertEqual(bool(ip_calls), retained)
                if not retained:
                    add_interface()
                with redirect_stdout(io.StringIO()):
                    namespace["discover"]()
                self.assertEqual((work / "ifname").read_text(), "usb0\n")
                namespace["ip"] = lambda *args: "1: usb0 inet 10.0.0.1/24\n"
                with self.assertRaises(ValueError), redirect_stdout(io.StringIO()):
                    namespace["discover"]()


if __name__ == "__main__":
    unittest.main()
