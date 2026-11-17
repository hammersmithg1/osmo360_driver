"""Camera installer tests use a private fake filesystem, never USB or SSH."""

import importlib.util
import json
import os
from pathlib import Path
import stat
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import zlib

SOURCE = (
    Path(__file__).resolve().parents[1] / "osmo360_camera/_device/helper_install.py"
)
spec = importlib.util.spec_from_file_location(
    "camera_helper_install_device_tests", SOURCE
)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


class DeviceInstallerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.base = Path(self.tmp.name).resolve()
        self.ram, self.data, self.etc, self.blackbox = (
            self.base / n for n in ("ram", "data", "etc", "blackbox")
        )
        for path in (self.ram, self.data, self.etc, self.blackbox):
            path.mkdir(mode=0o700)
        self.hook, self.parent = self.etc / "post.sh", self.etc / "main.sh"
        self.original, self.parent_bytes = (
            b"#!/bin/sh\necho stock\n",
            b"#!/bin/sh\nsh product\n",
        )
        self.hook.write_bytes(self.original)
        self.hook.chmod(0o755)
        self.parent.write_bytes(self.parent_bytes)
        self.parent.chmod(0o755)
        self.boot_id, self.upper = self.base / "boot_id", self.base / "upper_post.sh"
        self.boot_id.write_text("test-boot")
        self.patches = patch.multiple(
            m,
            ROOT=self.data / "osmo360-sdk",
            RAM=self.ram,
            EXPORT=self.blackbox / "export_tmp",
            HOOK=self.hook,
            PARENT=self.parent,
            HOOK_SHA=m.sha(self.original),
            PARENT_SHA=m.sha(self.parent_bytes),
            UID=os.getuid(),
            GID=os.getgid(),
            BOOT_ID=self.boot_id,
            UPPER_HOOK=self.upper,
        )
        self.patches.start()
        self.nonce = "123456abcdef"
        self.work = self.ram / ("osmo-helper-install-" + self.nonce)
        self.work.mkdir(mode=0o700)
        self.source = SOURCE.read_bytes()
        self.files = {name: (SOURCE.parent / name).read_bytes() for name in m.FILES}
        core = dict(
            schema=1,
            installer_sha256=m.sha(self.source),
            files={
                name: dict(size=len(data), sha256=m.sha(data))
                for name, data in self.files.items()
            },
        )
        self.manifest = dict(core, profile=m.sha(m.canonical(core)))
        plain = m.canonical(
            dict(
                schema=1,
                files={name: data.decode("ascii") for name, data in self.files.items()},
            )
        )
        self.packed = zlib.compress(plain, 9)
        self.request = dict(
            self.manifest,
            autostart=True,
            packed=dict(
                bytes=len(self.packed),
                sha256=m.sha(self.packed),
                plain_bytes=len(plain),
                plain_sha256=m.sha(plain),
            ),
        )
        self.write_bootstrap()
        self.script_patch = patch.object(
            m, "__file__", str(self.work / "helper_install.py")
        )
        self.script_patch.start()

    def tearDown(self):
        self.script_patch.stop()
        self.patches.stop()
        self.tmp.cleanup()

    def write_bootstrap(self):
        for name, data in (
            ("helper_install.py", self.source),
            ("request.json", m.canonical(self.request)),
            ("bundle.z", self.packed),
        ):
            (self.work / name).write_bytes(data)
            (self.work / name).chmod(0o600)

    def install(self):
        return m.install(self.work, self.nonce)

    def test_install_and_remove_exact_stock_hook(self):
        result = self.install()
        self.assertTrue(result["installed"])
        self.assertEqual(
            self.hook.read_bytes(),
            m.installed_hook(self.original, self.manifest["profile"]),
        )
        self.assertEqual((m.ROOT / "hook.original").read_bytes(), self.original)
        self.assertEqual(stat.S_IMODE(self.hook.stat().st_mode), 0o755)
        version, manifest = m.validate_version(self.manifest["profile"])
        self.assertEqual(manifest, self.manifest)
        self.assertEqual(
            {p.name for p in version.iterdir()},
            set(m.FILES) | {"helper_install.py", "manifest.json"},
        )
        removed = m.remove("bbbbbbbbbbbb")
        self.assertTrue(removed["removed"])
        self.assertTrue(removed["reboot_required"])
        self.assertEqual(self.hook.read_bytes(), self.original)
        self.assertFalse(m.ROOT.exists())
        self.assertTrue(m.remove("cccccccccccc")["removed"])

    def test_absent_status_and_foreign_root(self):
        self.assertFalse(m.status()["installed"])
        m.ROOT.mkdir(mode=0o700)
        (m.ROOT / "foreign").write_bytes(b"keep")
        with self.assertRaises((ValueError, FileNotFoundError)):
            m.status()
        with self.assertRaises((ValueError, FileNotFoundError)):
            m.remove(self.nonce)
        self.assertEqual((m.ROOT / "foreign").read_bytes(), b"keep")

    def test_trailing_bundle_rejected_before_hook_or_root(self):
        self.packed += b"trailing"
        self.request["packed"].update(bytes=len(self.packed), sha256=m.sha(self.packed))
        self.write_bootstrap()
        with self.assertRaises(ValueError):
            self.install()
        self.assertEqual(self.hook.read_bytes(), self.original)
        self.assertFalse(m.ROOT.exists())

    def test_decompression_bomb_and_profile_paths_rejected(self):
        packed = zlib.compress(b"Z" * 1000000)
        request = dict(
            self.request,
            packed=dict(
                bytes=len(packed),
                sha256=m.sha(packed),
                plain_bytes=100,
                plain_sha256=m.sha(b"Z" * 100),
            ),
        )
        with self.assertRaises(ValueError):
            m.decode_bundle(packed, request)
        value = json.loads(m.canonical(self.manifest))
        value["files"]["../../foreign"] = value["files"].pop(next(iter(m.FILES)))
        with self.assertRaises(ValueError):
            m.profile_manifest(value)
        with self.assertRaises(ValueError):
            m.profile_manifest(dict(self.manifest, profile="0" * 64))

    def test_foreign_hook_not_overwritten_on_remove(self):
        self.install()
        foreign = self.hook.read_bytes() + b"echo someone else\n"
        self.hook.write_bytes(foreign)
        with self.assertRaises(ValueError):
            m.remove("bbbbbbbbbbbb")
        self.assertEqual(self.hook.read_bytes(), foreign)
        self.assertEqual((m.ROOT / "hook.original").read_bytes(), self.original)

    def test_interrupted_terminal_journal_can_remove_changed_hook(self):
        original_json = m.json_file

        def interrupt(path, value, **options):
            if value["phase"] == "INSTALLED":
                raise RuntimeError("power loss before terminal journal")
            return original_json(path, value, **options)

        with patch.object(m, "json_file", side_effect=interrupt):
            with self.assertRaises(RuntimeError):
                self.install()
        self.assertEqual(m.state()["phase"], "HOOK_INTENT")
        self.assertNotEqual(self.hook.read_bytes(), self.original)
        self.assertTrue(m.remove("bbbbbbbbbbbb")["removed"])
        self.assertEqual(self.hook.read_bytes(), self.original)

    def test_partial_version_removed_before_hook_intent(self):
        original_create = m.create

        def interrupt(path, data, *args, **kwargs):
            if Path(path).name == "prepared_preview_wake_047.py":
                raise OSError("ENOSPC before hook")
            return original_create(path, data, *args, **kwargs)

        with patch.object(m, "create", side_effect=interrupt):
            with self.assertRaises(OSError):
                self.install()
        self.assertEqual(self.hook.read_bytes(), self.original)
        self.assertTrue(m.remove("bbbbbbbbbbbb")["removed"])
        self.assertFalse(m.ROOT.exists())

    def test_partial_install_hook_temporary_removed_by_journal_nonce(self):
        original_replace = m.replace

        def interrupt(path, data, **options):
            if Path(path) == self.hook:
                temporary = self.hook.with_name(
                    self.hook.name + ".osmo-" + options["nonce"]
                )
                temporary.write_bytes(data[: len(data) // 2])
                temporary.chmod(0o600)
                raise OSError("power loss during temporary hook")
            return original_replace(path, data, **options)

        with patch.object(m, "replace", side_effect=interrupt):
            with self.assertRaises(OSError):
                self.install()
        temporary = self.hook.with_name(self.hook.name + ".osmo-" + self.nonce)
        self.assertTrue(temporary.exists())
        m.remove("bbbbbbbbbbbb")
        self.assertFalse(temporary.exists())

    def test_immutable_file_hash_and_symlink_rejected(self):
        self.install()
        version = m.ROOT / "versions" / self.manifest["profile"]
        path = version / next(iter(m.FILES))
        path.chmod(0o600)
        path.write_bytes(b"foreign")
        path.chmod(0o400)
        with self.assertRaises(ValueError):
            m.status()
        version.chmod(0o700)
        path.unlink()
        path.symlink_to(self.hook)
        version.chmod(0o500)
        with self.assertRaises(ValueError):
            m.status()

    def test_stage_exact_group_fresh_inodes_no_execution(self):
        self.install()
        dest = self.ram / "osmo-usb-standby-abcdef123456"
        dest.mkdir(mode=0o700)
        result = m.stage(self.manifest["profile"], "owner", dest, "test-boot")
        self.assertTrue(result["staged"])
        self.assertTrue(result["installed"])
        self.assertEqual(set(result["files"]), set(m.GROUPS["owner"]))
        version = m.ROOT / "versions" / self.manifest["profile"]
        for name in m.GROUPS["owner"]:
            self.assertEqual((dest / name).read_bytes(), self.files[name])
            self.assertNotEqual(
                (dest / name).stat().st_ino, (version / name).stat().st_ino
            )
        with self.assertRaises(ValueError):
            m.stage(self.manifest["profile"], "owner", dest, "test-boot")
        self.assertFalse((dest / "config.json").exists())

    def test_stage_symlink_cannot_escape(self):
        self.install()
        foreign = self.base / "foreign"
        foreign.mkdir(mode=0o700)
        dest = self.ram / "osmo-usb-standby-abcdef123456"
        dest.symlink_to(foreign, target_is_directory=True)
        with self.assertRaises(ValueError):
            m.stage(self.manifest["profile"], "owner", dest, "test-boot")
        self.assertEqual(list(foreign.iterdir()), [])

    def test_boot_cache_only_sources_same_boot_idempotent(self):
        self.install()
        cache = m.hydrate(self.manifest["profile"], "test-boot")
        self.assertEqual(m.hydrate(self.manifest["profile"], "test-boot"), cache)
        self.assertEqual(
            m.status()["boot_cache"],
            dict(
                hydrated=True,
                boot_id="test-boot",
                profile=self.manifest["profile"],
                work=str(cache),
                mode="bytes-only",
            ),
        )
        self.assertEqual(
            set(p.name for p in cache.iterdir()), set(m.FILES) | {"cache.json"}
        )
        with self.assertRaises(ValueError):
            m.validate_cache(self.manifest["profile"], "different-boot")

    def test_pending_boot_copy_never_adopted_disk_fallback(self):
        self.install()
        cache = m.cache_path(self.manifest["profile"])
        cache.mkdir(mode=0o700)
        name = next(iter(m.FILES))
        (cache / name).write_bytes(b"partial not Python")
        (cache / name).chmod(0o600)
        self.assertEqual(m.status()["boot_cache"]["phase"], "pending")
        with self.assertRaises((ValueError, FileNotFoundError)):
            m.hydrate(self.manifest["profile"], "test-boot")
        dest = self.ram / "osmo-usb-standby-abcdef123456"
        dest.mkdir(mode=0o700)
        m.stage(self.manifest["profile"], "owner", dest, "test-boot")
        self.assertEqual((dest / name).read_bytes(), self.files[name])
        self.assertEqual((cache / name).read_bytes(), b"partial not Python")

    def test_first_marker_and_first_journal_partial_removable(self):
        m.ROOT.mkdir(mode=0o700)
        (m.ROOT / "managed.json").write_bytes(m.canonical(m.MARKER)[:10])
        (m.ROOT / "managed.json").chmod(0o600)
        self.assertTrue(m.remove("bbbbbbbbbbbb")["removed"])
        original_create = m.create

        def interrupt(path, data, *args, **kwargs):
            if Path(path).name == "install.json":
                Path(path).write_bytes(data[:12])
                Path(path).chmod(0o600)
                raise OSError("power loss during first journal")
            return original_create(path, data, *args, **kwargs)

        with patch.object(m, "create", side_effect=interrupt):
            with self.assertRaises(OSError):
                self.install()
        self.assertEqual(self.hook.read_bytes(), self.original)
        self.assertTrue(m.remove("cccccccccccc")["removed"])

    def test_interrupted_remove_restoration_can_resume(self):
        self.install()
        original_replace = m.replace

        def interrupt(path, data, **options):
            if Path(path) == self.hook:
                temporary = self.hook.with_name(
                    self.hook.name + ".osmo-" + options["nonce"]
                )
                temporary.write_bytes(data[:5])
                temporary.chmod(0o600)
                raise OSError("power loss during removal")
            return original_replace(path, data, **options)

        with patch.object(m, "replace", side_effect=interrupt):
            with self.assertRaises(OSError):
                m.remove("bbbbbbbbbbbb")
        self.assertEqual(m.state()["phase"], "REMOVE_INTENT")
        self.assertTrue(m.remove("cccccccccccc")["removed"])
        self.assertEqual(self.hook.read_bytes(), self.original)

    def test_foreign_cache_inventory_not_swept_or_adopted(self):
        self.install()
        cache = m.cache_path(self.manifest["profile"])
        cache.mkdir(mode=0o700)
        (cache / "foreign").write_bytes(b"keep")
        with self.assertRaises(ValueError):
            m.status()
        self.assertEqual((cache / "foreign").read_bytes(), b"keep")

    def test_export_bounded_exclusive_actual_inode(self):
        result = m.export(self.nonce, "status", m.status())
        path = m.EXPORT / ("osmo_helpers_" + self.nonce + ".json")
        self.assertEqual(json.loads(path.read_bytes()), result)
        self.assertEqual(
            result["export_identity"],
            dict(device=path.stat().st_dev, inode=path.stat().st_ino),
        )
        self.assertLess(path.stat().st_size, m.MAX_REPORT)
        with self.assertRaises(FileExistsError):
            m.export(self.nonce, "status", m.status())

    def test_environment_pid1_namespace_and_persistent_overlays(self):
        mounts = (
            "tmpfs " + str(self.ram) + " tmpfs rw 0 0\n"
            "/dev/sda13 /overlay ext4 rw,noexec 0 0\n"
            "overlay /data overlay rw,upperdir=/overlay/data 0 0\n"
            "overlay /etc overlay rw,upperdir=/overlay/etc 0 0\n"
        )
        original_read, original_stat = Path.read_text, Path.stat
        facts = {
            "/proc/mounts": mounts,
            "/proc/1/comm": "aosp_init\n",
            "/proc/cmdline": "boot=aosp",
        }

        def read_text(path, *args, **kwargs):
            return (
                facts[str(path)]
                if str(path) in facts
                else original_read(path, *args, **kwargs)
            )

        namespace = {"/proc/self/ns/mnt": 42, "/proc/1/ns/mnt": 42}

        def stat_path(path, *args, **kwargs):
            if str(path) in namespace:
                return SimpleNamespace(st_dev=1, st_ino=namespace[str(path)])
            return original_stat(path, *args, **kwargs)

        with (
            patch.object(m.os, "getuid", return_value=0),
            patch.object(m.sys, "platform", "linux"),
            patch.object(m, "private"),
            patch.object(m, "read", return_value=b"firmware"),
            patch.object(m, "SYS_SHA", m.sha(b"firmware")),
            patch.object(Path, "read_text", read_text),
            patch.object(Path, "stat", stat_path),
        ):
            self.assertEqual(m.environment(), "test-boot")
            namespace["/proc/self/ns/mnt"] = 43
            with self.assertRaisesRegex(ValueError, "mount namespace"):
                m.environment()
            namespace["/proc/self/ns/mnt"] = 42
            facts["/proc/mounts"] = mounts.replace(
                "upperdir=/overlay/etc", "upperdir=/tmp/etc"
            )
            with self.assertRaisesRegex(ValueError, "persistent rw overlay"):
                m.environment()

    def test_shared_sticky_ram_parent_is_valid_but_nonsticky_writable_is_not(self):
        self.ram.chmod(0o1777)
        m.ram_root()
        self.ram.chmod(0o777)
        with self.assertRaises(ValueError):
            m.ram_root()
        self.ram.chmod(0o700)

    def test_stock_overlay_copy_remaining_is_honestly_reported(self):
        self.install()
        self.assertFalse(m.state()["original_upper_present"])
        # Fake overlay copy-up: merged hook restoration also writes upper bytes.
        original_replace = m.replace

        def copy_up(path, data, **options):
            result = original_replace(path, data, **options)
            if Path(path) == self.hook:
                self.upper.write_bytes(data)
                self.upper.chmod(0o755)
            return result

        with patch.object(m, "replace", side_effect=copy_up):
            removed = m.remove("bbbbbbbbbbbb")
        self.assertTrue(removed["stock_overlay_copy_remaining"])
        self.assertEqual(self.upper.read_bytes(), self.original)
        self.assertEqual(self.hook.read_bytes(), self.original)


if __name__ == "__main__":
    unittest.main()
