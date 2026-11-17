"""Exact script data transfer and bounded, nonexecuting publication; no USB."""

import json
import os
from pathlib import Path
import struct
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import zlib

from osmo360_camera import _amt
from osmo360_camera._usb_bootstrap import USBBootstrap
from osmo360_camera._usb_bundle import build_bundle
from osmo360_camera._device import script_bundle as device
from osmo360_camera.transport import RequestTimeout


def sources():
    return {
        name: "# Exact nonexecuted fixture: " + name + "\n" for name in device.NAMES
    }


def manifest():
    return dict(
        schema=1,
        files={
            name: dict(
                source=value,
                bytes=len(value.encode("ascii")),
                sha256=device.digest(value.encode("ascii")),
            )
            for name, value in sources().items()
        },
    )


def packed(value=None, *, plain=None):
    if plain is None:
        plain = json.dumps(
            value or manifest(), sort_keys=True, separators=(",", ":")
        ).encode("ascii")
    data = zlib.compress(plain, 9)
    return data, (len(data), device.digest(data), len(plain), device.digest(plain))


class ScriptBundleTests(unittest.TestCase):
    def test_exact_round_trip_and_ready_marker_after_all_readbacks(self):
        data, args = packed()
        files = device.decode_bundle(data, *args)
        self.assertEqual(files, {k: v.encode() for k, v in sources().items()})
        link = os.link
        with tempfile.TemporaryDirectory() as directory:
            work = Path(directory)

            def publish_ready(source, destination, **kwargs):
                self.assertFalse((work / "bundle.ready").exists())
                for name, value in files.items():
                    self.assertEqual((work / name).read_bytes(), value)
                return link(source, destination, **kwargs)

            with patch.object(device.os, "link", side_effect=publish_ready) as atomic:
                device.publish(work, files, args[-1])
            atomic.assert_called_once()
            self.assertEqual((work / "bundle.ready").read_text(), args[-1] + "\n")
            self.assertFalse((work / "bundle.ready.tmp").exists())

    def test_compressed_corruption_is_rejected_before_decompression(self):
        data, args = packed()
        bad = bytes([data[0] ^ 1]) + data[1:]
        with patch.object(device.zlib, "decompressobj") as decoder:
            with self.assertRaisesRegex(ValueError, "Compressed bundle"):
                device.decode_bundle(bad, *args)
            decoder.assert_not_called()

    def test_truncated_trailing_concatenated_and_wrong_plain_hash_are_rejected(self):
        data, args = packed()
        for altered in (data[:-4], data + b"extra", data + data):
            with self.subTest(length=len(altered)), self.assertRaises(ValueError):
                device.decode_bundle(
                    altered, len(altered), device.digest(altered), *args[2:]
                )
        with self.assertRaises(ValueError):
            device.decode_bundle(data, *args[:-1], "0" * 64)

    def test_expansion_is_bounded_and_never_flushes_to_complete_a_bomb(self):
        data = zlib.compress(b"x" * (1024 * 1024))
        original = zlib.decompressobj
        calls = []

        class Bounded:
            def __init__(self):
                self.inner = original()

            def decompress(self, value, limit):
                calls.append(limit)
                return self.inner.decompress(value, limit)

            def __getattr__(self, key):
                if key == "flush":
                    raise AssertionError("Unbounded flush is forbidden")
                return getattr(self.inner, key)

        with patch.object(device.zlib, "decompressobj", Bounded):
            with self.assertRaises(ValueError):
                device.decode_bundle(
                    data, len(data), device.digest(data), 65536, "0" * 64
                )
        self.assertEqual(calls, [65537])
        for args in (
            (32769, "0" * 64, 1, "0" * 64),
            (1, "0" * 64, 65537, "0" * 64),
            (True, "0" * 64, 1, "0" * 64),
        ):
            with self.subTest(args=args), self.assertRaises(ValueError):
                device.decode_bundle(b"x", *args)

    def test_file_allowlist_duplicate_keys_and_per_file_integrity(self):
        cases = []
        value = manifest()
        value["files"]["../outside"] = value["files"].pop("apply.sh")
        cases.append(value)
        value = manifest()
        value["files"]["extra.py"] = dict(value["files"]["apply.sh"])
        cases.append(value)
        value = manifest()
        value["schema"] = True
        cases.append(value)
        value = manifest()
        value["files"]["apply.sh"]["sha256"] = "0" * 64
        cases.append(value)
        value = manifest()
        value["files"]["apply.sh"]["bytes"] += 1
        cases.append(value)
        value = manifest()
        value["files"]["apply.sh"]["source"] = "\0"
        cases.append(value)
        for value in cases:
            data, args = packed(value)
            with self.subTest(value=value), self.assertRaises(ValueError):
                device.decode_bundle(data, *args)
        plain = json.dumps(manifest(), separators=(",", ":")).encode()
        duplicate = plain.replace(b'"schema":1', b'"schema":1,"schema":1')
        data, args = packed(plain=duplicate)
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            device.decode_bundle(data, *args)

    def test_nonfresh_or_symlink_destination_never_overwrites_or_publishes_ready(self):
        data, args = packed()
        files = device.decode_bundle(data, *args)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            work = root / "work"
            work.mkdir()
            outside = root / "outside"
            outside.write_text("keep")
            for symlink in (False, True):
                target = work / "apply.sh"
                if symlink:
                    target.symlink_to(outside)
                else:
                    target.write_text("existing")
                with self.subTest(symlink=symlink), self.assertRaises(ValueError):
                    device.publish(work, files, args[-1])
                self.assertEqual(outside.read_text(), "keep")
                self.assertFalse((work / "bundle.ready").exists())
                self.assertEqual(list(work.iterdir()), [target])
                target.unlink()

    def test_mid_publication_failure_leaves_only_partial_owned_data_and_no_ready(self):
        data, args = packed()
        files = device.decode_bundle(data, *args)
        opened = os.open
        count = 0
        with tempfile.TemporaryDirectory() as directory:
            work = Path(directory)

            def fail_third(path, flags, *rest):
                nonlocal count
                if flags & os.O_CREAT:
                    count += 1
                    if count == 3:
                        raise OSError("injected no space")
                return opened(path, flags, *rest)

            with (
                patch.object(device.os, "open", side_effect=fail_third),
                patch.object(
                    Path, "unlink", side_effect=AssertionError("No failure cleanup")
                ),
            ):
                with self.assertRaisesRegex(OSError, "no space"):
                    device.publish(work, files, args[-1])
            self.assertEqual(len(list(work.iterdir())), 2)
            self.assertFalse((work / "bundle.ready").exists())
            for path in work.iterdir():
                self.assertEqual(path.read_bytes(), files[path.name])

    def test_uncertain_start_quarantines_upload_without_replay_unpack_or_cleanup(self):
        bundle = build_bundle(sources(), "/dev/shm/osmo-ncm-offline-bundle")
        replies = iter(
            [
                bytes.fromhex("0500e0f3"),
                RequestTimeout("lost START ACK"),
                bytes.fromhex("030000f5"),
            ]
        )
        calls = []

        def request(command_set, command, payload, **kwargs):
            calls.append(payload)
            response = next(replies)
            if isinstance(response, BaseException):
                raise response
            return SimpleNamespace(payload=response)

        transport = SimpleNamespace(request=request)
        session = _amt.D1Session(transport, reuse_owned_id=True)
        bootstrap = USBBootstrap(transport, session=session)
        with self.assertRaisesRegex(RequestTimeout, "lost START"):
            for command in bundle["commands"]:
                bootstrap.run_checked(command)
        self.assertTrue(session.quarantined)
        self.assertEqual([struct.unpack_from("<H", p)[0] for p in calls], [4, 0, 2])
        self.assertEqual(len(session.operations), 1)
        self.assertEqual(session.operations[0]["command"], bundle["commands"][0])
        count = len(calls)
        with self.assertRaisesRegex(RuntimeError, "quarantined"):
            bootstrap.run_checked(bundle["commands"][1])
        self.assertEqual(len(calls), count)


if __name__ == "__main__":
    unittest.main()
