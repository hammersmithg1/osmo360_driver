"""Osmo host provisioning in temporary filesystems; no service/USB mutation."""

from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from osmo360_driver import host_setup


class HostSetupTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)

    def test_clean_host_is_scoped_and_repeated_install_preserves_existing_files(self):
        unrelated = self.root / 'etc/systemd/network/10-wifi.network'
        unrelated.parent.mkdir(parents=True)
        unrelated.write_text('[Match]\nName=wlan0\n[Network]\nDHCP=yes\n')
        before = unrelated.read_bytes()
        self.assertTrue(host_setup.install_network_rules(self.root))
        rules = (self.root / 'etc/udev/rules.d/70-osmo360.rules').read_text()
        first, second = [line for line in rules.splitlines() if line.startswith('ACTION')]
        self.assertIn('ATTRS{idVendor}=="2ca3"', first)
        self.assertIn('ATTRS{idProduct}=="0025"', first)
        self.assertNotIn('DRIVERS', first)
        self.assertIn('ENV{ROVER_OSMO360_USB}=="1"', second)
        self.assertIn('DRIVERS=="cdc_ether"', second)
        self.assertIn('ENV{ROVER_OSMO360_ECM}="1"', second)
        profile = self.root / 'etc/systemd/network/05-osmo360.network'
        config = profile.read_text()
        for setting in ('Property=ROVER_OSMO360_ECM=1', 'Driver=cdc_ether', 'Unmanaged=yes'):
            self.assertIn(setting, config)
        for forbidden in ('Name=', 'MACAddress=', 'Address=', 'Gateway=', 'DHCP='):
            self.assertNotIn(forbidden, config)
        first_mtime = profile.stat().st_mtime_ns
        self.assertFalse(host_setup.install_network_rules(self.root))
        self.assertEqual(profile.stat().st_mtime_ns, first_mtime)
        self.assertEqual(unrelated.read_bytes(), before)

    def test_foreign_target_is_preserved_before_any_provisioning(self):
        profile = self.root / 'etc/systemd/network/05-osmo360.network'
        profile.parent.mkdir(parents=True)
        profile.write_text('# operator configuration\n')
        with self.assertRaisesRegex(RuntimeError, 'owned by another setup'):
            host_setup.install_network_rules(self.root)
        self.assertEqual(profile.read_text(), '# operator configuration\n')
        self.assertFalse((self.root / 'etc/udev/rules.d/70-osmo360.rules').exists())

    def test_exact_legacy_rules_are_replaced_then_retired(self):
        for relative, content in host_setup.LEGACY_FILES.items():
            path = self.root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(content)
        self.assertTrue(host_setup.install_network_rules(self.root))
        for relative in host_setup.LEGACY_FILES:
            self.assertFalse((self.root / relative).exists())
        for relative, expected in host_setup.FILES.items():
            path = self.root / relative
            self.assertEqual(path.read_text(), expected)
            self.assertEqual(path.stat().st_mode & 0o777, 0o644)
        self.assertFalse(host_setup.install_network_rules(self.root))

    def test_modified_legacy_rules_require_explicit_migration_before_any_write(self):
        relative, content = next(iter(host_setup.LEGACY_FILES.items()))
        path = self.root / relative
        path.parent.mkdir(parents=True)
        path.write_text(content + '# operator modification\n')
        with self.assertRaisesRegex(RuntimeError, 'migrate it explicitly'):
            host_setup.install_network_rules(self.root)
        self.assertEqual(path.read_text(), content + '# operator modification\n')
        self.assertFalse(any((self.root / target).exists() for target in host_setup.FILES))

    def test_symlink_target_is_not_followed_or_replaced(self):
        target = self.root / 'operator-file'
        target.write_text('keep')
        path = self.root / next(iter(host_setup.FILES))
        path.parent.mkdir(parents=True)
        path.symlink_to(target)
        with self.assertRaisesRegex(RuntimeError, 'symlink'):
            host_setup.install_network_rules(self.root)
        self.assertEqual(target.read_text(), 'keep')
        self.assertTrue(path.is_symlink())

    def test_standalone_source_runs_without_sdk_installed(self):
        source = Path(host_setup.__file__).read_text()
        for expected in ('changed', 'ready'):
            result = subprocess.run([sys.executable, '-I', '-c', source, '--root', str(self.root)],
                                    cwd=self.root, capture_output=True, text=True, check=True, timeout=10)
            self.assertEqual(result.stdout.strip(), expected)
        self.assertTrue((self.root / 'etc/systemd/network/05-osmo360.network').is_file())


if __name__ == '__main__':
    unittest.main()
