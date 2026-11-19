"""Install scoped Linux USB-ECM rules for Osmo without opening the camera.

This module uses only the Python standard library. A deployment tool may send
its source to a fresh host and run ``python3 -c SOURCE`` before installing the
SDK. It changes neither Wi-Fi nor default routes and does not restart network
services. The caller reloads udev/networkd only when installation changed.
"""

import argparse
import os
from pathlib import Path
import tempfile


HEADER = '# Managed by osmo360_driver.host_setup\n'
LEGACY_HEADER = '# Managed by rover scripts/setup-robot.sh\n'
# These are persistent udev property names, already present on attached
# interfaces. Keep them across the source-code extraction: changing them
# would require a USB replug before the new networkd profile could match.
NETWORK_PROPERTY_PREFIX = 'ROVER_OSMO360'


def _network_files(namespace, header, rules_name, profile_name):
    return {
        'etc/udev/rules.d/' + rules_name: header + f'''# Parent USB identity and interface driver are matched in separate rules:
# udev requires all parent-match keys in one rule to refer to the same ancestor.
ACTION=="add|change", SUBSYSTEM=="net", ATTRS{{idVendor}}=="2ca3", ATTRS{{idProduct}}=="0025", ENV{{{namespace}_USB}}="1"
ACTION=="add|change", SUBSYSTEM=="net", ENV{{{namespace}_USB}}=="1", DRIVERS=="cdc_ether", ENV{{{namespace}_ECM}}="1", ENV{{NM_UNMANAGED}}="1"
''',
        'etc/systemd/network/' + profile_name: header + f'''[Match]
Property={namespace}_ECM=1
Driver=cdc_ether

[Link]
Unmanaged=yes
''',
    }


FILES = _network_files(NETWORK_PROPERTY_PREFIX, HEADER, '70-osmo360.rules', '05-osmo360.network')
LEGACY_FILES = _network_files(NETWORK_PROPERTY_PREFIX, LEGACY_HEADER,
                              '70-rover-osmo360.rules', '05-rover-osmo360.network')


def _existing(path):
    if path.is_symlink():
        raise RuntimeError('Refusing a symlink at Osmo network configuration: ' + str(path))
    return path.read_text() if path.exists() else None


def install_network_rules(root=Path('/')):
    """Install atomically per file; validate every target before any mutation.

    Exact legacy files are retired only after the replacement files exist.
    Operator-owned or modified legacy configurations require an explicit
    manual migration; the installer never overwrites or removes them.
    ``root`` allows packaging and offline validation in a temporary tree.
    """
    root = Path(root)
    pending, legacy = [], []
    for relative, content in FILES.items():
        path = root / relative
        current = _existing(path)
        if current is not None and not current.startswith(HEADER):
            raise RuntimeError('Osmo network configuration is owned by another setup: ' + str(path))
        if current != content:
            pending.append((path, content))
    for relative, expected in LEGACY_FILES.items():
        path = root / relative
        current = _existing(path)
        if current is not None:
            if current != expected:
                raise RuntimeError('Legacy Osmo configuration was modified; migrate it explicitly: ' + str(path))
            legacy.append(path)
    for path, content in pending:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, prefix='.osmo360-', delete=False) as output:
                temporary = Path(output.name)
                output.write(content)
                output.flush()
                os.fsync(output.fileno())
            temporary.chmod(0o644)
            os.replace(temporary, path)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
    for path in legacy:
        path.unlink()
    return bool(pending or legacy)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path('/'),
                        help='Filesystem root for packaging/offline validation (default: /)')
    args = parser.parse_args(argv)
    print('changed' if install_network_rules(args.root) else 'ready')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
