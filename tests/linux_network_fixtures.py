"""Fake Linux USB/sysfs host shared by network ownership tests.

All files and command effects stay inside a TemporaryDirectory; no real host
network interface or USB device is accessed.
"""

import json
from pathlib import Path
import subprocess

from osmo360_camera import _linux_network as network

MAC = "96:03:ae:cc:f8:37"
NAME = "enx9603aeccf837"


class FakeHost:
    def __init__(self, root):
        self.root = Path(root)
        self.net = self.root / "class/net"
        self.net.mkdir(parents=True)
        self.bus = self.root / "bus/usb/devices"
        self.bus.mkdir(parents=True)
        self.state = self.root / "netif/links"
        self.state.mkdir(parents=True)
        self.physical = self.root / "devices"
        self.usb = self.physical / "usb1/1-1"
        self.usb.mkdir(parents=True)
        for key, value in dict(
            idVendor="2ca3", idProduct="0025", bDeviceClass="ef", busnum="1", devnum="4"
        ).items():
            (self.usb / key).write_text(value)
        (self.bus / "1-1").symlink_to(self.usb, target_is_directory=True)
        self.interface = self.usb / "1-1:1.5"
        self.interface.mkdir()
        self.nic = self.interface / "net" / NAME
        self.nic.mkdir(parents=True)
        (self.net / NAME).symlink_to(self.nic, target_is_directory=True)
        (self.nic / "device").symlink_to(self.interface, target_is_directory=True)
        (self.nic / "address").write_text(MAC)
        (self.nic / "ifindex").write_text("7")
        (self.state / "7").write_text("ADMIN_STATE=unmanaged\nOPER_STATE=off\n")
        self.present = self.up = False
        self.carrier = True
        self.addresses = []
        self.commands = []
        self.fail_after = self.fail_before = None
        self.now = 0.0
        self.expires_at = None

    def capture(self, camera=None):
        if self.expires_at is not None and self.now >= self.expires_at:
            if network.HOST in self.addresses:
                self.addresses.remove(network.HOST)
            self.expires_at = None
        addresses = [
            dict(
                ifindex=1,
                ifname="lo",
                address="00:00:00:00:00:00",
                flags=["UP"],
                operstate="UNKNOWN",
                addr_info=[dict(family="inet", local="127.0.0.1", prefixlen=8)],
            ),
            dict(
                ifindex=3,
                ifname="wlan0",
                address="02:12:34:56:78:9a",
                flags=["UP"],
                operstate="UP",
                addr_info=[dict(family="inet", local="192.168.1.87", prefixlen=24)],
            ),
        ]
        routes = [
            dict(
                dst="default",
                gateway="192.168.1.1",
                dev="wlan0",
                protocol="dhcp",
                flags=[],
            ),
            dict(
                dst="192.168.1.0/24",
                dev="wlan0",
                protocol="kernel",
                scope="link",
                flags=[],
            ),
        ]
        if self.present:
            addresses.append(
                dict(
                    ifindex=7,
                    ifname=NAME,
                    address=MAC,
                    flags=(["UP", "LOWER_UP"] if self.carrier else ["NO-CARRIER", "UP"])
                    if self.up
                    else [],
                    operstate="UP" if self.up and self.carrier else "DOWN",
                    addr_info=[
                        dict(
                            family="inet",
                            local=value,
                            prefixlen=30,
                            broadcast="192.168.253.3",
                        )
                        for value in self.addresses
                    ],
                )
            )
            if network.HOST in self.addresses:
                routes.extend(
                    [
                        dict(
                            dst=str(network.NETWORK),
                            dev=NAME,
                            protocol="kernel",
                            scope="link",
                            prefsrc=network.HOST,
                            flags=[] if self.up and self.carrier else ["linkdown"],
                        ),
                        dict(
                            type="local",
                            dst=network.HOST,
                            dev=NAME,
                            table="local",
                            protocol="kernel",
                            scope="host",
                            flags=[],
                        ),
                        dict(
                            type="broadcast",
                            dst="192.168.253.3",
                            dev=NAME,
                            table="local",
                            protocol="kernel",
                            scope="link",
                            flags=[],
                        ),
                    ]
                )
        result = {"resolv_conf": "nameserver 127.0.0.53\n", "commands": []}
        for argv, value in (
            (network.ADDR_COMMAND, addresses),
            (network.ROUTE_COMMAND, routes),
        ):
            result["commands"].append(
                dict(argv=list(argv), returncode=0, stdout=json.dumps(value), stderr="")
            )
        if camera:
            selected = (
                dict(dst=camera, dev=NAME, prefsrc=network.HOST, flags=[])
                if self.present
                and self.up
                and self.carrier
                and network.HOST in self.addresses
                else dict(
                    dst=camera,
                    dev="wlan0",
                    gateway="192.168.1.1",
                    prefsrc="192.168.1.87",
                    flags=[],
                )
            )
            result["commands"].append(
                dict(
                    argv=["ip", "-j", "-4", "route", "get", camera],
                    returncode=0,
                    stdout=json.dumps([selected]),
                    stderr="",
                )
            )
        return result

    def run(self, argv, **options):
        self.commands.append(list(argv))
        key = "add" if "add" in argv else "del" if "del" in argv else argv[-1]
        if key == self.fail_before:
            raise subprocess.TimeoutExpired(argv, 5)
        if key == "add":
            self.addresses.append(network.HOST)
            lifetime = argv[argv.index("valid_lft") + 1]
            self.expires_at = (
                None if lifetime == "forever" else self.now + float(lifetime)
            )
        elif key == "del":
            self.addresses.remove(network.HOST)
        elif key == "up":
            self.up = True
        elif key == "down":
            self.up = False
        if key == self.fail_after:
            raise subprocess.TimeoutExpired(argv, 5)
        return subprocess.CompletedProcess(argv, 0, "", "")
