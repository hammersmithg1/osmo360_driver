"""Explicit USB socket binding and logging for the preview transport."""

from __future__ import annotations

import ipaddress
import logging
import re
import socket
import sys


def bind_socket(sock, local_address=None, interface=None):
    """Bind one IPv4 socket to an explicit source/interface without fallback.

    Darwin IP_BOUND_IF=25 is verified in the local Apple SDK netinet/in.h:432
    (MacOSX.sdk, CommandLineTools); Python3.15 may omit the exported constant.
    Linux uses only Python's SO_BINDTODEVICE constant and propagates permission
    errors. Both Linux and macOS paths have been tested with this camera.
    """
    if local_address is not None:
        if not isinstance(local_address, str):
            raise ValueError('local_address must be an IPv4 address')
        address = ipaddress.IPv4Address(local_address)
        if address.is_unspecified or address.is_multicast or int(address) == 0xffffffff:
            raise ValueError('local_address must name a specific unicast IPv4 address')
        local_address = str(address)
    if interface is not None:
        if not isinstance(interface, str) or not re.fullmatch(r'[A-Za-z0-9_.:-]{1,15}', interface):
            raise ValueError('Invalid interface name')
        index = socket.if_nametoindex(interface)
        if sys.platform == 'darwin':
            sock.setsockopt(socket.IPPROTO_IP, getattr(socket, 'IP_BOUND_IF', 25), index)
        elif sys.platform.startswith('linux'):
            option = getattr(socket, 'SO_BINDTODEVICE', None)
            if option is None:
                raise OSError('Strict interface binding is unavailable on this Python/Linux')
            sock.setsockopt(socket.SOL_SOCKET, option, interface.encode('ascii') + b'\0')
        else:
            raise OSError('Strict interface binding is unsupported on this platform')
    sock.bind((local_address or '0.0.0.0', 0))


def log(message: str) -> None:
    logging.getLogger(__name__).info(message)
