"""Camera RAM only: one-host DHCP, no router/DNS options, finite lifetime.

Runs as root only on the user's camera. The USB overlay supplies a verified
interface and its exact host MAC. No subprocesses or persistent writes.
"""
import json
import os
import re
import signal
import socket
import struct
import sys
import time
from pathlib import Path

SERVER = '192.168.253.1'
CLIENT = '192.168.253.2'
MASK = '255.255.255.252'
COOKIE = b'\x63\x82\x53\x63'


def response(packet, expected_mac):
    """Ignore other clients, malformed packets, and foreign DHCP servers."""
    if len(packet) < 240 or packet[:3] != b'\x01\x01\x06':
        return None
    if packet[24:28] != bytes(4) or packet[28:34] != expected_mac:
        return None  # No relays; only the new USB Ethernet host.
    if packet[236:240] != COOKIE:
        return None
    options = {}
    offset = 240
    while offset < len(packet):
        kind = packet[offset]
        offset += 1
        if kind == 255:
            break
        if kind == 0:
            continue
        if offset >= len(packet):
            return None
        size = packet[offset]
        offset += 1
        if offset + size > len(packet) or kind in options:
            return None
        options[kind] = packet[offset:offset + size]
        offset += size
    message = options.get(53)
    server = socket.inet_aton(SERVER)
    client = socket.inet_aton(CLIENT)
    ciaddr = packet[12:16]
    if ciaddr not in (bytes(4), client):
        return None
    if options.get(54, server) != server:
        return None
    if message == b'\x01':
        reply_type = 2
    elif message == b'\x03':
        requested = options.get(50, packet[12:16])
        if requested != client:
            return None
        reply_type = 5
    else:
        return None
    bootp = bytearray(236)
    bootp[:4] = b'\x02\x01\x06\x00'
    bootp[4:8] = packet[4:8]  # Echo transaction ID.
    bootp[10:12] = b'\x80\x00' if ciaddr == bytes(4) else bytes(2)
    bootp[12:16] = ciaddr  # A renewal is unicasted to the address already owned.
    bootp[16:20] = client
    bootp[20:24] = server
    bootp[28:44] = packet[28:44]
    opts = (COOKIE + bytes((53, 1, reply_type, 54, 4)) + server
            + bytes((51, 4)) + struct.pack('!I', 30)
            + bytes((1, 4)) + socket.inet_aton(MASK)
            + bytes((58, 4)) + struct.pack('!I', 10)
            + bytes((59, 4)) + struct.pack('!I', 20) + b'\xff')
    return bytes(bootp) + opts + bytes(max(0, 300 - len(bootp) - len(opts)))


def main():
    work = Path(sys.argv[1])
    mac_text = sys.argv[2]
    if not re.fullmatch(r'/dev/shm/osmo-ncm-[A-Za-z0-9_-]{1,32}', str(work)):
        raise ValueError('Unexpected RAM directory')
    if not re.fullmatch(r'(?:[0-9a-f]{2}:){5}[0-9a-f]{2}', mac_text):
        raise ValueError('Invalid exact USB host MAC')
    interface = (work / 'ifname').read_text().strip()
    if not re.fullmatch(r'[A-Za-z0-9_.:-]{1,15}', interface):
        raise ValueError('Invalid verified interface')
    expected_mac = bytes.fromhex(mac_text.replace(':', ''))
    alive = True

    def stop(*_):
        nonlocal alive
        alive = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    deadline = time.monotonic() + 110
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.setsockopt(socket.SOL_SOCKET, 25, interface.encode() + b'\0')
        sock.bind(('', 67))
        sock.settimeout(.25)
        with (work / 'dhcp.jsonl').open('x') as log:
            with (work / 'dhcp.pid').open('x') as pid_file:
                pid_file.write(str(os.getpid()) + '\n')
            (work / 'dhcp.ready').write_text('READY\n')
            while alive and time.monotonic() < deadline:
                try:
                    packet, source = sock.recvfrom(4096)
                except socket.timeout:
                    continue
                reply = response(packet, expected_mac)
                if reply:
                    destination = (socket.inet_ntoa(reply[12:16])
                                   if reply[12:16] != bytes(4) else '255.255.255.255')
                    sock.sendto(reply, (destination, 68))
                    log.write(json.dumps({'monotonic': time.monotonic(),
                                          'source': source, 'reply_type': reply[242]}) + '\n')
                    log.flush()


if __name__ == '__main__':
    main()
