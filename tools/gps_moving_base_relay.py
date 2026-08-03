#!/usr/bin/env python3
"""Byte-transparent local relay for GNSS moving-base and RTK corrections.

The ESP32 modules announce themselves to this UDP endpoint. Legacy moving-base
stream packets retain their point-to-point routes. RTCM packets from module 1's
single NTRIP client are copied to modules 2..5 without decoding or modifying
the correction stream.
"""

from __future__ import annotations

import argparse
import signal
import socket
import struct
import time


MAGIC = b"GMB1"
VERSION = 1
KIND_HEARTBEAT = 1
KIND_STREAM = 2
KIND_RTCM = 3
HEADER = struct.Struct("!4sBBBBIH")
ROUTES = {3: 1, 1: 2}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=21600)
    parser.add_argument("--module-port", type=int, default=21601)
    parser.add_argument("--stats-interval", type=float, default=5.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    running = True

    def stop(_signum: int, _frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind, args.port))
    sock.settimeout(0.5)

    modules: dict[int, tuple[str, int]] = {}
    rx_packets = 0
    rx_bytes = 0
    forwarded_packets = 0
    forwarded_bytes = 0
    rtcm_fanout_packets = 0
    invalid_packets = 0
    no_route_packets = 0
    last_stats = time.monotonic()
    print(f"GPS moving-base relay listening on {args.bind}:{args.port}", flush=True)

    while running:
        try:
            datagram, sender = sock.recvfrom(2048)
        except socket.timeout:
            datagram = b""
            sender = ("", 0)
        except OSError:
            if running:
                raise
            break

        if datagram:
            rx_packets += 1
            rx_bytes += len(datagram)
            if len(datagram) < HEADER.size:
                invalid_packets += 1
            else:
                magic, version, kind, source, _flags, _sequence, length = HEADER.unpack_from(datagram)
                if (
                    magic != MAGIC
                    or version != VERSION
                    or len(datagram) != HEADER.size + length
                    or source < 1
                    or source > 5
                ):
                    invalid_packets += 1
                else:
                    modules[source] = (sender[0], args.module_port)
                    if kind == KIND_RTCM and source == 1:
                        for destination_id in range(2, 6):
                            destination = modules.get(destination_id)
                            if destination is None:
                                no_route_packets += 1
                                continue
                            try:
                                sock.sendto(datagram, destination)
                                forwarded_packets += 1
                                forwarded_bytes += length
                                rtcm_fanout_packets += 1
                            except OSError:
                                no_route_packets += 1
                    elif kind == KIND_STREAM:
                        destination_id = ROUTES.get(source)
                        destination = modules.get(destination_id) if destination_id else None
                        if destination is not None:
                            try:
                                sock.sendto(datagram, destination)
                                forwarded_packets += 1
                                forwarded_bytes += length
                            except OSError:
                                no_route_packets += 1
                        elif destination_id is not None:
                            no_route_packets += 1
                    elif kind not in (KIND_HEARTBEAT, KIND_RTCM):
                        invalid_packets += 1

        now = time.monotonic()
        if now - last_stats >= args.stats_interval:
            learned = ",".join(f"M{module}={address[0]}" for module, address in sorted(modules.items())) or "none"
            print(
                f"modules[{learned}] rx={rx_packets}/{rx_bytes}B "
                f"forwarded={forwarded_packets}/{forwarded_bytes}B "
                f"rtcm_fanout={rtcm_fanout_packets} "
                f"no_route={no_route_packets} invalid={invalid_packets}",
                flush=True,
            )
            last_stats = now

    sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
