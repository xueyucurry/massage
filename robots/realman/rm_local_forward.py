#!/usr/bin/env python3
"""Simple local TCP port forwarder for the RM web pendant."""

from __future__ import annotations

import argparse
import select
import signal
import socket
import threading


STOP = threading.Event()


def relay(client: socket.socket, target_host: str, target_port: int) -> None:
    upstream = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    upstream.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    upstream.connect((target_host, target_port))
    sockets = [client, upstream]
    try:
        while not STOP.is_set():
            readable, _, _ = select.select(sockets, [], [], 0.5)
            if not readable:
                continue
            for src in readable:
                data = src.recv(65536)
                if not data:
                    return
                dst = upstream if src is client else client
                dst.sendall(data)
    finally:
        for sock in sockets:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass


def serve(bind_host: str, bind_port: int, target_host: str, target_port: int) -> None:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((bind_host, bind_port))
    server.listen(16)
    server.settimeout(0.5)
    try:
        while not STOP.is_set():
            try:
                client, _ = server.accept()
            except socket.timeout:
                continue
            worker = threading.Thread(
                target=relay,
                args=(client, target_host, target_port),
                daemon=True,
            )
            worker.start()
    finally:
        server.close()


def main() -> int:
    parser = argparse.ArgumentParser(description="Local TCP forwarder")
    parser.add_argument("--bind-host", default="127.0.0.1")
    parser.add_argument("--target-host", required=True)
    parser.add_argument(
        "--map",
        action="append",
        required=True,
        help="Port mapping in LISTEN:TARGET format, e.g. 18080:80",
    )
    args = parser.parse_args()

    def stop_handler(signum, frame):  # type: ignore[unused-argument]
        STOP.set()

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    threads = []
    for item in args.map:
        listen_port_str, target_port_str = item.split(":", 1)
        thread = threading.Thread(
            target=serve,
            args=(
                args.bind_host,
                int(listen_port_str),
                args.target_host,
                int(target_port_str),
            ),
            daemon=True,
        )
        thread.start()
        threads.append(thread)

    for thread in threads:
        thread.join()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
