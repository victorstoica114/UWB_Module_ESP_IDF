#!/usr/bin/env python3
import argparse
import os
import re
import select
import socket
import sys
import time


class Ansi:
    RESET = "\033[0m"
    DIM = "\033[2m"
    RED = "\033[31m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    BLUE = "\033[34m"
    MAGENTA = "\033[35m"
    CYAN = "\033[36m"


def enable_windows_vt_mode():
    if sys.platform != "win32":
        return

    try:
        import ctypes

        kernel32 = ctypes.windll.kernel32
        handle = kernel32.GetStdHandle(-11)  # STD_OUTPUT_HANDLE
        mode = ctypes.c_uint32()
        if kernel32.GetConsoleMode(handle, ctypes.byref(mode)):
            mode.value |= 0x0004  # ENABLE_VIRTUAL_TERMINAL_PROCESSING
            kernel32.SetConsoleMode(handle, mode)
    except Exception:
        pass


def supports_color(no_color=False, force_color=False):
    if no_color:
        return False
    if force_color:
        return True

    force_color_env = os.environ.get("FORCE_COLOR", "").strip().lower()
    if force_color_env in ("1", "true", "yes", "on"):
        return True

    return sys.stdout.isatty()


def paint(text, color, enabled):
    if not enabled:
        return text
    return f"{color}{text}{Ansi.RESET}"


def normalize_level(level_code):
    levels = {
        "D": "DEBUG",
        "I": "INFO",
        "W": "WARN",
        "E": "ERROR",
    }
    return levels.get(level_code, "INFO")


def level_color(level):
    if level == "ERROR":
        return Ansi.RED
    if level == "WARN":
        return Ansi.YELLOW
    if level == "DEBUG":
        return Ansi.CYAN
    return Ansi.GREEN


def classify_plain_message(message):
    lowered = message.lower()
    if (
        " error" in lowered
        or "failed" in lowered
        or "timeout" in lowered
        or "panic" in lowered
        or "exception" in lowered
    ):
        return "ERROR"
    if "warn" in lowered or "fallback" in lowered or "retry" in lowered:
        return "WARN"
    if "progress" in lowered:
        return "DEBUG"
    return "INFO"


def format_payload(text, color_enabled):
    match = re.match(r"^(\[[^\]]+\])\s+(\[[^\]]+\])\s+(.*)$", text)
    if not match:
        level = classify_plain_message(text)
        level_tag = paint(f"[{level[0]}]", level_color(level), color_enabled)
        return f"{level_tag} {text}"

    host, uptime, message = match.groups()
    idf_match = re.match(r"^\[([DIWE])\]\[([^\]]+)\]\s+(.*)$", message)
    host_col = paint(host, Ansi.MAGENTA, color_enabled)
    uptime_col = paint(uptime, Ansi.BLUE, color_enabled)

    if idf_match:
        level_code, log_tag, plain_message = idf_match.groups()
        level = normalize_level(level_code)
        color = level_color(level)
        level_tag = paint(f"[{level_code}]", color, color_enabled)
        log_tag_col = paint(f"[{log_tag}]", Ansi.CYAN, color_enabled)
        message_col = paint(plain_message, color, color_enabled)
        return f"{level_tag} {log_tag_col} {host_col} {uptime_col} {message_col}"

    level = classify_plain_message(message)
    color = level_color(level)
    level_tag = paint(f"[{level[0]}]", color, color_enabled)
    message_col = paint(message, color, color_enabled)
    return f"{level_tag} {host_col} {uptime_col} {message_col}"


def close_client(sock, clients, buffers):
    try:
        sock.close()
    except OSError:
        pass
    clients.pop(sock, None)
    buffers.pop(sock, None)


def open_output_file(path):
    if not path:
        return None

    absolute = os.path.abspath(path)
    parent = os.path.dirname(absolute)
    if parent:
        os.makedirs(parent, exist_ok=True)
    return open(absolute, "a", encoding="utf-8", buffering=1024 * 1024)


def main():
    parser = argparse.ArgumentParser(description="Listen for ESP32 TCP logs")
    parser.add_argument("--host", default="0.0.0.0", help="Local bind host")
    parser.add_argument("--port", type=int, default=6055, help="TCP port")
    parser.add_argument(
        "--no-color",
        action="store_true",
        help="Disable ANSI colors even in an interactive terminal",
    )
    parser.add_argument(
        "--force-color",
        action="store_true",
        help="Force ANSI colors even when terminal auto-detection fails",
    )
    parser.add_argument(
        "--output",
        default="",
        help="Append all received log lines to this file",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Do not print every log line to the terminal",
    )
    parser.add_argument(
        "--summary-interval",
        type=float,
        default=5.0,
        help="Seconds between quiet-mode progress summaries; 0 disables them",
    )
    args = parser.parse_args()
    enable_windows_vt_mode()
    color_enabled = supports_color(
        no_color=args.no_color, force_color=args.force_color
    )
    output_file = open_output_file(args.output)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        server.bind((args.host, args.port))
        server.listen(8)
        server.setblocking(False)
    except OSError as exc:
        print(f"ERROR: cannot bind {args.host}:{args.port}: {exc}", flush=True)
        return 1

    print(f"Listening for wireless logs on {args.host}:{args.port}", flush=True)
    if output_file is not None:
        print(f"Writing wireless logs to {os.path.abspath(args.output)}", flush=True)

    clients = {}
    buffers = {}
    received_lines = 0
    received_bytes = 0
    last_summary = time.monotonic()

    def write_output(line):
        if output_file is not None:
            output_file.write(line + "\n")

    def emit_event(message):
        print(message, flush=True)
        write_output(message)

    def emit_log_line(addr, text):
        nonlocal received_lines, received_bytes
        received_lines += 1
        received_bytes += len(text) + 1

        source_plain = f"{addr[0]}:{addr[1]}"
        file_line = f"{source_plain} | {format_payload(text, False)}"
        write_output(file_line)

        if not args.quiet:
            source = paint(source_plain, Ansi.DIM, color_enabled)
            payload = format_payload(text, color_enabled)
            print(f"{source} | {payload}", flush=True)

    def maybe_emit_summary():
        nonlocal last_summary
        if not args.quiet or args.summary_interval <= 0:
            return

        now = time.monotonic()
        if now - last_summary < args.summary_interval:
            return

        last_summary = now
        if output_file is not None:
            output_file.flush()
        print(
            "summary: "
            f"clients={len(clients)} lines={received_lines} "
            f"bytes={received_bytes}",
            flush=True,
        )

    try:
        while True:
            read_list = [server]
            read_list.extend(clients.keys())
            readable, _, _ = select.select(read_list, [], [], 0.5)

            for ready in readable:
                if ready is server:
                    conn, addr = server.accept()
                    conn.setblocking(False)
                    clients[conn] = addr
                    buffers[conn] = b""
                    emit_event(f"Client connected: {addr[0]}:{addr[1]}")
                    continue

                addr = clients.get(ready, ("unknown", 0))
                try:
                    data = ready.recv(4096)
                except OSError:
                    close_client(ready, clients, buffers)
                    continue

                if not data:
                    pending = buffers.get(ready, b"")
                    if pending:
                        text = pending.decode("utf-8", errors="replace").strip()
                        if text:
                            emit_log_line(addr, text)
                    close_client(ready, clients, buffers)
                    emit_event(f"Client disconnected: {addr[0]}:{addr[1]}")
                    continue

                buf = buffers.get(ready, b"") + data
                while True:
                    newline = buf.find(b"\n")
                    if newline < 0:
                        break
                    raw_line = buf[:newline]
                    buf = buf[newline + 1 :]
                    text = raw_line.decode("utf-8", errors="replace").strip("\r")
                    if text:
                        emit_log_line(addr, text)
                buffers[ready] = buf
            maybe_emit_summary()
    except KeyboardInterrupt:
        print("\nStopped.", flush=True)
    finally:
        if output_file is not None:
            output_file.flush()
            output_file.close()
        for client in list(clients.keys()):
            close_client(client, clients, buffers)
        server.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
