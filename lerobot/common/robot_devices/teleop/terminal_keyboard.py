"""Shared, X11-independent keyboard input for an interactive POSIX terminal."""

from __future__ import annotations

import logging
import os
import select
import sys
import termios
import threading
import tty
from collections.abc import Callable

KeyCallback = Callable[[str], None]


class _TerminalKeyboardService:
    """Read one terminal in a single thread and fan keys out to subscribers."""

    def __init__(self) -> None:
        self._callbacks: set[KeyCallback] = set()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._fd: int | None = None
        self._old_settings = None

    def subscribe(self, callback: KeyCallback) -> None:
        with self._lock:
            self._callbacks.add(callback)
            if self._thread is None:
                self._start_locked()

    def unsubscribe(self, callback: KeyCallback) -> None:
        with self._lock:
            self._callbacks.discard(callback)
            should_stop = not self._callbacks
        if should_stop:
            self.stop()

    def _start_locked(self) -> None:
        if os.name != "posix" or not sys.stdin.isatty():
            raise RuntimeError("Keyboard control requires an interactive POSIX terminal (stdin must be a TTY)")
        self._fd = sys.stdin.fileno()
        self._old_settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="terminal-keyboard", daemon=True)
        self._thread.start()
        logging.info("Terminal keyboard ready (Wayland/X11 independent)")

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        with self._lock:
            self._thread = None
            self._callbacks.clear()
            if self._fd is not None and self._old_settings is not None:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)
            self._fd = None
            self._old_settings = None

    def _emit(self, key: str) -> None:
        with self._lock:
            callbacks = tuple(self._callbacks)
        for callback in callbacks:
            try:
                callback(key)
            except Exception:
                logging.exception("Terminal key callback failed for %r", key)

    def _run(self) -> None:
        assert self._fd is not None
        buffer = b""
        try:
            while not self._stop.is_set():
                readable, _, _ = select.select([self._fd], [], [], 0.05)
                if readable:
                    buffer += os.read(self._fd, 32)
                buffer = self._parse(buffer, flush_escape=not readable)
        finally:
            # Restoration is normally done by the last subscriber. Also restore
            # here after an unexpected reader failure.
            if self._fd is not None and self._old_settings is not None:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)

    def _parse(self, buffer: bytes, *, flush_escape: bool) -> bytes:
        arrows = {b"\x1b[C": "right", b"\x1b[D": "left", b"\x1b[A": "up", b"\x1b[B": "down"}
        while buffer:
            matched = next(((sequence, name) for sequence, name in arrows.items() if buffer.startswith(sequence)), None)
            if matched:
                sequence, name = matched
                self._emit(name)
                buffer = buffer[len(sequence) :]
                continue
            if buffer.startswith(b"\x1b"):
                if len(buffer) < 3 and not flush_escape:
                    break
                self._emit("esc")
                buffer = buffer[1:]
                continue
            byte, buffer = buffer[:1], buffer[1:]
            try:
                char = byte.decode("utf-8").lower()
            except UnicodeDecodeError:
                continue
            if char.isprintable():
                self._emit(char)
        return buffer


_SERVICE = _TerminalKeyboardService()


class TerminalKeyboardListener:
    """Small subscription handle around the process-wide terminal reader."""

    def __init__(self, on_key: KeyCallback):
        self._on_key = on_key
        self._started = False

    def start(self) -> None:
        if not self._started:
            _SERVICE.subscribe(self._on_key)
            self._started = True

    def stop(self) -> None:
        if self._started:
            _SERVICE.unsubscribe(self._on_key)
            self._started = False
