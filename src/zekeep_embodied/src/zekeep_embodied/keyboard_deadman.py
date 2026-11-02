"""Global X11 m-key hold, sampled independently of policy inference."""

import os
import math
import threading
import time


class KeyboardDeadman:
    def __init__(self, timeout_s=0.25):
        self.timeout_s = float(timeout_s)
        if not math.isfinite(self.timeout_s) or self.timeout_s <= 0:
            raise ValueError("Keyboard timeout must be positive and finite")
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._pressed = self._released = False
        self._updated = 0.0
        self.error = None

    def __enter__(self):
        if not os.environ.get("DISPLAY") or os.environ.get("XDG_SESSION_TYPE") == "wayland":
            raise RuntimeError("Global m control requires a local X11 desktop (DISPLAY)")
        from Xlib import XK, display

        connection = display.Display()
        try:
            keycode = connection.keysym_to_keycode(XK.string_to_keysym("m"))
            if not keycode:
                raise RuntimeError("No m key is mapped on the X11 desktop")
            self._sample(connection, keycode)
            self._thread = threading.Thread(
                target=self._run, args=(connection, keycode), daemon=True,
            )
            self._thread.start()
        except BaseException:
            connection.close()
            raise
        return self

    def _sample(self, connection, keycode):
        sampled = time.monotonic()
        keys = connection.query_keymap()
        pressed = bool(keys[keycode // 8] & (1 << (keycode % 8)))
        with self._lock:
            self._pressed, self._updated = pressed, sampled
            if not pressed:
                self._released = True

    def _run(self, connection, keycode):
        try:
            while not self._stop.wait(0.02):
                self._sample(connection, keycode)
        except Exception as exc:
            with self._lock:
                self.error = str(exc)
                self._pressed = self._released = False
        finally:
            connection.close()

    def _held(self):
        return (self._pressed and self.error is None and not self._stop.is_set()
                and time.monotonic() - self._updated <= self.timeout_s)

    @property
    def held(self):
        with self._lock:
            return self._held()

    @property
    def ready(self):
        with self._lock:
            return self._released and self._held()

    def disarm(self):
        with self._lock:
            self._released = False

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def __exit__(self, *_):
        self.close()
