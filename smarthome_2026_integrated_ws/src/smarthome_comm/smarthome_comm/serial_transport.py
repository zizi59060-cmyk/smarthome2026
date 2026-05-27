from __future__ import annotations

import time


class SerialTransport:
    def __init__(
        self,
        device: str,
        baudrate: int,
        timeout: float = 0.0,
        fake: bool = False,
        reconnect_interval: float = 1.0,
    ) -> None:
        self.device = device
        self.baudrate = int(baudrate)
        self.timeout = float(timeout)
        self.fake = bool(fake)
        self.reconnect_interval = max(0.1, float(reconnect_interval))
        self._serial = None
        self._next_reconnect_time = 0.0
        self._last_error = ""

    def open(self) -> bool:
        if self.fake:
            return True
        return self.try_reconnect(force=True)

    def close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            finally:
                self._serial = None

    def try_reconnect(self, force: bool = False) -> bool:
        if self.fake:
            return True
        if self.is_open:
            return True
        if not self.device:
            self._last_error = "serial_device is empty"
            return False

        now = time.monotonic()
        if not force and now < self._next_reconnect_time:
            return False

        try:
            import serial  # lazy import, so fake mode does not require pyserial

            self._serial = serial.Serial(self.device, self.baudrate, timeout=self.timeout)
            self._last_error = ""
            self._next_reconnect_time = 0.0
            return True
        except Exception as exc:
            self._serial = None
            self._last_error = str(exc)
            self._next_reconnect_time = now + self.reconnect_interval
            return False

    def mark_disconnected(self, error: Exception | str) -> None:
        self._last_error = str(error)
        self.close()
        self._next_reconnect_time = time.monotonic() + self.reconnect_interval

    def read_available(self) -> bytes:
        if self.fake:
            return b""
        if not self.is_open and not self.try_reconnect():
            return b""

        try:
            n = getattr(self._serial, "in_waiting", 0)
            if n <= 0:
                return b""
            return self._serial.read(n)
        except Exception as exc:
            self.mark_disconnected(exc)
            return b""

    def write(self, data: bytes) -> int:
        if self.fake:
            return len(data)
        if not self.is_open and not self.try_reconnect():
            return 0

        try:
            return int(self._serial.write(data))
        except Exception as exc:
            self.mark_disconnected(exc)
            return 0

    @property
    def is_open(self) -> bool:
        if self.fake:
            return True
        if self._serial is None:
            return False
        return bool(getattr(self._serial, "is_open", True))

    @property
    def last_error(self) -> str:
        return self._last_error
