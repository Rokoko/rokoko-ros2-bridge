"""Background RGMP v2 client: connect, decode, keep the latest frame
per hand. A consumer that falls behind sees the newest pose, not a
queue of stale ones."""

import json
import socket
import threading

from rkk_hand_bridge import rgmp_client as rgmp


class HandStream:
    def __init__(self, host, port, reconnect_backoff_s=0.5, connect=None, on_unusable_hand=None):
        self.host = host
        self.port = port
        self._reconnect_backoff_s = reconnect_backoff_s
        self._connect = connect or self._default_connect
        self._on_unusable_hand = on_unusable_hand

        self._cond = threading.Condition()
        self._hands = {}
        self._frames = {}
        self._generation = 0
        self._connected = False
        self._warned = set()

        self._stop = threading.Event()
        self._thread = None
        self._sock = None

    def _default_connect(self):
        return socket.create_connection((self.host, self.port))

    def start(self):
        if self._thread is not None:
            raise RuntimeError("already started")
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        with self._cond:
            self._cond.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None

    @property
    def connected(self) -> bool:
        with self._cond:
            return self._connected

    def hands(self) -> dict:
        with self._cond:
            return dict(self._hands)

    def latest(self) -> dict:
        with self._cond:
            return dict(self._frames)

    def wait(self, seen: int = 0, timeout: float | None = None) -> int:
        with self._cond:
            changed = self._cond.wait_for(
                lambda: self._generation != seen or self._stop.is_set(), timeout
            )
            if not changed:
                raise TimeoutError(f"no update within {timeout}s")
            return self._generation

    def _run(self):
        delay = self._reconnect_backoff_s
        while not self._stop.is_set():
            try:
                self._sock = self._connect()
                self._set_connected(True)
                delay = self._reconnect_backoff_s
                self._read_loop(self._sock)
            except (OSError, EOFError):
                pass
            finally:
                self._close_socket()
                self._drop_all()
            if self._stop.is_set():
                return
            self._stop.wait(delay)
            delay = min(delay * 2.0, 10.0)

    def _close_socket(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def _read_loop(self, reader):
        while not self._stop.is_set():
            msg_prefix, payload = rgmp.read_frame(reader)
            if msg_prefix == rgmp.MSG_DEFINITION:
                self._on_definition(payload)
            elif msg_prefix == rgmp.MSG_DATA:
                self._on_data(payload)
            elif msg_prefix == rgmp.MSG_DISCONNECT:
                self._on_disconnect(payload)

    def _on_definition(self, payload: bytes):
        definition = rgmp.decode_definition(payload)
        if definition is None:
            self._warn_once(payload)
            return
        with self._cond:
            self._hands[definition.device_id] = definition
            self._frames.pop(definition.device_id, None)
            self._generation += 1
            self._cond.notify_all()

    def _on_data(self, payload: bytes):
        device_id, group_id, _ = rgmp.decode_data_header(payload)
        with self._cond:
            definition = self._hands.get(device_id)
        if definition is None or group_id != definition.group_id:
            return
        frame = rgmp.decode_data(payload)
        with self._cond:
            self._frames[device_id] = frame
            self._generation += 1
            self._cond.notify_all()

    def _warn_once(self, payload: bytes):
        reason = rgmp.describe_unusable(payload)
        if reason is None or self._on_unusable_hand is None:
            return
        device_id = json.loads(payload).get("device_id")
        with self._cond:
            if device_id in self._warned:
                return
            self._warned.add(device_id)
        self._on_unusable_hand(device_id, reason)

    def _on_disconnect(self, payload: bytes):
        device_id = rgmp.decode_disconnect(payload).device_id
        with self._cond:
            had_hand = self._hands.pop(device_id, None) is not None
            had_frame = self._frames.pop(device_id, None) is not None
            self._warned.discard(device_id)
            if had_hand or had_frame:
                self._generation += 1
                self._cond.notify_all()

    def _set_connected(self, value: bool):
        with self._cond:
            self._connected = value

    def _drop_all(self):
        with self._cond:
            self._connected = False
            self._warned.clear()
            if self._hands or self._frames:
                self._hands.clear()
                self._frames.clear()
                self._generation += 1
                self._cond.notify_all()
