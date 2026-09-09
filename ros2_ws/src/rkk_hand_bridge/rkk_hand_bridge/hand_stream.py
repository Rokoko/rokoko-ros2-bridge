"""Background RGMP v2 client: connect, decode, keep the latest frame
per hand. A consumer that falls behind sees the newest pose, not a
queue of stale ones."""

import json
import socket
import struct
import threading

from rkk_hand_bridge import rgmp

# The doubling stops here, or at the starting delay if that is already
# longer — asking for a slower retry must never produce a faster one.
_MAX_RECONNECT_DELAY_S = 10.0

# Without this a host that is up but silent blocks the reader thread in
# the kernel's TCP retries, minutes at a time, with no reconnect.
_CONNECT_TIMEOUT_S = 5.0


class HandStream:
    def __init__(self, host, port, reconnect_delay_s=0.5, connect=None,
                 on_unsupported_hand=None, on_stream_error=None):
        self.host = host
        self.port = port
        self._reconnect_delay_s = reconnect_delay_s
        self._connect = connect or self._default_connect
        self.on_unsupported_hand = on_unsupported_hand
        self.on_stream_error = on_stream_error

        self._cond = threading.Condition()
        self._hands = {}
        self._frames = {}
        self._generation = 0
        self._warned = set()

        self._stop = threading.Event()
        self._sock_lock = threading.Lock()
        self._thread = None
        self._sock = None
        self._delivered = False

    def _default_connect(self):
        sock = socket.create_connection((self.host, self.port), timeout=_CONNECT_TIMEOUT_S)
        sock.settimeout(None)
        return sock

    def start(self):
        if self._thread is not None:
            raise RuntimeError("already started")
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._close_socket()
        with self._cond:
            self._cond.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None

    def hands(self) -> dict:
        with self._cond:
            return dict(self._hands)

    def latest(self) -> dict:
        with self._cond:
            return dict(self._frames)

    def wait(self, seen: int = 0, timeout: float | None = None) -> int | None:
        """The generation once it differs from `seen`, `seen` itself if
        `timeout` passed with no change, or None once the stream is
        stopped."""
        with self._cond:
            self._cond.wait_for(
                lambda: self._generation != seen or self._stop.is_set(), timeout
            )
            if self._stop.is_set():
                return None
            return self._generation

    def _run(self):
        delay = self._reconnect_delay_s
        while not self._stop.is_set():
            try:
                sock = self._connect()
                with self._sock_lock:
                    if self._stop.is_set():
                        sock.close()
                        return
                    self._sock = sock
                self._read_loop(sock)
            except (OSError, EOFError):
                pass
            except (json.JSONDecodeError, struct.error, KeyError, ValueError) as exc:
                self._report_error(f"undecodable frame, reconnecting: {exc!r}")
            finally:
                self._close_socket()
                self._drop_all()
            if self._stop.is_set():
                return
            if self._delivered:
                delay = self._reconnect_delay_s
                self._delivered = False
            self._stop.wait(delay)
            delay = min(delay * 2.0, max(_MAX_RECONNECT_DELAY_S, self._reconnect_delay_s))

    def _report_error(self, message: str):
        if self.on_stream_error is not None:
            self.on_stream_error(message)

    def _close_socket(self):
        with self._sock_lock:
            sock, self._sock = self._sock, None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    def _read_loop(self, reader):
        while not self._stop.is_set():
            msg_prefix, payload = rgmp.read_frame(reader)
            # A solver that accepts and immediately closes must still back
            # off, so the delay resets on data rather than on connect.
            self._delivered = True
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
        reason = rgmp.describe_unsupported(payload)
        if reason is None or self.on_unsupported_hand is None:
            return
        device_id = json.loads(payload).get("device_id")
        with self._cond:
            if device_id in self._warned:
                return
            self._warned.add(device_id)
        self.on_unsupported_hand(device_id, reason)

    def _on_disconnect(self, payload: bytes):
        device_id = rgmp.decode_disconnect(payload).device_id
        with self._cond:
            had_hand = self._hands.pop(device_id, None) is not None
            had_frame = self._frames.pop(device_id, None) is not None
            self._warned.discard(device_id)
            if had_hand or had_frame:
                self._generation += 1
                self._cond.notify_all()

    def _drop_all(self):
        with self._cond:
            self._warned.clear()
            if self._hands or self._frames:
                self._hands.clear()
                self._frames.clear()
                self._generation += 1
                self._cond.notify_all()
