"""header.stamp computation: auto/epoch/offset/receive strategies, and
the monotonicity guard that drops frames whose stamp would go backwards.

All times are nanoseconds; ROS Time (sec, nanosec) conversion is the
caller's job, since that's an rclpy concern, not this module's.
"""

from collections import deque

UNIX_EPOCH = "unix_epoch"
MODES = ("auto", "epoch", "offset", "receive")

OFFSET_WINDOW_NS = 30 * 1_000_000_000


class _WindowedMinimum:
    """Smallest value pushed within the last `window_ns`, in O(1) amortised."""

    def __init__(self, window_ns: int):
        self._window_ns = window_ns
        self._rising = deque()  # (time_ns, value), values strictly increasing

    def push(self, value: int, now_ns: int) -> int:
        while self._rising and self._rising[-1][1] >= value:
            self._rising.pop()
        self._rising.append((now_ns, value))
        while self._rising[0][0] <= now_ns - self._window_ns:
            self._rising.popleft()
        return self._rising[0][1]


class StampSource:
    """Per-hand stamp computation: the offset estimate and the last stamp
    published, so frames going backwards in time can be dropped."""

    def __init__(self, mode: str = "auto"):
        if mode not in MODES:
            raise ValueError(f"unknown stamp_source mode: {mode!r}")
        self._mode = mode
        self._offset = _WindowedMinimum(OFFSET_WINDOW_NS)
        self._last_stamp_ns = None

    def compute(self, timestamp_us: int, timestamp_epoch: str | None, receive_time_ns: int) -> int | None:
        """Returns the stamp to publish, or None if this frame should
        be dropped for going backwards relative to the last one."""
        mode = self._mode if self._mode != "auto" else self._auto_mode(timestamp_epoch)
        if mode == "epoch":
            stamp_ns = timestamp_us * 1000
        elif mode == "offset":
            stamp_ns = self._offset_estimate(timestamp_us, receive_time_ns)
        else:
            stamp_ns = receive_time_ns

        if self._last_stamp_ns is not None and stamp_ns < self._last_stamp_ns:
            return None
        self._last_stamp_ns = stamp_ns
        return stamp_ns

    def _auto_mode(self, timestamp_epoch: str | None) -> str:
        return "epoch" if timestamp_epoch == UNIX_EPOCH else "offset"

    def _offset_estimate(self, timestamp_us: int, receive_time_ns: int) -> int:
        # Every candidate is the true offset plus that frame's transport
        # delay, so the smallest is the closest. Windowed, or drift only
        # ever pulls the estimate one way.
        device_ns = timestamp_us * 1000
        offset_ns = self._offset.push(receive_time_ns - device_ns, receive_time_ns)
        return device_ns + offset_ns
