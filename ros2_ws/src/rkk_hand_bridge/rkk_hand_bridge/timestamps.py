"""header.stamp computation: auto/epoch/offset/receive strategies, and
the monotonicity guard that drops frames whose stamp would go backwards.

All times are nanoseconds; ROS Time (sec, nanosec) conversion is the
caller's job, since that's an rclpy concern, not this module's.
"""

UNIX_EPOCH = "unix_epoch"
MODES = ("auto", "epoch", "offset", "receive")


class StampSource:
    """Per-hand state: the running offset estimate and the last stamp
    published, so frames going backwards in time can be dropped."""

    def __init__(self, mode: str = "auto"):
        if mode not in MODES:
            raise ValueError(f"unknown stamp_source mode: {mode!r}")
        self._mode = mode
        self._offset_ns = None
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
        candidate = receive_time_ns - timestamp_us * 1000
        if self._offset_ns is None or candidate < self._offset_ns:
            self._offset_ns = candidate
        return timestamp_us * 1000 + self._offset_ns
