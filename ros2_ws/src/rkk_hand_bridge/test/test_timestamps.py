import pytest

from rkk_hand_bridge.timestamps import OFFSET_WINDOW_NS, StampSource


def test_epoch_mode_converts_microseconds_to_nanoseconds():
    source = StampSource(mode="epoch")
    assert source.compute(1_000_000, None, receive_time_ns=0) == 1_000_000_000


def test_auto_picks_epoch_for_unix_epoch():
    source = StampSource()
    assert source.compute(5, "unix_epoch", receive_time_ns=999) == 5_000


def test_auto_picks_offset_for_device_boot():
    source = StampSource()
    stamp = source.compute(1_000, "device_boot", receive_time_ns=10_000_000)
    assert stamp == 10_000_000  # offset makes first sample match receive time


def test_receive_mode_ignores_the_source_timestamp():
    source = StampSource(mode="receive")
    assert source.compute(999_999, "unix_epoch", receive_time_ns=42) == 42


def test_offset_mode_tracks_the_running_minimum():
    source = StampSource(mode="offset")
    first = source.compute(1_000, None, receive_time_ns=10_000_000)  # offset = 9_000_000
    second = source.compute(2_000, None, receive_time_ns=10_500_000)  # offset -> 8_500_000
    assert first == 10_000_000
    # smaller offset now applies retroactively to the running estimate
    assert second == 2_000 * 1000 + 8_500_000


def test_offset_estimate_forgets_a_stale_minimum():
    source = StampSource(mode="offset")
    source.compute(1_000, None, receive_time_ns=10_000_000)  # offset 9_000_000
    # Far outside the window, and arriving later relative to its own
    # device time, so the old minimum would pull this stamp backwards.
    late_us = 1_000 + OFFSET_WINDOW_NS // 1000
    receive_ns = late_us * 1000 + 50_000_000
    assert source.compute(late_us, None, receive_time_ns=receive_ns) == receive_ns


def test_offset_estimate_keeps_the_minimum_inside_the_window():
    source = StampSource(mode="offset")
    source.compute(1_000, None, receive_time_ns=10_000_000)  # offset 9_000_000
    stamp = source.compute(2_000, None, receive_time_ns=12_000_000)  # candidate 10_000_000
    assert stamp == 2_000 * 1000 + 9_000_000


def test_monotonicity_guard_drops_a_backwards_frame():
    source = StampSource(mode="epoch")
    assert source.compute(2_000, None, receive_time_ns=0) == 2_000_000
    assert source.compute(1_000, None, receive_time_ns=0) is None
    # a later, forward frame is accepted again
    assert source.compute(3_000, None, receive_time_ns=0) == 3_000_000


def test_invalid_mode_raises():
    with pytest.raises(ValueError):
        StampSource(mode="bogus")


def test_a_clock_reset_restarts_instead_of_stalling():
    source = StampSource(mode="epoch")
    source.compute(10_000_000, None, receive_time_ns=0)  # 10s
    restarted_us = 5_000  # device counter back near zero
    assert source.compute(restarted_us, None, receive_time_ns=0) == restarted_us * 1000
    assert source.clock_resets == 1
    assert source.dropped == 0


def test_small_backwards_steps_are_still_dropped():
    source = StampSource(mode="epoch")
    source.compute(2_000_000, None, receive_time_ns=0)
    assert source.compute(1_999_000, None, receive_time_ns=0) is None
    assert source.dropped == 1
    assert source.clock_resets == 0


def test_a_clock_reset_clears_the_offset_estimate():
    source = StampSource(mode="offset")
    source.compute(10_000_000, None, receive_time_ns=10_000_000_000)
    source.compute(1_000, None, receive_time_ns=20_000_000_000)  # reset
    stamp = source.compute(2_000, None, receive_time_ns=20_000_000_000)
    assert stamp is not None
    assert source.clock_resets == 1
