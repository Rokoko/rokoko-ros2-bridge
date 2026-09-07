import pytest

from rkk_hand_bridge.timestamps import StampSource


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


def test_monotonicity_guard_drops_a_backwards_frame():
    source = StampSource(mode="epoch")
    assert source.compute(2_000, None, receive_time_ns=0) == 2_000_000
    assert source.compute(1_000, None, receive_time_ns=0) is None
    # a later, forward frame is accepted again
    assert source.compute(3_000, None, receive_time_ns=0) == 3_000_000


def test_invalid_mode_raises():
    with pytest.raises(ValueError):
        StampSource(mode="bogus")
