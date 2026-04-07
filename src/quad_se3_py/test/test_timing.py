from builtin_interfaces.msg import Time

from quad_se3_py.timing import (
    resolve_trajectory_start_time,
    seconds_to_stamp,
    stamp_to_seconds,
    trajectory_time_from_stamp,
)


def test_stamp_to_seconds():
    stamp = Time(sec=12, nanosec=500_000_000)
    assert stamp_to_seconds(stamp) == 12.5


def test_seconds_to_stamp_round_trips():
    stamp = seconds_to_stamp(12.5)
    assert stamp_to_seconds(stamp) == 12.5


def test_resolve_trajectory_start_time_prefers_configured_value():
    assert resolve_trajectory_start_time(10.0, 3.0) == 10.0


def test_trajectory_time_from_stamp_clamps_negative_values():
    stamp = Time(sec=5, nanosec=0)
    assert trajectory_time_from_stamp(stamp, 8.0) == 0.0


def test_trajectory_time_from_stamp_is_monotonic():
    start_sec = 10.0
    first = trajectory_time_from_stamp(Time(sec=11, nanosec=0), start_sec)
    second = trajectory_time_from_stamp(
        Time(sec=11, nanosec=500_000_000),
        start_sec,
    )
    assert second > first
