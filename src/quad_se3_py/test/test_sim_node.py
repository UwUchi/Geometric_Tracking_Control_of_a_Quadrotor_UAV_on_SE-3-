import pytest

from quad_se3_py.sim_node import SimTimebase
from quad_se3_py.timing import stamp_to_seconds


def test_sim_timebase_initial_stamp_matches_epoch():
    timebase = SimTimebase(epoch_time_sec=25.0, dt=0.002)
    assert stamp_to_seconds(timebase.stamp_for_step(0)) == 25.0


def test_sim_timebase_stamps_are_monotonic_and_uniform():
    timebase = SimTimebase(epoch_time_sec=7.5, dt=0.002)
    first = stamp_to_seconds(timebase.stamp_for_step(3))
    second = stamp_to_seconds(timebase.stamp_for_step(4))
    third = stamp_to_seconds(timebase.stamp_for_step(5))

    assert first < second < third
    assert second - first == pytest.approx(0.002)
    assert third - second == pytest.approx(0.002)


def test_state_and_trajectory_can_share_same_stamp():
    timebase = SimTimebase(epoch_time_sec=100.0, dt=0.002)
    state_stamp = timebase.stamp_for_step(11)
    trajectory_stamp = timebase.stamp_for_step(11)

    assert state_stamp.sec == trajectory_stamp.sec
    assert state_stamp.nanosec == trajectory_stamp.nanosec
