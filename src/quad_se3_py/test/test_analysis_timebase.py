from builtin_interfaces.msg import Time

import pytest

from quad_se3_py.analysis_timebase import (
    choose_recorded_epoch_start_time,
    reconstruct_reference_from_stamps,
    resolve_reference_source,
    resolve_reference_time_offset_sec,
    resolve_time_function_start_time,
)


def test_choose_recorded_epoch_start_time_prefers_epoch_topic():
    start_time_sec, source = choose_recorded_epoch_start_time(10.0, 12.0)
    assert start_time_sec == 10.0
    assert source == 'trajectory_epoch_topic'


def test_resolve_reference_source_prefers_time_function_when_epoch_exists():
    reference_source = resolve_reference_source(
        'auto',
        {},
        epoch_time_sec=3.0,
    )
    assert reference_source == 'time_function'


def test_resolve_time_function_start_time_priority():
    experiment_metadata = {'trajectory_start_time_sec': 30.0}
    start_time_sec, source = resolve_time_function_start_time(
        configured_start_time_sec=10.0,
        epoch_time_sec=20.0,
        experiment_metadata=experiment_metadata,
        trajectory_fallback_time_sec=40.0,
    )
    assert start_time_sec == 10.0
    assert source == 'cli_argument'


def test_resolve_time_function_start_time_uses_epoch_before_metadata():
    experiment_metadata = {'trajectory_start_time_sec': 30.0}
    start_time_sec, source = resolve_time_function_start_time(
        configured_start_time_sec=None,
        epoch_time_sec=20.0,
        experiment_metadata=experiment_metadata,
        trajectory_fallback_time_sec=40.0,
    )
    assert start_time_sec == 20.0
    assert source == 'trajectory_epoch_topic'


def test_resolve_time_function_start_time_falls_back_to_trajectory_stamp():
    start_time_sec, source = resolve_time_function_start_time(
        configured_start_time_sec=None,
        epoch_time_sec=None,
        experiment_metadata={},
        trajectory_fallback_time_sec=40.0,
    )
    assert start_time_sec == 40.0
    assert source == 'first_trajectory_stamp'


def test_resolve_reference_time_offset_sec_prefers_cli_value():
    assert (
        resolve_reference_time_offset_sec(
            0.25,
            {'reference_time_offset_sec': 0.5},
        )
        == 0.25
    )


def test_reconstruct_reference_from_stamps_uses_reference_time_offset():
    stamps = [Time(sec=11, nanosec=0)]
    without_offset = reconstruct_reference_from_stamps(
        stamps,
        'paper_case_1_helix',
        trajectory_start_time_sec=10.0,
        reference_time_offset_sec=0.0,
    )
    with_offset = reconstruct_reference_from_stamps(
        stamps,
        'paper_case_1_helix',
        trajectory_start_time_sec=10.0,
        reference_time_offset_sec=0.25,
    )

    assert without_offset['trajectory_time'][0] == pytest.approx(1.0)
    assert with_offset['trajectory_time'][0] == pytest.approx(0.75)
    assert with_offset['position'][0, 0] != pytest.approx(
        without_offset['position'][0, 0]
    )
