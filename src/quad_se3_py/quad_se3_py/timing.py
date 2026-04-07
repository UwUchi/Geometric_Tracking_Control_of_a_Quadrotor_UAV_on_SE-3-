from builtin_interfaces.msg import Time


def stamp_to_seconds(stamp: Time) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def seconds_to_stamp(seconds: float) -> Time:
    total_nanoseconds = max(0, int(round(seconds * 1e9)))
    sec, nanosec = divmod(total_nanoseconds, 1_000_000_000)
    return Time(sec=sec, nanosec=nanosec)


def resolve_trajectory_start_time(
    configured_start_time_sec: float,
    fallback_start_time_sec: float,
) -> float:
    if configured_start_time_sec > 0.0:
        return configured_start_time_sec
    return fallback_start_time_sec


def trajectory_time_from_stamp(
    stamp: Time,
    trajectory_start_time_sec: float,
    reference_time_offset_sec: float = 0.0,
) -> float:
    t_sec = (
        stamp_to_seconds(stamp)
        - trajectory_start_time_sec
        - reference_time_offset_sec
    )
    return max(t_sec, 0.0)
