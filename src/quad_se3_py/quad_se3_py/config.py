from dataclasses import dataclass


DEFAULT_POSITION_GAIN_SCALE = 16.0
DEFAULT_VELOCITY_GAIN_SCALE = 5.6
DEFAULT_ATTITUDE_GAIN = 8.81
DEFAULT_ANGULAR_VELOCITY_GAIN = 2.54


@dataclass(frozen=True)
class ControlGains:
    kx: float
    kv: float
    kR: float
    kOmega: float


def make_control_gains(
    mass: float,
    position_gain_scale: float = DEFAULT_POSITION_GAIN_SCALE,
    velocity_gain_scale: float = DEFAULT_VELOCITY_GAIN_SCALE,
    attitude_gain: float = DEFAULT_ATTITUDE_GAIN,
    angular_velocity_gain: float = DEFAULT_ANGULAR_VELOCITY_GAIN,
) -> ControlGains:
    return ControlGains(
        kx=position_gain_scale * mass,
        kv=velocity_gain_scale * mass,
        kR=attitude_gain,
        kOmega=angular_velocity_gain,
    )
