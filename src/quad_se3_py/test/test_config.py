from quad_se3_py.config import (
    DEFAULT_ANGULAR_VELOCITY_GAIN,
    DEFAULT_ATTITUDE_GAIN,
    DEFAULT_POSITION_GAIN_SCALE,
    DEFAULT_VELOCITY_GAIN_SCALE,
    make_control_gains,
)


def test_make_control_gains_uses_shared_defaults():
    gains = make_control_gains(4.34)

    assert gains.kx == DEFAULT_POSITION_GAIN_SCALE * 4.34
    assert gains.kv == DEFAULT_VELOCITY_GAIN_SCALE * 4.34
    assert gains.kR == DEFAULT_ATTITUDE_GAIN
    assert gains.kOmega == DEFAULT_ANGULAR_VELOCITY_GAIN
