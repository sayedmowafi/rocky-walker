import copy
import tomllib
from math import radians

import pytest

from rocky.config import DEFAULT_CONFIG_PATH, ConfigError, load_config, parse_config


@pytest.fixture
def raw():
    with open(DEFAULT_CONFIG_PATH, "rb") as f:
        return tomllib.load(f)


def test_loads_default_config_in_si():
    cfg = load_config()
    assert cfg.leg.femur == pytest.approx(0.070)
    assert cfg.limits.tibia == pytest.approx((radians(-135), radians(-20)))
    assert cfg.servo.torque_rule == pytest.approx(0.490, abs=1e-3)
    assert cfg.servo.max_speed == pytest.approx(radians(400))
    assert cfg.gait.duty_factor["star"] == 0.6
    assert cfg.motion.loop_hz == 100


def _set(d, path, value):
    *head, last = path.split(".")
    for k in head:
        d = d[k]
    if value is _DELETE:
        del d[last]
    else:
        d[last] = value


_DELETE = object()


@pytest.mark.parametrize(
    ("path", "value", "key"),
    [
        ("leg.femur_mm", -5, "femur_mm"),
        ("limits.coxa_deg", [45, -45], "coxa_deg"),
        ("gait.duty_factor.wave", 1.2, "wave"),
        ("body.n_legs", 6, "n_legs"),
        ("motion.loop_hz", 30, "loop_hz"),
        ("servo", _DELETE, "servo"),
        ("gait.max_stride_mm", float("nan"), "max_stride_mm"),
        ("gait.period_s", float("inf"), "period_s"),
        ("limits.coxa_deg", [float("nan"), 45], "coxa_deg"),
    ],
)
def test_rejects_bad_values(raw, path, value, key):
    data = copy.deepcopy(raw)
    _set(data, path, value)
    with pytest.raises(ConfigError, match=key):
        parse_config(data)
