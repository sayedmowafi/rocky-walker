import copy
import math
import tomllib

import numpy as np
import pytest

from rocky.commands import BodyVelocity
from rocky.config import DEFAULT_CONFIG_PATH, ConfigError, load_config, parse_config
from rocky.kinematics.leg import leg_ik
from rocky.motion.controller import MotionController

CFG = load_config()
DT = 0.01


def ticks(ctl, target, seconds):
    return [ctl.tick(DT, target) for _ in range(round(seconds / DT))]


def test_starts_in_neutral_stance():
    ctl = MotionController(CFG)
    reach = CFG.gait.stance_radius - CFG.body.mount_radius
    leg = leg_ik(np.array([reach, 0.0, -CFG.gait.body_height]), CFG.leg)
    assert leg[0] == 0.0
    expected = np.tile(leg, (5, 1))
    np.testing.assert_allclose(ctl.angles, expected, atol=1e-9)


def test_idle_does_not_move():
    ctl = MotionController(CFG)
    start = ctl.angles
    for a in ticks(ctl, BodyVelocity(), 1.0):
        np.testing.assert_array_equal(a, start)


def test_command_ramps_at_accel_limit():
    ctl = MotionController(CFG)
    ctl.tick(DT, BodyVelocity(vx=0.04))
    assert ctl.command.vx == pytest.approx(CFG.motion.max_linear_accel * DT)
    n = math.ceil(0.04 / (CFG.motion.max_linear_accel * DT))
    for _ in range(n - 1):
        ctl.tick(DT, BodyVelocity(vx=0.04))
    assert ctl.command.vx == pytest.approx(0.04)

    ctl = MotionController(CFG)
    ctl.tick(DT, BodyVelocity(wz=1.0))
    assert ctl.command.wz == pytest.approx(CFG.motion.max_angular_accel * DT)


def test_reversal_respects_joint_speed():
    ctl = MotionController(CFG)
    prev = ctl.angles
    limit = CFG.servo.max_speed * DT + 1e-12
    for target in (BodyVelocity(vx=0.05), BodyVelocity(vx=-0.05)):
        for a in ticks(ctl, target, 3.0):
            assert np.abs(a - prev).max() <= limit
            prev = a


@pytest.mark.parametrize("gait", ["wave", "star"])
def test_extreme_command_stays_reachable(gait):
    ctl = MotionController(CFG, gait)
    lim = CFG.limits.as_array()
    for a in ticks(ctl, BodyVelocity(vx=5, vy=-5, wz=5), 10.0):
        assert (a >= lim[:, 0] - 1e-12).all() and (a <= lim[:, 1] + 1e-12).all()
    assert ctl.stats.unreachable == 0
    assert ctl.stats.clamped == 0
    assert ctl.stats.ticks == 1000


def test_unreachable_stance_rejected():
    with open(DEFAULT_CONFIG_PATH, "rb") as f:
        data = copy.deepcopy(tomllib.load(f))
    data["gait"]["stance_radius_mm"] = 400
    with pytest.raises(ConfigError, match="stance_radius_mm"):
        MotionController(parse_config(data))


def _v_max(gait="wave"):
    return CFG.gait.max_stride / (CFG.gait.duty_factor[gait] * CFG.gait.period)


def test_release_after_overdrive_stops_quickly():
    ctl = MotionController(CFG)
    ticks(ctl, BodyVelocity(vx=5, wz=5), 10.0)
    ctl.tick(DT, BodyVelocity())
    assert ctl.command.vx <= _v_max() + 1e-9
    after = ticks(ctl, BodyVelocity(), CFG.gait.period + 0.5)
    np.testing.assert_array_equal(after[-1], after[-20])


def test_reversal_after_overdrive_turns_around_promptly():
    ctl = MotionController(CFG)
    ticks(ctl, BodyVelocity(vx=5), 5.0)
    n = math.ceil(2 * _v_max() / (CFG.motion.max_linear_accel * DT)) + 1
    ticks(ctl, BodyVelocity(vx=-5), n * DT)
    assert ctl.command.vx < 0


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_non_finite_command_is_rejected(bad):
    ctl = MotionController(CFG)
    ticks(ctl, BodyVelocity(vx=0.03), 1.0)
    ctl.tick(DT, BodyVelocity(vx=bad))
    assert ctl.stats.rejected == 1
    assert np.isfinite(ctl.angles).all()
    assert math.isfinite(ctl.command.vx)
    for a in ticks(ctl, BodyVelocity(), 3.0):
        assert np.isfinite(a).all()


@pytest.mark.parametrize("gait", ["wave", "star"])
def test_normal_walking_never_needs_guards(gait):
    ctl = MotionController(CFG, gait)
    for target in (BodyVelocity(vx=_v_max(gait)), BodyVelocity(vx=-_v_max(gait)), BodyVelocity()):
        ticks(ctl, target, 3.0)
    ticks(ctl, BodyVelocity(vx=0.6 * _v_max(gait), wz=0.2), 3.0)
    assert ctl.stats.speed_limited == 0
    assert ctl.stats.clamped == 0
    assert ctl.stats.unreachable == 0


@pytest.mark.parametrize(
    ("path", "value"),
    [
        ("gait.max_stride_mm", 150),
        ("gait.step_height_mm", 200),
        ("limits.coxa_deg", [-20, 20]),
    ],
)
def test_walking_envelope_validated_at_startup(path, value):
    with open(DEFAULT_CONFIG_PATH, "rb") as f:
        data = copy.deepcopy(tomllib.load(f))
    section, key = path.split(".")
    data[section][key] = value
    with pytest.raises(ConfigError, match="max_stride_mm"):
        MotionController(parse_config(data))
