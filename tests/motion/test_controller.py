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
    assert ctl.command.wz == pytest.approx(math.radians(90) * DT)


def test_reversal_respects_joint_speed():
    ctl = MotionController(CFG)
    prev = ctl.angles
    limit = CFG.servo.max_speed * DT + 1e-12
    for target in (BodyVelocity(vx=0.05), BodyVelocity(vx=-0.05)):
        for a in ticks(ctl, target, 3.0):
            assert np.abs(a - prev).max() <= limit
            prev = a


def test_extreme_command_stays_reachable():
    ctl = MotionController(CFG)
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
