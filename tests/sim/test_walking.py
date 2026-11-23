import math

import mujoco
import numpy as np
import pytest

from rocky.backends.mujoco_backend import MujocoBackend
from rocky.commands import BodyVelocity
from rocky.config import load_config
from rocky.sim.model import load_model
from rocky.sim.runner import simulate

pytestmark = pytest.mark.sim

CFG = load_config()
DURATION = 10.0


def limits(gait="wave"):
    t_stance = CFG.gait.duty_factor[gait] * CFG.gait.period
    v_max = CFG.gait.max_stride / t_stance
    w_max = CFG.gait.max_stride / (t_stance * CFG.gait.stance_radius)
    return v_max, w_max


@pytest.mark.parametrize("heading_deg", [0, 36, 90, 197])
def test_walks_any_direction(heading_deg):
    v = 0.8 * limits()[0]
    h = math.radians(heading_deg)
    cmd = BodyVelocity(v * math.cos(h), v * math.sin(h), 0.0)
    res = simulate(CFG, lambda t: cmd, DURATION)
    d = res.torso_pos[-1, :2] - res.torso_pos[0, :2]
    along = d @ np.array([math.cos(h), math.sin(h)])
    perp = abs(d @ np.array([-math.sin(h), math.cos(h)]))
    assert along >= 0.7 * v * DURATION
    assert perp <= 0.25 * along
    assert abs(math.degrees(res.torso_yaw[-1] - res.torso_yaw[0])) < 10
    assert math.degrees(res.torso_tilt.max()) < 15
    assert res.torso_pos[:, 2].min() > 0.5 * CFG.gait.body_height


def test_turns_in_place():
    w = 0.8 * limits()[1]
    res = simulate(CFG, lambda t: BodyVelocity(wz=w), DURATION)
    assert res.torso_yaw[-1] - res.torso_yaw[0] >= 0.7 * w * DURATION
    assert np.linalg.norm(res.torso_pos[-1, :2] - res.torso_pos[0, :2]) < 0.05
    assert math.degrees(res.torso_tilt.max()) < 15


@pytest.mark.parametrize("gait", ["wave", "star"])
def test_gait_meets_torque_rule(gait):
    v = 0.8 * limits(gait)[0]
    res = simulate(CFG, lambda t: BodyVelocity(vx=v), DURATION, gait=gait)
    assert res.sustained_torque_peak().max() <= CFG.servo.torque_rule


def test_mujoco_backend_writes_ctrl():
    model = load_model(CFG)
    data = mujoco.MjData(model)
    a = np.arange(15, dtype=float).reshape(5, 3) / 100
    MujocoBackend(model, data).write(a)
    np.testing.assert_array_equal(data.ctrl, a.reshape(-1))
