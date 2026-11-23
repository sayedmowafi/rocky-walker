import mujoco
import numpy as np
import pytest

from rocky.config import load_config
from rocky.motion.controller import MotionController
from rocky.sim.model import FOOT_RADIUS, joint_names, load_model

pytestmark = pytest.mark.sim

CFG = load_config()


def test_model_compiles():
    model = load_model(CFG)
    assert model.nu == 15
    assert model.njnt == 16
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)]
    assert names == joint_names()
    assert joint_names()[:4] == ["leg0_coxa", "leg0_femur", "leg0_tibia", "leg1_coxa"]


def test_actuator_limits_match_config():
    model = load_model(CFG)
    tau = CFG.servo.stall_torque
    np.testing.assert_allclose(model.actuator_forcerange, np.tile([-tau, tau], (15, 1)))
    np.testing.assert_allclose(model.actuator_ctrlrange, np.tile(CFG.limits.as_array(), (5, 1)))


def test_stands_when_holding_neutral():
    model = load_model(CFG)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("stand").id)
    data.ctrl[:] = MotionController(CFG).angles.reshape(-1)
    for _ in range(round(2.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
    expected_z = CFG.gait.body_height + FOOT_RADIUS
    assert data.qpos[2] == pytest.approx(expected_z, rel=0.10)
    up = data.xmat[model.body("torso").id].reshape(3, 3)[:, 2]
    assert np.degrees(np.arccos(np.clip(up[2], -1, 1))) < 5
