import numpy as np
import pytest

from rocky.commands import BodyVelocity
from rocky.config import load_config
from rocky.gait.generator import GaitGenerator
from rocky.gait.patterns import make_pattern
from rocky.kinematics.body import neutral_feet

CFG = load_config()
DT = 0.01
N = neutral_feet(CFG)
H = CFG.gait.body_height
PERIOD = CFG.gait.period


def make(gait="wave"):
    return GaitGenerator(CFG, make_pattern(gait, CFG.gait.duty_factor[gait]))


def run(gen, cmd, seconds):
    out = []
    for _ in range(round(seconds / DT)):
        out.append(gen.step(DT, cmd))
    return np.array(out)


def xy_dist(feet):
    return np.linalg.norm(feet[..., :2] - N[:, :2], axis=-1)


def test_body_velocity_is_zero():
    assert BodyVelocity().is_zero()
    assert not BodyVelocity(vx=0.01).is_zero()


def test_idle_holds_neutral():
    gen = make()
    for feet in run(gen, BodyVelocity(), 2.0):
        np.testing.assert_array_equal(feet, N)


def test_stance_feet_move_against_command():
    gen = make()
    cmd = BodyVelocity(vx=0.03)
    run(gen, cmd, 3.0)
    before, swing_before = gen.feet, gen.swinging
    after = gen.step(DT, cmd)
    checked = 0
    for i in range(5):
        if not swing_before[i] and not gen.swinging[i]:
            np.testing.assert_allclose(after[i] - before[i], [-0.03 * DT, 0, 0], atol=1e-9)
            checked += 1
    assert checked >= 3


def test_swing_lifts_to_step_height():
    gen = make()
    cmd = BodyVelocity(vx=0.03)
    run(gen, cmd, PERIOD)
    feet = run(gen, cmd, PERIOD)
    for i in range(5):
        assert feet[:, i, 2].max() == pytest.approx(-H + CFG.gait.step_height, abs=1e-3)
        assert feet[:, i, 2].min() == pytest.approx(-H, abs=1e-12)


def _max_stride(gen, cmd):
    v = np.array([cmd.vx, cmd.vy])
    strides = [np.linalg.norm(v + cmd.wz * np.array([-n[1], n[0]])) for n in N]
    return max(strides) * gen.stance_time


def test_clamp_limits_stride():
    gen = make()
    c = gen.clamp(BodyVelocity(vx=10))
    assert c.vx * gen.stance_time == pytest.approx(CFG.gait.max_stride)
    c = gen.clamp(BodyVelocity(vx=10, wz=10))
    assert _max_stride(gen, c) == pytest.approx(CFG.gait.max_stride)
    assert c.vx / c.wz == pytest.approx(1.0)
    small = BodyVelocity(vx=0.01)
    assert gen.clamp(small) == small


def test_feet_stay_within_half_stride():
    gen = make()
    cmd = BodyVelocity(vx=10, vy=-3)
    startup = run(gen, cmd, PERIOD)
    duty = CFG.gait.duty_factor["wave"]
    assert xy_dist(startup).max() <= CFG.gait.max_stride / duty + 1e-3
    steady = run(gen, cmd, 5.0)
    assert xy_dist(steady).max() <= CFG.gait.max_stride / 2 + 1e-3


@pytest.mark.parametrize("gait", ["wave", "star"])
def test_combined_walk_and_turn_feet_bounded(gait):
    gen = make(gait)
    cmd = BodyVelocity(vx=10, vy=4, wz=-10)
    run(gen, cmd, PERIOD)
    steady = run(gen, cmd, 30.0)
    assert xy_dist(steady).max() <= CFG.gait.max_stride / 2 + 1e-3


def test_continuous_turn_feet_bounded():
    gen = make()
    cmd = BodyVelocity(wz=10)
    run(gen, cmd, PERIOD)
    steady = run(gen, cmd, 60.0)
    assert xy_dist(steady).max() <= CFG.gait.max_stride / 2 + 1e-3


def test_stop_mid_step_settles_to_neutral():
    gen = make()
    run(gen, BodyVelocity(vx=0.04), 2.37)
    settled = run(gen, BodyVelocity(), 2 * PERIOD)[-1]
    np.testing.assert_allclose(settled[:, :2], N[:, :2], atol=2e-3)
    np.testing.assert_array_equal(settled[:, 2], N[:, 2])
    for feet in run(gen, BodyVelocity(), 1.0):
        np.testing.assert_array_equal(feet, settled)


@pytest.mark.parametrize("gait", ["wave", "star"])
def test_resume_from_idle_is_continuous(gait):
    gen = make(gait)
    run(gen, BodyVelocity(), 1.0)
    cmd = BodyVelocity(vx=0.04)
    prev = gen.feet
    t_swing = (1 - CFG.gait.duty_factor[gait]) * PERIOD
    bound = DT * (np.pi * CFG.gait.step_height + 1.5 * CFG.gait.max_stride) / t_swing
    bound += 0.04 * DT + 1e-9
    for feet in run(gen, cmd, 2.0):
        assert np.linalg.norm(feet - prev, axis=1).max() <= bound
        prev = feet
