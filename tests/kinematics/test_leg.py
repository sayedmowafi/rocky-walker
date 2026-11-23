import itertools

import numpy as np
import pytest

from rocky.config import LegGeometry, load_config
from rocky.kinematics.leg import UnreachableError, leg_fk, leg_ik

GEOM = LegGeometry(coxa=0.04, femur=0.07, tibia=0.10)


def test_fk_straight_leg():
    assert leg_fk(np.array([0.0, 0.0, 0.0]), GEOM) == pytest.approx([0.21, 0.0, 0.0])


def test_fk_standing():
    assert leg_fk(np.array([0.0, 0.0, -np.pi / 2]), GEOM) == pytest.approx([0.11, 0.0, -0.10])


def test_fk_coxa_yaw():
    foot = leg_fk(np.array([np.pi / 2, 0.0, -np.pi / 2]), GEOM)
    assert foot == pytest.approx([0.0, 0.11, -0.10], abs=1e-12)


def test_ik_fk_round_trip():
    lim = load_config().limits.as_array()
    grids = [np.linspace(lo, hi, 9) for lo, hi in lim]
    for a in itertools.product(*grids):
        a = np.array(a)
        foot = leg_fk(a, GEOM)
        if np.hypot(foot[0], foot[1]) < 1e-3 or np.cos(a[0]) * foot[0] + np.sin(a[0]) * foot[1] < 0:
            continue  # foot behind the coxa axis: outside the IK's domain, never commanded
        sol = leg_ik(foot, GEOM)
        np.testing.assert_allclose(leg_fk(sol, GEOM), foot, atol=1e-9)
        np.testing.assert_allclose(sol, a, atol=1e-9)


def test_ik_unreachable_raises():
    with pytest.raises(UnreachableError):
        leg_ik(np.array([0.5, 0.0, 0.0]), GEOM)
