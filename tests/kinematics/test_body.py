import numpy as np
import pytest

from rocky.config import load_config
from rocky.kinematics.body import body_to_leg, leg_mounts, leg_to_body, neutral_feet, rot_z

CFG = load_config()


def test_five_mounts_at_72_degrees():
    mounts = leg_mounts(CFG.body)
    assert [m.index for m in mounts] == [0, 1, 2, 3, 4]
    assert np.degrees([m.yaw for m in mounts]) == pytest.approx([0, 72, 144, 216, 288])
    for m in mounts:
        assert np.linalg.norm(m.position) == pytest.approx(0.07)
        assert m.position[2] == 0.0


def test_frame_round_trip():
    rng = np.random.default_rng(0)
    for m in leg_mounts(CFG.body):
        for p in rng.uniform(-0.3, 0.3, size=(20, 3)):
            np.testing.assert_allclose(leg_to_body(body_to_leg(p, m), m), p, atol=1e-12)


def test_neutral_foot_in_leg_frame():
    feet = neutral_feet(CFG)
    assert feet.shape == (5, 3)
    for m in leg_mounts(CFG.body):
        reach = CFG.gait.stance_radius - CFG.body.mount_radius
        expected = [reach, 0.0, -CFG.gait.body_height]
        np.testing.assert_allclose(body_to_leg(feet[m.index], m), expected, atol=1e-12)


def test_radial_symmetry():
    feet = neutral_feet(CFG)
    rotated = feet @ rot_z(np.radians(72)).T
    np.testing.assert_allclose(rotated, np.roll(feet, -1, axis=0), atol=1e-12)
