import numpy as np
import pytest

from rocky.config import load_config
from rocky.kinematics.body import body_to_leg, leg_mounts, neutral_feet
from rocky.kinematics.leg import leg_ik
from rocky.kinematics.mass import center_of_mass, segment_coms, support_margin
from rocky.kinematics.pose import BodyPose
from rocky.motion.controller import MotionController

CFG = load_config()
STAND = MotionController(CFG).angles


def test_com_neutral_is_centered():
    com = center_of_mass(CFG, BodyPose(), STAND)
    np.testing.assert_allclose(com[:2], [0, 0], atol=1e-9)


def _planted_angles(pose):
    # feet stay where they stand; re-solve the legs for the moved body
    inv = np.linalg.inv(pose.matrix())
    out = []
    for m, foot in zip(leg_mounts(CFG.body), neutral_feet(CFG), strict=True):
        f = (inv @ np.append(foot, 1.0))[:3]
        out.append(leg_ik(body_to_leg(f, m), CFG.leg))
    return np.array(out)


def test_com_moves_with_body_shift():
    pose = BodyPose(x=0.02)
    com = center_of_mass(CFG, pose, _planted_angles(pose))
    assert 0 < com[0] < 0.02


def test_segment_coms_shape_and_order():
    coms = segment_coms(CFG, np.zeros((5, 3)))
    assert coms.shape == (5, 3, 3)
    r = CFG.body.mount_radius
    np.testing.assert_allclose(coms[0, 0], [r + CFG.mass.coxa_com, 0, 0], atol=1e-12)
    np.testing.assert_allclose(
        coms[0, 2], [r + CFG.leg.coxa + CFG.leg.femur + CFG.mass.tibia_com, 0, 0], atol=1e-12
    )


def test_support_margin_square():
    sq = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], float)
    assert support_margin(sq, np.array([0.5, 0.5])) == pytest.approx(0.5)
    assert support_margin(sq, np.array([1.5, 0.5])) == pytest.approx(-0.5)
    assert support_margin(sq[:2], np.array([0.5, 0.0])) == -np.inf


def test_pose_matrix_round_trip():
    pose = BodyPose(0.01, -0.02, 0.005, 0.1, -0.2, 0.3)
    m = pose.matrix()
    np.testing.assert_allclose(m[:3, :3] @ m[:3, :3].T, np.eye(3), atol=1e-12)
    np.testing.assert_allclose(m[:3, 3], [0.01, -0.02, 0.005])
    yaw_only = BodyPose(yaw=np.pi / 2).matrix()
    np.testing.assert_allclose(yaw_only[:3, :3] @ [1, 0, 0], [0, 1, 0], atol=1e-12)
