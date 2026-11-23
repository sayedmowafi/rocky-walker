import math
from pathlib import Path

import pytest

from rocky.config import load_config
from rocky.moves.move import load_move
from rocky.moves.player import MovePlayer
from rocky.sim.runner import simulate_angles

pytestmark = pytest.mark.sim

CFG = load_config()
ROUTINE = Path(__file__).resolve().parents[2] / "moves" / "routine.toml"


def test_routine_meets_torque_rule_in_physics():
    move = load_move(ROUTINE)
    dt = 1 / CFG.motion.loop_hz
    stream = [a for _, _, a in MovePlayer(CFG).play(move, dt)]
    res = simulate_angles(CFG, stream)
    assert len(res.times) == len(stream)
    assert res.sustained_torque_peak().max() <= CFG.servo.torque_rule
    assert math.degrees(res.torso_tilt.max()) < 20
