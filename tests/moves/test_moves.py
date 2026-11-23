from pathlib import Path

import numpy as np
import pytest

from rocky.config import ConfigError, load_config
from rocky.kinematics.pose import BodyPose
from rocky.moves.move import Keyframe, Move, load_move
from rocky.moves.player import MovePlayer

CFG = load_config()
MOVES = Path(__file__).resolve().parents[2] / "moves"
NEUTRAL = BodyPose()


def neutral_key(t):
    return Keyframe(t, NEUTRAL, {})


@pytest.mark.parametrize("path", sorted(MOVES.glob("*.toml")), ids=lambda p: p.stem)
def test_all_moves_validate(path):
    MovePlayer(CFG).validate(load_move(path))


def test_expected_moves_exist():
    names = {p.stem for p in MOVES.glob("*.toml")}
    assert {"sway", "bob", "twist", "tilt-circle", "leg-wave", "routine"} <= names


def test_rejects_unstable_move():
    # three legs up leaves two feet on the floor: no support polygon at all
    up = {leg: np.array([0.0, 0.0, 0.03]) for leg in (0, 1, 2)}
    move = Move("bad", (neutral_key(0.0), Keyframe(1.0, NEUTRAL, up), neutral_key(2.0)))
    with pytest.raises(ConfigError, match=r"bad at t=.*support"):
        MovePlayer(CFG).validate(move)


def test_rejects_out_of_reach_move():
    lifted = {0: np.array([0.0, 0.0, 0.04])}
    move = Move(
        "far", (neutral_key(0.0), Keyframe(1.0, BodyPose(x=0.06), lifted), neutral_key(2.0))
    )
    with pytest.raises(ConfigError, match=r"far at t="):
        MovePlayer(CFG).validate(move)


def test_move_must_start_and_end_neutral():
    move = Move("bad", (neutral_key(0.0), Keyframe(1.0, BodyPose(z=0.01), {})))
    with pytest.raises(ConfigError, match="neutral"):
        MovePlayer(CFG).validate(move)


def test_play_needs_no_guards():
    player = MovePlayer(CFG)
    move = load_move(MOVES / "routine.toml")
    frames = list(player.play(move, 0.01))
    assert len(frames) == round(move.duration / 0.01) + 1
    assert player.stats.clamped == player.stats.speed_limited == player.stats.unreachable == 0


def test_sample_eases_between_keys():
    move = Move("m", (neutral_key(0.0), Keyframe(1.0, BodyPose(x=0.02), {}), neutral_key(2.0)))
    player = MovePlayer(CFG)
    assert player.sample(move, 0.5)[0].x == pytest.approx(0.01)
    s = 0.25 * 0.25 * (3 - 2 * 0.25)
    assert player.sample(move, 0.25)[0].x == pytest.approx(0.02 * s)
    assert player.sample(move, 9.0)[0] == NEUTRAL


def test_routine_concatenates():
    parts = ["sway", "bob", "twist", "tilt-circle", "leg-wave"]
    total = sum(load_move(MOVES / f"{p}.toml").duration for p in parts)
    assert load_move(MOVES / "routine.toml").duration == pytest.approx(total)


def test_load_rejects_bad_times(tmp_path):
    f = tmp_path / "bad.toml"
    f.write_text('name = "bad"\n[[key]]\nt = 0.0\n[[key]]\nt = 0.0\n')
    with pytest.raises(ConfigError, match="increasing"):
        load_move(f)
