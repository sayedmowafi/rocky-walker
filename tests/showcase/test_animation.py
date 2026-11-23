import json
from pathlib import Path

import numpy as np
import pytest

from rocky.commands import BodyVelocity
from rocky.config import load_config
from rocky.moves.move import load_move
from rocky.showcase.animation import (
    CLIPS,
    feet_world,
    max_stance_drift,
    move_frames,
    read_animation,
    walk_frames,
    write_animation,
)

CFG = load_config()
ROUTINE = Path(__file__).resolve().parents[2] / "moves" / "routine.toml"


@pytest.mark.parametrize("clip", ["walk-forward", "walk-diagonal", "turn"])
def test_walk_feet_planted(clip):
    frames = CLIPS[clip](CFG)
    assert max_stance_drift(CFG, frames) < 0.001
    # the robot actually goes somewhere
    start, end = frames[0].pose_world, frames[-1].pose_world
    moved = np.linalg.norm(end[:2, 3] - start[:2, 3]) + abs(np.arctan2(end[1, 0], end[0, 0]))
    assert moved > 0.05


def test_frame_times_exact():
    frames = walk_frames(CFG, lambda t: BodyVelocity(vx=0.03), 2.0)
    for k, f in enumerate(frames):
        assert f.t == pytest.approx(k / 30, abs=1e-9)
    assert len(frames) == round(3.0 * 30) + 1  # 1 s settle + 2 s


def test_feet_on_floor_at_start():
    frames = CLIPS["walk-forward"](CFG)
    np.testing.assert_allclose(feet_world(CFG, frames[0])[:, 2], 0.0, atol=1e-9)


def test_dance_feet_planted():
    frames = move_frames(CFG, load_move(ROUTINE))
    assert max_stance_drift(CFG, frames) < 0.001


def test_json_round_trip(tmp_path):
    frames = move_frames(CFG, load_move(ROUTINE))[:10]
    path = tmp_path / "a.json"
    write_animation(frames, path)
    data = json.loads(path.read_text())
    assert data["fps"] == 30 and len(data["joint_names"]) == 15
    back = read_animation(path)
    for a, b in zip(frames, back, strict=True):
        np.testing.assert_allclose(a.pose_world, b.pose_world)
        np.testing.assert_allclose(a.angles, b.angles)
