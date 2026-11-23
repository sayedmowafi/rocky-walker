import os
import subprocess
from pathlib import Path

import pytest

from rocky.config import load_config
from rocky.moves.move import load_move
from rocky.showcase.animation import move_frames, write_animation

ROOT = Path(__file__).resolve().parents[2]
BLENDER = os.environ.get("BLENDER", "/Applications/Blender.app/Contents/MacOS/Blender")

pytestmark = [
    pytest.mark.blender,
    pytest.mark.skipif(not Path(BLENDER).exists(), reason="Blender not installed"),
]


def test_renders_two_frames(tmp_path):
    frames = move_frames(load_config(), load_move(ROOT / "moves" / "bob.toml"))
    anim = tmp_path / "bob.json"
    write_animation(frames, anim)
    out = tmp_path / "bob.mp4"
    cmd = [
        BLENDER,
        "--background",
        "--factory-startup",
        "--python",
        str(ROOT / "showcase" / "build_scene.py"),
        "--",
        "--meshes",
        str(ROOT / "cad" / "meshes"),
        "--config",
        str(ROOT / "config" / "rocky.toml"),
        "--anim",
        str(anim),
        "--out",
        str(out),
        "--frames",
        "2",
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
    assert proc.returncode == 0, proc.stdout[-3000:] + proc.stderr[-3000:]
    assert out.exists() and out.stat().st_size > 10_000
