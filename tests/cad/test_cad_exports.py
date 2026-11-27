import json
from pathlib import Path

import numpy as np
import pytest

from rocky.config import load_config

CAD = Path(__file__).resolve().parents[2] / "cad"
CFG = load_config()
STL_DTYPE = np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")])


def read_stl(path: Path) -> np.ndarray:
    """Vertices (N, 3, 3) of a binary STL."""
    data = path.read_bytes()
    count = int(np.frombuffer(data, "<u4", 1, 80)[0])
    tris = np.frombuffer(data, STL_DTYPE, count, 84)
    return tris["v"]


def test_printable_stls_fit_bed():
    printed = set(json.loads((CAD / "reports" / "leg.json").read_text())["printed"])
    printed |= set(json.loads((CAD / "reports" / "body.json").read_text())["printed"])
    printed.add("BenchMount")
    for name in printed:
        path = CAD / "stl" / f"{name}.stl"
        assert path.exists(), f"missing {path}"
        v = read_stl(path).reshape(-1, 3)
        size = v.max(0) - v.min(0)
        assert size.max() <= 180, f"{name} is {size} mm"


def test_manifest_meshes():
    manifest = json.loads((CAD / "meshes" / "manifest.json").read_text())
    assert manifest["units"] == "mm"
    segments = {p["segment"] for p in manifest["parts"]}
    assert {"torso", "leg_static", "coxa", "femur", "tibia"} <= segments
    for p in manifest["parts"]:
        assert (CAD / "meshes" / p["file"]).exists(), p["file"]
        assert p["color"] in {"blue", "white", "black", "grey", "silver"}


def test_tibia_tip_at_segment_length():
    manifest = json.loads((CAD / "meshes" / "manifest.json").read_text())
    # the foot tip is the contact point: printed tibia + TPU cap together reach the full length
    tips = []
    for part in (p for p in manifest["parts"] if p["segment"] == "tibia"):
        v = read_stl(CAD / "meshes" / part["file"]).reshape(-1, 3) - np.array(part["origin_mm"])
        tips.append(v[:, 0].max())
    assert max(tips) == pytest.approx(CFG.leg.tibia * 1000, abs=0.5)


def test_source_and_step_exist():
    for name in ("rocky.f3d", "rocky.step"):
        path = CAD / name
        assert path.exists() and path.stat().st_size > 1000, name
