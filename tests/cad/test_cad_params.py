import json
import tomllib
from pathlib import Path

import pytest

from rocky.config import DEFAULT_CONFIG_PATH

REPORT = Path(__file__).resolve().parents[2] / "cad" / "reports" / "params.json"


@pytest.fixture(scope="module")
def report():
    assert REPORT.exists(), f"missing {REPORT}: run cad/fusion/report_params.py"
    return json.loads(REPORT.read_text())


@pytest.fixture(scope="module")
def toml():
    with open(DEFAULT_CONFIG_PATH, "rb") as f:
        return tomllib.load(f)


def _mirrored(toml):
    out = {
        "coxa_mm": toml["leg"]["coxa_mm"],
        "femur_mm": toml["leg"]["femur_mm"],
        "tibia_mm": toml["leg"]["tibia_mm"],
        "mount_radius_mm": toml["body"]["mount_radius_mm"],
        "body_height_mm": toml["gait"]["body_height_mm"],
    }
    for joint in ("coxa", "femur", "tibia"):
        lo, hi = toml["limits"][f"{joint}_deg"]
        out[f"{joint}_min_deg"], out[f"{joint}_max_deg"] = lo, hi
    return out


def test_params_match_config(report, toml):
    bad = [
        f"{k}: fusion={report['params'].get(k)} toml={v}"
        for k, v in _mirrored(toml).items()
        if report["params"].get(k) is None or abs(report["params"][k] - v) > 1e-6
    ]
    assert not bad, "Fusion parameters differ from config/rocky.toml:\n" + "\n".join(bad)


def test_skeleton_matches_config(report, toml):
    leg = toml["leg"]
    frame = report["leg_frame"]
    assert frame["axis_femur_x"] == pytest.approx(leg["coxa_mm"], abs=1e-4)
    assert frame["axis_knee_x"] == pytest.approx(leg["coxa_mm"] + leg["femur_mm"], abs=1e-4)
    tip = leg["coxa_mm"] + leg["femur_mm"] + leg["tibia_mm"]
    assert frame["foot_tip_x"] == pytest.approx(tip, abs=1e-4)
    assert max(frame["axes_z"]) - min(frame["axes_z"]) < 1e-4
    assert frame["leg_origin_x"] == pytest.approx(toml["body"]["mount_radius_mm"], abs=1e-4)
