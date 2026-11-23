import json
import tomllib
from pathlib import Path

import numpy as np
import pytest
from test_cad_params import mirrored

from rocky.config import DEFAULT_CONFIG_PATH, load_config

REPORT = Path(__file__).resolve().parents[2] / "cad" / "reports" / "leg.json"
CFG = load_config()
with open(DEFAULT_CONFIG_PATH, "rb") as _f:
    TOML = tomllib.load(_f)


@pytest.fixture(scope="module")
def report():
    assert REPORT.exists(), f"missing {REPORT}: run cad/fusion/report_leg.py"
    return json.loads(REPORT.read_text())


def test_printable_parts_fit_bed(report):
    assert {"Coxa", "Femur", "Tibia"} <= set(report["printed"])
    for name, part in report["printed"].items():
        assert max(part["bbox_mm"]) <= 180, f"{name} is {part['bbox_mm']} mm"


def test_no_interference_at_stance(report):
    assert report["interference_zero"] == []
    assert report["interference_neutral"] == []


def test_joint_origins_match_config(report):
    # read from the actual Fusion joints, not from the parameters
    j = report["joints"]
    assert j["coxa"]["origin_mm"] == pytest.approx([0, 0, 0], abs=1e-4)
    assert j["femur"]["origin_mm"] == pytest.approx([CFG.leg.coxa * 1000, 0, 0], abs=1e-4)
    knee = (CFG.leg.coxa + CFG.leg.femur) * 1000
    assert j["knee"]["origin_mm"] == pytest.approx([knee, 0, 0], abs=1e-4)


def test_fusion_joint_limits_match_config(report):
    lim = np.degrees(CFG.limits.as_array())
    for k, name in enumerate(("coxa", "femur", "knee")):
        assert report["joints"][name]["limits_deg"] == pytest.approx(lim[k], abs=1e-6), name


def test_clear_range_matches_config_limits(report):
    lim = np.degrees(CFG.limits.as_array())
    for k, name in enumerate(("coxa", "femur", "knee")):
        assert report["clear_range_deg"][name] == pytest.approx(lim[k], abs=1e-6), name


def test_report_built_from_current_config(report):
    snap = report["params_snapshot"]
    for key, value in mirrored(TOML).items():
        assert snap[key] == pytest.approx(value, abs=1e-6), f"leg.json is stale: {key}"


def test_limits_are_physical(report):
    assert report["limit_sweep"] == {}, "joints collide at their configured limits"
