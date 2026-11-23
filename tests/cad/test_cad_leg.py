import json
from pathlib import Path

import pytest

from rocky.config import load_config

REPORT = Path(__file__).resolve().parents[2] / "cad" / "reports" / "leg.json"
CFG = load_config()


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
    o = report["joint_origins_mm"]
    assert o["femur"] == pytest.approx(CFG.leg.coxa * 1000)
    assert o["knee"] == pytest.approx((CFG.leg.coxa + CFG.leg.femur) * 1000)


def test_limits_are_physical(report):
    assert report["limit_sweep"] == {}, "joints collide at their configured limits"
