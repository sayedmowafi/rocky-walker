import json
from pathlib import Path

import pytest

from rocky.config import load_config

REPORT = Path(__file__).resolve().parents[2] / "cad" / "reports" / "body.json"
CFG = load_config()


@pytest.fixture(scope="module")
def report():
    assert REPORT.exists(), f"missing {REPORT}: run cad/fusion/report_body.py"
    return json.loads(REPORT.read_text())


def test_body_parts_fit_bed(report):
    assert {"ChassisTop", "ChassisBottom", "Deck", "Carapace", "BatteryTray"} <= set(
        report["printed"]
    )
    for name, part in report["printed"].items():
        assert max(part["bbox_mm"]) <= 180, f"{name} is {part['bbox_mm']} mm"


def test_body_no_interference(report):
    assert report["interference_neutral"] == []
    assert report["coxa_sweep"] == {}, "a leg collides with the body at a coxa limit"


def test_coxa_axis_at_mount_radius(report):
    assert report["coxa_axis_radius_mm"] == pytest.approx(CFG.body.mount_radius * 1000, abs=0.01)


def test_sim_body_thickness_matches_chassis(report):
    assert CFG.body.thickness * 1000 == pytest.approx(report["chassis_height_mm"], abs=1.0)
