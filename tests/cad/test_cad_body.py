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


PRINTED = {"ChassisTop", "ChassisBottom", "Deck", "Carapace", "Belly", "BellyHatch"} | {
    f"SidePanel_{i}" for i in range(1, 6)
}
# every part in docs/bom.md that lives in the body
COMPONENTS = (
    {
        "pi5",
        "camera",
        "respeaker",
        "speaker",
        "pca9685",
        "lipo",
        "xl4005",
        "ina219",
        "fuse",
        "switch",
        "port",
    }
    | {f"ubec_{i}" for i in range(1, 6)}
    | {f"servo_board_{i}" for i in range(1, 6)}
)


def test_body_parts_fit_bed(report):
    assert set(report["printed"]) == PRINTED
    for name, part in report["printed"].items():
        assert max(part["bbox_mm"]) <= 180, f"{name} is {part['bbox_mm']} mm"


def test_every_component_is_modelled(report):
    assert set(report["components"]) == COMPONENTS


def test_body_no_interference(report):
    assert report["interference_neutral"] == []
    assert report["coxa_sweep"] == {}, "a leg collides with the body at a coxa limit"


def test_coxa_axis_at_mount_radius(report):
    assert report["coxa_axis_radius_mm"] == pytest.approx(CFG.body.mount_radius * 1000, abs=0.01)


def test_sim_body_thickness_matches_chassis(report):
    assert CFG.body.thickness * 1000 == pytest.approx(report["chassis_height_mm"], abs=1.0)
