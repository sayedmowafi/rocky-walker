import json
import tomllib
from pathlib import Path

import pytest
from test_cad_params import mirrored

from rocky.config import DEFAULT_CONFIG_PATH, load_config

REPORTS = Path(__file__).resolve().parents[2] / "cad" / "reports"
CFG = load_config()


@pytest.fixture(scope="module")
def mass():
    path = REPORTS / "mass.json"
    assert path.exists(), f"missing {path}: run cad/fusion/report_mass.py"
    return json.loads(path.read_text())


def test_five_legs_no_interference(mass):
    assert mass["legs"] == 5
    assert mass["interference_neutral"] == []


def test_legs_on_their_mounts(mass):
    mounts = mass["leg_mounts_r_deg_z"]
    for k, (r, deg, z) in enumerate(mounts):
        assert r == pytest.approx(CFG.body.mount_radius * 1000, abs=0.01)
        assert deg == pytest.approx(72 * k, abs=0.01)
        assert z == pytest.approx(0, abs=0.01)


def test_mass_report_built_from_current_config(mass):
    with open(DEFAULT_CONFIG_PATH, "rb") as f:
        toml = tomllib.load(f)
    for key, value in mirrored(toml).items():
        snap = mass["params_snapshot"][key]
        assert snap == pytest.approx(value, abs=1e-6), f"mass.json is stale: {key}"


def test_config_masses_match_cad(mass):
    seg = mass["segments"]
    assert CFG.mass.total == pytest.approx(mass["total_kg"], abs=0.005)
    assert CFG.mass.coxa_link == pytest.approx(seg["coxa"]["kg"], abs=0.002)
    assert CFG.mass.femur_link == pytest.approx(seg["femur"]["kg"], abs=0.002)
    assert CFG.mass.tibia_link == pytest.approx(seg["tibia"]["kg"], abs=0.002)
    assert CFG.mass.femur_com * 1000 == pytest.approx(seg["femur"]["com_mm"][0], abs=1.0)
