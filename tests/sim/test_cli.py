import copy
import tomllib

import pytest

from rocky.config import DEFAULT_CONFIG_PATH, ConfigError, parse_config
from rocky.sim.__main__ import main
from rocky.sim.runner import simulate

pytestmark = pytest.mark.sim


def _bad_stance_toml(tmp_path):
    text = DEFAULT_CONFIG_PATH.read_text()
    with open(DEFAULT_CONFIG_PATH, "rb") as f:
        stance = tomllib.load(f)["gait"]["stance_radius_mm"]
    bad = tmp_path / "bad.toml"
    bad.write_text(text.replace(f"stance_radius_mm = {stance}", "stance_radius_mm = 400", 1))
    return bad


def test_simulate_bad_stance_raises_config_error():
    with open(DEFAULT_CONFIG_PATH, "rb") as f:
        data = copy.deepcopy(tomllib.load(f))
    data["gait"]["stance_radius_mm"] = 400
    with pytest.raises(ConfigError, match="stance_radius_mm"):
        simulate(parse_config(data), lambda t: None, 1.0)


def test_cli_bad_config_exits_cleanly(tmp_path, capsys):
    bad = _bad_stance_toml(tmp_path)
    assert main(["--config", str(bad), "walk", "--vx", "0.01", "--duration", "1"]) == 2
    assert "stance_radius_mm" in capsys.readouterr().err
