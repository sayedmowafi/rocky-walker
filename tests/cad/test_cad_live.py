"""Live check against the open Fusion design (skipped when Fusion's MCP server isn't reachable)."""

import json
import tomllib

import pytest
from test_cad_params import mirrored

from rocky.config import DEFAULT_CONFIG_PATH
from tools.fusion_build import REPO, render_script
from tools.fusion_mcp import FusionClient

pytestmark = pytest.mark.fusion


def test_live_fusion_params_match_config():
    with open(DEFAULT_CONFIG_PATH, "rb") as f:
        toml = tomllib.load(f)
    out = FusionClient(timeout=60).run_script(
        render_script(REPO / "cad" / "fusion" / "report_params.py", toml), read_only=True
    )
    params = json.loads(out)["params"]
    bad = [
        f"{k}: fusion={params.get(k)} toml={v}"
        for k, v in mirrored(toml).items()
        if params.get(k) is None or abs(params[k] - v) > 1e-6
    ]
    assert not bad, "the open Fusion design differs from config/rocky.toml:\n" + "\n".join(bad)
