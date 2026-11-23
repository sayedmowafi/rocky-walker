import pytest

from tools.fusion_build import render_script


def test_render_substitutes_dotted_keys(tmp_path):
    src = tmp_path / "s.py"
    src.write_text("a = {{leg.coxa_mm}}\nb = '{{gait.default}}'\nc = {'x': 1}\n")
    out = render_script(src, {"leg": {"coxa_mm": 40}, "gait": {"default": "wave"}})
    assert out == "a = 40\nb = 'wave'\nc = {'x': 1}\n"


def test_render_unknown_key_raises(tmp_path):
    src = tmp_path / "s.py"
    src.write_text("a = {{leg.nope}}\n")
    with pytest.raises(KeyError, match="leg.nope"):
        render_script(src, {"leg": {}})
