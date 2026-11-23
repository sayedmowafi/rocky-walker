"""Render a Fusion script template with values from config/rocky.toml and run it in Fusion.

Templates use ``{{section.key}}`` placeholders (dotted TOML keys), plus ``{{repo}}`` for the
repository's absolute path. Usage:

    uv run python -m tools.fusion_build cad/fusion/01_params.py
    uv run python -m tools.fusion_build cad/fusion/report_params.py --read-only \\
        --out cad/reports/params.json
"""

from __future__ import annotations

import argparse
import re
import sys
import tomllib
from pathlib import Path
from typing import Any

from rocky.config import DEFAULT_CONFIG_PATH
from tools.fusion_mcp import FusionClient, FusionError

REPO = Path(__file__).resolve().parent.parent
_PLACEHOLDER = re.compile(r"\{\{\s*([\w.]+)\s*\}\}")


def render_script(path: Path, cfg_raw: dict[str, Any]) -> str:
    """Substitute ``{{dotted.key}}`` placeholders with values from ``cfg_raw``."""
    values = {**cfg_raw, "repo": str(REPO)}

    def lookup(m: re.Match[str]) -> str:
        node: Any = values
        for part in m.group(1).split("."):
            if not isinstance(node, dict) or part not in node:
                raise KeyError(f"unknown template key {m.group(1)}")
            node = node[part]
        return str(node)

    return _PLACEHOLDER.sub(lookup, Path(path).read_text())


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m tools.fusion_build")
    parser.add_argument("script", type=Path)
    parser.add_argument("--read-only", action="store_true")
    parser.add_argument("--out", type=Path, default=None, help="write the script output here")
    args = parser.parse_args(argv)
    with open(DEFAULT_CONFIG_PATH, "rb") as f:
        cfg_raw = tomllib.load(f)
    try:
        out = FusionClient().run_script(render_script(args.script, cfg_raw), args.read_only)
    except FusionError as e:
        print(e, file=sys.stderr)
        return 1
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(out)
        print(f"wrote {args.out}")
    else:
        print(out, end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
