"""Export animation clips for Blender.

uv run python -m rocky.showcase export walk-forward out/anim/walk-forward.json
uv run python -m rocky.showcase export all out/anim/
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from rocky.config import load_config
from rocky.showcase.animation import CLIPS, max_stance_drift, write_animation


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m rocky.showcase")
    sub = parser.add_subparsers(dest="cmd", required=True)
    exp = sub.add_parser("export", help="write a clip's animation JSON")
    exp.add_argument("clip", choices=[*CLIPS, "all"])
    exp.add_argument("out", type=Path, help="JSON file, or a directory for 'all'")
    args = parser.parse_args(argv)
    cfg = load_config()
    names = list(CLIPS) if args.clip == "all" else [args.clip]
    for name in names:
        frames = CLIPS[name](cfg)
        path = args.out / f"{name}.json" if args.clip == "all" else args.out
        write_animation(frames, path)
        drift = max_stance_drift(cfg, frames) * 1000
        print(
            f"{name}: {len(frames)} frames, {frames[-1].t:.1f} s, "
            f"foot drift {drift:.3f} mm -> {path}"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
