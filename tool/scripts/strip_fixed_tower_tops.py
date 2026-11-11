#!/usr/bin/env python3
"""Remove static field meshes that are replaced or spawned separately.

L1 rails are rendered as explicit SDF box visuals. The L1-level center
divider is replaced by two SDF box segments; small L1-to-L2 steps are
removed, and tower tops are spawned
separately by the placement GUI.  Keeping these nodes in the field DAE would
create duplicate or undeletable visuals.
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_DAE = (
    REPO_ROOT
    / "src"
    / "robocon_gazebo"
    / "models"
    / "robocon_ground_full"
    / "meshes"
    / "Untitled.dae"
)

# Blender exported these as top-level visual-scene nodes with no nested nodes.
NODE_RE = re.compile(
    r'\n      <node id="(?:Cube\.(?:0(?:1[1-689]|2[0-9]|3[0-9]|4[0-3]))|Plane\.014)"'
    r'.*?\n      </node>',
    flags=re.DOTALL,
)


def strip_nodes(text: str) -> tuple[str, int]:
    cleaned, count = NODE_RE.subn("", text)
    return cleaned, count


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dae", nargs="?", type=Path, default=DEFAULT_DAE)
    args = parser.parse_args()

    text = args.dae.read_text(encoding="utf-8")
    cleaned, count = strip_nodes(text)
    if count == 0:
        print(f"{args.dae}: already clean (no replaceable static visual nodes)")
        return
    if "</visual_scene>" not in cleaned:
        raise SystemExit(f"{args.dae}: visual_scene missing after rewrite")
    args.dae.write_text(cleaned, encoding="utf-8")
    print(f"removed {count} replaceable static visual nodes from {args.dae}")


if __name__ == "__main__":
    main()
