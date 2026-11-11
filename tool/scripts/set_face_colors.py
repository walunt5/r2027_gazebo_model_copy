#!/usr/bin/env python3
"""Recolor four field faces to the regulation gray RGB(190,190,185).

Targets (all four are "green/white boundary" strips that Gazebo renders white
because their visual-scene node is self-closing with no <bind_material>, so the
geometry's material symbol cannot be resolved):

  1. `Plane.008` (L1 top, x -3.0~-2.25, y +/-0.5, z=0.602) -- left strip
  2. `Plane.009` (L1 top, x  2.25~ 3.0, y +/-0.5, z=0.602) -- right strip
  3. `Public_Zone2` (ground, cylinder pad x 3.65~4.65, y +/-0.5)
  4. `Cube.017` top (L2, 3x3 @ z=0.9) -- its 6 faces share one <triangles>, so
     instead of recoloring the walls we add a coplanar gray patch
     `L2_top_gray` at z=0.9005 and bind the walls back to the original material.

Nothing else is touched: `Material.025` / `Material.026` (the red / blue L1
playing-field halves) must stay red and blue.

Idempotent: running twice is a no-op.
"""

from __future__ import annotations

import argparse
import re
import shutil
import sys
import time
from pathlib import Path
from xml.etree import ElementTree

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
WORKSPACE = REPO_ROOT
DEFAULT_DAE = (
    WORKSPACE
    / "src"
    / "robocon_gazebo"
    / "models"
    / "robocon_ground_full"
    / "meshes"
    / "Untitled.dae"
)
BACKUP_DIR = WORKSPACE / ".cache" / "keep"

GRAY = "0.745098 0.745098 0.725490 1"  # RGB(190,190,185)

# Self-closing visual nodes that currently render white -> bind to Gray-material.
#   Plane.008 / Plane.009 : L1 top edge strips (x -3.0~-2.25 / 2.25~3.0, y +/-0.5)
#   Public_Zone2          : ground cylinder pad (x 3.65~4.65, y +/-0.5)
GRAY_BIND_NODES = ("Plane.008", "Plane.009", "Public_Zone2")

GRAY_EFFECT_ID = "Gray-effect"
GRAY_MATERIAL_ID = "Gray-material"
GRAY_GEOM_ID = "L2_top_gray-mesh"
GRAY_NODE_ID = "L2_top_gray"
GRAY_SYMBOL = "Gray"
L2_TOP_Z = "0.9005"

GRAY_EFFECT = """    <effect id="{eid}">
      <profile_COMMON>
        <technique sid="common">
          <lambert>
            <emission>
              <color sid="emission">0 0 0 1</color>
            </emission>
            <diffuse>
              <color sid="diffuse">{color}</color>
            </diffuse>
          </lambert>
        </technique>
      </profile_COMMON>
    </effect>
"""

GRAY_MATERIAL = """    <material id="{mid}" name="Gray">
      <instance_effect url="#{eid}" />
    </material>
"""

# 4-vertex quad, x/y in [-1.5, 1.5] at z = 0.9005, same layout as Plane.009.
GRAY_GEOMETRY = """    <geometry id="{gid}" name="L2_top_gray">
      <mesh>
        <source id="{gid}-positions">
          <float_array id="{gid}-positions-array" count="12">1.5 1.5 {z} -1.5 1.5 {z} 1.5 -1.5 {z} -1.5 -1.5 {z}</float_array>
          <technique_common>
            <accessor source="#{gid}-positions-array" count="4" stride="3">
              <param name="X" type="float" />
              <param name="Y" type="float" />
              <param name="Z" type="float" />
            </accessor>
          </technique_common>
        </source>
        <source id="{gid}-normals">
          <float_array id="{gid}-normals-array" count="3">0 0 1</float_array>
          <technique_common>
            <accessor source="#{gid}-normals-array" count="1" stride="3">
              <param name="X" type="float" />
              <param name="Y" type="float" />
              <param name="Z" type="float" />
            </accessor>
          </technique_common>
        </source>
        <vertices id="{gid}-vertices">
          <input semantic="POSITION" source="#{gid}-positions" />
        </vertices>
        <triangles count="2" material="{sym}">
          <input semantic="VERTEX" source="#{gid}-vertices" offset="0" />
          <input semantic="NORMAL" source="#{gid}-normals" offset="1" />
          <p>0 0 1 0 3 0 0 0 3 0 2 0</p>
        </triangles>
      </mesh>
    </geometry>
"""

GRAY_NODE = """      <node id="{nid}" name="{nid}" type="NODE">
        <instance_geometry url="#{gid}">
          <bind_material>
            <technique_common>
              <instance_material symbol="{sym}" target="#{mid}" />
            </technique_common>
          </bind_material>
        </instance_geometry>
      </node>
"""

BIND_TMPL = """<bind_material>
            <technique_common>
              <instance_material symbol="{sym}" target="#{mid}" />
            </technique_common>
          </bind_material>"""


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("dae", nargs="?", type=Path, default=DEFAULT_DAE)
    p.add_argument("--check", action="store_true", help="report changes, do not write")
    p.add_argument("--restore", action="store_true", help="restore newest backup")
    return p.parse_args()


def bind_white_nodes(text: str) -> tuple[str, list[str]]:
    """Expand self-closing instance_geometry nodes so they bind to Gray-material.

    These nodes render white because the geometry's `Material` symbol has no
    node-level binding. Their geometry triangles all use symbol `Material`, so
    binding is straightforward and per-node (does not affect Cube / Cube.001).
    """
    changed: list[str] = []
    for nid in GRAY_BIND_NODES:
        node_pat = re.compile(
            r'(<node id="' + re.escape(nid) + r'"[^>]*>\s*)'
            r'<instance_geometry url="#' + re.escape(nid) + r'-mesh" />(\s*</node>)',
            flags=re.DOTALL,
        )
        if not node_pat.search(text):
            if f'id="{nid}"' not in text:
                raise SystemExit(f"node {nid} not found")
            continue  # already expanded
        body = (
            f'<instance_geometry url="#{nid}-mesh">\n          '
            + BIND_TMPL.format(sym="Material", mid=GRAY_MATERIAL_ID)
            + "\n        </instance_geometry>"
        )
        text = node_pat.sub(lambda mm: mm.group(1) + body + mm.group(2), text, count=1)
        changed.append(nid)
    return text, changed


def add_gray_material(text: str) -> tuple[str, list[str]]:
    """Append Gray-effect / Gray-material if absent."""
    changed: list[str] = []
    if f'id="{GRAY_EFFECT_ID}"' not in text:
        text = text.replace(
            "  </library_effects>",
            GRAY_EFFECT.format(eid=GRAY_EFFECT_ID, color=GRAY)
            + "  </library_effects>",
            1,
        )
        changed.append(GRAY_EFFECT_ID)
    if f'id="{GRAY_MATERIAL_ID}"' not in text:
        text = text.replace(
            "  </library_materials>",
            GRAY_MATERIAL.format(mid=GRAY_MATERIAL_ID, eid=GRAY_EFFECT_ID)
            + "  </library_materials>",
            1,
        )
        changed.append(GRAY_MATERIAL_ID)
    return text, changed


def bind_cube017(text: str) -> tuple[str, bool]:
    """Bind Cube.017 walls back to the original Material (they render white now).

    Only the walls matter: the top face is covered by the L2_top_gray patch.
    """
    node_pat = re.compile(
        r'(<node id="Cube\.017"[^>]*>\s*)'
        r'<instance_geometry url="#Cube\.017-mesh" />(\s*</node>)',
        flags=re.DOTALL,
    )
    m = node_pat.search(text)
    if not m:
        if 'id="Cube.017"' not in text:
            raise SystemExit("node Cube.017 not found")
        return text, False  # already expanded
    body = (
        f'<instance_geometry url="#Cube.017-mesh">\n          '
        + BIND_TMPL.format(sym="Material", mid="Material-material")
        + "\n        </instance_geometry>"
    )
    text = node_pat.sub(lambda mm: mm.group(1) + body + mm.group(2), text, count=1)
    return text, True


def add_l2_patch(text: str) -> tuple[str, bool]:
    """Add the L2 top gray patch geometry + node."""
    if f'id="{GRAY_NODE_ID}"' in text:
        return text, False
    text = text.replace(
        "  </library_geometries>",
        GRAY_GEOMETRY.format(gid=GRAY_GEOM_ID, z=L2_TOP_Z, sym=GRAY_SYMBOL)
        + "  </library_geometries>",
        1,
    )
    text = text.replace(
        "    </visual_scene>",
        GRAY_NODE.format(nid=GRAY_NODE_ID, gid=GRAY_GEOM_ID, sym=GRAY_SYMBOL, mid=GRAY_MATERIAL_ID)
        + "    </visual_scene>",
        1,
    )
    return text, True


def transform(text: str) -> tuple[str, list[str]]:
    log: list[str] = []

    text, ids = add_gray_material(text)
    log += [f"added {i}" for i in ids]

    text, ids = bind_white_nodes(text)
    log += [f"bound {i} -> Gray-material" for i in ids]

    text, done = bind_cube017(text)
    if done:
        log.append("bound Cube.017 walls -> Material-material")

    text, done = add_l2_patch(text)
    if done:
        log.append(f"added {GRAY_NODE_ID} patch @ z={L2_TOP_Z}")

    return text, log


def validate(path: Path, text: str) -> None:
    ElementTree.fromstring(text)  # raises on malformed XML
    if "</visual_scene>" not in text:
        raise SystemExit(f"{path}: visual_scene missing after rewrite")
    for needle in (
        GRAY_EFFECT_ID,
        GRAY_MATERIAL_ID,
        GRAY_NODE_ID,
        GRAY_GEOM_ID,
        f'id="{GRAY_NODE_ID}"',
    ):
        if needle not in text:
            raise SystemExit(f"{path}: validation failed, missing {needle}")


def main() -> None:
    args = parse_args()
    path: Path = args.dae

    if args.restore:
        backups = sorted(BACKUP_DIR.glob(f"{path.stem}.dae.bak.*"))
        if not backups:
            raise SystemExit(f"no backups in {BACKUP_DIR}")
        newest = backups[-1]
        shutil.copy2(newest, path)
        print(f"restored {path} from {newest.name}")
        return

    original = path.read_text(encoding="utf-8")
    updated, log = transform(original)

    if updated == original:
        print(f"{path}: already up to date (nothing to do)")
        return

    validate(path, updated)
    print(f"{path}:")
    for line in log:
        print(f"  - {line}")

    if args.check:
        print("--check: not writing")
        return

    BACKUP_DIR.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    backup = BACKUP_DIR / f"{path.stem}.dae.bak.{stamp}"
    shutil.copy2(path, backup)
    path.write_text(updated, encoding="utf-8")
    print(f"  backup -> {backup}")


if __name__ == "__main__":
    sys.exit(main())
