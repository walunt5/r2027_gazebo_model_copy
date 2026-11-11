#!/usr/bin/python3
"""塔顶布局自检：检查 GUI 格位和统一的 Gazebo 红蓝模型。

用法:
    /usr/bin/python3 src/robocon_gazebo/scripts/tests/test_rule_layouts.py
"""
from __future__ import annotations

import ast
import math
import os
import random
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Optional

HERE = os.path.dirname(os.path.abspath(__file__))
GUI = os.path.normpath(
    os.path.join(HERE, "..", "tower_top_placement_gui.py")
)

with open(GUI, encoding="utf-8") as fh:
    tree = ast.parse(fh.read(), GUI)

wanted = {"RULE_LAYOUTS", "LAYOUT_NAMES", "TOWER_COUNT", "TOWER_TOP_MASS_KG", "GRID_CENTER_X", "GRID_CENTER_Y", "GRID_STEP_M", "GRID_HALF_SPAN_M"}
ns: dict[str, object] = {"math": math, "random": random, "time": time, "dataclass": dataclass, "Optional": Optional}
for node in tree.body:
    if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
        if node.target.id in wanted:
            exec(compile(ast.Module(body=[node], type_ignores=[]), GUI, "exec"), ns)
    elif isinstance(node, ast.Assign):
        for target in node.targets:
            if isinstance(target, ast.Name) and target.id in wanted:
                exec(compile(ast.Module(body=[node], type_ignores=[]), GUI, "exec"), ns)
    elif isinstance(node, ast.ClassDef) and node.name in {"TowerTopPlacement", "PlacementPlan"}:
        exec(compile(ast.Module(body=[node], type_ignores=[]), GUI, "exec"), ns)
    elif isinstance(node, ast.FunctionDef) and node.name in {"make_plan", "orientation_yaw"}:
        exec(compile(ast.Module(body=[node], type_ignores=[]), GUI, "exec"), ns)

LAYOUTS = ns["RULE_LAYOUTS"]
NAMES = ns["LAYOUT_NAMES"]
TOWER_COUNT = ns["TOWER_COUNT"]

# 四张用户俯视图逐格抄录：字母是红半面的方向，点为空格。
REFERENCE_ROWS = (
    ("N.N.S", ".E.W.", "S...N", ".W.E.", "N.S.S"),
    ("E.W.E", ".N.S.", "W...E", ".S.N.", "W.E.W"),
    ("W.E.W", ".N.S.", "W...E", ".S.N.", "E.W.E"),
    ("S.N.N", ".W.E.", "N...S", ".E.W.", "S.S.N"),
)

ok = True


def check(label: str, cond: bool, extra: str = "") -> None:
    global ok
    print(("  PASS  " if cond else "  FAIL  ") + label + (f" | {extra}" if extra else ""))
    ok = ok and bool(cond)


# 顶视图字符画：每格 2x2 字符, R=红面 B=蓝面
TOP = {"N": ("RR", "BB"), "S": ("BB", "RR"), "W": ("RB", "RB"), "E": ("BR", "BR")}


def render(matrix) -> list[str]:
    out: list[str] = []
    for row in matrix:
        top_line = " ".join(TOP[o][0] if o else ".." for o in row)
        bot_line = " ".join(TOP[o][1] if o else ".." for o in row)
        out += [top_line, bot_line, ""]
    return out


for index, matrix in enumerate(LAYOUTS):
    print(f"\n== 布局 {index} · {NAMES[index]} ==")
    for line in render(matrix):
        print("   " + line)
    filled = [o for row in matrix for o in row if o]
    check(f"布局{index + 1} 与第{index + 1}张参考图逐格一致",
          tuple("".join(value or "." for value in row) for row in matrix) == REFERENCE_ROWS[index])
    check(f"布局{index} 数量为 {TOWER_COUNT}", len(filled) == TOWER_COUNT, str(len(filled)))
    check(f"布局{index} 中心格空置", matrix[2][2] is None)
    check(f"布局{index} 方向值合法", set(filled) <= set("NSEW"), str(sorted(set(filled))))

# 所有实例读取同一个模型文件，只通过初始位姿旋转。
model = ET.parse(os.path.normpath(os.path.join(HERE, "../../models/robocon_tower_top/model.sdf")))
red_pose = model.find(".//visual[@name='red_half']/pose")
blue_pose = model.find(".//visual[@name='blue_half']/pose")
check("同一模型红 +x、蓝 -x", float(red_pose.text.split()[0]) == 0.05 and float(blue_pose.text.split()[0]) == -0.05)
check("模型质量为 115 g", float(model.find(".//inertial/mass").text) == ns["TOWER_TOP_MASS_KG"])
EXPECT = {"N": (1, 0), "S": (-1, 0), "W": (0, 1), "E": (0, -1)}
for layout_index, matrix in enumerate(LAYOUTS):
    plan = ns["make_plan"](layout_index=layout_index, seed=2027)
    for item in plan.placements:
        row = round((ns["GRID_CENTER_X"] + ns["GRID_HALF_SPAN_M"] - item.x) / ns["GRID_STEP_M"])
        col = round((ns["GRID_CENTER_Y"] + ns["GRID_HALF_SPAN_M"] - item.y) / ns["GRID_STEP_M"])
        direction = (round(math.cos(item.yaw)), round(math.sin(item.yaw)))
        check(f"布局{layout_index} 格({item.row + 1},{item.col + 1}) 与 Gazebo 坐标和朝向一致",
              (row, col) == (item.row, item.col) and matrix[row][col] == item.orientation
              and abs(item.x - (ns["GRID_CENTER_X"] + ns["GRID_HALF_SPAN_M"] - item.row * ns["GRID_STEP_M"])) < 1e-9
              and abs(item.y - (ns["GRID_CENTER_Y"] + ns["GRID_HALF_SPAN_M"] - item.col * ns["GRID_STEP_M"])) < 1e-9
              and direction == EXPECT[item.orientation] and item.mass_kg == ns["TOWER_TOP_MASS_KG"])

print("\nRESULT:", "ALL PASS" if ok else "FAILURES PRESENT")
sys.exit(0 if ok else 1)
