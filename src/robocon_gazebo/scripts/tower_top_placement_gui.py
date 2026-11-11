#!/usr/bin/python3
"""RC2027 tower-top placement GUI and Gazebo spawner.

The placement geometry follows rule V3, sections 2.13 and 4.1.4, and the four
published examples in figures 6 and 7:

* 12 tower tops on a 5 x 5 checkerboard;
* the centre cell is empty;
* every block uses the same model: red on +x, blue on -x;
* 200 x 200 x 200 mm and 80-150 g.

方位约定（与规则图 6/7 的俯视图一致：红半场画在左边、蓝半场画在右边）:

    N = 图上方 = 世界 +x        S = 图下方 = 世界 -x
    W = 图左方 = 世界 +y(红半场) E = 图右方 = 世界 -y(蓝半场)

同一模型的局部 +x 为红半面、-x 为蓝半面；各实例按规则图旋转。

The GUI selects one of the four rule layouts, previews the result, and creates every tower top as an
independent dynamic Gazebo entity.
"""

from __future__ import annotations

import argparse
import json
import math
import random
import signal
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional

import rclpy
import tkinter as tk
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from rclpy.node import Node


TOWER_TOP_MASS_KG = 0.115
TOWER_TOP_MODEL = Path(get_package_share_directory("robocon_gazebo")) / "models/robocon_tower_top/model.sdf"
GRID_CENTER_X = -4.25
GRID_CENTER_Y = 0.0
GRID_STEP_M = 0.24
GRID_HALF_SPAN_M = 2 * GRID_STEP_M
TOWER_COUNT = 12

# Each entry is the direction the red half of the upward face points, in the
# Rule-figure convention: N = 图上(+x), S = 图下(-x), W = 图左(+y), E = 图右(-y).
# The four matrices are the layouts of rule V3 figures 6 and 7 (left/middle/right).
RULE_LAYOUTS: tuple[tuple[tuple[str | None, ...], ...], ...] = (
    # 图 6
    (
        ("N", None, "N", None, "S"),
        (None, "E", None, "W", None),
        ("S", None, None, None, "N"),
        (None, "W", None, "E", None),
        ("N", None, "S", None, "S"),
    ),
    # 图 7 · 左
    (
        ("E", None, "W", None, "E"),
        (None, "N", None, "S", None),
        ("W", None, None, None, "E"),
        (None, "S", None, "N", None),
        ("W", None, "E", None, "W"),
    ),
    # 图 7 · 中（外圈红蓝方向与外圈相邻格交替，旧版有 6 格写反）
    (
        ("W", None, "E", None, "W"),
        (None, "N", None, "S", None),
        ("W", None, None, None, "E"),
        (None, "S", None, "N", None),
        ("E", None, "W", None, "E"),
    ),
    # 图 7 · 右（中间两行的左右方向与上下行相反，旧版有 4 格写反）
    (
        ("S", None, "N", None, "N"),
        (None, "W", None, "E", None),
        ("N", None, None, None, "S"),
        (None, "E", None, "W", None),
        ("S", None, "S", None, "N"),
    ),
)

LAYOUT_NAMES = ("图 6", "图 7 · 左", "图 7 · 中", "图 7 · 右")
RED = "#df1717"
BLUE = "#1717dc"
PANEL = "#f1efe8"
CELL = "#c9c9c6"
EMPTY = "#353535"
INK = "#f5f2e9"
MUTED = "#b9b5aa"
ACCENT = "#d89b35"


@dataclass(frozen=True)
class TowerTopPlacement:
    index: int
    row: int
    col: int
    orientation: str
    mass_kg: float
    x: float
    y: float
    z: float = 0.101
    yaw: float = 0.0

    @property
    def name(self) -> str:
        return f"rc2027_tower_top_{self.index:02d}"


@dataclass(frozen=True)
class PlacementPlan:
    seed: int
    layout_index: int
    placements: tuple[TowerTopPlacement, ...]

    @property
    def layout_name(self) -> str:
        return LAYOUT_NAMES[self.layout_index]

    @property
    def total_mass_kg(self) -> float:
        return sum(item.mass_kg for item in self.placements)


def orientation_yaw(orientation: str) -> float:
    """Rotate the one model so its +x red half faces the GUI direction."""
    return {"N": 0.0, "W": math.pi / 2, "S": math.pi, "E": -math.pi / 2}[orientation]


def make_plan(
    layout_index: Optional[int] = None,
    seed: Optional[int] = None,
) -> PlacementPlan:
    seed = int(time.time_ns() & 0xFFFFFFFF) if seed is None else int(seed)
    rng = random.Random(seed)
    if layout_index is None:
        layout_index = rng.randrange(len(RULE_LAYOUTS))
    if not 0 <= layout_index < len(RULE_LAYOUTS):
        raise ValueError(f"layout_index must be 0..{len(RULE_LAYOUTS) - 1}")

    placements: list[TowerTopPlacement] = []
    for row, matrix_row in enumerate(RULE_LAYOUTS[layout_index]):
        for col, orientation in enumerate(matrix_row):
            if orientation is None:
                continue
            index = len(placements)
            placements.append(
                TowerTopPlacement(
                    index=index,
                    row=row,
                    col=col,
                    orientation=orientation,
                    mass_kg=TOWER_TOP_MASS_KG,
                    x=GRID_CENTER_X + GRID_HALF_SPAN_M - row * GRID_STEP_M,
                    y=GRID_CENTER_Y + GRID_HALF_SPAN_M - col * GRID_STEP_M,
                    yaw=orientation_yaw(orientation),
                )
            )

    if len(placements) != TOWER_COUNT:
        raise AssertionError(f"rule layout has {len(placements)} tower tops")
    if any(item.row == 2 and item.col == 2 for item in placements):
        raise AssertionError("the centre checkerboard cell must remain empty")
    return PlacementPlan(seed, layout_index, tuple(placements))


class PlacementGui:
    def __init__(self, auto_spawn: bool = True) -> None:
        rclpy.init()
        self.node = Node("rc2027_tower_top_placement")
        self.spawn_client = self.node.create_client(SpawnEntity, "/spawn_entity")
        self.delete_client = self.node.create_client(DeleteEntity, "/delete_entity")
        self.model_xml = TOWER_TOP_MODEL.read_text(encoding="utf-8")

        self.root = tk.Tk()
        self.root.title("RC2027 塔顶随机摆放")
        self.root.geometry("900x660")
        self.root.minsize(820, 620)
        self.root.configure(bg="#242422")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.plan = make_plan()
        self._pending_initial_plan: Optional[PlacementPlan] = (
            self.plan if auto_spawn else None
        )
        self._apply_plan: Optional[PlacementPlan] = None
        self._phase = "idle"
        self._delete_index = 0
        self._spawn_index = 0
        self._future = None
        self._future_started = 0.0
        self._future_kind = ""
        self._future_entity = ""
        self._closed = False
        self.action_buttons: list[tk.Button] = []

        self._build_ui()
        self._draw_preview()
        self.root.after(80, self._spin)

    def _build_ui(self) -> None:
        header = tk.Frame(self.root, bg="#171716", height=78)
        header.pack(fill="x")
        header.pack_propagate(False)
        tk.Label(
            header,
            text="RC2027  塔顶随机摆放",
            bg="#171716",
            fg=INK,
            font=("Noto Sans CJK SC", 20, "bold"),
        ).pack(anchor="w", padx=24, pady=(12, 0))
        tk.Label(
            header,
            text="5 × 5 交替棋盘  ·  12 个动态塔顶  ·  中心格空置  ·  共用同一个模型",
            bg="#171716",
            fg=MUTED,
            font=("Noto Sans CJK SC", 10),
        ).pack(anchor="w", padx=25, pady=(2, 10))

        body = tk.Frame(self.root, bg="#242422")
        body.pack(fill="both", expand=True, padx=20, pady=18)

        preview_card = tk.Frame(
            body, bg="#30302e", highlightbackground="#494945", highlightthickness=1
        )
        preview_card.pack(side="left", fill="both", expand=True)
        self.canvas = tk.Canvas(
            preview_card,
            width=440,
            height=440,
            bg="#30302e",
            highlightthickness=0,
        )
        self.canvas.pack(fill="both", expand=True, padx=10, pady=10)

        controls = tk.Frame(body, bg="#242422", width=300)
        controls.pack(side="right", fill="y", padx=(18, 0))
        controls.pack_propagate(False)

        self.layout_label = tk.Label(
            controls,
            text="",
            bg="#242422",
            fg=INK,
            font=("Noto Sans CJK SC", 12, "bold"),
            justify="left",
            anchor="w",
        )
        self.layout_label.pack(fill="x", pady=(2, 12))

        self._button(controls, "按规则随机生成", self._random_clicked, primary=True)
        tk.Label(
            controls,
            text="指定规则图中的布局",
            bg="#242422",
            fg=MUTED,
            font=("Noto Sans CJK SC", 9),
        ).pack(anchor="w", pady=(14, 5))
        grid = tk.Frame(controls, bg="#242422")
        grid.pack(fill="x")
        self.layout_buttons: list[tk.Button] = []
        for index, label in enumerate(("1 图 6", "2 图 7 左", "3 图 7 中", "4 图 7 右")):
            button = self._button(
                grid,
                label,
                lambda i=index: self._layout_clicked(i),
                pack=False,
            )
            button.grid(row=index // 2, column=index % 2, sticky="ew", padx=2, pady=3)
            self.layout_buttons.append(button)
        grid.columnconfigure(0, weight=1)
        grid.columnconfigure(1, weight=1)
        tk.Label(
            controls,
            text="快捷键  1-4 选布局 · R 随机 · C 清空",
            bg="#242422",
            fg="#7f7f77",
            font=("Noto Sans CJK SC", 9),
        ).pack(anchor="w", pady=(6, 0))

        self._button(controls, "清空动态塔顶", self._clear_clicked)

        rule = tk.LabelFrame(
            controls,
            text=" 规则约束 ",
            bg="#242422",
            fg=MUTED,
            bd=0,
            font=("Noto Sans CJK SC", 9),
        )
        rule.pack(fill="x", pady=(18, 8), ipady=5)
        tk.Label(
            rule,
            text="尺寸  200 × 200 × 200 mm\n"
            "质量  115 g（统一模型）\n"
            "数量  12 个，塔顶某侧面向上",
            bg="#242422",
            fg=INK,
            justify="left",
            font=("Noto Sans CJK SC", 10),
        ).pack(anchor="w", padx=8, pady=5)

        self.verify_var = tk.StringVar(value="")
        self.verify_label = tk.Label(
            controls,
            textvariable=self.verify_var,
            bg="#242422",
            fg="#5fbf6a",
            justify="left",
            anchor="w",
            font=("Noto Sans CJK SC", 9),
        )
        self.verify_label.pack(fill="x", pady=(2, 6))

        self.status_var = tk.StringVar(value="等待 Gazebo 服务…")
        self.status_label = tk.Label(
            controls,
            textvariable=self.status_var,
            bg="#242422",
            fg=ACCENT,
            justify="left",
            anchor="nw",
            wraplength=285,
            font=("Noto Sans CJK SC", 9),
        )
        self.status_label.pack(fill="x", pady=(8, 0))

        # 快捷键：1-4 选规则布局、R 随机、C 清空（焦点在输入控件时不拦截）
        for index in range(len(RULE_LAYOUTS)):
            self.root.bind(str(index + 1), lambda _e, i=index: self._layout_clicked(i))
        for key in ("<Key-r>", "<Key-R>"):
            self.root.bind(key, lambda _e: self._random_clicked())
        for key in ("<Key-c>", "<Key-C>"):
            self.root.bind(key, lambda _e: self._clear_clicked())

    def _button(
        self,
        parent: tk.Widget,
        text: str,
        command,
        primary: bool = False,
        pack: bool = True,
    ) -> tk.Button:
        button = tk.Button(
            parent,
            text=text,
            command=command,
            bg=ACCENT if primary else "#3b3b38",
            fg="#161616" if primary else INK,
            activebackground="#e5b45b" if primary else "#4a4a46",
            activeforeground="#161616",
            relief="flat",
            bd=0,
            padx=12,
            pady=8,
            cursor="hand2",
            font=("Noto Sans CJK SC", 10, "bold" if primary else "normal"),
        )
        if pack:
            button.pack(fill="x", pady=3)
        self.action_buttons.append(button)
        return button

    def _draw_preview(self) -> None:
        canvas = self.canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 440)
        height = max(canvas.winfo_height(), 440)
        size = min(width - 130, height - 110, 360)
        cell = size / 5.0
        left = (width - size) / 2.0
        top = 66.0

        canvas.create_text(
            width / 2,
            18,
            text="第一公共区俯视预览",
            fill=INK,
            font=("Noto Sans CJK SC", 11, "bold"),
        )
        canvas.create_text(
            width / 2,
            38,
            text="上 N（+x）   下 S（−x）   左 W（+y）   右 E（−y）",
            fill=MUTED,
            font=("Noto Sans CJK SC", 8),
        )
        canvas.create_text(
            width / 2,
            height - 10,
            text="红半面方向对应 Gazebo 中该模型局部 +x 的朝向",
            fill="#7f7f77",
            font=("Noto Sans CJK SC", 8),
        )

        canvas.create_rectangle(left - 31, top, left, top + size, fill="#efb5b5", outline="")
        canvas.create_rectangle(left + size, top, left + size + 31, top + size, fill="#b4d2e6", outline="")
        canvas.create_text(
            left - 15,
            top + size / 2,
            text="红\n半\n场",
            fill="#9e1c1c",
            font=("Noto Sans CJK SC", 9, "bold"),
        )
        canvas.create_text(
            left + size + 16,
            top + size / 2,
            text="蓝\n半\n场",
            fill="#183d72",
            font=("Noto Sans CJK SC", 9, "bold"),
        )
        canvas.create_rectangle(left, top, left + size, top + size, fill=PANEL, outline="#8f8d84")

        # 行/列刻度 1-5：与规则图逐格核对时按 (行, 列) 报位置
        for i in range(5):
            canvas.create_text(
                left + (i + 0.5) * cell, top - 8, text=str(i + 1),
                fill="#6f6d65", font=("Noto Sans CJK SC", 7),
            )
            canvas.create_text(
                left - 40, top + (i + 0.5) * cell, text=str(i + 1),
                fill="#6f6d65", font=("Noto Sans CJK SC", 7),
            )

        for row in range(5):
            for col in range(5):
                x0 = left + col * cell
                y0 = top + row * cell
                fill = EMPTY if (row, col) == (2, 2) else CELL
                canvas.create_rectangle(
                    x0 + 1,
                    y0 + 1,
                    x0 + cell - 1,
                    y0 + cell - 1,
                    fill=fill,
                    outline="",
                )

        canvas.create_text(
            left + 2.5 * cell,
            top + 2.5 * cell,
            text="空置",
            fill="#d8d5cd",
            font=("Noto Sans CJK SC", 8),
        )

        margin = cell * 0.16
        for item in self.plan.placements:
            cx = left + (item.col + 0.5) * cell
            cy = top + (item.row + 0.5) * cell
            half = (cell - 2 * margin) / 2.0
            x0, y0 = cx - cell / 2 + margin, cy - cell / 2 + margin
            x1, y1 = cx + cell / 2 - margin, cy + cell / 2 - margin
            if item.orientation == "N":
                red_box = (x0, y0, x1, y0 + half)
                blue_box = (x0, y0 + half, x1, y1)
            elif item.orientation == "S":
                red_box = (x0, y0 + half, x1, y1)
                blue_box = (x0, y0, x1, y0 + half)
            elif item.orientation == "W":
                red_box = (x0, y0, x0 + half, y1)
                blue_box = (x0 + half, y0, x1, y1)
            else:
                red_box = (x0 + half, y0, x1, y1)
                blue_box = (x0, y0, x0 + half, y1)
            canvas.create_rectangle(*red_box, fill=RED, outline="#702020")
            canvas.create_rectangle(*blue_box, fill=BLUE, outline="#202070")

        self._highlight_layout()
        self.layout_label.configure(
            text=f"当前布局：{self.plan.layout_name}\n"
            f"12 块  ·  合计 {self.plan.total_mass_kg * 1000:.0f} g\n"
            f"随机种子 {self.plan.seed}"
        )
        self._refresh_verify()

    def _highlight_layout(self) -> None:
        """当前布局按钮高亮，避免看错选了哪张规则图。"""
        for index, button in enumerate(getattr(self, "layout_buttons", [])):
            active = index == self.plan.layout_index
            button.configure(
                bg=ACCENT if active else "#3b3b38",
                fg="#161616" if active else INK,
                activebackground="#e5b45b" if active else "#4a4a46",
            )

    def _refresh_verify(self) -> None:
        placed = len(self.plan.placements)
        centre_empty = all(
            not (item.row == 2 and item.col == 2) for item in self.plan.placements
        )
        good = placed == TOWER_COUNT and centre_empty
        self.verify_var.set(
            ("✓ " if good else "✗ ")
            + f"{placed} 块 · 中心格空置"
            + (" · 按布局旋转" if good else " · 请检查布局")
        )
        self.verify_label.configure(fg="#5fbf6a" if good else "#d75b52")

    def _random_clicked(self) -> None:
        self._request_plan(make_plan())

    def _layout_clicked(self, index: int) -> None:
        self._request_plan(make_plan(layout_index=index))

    def _request_plan(self, plan: PlacementPlan) -> None:
        self.plan = plan
        self._draw_preview()
        if not self._services_ready():
            self.status_var.set("已预览，等待 Gazebo 的生成服务…")
            self._pending_initial_plan = plan
            return
        self._start_replace(plan)

    def _clear_clicked(self) -> None:
        if not self.delete_client.service_is_ready():
            self.status_var.set("删除服务尚未就绪。")
            return
        self._pending_initial_plan = None
        self._start_replace(None)

    def _services_ready(self) -> bool:
        return (
            self.spawn_client.service_is_ready()
            and self.delete_client.service_is_ready()
        )

    def _set_busy(self, busy: bool) -> None:
        state = "disabled" if busy else "normal"
        for button in self.action_buttons:
            button.configure(state=state)

    def _start_replace(self, plan: Optional[PlacementPlan]) -> None:
        if self._phase != "idle":
            return
        self._pending_initial_plan = None
        self._apply_plan = plan
        self._delete_index = 0
        self._spawn_index = 0
        self._phase = "delete"
        self._set_busy(True)
        self.status_var.set("正在清除上一组动态塔顶…")
        self._advance_delete()

    def _advance_delete(self) -> None:
        if self._delete_index >= TOWER_COUNT:
            if self._apply_plan is None:
                self._finish()
                self.status_var.set("场地中已无 GUI 生成的动态塔顶。")
                return
            self._phase = "spawn"
            self.status_var.set("正在生成 12 个独立动态塔顶…")
            self._advance_spawn()
            return

        name = f"rc2027_tower_top_{self._delete_index:02d}"
        self._delete_index += 1
        request = DeleteEntity.Request()
        request.name = name
        self._call_future("delete", name, self.delete_client.call_async(request))

    def _advance_spawn(self) -> None:
        plan = self._apply_plan
        if plan is None or self._spawn_index >= len(plan.placements):
            self._finish()
            if plan is not None:
                self.status_var.set(
                    f"已生成 {len(plan.placements)} 个塔顶 · {plan.layout_name} · "
                    f"{plan.total_mass_kg * 1000:.0f} g"
                )
            return

        item = plan.placements[self._spawn_index]
        self._spawn_index += 1
        request = SpawnEntity.Request()
        request.name = item.name
        request.xml = self.model_xml
        request.robot_namespace = ""
        request.reference_frame = "world"
        request.initial_pose.position.x = item.x
        request.initial_pose.position.y = item.y
        request.initial_pose.position.z = item.z
        request.initial_pose.orientation.z = math.sin(item.yaw / 2.0)
        request.initial_pose.orientation.w = math.cos(item.yaw / 2.0)
        self._call_future("spawn", item.name, self.spawn_client.call_async(request))
        self.status_var.set(
            f"生成中 {self._spawn_index}/{len(plan.placements)}：{item.name}"
        )

    def _call_future(self, kind: str, entity: str, future) -> None:
        self._future = future
        self._future_kind = kind
        self._future_entity = entity
        self._future_started = time.monotonic()

    def _spin(self) -> None:
        if self._closed:
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except Exception as exc:  # keep the UI alive long enough to report errors
            self.status_var.set(f"ROS 通信异常：{exc}")

        if self._future is not None:
            if self._future.done():
                self._consume_future()
            elif time.monotonic() - self._future_started > 8.0:
                kind = self._future_kind
                entity = self._future_entity
                self._future = None
                self._fail(f"{kind} {entity} 超时。")

        if (
            self._phase == "idle"
            and self._pending_initial_plan is not None
            and self._services_ready()
        ):
            self._start_replace(self._pending_initial_plan)

        self.root.after(30, self._spin)

    def _consume_future(self) -> None:
        future = self._future
        kind = self._future_kind
        entity = self._future_entity
        self._future = None
        try:
            response = future.result()
        except Exception as exc:
            self._fail(f"{kind} {entity} 调用失败：{exc}")
            return

        success = bool(getattr(response, "success", False))
        message = str(getattr(response, "status_message", ""))
        if kind == "delete":
            # A failed delete normally means this name did not exist yet.
            self._advance_delete()
        elif success:
            self._advance_spawn()
        else:
            self._fail(f"生成 {entity} 失败：{message or 'Gazebo 未返回原因'}")

    def _finish(self) -> None:
        self._future = None
        self._phase = "idle"
        self._apply_plan = None
        self._set_busy(False)

    def _fail(self, message: str) -> None:
        self._finish()
        self.status_var.set(message)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self.node.destroy_node()
            rclpy.shutdown()
        finally:
            self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--print-plan", action="store_true", help="print a plan and exit")
    parser.add_argument(
        "--preview-only",
        action="store_true",
        help="open the GUI without automatically spawning the initial layout",
    )
    parser.add_argument("--layout", type=int, choices=range(4), help="rule layout index")
    parser.add_argument("--seed", type=int, help="random seed for reproducible plans")
    args, _ = parser.parse_known_args()

    if args.print_plan:
        plan = make_plan(args.layout, args.seed)
        print(json.dumps(asdict(plan), ensure_ascii=False, indent=2))
        return

    if not sys.platform.startswith("linux"):
        raise SystemExit("This GUI requires a desktop Linux session with X11.")
    app = PlacementGui(auto_spawn=not args.preview_only)
    signal.signal(signal.SIGINT, lambda *_: app.root.after(0, app.close))
    signal.signal(signal.SIGTERM, lambda *_: app.root.after(0, app.close))
    app.run()


if __name__ == "__main__":
    main()
