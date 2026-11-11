#!/usr/bin/python3
"""Spawn reusable red/blue tower-base models in four marked field regions."""

from __future__ import annotations

import re
import signal
import time
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import GetModelList, SpawnEntity
from rclpy.node import Node


MODEL_DIR = Path(get_package_share_directory("robocon_gazebo")) / "models"
BLOCK_SIZE = 0.35
GRID_STEP = 0.45
MODEL_PREFIX = "rc2027_tower_base"
NAME_RE = re.compile(r"^rc2027_tower_base_p([1-4])_s(\d{2})_l([01])_(red|blue)$")


@dataclass(frozen=True)
class Region:
    label: str
    x: float
    y: float
    surface_z: float
    rows: int
    cols: int

    @property
    def capacity(self) -> int:
        return self.rows * self.cols * 2


# 俯视图上方 +x、左方 +y。位置 3/4 的台面来自场地 SDF 中的 1×1 m L1 传递区。
REGIONS = {
    1: Region("左上 · 地面", 5.0, 4.5, 0.0, 2, 4),
    2: Region("右上 · 地面", 5.0, -4.5, 0.0, 2, 4),
    3: Region("左下 · L1", -1.3, 3.5, 0.6, 2, 2),
    4: Region("右下 · L1", -1.3, -3.5, 0.6, 2, 2),
}


def placement_pose(region_number: int, index: int) -> tuple[float, float, float]:
    """Return the centre of one block; two consecutive indices share a cell."""
    region = REGIONS[region_number]
    if not 0 <= index < region.capacity:
        raise ValueError("region is full")
    slot, layer = divmod(index, 2)
    row, col = divmod(slot, region.cols)
    x = region.x + (region.rows - 1) * GRID_STEP / 2 - row * GRID_STEP
    y = region.y + (region.cols - 1) * GRID_STEP / 2 - col * GRID_STEP
    z = region.surface_z + BLOCK_SIZE / 2 + layer * BLOCK_SIZE
    return round(x, 4), round(y, 4), round(z, 4)


def entity_name(region_number: int, index: int, color: str) -> str:
    slot, layer = divmod(index, 2)
    return f"{MODEL_PREFIX}_p{region_number}_s{slot:02d}_l{layer}_{color}"


def occupied_indices(model_names: list[str] | tuple[str, ...], region_number: int) -> set[int]:
    """Count both colors together and avoid filling beneath an existing upper block."""
    occupied: set[int] = set()
    for name in model_names:
        match = NAME_RE.fullmatch(name)
        if match and int(match.group(1)) == region_number:
            slot, layer = int(match.group(2)), int(match.group(3))
            if slot < REGIONS[region_number].rows * REGIONS[region_number].cols:
                occupied.add(slot * 2 + layer)
                if layer == 1:
                    occupied.add(slot * 2)
    return occupied


def next_index(model_names: list[str] | tuple[str, ...], region_number: int) -> int | None:
    occupied = occupied_indices(model_names, region_number)
    return next((i for i in range(REGIONS[region_number].capacity) if i not in occupied), None)


class TowerBaseGui:
    def __init__(self) -> None:
        rclpy.init()
        self.node = Node("rc2027_tower_base_placement")
        self.list_client = self.node.create_client(GetModelList, "/get_model_list")
        self.spawn_client = self.node.create_client(SpawnEntity, "/spawn_entity")
        self.model_xml = {
            color: (MODEL_DIR / f"robocon_tower_base_{color}" / "model.sdf").read_text(encoding="utf-8")
            for color in ("red", "blue")
        }
        self.root = tk.Tk()
        self.root.title("RC2027 地基块生成")
        self.root.geometry("420x390")
        self.root.resizable(False, False)
        self.root.configure(bg="#262624")
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.color = tk.StringVar(value="red")
        self.region_number = tk.IntVar(value=1)
        self.status = tk.StringVar(value="等待 Gazebo 服务…")
        self.occupancy = tk.StringVar(value="")
        self._future = None
        self._future_kind = ""
        self._started = 0.0
        self._pending_name = ""
        self._model_names: tuple[str, ...] | None = None
        self._next_refresh_at = 0.0
        self._closed = False
        self._build_ui()
        self.root.after(60, self._spin)

    def _build_ui(self) -> None:
        bg, fg = "#262624", "#f4f2eb"
        tk.Label(self.root, text="地基块生成", bg=bg, fg=fg,
                 font=("Noto Sans CJK SC", 17, "bold")).pack(anchor="w", padx=20, pady=(16, 4))
        tk.Label(self.root, text="每次生成 1 块 · 每格先叠 2 块，再排下一格",
                 bg=bg, fg="#bab8ae").pack(anchor="w", padx=20)

        colors = tk.Frame(self.root, bg=bg)
        colors.pack(fill="x", padx=20, pady=(14, 8))
        tk.Label(colors, text="颜色", bg=bg, fg=fg).pack(side="left", padx=(0, 15))
        for value, label, tint in (("red", "红色", "#df2222"), ("blue", "蓝色", "#3200ff")):
            tk.Radiobutton(colors, text=label, variable=self.color, value=value,
                           bg=bg, fg=tint, selectcolor="#444440", activebackground=bg,
                           activeforeground=tint).pack(side="left", padx=(0, 25))

        tk.Label(self.root, text="位置（上 +x · 左 +y）", bg=bg, fg=fg).pack(anchor="w", padx=20)
        positions = tk.Frame(self.root, bg=bg)
        positions.pack(fill="x", padx=20, pady=(6, 8))
        for number, region in REGIONS.items():
            button = tk.Radiobutton(
                positions, text=f"{number}  {region.label}\n最多 {region.capacity} 块",
                variable=self.region_number, value=number, command=self._update_occupancy,
                bg="#3b3b38", fg=fg, selectcolor="#6c5936", activebackground="#4c4c48",
                activeforeground=fg, indicatoron=False, width=18, height=3,
            )
            button.grid(row=(number - 1) // 2, column=(number - 1) % 2,
                        sticky="ew", padx=3, pady=3)
        positions.columnconfigure(0, weight=1)
        positions.columnconfigure(1, weight=1)

        tk.Label(self.root, textvariable=self.occupancy, bg=bg, fg="#b7d3df").pack(anchor="w", padx=20)
        self.spawn_button = tk.Button(self.root, text="生成 1 个地基块", command=self._spawn_clicked,
                                      bg="#d89b35", fg="#181818", relief="flat",
                                      font=("Noto Sans CJK SC", 12, "bold"))
        self.spawn_button.pack(fill="x", padx=20, pady=(12, 6), ipady=6)
        tk.Label(self.root, textvariable=self.status, bg=bg, fg="#e6c17f",
                 wraplength=380, justify="left").pack(anchor="w", padx=20)

    def _update_occupancy(self) -> None:
        if self._model_names is None:
            self.occupancy.set("正在读取已有实例…")
            return
        number = self.region_number.get()
        occupied = occupied_indices(self._model_names, number)
        next_block = next_index(self._model_names, number)
        self.occupancy.set(
            f"位置 {number}：已占 {len(occupied)}/{REGIONS[number].capacity} 个层位"
            + (" · 已满" if next_block is None else f" · 下一块为第 {next_block + 1} 块")
        )

    def _call(self, kind: str, future) -> None:
        self._future = future
        self._future_kind = kind
        self._started = time.monotonic()
        self.spawn_button.configure(state="disabled")

    def _request_list(self, kind: str) -> None:
        if not self.list_client.service_is_ready():
            self.status.set("Gazebo 模型列表服务尚未就绪。")
            return
        self._call(kind, self.list_client.call_async(GetModelList.Request()))

    def _spawn_clicked(self) -> None:
        if self._future is not None:
            return
        if not self.spawn_client.service_is_ready():
            self.status.set("Gazebo 生成服务尚未就绪。")
            return
        self.status.set("正在核对 Gazebo 中已有的地基块…")
        self._request_list("before_spawn")

    def _spawn_from_list(self) -> None:
        number = self.region_number.get()
        color = self.color.get()
        index = next_index(self._model_names, number)
        if index is None:
            self.status.set(f"位置 {number} 已满，不能再生成。")
            return
        x, y, z = placement_pose(number, index)
        name = entity_name(number, index, color)
        request = SpawnEntity.Request()
        request.name = name
        request.xml = self.model_xml[color]
        request.robot_namespace = ""
        request.reference_frame = "world"
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = z
        request.initial_pose.orientation.w = 1.0
        self._pending_name = name
        self.status.set(f"正在生成 {name} 于 ({x}, {y}, {z})…")
        self._call("spawn", self.spawn_client.call_async(request))

    def _consume_future(self) -> None:
        future, kind = self._future, self._future_kind
        self._future = None
        try:
            response = future.result()
        except Exception as exc:
            self.status.set(f"Gazebo 服务调用失败：{exc}")
            return
        if kind in ("initial_list", "before_spawn", "after_spawn"):
            if not response.success:
                self.status.set("无法读取 Gazebo 模型列表。")
                self._next_refresh_at = time.monotonic() + 1.0
                return
            self._model_names = tuple(response.model_names)
            self._update_occupancy()
            if kind == "before_spawn":
                self._spawn_from_list()
            elif kind == "after_spawn":
                self.status.set(f"已生成 {self._pending_name}")
        elif kind == "spawn":
            if response.success:
                self._request_list("after_spawn")
            else:
                self.status.set(f"生成失败：{response.status_message}")

    def _spin(self) -> None:
        if self._closed:
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except Exception as exc:
            self.status.set(f"ROS 通信异常：{exc}")
        if self._future is not None:
            if self._future.done():
                self._consume_future()
            elif time.monotonic() - self._started > 10.0:
                self._future = None
                self._next_refresh_at = time.monotonic() + 1.0
                self.status.set("Gazebo 服务响应超时；再次点击前会重新查询已有实例。")
        elif (self._model_names is None and self.list_client.service_is_ready()
              and time.monotonic() >= self._next_refresh_at):
            self._request_list("initial_list")
        ready = (
            self._future is None and self._model_names is not None
            and self.spawn_client.service_is_ready()
            and self.list_client.service_is_ready()
        )
        self.spawn_button.configure(state="normal" if ready else "disabled")
        self.root.after(40, self._spin)

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
    app = TowerBaseGui()
    signal.signal(signal.SIGINT, lambda *_: app.root.after(0, app.close))
    signal.signal(signal.SIGTERM, lambda *_: app.root.after(0, app.close))
    app.run()


if __name__ == "__main__":
    main()
