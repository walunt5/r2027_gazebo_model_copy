#!/usr/bin/python3
"""Check four marked regions, two-block stacks, and recovered occupancy."""

import runpy
from pathlib import Path


module = runpy.run_path(str(Path(__file__).resolve().parents[1] / "tower_base_placement_gui.py"))
regions = module["REGIONS"]
pose = module["placement_pose"]
name = module["entity_name"]
next_index = module["next_index"]

assert {number: region.capacity for number, region in regions.items()} == {
    1: 16, 2: 16, 3: 8, 4: 8,
}

for number, first, second, third in (
    (1, (5.225, 5.175, 0.175), (5.225, 5.175, 0.525), (5.225, 4.725, 0.175)),
    (2, (5.225, -3.825, 0.175), (5.225, -3.825, 0.525), (5.225, -4.275, 0.175)),
    (3, (-1.075, 3.725, 0.775), (-1.075, 3.725, 1.125), (-1.075, 3.275, 0.775)),
    (4, (-1.075, -3.275, 0.775), (-1.075, -3.275, 1.125), (-1.075, -3.725, 0.775)),
):
    assert pose(number, 0) == first
    assert pose(number, 1) == second
    assert pose(number, 2) == third
    assert next_index([], number) == 0

# Red and blue share the same slots; model names encode region, cell and layer.
existing = [name(3, 0, "red"), name(3, 1, "blue")]
assert next_index(existing, 3) == 2
assert next_index(existing, 4) == 0
assert next_index([name(3, i, "red" if i % 2 else "blue") for i in range(8)], 3) is None

# If the lower block was removed externally, do not spawn beneath an upper block.
assert next_index([name(3, 1, "red")], 3) == 2
assert pose(1, 15)[2] == 0.525
assert pose(3, 7)[2] == 1.125
print("tower-base placement: all checks passed")
