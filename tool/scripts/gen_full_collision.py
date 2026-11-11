#!/usr/bin/env python3
"""从视觉 DAE 自动生成 robocon_ground_full 的 model.sdf (完整碰撞体).

规则:
  - z 向尺寸≈0 的部件是 2D 贴图/标线, 跳过
  - Cube.001 / Cube.006 是楔形坡道, 用 meshes/ramp.obj 做网格碰撞
  - Cube.020~Cube.043 是动态塔顶的两个半面, 不属于场地碰撞
  - Cylinder* 用圆柱原语, 其余 3D 部件用包围盒 box
  - 地面与中央隔板单独手写 (视觉里是平面, 无厚度)

用法:
    /usr/bin/python3 tool/scripts/gen_full_collision.py
"""
import os
import re
import xml.etree.ElementTree as ET
import numpy as np

# 本文件位于 <ws>/tool/scripts/, 上溯三层才是工作区根
WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DAE = os.path.join(WS, 'tool', 'gazebo_models', 'robocon_ground', 'meshes', 'Untitled.dae')
OUT = os.path.join(WS, 'src', 'robocon_gazebo', 'models', 'robocon_ground_full', 'model.sdf')

NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
RAMPS = {'Cube.001': ('ramp_north', '0 3 0 0 0 0'), 'Cube.006': ('ramp_south', '0 -4 0 0 0 0')}
# L1 六段围栏: DAE 中的视觉节点由 strip_fixed_tower_tops.py 删除,
# 这里用与碰撞相同的 box 显式补齐视觉, 避免 Collada 子网格渲染缺失。
RAIL_NODES = {f'Cube.{i:03d}' for i in range(11, 17)}
RAIL_COLOR = '0.128830 0 0.00178466 1'  # L1 台面护栏, 取地面 Boundary_Wall 四色均值(深棕)
# L2 侧面与 L2 顶面同一规则灰（顶面灰片 L2_top_gray 由 set_face_colors.py 写进 DAE）
L2_SIDE_COLOR = '0.745098 0.745098 0.725490 1'
# L2 台体 z 0.6~0.9, 侧面 4 片竖直薄板包边。DAE 里 Cube.017 六面共用同一个红色材质,
# 顶面已用灰片盖住, 侧面不包就会露出一圈红墙裙。
#   (方向, 尺寸, 位置) —— 板长 3.008 = 3 + 2*0.004, 让四角互相盖住不留缝
L2_SIDES = [
    ('n', ('3.008', '0.004', '0.3'), ('0', '1.502', '0.75')),
    ('s', ('3.008', '0.004', '0.3'), ('0', '-1.502', '0.75')),
    ('e', ('0.004', '3.008', '0.3'), ('1.502', '0', '0.75')),
    ('w', ('0.004', '3.008', '0.3'), ('-1.502', '0', '0.75')),
]
# 地面中央隔板的深红棕色, 取自 DAE 中 Material.015 的 diffuse
CENTER_DIVIDER_COLOR = '0.140994 0 0.002095 1'
# L1->L2 之间的两个小台阶 (南北各一), 已从视觉与碰撞中删除
SKIP_NODES = {
    'Cube.018', 'Cube.019',
    # 12 个塔顶各由两个半色网格组成。塔顶由 placement GUI 动态生成,
    # 不能再并入静态场地模型，否则会留下不可删除的“双重塔顶”。
    *(f'Cube.{i:03d}' for i in range(20, 44)),
}


def collect():
    root = ET.parse(DAE).getroot()
    lib_geom = {g.get('id'): g for g in root.findall('.//c:geometry', NS)}
    scene = root.find('.//c:visual_scene', NS)
    out = []

    def geom_bbox(url):
        mesh = lib_geom[url].find('c:mesh', NS)
        pos = mesh.find("c:vertices/c:input[@semantic='POSITION']", NS)
        src = mesh.find("c:source[@id='%s']" % pos.get('source')[1:], NS)
        vals = np.array([float(v) for v in src.find('c:float_array', NS).text.split()]).reshape(-1, 3)
        return vals.min(axis=0), vals.max(axis=0)

    def walk(node, M):
        m = node.find('c:matrix', NS)
        if m is not None:
            M = M @ np.array([float(v) for v in m.text.split()]).reshape(4, 4)
        for ig in node.findall('c:instance_geometry', NS):
            mn, mx = geom_bbox(ig.get('url')[1:])
            c = np.hstack([np.vstack([mn, mx]), np.ones((2, 1))]) @ M.T
            out.append((node.get('name') or ig.get('url'),
                        c[:, :3].min(axis=0), c[:, :3].max(axis=0)))
        for ch in node.findall('c:node', NS):
            walk(ch, M)

    for node in scene.findall('c:node', NS):
        walk(node, np.eye(4))
    return out


def num(x):
    s = f'{float(x):.4f}'.rstrip('0').rstrip('.')
    return s if s else '0'


def num_join(vals):
    """把一组数字拼成 SDF 里空格分隔的串, 如 '2.75 0.08 0.08'."""
    return ' '.join(num(v) for v in vals)


def pose_str(c, rpy=(0, 0, 0)):
    return f'{num(c[0])} {num(c[1])} {num(c[2])} {num(rpy[0])} {num(rpy[1])} {num(rpy[2])}'


def surface(mu=0.8):
    return [
        '        <surface>',
        '          <friction><ode><mu>%s</mu><mu2>%s</mu2></ode></friction>' % (mu, mu),
        '          <contact><ode><max_vel>0.1</max_vel><min_depth>0.001</min_depth></ode></contact>',
        '        </surface>',
    ]


def preserved_films(out_path):
    """保留场地编辑器加上的台面着色层 (visual name="colorfilm_*")。

    着色层是纯视觉薄片, 与本脚本从 DAE 推导的几何无关; 但写回时本脚本会整体
    重写 model.sdf, 不显式搬运就会把用户调好的颜色一起清掉。
    """
    if not os.path.isfile(out_path):
        return []
    text = open(out_path, encoding='utf-8').read()
    pat = re.compile(r'[ \t]*<visual name="colorfilm_[^"]*">.*?</visual>', re.S)
    return [m.rstrip() for m in pat.findall(text)]


def gen():
    assert not RAIL_NODES & SKIP_NODES, '围栏必须保留碰撞, 不能加入 SKIP_NODES'
    parts = collect()
    body = []
    rail_seen = set()

    # Plane.014 (L1 顶面中央隔板) 已按要求整条删除; 若上游把它改成有厚度的
    # 实体网格, 通用循环会自动生成碰撞, 与"删除"意图冲突, 这里提前拦截。
    for name, mn, mx in parts:
        if name == 'Plane.014' and (mx - mn)[2] >= 1e-3:
            raise RuntimeError('上游 Plane.014 不再是平面, 它会经由通用循环生成 L1 隔板碰撞')

    # 地面与地面中央隔板保持原状 (视觉为无厚度平面, 手写)
    body += [
        '      <!-- ground (z=0 顶面) -->',
        '      <collision name="ground">',
        '        <geometry><box><size>11 11 0.02</size></box></geometry>',
        '        <pose>0 0 -0.01 0 0 0</pose>',
    ] + surface() + ['      </collision>']

    # 地面中央隔板拆成 4 段。视觉颜色取 DAE 中 Material.015 的 diffuse（深红棕）;
    # 分段是为了能在场地编辑器里逐段独立调整。
    # 下方数值是用户在场地编辑器里逐段调过的最终结果（commit 40f9532）。
    # 注意碰撞与视觉的尺寸/位置**各自独立**（用户在编辑器里分别调过），
    # 不能合并成同一套数值。
    #   (索引, 碰撞尺寸, 碰撞位置, 视觉尺寸, 视觉位置)
    DIVIDERS = [
        (1, ('0.65', '0.1',   '0.08'), ('-5.175', '0',     '0.04'),
            ('0.65', '0.1',   '0.08'), ('-5.175', '0',     '0.04')),
        (2, ('3.6',  '0.061', '0.08'), ('-1.84',  '0.015', '0.04'),
            ('2.75', '0.08',  '0.08'), ('-2.27',  '0',     '0.04')),
        (3, ('3.65', '0.081', '0.08'), ('1.825',  '0.01',  '0.04'),
            ('3.691','0.1',   '0.08'), ('1.795',  '0.01',  '0.04')),
        (4, ('0.9',  '0.05',  '0.08'), ('5.125',  '0.015', '0.04'),
            ('0.831','0.08',  '0.08'), ('5.07',   '0',     '0.04')),
    ]
    for i, csize, cpos, vsize, vpos in DIVIDERS:
        body += [
            f'      <collision name="center_divider_{i}">',
            f'        <geometry><box><size>{num_join(csize)}</size></box></geometry>',
            f'        <pose>{num_join(cpos)} 0 0 0</pose>',
            *surface(),
            '      </collision>',
            f'      <visual name="center_divider_{i}_visual">',
            f'        <geometry><box><size>{num_join(vsize)}</size></box></geometry>',
            f'        <pose>{num_join(vpos)} 0 0 0</pose>',
            '        <material>',
            f'          <ambient>{CENTER_DIVIDER_COLOR}</ambient>',
            f'          <diffuse>{CENTER_DIVIDER_COLOR}</diffuse>',
            '        </material>',
            '      </visual>',
        ]

    # 用户在编辑器里额外复制了一段视觉（center_divider_2_visual_copy），
    # 只有视觉没有碰撞。同步保留，避免重跑时丢失。
    body += [
        '      <visual name="center_divider_2_visual_copy">',
        '        <geometry><box><size>2.75 0.08 0.08</size></box></geometry>',
        '        <pose>-1.77 0 0.04 0 0 0</pose>',
        '        <material>',
        f'          <ambient>{CENTER_DIVIDER_COLOR}</ambient>',
        f'          <diffuse>{CENTER_DIVIDER_COLOR}</diffuse>',
        '        </material>',
        '      </visual>',
    ]

    # L1 顶面横向围栏 (z=0.625)。碰撞与视觉尺寸/位置已由用户在编辑器里分别调过,
    # 两者不再一致 —— 下面按实际值分别硬编码, 不要合并成同一个 length。
    L1_RAILS = [
        # (side, 碰撞尺寸, 碰撞位置, 视觉尺寸, 视觉位置)
        ('left',  ('1.85', '0.05', '0.05'), ('-1.3255', '0', '0.625'),
                  ('2.1',  '0.05', '0.05'), ('-1.2',    '0', '0.625')),
        ('right', ('2.25', '0.05', '0.05'), ('1.125',   '0', '0.625'),
                  ('1.6',  '0.05', '0.05'), ('1.45',    '0', '0.625')),
    ]
    for side, csize, cpos, vsize, vpos in L1_RAILS:
        body += [
            f'      <collision name="l1_center_divider_{side}">',
            f'        <geometry><box><size>{num_join(csize)}</size></box></geometry>',
            f'        <pose>{num_join(cpos)} 0 0 0</pose>',
            *surface(),
            '      </collision>',
            f'      <visual name="l1_center_divider_{side}_visual">',
            f'        <geometry><box><size>{num_join(vsize)}</size></box></geometry>',
            f'        <pose>{num_join(vpos)} 0 0 0</pose>',
            '        <material>',
            f'          <ambient>{RAIL_COLOR}</ambient>',
            f'          <diffuse>{RAIL_COLOR}</diffuse>',
            '        </material>',
            '      </visual>',
        ]

    n_box = n_cyl = 0
    for name, mn, mx in sorted(parts):
        size = mx - mn
        if name in SKIP_NODES:        # 已删除的台阶
            continue
        if size[2] < 1e-3:            # 2D 标线
            continue
        c = (mn + mx) / 2
        lines = [f'      <!-- {name} -->', f'      <collision name="{name}">']
        if name in RAMPS:             # 坡道 -> 网格碰撞
            lines[1] = f'      <collision name="{RAMPS[name][0]}">'
            lines.append('        <geometry><mesh><uri>meshes/ramp.obj</uri></mesh></geometry>')
            lines.append(f'        <pose>{RAMPS[name][1]}</pose>')
            lines += surface(0.9)
        elif name.startswith('Cylinder'):
            n_cyl += 1
            r = min(size[0], size[1]) / 2
            lines.append('        <geometry><cylinder>'
                         f'<radius>{num(r)}</radius><length>{num(size[2])}</length>'
                         '</cylinder></geometry>')
            lines.append(f'        <pose>{pose_str(c)}</pose>')
            lines += surface()
        else:
            n_box += 1
            size_str = f'{num(size[0])} {num(size[1])} {num(size[2])}'
            pose = pose_str(c)
            lines.append('        <geometry><box>'
                         f'<size>{size_str}</size>'
                         '</box></geometry>')
            lines.append(f'        <pose>{pose}</pose>')
            lines += surface()
        lines.append('      </collision>')
        if name in RAIL_NODES:
            rail_seen.add(name)
            lines += [
                f'      <visual name="{name}_visual">',
                f'        <geometry><box><size>{size_str}</size></box></geometry>',
                f'        <pose>{pose}</pose>',
                '        <material>',
                f'          <ambient>{RAIL_COLOR}</ambient>',
                f'          <diffuse>{RAIL_COLOR}</diffuse>',
                '        </material>',
                '      </visual>',
            ]
        body += lines

    if rail_seen != RAIL_NODES:
        missing = ', '.join(sorted(RAIL_NODES - rail_seen))
        raise RuntimeError(f'上游 DAE 缺少 L1 围栏节点: {missing}')

    # L2 侧面灰色包边：纯视觉薄板, 无碰撞; 高度正好覆盖 z 0.6~0.9
    body.append('      <!-- L2 侧面包边 (z 0.6~0.9, 与 L2 顶面同灰) -->')
    for tag, size, pos in L2_SIDES:
        body += [
            f'      <visual name="l2_side_gray_{tag}">',
            f'        <geometry><box><size>{num_join(size)}</size></box></geometry>',
            f'        <pose>{num_join(pos)} 0 0 0</pose>',
            '        <material>',
            f'          <ambient>{L2_SIDE_COLOR}</ambient>',
            f'          <diffuse>{L2_SIDE_COLOR}</diffuse>',
            '        </material>',
            '      </visual>',
        ]

    films = preserved_films(OUT)

    xml = '\n'.join([
        '<?xml version="1.0"?>',
        '<!-- 由 tool/scripts/gen_full_collision.py 从视觉 DAE 自动生成, 请勿手改 -->',
        '<sdf version="1.6">',
        '  <model name="robocon_ground_full">',
        '    <static>true</static>',
        '    <link name="link">',
        '      <visual name="visual">',
        '        <geometry><mesh><uri>model://robocon_ground_full/meshes/Untitled.dae</uri></mesh></geometry>',
        '      </visual>',
        *body,
        # 台面着色层排在最后: 纯视觉覆盖, 无碰撞, 与上方生成结果互不影响
        *films,
        '    </link>',
        '  </model>',
        '</sdf>',
    ]) + '\n'

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, 'w', encoding='utf-8') as f:
        f.write(xml)
    print(f'写入 {OUT}: {n_box} box + {n_cyl} cylinder + {len(RAMPS)} ramp mesh'
          + (f' + {len(films)} 台面着色层(保留)' if films else ''))


if __name__ == '__main__':
    gen()
