#!/usr/bin/env python3
"""场地可视化编辑器后端。

职责:
  GET  /                -> 返回 editor/index.html
  GET  /api/state       -> 解析 model.sdf, 返回可编辑的 JSON 场景
  POST /api/preview     -> 接收场景 JSON, 返回将要写入的 SDF 文本 + diff 摘要 (不落盘)
  POST /api/apply       -> 接收场景 JSON, 备份 + 写回 model.sdf + colcon build
  GET  /api/git         -> 当前 git 状态, 判断是否有未提交改动

设计约定:
  - 唯一真值是 model.sdf。编辑器不解析 DAE, 只把 SDF 的 collision/visual 读出来。
  - 写回时保留原文件的注释与顺序尽可能少改动: 采用"按 name 匹配、原地替换"策略,
    新增项追加到 </link> 之前, 删除项整块移除。
  - 每次 apply 前把旧文件备份到 .cache/sdf_backup/<时间戳>.sdf。

用法:
    /usr/bin/python3 tool/scripts/field_editor_server.py [--port 8790]
"""
import argparse
import json
import os
import re
import shutil
import subprocess
import time
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# 本文件位于 <ws>/tool/scripts/, 上溯三层才是工作区根
WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SDF = os.path.join(WS, 'src', 'robocon_gazebo', 'models', 'robocon_ground_full', 'model.sdf')
BACKUP_DIR = os.path.join(WS, '.cache', 'sdf_backup')
EDITOR_DIR = os.path.join(WS, 'tool', 'scripts', 'field_editor')

NUM_RE = re.compile(r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?')

# 台面着色层（color film）: 一层只有颜色、没有碰撞的薄片, 贴在某段台面的顶面上方。
# 名称前缀即约定 —— 编辑器靠它把普通 visual 和着色层区分开, 便于单独分组、
# 单独换色, 同时保证写回后重新读取时能自证身份（SDF 里不需要额外字段）。
FILM_PREFIX = 'colorfilm'
FILM_THICKNESS = 0.002          # 默认厚度 2 mm, 只为了不与被覆盖面 z-fighting


def fnum(x):
    s = f'{float(x):.4f}'.rstrip('0').rstrip('.')
    if s in ('', '-0'):
        return '0'
    return s


def parse_nums(text, default):
    if not text:
        return list(default)
    vals = [float(v) for v in NUM_RE.findall(text)]
    while len(vals) < len(default):
        vals.append(default[len(vals)])
    return vals


def parse_color(mat):
    """从 <material> 里取 RGB (0~1 浮点)。

    着色层的颜色只存在于 <emissive>（diffuse 故意为 0）, 所以 emissive 优先,
    再退到 diffuse / ambient 兼容旧文件和普通 visual。
    """
    if mat is None:
        return None
    for tag in ('emissive', 'diffuse', 'ambient'):
        t = mat.findtext(tag)
        if t:
            vals = [float(v) for v in NUM_RE.findall(t)]
            if len(vals) >= 3:
                return vals[:3]      # 纯黑 (0,0,0) 也是合法颜色, 不能当缺失跳过
    return None


# ---------------------------------------------------------------- 读 SDF

def read_state():
    tree = ET.parse(SDF)
    root = tree.getroot()
    model = root.find('model')
    link = model.find('link')

    items = []
    # 按文档原始顺序遍历, 不要按 kind 分组 —— 分组会把整体的 DAE 视觉节点
    # 挪到文件末尾, 写回后场地图案全部丢失, 只剩 SDF 图元, 看起来"全变灰"。
    for el in list(link):
        kind = el.tag
        if kind not in ('collision', 'visual'):
            continue
        name = el.get('name') or f'{kind}_{len(items)}'
        geo = el.find('geometry')
        pose = parse_nums(el.findtext('pose'), [0, 0, 0, 0, 0, 0])
        entry = {
            'kind': kind,
            'name': name,
            'pose': pose,
            'geometry': None,
            'static': True,          # 是否是原 SDF 里就有的（用于区分新增）
            'hadPose': el.find('pose') is not None,
        }
        if geo is not None:
            box = geo.find('box')
            cyl = geo.find('cylinder')
            mesh = geo.find('mesh')
            if box is not None:
                s = parse_nums(box.findtext('size'), [1, 1, 1])
                entry['geometry'] = {'type': 'box', 'size': s}
            elif cyl is not None:
                entry['geometry'] = {
                    'type': 'cylinder',
                    'radius': parse_nums(cyl.findtext('radius'), [0.5])[0],
                    'length': parse_nums(cyl.findtext('length'), [1])[0],
                }
            elif mesh is not None:
                uri = (mesh.findtext('uri') or '').strip()
                entry['geometry'] = {'type': 'mesh', 'uri': uri}
                entry['static'] = True
                entry['locked'] = True   # 网格不做几何编辑, 只能整体位移
        mu = el.findtext('surface/friction/ode/mu')
        entry['mu'] = parse_nums(mu, [0.8])[0] if mu else None
        entry['max_vel'] = parse_nums(el.findtext('surface/contact/ode/max_vel'), [0.1])[0]
        entry['min_depth'] = parse_nums(el.findtext('surface/contact/ode/min_depth'), [0.001])[0]
        # 原始 material 的完整文本, 写回时原样还原, 避免浮点格式化改坏精度
        # (0.745098 被写成 0.7451 会导致肉眼可见的颜色偏差)
        mat = el.find('material')
        entry['materialRaw'] = (
            ET.tostring(mat, encoding='unicode').strip() if mat is not None else None)
        if kind == 'visual' and name.startswith(FILM_PREFIX):
            # 台面着色层: 只有颜色, 没有碰撞; 颜色由字段 color 承载, 便于单独调色
            entry['film'] = True
            entry['color'] = parse_color(mat)
            if entry['geometry'] and entry['geometry']['type'] == 'box':
                entry['thickness'] = entry['geometry']['size'][2]
        items.append(entry)
    return {'model': model.get('name'), 'sdfPath': SDF, 'items': items}


# ---------------------------------------------------------------- 写 SDF

HEADER = ('<?xml version="1.0"?>\n'
          '<!-- 由 tool/scripts/gen_full_collision.py 从视觉 DAE 自动生成, 请勿手改 -->\n'
          '<!-- 场地编辑器写回时间: {ts} -->\n'
          '<sdf version="1.6">\n'
          '  <model name="{model}">\n'
          '    <static>true</static>\n'
          '    <link name="link">\n')

FOOTER = ('    </link>\n'
          '  </model>\n'
          '</sdf>\n')


def render_item(it, indent='      '):
    name = it['name']
    pose = ' '.join(fnum(v) for v in it['pose'])
    g = it['geometry']
    if g['type'] == 'box':
        geo = ('<geometry><box><size>%s</size></box></geometry>'
               % ' '.join(fnum(v) for v in g['size']))
    elif g['type'] == 'cylinder':
        geo = ('<geometry><cylinder><radius>%s</radius><length>%s</length></cylinder></geometry>'
               % (fnum(g['radius']), fnum(g['length'])))
    else:
        geo = f'<geometry><mesh><uri>{g["uri"]}</uri></mesh></geometry>'

    out = [f'{indent}<{it["kind"]} name="{name}">',
           f'{indent}  {geo}']
    # 原文件没有 <pose> 的节点（如整体 DAE 网格）不要凭空补一个全零 pose
    if it.get('hadPose', True):
        out.append(f'{indent}  <pose>{pose}</pose>')
    has_surface = it.get('mu') is not None or it['kind'] == 'collision'
    if has_surface and it['kind'] == 'collision':
        out += [
            f'{indent}  <surface>',
            f'{indent}    <friction><ode><mu>{fnum(it.get("mu") or 0.8)}</mu>'
            f'<mu2>{fnum(it.get("mu") or 0.8)}</mu2></ode></friction>',
            f'{indent}    <contact><ode><max_vel>{fnum(it.get("max_vel") or 0.1)}</max_vel>'
            f'<min_depth>{fnum(it.get("min_depth") or 0.001)}</min_depth></ode></contact>',
            f'{indent}  </surface>',
        ]
    if it['kind'] == 'visual' and it.get('film'):
        # 台面着色层: 材质完全由 color 生成, 不复用 materialRaw。
        #
        # 颜色只走 <emissive>, ambient/diffuse/specular 全部置 0。
        # 原因: Gazebo Classic (Ogre) 的最终颜色 ≈ ambient*环境光 + diffuse*光照 + emissive,
        # 三者同色会把颜色叠加 1.5~2 倍 -> 编辑器里设置的 #RRGGBB 到了 Gazebo 变成更亮更偏的色。
        # 只留 emissive 时最终颜色恒等于设定值, 且不受光源方向影响（俯视/斜视/侧视同色）。
        c = ' '.join(f'{float(v):.6f}'.rstrip('0').rstrip('.')
                     for v in (it.get('color') or [0.745098, 0.745098, 0.72549]))
        out += [
            f'{indent}  <cast_shadows>false</cast_shadows>',
            f'{indent}  <material>',
            f'{indent}    <ambient>0 0 0 1</ambient>',
            f'{indent}    <diffuse>0 0 0 1</diffuse>',
            f'{indent}    <specular>0 0 0 1</specular>',
            f'{indent}    <emissive>{c} 1</emissive>',
            f'{indent}  </material>',
        ]
    elif it['kind'] == 'visual':
        raw = it.get('materialRaw')
        if raw:
            # 原样还原原始 <material> 块, 数值一个字节都不改（保留 0.745098 这种精度）
            lines = [ln.strip() for ln in raw.splitlines() if ln.strip()]
            out.append(f'{indent}  {lines[0]}')
            # 子元素比 <material> 多缩进两格, 与原始排版一致
            out += [f'{indent}    {ln}' for ln in lines[1:-1]]
            if len(lines) > 1:
                out.append(f'{indent}  {lines[-1]}')
        elif it.get('color'):
            c = ' '.join(f'{float(v):.6f}'.rstrip('0').rstrip('.') for v in it['color'])
            out += [
                f'{indent}  <material>',
                f'{indent}    <ambient>{c} 1</ambient>',
                f'{indent}    <diffuse>{c} 1</diffuse>',
                f'{indent}  </material>',
            ]
        # 原本就没有 <material> 的 visual（例如整体 DAE 网格）保持无材质,
        # 绝不能凭空补一个灰色材质, 否则整片场地会被刷成灰色。
    out.append(f'{indent}</{it["kind"]}>')
    return out


def emit_sdf(model_name, items):
    """保持 items 的顺序输出完整 SDF。"""
    body = []
    for it in items:
        if it.get('hidden'):
            continue
        body += render_item(it)
    ts = time.strftime('%Y-%m-%d %H:%M:%S')
    return HEADER.format(ts=ts, model=model_name) + '\n'.join(body) + '\n' + FOOTER


def diff_summary(old, new):
    old_map = {(i['kind'], i['name']): i for i in old['items']}
    new_map = {(i['kind'], i['name']): i for i in new['items']}
    added, removed, changed = [], [], []
    for k in new_map:
        if k not in old_map:
            added.append(k[1])
        else:
            a, b = old_map[k], new_map[k]
            fields = {}
            if a['pose'] != b['pose']:
                fields['pose'] = {'from': a['pose'], 'to': b['pose']}
            ga, gb = a['geometry'], b['geometry']
            if ga != gb:
                fields['geometry'] = {'from': ga, 'to': gb}
            if a.get('hidden') != b.get('hidden'):
                fields['hidden'] = {'from': a.get('hidden'), 'to': b.get('hidden')}
            if a.get('color') != b.get('color'):
                fields['color'] = {'from': a.get('color'), 'to': b.get('color')}
            if fields:
                changed.append({'name': k[1], 'kind': k[0], 'fields': fields})
    for k in old_map:
        if k not in new_map:
            removed.append(k[1])
    return {'added': added, 'removed': removed, 'changed': changed}


# ---------------------------------------------------------------- 应用

def validate_against_qing(items):
    """基础合法性检查, 返回 (errors, warnings)。"""
    errors, warnings = [], []
    names = {}
    for it in items:
        if it.get('hidden'):
            continue
        key = (it['kind'], it['name'])
        if key in names:
            errors.append(f'{it["kind"]} 名称重复: {it["name"]}')
        names[key] = True
        g = it['geometry']
        if g['type'] == 'box':
            if any(float(v) <= 0 for v in g['size']):
                errors.append(f'{it["name"]}: box 尺寸必须全部 > 0')
        elif g['type'] == 'cylinder':
            if g['radius'] <= 0 or g['length'] <= 0:
                errors.append(f'{it["name"]}: 圆柱半径/长度必须 > 0')
        if it['kind'] == 'collision' and g['type'] == 'box':
            # 场地边界 11x11, 超界只告警不阻止
            p = it['pose']
            half = [g['size'][0] / 2, g['size'][1] / 2]
            if abs(p[0]) + half[0] > 5.6 or abs(p[1]) + half[1] > 5.6:
                warnings.append(f'{it["name"]}: 超出场地 11x11 边界')
        if it.get('film'):
            if it['kind'] != 'visual':
                errors.append(f'{it["name"]}: 台面着色层只能是 visual, 不能带碰撞')
            elif g['type'] != 'box':
                errors.append(f'{it["name"]}: 台面着色层必须是 box 薄片')
            elif g['size'][2] < 5e-4:
                errors.append(f'{it["name"]}: 台面着色层厚度必须 >= 0.0005 m')
    return errors, warnings


def apply_state(payload):
    new_state = payload
    old_state = read_state()
    errors, warnings = validate_against_qing(new_state['items'])
    if errors:
        return {'ok': False, 'errors': errors, 'warnings': warnings}

    if not os.path.isdir(BACKUP_DIR):
        os.makedirs(BACKUP_DIR, exist_ok=True)
    stamp = time.strftime('%Y%m%d-%H%M%S')
    backup = os.path.join(BACKUP_DIR, f'model_{stamp}.sdf')
    shutil.copy2(SDF, backup)

    text = emit_sdf(old_state['model'], new_state['items'])
    try:
        ET.fromstring(text.encode('utf-8'))
    except ET.ParseError as e:
        return {'ok': False, 'errors': [f'生成的 SDF 不合法: {e}'], 'warnings': warnings}

    with open(SDF, 'w', encoding='utf-8') as f:
        f.write(text)

    # gz sdf -k 校验
    chk = subprocess.run(['gz', 'sdf', '-k', SDF], capture_output=True, text=True)
    chk_ok = 'Check complete' in (chk.stdout + chk.stderr)

    build = subprocess.run(
        ['bash', '-lc',
         'source /opt/ros/humble/setup.bash && '
         'cd "%s" && PATH=/usr/bin:$PATH colcon build --symlink-install '
         '--packages-select robocon_gazebo --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3'
         % WS],
        capture_output=True, text=True)

    return {
        'ok': True,
        'backup': backup,
        'diff': diff_summary(old_state, new_state),
        'sdfCheck': chk_ok,
        'sdfCheckOutput': (chk.stdout + chk.stderr).strip(),
        'buildOk': build.returncode == 0,
        'buildOutput': (build.stdout + build.stderr).strip()[-3000:],
        'warnings': warnings,
    }


# ---------------------------------------------------------------- HTTP

class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def _send(self, code, body, ctype='application/json'):
        data = body.encode('utf-8') if isinstance(body, str) else body
        self.send_response(code)
        self.send_header('Content-Type', ctype + '; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def _json(self, obj, code=200):
        self._send(code, json.dumps(obj, ensure_ascii=False))

    def do_GET(self):
        path = self.path.split('?')[0]
        if path in ('/', '/index.html'):
            fp = os.path.join(EDITOR_DIR, 'index.html')
            if not os.path.isfile(fp):
                return self._send(404, 'editor/index.html 缺失', 'text/plain')
            with open(fp, 'rb') as f:
                return self._send(200, f.read(), 'text/html')
        if path == '/api/state':
            return self._json(read_state())
        if path == '/api/git':
            r = subprocess.run(['git', 'status', '--short', '--', SDF],
                               cwd=WS, capture_output=True, text=True)
            return self._json({'dirty': bool(r.stdout.strip()), 'raw': r.stdout})
        return self._send(404, 'not found', 'text/plain')

    def do_POST(self):
        n = int(self.headers.get('Content-Length') or 0)
        raw = self.rfile.read(n) if n else b'{}'
        try:
            payload = json.loads(raw.decode('utf-8'))
        except Exception as e:
            return self._json({'ok': False, 'errors': [f'请求体不是合法 JSON: {e}']}, 400)

        path = self.path.split('?')[0]
        if path == '/api/preview':
            old = read_state()
            errors, warnings = validate_against_qing(payload.get('items', []))
            text = emit_sdf(old['model'], payload.get('items', []))
            return self._json({
                'ok': not errors,
                'errors': errors,
                'warnings': warnings,
                'diff': diff_summary(old, payload),
                'sdf': text,
            })
        if path == '/api/apply':
            return self._json(apply_state(payload))
        return self._send(404, 'not found', 'text/plain')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8790)
    ap.add_argument('--host', default='127.0.0.1')
    a = ap.parse_args()
    srv = ThreadingHTTPServer((a.host, a.port), Handler)
    print(f'场地编辑器: http://{a.host}:{a.port}')
    srv.serve_forever()


if __name__ == '__main__':
    main()
