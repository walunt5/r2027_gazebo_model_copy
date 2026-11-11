#!/usr/bin/env python3
"""台面着色层（colorfilm）闭环自测：读 SDF -> 加一层 -> 写出 -> 再读回 -> 校验。

不改动工程里的 model.sdf，全程在 .cache/tests/ 下的副本上跑。
"""
import importlib.util
import json
import os
import shutil
import subprocess
import sys

WS = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
TMP = os.path.join(WS, '.cache', 'tests')
os.makedirs(TMP, exist_ok=True)
SDF_REAL = os.path.join(WS, 'src', 'robocon_gazebo', 'models', 'robocon_ground_full', 'model.sdf')


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


srv = load('field_editor_server', os.path.join(WS, 'tool', 'scripts', 'field_editor_server.py'))
gen = load('gen_full_collision', os.path.join(WS, 'tool', 'scripts', 'gen_full_collision.py'))

ok = True


def check(label, cond, extra=''):
    global ok
    print(('  PASS  ' if cond else '  FAIL  ') + label + (' | ' + str(extra) if extra else ''))
    ok = ok and bool(cond)


# ---- 1. 生成器幂等性（在副本上跑, 不碰工程文件）
# 基线不能用工程里的 model.sdf —— 编辑器写回会丢结构注释, 与生成器输出天然有差异,
# 所以这里比对"连跑两次是否一致", 以及着色层是否被搬运。
tmp_gen = os.path.join(TMP, 'gen_out.sdf')
shutil.copy2(SDF_REAL, tmp_gen)
gen.OUT = tmp_gen
gen.gen()
first = open(tmp_gen, encoding='utf-8').read()
gen.gen()
second = open(tmp_gen, encoding='utf-8').read()
check('gen_full_collision 自幂等（连跑两次输出一致）', first == second)
n_keep = first.count('<visual name="colorfilm')
n_now = open(SDF_REAL, encoding='utf-8').read().count('<visual name="colorfilm')
check('生成器保留已有着色层', n_keep == n_now, f'{n_keep} / {n_now}')

# ---- 2. 在副本 SDF 上加一层着色层, 走 emit/read 闭环
tmp = os.path.join(TMP, 'film_roundtrip.sdf')
shutil.copy2(SDF_REAL, tmp)
srv.SDF = tmp
srv.BACKUP_DIR = TMP

st = srv.read_state()
n0 = len(st['items'])
check('读取原始 SDF', n0 > 30, f'{n0} items')
check('SDF 里已有着色层被识别', any(i.get('film') for i in st['items']) == (n_now > 0))

film = {
    'kind': 'visual',
    'name': 'colorfilm_test',
    'pose': [0, 0.5, 0.603, 0, 0, 0],
    'geometry': {'type': 'box', 'size': [5.9, 2.4, 0.002]},
    'static': False, 'hadPose': True,
    'mu': None, 'max_vel': 0.1, 'min_depth': 0.001,
    'materialRaw': None, 'film': True, 'color': [0.2, 0.55, 0.9], 'thickness': 0.002,
}
items = st['items'] + [film]

text = srv.emit_sdf(st['model'], items)
open(tmp, 'w', encoding='utf-8').write(text)
check('写出的 SDF 含 cast_shadows=false', '<cast_shadows>false</cast_shadows>' in text)
check('颜色只走 emissive（避免 Gazebo 里叠加变亮）',
      '<emissive>0.2 0.55 0.9 1</emissive>' in text)
check('ambient/diffuse/specular 全部归零',
      '<ambient>0 0 0 1</ambient>' in text and '<diffuse>0 0 0 1</diffuse>' in text
      and '<specular>0 0 0 1</specular>' in text)
check('写出的 SDF 无 <collision name="colorfilm', 'collision name="colorfilm' not in text)

st2 = srv.read_state()
back = [i for i in st2['items'] if i.get('name') == 'colorfilm_test']
check('重新读取识别到 film', len(back) == 1, back[0]['name'] if back else None)
if back:
    f = back[0]
    check('color 往返一致（读 emissive）', [round(v, 4) for v in f['color']] == [0.2, 0.55, 0.9], f['color'])
    check('几何/厚度一致', f['geometry']['size'] == [5.9, 2.4, 0.002], f['geometry']['size'])
    check('pose 一致', [round(v, 4) for v in f['pose'][:3]] == [0, 0.5, 0.603], f['pose'][:3])

# ---- 3. gz sdf -k 校验
r = subprocess.run(['gz', 'sdf', '-k', tmp], capture_output=True, text=True)
check('gz sdf -k 通过', 'Check complete' in (r.stdout + r.stderr), (r.stdout + r.stderr).strip()[:100])

# ---- 4. 校验器：film 不允许带碰撞 / 厚度过薄
errs, _ = srv.validate_against_qing([dict(film, kind='collision')])
check('校验器拒绝带碰撞的 film', bool(errs), errs)
errs, _ = srv.validate_against_qing([dict(film, geometry={'type': 'box', 'size': [1, 1, 0.0001]})])
check('校验器拒绝过薄 film', bool(errs), errs)

# ---- 5. 重跑生成器应保留着色层
text2 = open(tmp, encoding='utf-8').read()
check('生成的 tmp 含着色层', '<visual name="colorfilm_test">' in text2)
gen.OUT = tmp
gen.gen()
kept = open(tmp, encoding='utf-8').read()
check('gen_full_collision 保留 colorfilm', '<visual name="colorfilm_test">' in kept
      and '<emissive>0.2 0.55 0.9 1</emissive>' in kept)
r = subprocess.run(['gz', 'sdf', '-k', tmp], capture_output=True, text=True)
check('保留后 SDF 仍合法', 'Check complete' in (r.stdout + r.stderr))

print('\nRESULT:', 'ALL PASS' if ok else 'FAILURES PRESENT')
sys.exit(0 if ok else 1)
