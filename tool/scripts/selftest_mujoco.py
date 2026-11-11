#!/usr/bin/env python3
"""无界面自检: 在 MuJoCo 中跑 ONNX 策略 N 秒, 检查机器人是否稳定站立。

用途: 验证环境 (mujoco/onnxruntime) 与模型资源是否正常, 不需要显示器。

用法:
    ~/.venvs/robocon_mujoco/bin/python tool/scripts/selftest_mujoco.py [--seconds 3]
"""

import argparse
import sys
from pathlib import Path

# 本文件位于 <ws>/tool/scripts/, 上溯三层才是工作区根
WS = Path(__file__).resolve().parent.parent.parent
DEMO_DIR = WS / 'tool' / 'gazebo_models' / 'robocon_mujoco'
sys.path.insert(0, str(DEMO_DIR))

import mujoco                       # noqa: E402
import numpy as np                  # noqa: E402
import onnxruntime as ort           # noqa: E402
import yaml                         # noqa: E402
import play_robocon as P            # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=3.0, help='仿真时长')
    ap.add_argument('--no-policy', action='store_true', help='只做 PD 站立')
    args = ap.parse_args()

    cfg_raw = yaml.safe_load(P.DEFAULT_CONFIG.read_text(encoding='utf-8'))
    kps = np.array(cfg_raw['kps'], dtype=np.float64)
    kds = np.array(cfg_raw['kds'], dtype=np.float64)
    action_scale = cfg_raw['action_scale']
    dt = cfg_raw['simulation_dt']
    decim = cfg_raw['control_decimation']
    cfg = {
        'cmd_scale': np.array(cfg_raw['cmd_scale'][:3], dtype=np.float32),
        'ang_vel_scale': cfg_raw['ang_vel_scale'],
        'dof_pos_scale': cfg_raw['dof_pos_scale'],
        'dof_vel_scale': cfg_raw['dof_vel_scale'],
        'clip_obs': cfg_raw.get('clip_obs', 100.0),
    }

    model = mujoco.MjModel.from_xml_path(str(P.DEFAULT_XML))
    model.opt.timestep = dt
    data = mujoco.MjData(model)
    P.reset_robot(model, data)

    sess = None
    if not args.no_policy:
        sess = ort.InferenceSession(str(P.DEFAULT_ONNX),
                                    providers=['CPUExecutionProvider'])
        in_name = sess.get_inputs()[0].name
        out_name = sess.get_outputs()[0].name

    hist = P.ObsHistoryBuffer(P.HISTORY_LEN, P.NUM_ONE_STEP_OBS)
    target_q = P.DEFAULT_ANGLES_MUJOCO.copy()
    last_action = np.zeros(P.NUM_ACTIONS, dtype=np.float32)
    action = np.zeros(P.NUM_ACTIONS, dtype=np.float64)
    cmd = np.zeros(3, dtype=np.float32)

    steps = int(args.seconds / dt)
    for i in range(steps):
        q_m = data.qpos[7:19].astype(np.float64)
        dq_m = data.qvel[6:18].astype(np.float64)
        q_i = q_m[P.MUJOCO_TO_ISAAC]
        dq_i = dq_m[P.MUJOCO_TO_ISAAC]
        qw = data.qpos[3:7]
        quat_xyzw = np.array([qw[1], qw[2], qw[3], qw[0]])
        omega = data.qvel[3:6].astype(np.float64)

        if i % decim == 0:
            if sess is not None:
                obs = P.build_single_obs(quat_xyzw, omega, q_i, dq_i,
                                         last_action, cmd, cfg, 0.25)
                hist.push(obs)
                raw = sess.run([out_name], {in_name: hist.get()})[0][0]
                action[:] = np.clip(raw, -10.0, 10.0)
                last_action = action.astype(np.float32)
                target_q = (action * action_scale + P.DEFAULT_ANGLES_ISAAC)[P.ISAAC_TO_MUJOCO]
            else:
                target_q = P.DEFAULT_ANGLES_MUJOCO.copy()

        data.ctrl[:P.NUM_ACTIONS] = kps * (target_q - q_m) - kds * dq_m
        mujoco.mj_step(model, data)

    pos = data.qpos[:3]
    grav_z = P.quat_rotate_inverse(
        np.array([data.qpos[4], data.qpos[5], data.qpos[6], data.qpos[3]]),
        np.array([0., 0., -1.]))[2]
    print(f'sim {args.seconds:.1f}s / {steps} steps ({"policy" if sess else "PD only"})')
    print(f'  base pos = ({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f})  期望 z≈0.45')
    print(f'  base 倾角(重力投影 z) = {grav_z:+.3f}  期望 ≈ -1.0 (直立)')
    ok = abs(pos[2] - 0.45) < 0.15 and grav_z < -0.7
    print('  结果:', 'PASS' if ok else 'FAIL (可能穿模/翻倒, 检查模型与策略)')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
