"""Visual playback only. No physical-robot SDK, motor control, or USB access."""
import argparse
import csv
from pathlib import Path
import threading
import time

import mujoco
import numpy as np

BASE = Path(__file__).resolve().parent
# Approximate visual calibration, not a measured mechanism model.
GRIPPER_CLOSED_RAD = 0.0
GRIPPER_OPEN_RAD = 2.0
FINGER_TRAVEL_M = 0.0405


def load_tracks(path):
    with path.open(newline='', encoding='utf-8-sig') as f:
        rows = list(csv.DictReader(f))
    tracks = {}
    for role in ('Leader', 'Follower'):
        selected = [r for r in rows if r['arm'] == role]
        if len(selected) < 2:
            raise ValueError(f'Need at least two rows for {role}')
        t = np.array([float(r['elapsed_s']) for r in selected])
        q = np.array([[float(r[f'j{i}_q_rad']) for i in range(1, 7)] for r in selected])
        g = np.array([float(r['gripper_q_rad']) for r in selected])
        if not (np.isfinite(t).all() and np.isfinite(q).all() and np.isfinite(g).all()):
            raise ValueError(f'{role}: nonfinite data')
        if not np.all(np.diff(t) > 0):
            raise ValueError(f'{role}: timestamps must strictly increase')
        tracks[role.lower()] = (t, q, g)
    # One shared origin preserves the small offset between the arm readings.
    origin = min(x[0][0] for x in tracks.values())
    tracks = {k: (t-origin, q, g) for k, (t, q, g) in tracks.items()}
    return tracks, max(x[0][-1] for x in tracks.values())


def addresses(model):
    result = {}
    for role in ('leader', 'follower'):
        ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f'{role}_joint{i}') for i in range(1, 7)]
        if min(ids) < 0:
            raise ValueError('Missing named joint in scene')
        if any(model.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE for j in ids):
            raise ValueError('Expected six hinge joints per arm')
        result[role] = model.jnt_qposadr[ids]
        fingers = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f'{role}_gripper_{side}') for side in ('left', 'right')]
        if min(fingers) < 0:
            raise ValueError('Missing gripper joints: replace scene.xml and meshes too')
        result[role+'_gripper'] = model.jnt_qposadr[fingers]
    return result


def pose(model, data, tracks, addr, seconds):
    for role, (t, q, g) in tracks.items():
        data.qpos[addr[role]] = [np.interp(seconds, t, q[:, j]) for j in range(6)]
        angle = np.interp(seconds, t, g)
        opening = np.clip((angle-GRIPPER_CLOSED_RAD)/(GRIPPER_OPEN_RAD-GRIPPER_CLOSED_RAD), 0, 1)
        data.qpos[addr[role+'_gripper']] = FINGER_TRAVEL_M*(1-opening)
    data.qvel[:] = 0
    data.time = seconds
    mujoco.mj_forward(model, data)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--csv', type=Path, default=BASE/'joint_data.csv')
    p.add_argument('--speed', type=float, default=1.0)
    p.add_argument('--loop', action='store_true')
    p.add_argument('--check', action='store_true', help='Validate all recorded poses without opening a window')
    a = p.parse_args()
    if not np.isfinite(a.speed) or a.speed <= 0:
        p.error('--speed must be positive and finite')
    tracks, duration = load_tracks(a.csv)
    model = mujoco.MjModel.from_xml_path(str(BASE/'scene.xml'))
    data = mujoco.MjData(model)
    addr = addresses(model)
    print(f'Loaded {duration:.2f} seconds; ' + ', '.join(f'{k}: {len(v[0])} samples' for k,v in tracks.items()))
    print('Visual replay, not dynamic tracking. Base spacing is illustrative. Grippers use approximate parallel motion calibrated as 0 rad closed, 2 rad open.')
    if a.check:
        for t in np.unique(np.concatenate([v[0] for v in tracks.values()])):
            pose(model, data, tracks, addr, float(t))
            if not np.isfinite(data.xpos).all():
                raise ValueError('Nonfinite forward kinematics')
        print('PASS: both tracks mapped to 12 arm joints and 4 finger joints; all poses evaluated.')
        return

    from mujoco import viewer as viewer_module
    commands = []
    mutex = threading.Lock()
    def key(k):
        with mutex:
            commands.append(k)

    print('SPACE: pause/resume | R: restart | close window: exit')
    seconds = 0.0
    paused = False
    pose(model, data, tracks, addr, seconds)
    with viewer_module.launch_passive(model, data, key_callback=key) as viewer:
        viewer.cam.lookat[:] = [-0.05, 0, 0.25]
        viewer.cam.distance = 2.3
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -25
        viewer.opt.label = mujoco.mjtLabel.mjLABEL_SITE
        last = time.monotonic()
        last_print = last
        while viewer.is_running():
            now = time.monotonic()
            dt = now-last
            last = now
            with mutex:
                pending, commands[:] = commands[:], []
            for k in pending:
                if k == 32:
                    paused = not paused
                elif k in (ord('R'), ord('r')):
                    seconds = 0
                    paused = False
                    dt = 0
            if not paused:
                seconds += dt*a.speed
                if seconds >= duration:
                    if a.loop:
                        seconds %= duration
                    else:
                        seconds = duration
                        paused = True
            with viewer.lock():
                pose(model, data, tracks, addr, seconds)
            viewer.sync()
            if now-last_print >= .5:
                grippers = [np.interp(seconds, tracks[r][0], tracks[r][2]) for r in ('leader','follower')]
                print(f'\r{seconds:5.1f}/{duration:.1f}s | gripper rad: Leader {grippers[0]:.3f}, Follower {grippers[1]:.3f} | {"PAUSED" if paused else "PLAYING"}', end='', flush=True)
                last_print = now
            time.sleep(1/120)
    print()


if __name__ == '__main__':
    main()
