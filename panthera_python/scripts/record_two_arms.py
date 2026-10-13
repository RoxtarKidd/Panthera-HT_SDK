#!/usr/bin/env python3
"""Independent Panthera hand-guided recording; NO cameras or arm following.

Place in Panthera-HT_SDK/panthera_python/scripts and run in panthera-env.
python record_two_arms.py --seconds 10   # first short session
python record_two_arms.py                # Ctrl+C stops a session

Based on the user's SDK gravity-compensation example. Hardware-unverified.
Both arms receive their OWN model's gravity torque, zero kp/kd, and a free
gripper. Support arms before starting/stopping: compensation can disappear.
No payload compensation, collision detection, or hardware-synchronous sampling.
The SDK caches state: timestamps below measure host reads, NOT sensor time.
An unchanged cache cannot be distinguished from a stationary joint by this API.
The motor command watchdog is requested at 250 ms; it is not an emergency stop.
Keep a physical cutoff available. Never run another arm controller concurrently.
The serial IDs depend on USB enumeration; verify mappings after reconnecting.

Outputs: CSV (one row per arm per sample) and JSON metadata in recordings/.
Target control/log rate 100 Hz; actual timing is recorded, not guaranteed.
Torque fields are SDK-reported torque and the requested gravity feedforward.
Limits on the zero position target are not physical joint-limit enforcement.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import queue
import threading
import time
from datetime import datetime, timezone


def vector(values, length, label):
    result = [float(x) for x in values]
    if len(result) != length or not all(math.isfinite(x) and abs(x) < 900 for x in result):
        raise RuntimeError(f"Invalid/uninitialized {label}: {result}")
    return result


def sample(robot):
    begin = time.monotonic_ns()
    q = vector(robot.get_current_pos(), 6, 'joint positions')
    dq = vector(robot.get_current_vel(), 6, 'joint velocities')
    tau = vector(robot.get_current_torque(), 6, 'joint torques')
    g = robot.get_current_state_gripper()
    grip = vector([g.position, g.velocity, g.torque], 3, 'gripper state')
    return begin, time.monotonic_ns(), q, dq, tau, grip


def writer_task(path, rows, done, errors):
    try:
        with path.open('x', newline='') as stream:
            writer = csv.writer(stream)
            header = ['sample_id', 'arm', 'host_read_start_ns', 'host_read_end_ns', 'elapsed_s']
            for field in ('q_rad', 'dq_rad_s', 'measured_tau_Nm', 'command_tau_Nm'):
                header += [f'j{i}_{field}' for i in range(1, 7)]
            header += ['gripper_q_rad', 'gripper_dq_rad_s', 'gripper_measured_tau_Nm']
            writer.writerow(header)
            last_flush = time.monotonic()
            while not done.is_set() or not rows.empty():
                try:
                    row = rows.get(timeout=0.1)
                except queue.Empty:
                    continue
                writer.writerow(row)
                if time.monotonic() - last_flush >= 1:
                    stream.flush()
                    last_flush = time.monotonic()
            stream.flush()
            os.fsync(stream.fileno())
    except BaseException as exc:
        errors.append(str(exc))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--seconds', type=float, default=0, help='0 = until Ctrl+C')
    parser.add_argument('--output', type=Path, default=Path('recordings'))
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds < 0:
        parser.error('--seconds must be finite and nonnegative')

    # Import only here so --help does not load the hardware SDK.
    from Panthera_lib import Panthera
    import yaml

    base = Path(__file__).resolve().parent
    configs = {name: (base / '../robot_param' / f'{name}.yaml').resolve()
               for name in ('Leader', 'Follower')}
    config_text = {}
    selections = []
    for name, path in configs.items():
        config_text[name] = path.read_text()
        cfg = yaml.safe_load(config_text[name])
        motor_path = (path.parent / cfg['robot']['param_file']).resolve()
        config_text[name + '_motors'] = motor_path.read_text()
        motor_cfg = yaml.safe_load(config_text[name + '_motors'])['robot']
        boards = motor_cfg['CANboard']
        ports = [port for board in boards.values() for port in board['CANport'].values()]
        if len(ports) != 1:
            raise RuntimeError('This recorder expects one motor channel per arm')
        selection = (motor_cfg['Serial_Type'], ports[0]['serial_id'])
        selections.append(selection)
        print(f'{name}: {path.name}, serial prefix={selection[0]}, serial_id={selection[1]}')
    if selections[0] == selections[1]:
        raise RuntimeError('Both configs select the same serial interface')

    print('\nBoth arms will get independent gravity compensation; neither follows the other.')
    print('Stop other control programs. Support both arms; keep fingers out of joints/grippers.')
    print('Ctrl+C ends recording and requests stop; support may be lost. Keep hardware cutoff ready.')
    if input('Type RECORD to initialize and start: ').strip() != 'RECORD':
        print('Cancelled before connecting to motors.')
        return 0

    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    folder = args.output.expanduser().resolve() / ('arms_' + stamp)
    folder.mkdir(parents=True, exist_ok=False)
    meta_path = folder / 'metadata.json'
    metadata = {'started_utc': stamp, 'control_hz_target': 100,
                'motor_timeout_ms_requested': 250, 'gravity_ramp_s': 1.0,
                'cameras': False, 'following': False, 'configs': config_text,
                'timestamp_note': 'Host monotonic timestamps of cached SDK reads; not sensor timestamps.',
                'units': {'position': 'rad', 'velocity': 'rad/s', 'torque': 'Nm'},
                'status': 'initializing'}
    meta_path.write_text(json.dumps(metadata, indent=2))
    robots = {}
    rows = queue.Queue(maxsize=2000)
    done = threading.Event()
    errors = []
    writer = threading.Thread(target=writer_task, args=(folder / 'joint_data.csv', rows, done, errors), daemon=True)
    writer.start()
    count = 0
    skipped_deadlines = 0
    status = 'complete'
    stop_errors = []
    start = None
    zeros = [0.0] * 6
    limits = [15.0, 30.0, 30.0, 15.0, 5.0, 5.0]
    try:
        for name, path in configs.items():
            robot = Panthera(str(path))
            robots[name] = robot
            if robot.motor_count != 6:
                raise RuntimeError(f'{name} must have six arm joints')
            robot.set_timeout(250)
            for _ in range(10):
                robot.send_get_motor_state_cmd()
                robot.motor_send_cmd()
                time.sleep(0.02)
            sample(robot)  # reject 999 sentinels/NaN before active commands

        print(f'RECORDING: {folder}\nMove each arm independently. Ctrl+C to stop.', flush=True)
        start = time.monotonic_ns()
        metadata['host_monotonic_start_ns'] = start
        metadata['host_wall_time_ns_at_start'] = time.time_ns()
        previous = start
        deadline = time.monotonic()
        while True:
            now = time.monotonic_ns()
            elapsed = (now - start) / 1e9
            if args.seconds and elapsed >= args.seconds:
                break
            if (now - previous) / 1e9 > 0.1:
                raise RuntimeError('Control gap exceeded 100 ms; stopping')
            previous = now
            if errors:
                raise RuntimeError('CSV writer failed: ' + errors[0])
            ramp = min(elapsed / 1.0, 1.0)
            for name, robot in robots.items():
                begin, end, q, dq, tau, grip = sample(robot)
                gravity = vector(robot.get_Gravity(q), 6, name + ' gravity')
                command = [max(-lim, min(lim, value * ramp)) for value, lim in zip(gravity, limits)]
                # Zero stiffness/damping: each arm receives only its own gravity torque.
                if robot.pos_vel_tqe_kp_kd(zeros, zeros, command, zeros, zeros) is False:
                    raise RuntimeError(name + ' rejected arm command')
                if robot.gripper_control_MIT(0.0, 0.0, 0.0, 0.0, 0.0) is False:
                    raise RuntimeError(name + ' rejected gripper command')
                rows.put_nowait([count, name, begin, end, (begin-start)/1e9] + q + dq + tau + command + grip)
            count += 1
            deadline += 0.01
            delay = deadline - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                skipped_deadlines += 1
                deadline = time.monotonic()
    except KeyboardInterrupt:
        status = 'stopped_by_user'
    except BaseException as exc:
        status = 'error: ' + str(exc)
        print(status, flush=True)
    finally:
        # Stop commands precede disk flushing. Watchdog remains enabled.
        for name, robot in robots.items():
            try:
                robot.set_stop()
                robot.motor_send_cmd()
            except BaseException as exc:
                stop_errors.append(name + ': ' + str(exc))
        done.set()
        writer.join(timeout=5)
        metadata.update(status=status, complete_sample_pairs=count,
                        skipped_deadlines=skipped_deadlines, writer_errors=errors,
                        writer_finished=not writer.is_alive(), stop_errors=stop_errors,
                        ended_utc=datetime.now(timezone.utc).isoformat())
        meta_path.write_text(json.dumps(metadata, indent=2))
        print(f'Ended: {status}; {count} paired samples. Files: {folder}', flush=True)
        if errors or stop_errors or writer.is_alive():
            print('CHECK ERRORS: ', errors, stop_errors, 'writer still running:', writer.is_alive())
    return 1 if status.startswith('error') or errors or stop_errors or writer.is_alive() else 0


if __name__ == '__main__':
    raise SystemExit(main())
