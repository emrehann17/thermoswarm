#!/usr/bin/env python3
"""Real OGRE2 render/bridge test on an isolated, elevated STATIC X500 fixture.

No PX4 or flight is started. A private Gazebo partition and ROS domain isolate
this optical test from SITL. Moving the static X500 via set_pose tests attachment,
not flight dynamics. Raw sensor images and JSON evidence are saved.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

from wildfire_assets import ROOT, WORLD, prepare, resource_env


def run_test(output, render_mode='egl'):
    prepare()
    output = output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    env = resource_env()
    env['GZ_PARTITION'] = 'thermoswarm-optics-' + uuid.uuid4().hex
    env['ROS_DOMAIN_ID'] = '88'
    env['ROS_LOG_DIR'] = str(output / 'ros-logs')
    tree = ET.parse(ROOT / 'worlds/thermoswarm_wildfire.sdf')
    for model in tree.findall('./world/include'):
        ET.SubElement(model, 'static').text = 'true'
        if model.findtext('name') == 'x500_1':
            model.find('pose').text = '0 0 5 0 0 0'
    fixture = output / 'static_optical_fixture.sdf'
    tree.write(fixture, encoding='unicode', xml_declaration=True)
    processes = []
    def launch(argv, log):
        with (output / log).open('wb') as stream:
            proc = subprocess.Popen([sys.executable, str(ROOT / 'scripts/sim_process.py'), *argv],
                                    env=env, stdout=stream, stderr=subprocess.STDOUT,
                                    stdin=subprocess.DEVNULL, start_new_session=True)
        processes.append(proc)
    def capture(transport, name):
        cmd = ['/usr/bin/python3', str(ROOT / 'scripts/thermal_check.py'), '--transport', transport,
               '--frames', '16', '--timeout', '45', '--require-hot', '--min-hz', '1',
               '--output', str(output / (name + '.png'))]
        result = subprocess.run(cmd, env=env, text=True, capture_output=True, timeout=55)
        (output / (name + '.log')).write_text(result.stdout + result.stderr)
        if result.returncode:
            raise RuntimeError(f'{name}: {result.stdout}{result.stderr}')
        return json.loads((output / (name + '.json')).read_text())
    try:
        argv = ['gz', 'sim', '-r', '-s', '--render-engine', 'ogre2']
        if render_mode == 'egl':
            argv.append('--headless-rendering')
        launch([*argv, str(fixture)], 'gazebo.log')
        launch(['/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge', '--ros-args', '-p',
                f'config_file:={ROOT / "config/thermal_bridge.yaml"}'], 'bridge.log')
        gz = capture('gz', 'elevated-gz')
        ros = capture('ros', 'elevated-ros')
        # Only our isolated STATIC fixture is repositioned. Never address live SITL.
        move = subprocess.run(['gz', 'service', '-s', f'/world/{WORLD}/set_pose',
                               '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                               '--timeout', '3000', '--req',
                               'name: "x500_1", position: {x: 0, y: 2, z: 5}, orientation: {w: 1}'],
                              env=env, capture_output=True, text=True, timeout=5)
        if move.returncode or 'data: true' not in move.stdout:
            raise RuntimeError(f'Static fixture move failed: {move.stdout}{move.stderr}')
        time.sleep(1)
        moved = capture('gz', 'translated-gz')
        shift = math.dist(gz['hot_centroid_px'], moved['hot_centroid_px'])
        if shift < 20:
            raise RuntimeError(f'Camera did not follow the X500: centroid shift {shift:.2f}px')
        report = {'pass': True, 'test_kind': 'real rendering, STATIC elevated X500; NOT a flight test',
                  'gz_partition': env['GZ_PARTITION'], 'ros_domain_id': 88, 'render_mode': render_mode,
                  'model_poses': [[0, 0, 5], [0, 2, 5]], 'camera_follows_model_shift_px': shift,
                  'gazebo': gz, 'ros': ros, 'translated_gazebo': moved}
        (output / 'render-test.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))
        print('THERMAL OPTICAL TEST PASS (not flight)')
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    proc.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait(timeout=5)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'artifacts/phase6/optics')
    parser.add_argument('--render-mode', choices=('egl', 'x11'), default='egl')
    args = parser.parse_args()
    try:
        run_test(args.output, args.render_mode)
    except (RuntimeError, subprocess.SubprocessError) as exc:
        print(f'THERMAL OPTICAL TEST FAIL: {exc}', file=sys.stderr)
        raise SystemExit(1)
