"""Derive a sensor-equipped X500 without editing the PX4 checkout or its dynamics."""
import argparse
import copy
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
WORLD = 'thermoswarm_wildfire'
TOPIC = '/thermoswarm/x500_1/thermal/image'
FRAME = 'x500_1/thermal_camera_optical'
STATIC_MODELS = ('ground_plane', 'fire_patch', 'ambient_patch', *(f'tree_{i}' for i in range(1, 7)))


def resource_env(root=ROOT):
    env = os.environ.copy()
    paths = [root / '.runtime/wildfire/models', root / 'models',
             root / 'PX4-Autopilot/Tools/simulation/gz/models', root / 'worlds']
    for key in ('GZ_SIM_RESOURCE_PATH', 'SDF_PATH'):
        env[key] = ':'.join(map(str, paths)) + (':' + env[key] if env.get(key) else '')
    env['GZ_IP'] = '127.0.0.1'
    env['GZ_CONFIG_PATH'] = '/usr/share/gz' + (':' + env['GZ_CONFIG_PATH'] if env.get('GZ_CONFIG_PATH') else '')
    return env


def derive_model(expanded, fragment):
    """Only add a sensor to base_link; verify the input has all four PX4 motors."""
    tree = ET.fromstring(expanded)
    model = tree.find('model')
    if model is None or model.find("link[@name='base_link']") is None:
        raise ValueError('Expanded PX4 X500 has no base_link')
    motors = model.findall("plugin[@name='gz::sim::systems::MulticopterMotorModel']")
    if len(motors) != 4:
        raise ValueError('Expected four unchanged PX4 motor plugins')
    link = model.find("link[@name='base_link']")
    sensors = {s.get('name') for s in link.findall('sensor')}
    if not {'imu_sensor', 'navsat_sensor', 'magnetometer_sensor', 'air_pressure_sensor'} <= sensors:
        raise ValueError('PX4 flight sensors are incomplete')
    if 'thermal_camera' in sensors:
        raise ValueError('Refusing a duplicate thermal sensor')
    sensor = ET.fromstring(fragment)
    if sensor.tag != 'sensor' or sensor.get('type') != 'thermal':
        raise ValueError('Expected a real thermal sensor fragment')
    model.set('name', 'x500_thermal')
    link.append(copy.deepcopy(sensor))
    ET.indent(tree)
    return tree


def prepare(root=ROOT, write=True):
    source = root / 'PX4-Autopilot/Tools/simulation/gz/models/x500/model.sdf'
    result = subprocess.run(['gz', 'sdf', '-p', str(source)], env=resource_env(root),
                            capture_output=True, text=True, timeout=30, check=False)
    if result.returncode or 'Error Code' in result.stderr:
        raise ValueError(f'PX4 SDF expansion failed: {result.stderr}{result.stdout[:1000]}')
    tree = derive_model(result.stdout, (root / 'models/x500_thermal/thermal_sensor.sdf').read_text())
    target = root / '.runtime/wildfire/models/x500_thermal'
    if write:
        target.mkdir(parents=True, exist_ok=True)
        temp = target / 'model.sdf.tmp'
        temp.write_text(ET.tostring(tree, encoding='unicode') + '\n')
        temp.replace(target / 'model.sdf')
        (target / 'model.config').write_text(
            '<model><name>x500_thermal</name><version>1.0</version>'
            '<sdf version="1.11">model.sdf</sdf></model>\n')
    return target / 'model.sdf'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Validate derivation without writing assets')
    args = parser.parse_args()
    print(prepare(write=not args.check))
