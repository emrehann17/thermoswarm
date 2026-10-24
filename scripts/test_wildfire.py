"""Offline checks. Synthetic byte fixtures below are unit data, never sensor evidence."""
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

import sim_launcher as sim
import thermal_check as thermal
import thermoswarm_gz_world_check as world_check
import wildfire_assets as assets

ROOT = assets.ROOT


def normalized(element):
    return (element.tag, sorted(element.attrib.items()), (element.text or '').strip(),
            tuple(normalized(child) for child in element))


class WildfireTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.world = ET.parse(ROOT / 'worlds/thermoswarm_wildfire.sdf').getroot().find('world')
        cls.sensor = ET.parse(ROOT / 'models/x500_thermal/thermal_sensor.sdf').getroot()

    def test_temperature_uses_real_thermal_plugin(self):
        for name, temperature in [('fire_patch', 600), ('ambient_patch', 300), ('ground_plane', 300)]:
            plugin = self.world.find(f"model[@name='{name}']/link/visual/plugin")
            self.assertEqual(plugin.get('filename'), 'gz-sim-thermal-system')
            self.assertEqual(float(plugin.findtext('temperature')), temperature)
        self.assertIsNone(self.world.find("model[@name='fire_patch']/link/collision"))
        self.assertIsNone(self.world.find("model[@name='ambient_patch']/link/collision"))

    def test_spawn_names_and_positions_match_fleet(self):
        fleet = sim.load_fleet()
        includes = self.world.findall('include')
        self.assertEqual(len(includes), 3)
        for inc, vehicle in zip(includes, fleet.vehicles):
            self.assertEqual(inc.findtext('name'), vehicle.name)
            pose = list(map(float, inc.findtext('pose').split()))
            self.assertEqual(pose[:2], list(vehicle.spawn_xy_m))
        self.assertEqual(includes[0].findtext('uri'), 'model://x500_thermal')
        self.assertTrue(all(inc.findtext('uri') == 'model://x500' for inc in includes[1:]))

    def test_tree_collisions_clear_all_routes(self):
        for model in self.world.findall('model'):
            if not model.get('name').startswith('tree_'):
                continue
            x, y, *_ = map(float, model.findtext('pose').split())
            # Bounding rectangle encloses all three 2 m square missions.
            dx, dy = max(0, -x, x - 12), max(0, -y, y - 2)
            self.assertGreater(math.hypot(dx, dy), 3)

    def test_camera_orientation_and_full_patch_visible_at_waypoints(self):
        pose = list(map(float, self.sensor.findtext('pose').split()))
        self.assertAlmostEqual(pose[4], math.pi / 2)
        self.assertEqual(pose[3], 0)
        self.assertEqual(pose[5], 0)
        self.assertEqual(self.sensor.get('type'), 'thermal')
        self.assertEqual(self.sensor.findtext('camera/image/format'), 'L8')
        hfov = float(self.sensor.findtext('camera/horizontal_fov'))
        # Conservative 5m altitude minus camera offset, ignoring stock +.24m.
        half_w = (5 + pose[2]) * math.tan(hfov / 2)
        half_h = half_w * 240 / 320
        for x, y in [(0, 0), (2, 0), (2, 2), (0, 2)]:
            self.assertLess(abs(1-x) + .6, half_h)
            self.assertLess(abs(1-y) + .6, half_w)

    def test_generated_model_changes_only_name_and_sensor(self):
        # Uses installed SDFormat parser and real PX4 source, without running Gazebo.
        result = subprocess.run(['gz','sdf','-p',str(ROOT/'PX4-Autopilot/Tools/simulation/gz/models/x500/model.sdf')],
                                env=assets.resource_env(), capture_output=True, text=True, check=True)
        original = ET.fromstring(result.stdout)
        derived = assets.derive_model(result.stdout, ET.tostring(self.sensor, encoding='unicode'))
        model = derived.find('model')
        sensor = model.find("link[@name='base_link']/sensor[@type='thermal']")
        self.assertIsNotNone(sensor)
        model.find("link[@name='base_link']").remove(sensor)
        model.set('name', original.find('model').get('name'))
        self.assertEqual(normalized(derived), normalized(original))

    def test_default_world_rejects_wildfire_decor(self):
        models = ['x500_1','x500_2','x500_3',*assets.STATIC_MODELS]
        poses = {f'x500_{i}': ((i-1)*5,0,0) for i in range(1,4)}
        self.assertTrue(world_check.evaluate(models, poses, list(world_check.DEFAULT_EXPECTED), 1.5))
        # A name allowlist alone must never exempt unverified physical geometry.
        self.assertTrue(world_check.evaluate(models, poses, list(world_check.DEFAULT_EXPECTED), 1.5, 'wildfire'))

    def test_wildfire_rejects_orphan_and_overlapping_x500(self):
        models = ['x500_1','x500_2','x500_3','x500_0',*assets.STATIC_MODELS]
        poses = {'x500_1':(0,0,0), 'x500_2':(0,0,0), 'x500_3':(10,0,0), 'x500_0':(15,0,0)}
        problems = world_check.evaluate(models,poses,list(world_check.DEFAULT_EXPECTED),1.5,'wildfire')
        self.assertTrue(any('x500_0' in p for p in problems))
        self.assertTrue(any('cakisi' in p for p in problems))

    def test_world_check_rejects_wrong_world(self):
        with mock.patch.object(world_check, '_gz', return_value='Requesting state for world [default]...\n - x500_1\n'):
            ok, problems = world_check.check_world(world='wildfire')
        self.assertFalse(ok)
        self.assertIn('dünya', problems[0])

    def test_check_uses_manifest_world(self):
        with mock.patch.object(sim, 'load_state', return_value={'world':'wildfire','processes':[]}), \
             mock.patch.object(sim.subprocess, 'run', return_value=mock.Mock(returncode=0)) as run:
            sim.check()
        self.assertEqual(run.call_args.args[0][-2:], ['--world','wildfire'])

    def test_dry_run_does_not_generate_or_spawn(self):
        with mock.patch.object(sim,'load_state',return_value={'processes':[]}), \
             mock.patch.object(sim,'require_dependencies'), \
             mock.patch.object(sim,'prepare_wildfire') as prepare, \
             mock.patch.object(sim,'launch_one', side_effect=AssertionError('must not launch')):
            sim.start(dry_run=True, world='wildfire')
        prepare.assert_called_once_with(ROOT, write=False)

    def test_supervisor_identity_survives_child_argv_change(self):
        proc = subprocess.Popen([sys.executable, str(ROOT/'scripts/sim_process.py'),
                                 'bash','-c','exec -a renamed-gazebo sleep 30'], start_new_session=True)
        try:
            rec = sim.process_record('supervisor-test',proc,[])
            deadline = time.monotonic() + 3
            renamed = False
            while time.monotonic() < deadline:
                children = Path(f'/proc/{proc.pid}/task/{proc.pid}/children').read_text().split()
                if any((info := sim.read_proc(int(pid))) and info['argv'][0] == 'renamed-gazebo' for pid in children):
                    renamed = True
                    break
                time.sleep(.02)
            self.assertTrue(renamed, 'child must actually change argv before ownership check')
            self.assertTrue(sim.matches(rec))
            os.killpg(proc.pid,signal.SIGINT)
            proc.wait(timeout=4)
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid,signal.SIGKILL)
                proc.wait(timeout=4)


class ImageTests(unittest.TestCase):
    def image(self, pixels=None, **kwargs):
        args = dict(width=320,height=240,encoding='mono8',step=320,
                    data=pixels or bytes([100])*76800, stamp=1.25,frame=assets.FRAME)
        args.update(kwargs)
        return thermal.unpack_image(**args)

    def test_uniform_ground_is_not_hot_evidence(self):
        self.assertFalse(thermal.contrast(self.image())['hot_visible'])

    def test_hot_contrast_metric(self):
        pixels = bytes([200])*1000 + bytes([100])*75800
        result = thermal.contrast(self.image(pixels))
        self.assertEqual(result['contrast_gray'],100)
        self.assertEqual(result['hot_pixels'],1000)
        self.assertTrue(result['hot_visible'])

    def test_rejects_bad_metadata(self):
        for kwargs in ({'width':640},{'encoding':'rgb8'},{'frame':'world'}, {'stamp':0}, {'step':319},{'data':b'bad'}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                self.image(**kwargs)

    def test_stride_padding_is_not_interpreted_as_pixels(self):
        frame = self.image(data=(bytes([100])*320+b'\xff')*240,step=321)
        self.assertEqual(set(frame['pixels']), {100})

    def test_png_is_lossless_received_pixels(self):
        import struct, zlib
        frame = self.image(bytes(range(256))*300)
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'unit-fixture-not-sensor-evidence.png'
            thermal.write_png(path,frame)
            png=path.read_bytes(); offset=8; compressed=b''
            while offset < len(png):
                length=struct.unpack('!I',png[offset:offset+4])[0]
                if png[offset+4:offset+8] == b'IDAT': compressed += png[offset+8:offset+8+length]
                offset += length+12
            raw=zlib.decompress(compressed)
            recovered=b''.join(raw[i*321+1:(i+1)*321] for i in range(240))
            self.assertEqual(recovered,frame['pixels'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
