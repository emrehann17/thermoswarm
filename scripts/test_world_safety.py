"""Offline safety regression tests; fixtures are not live Gazebo evidence."""
import json
from pathlib import Path
import subprocess
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

import thermoswarm_gz_world_check as check

ROOT = Path(__file__).resolve().parents[1]


class WorldSafetyTests(unittest.TestCase):
    def setUp(self):
        self.sdf = ET.parse(ROOT / 'worlds/thermoswarm_wildfire.sdf').getroot()
        self.world = self.sdf.find('world')
        # Substitute the three includes with minimal models: no rendering/dynamics
        # are exercised by these tests. Live export uses expanded actual X500s.
        for inc in self.world.findall('include'):
            model = ET.SubElement(self.world, 'model', name=inc.findtext('name'))
            ET.SubElement(model, 'pose').text = inc.findtext('pose')
            self.world.remove(inc)
        self.models = [m.get('name') for m in self.world.findall('model')]
        self.poses = {m.get('name'): check.sdf_pose(m)[:3] for m in self.world.findall('model')}

    def evaluate(self, **kwargs):
        args = dict(models=self.models, poses=self.poses, expected=check.DEFAULT_EXPECTED,
                    min_separation_m=1.5, world='wildfire', world_sdf=self.sdf)
        args.update(kwargs)
        return check.evaluate(**args)

    def test_default_three_pass(self):
        self.assertEqual(self.evaluate(world='default', models=['ground_plane', *check.DEFAULT_EXPECTED]), [])

    def test_wildfire_three_and_eight_decor_pass(self):
        self.assertEqual(len(self.models), 12)  # 3 UAV + 8 decor + ground
        self.assertEqual(self.evaluate(), [])
        self.assertLess(2**.5, 1.5)  # patches at (+/-1,1) are legitimately exempt

    def test_default_rejects_all_wildfire_decor(self):
        problems = ' '.join(self.evaluate(world='default'))
        for name in (*check.PATCHES, *check.TREES):
            self.assertIn(name, problems)

    def test_extra_x500_zero_fails(self):
        self.models.append('x500_0')
        self.poses['x500_0'] = (0, 0, 0)
        problems = ' '.join(self.evaluate())
        self.assertIn('yetim', problems)
        self.assertIn('x500_0', problems)
        self.assertIn('cakisiyor', problems)

    def test_missing_uav_fails(self):
        for name in check.DEFAULT_EXPECTED:
            self.assertTrue(self.evaluate(models=[m for m in self.models if m != name]))

    def test_duplicate_uav_or_decor_fails(self):
        for name in ('x500_1', 'fire_patch', 'tree_1'):
            self.assertIn('tekrarlanan', ' '.join(self.evaluate(models=[*self.models, name])))

    def test_unauthorized_obstacle_at_spawn_fails(self):
        self.models.append('crate')
        self.poses['crate'] = (0, 0, 0)
        self.assertIn('crate', ' '.join(self.evaluate()))

    def test_physical_tree_at_spawn_fails(self):
        self.poses['tree_1'] = (0, 0, 0)
        self.assertIn('tree_1 ile x500_1', ' '.join(self.evaluate()))

    def test_tree_canopy_extent_preserves_clearance(self):
        self.poses['tree_1'] = (2, 0, 0)  # >1.5m from origin but <1.5m from canopy
        self.assertIn('cakisiyor', ' '.join(self.evaluate()))

    def test_other_aircraft_overlap_fails(self):
        self.poses['x500_2'] = self.poses['x500_1']
        self.assertIn('x500_1 ile x500_2', ' '.join(self.evaluate()))

    def test_invalid_world_and_threshold_fail(self):
        for world in ('unknown', '', None):
            self.assertTrue(self.evaluate(world=world))
            with mock.patch.object(check, '_gz', side_effect=AssertionError('must not query')):
                self.assertFalse(check.check_world(world=world)[0])
        for distance in (0, -1, float('nan'), float('inf')):
            self.assertTrue(self.evaluate(min_separation_m=distance))

    def test_missing_tree_or_patch_fails(self):
        for name in (*check.TREES, *check.PATCHES):
            self.assertTrue(self.evaluate(models=[m for m in self.models if m != name]))

    def test_collision_free_thermal_surface_only(self):
        for name, kelvin in check.PATCHES.items():
            model = self.world.find(f"model[@name='{name}']")
            self.assertFalse(model.findall('.//collision'))
            plugin = model.find('link/visual/plugin')
            self.assertEqual(float(plugin.findtext('temperature')), kelvin)
            collision = ET.SubElement(model.find('link'), 'collision', name='physical_surface')
            self.assertIn('collision içeriyor', ' '.join(self.evaluate()))
            model.find('link').remove(collision)

    def test_moving_surface_or_tree_is_not_allowed_static_decor(self):
        for name in ('fire_patch', 'tree_1'):
            static = self.world.find(f"model[@name='{name}']/static")
            static.text = 'false'
            self.assertTrue(self.evaluate())
            static.text = 'true'

    def test_surface_temperature_or_height_change_fails(self):
        model = self.world.find("model[@name='fire_patch']")
        model.find('link/visual/plugin/temperature').text = '300'
        self.assertTrue(self.evaluate())
        model.find('link/visual/plugin/temperature').text = '600'
        model.find('pose').text = '1 1 2 0 0 0'
        self.assertTrue(self.evaluate())

    def test_hidden_nested_collision_or_include_fails(self):
        model = self.world.find("model[@name='ambient_patch']")
        nested = ET.SubElement(model, 'model', name='hidden')
        ET.SubElement(ET.SubElement(nested, 'link'), 'collision')
        self.assertTrue(self.evaluate())
        model.remove(nested)
        ET.SubElement(model, 'include')
        self.assertTrue(self.evaluate())

    def test_missing_or_nonfinite_pose_fails(self):
        for name in ('x500_1', 'tree_1'):
            for pose in (None, (float('nan'), 0, 0), (0, 0, float('inf'))):
                self.assertTrue(self.evaluate(poses={**self.poses, name: pose}))

    def test_no_live_sdf_means_no_patch_exemption(self):
        self.assertTrue(self.evaluate(world_sdf=None))

    def fake_gz(self, args, timeout):
        if '/gazebo/worlds' in args:
            return 'data: "thermoswarm_wildfire"\n'
        if 'generate_world_sdf' in ' '.join(args):
            return 'data: ' + json.dumps(ET.tostring(self.sdf, encoding='unicode')) + '\n'
        marker = 'Requesting state for world [thermoswarm_wildfire]...\n'
        if '--list' in args:
            return marker + ''.join(f' - {name}\n' for name in self.models)
        name = args[args.index('-m') + 1]
        return marker + 'Pose [ XYZ (m) ] [ RPY (rad) ]:\n [' + ' '.join(map(str, self.poses[name])) + ']\n'

    def test_auto_and_explicit_selection_use_same_live_policy(self):
        with mock.patch.object(check, '_gz', side_effect=self.fake_gz):
            for world in ('auto', 'wildfire'):
                self.assertEqual(check.check_world(world=world, log=lambda _: None), (True, []))
            self.assertFalse(check.check_world(world='default')[0])

    def test_multiple_or_unknown_worlds_fail(self):
        for output in ('', 'data: "default"\ndata: "thermoswarm_wildfire"\n', 'data: "other"\n'):
            with mock.patch.object(check, '_gz', return_value=output):
                self.assertFalse(check.check_world()[0])

    def test_export_timeout_and_malformed_xml_fail_closed(self):
        for output in ('garbage', 'data: "<sdf>"', 'data: ""'):
            def query(args, timeout):
                if 'generate_world_sdf' in ' '.join(args):
                    return output
                return self.fake_gz(args, timeout)
            with mock.patch.object(check, '_gz', side_effect=query):
                self.assertFalse(check.check_world(log=lambda _: None)[0])

    def test_query_failure_cannot_pass(self):
        for failure in (OSError('query failed'), subprocess.TimeoutExpired('gz', 20)):
            with mock.patch.object(check, '_gz', side_effect=failure):
                self.assertIsNone(check.check_world()[0])

    def test_pose_from_different_world_fails(self):
        def query(args, timeout):
            result = self.fake_gz(args, timeout)
            return result.replace('[thermoswarm_wildfire]', '[default]') if '-p' in args else result
        with mock.patch.object(check, '_gz', side_effect=query):
            self.assertFalse(check.check_world(log=lambda _: None)[0])

    def test_nonfinite_surface_dimension_fails(self):
        self.world.find("model[@name='fire_patch']/link/visual/geometry/box/size").text = 'nan 1.2 .01'
        self.assertTrue(self.evaluate())

    def test_script_and_package_are_the_same_module(self):
        from thermoswarm_control import thermoswarm_gz_world_check as packaged
        self.assertIs(check, packaged)

    def test_launcher_startup_forwards_explicit_world(self):
        import sim_launcher
        for world in ('default', 'wildfire'):
            with mock.patch.object(sim_launcher.subprocess, 'run',
                                   return_value=mock.Mock(returncode=0, stdout='GZ CHECK PASS', stderr='')) as run:
                sim_launcher.verify_world(world)
            self.assertEqual(run.call_args.args[0][-2:], ['--world', world])


if __name__ == '__main__':
    unittest.main(verbosity=2)
