"""Fleet YAML schema and fail-closed configuration tests."""

from pathlib import Path
import tempfile
import unittest
from unittest import mock

import yaml

from thermoswarm_control.config import ConfigError, load_config
from thermoswarm_control import mission_node
from thermoswarm_control.vehicle import create_fleet


class FleetConfigTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config_path = load_config().source
        cls.raw = yaml.safe_load(cls.config_path.read_text(encoding="utf-8"))

    def write_config(self, mutate=None):
        raw = yaml.safe_load(yaml.safe_dump(self.raw))
        if mutate:
            mutate(raw)
        handle = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", encoding="utf-8", delete=False)
        self.addCleanup(Path(handle.name).unlink, missing_ok=True)
        with handle:
            yaml.safe_dump(raw, handle)
        return handle.name

    def test_default_yaml_preserves_verified_fleet_and_route(self):
        config = load_config()
        fleet = create_fleet(config)
        self.assertEqual(
            [(a.px4_instance, a.sysid, a.namespace, a.model, a.altitude_m, a.spawn_xy_m)
             for a in fleet],
            [(1, 2, "/px4_1", "x500", 5.0, (0.0, 0.0)),
             (2, 3, "/px4_2", "x500", 8.0, (5.0, 0.0)),
             (3, 4, "/px4_3", "x500", 11.0, (10.0, 0.0))],
        )
        self.assertEqual(
            [(p.name, p.north_m, p.east_m) for p in config.route],
            [("KALKIS", 0.0, 0.0), ("KUZEY_2M", 2.0, 0.0),
             ("DOGU_2M", 2.0, 2.0), ("GUNEY_2M", 0.0, 2.0),
             ("EVE_DON", 0.0, 0.0)],
        )

    def test_duplicate_sysid_and_namespace_are_rejected(self):
        for field in ("system_id", "namespace"):
            with self.subTest(field=field):
                path = self.write_config(lambda raw: raw["vehicles"][1].__setitem__(
                    field, raw["vehicles"][0][field]))
                with self.assertRaisesRegex(ConfigError, "repeated"):
                    load_config(path)

    def test_missing_field_invalid_type_and_negative_altitude_are_rejected(self):
        mutations = (
            lambda raw: raw["vehicles"][0].pop("system_id"),
            lambda raw: raw["vehicles"][0].__setitem__("px4_instance", True),
            lambda raw: raw["vehicles"][0].__setitem__("takeoff_altitude_m", -1),
        )
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                with self.assertRaises(ConfigError):
                    load_config(self.write_config(mutate))

    def test_unsupported_fleet_size_is_rejected(self):
        path = self.write_config(lambda raw: raw["vehicles"].pop())
        with self.assertRaisesRegex(ConfigError, "exactly 3"):
            load_config(path)

    def test_unavailable_model_and_bad_safety_type_are_rejected(self):
        for mutate in (
            lambda raw: raw["vehicles"][0].__setitem__("model", "x500_v2"),
            lambda raw: raw["mission"]["safety"].__setitem__("leg_timeout_s", "40"),
        ):
            with self.subTest(mutate=mutate):
                with self.assertRaises(ConfigError):
                    load_config(self.write_config(mutate))

    def test_missing_file_has_actionable_error(self):
        with self.assertRaisesRegex(ConfigError, "cannot read fleet config"):
            load_config("/tmp/thermoswarm-does-not-exist.yaml")

    def test_invalid_config_exits_before_ros_or_gazebo_in_audit_and_execute(self):
        for execute in (False, True):
            args = ["--config", "/tmp/thermoswarm-does-not-exist.yaml"]
            if execute:
                args.append("--execute")
            with self.subTest(execute=execute), \
                    mock.patch.object(mission_node.rclpy, "init") as init, \
                    mock.patch(
                        "thermoswarm_control.thermoswarm_gz_world_check.check_world"
                    ) as gz_check:
                self.assertEqual(mission_node.main(args), 2)
                init.assert_not_called()
                gz_check.assert_not_called()


if __name__ == "__main__":
    unittest.main()
