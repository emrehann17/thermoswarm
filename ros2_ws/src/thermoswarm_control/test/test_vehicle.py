import unittest

from thermoswarm_control.vehicle import Aircraft, create_fleet


class VehicleModelTests(unittest.TestCase):
    def test_fleet_preserves_indices_sysids_and_altitudes(self):
        aircraft = create_fleet()
        self.assertEqual([(a.index, a.sysid, a.altitude_m) for a in aircraft], [
            (1, 2, 5.0),
            (2, 3, 8.0),
            (3, 4, 11.0),
        ])

    def test_mutable_vehicle_state_is_independent(self):
        first, second, _ = create_fleet()
        first.route.append(("A", (0.0, 0.0, 0.0, 0.0)))
        first.acks[1] = (0, 0.0)

        self.assertEqual(second.route, [])
        self.assertEqual(second.acks, {})
        self.assertIsInstance(first, Aircraft)


if __name__ == "__main__":
    unittest.main()
