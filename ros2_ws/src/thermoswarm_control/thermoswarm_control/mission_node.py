"""ROS 2 node lifecycle and command-line entry point for ThermoSwarm."""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node

from .mission import Mission
from .offboard import OffboardInterface


class MissionNode(Node):
    """Compose mission policy with ROS 2 lifecycle and PX4 transport."""

    def __init__(self, execute, isolate_failures=False):
        super().__init__('thermoswarm_three_x500_test')
        self.execute = execute
        self.isolate = isolate_failures
        self.mission = Mission(execute, isolate_failures, logger=self.get_logger())
        self.mission.offboard = OffboardInterface(self, self.mission.aircraft)
        self.create_timer(0.1, self.mission.tick)
        self.get_logger().info(
            'Uc X500 telemetri/preflight kontrolu (v3). ' +
            ('--execute AKTIF: hazir olduklarinda ucus baslar.' if execute
             else 'Salt-okunur mod: HICBIR ucus komutu gonderilmeyecek.') +
            (' Ariza izolasyonu: AKTIF.' if (execute and isolate_failures) else ''))


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        '--execute', action='store_true',
        help='Gazebo SITL ortaminda uc arac icin otomatik ucusu etkinlestir',
    )
    parser.add_argument(
        '--isolate-failures', action='store_true',
        help='Ariza yalnizca ilgili araci indirir; digerleri rotaya devam eder',
    )
    parser.add_argument(
        '--skip-gz-check', action='store_true',
        help='Ucus oncesi Gazebo dunya denetimini atla (onerilmez)',
    )
    args = parser.parse_args(argv)
    if args.execute and not args.skip_gz_check:
        from .thermoswarm_gz_world_check import check_world

        print('Ucus oncesi Gazebo dunya denetimi (salt-okunur)...')
        ok, problems = check_world()
        if not ok:
            for problem in problems:
                print(f'GZ CHECK FAIL: {problem}')
            print('Ucus BASLATILMADI. Gazebo/PX4 oturumunu temiz baslatip tekrar deneyin.')
            return 1
        print('GZ CHECK PASS.')

    rclpy.init()
    node = MissionNode(execute=args.execute, isolate_failures=args.isolate_failures)
    try:
        while rclpy.ok() and node.mission.phase not in ('DONE', 'FAILED'):
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        if args.execute and node.mission.phase not in ('WAIT', 'DONE', 'FAILED'):
            node.mission.safety.request_landing('Ctrl+C', abort=True)
            deadline = time.monotonic() + 5
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if node.mission.phase == 'DONE' else 1


if __name__ == '__main__':
    sys.exit(main())
