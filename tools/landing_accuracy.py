#!/usr/bin/env python3
"""Landing accuracy in SITL: the drone's position at every touchdown on a die vs the die's true
centroid from the world manifest. Run it next to mission_manager started with
`-p descent_test_count:=20` (DESCENDING -> ON_DIE_TEST -> LIFTOFF -> DESCENDING ... 20 times,
then RETURNING). Prints each touchdown's offset, then the mean and worst.

  python3 tools/landing_accuracy.py                 # stops after 20 touchdowns or mission end
  python3 tools/landing_accuracy.py --count 5 --manifest worlds/avc_field_manifest.json

Needs the gnc environment (ROS 2 + the built workspace). Sends nothing.
"""
import argparse
import json
import math
import statistics
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'gnc_offboard'))   # works with or without the ROS workspace

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from avc_msgs.msg import MissionState  # noqa: E402
from px4_msgs.msg import VehicleOdometry  # noqa: E402

from gnc_offboard.field_map import default_manifest_path  # noqa: E402
from gnc_offboard.frames import FieldFrame  # noqa: E402
from gnc_offboard.mission import qos  # noqa: E402
from gnc_offboard.mission import state_names as S  # noqa: E402


class LandingAccuracy(Node):
    def __init__(self, manifest_path, count, odometry_topic):
        super().__init__('landing_accuracy')
        manifest = json.loads(Path(manifest_path).read_text())
        self.frame = FieldFrame.from_manifest(manifest)
        self.dice = [(t['color'], float(t['x']), float(t['y'])) for t in manifest['targets']]
        self.count = count
        self.odom = None
        self.state = ''
        self.offsets = []
        self.done = False
        self.create_subscription(VehicleOdometry, odometry_topic, self.on_odom, qos.PX4)
        self.create_subscription(MissionState, '/mission/state', self.on_state, qos.MISSION_STATE)
        self.get_logger().info(f'Waiting for touchdowns (ON_DIE_TEST), up to {count}...')

    def on_odom(self, msg):
        self.odom = msg

    def on_state(self, msg):
        entered = msg.state != self.state
        self.state = msg.state
        if entered and msg.state == S.ON_DIE_TEST and self.odom is not None:
            x, y, _ = self.frame.to_field(*[float(v) for v in self.odom.position])
            color = msg.active_target
            same = [(c, dx, dy) for c, dx, dy in self.dice if c == color] or self.dice
            _, cx, cy = min(same, key=lambda d: math.hypot(d[1] - x, d[2] - y))
            offset = math.hypot(x - cx, y - cy)
            self.offsets.append(offset)
            print(f'touchdown {len(self.offsets):2d}: {color} die at ({cx:.3f}, {cy:.3f}), '
                  f'drone at ({x:.3f}, {y:.3f}), offset {offset * 100:.1f} cm', flush=True)
        if len(self.offsets) >= self.count or msg.state in S.END_STATES:
            self.done = True

    def summary(self):
        if not self.offsets:
            return 'no touchdowns recorded'
        return (f'{len(self.offsets)} touchdowns: mean offset '
                f'{statistics.mean(self.offsets) * 100:.1f} cm, worst '
                f'{max(self.offsets) * 100:.1f} cm, median '
                f'{statistics.median(self.offsets) * 100:.1f} cm')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--count', type=int, default=20)
    parser.add_argument('--manifest', default=str(default_manifest_path()))
    parser.add_argument('--odometry-topic', default='/fmu/out/vehicle_odometry')
    args = parser.parse_args()
    rclpy.init()
    node = LandingAccuracy(args.manifest, args.count, args.odometry_topic)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        print(node.summary(), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
