#!/usr/bin/env python3
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class LidarChecker(Node):
    def __init__(self, topic):
        super().__init__("lidar_checker")
        self.create_subscription(PointCloud2, topic, self.callback, 10)
        self.done = False

    def callback(self, msg: PointCloud2):
        points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False))
        arr = np.array(points, dtype=[("x", "f4"), ("y", "f4"), ("z", "f4")])
        xs = arr["x"]
        finite_mask = np.isfinite(xs) & np.isfinite(arr["y"]) & np.isfinite(arr["z"])
        n_total = len(arr)
        n_finite = int(finite_mask.sum())
        print(f"Total points: {n_total}")
        print(f"Points valides (finis): {n_finite}")
        print(f"Points invalides (inf/NaN): {n_total - n_finite}")
        if n_finite > 0:
            fx, fy, fz = xs[finite_mask], arr["y"][finite_mask], arr["z"][finite_mask]
            print(f"x range: [{fx.min():.4f}, {fx.max():.4f}]")
            print(f"y range: [{fy.min():.4f}, {fy.max():.4f}]")
            print(f"z range: [{fz.min():.4f}, {fz.max():.4f}]")
        self.done = True


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else "/lidar_1/points"
    rclpy.init()
    node = LidarChecker(topic)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=1.0)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
