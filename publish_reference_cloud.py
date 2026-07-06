#!/usr/bin/env python3
import sys
import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class ReferenceCloudPublisher(Node):
    def __init__(self, pcd_path):
        super().__init__("reference_cloud_debug_publisher")
        self.pub = self.create_publisher(
            PointCloud2, "/stylet/perception/reference_cloud_debug", 10
        )
        pcd = o3d.io.read_point_cloud(pcd_path)
        points = np.asarray(pcd.points, dtype=np.float32).tolist()

        header = Header()
        header.frame_id = "world"
        self.cloud_msg = point_cloud2.create_cloud_xyz32(header, points)

        self.timer = self.create_timer(1.0, self.publish_once)

    def publish_once(self):
        self.cloud_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.cloud_msg)
        self.get_logger().info("Nuage de référence publié.")


def main():
    rclpy.init()
    node = ReferenceCloudPublisher(sys.argv[1])
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
