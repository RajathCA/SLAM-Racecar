#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

class KeyPoseLogger(Node):
    def __init__(self):
        super().__init__('key_pose_logger')
        self.sub = self.create_subscription(
            PointCloud2,
            '/key_pose_origin',
            self.callback,
            10
        )
        self.file = open("poses.txt", "w")
        self.get_logger().info("Subscribed to /key_pose_origin")

    def callback(self, msg: PointCloud2):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        for point in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y, z = point
            self.file.write(f"{timestamp:.9f} {x:.6f} {y:.6f} {z:.6f} 0 0 0 1\n")
        self.file.flush()
        self.get_logger().info(f"Logged {msg.width} points at time {timestamp:.3f}")

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = KeyPoseLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
