#!/usr/bin/env python3
import struct

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from rclpy.qos import qos_profile_sensor_data

# Use "ros2 run PrepData PrepData" as the command
class PrepDataNode(Node):
    def __init__(self):
        super().__init__('PrepData')
        # subscribe raw cloud
        self.sub = self.create_subscription(
            PointCloud2,
            '/luminar_front/points',
            self.callback,
            qos_profile_sensor_data
        )
        # publish reformatted cloud
        self.pub = self.create_publisher(
            PointCloud2,
            '/velodyne_points',
            10)
        self.get_logger().info(f"Initialized PrepData node...")

    def callback(self, msg: PointCloud2):
        points = point_cloud2.read_points(
            msg,
            field_names=['x', 'y', 'z', 'reflectance', 'line_index'],
            skip_nans=True
        )

        # Prepare reformatted points
        reformatted_points = []
        for p in points:
            x, y, z, reflectance, ring = p
            reformatted_points.append((x, y, z, reflectance, int(ring)))  # intensity <- reflectance, ring <- line_index

        # Define new fields as in sample rosbag
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=20, datatype=PointField.UINT16, count=1),
        ]

        # Pad to match point_step of 32 bytes
        # x: 4, y: 4, z: 4, pad 4 (12+4), intensity: 4 (16), ring: 2 (20), pad 10 more to 32
        cloud = point_cloud2.create_cloud(
            header=msg.header,
            fields=fields,
            points=reformatted_points
        )

        # cloud.is_bigendian = False
        # cloud.is_dense = True

        self.pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = PrepDataNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
