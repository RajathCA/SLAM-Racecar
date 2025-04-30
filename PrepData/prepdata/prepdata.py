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
            '/luminar_left/points',
            self.callback,
            qos_profile_sensor_data
        )
        # publish deskewable cloud
        self.pub = self.create_publisher(
            PointCloud2,
            '/points_correct',
            10)
        self.get_logger().info(f"Initialized PrepData node...")

    def callback(self, msg: PointCloud2):
        print(f"Got a callback from luminar_left/points")

        # # collect unique line indices and azimuths
        # lines = set()
        # for (li,) in point_cloud2.read_points(msg,
        #                                       field_names=('line_index',),
        #                                       skip_nans=True):
        #     lines.add(int(li))

        # n_scan = len(lines)
        # # if height=1, width = total_points = n_scan * horizon_scan
        # horizon_scan = int(msg.width / n_scan)

        # self.get_logger().info(f'N_SCAN = {n_scan}')
        # self.get_logger().info(f'Horizon_SCAN ≃ {horizon_scan}')

        # rclpy.shutdown()

        # --- 1) Locate the raw timestamp field ---
        ts_field = next((f for f in msg.fields if f.name == 'timestamp'), None)
        if ts_field is None:
            self.get_logger().error("No 'timestamp' field in incoming cloud")
            return
        ts_offset = ts_field.offset      # byte offset where the 8 bytes start
        ts_count  = ts_field.count       # should be 8

        # --- 2) Read the other single‐count fields via read_points ---
        # (x,y,z,reflectance,line_index all have count==1)
        raw_pts = list(point_cloud2.read_points(
            msg,
            field_names=('x','y','z','reflectance','line_index'),
            skip_nans=False))

        # --- 3) Manually extract each point's uint64 timestamp ---
        stamps = []
        for i in range(len(raw_pts)):
            start = i * msg.point_step + ts_offset
            raw_bytes = bytes(msg.data[start : start + ts_count])
            ts = struct.unpack('<Q', raw_bytes)[0]  # nanoseconds
            stamps.append(ts)
        t0 = min(stamps)

        # --- 4) Build the new cloud exactly as before ---
        new_pts = []
        for (x, y, z, reflect, ring), ts in zip(raw_pts, stamps):
            intensity = reflect
            rel_time = (ts - t0) * 1e-9
            # print(f"relative time is {rel_time} sec")
            # print(f"Ring number is {ring}")
            new_pts.append((x, y, z, intensity, int(ring), rel_time))

        # 5) Define fields properly via attribute assignment
        fields = []
        for name, offset, datatype, count in [
            ('x',         0, PointField.FLOAT32, 1),
            ('y',         4, PointField.FLOAT32, 1),
            ('z',         8, PointField.FLOAT32, 1),
            ('intensity',12, PointField.FLOAT32, 1),
            ('ring',     16, PointField.UINT16,  1),
            ('time',     18, PointField.FLOAT32, 1),
        ]:
            f = PointField()
            f.name     = name
            f.offset   = offset
            f.datatype = datatype
            f.count    = count
            fields.append(f)

        out = point_cloud2.create_cloud(msg.header, fields, new_pts)
        self.pub.publish(out)

        # print(f"Published data")


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
