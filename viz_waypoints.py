#!/usr/bin/env python3
"""
Publishes waypoints from CSV as visualization markers for Foxglove.
Usage: python3 viz_waypoints.py
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import csv

WAYPOINT_FILE = "/home/nvidia/f1tenth_ws/src/centerline.csv"

class WaypointViz(Node):
    def __init__(self):
        super().__init__('waypoint_viz')

        self.pub = self.create_publisher(Marker, '/raceline', 10)
        self.timer = self.create_timer(1.0, self.publish_waypoints)

        # Load waypoints
        self.points = []
        with open(WAYPOINT_FILE, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) >= 2:
                    try:
                        self.points.append((float(row[0]), float(row[1])))
                    except ValueError:
                        continue

        self.get_logger().info(f"Loaded {len(self.points)} waypoints")

    def publish_waypoints(self):
        # Line strip connecting all waypoints
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "waypoints"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # line width
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        for x, y in self.points:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)

        # Close the loop
        if self.points:
            p = Point()
            p.x = self.points[0][0]
            p.y = self.points[0][1]
            p.z = 0.0
            marker.points.append(p)

        self.pub.publish(marker)

def main():
    rclpy.init()
    node = WaypointViz()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()