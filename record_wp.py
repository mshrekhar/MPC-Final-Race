import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

class Rec(Node):
    def __init__(self):
        super().__init__('wp_recorder')
        self.f = open('/home/nvidia/f1tenth_ws/src/frenet_mpc/maps/halfmap.csv', 'w')
        self.prev = None
        self.create_subscription(PoseStamped, '/pf/viz/inferred_pose', self.cb, 10)
        print('Recording started. Drive slowly.')

    def cb(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        if self.prev is None or math.sqrt((x-self.prev[0])**2 + (y-self.prev[1])**2) > 0.1:
            self.f.write(f"{x},{y},{yaw}\n")
            self.f.flush()
            self.prev = (x, y)
            print(f"Saved: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

def main():
    rclpy.init()
    node = Rec()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.f.close()
        print('Saved.')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()