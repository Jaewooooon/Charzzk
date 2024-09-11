import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32
from sensor_msgs.msg import LaserScan, PointCloud
from math import pi, cos, sin, atan2, sqrt
import numpy as np

class LidarTrans(Node):
    def __init__(self):
        super().__init__('lidar_trans')
        self.lidar_sub = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.pcd_pub = self.create_publisher(PointCloud, 'pcd', 10)

    def lidar_callback(self, msg):
        pcd_msg = PointCloud()
        pcd_msg.header.frame_id = 'laser'

        for angle, r in enumerate(msg.ranges):
            lidar_point = Point32()

            if 0.0 < r < 12:
                # 극좌표에서 직교좌표로 변환
                lidar_point.x = r * cos(angle * pi / 180)
                lidar_point.y = r * sin(angle * pi / 180)
                pcd_msg.points.append(lidar_point)

                # 변환된 직교좌표를 다시 극좌표로 변환
                r_back, theta_back = self.cartesian_to_polar(lidar_point.x, lidar_point.y)
                self.get_logger().info(f'Polar to Cartesian: ({r}, {angle}) -> ({lidar_point.x}, {lidar_point.y}) -> Back to Polar: ({r_back}, {theta_back})')
        
        self.pcd_pub.publish(pcd_msg)

    # 극좌표 -> 직교좌표
    def polar_to_cartesian(self, r, theta):
        x = r * cos(theta)
        y = r * sin(theta)
        return x, y

    # 직교좌표 -> 극좌표 
    def cartesian_to_polar(self, x, y):
        r = sqrt(x**2 + y**2)
        theta = atan2(y, x)
        return r, theta

def main(args=None):
    rclpy.init(args=args)
    lidar_trans = LidarTrans()
    rclpy.spin(lidar_trans)
    lidar_trans.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
