import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Point32
from ssafy_msgs.msg import TurtlebotStatus
from sensor_msgs.msg import LaserScan, PointCloud
from squaternion import Quaternion
from nav_msgs.msg import Odometry, Path
from math import pi, cos, sin, sqrt, atan2
import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class followTheCarrot(Node):
    def __init__(self):
        super().__init__('path_tracking_PID')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.status_sub = self.create_subscription(TurtlebotStatus, '/turtlebot_status', self.status_callback, 10)
        self.path_sub = self.create_subscription(Path, '/local_path', self.path_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        time_period = 0.1
        self.timer = self.create_timer(time_period, self.timer_callback)

        self.is_odom = False
        self.is_path = False
        self.is_status = False
        self.is_lidar = False
        self.collision = False

        self.odom_msg = Odometry()
        self.path_msg = Path()
        self.cmd_msg = Twist()
        self.lidar_msg = LaserScan()
        self.robot_yaw = 0.0
        self.lidar_points = []

        self.lfd = 0.1
        self.min_lfd = 0.1
        self.max_lfd = 2.0
        self.forward_point = None
        self.current_point = None
        self.collision_threshold = 0.5

        # PID 컨트롤러 초기화
        self.pid_controller = PIDController(1.0, 0.0, 0.1)  # kp, ki, kd 값 조정 가능

    def lidar_callback(self, msg):
        self.lidar_msg = msg
        if self.is_path and self.is_odom:
            pcd_msg = PointCloud()
            pcd_msg.header.frame_id = 'map'

            pose_x = self.odom_msg.pose.pose.position.x
            pose_y = self.odom_msg.pose.pose.position.y
            theta = self.robot_yaw

            t = np.array([
                [cos(theta), -sin(theta), pose_x],
                [sin(theta), cos(theta), pose_y],
                [0, 0, 1]
            ])

            for angle, r in enumerate(msg.ranges):
                global_point = Point32()
                if 0.0 < r < 12.0:
                    rad_angle = np.radians(angle)
                    local_x = r * cos(rad_angle)
                    local_y = r * sin(rad_angle)
                    local_point = np.array([[local_x], [local_y], [1]])
                    global_result = t.dot(local_point)
                    global_point.x = global_result[0][0]
                    global_point.y = global_result[1][0]
                    pcd_msg.points.append(global_point)

            self.collision = False
            for waypoint in self.path_msg.poses:
                for lidar_point in pcd_msg.points:
                    distance = sqrt(pow(waypoint.pose.position.x - lidar_point.x, 2) + pow(waypoint.pose.position.y - lidar_point.y, 2))
                    if distance < 0.1:
                        self.collision = True
                        print("Collision")

            self.is_lidar = True

    def check_collision(self, robot_pose_x, robot_pose_y):
        if not self.is_lidar or not self.is_path:
            return False
        
        for lidar_point in self.lidar_points:
            local_point = np.array([lidar_point[0], lidar_point[1], 1])
            trans_matrix = np.array([
                [cos(self.robot_yaw), -sin(self.robot_yaw), robot_pose_x],
                [sin(self.robot_yaw), cos(self.robot_yaw), robot_pose_y],
                [0, 0, 1]
            ])
            global_point = trans_matrix.dot(local_point)

            for waypoint in self.path_msg.poses:
                wp_x = waypoint.pose.position.x
                wp_y = waypoint.pose.position.y
                distance = sqrt((wp_x - global_point[0]) ** 2 + (wp_y - global_point[1]) ** 2)
                if distance < self.collision_threshold:
                    self.get_logger().info(f"Collision detected! Distance to waypoint: {distance}")
                    return True
        
        return False

    def timer_callback(self):
        if self.is_status and self.is_odom and self.is_path and self.is_lidar:
            if len(self.path_msg.poses) > 1:
                self.is_look_forward_point = False
                robot_pose_x = self.odom_msg.pose.pose.position.x
                robot_pose_y = self.odom_msg.pose.pose.position.y
                
                goal_point = self.path_msg.poses[-1].pose.position
                goal_distance = sqrt(pow(goal_point.x - robot_pose_x, 2) + pow(goal_point.y - robot_pose_y, 2))

                if goal_distance < 0.1:
                    self.cmd_msg.linear.x = 0.0
                    self.cmd_msg.angular.z = 0.0
                    self.cmd_pub.publish(self.cmd_msg)
                    self.get_logger().info("Arrived at goal. Stopping robot.")
                    return

                if self.check_collision(robot_pose_x, robot_pose_y):
                    self.cmd_msg.linear.x = 0.0
                    self.cmd_msg.angular.z = 0.0
                    self.cmd_pub.publish(self.cmd_msg)
                    self.get_logger().info("Collision detected! Stopping robot.")
                    return

                lateral_error = sqrt(pow(self.path_msg.poses[0].pose.position.x - robot_pose_x, 2) + pow(self.path_msg.poses[0].pose.position.y - robot_pose_y, 2))
                self.lfd = (self.status_msg.twist.linear.x + lateral_error) * 0.7
                self.lfd = max(self.min_lfd, min(self.lfd, self.max_lfd))

                min_dis = float('inf')
                for waypoint in self.path_msg.poses:
                    dis = sqrt(pow(waypoint.pose.position.x - robot_pose_x, 2) + pow(waypoint.pose.position.y - robot_pose_y, 2))
                    if dis > self.lfd and dis < min_dis:
                        min_dis = dis
                        self.forward_point = waypoint.pose.position
                        self.is_look_forward_point = True

                if self.is_look_forward_point:
                    global_forward_point = [self.forward_point.x, self.forward_point.y, 1]
                    trans_matrix = np.array([
                        [cos(self.robot_yaw), -sin(self.robot_yaw), robot_pose_x],
                        [sin(self.robot_yaw), cos(self.robot_yaw), robot_pose_y],
                        [0, 0, 1]
                    ])
                    det_trans_matrix = np.linalg.inv(trans_matrix)
                    local_forward_point = det_trans_matrix.dot(global_forward_point)
                    
                    angular_error = -atan2(local_forward_point[1], local_forward_point[0])
                    dt = 0.1  # 타이머 주기
                    self.cmd_msg.angular.z = self.pid_controller.compute(angular_error, dt)
                    self.cmd_msg.linear.x = max(0.2, 1.0 - abs(angular_error))  # 각도에 따라 선속도 조정

                    if self.collision:
                        self.cmd_msg.linear.x = 0.0

                    self.cmd_pub.publish(self.cmd_msg)

            else:
                self.cmd_msg.linear.x = 0.0
                self.cmd_msg.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_msg)

    def odom_callback(self, msg):
        self.is_odom = True
        self.odom_msg = msg
        q = Quaternion(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)
        _, _, self.robot_yaw = q.to_euler()

    def path_callback(self, msg):
        self.is_path = True
        self.path_msg = msg

    def status_callback(self, msg):
        self.is_status = True
        self.status_msg = msg

def main(args=None):
    rclpy.init(args=args)
    node = followTheCarrot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
