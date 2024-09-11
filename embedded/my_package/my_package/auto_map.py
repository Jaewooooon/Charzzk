import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Point32
from nav_msgs.msg import Odometry, Path
from ssafy_msgs.msg import TurtlebotStatus
from sensor_msgs.msg import LaserScan
from squaternion import Quaternion
from math import pi, cos, sin, sqrt, atan2
import numpy as np
import os

class followTheCarrot(Node):

    def __init__(self):
        super().__init__('path_tracking')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, 'new_path', 10)  # 경로를 기록한 후 publish 할 publisher
        
        # 로봇의 위치(/odom), 로봇의 상태(/turtlebot_status), 경로(/local_path)를 받아
        # 로봇의 제어 입력(/cmd_vel)을 보내주기 위한 publish, subscriber 생성
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.status_sub = self.create_subscription(TurtlebotStatus, '/turtlebot_status', self.status_callback, 10)
        self.path_sub = self.create_subscription(Path, '/local_path', self.path_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        
        time_period = 0.1
        self.timer = self.create_timer(time_period, self.timer_callback)
        
        # 초기 변수 설정
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

        # 경로 기록을 위한 변수
        self.previous_position = None
        self.path_recording = Path()  # 기록된 경로를 저장할 Path 메시지
        self.path_file = open(os.path.join(os.getcwd(), 'path', 'test.txt'), 'w')  # 기록할 경로 파일 오픈

    def lidar_callback(self, msg):
        # 라이다 데이터 수신 후, 극좌표 -> 직교좌표 변환
        self.lidar_msg=msg
        if self.is_path ==True and self.is_odom:
            
            pcd_msg = PointCloud()
            pcd_msg.header.frame_id='map'

            pose_x = self.odom_msg.pose.pose.position.x
            pose_y = self.odom_msg.pose.pose.position.y
            theta = self.robot_yaw

            # 변환행렬 생성
            t = np.array([
            [cos(theta), -sin(theta),pose_x],
            [sin(theta),  cos(theta),pose_y],
            [0, 0, 1]
        ])
            
            for angle, r in enumerate(msg.ranges):
                global_point = Point32()

                if 0.0 < r < 12.0:  # 유효 범위 내의 거리만 처리
                    # 극좌표 -> 직교좌표 변환
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
        # 경로점과 라이다 포인트 간의 거리 비교로 충돌 여부 확인
        if not self.is_lidar or not self.is_path:
            return False
        
        for lidar_point in self.lidar_points:
            # 라이다 좌표를 글로벌 좌표로 변환
            local_point = np.array([lidar_point[0], lidar_point[1], 1])
            trans_matrix = np.array([
                [cos(self.robot_yaw), -sin(self.robot_yaw), robot_pose_x],
                [sin(self.robot_yaw), cos(self.robot_yaw), robot_pose_y],
                [0, 0, 1]
            ])
            global_point = trans_matrix.dot(local_point)

            # 경로점과 비교하여 충돌 여부 확인
            for waypoint in self.path_msg.poses:
                wp_x = waypoint.pose.position.x
                wp_y = waypoint.pose.position.y
                distance = sqrt((wp_x - global_point[0]) ** 2 + (wp_y - global_point[1]) ** 2)
                if distance < self.collision_threshold:
                    self.get_logger().info(f"Collision detected! Distance to waypoint: {distance}")
                    return True
        
        return False
    
    # 터틀봇의 현재 위치를 기록하는 함수. 일정 거리 이상 이동 시에만 경로를 기록하고 파일에 저장.
    def record_path(self):
        current_position = self.odom_msg.pose.pose.position
        if self.previous_position is None:
            self.previous_position = current_position
            return

        # 이전 위치와 현재 위치 간 거리 계산
        distance = sqrt(
            pow(current_position.x - self.previous_position.x, 2) +
            pow(current_position.y - self.previous_position.y, 2)
        )

        if distance > 0.1:  # 10cm 이상 이동한 경우에만 경로점 추가
            self.previous_position = current_position

            # 경로 메시지에 새로운 경로점 추가
            path_pose = self.odom_msg.pose
            self.path_recording.poses.append(path_pose)

            # 경로를 publish
            self.path_pub.publish(self.path_recording)

            # 경로를 텍스트 파일에 기록 (탭으로 구분)
            self.path_file.write(f"{current_position.x}\t{current_position.y}\n")

            # 로깅
            self.get_logger().info(f"Path recorded at position: ({current_position.x}, {current_position.y})")

    # 기록된 경로를 불러와서 경로 추종 알고리즘을 실행하는 함수
    def load_path(self):
        with open(os.path.join(os.getcwd(), 'path', 'path.txt'), 'r') as file:
            path_msg = Path()
            for line in file:
                x, y = map(float, line.strip().split('\t'))
                pose = self.odom_msg.pose  # 현재 odom 메시지를 사용하여 경로점을 생성
                pose.position.x = x
                pose.position.y = y
                path_msg.poses.append(pose)

            self.path_msg = path_msg
            self.is_path = True
            self.get_logger().info(f"Loaded path with {len(self.path_msg.poses)} waypoints")

    def timer_callback(self):
        # 경로 기록 함수 호출
        if self.is_odom and not self.is_path:
            self.record_path()

        # 경로가 있으면 경로 추종
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
                    theta = -atan2(local_forward_point[1], local_forward_point[0])
                    self.cmd_msg.linear.x = max(0.5, 1.0 - abs(theta))
                    self.cmd_msg.angular.z = theta * 1.5
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
