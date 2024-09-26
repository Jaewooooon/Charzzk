import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Point32
from ssafy_msgs.msg import TurtlebotStatus
from sensor_msgs.msg import LaserScan, PointCloud
from squaternion import Quaternion
from nav_msgs.msg import Odometry, Path
from math import pi, cos, sin, sqrt, atan2
import numpy as np

class followTheCarrot(Node):

    def __init__(self):
        super().__init__('path_tracking')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 로봇의 위치(/odom), 로봇의 상태/turtlebot_status), 경로(/local_path)를 받아
        # 로봇의 제어 입력(/cmd_vel)을 보(내주기 위한 publish, subscriber 생성
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.status_sub = self.create_subscription(TurtlebotStatus, '/turtlebot_status', self.status_callback, 10)
        self.path_sub = self.create_subscription(Path, '/local_path', self.path_callback, 10)
        
        # 라이다 데이터를 수신하는 subscriber 생성
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        
        time_period = 0.1
        self.timer = self.create_timer(time_period, self.timer_callback)

        self.is_odom = False
        self.is_path = False
        self.is_status = False
        self.is_lidar = False # 라이다 메시지 수신여부 저장 변수
        self.collision = False # 장애물 충돌여부 저장 변수

        self.odom_msg = Odometry()
        self.path_msg = Path()
        self.cmd_msg = Twist()
        self.lidar_msg = LaserScan()
        self.robot_yaw = 0.0
        self.lidar_points = []

        # 전방주시 거리(설정 단위는 m)
        self.lfd = 0.1
        self.min_lfd = 0.1
        self.max_lfd = 2.0

        # 포인트 초기화
        self.forward_point = None
        self.current_point = None

        # 충돌 감지 거리
        self.collision_threshold = 0.5

    def lidar_callback(self, msg):
        # 라이다 데이터 수신 후, 극좌표 -> 직교좌표 변환
        self.lidar_msg=msg
        if self.is_path ==True and self.is_odom:
            
            pcd_msg=PointCloud()
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

    # 로봇 제어 값을 계산할 타이머 콜백함수
    def timer_callback(self):
        if self.is_status and self.is_odom and self.is_path and self.is_lidar:
            if len(self.path_msg.poses) > 1:
                self.is_look_forward_point = False

                # odom으로부터 받은 x,y 좌표 저장
                robot_pose_x = self.odom_msg.pose.pose.position.x
                robot_pose_y = self.odom_msg.pose.pose.position.y
                
                # 목적지와의 거리 계산
                goal_point = self.path_msg.poses[-1].pose.position
                goal_distance = sqrt(pow(goal_point.x - robot_pose_x, 2) + pow(goal_point.y - robot_pose_y, 2))

                # 목적지 도착 시 멈춤
                if goal_distance < 0.1:  # 임계값 설정 (10cm 이내일 때)
                    self.cmd_msg.linear.x = 0.0
                    self.cmd_msg.angular.z = 0.0
                    self.cmd_pub.publish(self.cmd_msg)
                    self.get_logger().info("Arrived at goal. Stopping robot.")
                    return
                
                # 충돌 감지
                if self.check_collision(robot_pose_x, robot_pose_y):
                    # 충돌 감지 시 정지
                    self.cmd_msg.linear.x = 0.0
                    self.cmd_msg.angular.z = 0.0
                    self.cmd_pub.publish(self.cmd_msg)
                    self.get_logger().info("Collision detected! Stopping robot.")
                    return

                # 경로 탐색 및 전방 포인트 찾기
                lateral_error = sqrt(pow(self.path_msg.poses[0].pose.position.x - robot_pose_x, 2) + pow(self.path_msg.poses[0].pose.position.y - robot_pose_y, 2))
                self.lfd = (self.status_msg.twist.linear.x + lateral_error) * 0.7
                self.lfd = max(self.min_lfd, min(self.lfd, self.max_lfd))

                min_dis = float('inf')
                for waypoint in self.path_msg.poses:
                    # 로봇과 각 waypoint 사이의 거리 계산
                    dis = sqrt(pow(waypoint.pose.position.x - robot_pose_x, 2) + pow(waypoint.pose.position.y - robot_pose_y, 2))

                    # Look-Ahead Distance(lfd)에 가장 근접한 포인트를 선택
                    if dis > self.lfd and dis < min_dis:
                        min_dis = dis
                        self.forward_point = waypoint.pose.position
                        self.is_look_forward_point = True

                if self.is_look_forward_point:
                    # 변환행렬
                    global_forward_point = [self.forward_point.x, self.forward_point.y, 1]
                    trans_matrix = np.array([
                        [cos(self.robot_yaw), -sin(self.robot_yaw), robot_pose_x],
                        [sin(self.robot_yaw), cos(self.robot_yaw), robot_pose_y],
                        [0, 0, 1]
                    ])
                    # 변환행렬의 역행렬
                    det_trans_matrix = np.linalg.inv(trans_matrix)
                    local_forward_point = det_trans_matrix.dot(global_forward_point)
                    
                    theta = -atan2(local_forward_point[1], local_forward_point[0])
                    self.cmd_msg.linear.x = max(0.55, 1.0 - abs(theta))  # 각도에 따라 선속도를 조정
                    self.cmd_msg.angular.z = theta * 1.5
                     
                    # 충돌체크 후 정지
                    if self.collision:
                        self.cmd_msg.linear.x=0.0

                    self.cmd_pub.publish(self.cmd_msg)
                
            else:
                # 경로를 찾을 수 없을 때 정지
                self.cmd_msg.linear.x = 0.0
                self.cmd_msg.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_msg)

    # odom 토픽을 받아 위치 저장. 쿼터니언을 오일러각으로 변환해서 로봇의 yaw로 사용
    def odom_callback(self, msg):
        self.is_odom = True
        self.odom_msg = msg
        q = Quaternion(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)
        _, _, self.robot_yaw = q.to_euler()

    # 경로, 상태 데이터 저장
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