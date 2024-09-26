import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from math import sqrt
import os

class pathPub(Node):

    def __init__(self):
        super().__init__('path_pub')
        # 로봇의 위치(/odom)을 받아서 전역경로(/global_path), 지역경로(/local_path)를 
        # 내보내주기 위해 publisher, subscriber 생성
        self.global_path_pub = self.create_publisher(Path, '/global_path', 10)
        self.local_path_pub = self.create_publisher(Path, '/local_path', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.listener_callback, 10)

        self.odom_msg = Odometry()
        self.is_odom = False

        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = 'odom'

        # 파일 읽어서 global path 생성
        self.f = open('C:\\Users\\SSAFY\\Desktop\\path.txt', 'r')
        lines = self.f.readlines()
        self.f.close()

        # 라인을 split 해서 x,y를 따로 나누고, global_path_msg에 append 함
        for line in lines:
            # 쉼표 제거 및 공백 제거
            tmp = line.replace(',', '').split()
            if len(tmp) == 2:
                try:
                    x = float(tmp[0])
                    y = float(tmp[1])
                    read_pose = PoseStamped()
                    read_pose.pose.position.x = x
                    read_pose.pose.position.y = y
                    read_pose.pose.orientation.w = 1.0
                    self.global_path_msg.poses.append(read_pose)
                except ValueError:
                    self.get_logger().warn(f"Skipping line due to conversion error: {line}")

        # 주기가 0.02초인 타이머 함수 설정
        time_period = 0.02
        self.timer = self.create_timer(time_period, self.timer_callback)
        # 지역경로의 경로점 개수, 경로 데이터는 0.1m 거리로 기록되어 있기 때문에 지역경로의 길이는 1.5m 정도
        self.local_path_size = 15
        self.count = 0

    # odometry 메시지 저장
    def listener_callback(self, msg):
        self.is_odom = True
        self.odom_msg = msg

    def timer_callback(self):

        local_path_msg = Path()
        local_path_msg.header.frame_id = "/odom"

        if self.is_odom:
            x = self.odom_msg.pose.pose.position.x
            y = self.odom_msg.pose.pose.position.y

            # 최소값을 저장하기 위한 변수 초기 값을 'inf'로 설정함
            min_dis = float('inf')
            current_waypoint = -1
            for i, waypoint in enumerate(self.global_path_msg.poses):
                # 전역경로점들 중에 로봇과 가장 가까운 포인트를 찾는 과정
                distance = sqrt(pow(x - waypoint.pose.position.x, 2) + pow(y - waypoint.pose.position.y, 2))
                if distance < min_dis:
                    min_dis = distance
                    current_waypoint = i

            # 로컬 경로 발행
            if current_waypoint != -1:
                if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
                    # global path의 끝을 넘어가지 않을 때
                    for num in range(current_waypoint, current_waypoint + self.local_path_size):
                        tmp_pose = PoseStamped()
                        tmp_pose.pose.position.x = self.global_path_msg.poses[num].pose.position.x
                        tmp_pose.pose.position.y = self.global_path_msg.poses[num].pose.position.y
                        tmp_pose.pose.orientation.w = 1.0
                        local_path_msg.poses.append(tmp_pose)
                else:
                    # global path의 끝을 넘어갈 때
                    for num in range(current_waypoint, len(self.global_path_msg.poses)):
                        tmp_pose = PoseStamped()
                        tmp_pose.pose.position.x = self.global_path_msg.poses[num].pose.position.x
                        tmp_pose.pose.position.y = self.global_path_msg.poses[num].pose.position.y
                        tmp_pose.pose.orientation.w = 1.0
                        local_path_msg.poses.append(tmp_pose)

        self.local_path_pub.publish(local_path_msg)

        # global path 주기적 발행
        if self.count % 10 == 0:
            self.global_path_pub.publish(self.global_path_msg)
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = pathPub()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
