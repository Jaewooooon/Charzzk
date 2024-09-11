import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Point32
from ssafy_msgs.msg import TurtlebotStatus
from squaternion import Quaternion
from nav_msgs.msg import Odometry, Path
from math import pi, cos, sin, sqrt, atan2
import numpy as np # 행렬연산을 위한 모듈

class followTheCarrot(Node):

    def __init__(self):
        super().__init__('path_tracking')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # 로봇의 위치(/odom), 로봇의 상태(/turtlebot_status), 경로(/local_path)를 받아
        # 로봇의 제어 입력(/cmd_vel)을 보내주기 위한 publish, subscriber 생성
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        #print("정보가 오고 있는거 맞지? : ",self.status_callback)
        self.status_sub = self.create_subscription(TurtlebotStatus, '/turtlebot_status', self.status_callback, 10)
        self.path_sub = self.create_subscription(Path, '/local_path', self.path_callback, 10)

        time_period = 0.2
        self.timer = self.create_timer(time_period, self.timer_callback)

        self.is_odom = False
        self.is_path = False
        self.is_status = False
        self.odom_msg = Odometry()
        self.path_msg = Path()
        self.cmd_msg=Twist()
        self.robot_yaw = 0.0

        # 전방주시 거리(설정 단위는 m)
        self.lfd = 0.1
        self.min_lfd = 0.1
        self.max_lfd = 2.0

        # 포인트 초기화
        self.forward_point = None
        self.current_point = None

    # 로봇 제어 값을 계산할 타이머 콜백함수
    def timer_callback(self):

        if self.is_status and self.is_odom and self.is_path:
            if len(self.path_msg.poses) > 1:
                self.is_look_forward_point = False

                # odom으로부터 받은 x,y 좌표 저장
                robot_pose_x = self.odom_msg.pose.pose.position.x
                robot_pose_y = self.odom_msg.pose.pose.position.y
                
                # 로봇이 목표 경로로부터 떨어진 거리 계산
                lateral_error = sqrt(pow(self.path_msg.poses[0].pose.position.x - robot_pose_x, 2) + pow(self.path_msg.poses[0].pose.position.y - robot_pose_y, 2))

                # print("lateral_error: ",lateral_error)

                # 로봇의 선속도, 경로로부터 떨어진 거리를 이용해 전방 주시 거리 설정
                self.lfd = (self.status_msg.twist.linear.x + lateral_error) * 0.7
                # 계산한 전방주시거리가 최소, 최대값을 넘어갔으면 최소, 최대값으로 제한
                if self.lfd < self.min_lfd:
                    self.lfd = self.min_lfd
                elif self.lfd > self.max_lfd:
                    self.lfd = self.max_lfd
                
                # print(self.lfd)

                min_dis = float('inf')

                for num, waypoint in enumerate(self.path_msg.poses):
                    self.current_point = waypoint.pose.position
                    # 로봇과 가장 가까운 경로점과 모든 경로점과의 거리 탐색
                    dis = sqrt(pow(self.path_msg.poses[num].pose.position.x - self.current_point.x, 2) + pow(self.path_msg.poses[num].pose.position.y - self.current_point.y, 2))
                    
                    # 전방주시거리에 가장 가깝게 있는 경로점 선택
                    if abs(dis - self.lfd) < min_dis:
                        min_dis = abs(dis - self.lfd)
                        self.forward_point = self.current_point
                        self.is_look_forward_point = True

                if self.is_look_forward_point:
                    # 전방 주시 포인트를 로봇 좌표계로 변경 후, 로봇과 전방 주시 포인트와의 각도 계산
                    global_forward_point = [self.forward_point.x, self.forward_point.y, 1]

                    trans_matrix = np.array([
                        [cos(self.robot_yaw), -sin(self.robot_yaw), robot_pose_x],
                        [sin(self.robot_yaw), cos(self.robot_yaw), robot_pose_y],
                        [0, 0, 1]
                    ])
                    det_trans_matrix = np.linalg.inv(trans_matrix)
                    local_forward_point = det_trans_matrix.dot(global_forward_point)
                    #print("local_point : ",local_forward_point)
                    #print("global_point : ",global_forward_point)
                    
                    theta = -atan2(local_forward_point[1], local_forward_point[0])
                    #print("local_point, theta : ", local_forward_point[1],local_forward_point[0],theta)
                    if theta > pi:
                        theta = pi
                    elif theta < -pi:
                        theta = -pi

                    self.cmd_msg.linear.x = max(0.5, 1.0 - abs(theta))  # 각도에 따라 선속도를 조정
                    # 각속도를 theta에 2를 곱해서 사용. 클수록 더 빠르게 경로에 수렴
                    self.cmd_msg.angular.z = theta * 1.5

                    self.cmd_pub.publish(self.cmd_msg)

                    # 디버깅 로그 추가
                    self.get_logger().info(f"Setting linear.x to: {self.cmd_msg.linear.x}")
                    self.get_logger().info(f"Setting angular.z to: {self.cmd_msg.angular.z}")
                    self.get_logger().info(f"Robot pose: ({robot_pose_x}, {robot_pose_y}), Forward point: ({self.forward_point.x}, {self.forward_point.y}), Theta: {theta}")
                    self.get_logger().info(f"Calculated LFD: {self.lfd}")
                    self.get_logger().info(f"Local forward point: {local_forward_point}, Calculated theta: {theta}")


                else:
                    # 경로를 찾을 수 없을 때 정지
                    print("no found forward point")
                    self.cmd_msg.linear.x = 0.0
                    self.cmd_msg.angular.z = 0.0
                    # 계산한 제어값 publish
                    self.cmd_pub.publish(self.cmd_msg)
                    self.get_logger().info("No forward point found. Setting linear.x and angular.z to 0.")

    # odom 토픽을 받아 위치 저장. 쿼터니언을 오일러각으로 변환해서 로봇의 yaw로 사용
    def odom_callback(self, msg):
        self.is_odom = True
        self.odom_msg = msg
        q = Quaternion(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z)
        _, _, self.robot_yaw = q.to_euler()
        self.get_logger().info(f"Odom received: x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}")

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



