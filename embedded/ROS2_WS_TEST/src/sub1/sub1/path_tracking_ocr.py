import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Point32
from ssafy_msgs.msg import TurtlebotStatus
from sensor_msgs.msg import LaserScan, PointCloud, CompressedImage
from squaternion import Quaternion
from nav_msgs.msg import Odometry, Path
from math import pi, cos, sin, sqrt, atan2, degrees, radians
import numpy as np
import os
import subprocess
import time
import easyocr
import numpy as np
import cv2
import matplotlib.pyplot as plt
from fastapi import FastAPI
import json
from .message_producer import MessageProducer
import datetime


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
        super().__init__('path_tracking_ocr')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.status_sub = self.create_subscription(TurtlebotStatus, '/turtlebot_status', self.status_callback, 10)
        self.path_sub = self.create_subscription(Path, '/local_path', self.path_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        time_period = 0.01
        self.timer = self.create_timer(time_period, self.timer_callback)

        self.is_odom = False
        self.is_path = False
        self.is_status = False
        self.is_lidar = False
        self.collision = False
        self.turnbool = False  # 전역적으로 관리되는 변수
        self.picture_taken = False
        self.leftSide = False
        self.finish = False
        self.same_head = False
        self.Cnt = 0
        self.pictureCnt=0

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
        self.pid_controller = PIDController(0.4, 0.0, 0.07)  # kp, ki, kd 값 조정 가능
        
        
    def take_picture_and_run_ocr(self):
        # 사진 찍는 로직 (예시)
        #env = os.environ.copy()
        print("Taking picture and running OCR", flush=True)
        self.cmd_msg.angular.z = 0.0
        subprocess.Popen('start cmd /k "cd C:\\Users\\SSAFY\\Desktop\\ws && call C:\\dev\\ros2-windows\\setup.bat && call C:\\Users\\SSAFY\\Desktop\\ws\\install\\setup.bat && ros2 run sub1 picture_ocr"', shell=True)
        # 한 번 실행 후 다시 실행되지 않도록 설정
        time.sleep(5)
        return
        

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
                    if distance < 0.01:
                        self.collision = True
                        print("Collision")

            self.is_lidar = True

    def vcheck_collision(self, robot_pose_x, robot_pose_y):
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


    def load_json_data(self):
        filename_txt="charge_command.json"
        filename = os.path.join("C:\\Users\\SSAFY\\Desktop\\ros2_ws\\src\\sub1\\sub1", filename_txt)  # 절대 경로 설정
        try:
            with open(filename, "r",encoding="utf-8") as f:
                data = json.load(f)
                return data
            
        except FileNotFoundError:
            #print("Error: JSON file not found.")
            return None
        

    def timer_callback(self):
        if self.is_status and self.is_odom and self.is_path and self.is_lidar:
            if len(self.path_msg.poses) > 1:
                self.is_look_forward_point = False
                robot_pose_x = self.odom_msg.pose.pose.position.x
                robot_pose_y = self.odom_msg.pose.pose.position.y
                
                goal_point = self.path_msg.poses[-1].pose.position
                goal_distance = sqrt(pow(goal_point.x - robot_pose_x, 2) + pow(goal_point.y - robot_pose_y, 2))
                
                # if self.Cnt <3001:
                #     self.Cnt+=1
                
                
                if goal_point.y < 0:
                        if goal_point.y <-4 and goal_point.x > 6:
                            self.finish = True
                        goal_angle = -1.5708
                        self.leftSide = True
                else:
                    goal_angle = 1.5708
                    self.leftSide = False
                    
                    
                if self.robot_yaw <0:
                        if self.robot_yaw > goal_angle:
                            angleValue = abs(goal_angle) - abs(self.robot_yaw)
                        else:
                            angleValue = self.robot_yaw + abs(goal_angle)
                else:
                    if self.robot_yaw > abs(goal_angle):
                        angleValue = -(3-self.robot_yaw) + goal_angle
                    else:
                        angleValue = self.robot_yaw + abs(goal_angle)
                if angleValue > radians(180):
                    angleValue -=radians(360)
                elif angleValue < radians(-180):
                    angleValue += radians(360) 
                    
                print(f"Goal angle: {goal_angle}, Robot yaw: {self.robot_yaw}, Angle diff: {angleValue}", flush=True)
            
                if goal_distance >= 0.35:
                    self.same_head = False
                    
                    # 타임 주기
                    
                # 도착했을 때
                if goal_distance < 0.35:
                   # print("Cnt", self.Cnt,flush=True)
                    # if self.Cnt == 2000:
                    #     if self.pictureCnt==0:
                    #         print("Again go!!",flush=True)
                    #         self.take_picture_and_run_ocr()
                    #         if self.Cnt ==3000:
                    #             self.take_picture_and_run_ocr()
                    #     elif self.pictureCnt==0:
                    #         print("Again go!!",flush=True)
                    #         self.take_picture_and_run_ocr()
                         
                    
                    # 머리 방향이 맞다면
                    if self.same_head:
                        self.cmd_msg.linear.x = 0.0
                        self.cmd_msg.angular.z = 0.0
                        return
                
                    #self.get_logger().info("Arrived at goal")
                    self.cmd_msg.linear.x = 0.0
                    self.cmd_msg.angular.z = 0.0

                    #current_yaw = self.robot_yaw

                    self.turnbool = True  # 회전 시작
                    #self.target_angle = target_angle  # 목표 각도를 저장
                    self.goal_angle = goal_angle
                    
                    
                    
                # 턴이 활성화된 경우 목표 각도로 회전
                if self.turnbool:
                    
                    
                    # PID 제어 값 설정
                    if abs(angleValue) > radians(90):  # 90도 이상 차이
                        kp = 0.18
                        kd = 0.1
                    elif radians(55) < abs(angleValue) <= radians(90):
                        kp = 0.15
                        kd = 0.08
                    elif radians(40) < abs(angleValue) <= radians(55):
                        kp = 0.13
                        kd = 0.06
                    elif radians(20) < abs(angleValue) <= radians(40):
                        kp = 0.1
                        kd = 0.05
                    else:  # 20도 이내 차이
                        kp = 0.01
                        kd = 0.02

                    # angleValue를 기준으로 PID 컨트롤러 생성
                    self.pid_controller = PIDController(kp, 0.0, kd)

                    # 목표 각도에 점점 접근하도록 turnSpeed 계산
                    turnSpeed = abs(self.pid_controller.compute(angleValue, 0.05))
                    self.cmd_msg.angular.z = turnSpeed if angleValue > 0 else -turnSpeed  # 방향 설정
                    # if abs(angleValue) > radians(90):  # 20도 이상 차이가 나면
                    #     #print("20 degrees up!!!")
                    #     kp = 0.18  # 큰 PID 값으로 빠르게 회전
                    #     kd = 0.1
                    # elif radians(55) < abs(angleValue) <= radians(90):  # 5도에서 20도 사이 차이
                    #     #print("med head!!!")
                    #     kp = 0.15  # 중간 정도의 PID 값
                    #     kd = 0.08
                    # elif radians(40) < abs(angleValue) <= radians(55):  # 5도에서 20도 사이 차이
                    #     #print("med head!!!")
                    #     kp = 0.13  # 중간 정도의 PID 값
                    #     kd = 0.06
                    # elif radians(20) < abs(angleValue) <= radians(40):  # 5도에서 20도 사이 차이
                    #     #print("med head!!!")
                    #     kp = 0.1  # 중간 정도의 PID 값
                    #     kd = 0.05
                    # else:  # 5도 이내 차이
                    #     #print("small angle!!!")
                    #     kp = 0.01  # 작은 PID 값으로 미세 조정
                    #     kd = 0.02
                    
                    # print("angleValue :", angleValue, flush=True)
                    # self.pid_controller = PIDController(kp, 0.0, kd)
                    
                    # #print("angleValue : ", angleValue)
                    # #print("angleValue : ", degrees(angleValue), flush=True)
                    # turnSpeed = abs(self.pid_controller.compute(angleValue, 0.05))
                    # self.cmd_msg.angular.z = turnSpeed

                    # 회전 동작을 실행
                    self.cmd_pub.publish(self.cmd_msg)
                    #angle_dif = abs(angleValue)
                    
                    # 목표 각도에 도달하면 로봇을 정지
                    if abs(angleValue) < 0.10:  # 각도 차이가 거의 없으면
                        self.same_head = True
                        self.cmd_msg.angular.z = 0.0
                        self.cmd_msg.linear.x = 0.0
                        self.cmd_pub.publish(self.cmd_msg)
                        #time.sleep(2)
                        self.get_logger().info("Arrived at goal and oriented correctly.")
                        self.turnbool = False  # 회전 종료
                        self.pid_controller = PIDController(0.4, 0.0, 0.07)
                        if not self.finish:
                            self.cmd_msg.angular.z = 0.0
                            self.pictureCnt+=1

                            # BE에서 데이터 받아오기
                            jsondata = self.load_json_data()
                            if jsondata is None:
                                self.get_logger().error('No data found, cannot publish goal')
                                return
                            storedNumber = jsondata['carNumber']
                            print(f"jsondata: {storedNumber}")

                            message = {
                                "type": "recognition_status",
                                "status": "success",
                                "reservationId": jsondata['reservationId'],
                                "chargerId": jsondata['chargerId'],
                                "carNumber": jsondata['carNumber'],
                                "timestamp": datetime.datetime.now().isoformat()
                                }
                            self.message_producer.send_message_to_queue(message)
                            print("Send success!", flush=True)

                            if self.message_producer:
                                self.message_producer.close_connection()
                                print("RabbitMQ connection closed", flush=True)
                            self.destroy_node()  # 노드를 종료
                            print("OCR shutdown success", flush = True)



                            #self.take_picture_and_run_ocr()    
                    
                    
                
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
                    #print("angular_error : ", angular_error)
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