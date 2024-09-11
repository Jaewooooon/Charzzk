import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ssafy_msgs.msg import TurtlebotStatus

class Communicator(Node):

    def __init__(self):
        super().__init__('communication')
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.turtlebot_status_sub = self.create_subscription(TurtlebotStatus, 'turtlebot_status', self.status_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.cmd_msg = Twist()
        self.turtlebot_status_msg = None() # 초기화

    def status_callback(self, msg):
        self.turtlebot_status_msg = msg()
        self.get_logger().info(f"Received Turtlebot Status: Linear Velocity: {msg.twist.linear.x}, Angular Velocity: {msg.twist.angular.z}")

    def timer_callback(self):
        if self.turtlebot_status_msg:  # 상태 메시지가 들어왔을 때만 제어 값 설정
            # Turtlebot의 현재 선속도를 받아서 그에 맞춰 제어
            current_linear_velocity = self.turtlebot_status_msg.twist.linear.x
            current_angular_velocity = self.turtlebot_status_msg.twist.angular.z

            # 선속도를 동적으로 설정 (현재 속도가 낮으면 속도를 올림)
            self.cmd_msg.linear.x = max(0.5, current_linear_velocity + 0.1)
            # 각속도도 현재 상태에 따라 조정
            self.cmd_msg.angular.z = max(0.1, current_angular_velocity + 0.05)

            # 수정된 제어 값을 퍼블리시
            self.cmd_publisher.publish(self.cmd_msg)

            # 디버그 로그 출력
            self.get_logger().info(f"Publishing cmd_vel: Linear: {self.cmd_msg.linear.x}, Angular: {self.cmd_msg.angular.z}")
        else:
            # 상태 메시지가 없으면 기본값 유지
            self.get_logger().info("No Turtlebot status received yet. Waiting...")

def main(args=None):
    rclpy.init(args=args)
    com = Communicator()
    rclpy.spin(com)
    com.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()