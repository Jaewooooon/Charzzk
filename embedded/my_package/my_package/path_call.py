import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class GoForward(Node):
    def __init__(self):
        super().__init__('go_forward')
        # cmd_vel 토픽에 Twist 메시지를 퍼블리시하는 퍼블리셔 생성
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 주기적으로 명령을 퍼블리시하기 위한 타이머 생성 (1초마다)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Twist 메시지 생성 (선속도와 각속도를 담음)
        self.cmd_msg = Twist()
        self.cmd_msg.linear.x = 0.5  # 로봇이 앞으로 가도록 선속도 설정 (0.5 m/s)
        self.cmd_msg.angular.z = 0.0  # 각속도는 0으로 설정하여 직진만 하도록 설정
    
    def timer_callback(self):
        # 주기적으로 'cmd_vel' 토픽에 Twist 메시지를 퍼블리시
        self.cmd_pub.publish(self.cmd_msg)
        self.get_logger().info('Moving forward with linear velocity 0.5 m/s')

def main(args=None):
    rclpy.init(args=args)
    node = GoForward()  # 노드 생성
    rclpy.spin(node)     # 노드 실행
    node.destroy_node()  # 노드 종료
    rclpy.shutdown()

if __name__ == '__main__':
    main()