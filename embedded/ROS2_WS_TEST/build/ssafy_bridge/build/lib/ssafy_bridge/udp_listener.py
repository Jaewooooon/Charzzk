import rclpy
from ssafy_udp_parser import erp_udp_parser  # erp_udp_parser 클래스가 정의된 파일에서 가져오기
from ssafy_msgs.msg import TurtlebotStatus

def main():
    rclpy.init()

    # 노드를 초기화하고 publisher를 생성
    node = rclpy.create_node('udp_listener')
    publisher = node.create_publisher(TurtlebotStatus, 'turtlebot_status', 10)
    
    # erp_udp_parser 인스턴스를 생성하여 UDP 데이터를 수신하도록 설정
    ip = '127.0.0.1'  # 시뮬레이터 IP로 변경 필요
    port = 12345          # 사용할 포트 번호로 변경 필요
    parser = erp_udp_parser(publisher, ip, port, 'turtlebot_status')  # 데이터 타입에 맞게 'imu', 'app_status' 등 선택

    try:
        rclpy.spin(node)  # ROS2 노드를 실행하여 데이터 수신 대기
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
