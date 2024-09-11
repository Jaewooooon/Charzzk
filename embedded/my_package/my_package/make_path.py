import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from squaternion import Quaternion
from nav_msgs.msg import Odometry, Path
import os
from math import sqrt

class makePath(Node):

    def __init__(self):
        super().__init__('make_path')
        
        # 기록한 경로를 publish할 pubulisher 생성
        # 터틀봇의 위치를 받아서 기록하기 때문에 Odometry 매시지 sub생성
        self.path_pub = self.create_publisher(Path, 'global_path', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.listener_callback, 10)
        
        # 패키지의 path 폴더 내에 test.txt로 write할 텍스트파일 open
        pkg_path = 'C:\\Users\\SSAFY\\Desktop'  # 절대 경로로 설정
        folder_name = 'catkin_ws'
        file_name = 'test.txt'
        full_path = os.path.join(pkg_path, folder_name, file_name)
        self.f = open(full_path, 'w')
        self.is_odom = True
        self.prev_x = 0.0
        self.prev_y = 0.0

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'
    
    def listener_callback(self, msg):
        # 초기 위치 저장
        if self.is_odom == False:
            self.is_odom = True
            self.prev_x = msg.pose.pose.position.x
            self.prev_y = msg.pose.pose.position.y
        else:
            waypoint = PoseStamped()
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            yaw = msg.pose.pose.orientation.w
            # 이전 위치와 현재 위치의 거리 비교
            distance = sqrt((x - self.prev_x) ** 2 + (y - self.prev_y) ** 2)
            
            # 거리 차이가 0.1m 이상 차이가 나면 path_msg에 경로점을 추가하고 메시지 publish
            if distance > 0.1:
                waypoint.pose.position.x = x
                waypoint.pose.position.y = y
                waypoint.pose.orientation.w = yaw
                self.path_msg.poses.append(waypoint)
                self.path_pub.publish(self.path_msg)
                # 텍스트에 x,y 데이터를 write 한다. 데이터 구분은 \t
                data = '{0}\t,{1}\n'.format(x, y)
                self.f.write(data)
                # 이전 위치 저장
                self.prev_x = x
                self.prev_y = y

def main(args=None):
    rclpy.init(args=args)
    odom_based_make_path = makePath()
    rclpy.spin(odom_based_make_path)
    odom_based_make_path.f.close()
    odom_based_make_path.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
