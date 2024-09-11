import rclpy
from rclpy.node import Node
from ssafy_msgs.msg import TurtlebotStatus  # 터틀봇 메시지를 사용하기 위해 import
from squaternion import Quaternion  # 쿼터니언에서 오일러각, 오일러각에서 쿼터니언으로 변환하기 위해 import
from nav_msgs.msg import Odometry
from math import pi, cos, sin
import tf2_ros      # tf2_ros에 broadcaster를 사용하기 위해 import
import geometry_msgs.msg    # broadcast 할 메시지를 담고 있는 geometry_msgs 사용을 위해 import

class odom(Node):

    def __init__(self):
        super().__init__('odom')
        # 터틀봇의 상태 메시지(선,각속도)를 받아서 odometry를 계산 -> 내보내주기를 위해 pub과 sub 생성
        self.status_sub = self.create_subscription(TurtlebotStatus, '/turtlebot_status', self.status_callback, 100)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 100)       # 오도메트리 데이터 발행

        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.odom_msg = Odometry()

        # broadcast 할 좌표계 메시지 생성
        self.base_link_transform = geometry_msgs.msg.TransformStamped()
        self.laser_transform = geometry_msgs.msg.TransformStamped()

        self.is_status = False
        self.is_calc_theta = False

        # 사용할 변수 초기화
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_time = 0.0

        # odometry 메시지에 좌표계 이름을 적어줌. map 좌표계 위에 odometry가 그려짐
        self.odom_msg.header.frame_id = 'map'
        self.odom_msg.child_frame_id = 'base_link'

        # map -> base_link 좌표계 이름 설정
        self.base_link_transform = geometry_msgs.msg.TransformStamped()
        self.base_link_transform.header.frame_id = 'map'
        self.base_link_transform.child_frame_id = 'base_link'

        # base_link -> laser 좌표계 이름 설정
        self.laser_transform = geometry_msgs.msg.TransformStamped()
        self.laser_transform.header.frame_id = 'base_link'
        self.laser_transform.child_frame_id = 'laser'

        # base_link로부터 laser 좌표계가 이동(translation)된 값 입력. 터틀 봇으로부터 1m 위에 달려있다는 것을 의미
        self.laser_transform.transform.translation.x = 0.0
        self.laser_transform.transform.translation.y = 0.0
        self.laser_transform.transform.translation.z = 0.11

        # 쿼터니언의 x,y,z = 0이고, w=1 이면 회전이 없다는 것을 의미
        # 즉, base_link로부터 laser 좌표계는 회전되어 있지 않음
        # 생성함수(__init__)에 넣은 이유는 센서의 위치는 바뀌지 않기 때문
        self.laser_transform.transform.rotation.w = 1.0

    def status_callback(self, msg):

        # 상태 메시지가 처음 들어왔을 때는 들어온 시간만 저장
        if self.is_status == False:
            self.is_status = True
            self.prev_time = rclpy.clock.Clock().now()
        else:
            # 적분을 위해 이전 들어온 데이터와 현재 들어온 데이터 사이의 시간 측정
            self.current_time = rclpy.clock.Clock().now()
            self.period = (self.current_time - self.prev_time).nanoseconds / 1000000000
            # 수신한 메시지에서 선,각속도 저장
            # 각속도는 방향이 반대이기 때문에 마이너스를 붙여줌
            linear_x = msg.twist.linear.x
            angular_z = -msg.twist.angular.z
            # x,y,theta를 계산
            # 위의 이론 부분에 odometry 계산하는 수식과 동일
            self.x += linear_x * cos(self.theta) * self.period
            self.y += linear_x * sin(self.theta) * self.period
            self.theta += angular_z * self.period
            
            # 계산한 theta = 오일러각
            # 쿼터니언으로 변환
            q = Quaternion.from_euler(0, 0, self.theta)

            # 좌표계를 broadcast 할 때는 시간을 꼭 넣어줘야 하기 때문에 시간을 넣어줌
            self.base_link_transform.header.stamp = rclpy.clock.Clock().now().to_msg()
            self.laser_transform.header.stamp = rclpy.clock.Clock().now().to_msg()


             # Transform 이전 base_link 값 로그 출력
            print(f"Before Transform - base_link Translation: x={self.base_link_transform.transform.translation.x}, "
                f"y={self.base_link_transform.transform.translation.y}")
            print(f"Before Transform - base_link Rotation: x={self.base_link_transform.transform.rotation.x}, "
                f"y={self.base_link_transform.transform.rotation.y}, "
                f"z={self.base_link_transform.transform.rotation.z}, "
                f"w={self.base_link_transform.transform.rotation.w}")


            # 계산한 x,y가 이동(translation) 값이 됨
            self.base_link_transform.transform.translation.x = self.x
            self.base_link_transform.transform.translation.y = self.y
            # self.base_link_transform.transform.translation.z = 0.0

            # 계산한 q가 회전(rotation) 값이 됨
            self.base_link_transform.transform.rotation.x = q.x
            self.base_link_transform.transform.rotation.y = q.y
            self.base_link_transform.transform.rotation.z = q.z
            self.base_link_transform.transform.rotation.w = q.w

            # Transform 이후 base_link 값 로그 출력
            print(f"After Transform - base_link Translation: x={self.base_link_transform.transform.translation.x}, "
                f"y={self.base_link_transform.transform.translation.y}")
            print(f"After Transform - base_link Rotation: x={self.base_link_transform.transform.rotation.x}, "
                f"y={self.base_link_transform.transform.rotation.y}, "
                f"z={self.base_link_transform.transform.rotation.z}, "
                f"w={self.base_link_transform.transform.rotation.w}")

            # map 값 로그 출력 (고정된 값)
            print(f"Map Frame - Reference Position: (0, 0, 0) and Orientation: (0, 0, 0, 1)")
            print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")


            # odometry 메시지도 이동, 회전, 제어 값을 채움
            self.odom_msg.pose.pose.position.x = self.x
            self.odom_msg.pose.pose.position.y = self.y
            #self.odom_msg.pose.pose.position.z = 0.0
            
            self.odom_msg.pose.pose.orientation.x = q.x
            self.odom_msg.pose.pose.orientation.y = q.y
            self.odom_msg.pose.pose.orientation.z = q.z
            self.odom_msg.pose.pose.orientation.w = q.w
            self.odom_msg.twist.twist.linear.x = linear_x
            self.odom_msg.twist.twist.angular.z = angular_z

            # 좌표계를 broadcast하고 odometry메시지를 pub 함
            self.broadcaster.sendTransform(self.base_link_transform)
            self.broadcaster.sendTransform(self.laser_transform)

            self.odom_publisher.publish(self.odom_msg)
            self.prev_time = self.current_time

def main(args=None):
    rclpy.init(args=args)
    node = odom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()