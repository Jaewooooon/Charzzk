import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalPublisher(Node):

    def __init__(self):
        super().__init__('goal_publisher')
        print("goal publisher!!!!")
        self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.timer = self.create_timer(1.0, self.publish_goal)

    def publish_goal(self):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = 1.0  # 원하는 x 좌표
        msg.pose.position.y = 1.0  # 원하는 y 좌표
        msg.pose.orientation.w = 1.0  # 회전 정보 (여기서는 회전 없음)

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing goal pose: x={msg.pose.position.x}, y={msg.pose.position.y}')

def main(args=None):
    rclpy.init(args=args)
    goal_publisher = GoalPublisher()
    rclpy.spin(goal_publisher)
    goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
