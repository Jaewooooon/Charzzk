import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

class Odom(Node):
    def __init__(self):
        super().__init__('odom')
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        time_period = 0.1
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = 'map'
    
    def timer_callback(self):
        # Correctly create a Quaternion object
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = 0.0
        q.w = 1.0

        # Assign the Quaternion object to the orientation field
        self.odom_msg.pose.pose.orientation = q

        # Publish the message
        self.odom_publisher.publish(self.odom_msg)
        
def main(args=None):
    rclpy.init(args=args)
    odom_node = Odom()
    rclpy.spin(odom_node)
    odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
