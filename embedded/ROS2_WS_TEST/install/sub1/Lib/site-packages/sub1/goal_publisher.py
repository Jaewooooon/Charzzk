import rclpy
import os
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import json

# 절대 경로 설정 (수동으로 파일 경로 지정)
def get_json_filepath():
    filename = "charge_command.json"
    # 절대 경로를 명시적으로 지정
    return os.path.join("C:\\Users\\SSAFY\\Desktop\\ros2_ws\\src\\sub1\\sub1", filename)

# JSON 파일에서 데이터를 불러오는 함수
def load_json_data():
    filename = get_json_filepath()  # 절대 경로 설정
    
    try:
        with open(filename, "r",encoding="utf-8") as f:
            data = json.load(f)
            print("goal_publisher success read json", flush=True)
            return data
    except FileNotFoundError:
        print("Error: JSON file not found.")
        return None

class GoalPublisher(Node):

    def __init__(self):
        super().__init__('goal_publisher')
        #print("goal publisher!!!!", flush=True)
        self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.timer = self.create_timer(1.0, self.publish_goal)

        # 맵 해상도와 오프셋 초기화
        self.map_resolution = 0.05  # 예시로 5cm 그리드
        self.map_offset_x = -2.1 - 8.75   # 예시로 맵의 x축 오프셋
        self.map_offset_y = -2.1 - 8.75   # 예시로 맵의 y축 오프셋

    def grid_cell_to_pose(self, grid_cell):
        x = self.map_offset_x + (grid_cell[0] * self.map_resolution)
        y = self.map_offset_y + (grid_cell[1] * self.map_resolution)

        return [x, y]

    def publish_goal(self):
            
        # 데이터 불러오는 로직
        data = load_json_data()
        if data is None:
            self.get_logger().error('No data found, cannot publish goal')
            return
        else:
            print("Success load data in goal pub", flush=True)

        # JSON 데이터에서 그리드 셀 좌표를 가져와서 좌표 변환
        grid_cell = [data['latitude'], data['longitude']]  # grid_x, grid_y를 JSON에서 가져옴
        x, y = self.grid_cell_to_pose(grid_cell)

        msg = PoseStamped()
        msg.header.frame_id = 'map'
        # grid 셀 좌표를 변환한 x, y를 Pose에 넣음
        msg.pose.position.x = x  # 변환된 x 좌표
        msg.pose.position.y = y  # 변환된 y 좌표
        msg.pose.orientation.w = 1.0  # 회전 정보 (여기서는 회전 없음)

        self.publisher_.publish(msg)
        print(f'Publishing goal pose: x={msg.pose.position.x}, y={msg.pose.position.y}, carNumber={data["carNumber"]}', flush=True)
        print("Let's go!!", flush=True)

def main(args=None):
    rclpy.init(args=args)
    goal_publisher = GoalPublisher()
    rclpy.spin(goal_publisher)
    goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
