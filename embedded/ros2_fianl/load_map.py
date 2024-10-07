import rclpy
import numpy as np
import matplotlib.pyplot as plt  # 이미지 저장을 위한 라이브러리
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, MapMetaData

class loadMap(Node):

    def __init__(self):
        super().__init__('load_map')
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 1)
        time_period = 1  
        self.timer = self.create_timer(time_period, self.timer_callback)

        self.map_msg = OccupancyGrid()
        self.map_size_x = 420
        self.map_size_y = 420
        self.map_resolution = 0.05
        self.map_offset_x = -2.1 - 8.75
        self.map_offset_y = -2.1 - 8.75
        self.map_data = [0 for _ in range(self.map_size_x * self.map_size_y)]

        self.map_msg.header.frame_id = "map"

        m = MapMetaData()
        m.resolution = self.map_resolution
        m.width = self.map_size_x
        m.height = self.map_size_y
        m.origin = Pose()
        m.origin.position.x = self.map_offset_x
        m.origin.position.y = self.map_offset_y

        self.map_meta_data = m
        self.map_msg.info = self.map_meta_data

        # 맵 데이터 읽기
        self.f = open('C:\\Users\\SSAFY\\Desktop\\map.txt', 'r')
        lines = self.f.readlines()  # 파일의 모든 라인을 읽음
        self.f.close()

        print("load_map read success", flush=True)


        # 각 줄을 읽어 정수형 데이터로 변환
        line_data = []
        for line in lines:
            line_data.extend(list(map(int, line.split())))

        # 맵 데이터를 OccupancyGrid로 변환
        for num, data in enumerate(line_data):
            self.map_data[num] = data

        # 맵 데이터를 2D 그리드로 변환
        grid = np.array(self.map_data).reshape(self.map_size_x, self.map_size_y)

        # 값 변환 (임계값 설정)
        for y in range(self.map_size_y):
            for x in range(self.map_size_x):
                if grid[x][y] <= 20:
                    grid[x][y] = 0
                elif grid[x][y] > 20:
                    grid[x][y] = 100

        # 맵 데이터를 이미지로 저장
        #plt.imshow(grid, cmap='gray', origin='lower')  # 그리드를 이미지로 출력
        #plt.title("Map Data")
        #plt.colorbar(label='Occupancy')
        #plt.savefig('C:\\Users\\SSAFY\\Desktop\\map_image.png')  # 이미지 파일로 저장
        #plt.close()

        # OccupancyGrid 메시지로 변환
        np_map_data = grid.reshape(1, self.map_size_x * self.map_size_y)
        list_map_data = np_map_data.tolist()
        self.map_msg.data = list_map_data[0]

    def timer_callback(self):
        print("load_map timer start", flush=True)
        self.map_msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        self.map_pub.publish(self.map_msg)

def main(args=None):
    rclpy.init(args=args)
    load_map = loadMap()
    rclpy.spin(load_map)
    load_map.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
