import rclpy
import numpy as np
from rclpy.node import Node
import heapq
import os
from geometry_msgs.msg import Pose, PoseStamped
from squaternion import Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData, Path
from math import pi, cos, sin
from collections import deque
import matplotlib.pyplot as plt

# a_star 노드는 OccupancyGrid map을 받아서 로봇이 목적지까지 가는 최단 경로를 생성하는 ROS2 노드입니다.
# 주로 odom (로봇 위치), map (OccupancyGrid 형식), goal_pose (목표 위치)를 받아 global_path로 최단 경로를 생성하고 퍼블리시합니다.
def show_grid(grid):
    plt.figure(figsize=(10, 10))
    plt.imshow(grid, cmap='gray_r', origin='upper')  # 그리드 데이터를 이미지로 표시 (흑백)
    plt.colorbar(label='Occupancy')  # 색상 막대 추가
    plt.title("Occupancy Grid Map")
    plt.xlabel("X (grid cells)")
    plt.ylabel("Y (grid cells)")
    plt.show()


class AStar(Node):

    def __init__(self):
        # ROS2 노드 초기화 및 부모 클래스인 Node를 초기화합니다.
        super().__init__('a_star')

        # 로직 1. publisher, subscriber 만들기
        # 'map' 토픽을 구독하고 OccupancyGrid 형식의 메시지를 수신합니다.
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 1)

        # 'odom' 토픽을 구독하고 Odometry 메시지를 수신합니다. 이 메시지는 로봇의 현재 위치를 나타냅니다.
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 1)

        # 'goal_pose' 토픽을 구독하고 목표 위치(PoseStamped)를 수신합니다.
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 1)

        # 'global_path' 토픽으로 Path 메시지를 퍼블리시합니다. 최종 경로를 퍼블리시하는 역할을 합니다.
        #self.a_star_pub = self.create_publisher(Path, 'global_path', 1)

        # 각종 메시지와 상태 변수 초기화
        self.map_msg = OccupancyGrid()  # OccupancyGrid 형식의 맵 데이터를 저장할 변수
        self.odom_msg = Odometry()  # Odometry 형식의 로봇 위치 데이터를 저장할 변수
        self.is_map = False  # 맵 데이터가 수신되었는지 여부를 나타내는 플래그
        self.is_odom = False  # odom 데이터가 수신되었는지 여부를 나타내는 플래그
        self.is_found_path = False  # 경로가 성공적으로 발견되었는지 여부를 나타내는 플래그
        self.is_grid_update = False  # 맵 데이터가 업데이트 되었는지 여부를 나타내는 플래그

        # 로직 2. 파라미터 설정
        # 기본 목표 위치를 설정 (맵의 그리드 좌표로 [184, 224])
        self.goal = [184, 224]

        # 맵의 크기 설정 (맵의 x, y 크기: 350x350)
        self.map_size_x = 420
        self.map_size_y = 420

        # 맵의 해상도 설정 (한 그리드 셀당 0.05미터)
        self.map_resolution = 0.05

        # 맵의 오프셋 설정 (맵의 좌상단 기준, 맵의 물리적 위치를 결정하는 좌표 오프셋)
        self.map_offset_x = -2.1- 8.75
        self.map_offset_y = -2.1- 8.75

        # 그리드 맵 크기 설정 (맵의 그리드 셀 수: 350x350)
        self.GRIDSIZE = 420

        # 8방향으로 이동할 때 각 좌표 변화량을 나타내는 배열 (상, 하, 좌, 우, 대각선)
        self.dx = [-1, 0, 0, 1, -1, -1, 1, 1]
        self.dy = [0, 1, -1, 0, -1, 1, -1, 1]

        # 각 이동 방향에 따른 비용 (대각선 방향의 비용이 1.414로 더 큼)
        self.dCost = [1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414]

        self.timer = self.create_timer(5.0, self.timer_callback)  # 1.0초마다 콜백 실행

    def grid_update(self):
        # 맵 데이터를 그리드 형태로 업데이트
        # self.is_grid_update = True
        # OccupancyGrid 메시지에서 map 데이터를 그리드 배열로 변환
        self.is_grid_update = True

        # 로직 3. 맵 데이터를 행렬로 바꾸기
        # OccupancyGrid의 데이터를 2차원 그리드로 변환
        width = self.map_msg.info.width  # 맵의 너비
        height = self.map_msg.info.height  # 맵의 높이
        data = self.map_msg.data  # 맵의 데이터 (1차원 배열로 제공됨)

        # OccupancyGrid 데이터는 1차원 배열로 제공되므로 이를 2차원 배열로 변환
        self.grid = np.zeros((height, width))  # 맵 크기만큼 그리드를 초기화

        for y in range(height):
            for x in range(width):
                # 1차원 데이터를 2차원 좌표로 변환
                idx = x + (y * width)
                # 데이터 값이 0 이상이면 해당 셀에 장애물 또는 탐색 불가 영역이 있다는 것을 의미
                # 만약 해당 칸이 장애물이라면 주변 2칸까지도 장애물로 설정
                if data[idx] == -1:  # -1이면 정보 없음
                    self.grid[y][x] = 100  # 장애물로 간주하여 100으로 설정
                else:
                    self.grid[y][x] = data[idx]  # 실제 맵 데이터 값으로 설정
                    
                if self.grid[y][x] == 100:
                    for i in range(len(self.dx)):  # 8 방향을 순회
                        for distance in range(1, 5):  # 1칸, 2칸씩 탐색
                            ny = y + self.dy[i] * distance
                            nx = x + self.dx[i] * distance
                            # 맵 경계 체크
                            if 0 <= ny < height and 0 <= nx < width:
                                self.grid[ny][nx] = 100  # 주변 2칸을 장애물로 설정
                    

        #self.grid = np.flipud(self.grid)
        print("Grid map updated:", self.grid, flush=True)
        #show_grid(self.grid)
        
        

    def pose_to_grid_cell(self, x, y):
        # 로봇의 물리적 위치(x, y)를 그리드 셀 좌표로 변환하는 함수
        # 로봇의 실제 좌표 (x, y)를 맵 상의 그리드 셀로 변환
        # 그리드 셀의 좌표는 맵의 좌표계에서 로봇의 좌표를 오프셋으로 이동시키고 해상도로 나누어 결정됨

        # map_point_x는 로봇의 x 좌표를 오프셋 처리하고 맵의 해상도로 나눈 값
        # 예를 들어 -8 - (-16.75) = 8.75
        # 8.75/0.05 = 175
        map_point_x = int((x - self.map_offset_x) / self.map_resolution)

        # map_point_y는 로봇의 y 좌표를 오프셋 처리하고 맵의 해상도로 나눈 값
        map_point_y = int((y - self.map_offset_y) / self.map_resolution)

        # 그리드 셀 좌표를 반환
        return map_point_x, map_point_y

    def grid_cell_to_pose(self, grid_cell):
        # 그리드 셀 좌표를 물리적 위치(x, y)로 변환하는 함수
        # 그리드로 변환 과정을 반대로 진행한다.
        # grid_cell의 0번째는 x그리드 값, 1번째는 y그리드 값
        x = self.map_offset_x + (grid_cell[0] * self.map_resolution)
        y = self.map_offset_y + (grid_cell[1] * self.map_resolution)

        return [x, y]

    def odom_callback(self, msg):
        # odom 콜백 함수: 로봇의 위치 정보를 저장하고 플래그를 True로 설정
        self.is_odom = True
        self.odom_msg = msg

    def timer_callback(self):
        # odom_msg가 초기화된 상태에서만 실행
        if self.odom_msg is not None:
            # 현재 좌표
            x = self.odom_msg.pose.pose.position.x
            y = self.odom_msg.pose.pose.position.y

            # 물리적 좌표 출력
            #print("start (physical):", [x, y])

            # 그리드 좌표로 변환해서 출력
            grid_x, grid_y = self.pose_to_grid_cell(x, y)
            #print("start (grid):", [grid_x, grid_y])

    def map_callback(self, msg):
        # 맵 콜백 함수: 맵 정보를 저장하고 플래그를 True로 설정
        self.is_map = True
        self.map_msg = msg
        self.grid_update()

    def goal_callback(self, msg):
        print("a_star goal callback", flush=True)
        # goal_pose 콜백 함수: 목표 위치를 받아 경로 탐색을 시작
        if msg.header.frame_id == 'map':
            # 로직 6. goal_pose 메시지 수신하여 목표 위치 설정
            goal_x = msg.pose.position.x  # 메시지에서 목표 위치 x 좌표 가져오기
            goal_y = msg.pose.position.y  # 메시지에서 목표 위치 y 좌표 가져오기
            goal_cell = self.pose_to_grid_cell(goal_x, goal_y)  # 목표 위치를 그리드 셀로 변환
            self.goal = goal_cell  # 목표 그리드 셀을 저장
            print("goal:", self.goal, flush = True)

            # 맵과 odom 데이터가 모두 수신되었고, 그리드가 업데이트되지 않았을 경우 그리드를 업데이트
            if self.is_map == True and self.is_odom == True:

                # 경로를 저장할 리스트 초기화
                self.final_path = []

                # 현재 로봇의 위치를 odom 데이터에서 가져와 그리드 셀로 변환
                x = self.odom_msg.pose.pose.position.x
                y = self.odom_msg.pose.pose.position.y
                start_grid_cell = self.pose_to_grid_cell(x, y)
                print("start cell test : ", start_grid_cell, flush=True)
                isAstra = False

                if self.grid[start_grid_cell[1]][start_grid_cell[0]] < 50 and start_grid_cell != self.goal:
                    self.astar(start_grid_cell)
                    isAstra = True

                if isAstra:
                    print("Astar End!!!!", flush=True)

                # Path 메시지 초기화
                self.global_path_msg = Path()
                self.global_path_msg.header.frame_id = 'map'
                #print("Final_path :", self.final_path, flush=True)

                for grid_cell in reversed(self.final_path):
                    tmp_pose = PoseStamped()
                    waypoint_x, waypoint_y = self.grid_cell_to_pose(grid_cell)
                    tmp_pose.pose.position.x = waypoint_x
                    tmp_pose.pose.position.y = waypoint_y
                    tmp_pose.pose.orientation.w = 1.0
                    self.global_path_msg.poses.append(tmp_pose)

                    #if len(self.final_path) > 0:
                        #self.a_star_pub.publish(self.global_path_msg)
                        #print("Published global path.")
                    #else:
                        #print("No path generated!")

        
        self.is_astar_running = False  # A* 실행 완료 상태로 변경

    def heuristic(self, node):
        return abs(node[0] - self.goal[0]) + abs(node[1] - self.goal[1])

    def astar(self, start):
        # print("Start:", start)
        # 경로 및 비용 배열 초기화
        self.path = [[None for _ in range(self.GRIDSIZE)] for _ in range(self.GRIDSIZE)]
        self.cost = np.full((self.GRIDSIZE, self.GRIDSIZE), np.inf)

        self.is_astar_running = True  # A* 실행 중 상태로 변경
        #print("Goal:", self.goal)

        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic(start), start))
        self.cost[start[1]][start[0]] = 0

        while open_set:
            # print("openset 길이 :", len(open_set))
            current_cost, current = heapq.heappop(open_set)

            if current == tuple(self.goal):
                print("Path found!", flush=True)
                print("goal check :", self.goal, flush=True)
                self.retrace_path(current)
                return

            for i in range(8):
                next_x = current[0] + self.dx[i]
                next_y = current[1] + self.dy[i]
                next_node = (next_x, next_y)

                if (0 <= next_x < self.GRIDSIZE) and (0 <= next_y < self.GRIDSIZE):
                    if self.grid[next_y][next_x] < 50:
                        new_cost = self.cost[current[1]][current[0]] + 1
                        if self.cost[next_y][next_x] > new_cost:
                            self.cost[next_y][next_x] = new_cost
                            priority = new_cost + self.heuristic(next_node)
                            heapq.heappush(open_set, (priority, next_node))
                            self.path[next_y][next_x] = current

    def retrace_path(self, end_node):
        node = end_node
        while node is not None:
            self.final_path.append(node)
            node = self.path[node[1]][node[0]]
        self.final_path.reverse()
        # print("Final path:", self.final_path)

        # 여기서 path.txt를 만들어 버리자
        # 경로를 path.txt 파일로 저장
        if len(self.final_path) != 0:
            with open('C:\\Users\\SSAFY\\Desktop\\ros2_ws\\src\\sub1\\sub1\\tg.txt', 'w') as f:
                print("tgfile save!!!!", flush=True)
                print("A* Success", flush=True)
                for point in self.final_path:
                    waypoint_x, waypoint_y = self.grid_cell_to_pose(point)
                    # 각 포인트를 "x,y" 형식으로 저장
                    f.write(f"{waypoint_x},{waypoint_y}\n") 
                    #f.write(f"{point[0]},{point[1]}\n")  # 각 점을 새로운 줄에 저장
                    #f.write(f"{(point[0] * 0.05) - 8.75},{(point[1] * 0.05) - 8.75}\n")  # 각 점을 새로운 줄에 저장
        else:
            print("No path!", flush=True)


def main(args=None):
    # ROS2 초기화
    rclpy.init(args=args)

    # a_star 노드 생성
    global_planner = AStar()

    # 노드 실행
    rclpy.spin(global_planner)

    # 종료 시 노드 제거 및 ROS2 종료
    global_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
