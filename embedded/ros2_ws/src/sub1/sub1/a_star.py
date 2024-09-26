import rclpy
import numpy as np
from rclpy.node import Node
import os
from geometry_msgs.msg import Pose, PoseStamped
from squaternion import Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData, Path
from math import pi, cos, sin
from collections import deque

# a_star 노드는 OccupancyGrid map을 받아서 로봇이 목적지까지 가는 최단 경로를 생성하는 ROS2 노드입니다.
# 주로 odom (로봇 위치), map (OccupancyGrid 형식), goal_pose (목표 위치)를 받아 global_path로 최단 경로를 생성하고 퍼블리시합니다.

class a_star(Node):

    def __init__(self):
        # ROS2 노드 초기화 및 부모 클래스인 Node를 초기화합니다.
        super().__init__('a_Star')
        
        # 로직 1. publisher, subscriber 만들기
        # 'map' 토픽을 구독하고 OccupancyGrid 형식의 메시지를 수신합니다.
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 1)
        
        # 'odom' 토픽을 구독하고 Odometry 메시지를 수신합니다. 이 메시지는 로봇의 현재 위치를 나타냅니다.
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 1)
        
        # 'goal_pose' 토픽을 구독하고 목표 위치(PoseStamped)를 수신합니다.
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 1)
        
        # 'global_path' 토픽으로 Path 메시지를 퍼블리시합니다. 최종 경로를 퍼블리시하는 역할을 합니다.
        self.a_star_pub = self.create_publisher(Path, 'global_path', 1)

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
        self.map_size_x = 350
        self.map_size_y = 350
        
        # 맵의 해상도 설정 (한 그리드 셀당 0.05미터)
        self.map_resolution = 0.05
        
        # 맵의 오프셋 설정 (맵의 좌상단 기준, 맵의 물리적 위치를 결정하는 좌표 오프셋)
        self.map_offset_x = -8 - 8.75
        self.map_offset_y = -4 - 8.75

        # 그리드 맵 크기 설정 (맵의 그리드 셀 수: 350x350)
        self.GRIDSIZE = 350

        # 8방향으로 이동할 때 각 좌표 변화량을 나타내는 배열 (상, 하, 좌, 우, 대각선)
        self.dx = [-1, 0, 0, 1, -1, -1, 1, 1]
        self.dy = [0, 1, -1, 0, -1, 1, -1, 1]

        # 각 이동 방향에 따른 비용 (대각선 방향의 비용이 1.414로 더 큼)
        self.dCost = [1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414]

    def grid_update(self):
        # 맵 데이터를 그리드 형태로 업데이트
        # self.is_grid_update = True
        '''
        로직 3. 맵 데이터 행렬로 바꾸기
        map_to_grid=  # 맵 데이터를 그리드로 변환
        self.grid=  # 변환한 그리드 데이터를 저장
        '''
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
                if data[idx] == -1:  # -1이면 정보 없음
                    self.grid[y][x] = 100  # 장애물로 간주하여 100으로 설정
                else:
                    self.grid[y][x] = data[idx]  # 실제 맵 데이터 값으로 설정

        print("Grid map updated:", self.grid)


    def pose_to_grid_cell(self, x, y):
        # 로봇의 물리적 위치(x, y)를 그리드 셀 좌표로 변환하는 함수
        map_point_x = 0
        map_point_y = 0
        '''
        로직 4. 위치(x,y)를 map의 grid cell로 변환 
        (테스트) pose가 (-8,-4)라면 맵의 중앙에 위치하게 된다. 따라서 map_point_x,y는 map size의 절반인 (175,175)가 된다.
        map_point_x= ?
        map_point_y= ?
        '''   
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
        x = 0
        y = 0
        '''
        로직 5. map의 grid cell을 위치(x,y)로 변환
        (테스트) grid cell이 (175,175)라면 맵의 중앙에 위치하게 된다. 따라서 pose로 변환하면 (-8,-4)가 된다.
        x=?
        y=?
        '''
        # 그리드로 변환 과정을 반대로 진행한다.
        # grid_cell의 0번째는 x그리드 값, 1번째는 y그리드 값
        x = self.map_offset_x + (grid_cell[0] * self.map_resolution)
        y = self.map_offset_y + (grid_cell[1] * self.map_resolution)
        
        return [x, y]

    def odom_callback(self, msg):
        # odom 콜백 함수: 로봇의 위치 정보를 저장하고 플래그를 True로 설정
        self.is_odom = True
        self.odom_msg = msg

    def map_callback(self, msg):
        # 맵 콜백 함수: 맵 정보를 저장하고 플래그를 True로 설정
        self.is_map = True
        self.map_msg = msg

    def goal_callback(self, msg):
        # goal_pose 콜백 함수: 목표 위치를 받아 경로 탐색을 시작
        if msg.header.frame_id == 'map':
            # 로직 6. goal_pose 메시지 수신하여 목표 위치 설정
            goal_x =  msg.pose.position.x # 메시지에서 목표 위치 x 좌표 가져오기
            goal_y =  msg.pose.position.y # 메시지에서 목표 위치 y 좌표 가져오기
            goal_cell =  self.pose_to_grid_cell(goal_x, goal_y) # 목표 위치를 그리드 셀로 변환
            self.goal =  goal_cell # 목표 그리드 셀을 저장
            print("msg : ", msg)
            
            # 맵과 odom 데이터가 모두 수신되었고, 그리드가 업데이트되지 않았을 경우 그리드를 업데이트
            if self.is_map == True and self.is_odom == True:
                if self.is_grid_update == False:
                    self.grid_update()
                
                # 경로를 저장할 리스트 초기화
                self.final_path = []

                # 현재 로봇의 위치를 odom 데이터에서 가져와 그리드 셀로 변환
                x = self.odom_msg.pose.pose.position.x
                y = self.odom_msg.pose.pose.position.y
                start_grid_cell = self.pose_to_grid_cell(x, y)

                # 경로와 비용 배열 초기화 (모든 셀에 대해 최대 비용으로 초기화)
                self.path = [[0 for col in range(self.GRIDSIZE)] for row in range(self.GRIDSIZE)]
                self.cost = np.array([[self.GRIDSIZE * self.GRIDSIZE for col in range(self.GRIDSIZE)] for row in range(self.GRIDSIZE)])

                # 다익스트라 알고리즘을 통해 경로를 탐색하는 부분
                if self.grid[start_grid_cell[0]][start_grid_cell[1]] == 0  and self.grid[self.goal[0]][self.goal[1]] == 0  and start_grid_cell != self.goal :
                    self.dijkstra(start_grid_cell)

                # Path 메시지를 초기화하고, 경로의 각 점을 global_path_msg에 추가
                self.global_path_msg = Path()
                self.global_path_msg.header.frame_id = 'map'
                
                # 최종 경로(final_path)를 역순으로 탐색하여 PoseStamped 메시지로 변환
                for grid_cell in reversed(self.final_path):
                    tmp_pose = PoseStamped()
                    waypoint_x, waypoint_y = self.grid_cell_to_pose(grid_cell)
                    tmp_pose.pose.position.x = waypoint_x
                    tmp_pose.pose.position.y = waypoint_y
                    tmp_pose.pose.orientation.w = 1.0
                    self.global_path_msg.poses.append(tmp_pose)
                
                # 최종 경로가 있다면 퍼블리시
                if len(self.final_path) != 0:
                    self.a_star_pub.publish(self.global_path_msg)

    def dijkstra(self, start):
        # 다익스트라 알고리즘을 사용하여 최단 경로를 찾는 함수
        Q = deque()  # BFS 탐색을 위한 큐
        Q.append(start)  # 시작 지점을 큐에 추가
        self.cost[start[0]][start[1]] = 1  # 시작 지점의 비용을 1로 설정 (최소 비용)
        found = False  # 경로를 찾았는지 여부를 나타내는 플래그
        
        # 로직 7. grid 기반 최단경로 탐색
        while Q:  # 탐색할 노드가 남아있는 동안 반복
            current = Q.popleft()  # 큐에서 현재 노드를 꺼냄
            
            if current == self.goal:  # 현재 노드가 목표 지점이면 탐색 종료
                found = True
                break
            
            # 현재 위치에서 이동할 수 있는 8방향(상, 하, 좌, 우, 대각선)으로 탐색
            for i in range(8):
                next_x = current[0] + self.dx[i]  # x축 이동
                next_y = current[1] + self.dy[i]  # y축 이동
                next = (next_x, next_y)
                
                # 그리드 내부에 있는지 확인
                if next[0] >= 0 and next[1] >= 0 and next[0] < self.GRIDSIZE and next[1] < self.GRIDSIZE:
                    # 장애물 또는 탐색 불가능한 영역이 아닌 경우
                    if self.grid[next[0]][next[1]] < 50:  # 맵의 값이 50 미만이면 이동 가능 (장애물 테스트)
                        new_cost = self.cost[current[0]][current[1]] + self.dCost[i]  # 이동 비용 계산
                        # 기존 비용보다 새로운 경로의 비용이 적을 경우
                        if self.cost[next[0]][next[1]] > new_cost:
                            Q.append(next)  # 이동할 다음 위치를 큐에 추가
                            self.path[next[0]][next[1]] = current  # 경로를 저장
                            self.cost[next[0]][next[1]] = new_cost  # 비용 갱신

        # 경로가 발견된 경우
        if found:
            node = self.goal  # 목표 지점부터 역으로 경로를 추적
            while node != start:  # 시작 지점에 도달할 때까지 반복
                self.final_path.append(node)  # 경로에 현재 노드를 추가
                node = self.path[node[0]][node[1]]  # 이전 노드로 이동
            self.final_path.append(start)  # 마지막으로 시작 지점을 경로에 추가  
            print("path : ", self.final_path)  

        
def main(args=None):
    # ROS2 초기화
    rclpy.init(args=args)

    # a_star 노드 생성
    global_planner = a_star()

    # 노드 실행
    rclpy.spin(global_planner)

    # 종료 시 노드 제거 및 ROS2 종료
    global_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()