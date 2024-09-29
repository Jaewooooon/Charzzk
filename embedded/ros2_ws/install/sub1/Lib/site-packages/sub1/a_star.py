import rclpy
import numpy as np
from rclpy.node import Node
import heapq
import os
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path

class AStar(Node):

    def __init__(self):
        super().__init__('a_star')
        print("ASTAR!!!!")
        
        # Subscriber 및 Publisher 설정
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        self.goal_sub = self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 1)
        self.a_star_pub = self.create_publisher(Path, '/global_path', 1)

        self.map_msg = OccupancyGrid()  
        self.odom_msg = Odometry()  
        self.is_map = False  
        self.is_odom = False  
        self.is_found_path = False  
        self.is_grid_update = False  

        self.goal = [184, 224]  
        self.GRIDSIZE = 350

        self.dx = [-1, 0, 0, 1, -1, -1, 1, 1]
        self.dy = [0, 1, -1, 0, -1, 1, -1, 1]
        self.dCost = [1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414]

        # 경로 및 비용 배열 초기화
        self.path = [[None for _ in range(self.GRIDSIZE)] for _ in range(self.GRIDSIZE)]
        self.cost = np.full((self.GRIDSIZE, self.GRIDSIZE), np.inf)

    def grid_update(self):
        self.is_grid_update = True
        
        width = self.map_msg.info.width  
        height = self.map_msg.info.height  
        data = self.map_msg.data  
        
        self.grid = np.zeros((height, width))  
        
        for y in range(height):
            for x in range(width):
                idx = x + (y * width)
                if data[idx] == -1:  
                    self.grid[y][x] = 100  
                else:
                    self.grid[y][x] = data[idx]  

        print("Grid map updated:", self.grid)

    def pose_to_grid_cell(self, x, y):
        # 0,0이면 그리드 한 가운데로 위치
        map_point_x = int((x + 8.75) / 0.05)  
        map_point_y = int((y + 8.75) / 0.05)  
        return map_point_x, map_point_y

    def grid_cell_to_pose(self, grid_cell):
        # 175면 0,0으로 변환
        x = (grid_cell[0] * 0.05) - 8.75
        y = (grid_cell[1] * 0.05) - 8.75
        return [x, y]

    def odom_callback(self, msg):
        self.is_odom = True
        self.odom_msg = msg

    def map_callback(self, msg):
        self.is_map = True
        self.map_msg = msg
        self.grid_update()  # 맵 업데이트 호출

    def goal_callback(self, msg):
        if msg.header.frame_id == 'map':
            goal_x = msg.pose.position.x
            goal_y = msg.pose.position.y
            goal_cell = self.pose_to_grid_cell(goal_x, goal_y)
            self.goal = goal_cell

            if self.is_map and self.is_odom:
                x = self.odom_msg.pose.pose.position.x
                y = self.odom_msg.pose.pose.position.y
                start_grid_cell = self.pose_to_grid_cell(x, y)

                self.final_path = []
                
                if self.grid[start_grid_cell[1]][start_grid_cell[0]] < 50 and start_grid_cell != self.goal:
                    self.astar(start_grid_cell)

                # Path 메시지 초기화
                self.global_path_msg = Path()
                self.global_path_msg.header.frame_id = 'map'
                
                for grid_cell in reversed(self.final_path):
                    #print("message plus!!!")
                    tmp_pose = PoseStamped()
                    waypoint_x, waypoint_y = self.grid_cell_to_pose(grid_cell)
                    tmp_pose.pose.position.x = waypoint_x
                    tmp_pose.pose.position.y = waypoint_y
                    tmp_pose.pose.orientation.w = 1.0
                    self.global_path_msg.poses.append(tmp_pose)
                    
                    if len(self.final_path) != 0:
                        self.a_star_pub.publish(self.global_path_msg)

    def heuristic(self, node):
        return abs(node[0] - self.goal[0]) + abs(node[1] - self.goal[1])       

    def astar(self, start):
        print("Start:", start)
        print("Goal:", self.goal)
        
        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic(start), start))
        self.cost[start[1]][start[0]] = 0
        
        while open_set:
            current_cost, current = heapq.heappop(open_set)
            
            if current == tuple(self.goal):
                print("Path found!", flush=True)
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
        print("Final path:", self.final_path)
        
        # 여기서 path.txt를 만들어 버리자
        # 경로를 path.txt 파일로 저장
        with open('./tg.txt', 'w') as f:
            for point in self.final_path:
                # 각 포인트를 "x,y" 형식으로 저장
                #f.write(f"{point[0]},{point[1]}\n")  # 각 점을 새로운 줄에 저장
                f.write(f"{(point[0]*0.05)-8.75},{(point[1]*0.05)-8.75}\n")  # 각 점을 새로운 줄에 저장

def main(args=None):
    rclpy.init(args=args)
    global_planner = AStar()
    rclpy.spin(global_planner)
    global_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
