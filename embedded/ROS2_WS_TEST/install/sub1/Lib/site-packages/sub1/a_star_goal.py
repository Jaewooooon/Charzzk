import rclpy
import numpy as np
from rclpy.node import Node
import os
from geometry_msgs.msg import Pose,PoseStamped
from squaternion import Quaternion
from nav_msgs.msg import Odometry,OccupancyGrid,MapMetaData,Path
from math import pi,cos,sin
from collections import deque

# a_star 노드는  OccupancyGrid map을 받아 grid map 기반 최단경로 탐색 알고리즘을 통해 로봇이 목적지까지 가는 경로를 생성하는 노드입니다.
# 로봇의 위치(/pose), 맵(/map), 목표 위치(/goal_pose)를 받아서 전역경로(/global_path)를 만들어 줍니다. 
# goal_pose는 rviz2에서 2D Goal Pose 버튼을 누르고 위치를 찍으면 메시지가 publish 됩니다. 


# 노드 로직 순서
# 1. publisher, subscriber 만들기
# 2. 파라미터 설정
# 3. 맵 데이터 행렬로 바꾸기
# 4. 위치(x,y)를 map의 grid cell로 변환
# 5. map의 grid cell을 위치(x,y)로 변환
# 6. goal_pose 메시지 수신하여 목표 위치 설정
# 7. grid 기반 최단경로 탐색

class a_star(Node):

    def __init__(self):
        super().__init__('a_star_goal')
        # 로직 1. publisher, subscriber 만들기
        self.map_sub = self.create_subscription(OccupancyGrid,'map',self.map_callback,1)
        self.odom_sub = self.create_subscription(Odometry,'odom',self.odom_callback,1)
        self.goal_sub = self.create_subscription(PoseStamped,'goal_pose',self.goal_callback,1)
        self.a_star_pub= self.create_publisher(Path, 'global_path', 1)
        
        self.map_msg=OccupancyGrid()
        self.odom_msg=Odometry()
        self.is_map=False
        self.is_odom=False
        self.is_found_path=False
        self.is_grid_update=False



        # 로직 2. 파라미터 설정
        self.goal = [184,224] 
        self.map_size_x=350
        self.map_size_y=350
        self.map_resolution=0.05
        self.map_offset_x=-8-8.75
        self.map_offset_y=-4-8.75
    
        self.GRIDSIZE=350 
 
        self.dx = [-1,0,0,1,-1,-1,1,1]
        self.dy = [0,1,-1,0,-1,1,-1,1]
        self.dCost = [1,1,1,1,1.414,1.414,1.414,1.414]

     
        

    def grid_update(self):
        self.is_grid_update=True
        '''
        로직 3. 맵 데이터 행렬로 바꾸기
        '''
        map_to_grid=np.array(self.map_msg.data)
        self.grid=np.reshape(map_to_grid,(350, 350),order='F')



    def pose_to_grid_cell(self,x,y):
        '''
        로직 4. 위치(x,y)를 map의 grid cell로 변환 
        '''
        map_point_x= int((x - self.map_offset_x) / self.map_resolution)
        map_point_y= int((y - self.map_offset_y) / self.map_resolution) 
        return map_point_x,map_point_y

        
    def grid_cell_to_pose(self,grid_cell):

        '''
        로직 5. map의 grid cell을 위치(x,y)로 변환
        '''

        x=(grid_cell[0]+(1/self.map_resolution)*self.map_offset_x)/20
        y=(grid_cell[1]+(1/self.map_resolution)*self.map_offset_y)/20

        return [x,y]
        
    def odom_callback(self,msg):
        self.is_odom=True
        self.odom_msg=msg

    def map_callback(self,msg):
        self.is_map=True
        self.map_msg=msg

    def goal_callback(self,msg):
        '''
        로직 6. goal_pose 메시지 수신하여 목표 위치 설정
        '''
        if msg.header.frame_id=='map':
            goal_x=msg.pose.position.x
            goal_y=msg.pose.position.y
            goal_cell=self.pose_to_grid_cell(goal_x,goal_y)
            self.goal = [goal_cell[0],goal_cell[1]]
            print(self.goal)
            

            if self.is_map ==True and self.is_odom==True  :
                if self.is_grid_update==False :
                    self.grid_update()

        
                self.final_path=[]

                x=self.odom_msg.pose.pose.position.x
                y=self.odom_msg.pose.pose.position.y
                start_grid_cell=self.pose_to_grid_cell(x,y)

                self.path = [[0 for col in range(self.GRIDSIZE)] for row in range(self.GRIDSIZE)]
                self.cost = np.array([[self.GRIDSIZE*self.GRIDSIZE for col in range(self.GRIDSIZE)] for row in range(self.GRIDSIZE)])

                # 다익스트라 알고리즘을 완성하고 주석을 해제 시켜주세요. 
                # 시작지, 목적지가 탐색가능한 영역이고, 시작지와 목적지가 같지 않으면 경로탐색을 합니다.
                if self.grid[start_grid_cell[0]][start_grid_cell[1]] ==0  and self.grid[self.goal[0]][self.goal[1]] ==0  and start_grid_cell != self.goal :
                    self.dijkstra(start_grid_cell)

                self.global_path_msg=Path()
                self.global_path_msg.header.frame_id='map'
                for grid_cell in reversed(self.final_path) :
                    tmp_pose=PoseStamped()
                    waypoint_x,waypoint_y=self.grid_cell_to_pose(grid_cell)
                    tmp_pose.pose.position.x=waypoint_x
                    tmp_pose.pose.position.y=waypoint_y
                    tmp_pose.pose.orientation.w=1.0
                    self.global_path_msg.poses.append(tmp_pose)
            
                if len(self.final_path)!=0 :
                    self.a_star_pub.publish(self.global_path_msg)


    def dijkstra(self,start):
        Q = deque()
        Q.append(start)
        self.cost[start[0]][start[1]] = 1
        found = False
        '''
        로직 7. grid 기반 최단경로 탐색
        '''
        while not len(Q) == 0:
            # 목표지점에 도달하거나 더이상 갈곳이
            # 없을 경우 종료
            if found:
                break
            # 큐에 저장된 node
            current = Q.popleft()
            # grid 기반이므로 인접 그리드 8개 탐색
            for i in range(8):
                next = [current[0]+self.dx[i],current[1]+self.dy[i]]
                # grid 안의 셀이어야함
                if next[0] >= 0 and next[1] >= 0 and next[0] < self.GRIDSIZE and next[1] < self.GRIDSIZE:
                        # obstacle check
                        if self.grid[next[0]][next[1]] < 50:
                            # cost가 더 낮은 경우 갱신
                            if self.cost[next[0]][next[1]] > self.cost[current[0]][current[1]] + self.dCost[i]:
                                # 갱신된 node를 큐에 넣고 계속 탐색
                                Q.append(next)
                                # 갱신되었을때 현재 셀을 path로 저장
                                self.path[next[0]][next[1]] = current
                                # 갱신
                                self.cost[next[0]][next[1]] = self.cost[current[0]][current[1]] + self.dCost[i]
        

        node = self.goal
        while node != start:
            nextNode = self.path[node[0]][node[1]]
            self.final_path.append(node)
            print(node)
            node = nextNode
            

        
def main(args=None):
    rclpy.init(args=args)

    global_planner = a_star()

    rclpy.spin(global_planner)


    global_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




       
   
