map_resolution = 0.05
map_offset_x = -2- 8.75
map_offset_y = -2- 8.75

def pose_to_grid_cell(x, y):
    map_point_x = int((x - map_offset_x) / map_resolution)
    map_point_y = int((y - map_offset_y) / map_resolution)
    
    return map_point_x, map_point_y

def grid_cell_to_pose(x,y):
    x1 = map_offset_x + (x * map_resolution)
    y1= map_offset_y + (y * map_resolution)

    return [x1, y1]


# 그리드에서 255 335나옴
x1,y1 = pose_to_grid_cell(0,0)
print(x1,y1)

x,y = grid_cell_to_pose(220,220)
print(x,y)