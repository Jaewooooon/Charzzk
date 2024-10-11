from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='sub1',
        #     node_executable='controller',
        #     node_name='controller',
        #     output = 'screen'
        # ),

        # 영상 윈도우 뜨게 함 -> 실시간 송출할 일이 있을까?
        # 위급 상황에서 실행 되게끔??

        # Node(
        #     package='sub1',
        #     node_executable='perception',
        #     node_name='image_convertor',
        #     output = 'screen'
        # ),
        
        Node(
            package='sub1',
            node_executable='odom',
            node_name='odom',
            output = 'screen'
        ),
        # Node(
        #     package='sub1',
        #     node_executable='picture',
        #     node_name='picture',
        #     output = 'screen'
        # ),
        # Node(
        #     package='sub1',
        #     node_executable='goal_publisher',
        #     node_name='goal_publisher',
        #     output = 'screen'
        # ),

        # Node(
        #     package='sub2',
        #     node_executable='load_map',
        #     node_name='load_map',
        #     output = 'screen'
        # ),
        # Node(
        #     package = 'sub1',
        #     node_executable = 'path_pub',
        #     node_name = 'path_pub',
        #     output = 'screen'
        # ),
        # Node(
        #     package = 'sub1',
        #     node_executable = 'path_tracking_PID',
        #     node_name = 'path_tracking_PID',
        #     output = 'screen'
        # ),
        # Node(
        #     package='sub1',
        #     node_executable='make_path',
        #     node_name='make_path',
        #     output = 'screen'
        # ),

        # Node(
        #     package = 'sub1',
        #     node_executable = 'a_star',
        #     node_name = 'a_star',
        #     output = 'screen'
        # ),
        # Node(
        #     package = 'sub1',
        #     node_executable = 'a_star_local_path',
        #     node_name = 'a_star_local_path',
        #     output = 'screen'
        # ),
        # Node(
        #     package = 'sub1',
        #     node_executable = 'load_map',
        #     node_name = 'load_map',
        #     output = 'screen'
        # ),
        

        Node(
            package='ssafy_bridge',
            node_executable='udp_to_pub',
            node_name='udp_to_pub'
        ),
        Node(
            package='ssafy_bridge',
            node_executable='sub_to_udp',
            node_name='sub_to_udp'
        ),
        Node(
            package='ssafy_bridge',
            node_executable='udp_to_cam',
            node_name='udp_to_cam'
        ),

        Node(
            package='ssafy_bridge',
            node_executable='udp_to_laser',
            node_name='udp_to_laser'
        ) 
    ])



