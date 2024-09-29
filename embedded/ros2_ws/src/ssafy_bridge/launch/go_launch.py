from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # 1. 처음부터 실행되는 노드들
    udp_to_pub = Node(
        package='ssafy_bridge',
        node_executable='udp_to_pub',
        node_name='udp_to_pub'
    )
    sub_to_udp = Node(
        package='ssafy_bridge',
        node_executable='sub_to_udp',
        node_name='sub_to_udp'
    )
    udp_to_cam = Node(
        package='ssafy_bridge',
        node_executable='udp_to_cam',
        node_name='udp_to_cam'
    )
    udp_to_laser = Node(
        package='ssafy_bridge',
        node_executable='udp_to_laser',
        node_name='udp_to_laser'
    )
    odom = Node(
        package='sub1',
        node_executable='odom',
        node_name='odom',
        output='screen'
    )
    goal_publisher = Node(
        package='sub1',
        node_executable='goal_publisher',
        node_name='goal_publisher',
        output='screen'
    )

    # 2. load_map 실행 후 a_star 실행
    load_map = Node(
        package='sub1',
        node_executable='load_map',
        node_name='load_map',
        output='screen'
    )
    a_star = Node(
        package='sub1',
        node_executable='a_star',
        node_name='a_star',
        output='screen'
    )
    
    # 10초 대기 후 a_star 실행
    a_star_after_delay = TimerAction(
        period=10.0,  # 10초 대기
        actions=[a_star]
    )

    # 3. a_star 종료 후 a_star_local_path, path_pub 실행
    a_star_local_path = Node(
        package='sub1',
        node_executable='a_star_local_path',
        node_name='a_star_local_path',
        output='screen'
    )
    path_pub = Node(
        package='sub1',
        node_executable='path_pub',
        node_name='path_pub',
        output='screen'
    )
    
    a_star_local_path_after_a_star = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=a_star,
            on_exit=[a_star_local_path, path_pub]
        )
    )

    # 4. a_star_local_path와 path_pub 실행 후 path_tracking_PID 실행
    path_tracking_PID = Node(
        package='sub1',
        node_executable='path_tracking_PID',
        node_name='path_tracking_PID',
        output='screen'
    )
    
    path_tracking_after_path_pub_and_local_path = TimerAction(
        period=5.0,  # 필요 시 대기 시간 설정
        actions=[path_tracking_PID]
    )

    return LaunchDescription([
        # 처음부터 실행되는 노드들
        udp_to_pub,
        sub_to_udp,
        udp_to_cam,
        udp_to_laser,
        odom,
        goal_publisher,
        
        # load_map 실행 후 a_star 실행
        load_map,
        a_star_after_delay, # 일정 시간 대기 후 a_star 실행

        # a_star 종료 후 a_star_local_path, path_pub 실행
        a_star_local_path_after_a_star,

        # a_star_local_path 종료 후 path_tracking_PID 실행
        path_tracking_after_path_pub_and_local_path,
    ])
