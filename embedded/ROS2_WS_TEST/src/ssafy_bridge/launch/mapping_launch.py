from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, RegisterEventHandler
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
    
    # 2. run_mapping 노드
    # run_mapping = Node(
    #     package='sub1',
    #     node_executable='run_mapping',
    #     node_name='run_mapping',
    #     output='screen'
    # )

    # 3. load_map 노드
    load_map = Node(
        package='sub1',
        node_executable='load_map',
        node_name='load_map',
        output='screen'
    )

    # 4. run_localization 노드
    run_localization = Node(
        package='sub1',
        node_executable='run_localization',
        node_name='run_localization',
        output='screen'
    )

    # 6. load_map 실행 후 3초 대기 후 run_localization 실행
    run_localization_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_map,
            on_exit=[
                TimerAction(
                    period=3.0,  # load_map 실행 후 3초 대기
                    actions=[run_localization]
                )
            ]
        )
    )

    return LaunchDescription([
        # 처음부터 실행되는 노드들
        udp_to_pub,
        sub_to_udp,
        udp_to_cam,
        udp_to_laser,
        odom,
        load_map,
        # load_map 종료 후 3초 대기 후 run_localization 실행
        run_localization_handler
    ])
