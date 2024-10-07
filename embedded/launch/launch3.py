from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    
    # 노드 사이 쉼표 없애야함 (쉼표가 포함되어 있으면 tuple로 처리됨)
    a_star = Node(
        package='sub1',
        node_executable='a_star',
        node_name='a_star',
        output='screen'
    )
    
    path_pub = Node(
        package='sub1',
        node_executable='path_pub',
        node_name='path_pub',
        output='screen'
    )
    
    # path_tracking = Node(
    #     package='sub1',
    #     node_executable='path_tracking',
    #     node_name='path_tracking',
    #     output='screen'
    # )

    # picture_ocr = Node(
    #     package='sub1',
    #     node_executable='picture_ocr',
    #     node_name='picture_ocr',
    #     output='screen'
    # )

    # charge_ing = Node(
    #     package='sub1',
    #     node_executable='charge_ing',
    #     node_name='charge_ing',
    #     output='screen'
    # )

    path_pub_after_a_star = TimerAction(
        period = 10.0,
        actions = [path_pub]
    )

    # path_tracking_after_path_pub = TimerAction(
    #     period = 3.0,
    #     actions = [path_tracking]
    # )

    # path_tracking_after_path_pub = TimerAction(
    #     period = 3.0,
    #     actions = [path_tracking]
    # )
    # # path_tracking 종료 후 picture_ocr 실행
    # picture_ocr_after_path_tracking = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=path_tracking,
    #         on_exit=[picture_ocr],
    #     )
    # )

    # charge_ing_after_picture_ocr = TimerAction(
    #     period = 3.0,
    #     actions = [charge_ing]
    # )

    return LaunchDescription([
        # 처음부터 실행되는 노드들
       
        a_star,
        path_pub_after_a_star
        # path_tracking_after_path_pub,
        # picture_ocr_after_path_tracking,
        # charge_ing_after_picture_ocr

    ])


