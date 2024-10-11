from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, TimerAction

def generate_launch_description():

    load_map = Node(
            package='sub1',
            node_executable='load_map',
            node_name='load_map',
            output = 'screen'
        )
    
    goal_publisher_after_load_map = TimerAction(
        period = 3.0,
        actions = [load_map]
    )


    goal_publisher = Node(
        package='sub1',
        node_executable='goal_publisher',
        node_name='goal_publisher',
        output = 'screen'
    )


    a_star = Node(
        package='sub1',
        node_executable='a_star',
        node_name='a_star',
        output='screen'
    )
     

    return LaunchDescription([
       
    ])
