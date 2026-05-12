from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. Interactive Marker Node
    marker_node = Node(
        package='ros2_course',
        executable='interactive_marker',
        output='screen',
        emulate_tty=True # Keeps console colors intact
    )

    # 2. PSM Interactive Grasp Node
    grasp_node = Node(
        package='ros2_course',
        executable='psm_interactive_grasp',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        marker_node,
        grasp_node
    ])