import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Dynamically find the paths to your configuration files
    dvrk_config_share = get_package_share_directory('dvrk_config')
    dvrk_model_share = get_package_share_directory('dvrk_model')
    
    json_config_path = os.path.join(dvrk_config_share, 'system', 'system-PSM1_Classic_KIN_SIMULATED.json')
    rviz_config_path = os.path.join(dvrk_model_share, 'rviz', 'InteractivePSM1.rviz')

    # 2. dVRK System Node
    dvrk_system_node = Node(
        package='dvrk_robot',
        executable='dvrk_system',
        arguments=['-j', json_config_path],
        output='screen'
    )

    # 3. Include the arm_state_publishers launch file
    # Note: ROS 2 launch files are usually stored in a 'launch' subdirectory.
    arm_state_pub_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dvrk_model_share, 'launch', 'arm_state_publishers.launch.py')
        ),
        launch_arguments={
            'arm': 'PSM1',
            'generation': 'Classic',
            'suj': 'false'
        }.items()
    )

    # 4. RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # 5. Build and return the LaunchDescription
    return LaunchDescription([
        dvrk_system_node,
        
        # We use TimerAction to simulate the 'sleep 2' from your bash script
        # to ensure dvrk_system is fully up before publishing states
        TimerAction(
            period=2.0,
            actions=[arm_state_pub_launch]
        ),
        
        TimerAction(
            period=5.0,
            actions=[rviz_node]
        )
    ])