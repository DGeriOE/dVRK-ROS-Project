import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. dVRK 2.1.1 (Labor PC) elérési utak dinamikus meghatározása
    saw_share = get_package_share_directory('sawIntuitiveResearchKit')
    dvrk_model_share = get_package_share_directory('dvrk_model')
    
    # A JSON fájl a bash script alapján (a '/share/console/' mappa alatt található a csomagon belül)
    json_config_path = os.path.join(saw_share, 'share', 'console', 'console-PSM1_KIN_SIMULATED.json')
    
    # RViz config a dvrk 2.1.1 fájlstruktúrája alapján
    rviz_config_path = os.path.join(dvrk_model_share, 'rviz', 'PSM1.rviz')

    # 2. dVRK Console Node (2.4.0 dvrk_system helyett dvrk_console_json)
    dvrk_system_node = Node(
        package='dvrk_robot',
        executable='dvrk_console_json',
        arguments=['-j', json_config_path],
        output='screen'
    )

    # 3. State publisher launch fájl meghívása (kevesebb argumentummal)
    arm_state_pub_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dvrk_model_share, 'launch', 'dvrk_state_publisher.launch.py')
        ),
        launch_arguments={
            'arm': 'PSM1'
        }.items()
    )

    # 4. RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # 5. LaunchDescription összeállítása TimerAction késleltetésekkel (a 'sleep' kiváltására)
    return LaunchDescription([
        dvrk_system_node,
        
        TimerAction(
            period=2.0,
            actions=[arm_state_pub_launch]
        ),
        
        TimerAction(
            period=5.0,
            actions=[rviz_node]
        )
    ])