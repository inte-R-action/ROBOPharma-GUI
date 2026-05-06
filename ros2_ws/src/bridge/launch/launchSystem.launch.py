import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction # Added TimerAction

def generate_launch_description():
    # Paths 
    coppelia_exe = '/home/roberta/Documents/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04/coppeliaSim.sh'
    scene_file = '/home/roberta/Documents/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu24_04/scenes/FYP_Work/finalv2.ttt'

    return LaunchDescription([
        # CoppeliaSim 
        ExecuteProcess(
            cmd=[
                coppelia_exe, 
                '-s1000000000',  # Start and run for a long time (ms)
                scene_file
            ],
            output='screen'
        ),

        # Dashboard 
        Node(
            package='gui',
            executable='dashboard_node_exe',
            name='dashboard'
        ),

        #  Logic Node
        Node(
            package='bridge',
            executable='logic_node',
            name='logic'
        ),
        
        Node(
            package='bridge',
            executable='data_logger_node',
            name='data_logger'
        ),
 
        # Automatically Open Browser 
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=['google-chrome', '--new-window', 'http://localhost:8080'],
                    output='screen'
                )
            ]
        ),
    ])
