import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    pitcher_arg = DeclareLaunchArgument(
            'pitcher',
            default_value='Collie',
            description='Starting pitcher name'
            )

    pitcher_config = LaunchConfiguration('pitcher')

    server = Node(
            package='mypkg',
            executable='pitch_server',
            output='screen',
            parameters=[{'start_pitcher': pitcher_config}]
            
            )
    client = Node(
            package='mypkg',
            executable='pitch_client',
            output='screen'
            )

    return LaunchDescription([pitcher_arg, server, client])

