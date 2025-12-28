import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():

    server = launch_ros.actions.Node(
            package='mypkg',
            executable='pitch_server',
            output='screen',
            )
    client = launch_ros.actions.Node(
            package='mypkg',
            executable='pitch_client',
            output='screen'
            )

    return launch.LaunchDescription([server, client])

