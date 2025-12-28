import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():

    server = launch_ros.actions.Node(
            package='mypkg',
            executable='PitchServer',
            output='screen',
            parameters=[
                {'pitch_limit': 5}
                ]
            )
    client = launch_ros.actions.Node(
            package='mypkg',
            executable='PitchClient',
            output='screen'
            )

    return launch.LaunchDescription([server, client])

