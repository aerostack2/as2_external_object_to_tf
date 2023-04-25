from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='drone0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='as2_external_object_to_tf',
            executable='as2_external_object_to_tf_node',
            name='as2_external_object_to_tf',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'config_file': LaunchConfiguration('config_file')}],
            output='screen',
            emulate_tty=True
        )
    ])
