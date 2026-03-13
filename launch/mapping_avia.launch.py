import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')
    stdout_colorized_envvar = SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1')

    config_file = PathJoinSubstitution([
        get_package_share_directory('ligo'), 'config', 'avia_septentrio.yaml'
    ])
    rviz_config = PathJoinSubstitution([
        get_package_share_directory('ligo'), 'rviz_cfg', 'loam_livox.rviz'
    ])

    ligo_node = Node(
        package='ligo',
        executable='ligo_mapping',
        name='laserMapping',
        output='screen',
        parameters=[config_file],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config, '--ros-args', '--log-level', 'warn'],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(stdout_colorized_envvar)
    ld.add_action(ligo_node)
    ld.add_action(rviz_node)
    return ld
