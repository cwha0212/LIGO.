from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1")
    stdout_colorized_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    use_rviz = LaunchConfiguration("use_rviz")
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Run RViz2 (default: false)",
    )

    pkg = get_package_share_directory("ligo")
    base_config = PathJoinSubstitution([pkg, "config", "avia.yaml"])
    mode_config = PathJoinSubstitution([pkg, "config", "nx_mode_odometry.yaml"])
    rviz_config = PathJoinSubstitution([pkg, "rviz_cfg", "loam_livox.rviz"])

    ligo_node = Node(
        package="ligo",
        executable="ligo_mapping",
        name="laserMapping",
        output="screen",
        parameters=[base_config, mode_config],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config, "--ros-args", "--log-level", "warn"],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    mqtt_bridge = Node(
        package="ligo",
        executable="ligo_topic_to_mqtt.py",
        name="ligo_topic_to_mqtt",
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(stdout_colorized_envvar)
    ld.add_action(use_rviz_arg)
    ld.add_action(ligo_node)
    ld.add_action(mqtt_bridge)
    ld.add_action(rviz_node)
    return ld
