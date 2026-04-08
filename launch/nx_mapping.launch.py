import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
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

    config_file = PathJoinSubstitution([get_package_share_directory("ligo"), "config", "avia.yaml"])
    rviz_config = PathJoinSubstitution([get_package_share_directory("ligo"), "rviz_cfg", "loam_livox.rviz"])

    ligo_node = Node(
        package="ligo",
        executable="ligo_mapping",
        name="laserMapping",
        output="screen",
        parameters=[config_file],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config, "--ros-args", "--log-level", "warn"],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    share_dir = get_package_share_directory("ligo")
    # install/.../share/ligo 기준으로 워크스페이스 src/LIGO./scripts 경로를 역추적.
    mqtt_script = os.path.normpath(
        os.path.join(share_dir, "..", "..", "..", "..", "src", "LIGO.", "scripts", "ligo_topic_to_mqtt.py")
    )
    mqtt_bridge = ExecuteProcess(
        cmd=["python3", mqtt_script],
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
