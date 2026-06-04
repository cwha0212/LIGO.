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
    map_name = LaunchConfiguration("map_name")
    sub_map_name = LaunchConfiguration("sub_map_name")
    nmea_enable = LaunchConfiguration("nmea_enable")
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="Run RViz2 (default: false)",
    )
    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="map",
        description="Top-level map name for mapping outputs",
    )
    sub_map_name_arg = DeclareLaunchArgument(
        "sub_map_name",
        default_value="sub_map",
        description="Sub-map name under map_name directory",
    )
    nmea_enable_arg = DeclareLaunchArgument(
        "nmea_enable",
        default_value="true",
        description="Enable NMEA fusion/publish path",
    )
    # launch ExecuteLocal: SIGINT 후 sigterm_timeout 초 뒤 SIGTERM, (sigterm_timeout + sigkill_timeout) 초 뒤 SIGKILL.
    # ligo_mapping은 main() 종료 시점에 맵 PCD를 저장하므로, 기본값을 크게 두어 저장 중 SIGKILL을 피함.
    map_save_shutdown_timeout_sec = LaunchConfiguration("map_save_shutdown_timeout_sec")
    map_save_shutdown_timeout_arg = DeclareLaunchArgument(
        "map_save_shutdown_timeout_sec",
        default_value="86400",
        description=(
            "ligo_mapping 종료 시: SIGINT 후 SIGTERM까지 대기(초), SIGKILL은 이 값과 합으로 더 지연. "
            "맵 PCD 저장이 길 수 있으므로 크게 두는 것을 권장 (기본 24h 단계)."
        ),
    )

    pkg = get_package_share_directory("navi")
    base_config = PathJoinSubstitution([pkg, "config", "avia.yaml"])
    mode_config = PathJoinSubstitution([pkg, "config", "nx_mode_mapping.yaml"])
    rviz_config = PathJoinSubstitution([pkg, "rviz_cfg", "loam_livox.rviz"])

    ligo_node = Node(
        package="navi",
        executable="ligo_mapping",
        name="laserMapping",
        output="screen",
        parameters=[
            base_config,
            mode_config,
            {
                "pcd_save.map_name": map_name,
                "pcd_save.sub_map_name": sub_map_name,
                "nmea.nmea_enable": nmea_enable,
            },
        ],
        sigterm_timeout=map_save_shutdown_timeout_sec,
        sigkill_timeout=map_save_shutdown_timeout_sec,
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
        package="navi",
        executable="ligo_topic_to_mqtt.py",
        name="ligo_topic_to_mqtt",
        output="screen",
        parameters=[{"mqtt.nmea_enable": nmea_enable}],
        sigterm_timeout="20",
        sigkill_timeout="5",
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(stdout_colorized_envvar)
    ld.add_action(use_rviz_arg)
    ld.add_action(map_name_arg)
    ld.add_action(sub_map_name_arg)
    ld.add_action(nmea_enable_arg)
    ld.add_action(map_save_shutdown_timeout_arg)
    ld.add_action(ligo_node)
    ld.add_action(mqtt_bridge)
    ld.add_action(rviz_node)
    return ld
