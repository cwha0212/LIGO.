from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """LIGO 실내 위치추정 모드 (PCD 기반, GNSS off).

    핵심:
      * config/avia.yaml + config/nx_mode_indoor_localization.yaml 오버레이
      * launch 인자 prior_pcd 로 prior PCD 절대 경로 지정 (yaml 의 localization.prior_pcd_path 덮어씀)
      * RViz "2D Pose Estimate" (/initialpose) 로 초기 pose 입력
      * ligo_topic_to_mqtt: /aft_mapped_to_init → MQTT position/heading (config/mqtt_topics.yaml)
      * /cloud_registered → /scan (pointcloud_to_laserscan, target_frame=base_link_2d) for Nav2
    """
    stdout_linebuf_envvar = SetEnvironmentVariable("RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1")
    stdout_colorized_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    use_rviz = LaunchConfiguration("use_rviz")
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Run RViz2 (default: true — 초기 pose 입력에 필요).",
    )

    prior_pcd = LaunchConfiguration("prior_pcd")
    prior_pcd_arg = DeclareLaunchArgument(
        "prior_pcd",
        default_value="/home/maum/map2.pcd",
        description=(
            "Prior PCD 절대 경로 (예: /home/tae/map.pcd). "
            "비우면 yaml 의 localization.prior_pcd_path 사용."
        ),
    )

    navi_pkg = get_package_share_directory("navi")
    ligo_pkg = get_package_share_directory("ligo")
    base_config = PathJoinSubstitution([ligo_pkg, "config", "avia.yaml"])
    mode_config = PathJoinSubstitution([navi_pkg, "config", "nx_mode_indoor_localization.yaml"])
    rviz_config = PathJoinSubstitution([navi_pkg, "rviz_cfg", "loam_livox.rviz"])

    ligo_node = Node(
        package="navi",
        executable="ligo_mapping",
        name="laserMapping",
        output="screen",
        parameters=[
            base_config,
            mode_config,
            {"localization.prior_pcd_path": prior_pcd},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config, "--ros-args", "--log-level", "warn"],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    # Nav2 local_costmap: /cloud_registered(camera_init) → /scan(base_link_2d)
    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        remappings=[
            ("cloud_in", "/cloud_registered"),
            ("scan", "/scan"),
        ],
        parameters=[{
            "target_frame": "base_link_2d",
            "use_inf": True,
        }],
    )

    # LIGO camera_init→aft_mapped TF + Nav2 robot_base_frame 체인
    aft_mapped_to_base_link_2d_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aft_mapped_to_base_link_2d_tf",
        arguments=["0", "0", "0", "0", "0", "0", "aft_mapped", "base_link_2d"],
    )

    # GNSS off — local(x,y,z) 경로. pose 소스는 ligo_mapping 의 /aft_mapped_to_init (camera_init).
    mqtt_bridge = Node(
        package="navi",
        executable="ligo_topic_to_mqtt.py",
        name="ligo_topic_to_mqtt",
        output="screen",
        parameters=[
            {
                "mqtt.nmea_enable": False,
                "topic.local_odom": "/aft_mapped_to_init",
            }
        ],
        sigterm_timeout="20",
        sigkill_timeout="5",
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(stdout_colorized_envvar)
    ld.add_action(use_rviz_arg)
    ld.add_action(prior_pcd_arg)
    ld.add_action(ligo_node)
    ld.add_action(aft_mapped_to_base_link_2d_tf)
    ld.add_action(pointcloud_to_laserscan)
    ld.add_action(mqtt_bridge)
    ld.add_action(rviz_node)
    return ld
