from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import yaml


def _load_config():
    share = os.path.join(
        os.path.dirname(__file__), "..", "config", "default.yaml"
    )
    with open(os.path.abspath(share), "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _launch_setup(context, *args, **kwargs):
    cfg = _load_config()
    map_dir = LaunchConfiguration("map_dir").perform(context) or cfg["map_dir"]
    video_path = LaunchConfiguration("video_path").perform(context) or cfg["mapping_video"]
    camera_yaml = LaunchConfiguration("camera_yaml").perform(context) or cfg["camera_yaml"]
    vocabulary = LaunchConfiguration("vocabulary").perform(context) or cfg["vocabulary"]
    rate = LaunchConfiguration("rate").perform(context) or str(cfg["video_rate"])
    loop = LaunchConfiguration("loop").perform(context) or str(cfg["video_loop"]).lower()
    visualize = LaunchConfiguration("visualize").perform(context) or str(cfg["slam_visualization"]).lower()

    os.makedirs(map_dir, exist_ok=True)

    video_node = Node(
        package="veye_local",
        executable="video_player",
        name="video_player",
        output="screen",
        arguments=[video_path, rate, loop],
    )

    slam_cmd = [
        "ros2", "run", "orbslam3", "mono",
        vocabulary,
        camera_yaml,
    ]
    if visualize == "false":
        slam_cmd.append("false")
    slam_cmd.extend(["--ros-args", "-p", f"map_output_dir:={map_dir}"])

    slam_node = ExecuteProcess(
        cmd=slam_cmd,
        output="screen",
        additional_env={
            "RCUTILS_COLORIZED_OUTPUT": "1",
        },
    )

    # Start SLAM immediately; video_player waits for /orb_slam3/tracking_state internally.
    return [slam_node, video_node]


def generate_launch_description():
    cfg = _load_config()
    return LaunchDescription([
        DeclareLaunchArgument("map_dir", default_value=cfg["map_dir"]),
        DeclareLaunchArgument("video_path", default_value=cfg["mapping_video"]),
        DeclareLaunchArgument("camera_yaml", default_value=cfg["camera_yaml"]),
        DeclareLaunchArgument("vocabulary", default_value=cfg["vocabulary"]),
        DeclareLaunchArgument("rate", default_value=str(cfg["video_rate"])),
        DeclareLaunchArgument("loop", default_value=str(cfg["video_loop"]).lower()),
        DeclareLaunchArgument("visualize", default_value=str(cfg["slam_visualization"]).lower()),
        OpaqueFunction(function=_launch_setup),
    ])
