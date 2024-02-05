# ROS2 imports
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import TextSubstitution

# python imports
import os
import yaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # launch ros2 aruco pose estimation node
    aruco_params_file = os.path.join(
        get_package_share_directory(
            "ros2_aruco_pose_estimation"), "config", "aruco_parameters.yaml"
    )

    # load yaml file
    with open(aruco_params_file, "r") as f:
        config_yaml = yaml.safe_load(f.read())
        camera_frame = config_yaml["/aruco_node"]["ros__parameters"]["camera_frame"]

    # camera frame name argument to pass to the node
    camera_frame_arg = DeclareLaunchArgument(
        name="camera_frame",
        # set camera frame arg equal to the camera frame from the yaml file
        default_value=TextSubstitution(text=camera_frame),
        description="Camera frame of the aruco markers detected",
    )

    aruco_node = Node(
        package="ros2_aruco_pose_estimation",
        executable="aruco_node",
        parameters=[aruco_params_file],
        output="screen",
        emulate_tty=True,
    )

    # launch realsense camera node
    cam_feed_launch_file = PathJoinSubstitution(
        [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
    )

    camera_feed_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(cam_feed_launch_file),
        launch_arguments={
            "pointcloud.enable": "true",
            "enable_color": "true",
        }.items(),
    )

    # launch multi aruco plane detection node
    multi_aruco_node = Node(
        package="multi_aruco_plane_detection",
        executable="multi_aruco_plane_detection",
        name="multi_aruco_plane_detection",
        parameters=[
            {
                "camera_frame": LaunchConfiguration("camera_frame"),
            }
        ],
        output="screen",
        emulate_tty=True,
    )

    rviz_file = PathJoinSubstitution(
        [FindPackageShare("multi_aruco_plane_detection"),
         "rviz", "multi_aruco.rviz"]
    )

    rviz2_node = Node(package="rviz2", executable="rviz2",
                      arguments=["-d", rviz_file])

    return LaunchDescription(
        [camera_frame_arg, aruco_node, camera_feed_node, multi_aruco_node, rviz2_node]
    )
