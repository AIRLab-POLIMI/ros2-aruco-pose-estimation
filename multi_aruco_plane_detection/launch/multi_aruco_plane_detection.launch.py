# ROS2 imports
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Python imports
import os
from ament_index_python.packages import get_package_share_directory
import yaml


def generate_launch_description():

    aruco_params_file = os.path.join(
        get_package_share_directory('aruco_pose_estimation'),
        'config',
        'aruco_parameters.yaml'
    )

    with open(aruco_params_file, 'r') as file:
        config = yaml.safe_load(file)

    config = config["/aruco_node"]["ros__parameters"]

    camera_frame_arg = DeclareLaunchArgument(
        name='camera_frame',
        default_value=config['camera_frame'],
        description='Name of the camera frame where the estimated pose will be',
    )

    # include launch description from aruco_pose_estimation package
    aruco_pose_estimation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("aruco_pose_estimation"),
                "launch",
                "aruco_pose_estimation.launch.py"])
        )
    )

    # launch multi aruco plane detection node
    multi_aruco_node = Node(
        package="multi_aruco_plane_detection",
        executable="multi_aruco_plane_detection",
        name="multi_aruco_plane_detection",
        parameters=[{
            "camera_frame": LaunchConfiguration("camera_frame"),
        }],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription(
        [camera_frame_arg,
         aruco_pose_estimation_launch,
         multi_aruco_node]
    )
