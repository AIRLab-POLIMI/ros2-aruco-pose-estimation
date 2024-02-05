# ROS2 imports
from launch.substitutions import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    aruco_params = PathJoinSubstitution([
        FindPackageShare('ros2_aruco_pose_estimation'),
        'config',
        'aruco_parameters.yaml'
    ])

    aruco_node = Node(
        package='ros2_aruco_pose_estimation',
        executable='aruco_node',
        parameters=[aruco_params],
        output='screen',
        emulate_tty=True
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

    rviz_file = PathJoinSubstitution([
        FindPackageShare('ros2_aruco_pose_estimation'),
        'rviz',
        'cam_detect.rviz'
    ])

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file]
    )

    return LaunchDescription([
        aruco_node, camera_feed_node, rviz2_node
    ])
