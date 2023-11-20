import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    aruco_params = os.path.join(
        get_package_share_directory('ros2_aruco'),
        'config',
        'aruco_parameters.yaml'
    )

    aruco_node = Node(
        package='ros2_aruco',
        executable='aruco_node',
        parameters=[aruco_params],
        output='screen',
        emulate_tty=True
    )

    rviz_file = os.path.join(
        get_package_share_directory('ros2_aruco'),
        'rviz',
        'cam_detect.rviz'
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file]
    )

    cam_feed_node = Node(
        package='ros2_aruco',
        executable='cam_feed',
        output='screen',
        emulate_tty=True
    )
    
    static_tf = Node(
        package='tf2_ros',
        namespace='world_to_cam',
        executable='static_transform_publisher',
        arguments=["0", "0", "0", "0", "0", "0", "world", "camera_frame"]
    )

    return LaunchDescription([
        aruco_node, rviz2_node
    ])
