# Aruco Pose Estimation with ROS2, using RGB and Depth camera images from Realsense D435

Code developed by: __Simone Giampà__

Project and experimentation conducted at __Politecnico di Milano, Artificial Intelligence and Robotics Laboratory, 2024__

_Project part of my Master's Thesis project at Politecnico di Milano, Italy._

ROS2 wrapper for Aruco marker detection and pose estimation, using OpenCV library. The marker detection and pose estimation is
done using RGB and optionally Depth images. This package works for ROS2 Humble and Iron.

This package allows to use cameras to detect Aruco markers and estimate their poses. It allows to use any camera with ROS2 drivers.
The code is a ROS2 publisher-subscriber working with RGB camera images for marker detection and RGB or depth images for pose estimation. 
It also allows using multiple aruco markers at the same time, and each of them will be published as a separate pose. 
The code supports many different Aruco dictionaries and sizes of markers.

This package was tested for the Realsense D435 camera, compatible with ROS2 `realsense-ros` driver,
available at [ros2_intel_realsense](https://github.com/IntelRealSense/realsense-ros). The code should work equally well on other and
different cameras, provided a proper calibration of the camera parameters.

## Installation

This package depends on a recent version of OpenCV python library and transforms libraries:

```bash
$ pip3 install opencv-python opencv-contrib-python transforms3d

$ sudo apt install ros-iron-tf-transformations
```

Build the package from source with `colcon build --symlink-install` in the workspace root.

## Aruco Pose Detection and Estimation ROS2 nodes description

This node subscribes to the RGB and optionally Depth images from the camera, and the camera inof topic for
intrinsic and distortion parameters. It detects Aruco markers in the RGB image, and estimates their poses using the
camera intrinsic parameters or the depth image. The poses are published as PoseArray message, and the detected markers
are published as ArucoMarkers messages. The output image contains the detected markers and aruco bounding boxes drawn on it.

__Subscribed topics__ (topic names can be changed in the `config/aruco_parameters.yaml` file):

* `/camera/image_raw`: RGB image input (`sensor_msgs.msg.Image`)
* `/camera/depth/image_rect_raw`: Depth image input (`sensor_msgs.msg.Image`)
* `/camera/camera_info`: Camera intrinsic, projection, distortion parameters (`sensor_msgs.msg.CameraInfo`)

__Published topics__ (topic names can be changed in the `config/aruco_parameters.yaml` file):

* `/aruco/poses`: Poses of all detected markers, suitable for rviz visualization - (`geometry_msgs.msg.PoseArray`) - 
* `/aruco/markers`: Provides an array of all poses along with the corresponding marker ids - (`aruco_interfaces.msg.ArucoMarkers`)
* `/aruco/image`: Output image with detected markers drawn on it, for visualization purposes - (`sensor_msgs.msg.Image`)

__Parameters__ for the node can be set in the `config/aruco_parameters.yaml` file, and include the following options:

* `marker_size` - size of the markers in meters
* `aruco_dictionary_id` - dictionary type that was used to generate markers (example `DICT_5X5_250`)
* `use_depth_input` - use depth image for pose estimation (default `false`)
* `image_topic` - RGB image topic to subscribe to, provided by the camera ROS2 driver
* `depth_image_topic` - Depth image topic to subscribe to, provided by the camera ROS2 driver
* `camera_info_topic` - Camera info topic to subscribe to, providing intrinsic and distortion parameters
* `camera_frame` - Camera optical frame to use (default to the frame id provided by the camera info message.)
* `detecter_markers_topic` - Topic to publish the detected markers as ArucoMarkers message
* `markers_visualization_topic` - Topic to publish the detected markers as PoseArray message
* `output_image_topic` - Topic to publish the output image with detected markers drawn on it, for visualization purposes

## Running Marker Detection for Pose Estimation

Launch the aruco pose estimation node with this command. The parameters will be loaded from _aruco\_parameters.yaml_,
but can also be changed directly in the launch file with command line arguments.

```bash
ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py
```

Change the parameters directly in the launch file:

```bash
ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py marker_size:=0.1 aruco_dictionary_id:=DICT_5X5_250 camera_frame:=camera_link
```

### Future updates

It will soon be possible to load the camera calibrated parameters from a yaml configuration file, so that
the camera intrinsic and distortion parameters can be loaded without relying on the camera_info topic or service.