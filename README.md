# ros2-aruco-pose-estimation

ROS2 Wrapper for OpenCV Aruco Marker Tracking and Pose Estimation.

This package allows to use a camera to track Aruco markers and estimate their poses. It allows to use any camera supported by ROS2 and OpenCV.
It also allows using multiple aruco markers at the same time, and each of them will be published as a separate pose. Very versatile with respect
to the dictionary of the Aruco and their dimensions. This package was tested for the Realsense D535 camera, compatible with ROS2 Iron
thanks to the code avaiable at [ros2_intel_realsense](https://github.com/IntelRealSense/realsense-ros)

This package depends on a recent version of OpenCV python bindings and transforms3d library:

```bash
$ pip3 install opencv-contrib-python transforms3d

4 sudo apt install ros-iron-tf-transformations
```

## ROS2 API for the ros2_aruco Node

This node locates Aruco AR markers in images and publishes their ids and poses.

Subscriptions:
* `/camera/image_raw` (`sensor_msgs.msg.Image`)
* `/camera/camera_info` (`sensor_msgs.msg.CameraInfo`)

Note: the subscription topics can be changed in the `config/aruco_parameters.yaml` file.

Published Topics:
* `/aruco_poses` (`geometry_msgs.msg.PoseArray`) - Poses of all detected markers (suitable for rviz visualization)
* `/aruco_markers` (`ros2_aruco_interfaces.msg.ArucoMarkers`) - Provides an array of all poses along with the corresponding marker ids

Parameters:
* `marker_size` - size of the markers in meters (default .0625)
* `aruco_dictionary_id` - dictionary that was used to generate markers (default `DICT_5X5_250`)
* `image_topic` - image topic to subscribe to (default `/camera/image_raw`)
* `camera_info_topic` - Camera info topic to subscribe to (default `/camera/camera_info`)
* `camera_frame` - Camera optical frame to use (default to the frame id provided by the camera info message.)

## Running Marker Detection for Pose Estimation

Using the launch file - parameters will be loaded from _aruco\_parameters.yaml_.
```bash
ros2 launch ros2_aruco aruco_recognition.launch.py
```

Change the parameters in the launch file directly:
```bash
ros2 launch ros2_aruco aruco_recognition.launch.py marker_size:=0.05
```

## Generating Marker Images

```bash
ros2 run ros2_aruco aruco_generate_marker
```

Pass the `-h` flag for usage information: 

```bash
usage: aruco_generate_marker [-h] [--id ID] [--size SIZE] [--dictionary]

Generate a .png image of a specified maker.

optional arguments:
  -h, --help     show this help message and exit
  --id ID        Marker id to generate (default: 1)
  --size SIZE    Side length in pixels (default: 200)
  --dictionary   Dictionary to use. Valid options include: DICT_4X4_100,
                 DICT_4X4_1000, DICT_4X4_250, DICT_4X4_50, DICT_5X5_100,
                 DICT_5X5_1000, DICT_5X5_250, DICT_5X5_50, DICT_6X6_100,
                 DICT_6X6_1000, DICT_6X6_250, DICT_6X6_50, DICT_7X7_100,
                 DICT_7X7_1000, DICT_7X7_250, DICT_7X7_50, DICT_ARUCO_ORIGINAL
                 (default: DICT_5X5_250)
```
