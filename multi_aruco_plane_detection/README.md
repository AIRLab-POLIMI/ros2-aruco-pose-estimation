# Robust and Noise-Tolerant Plane Detection using Multiple ArUco Markers

Given a multi-aruco setup, consisting of multiple Aruco markers placed on a plane, this package detects the plane 
of the aruco markers and correct orientation of the multi-aruco setup. The package is designed to be robust to noise 
and can handle (in theory) a large number of Aruco markers.

Code developed by: __Simone Giampà, Politecnico di Milano, 2024__

Project part of my Master's Thesis at Politecnico di Milano, Italy.

## Description of the Aruco Setup

The setup used for these tests is composed of 7 markers in a plane, 2 in the top row, and 5 in the bottom row.
The markers are used to estimate the positions of the buttons on the setup, which is used for robotic manipulation tasks.

The system is:
- **Robust to noise**: SVD and PCA algorithms are used to estimate the plane and the orientation of the multi-aruco setup. 
  These algorithms are robust to noise and can handle a large number of markers.
- **Fast**: operates at 30Hz on a standard laptop, the same frequency of the camera image publisher.
- **Robust to outliers**: the RANSAC algorithm is used to estimate the plane of the multi-aruco setup. This algorithm is 
  robust to outliers and can discard non-relevant data points outside the expected data distribution.

## Description of the Algorithm

The algorithm is based on the following steps:
1. Given the Aruco markers in the image, and their poses estimated (positions and orientations)
2. The algorithm estimates the plane of the Aruco markers using a RANSAC algorithm. Each iteration of the RANSAC algorithm
   estimates a plane using a random subset of Aruco markers. The plane with the most inliers is selected as the best plane.
3. The algorithm used for estimation of the plane coordinates is based on SVD (Singular Value Decomposition) of the Aruco markers
   positions. The SVD algorithm is used to estimate the normal vector of the plane.
4. The normal vector to the plane corresponds to the z axis of the multi-aruco setup.
5. Then the algorithm estimates the x,y axes orientation of the plane using the other markers positions. The x axis orientation
    is estimated using with the PCA algorithm (Principal Component Analysis) of the bottom markers positions projected onto the plane.
6. The y axis orientation is estimated using the cross product between the z axis and the x axis computed.
7. Then the final orientation of the multi-aruco setup is obtained by converting the transformation matrix of the vectors into a
    rotation quaternion. The orientation is then applied to all aruco markers.

## Usage

Launch the Aruco pose estimator, the camera ROS2 driver node and the multi-aruco plane detection node with:

```bash
ros2 launch multi_aruco_plane_detection multi_aruco_plane_detection.launch.py
```

## Installation

Install the following dependencies:

```bash
sudo apt install ros-$ROS_DISTRO-eigen3-cmake-module
```

Install the Eigen3 library with:

```bash
sudo apt install libeigen3-dev
```
