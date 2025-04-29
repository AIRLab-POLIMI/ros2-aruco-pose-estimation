#!/usr/bin/env python3

# Code taken and readapted from:
# https://github.com/GSNCodes/ArUCo-Markers-Pose-Estimation-Generation-Python/tree/main

# Python imports
import numpy as np
import cv2
import tf_transformations
from scipy.spatial.transform import Rotation as R

# ROS2 imports
from rclpy.impl import rcutils_logger

# ROS2 message imports
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from aruco_interfaces.msg import ArucoMarkers

# utils import python code
from aruco_pose_estimation.utils import aruco_display


def pose_estimation(rgb_frame: np.array, depth_frame: np.array, aruco_detector: cv2.aruco.ArucoDetector, marker_size: float,
                    matrix_coefficients: np.array, distortion_coefficients: np.array,
                    pose_array: PoseArray, markers: ArucoMarkers) -> list[np.array, PoseArray, ArucoMarkers]:
    '''
    rgb_frame - Frame from the RGB camera stream
    depth_frame - Depth frame from the depth camera stream
    matrix_coefficients - Intrinsic matrix of the calibrated camera
    distortion_coefficients - Distortion coefficients associated with your camera
    pose_array - PoseArray message to be published
    markers - ArucoMarkers message to be published

    return:-
    frame - The frame with the axis drawn on it
    pose_array - PoseArray with computed poses of the markers
    markers - ArucoMarkers message containing markers id number and pose
    '''

    # old code version
    # parameters = cv2.aruco.DetectorParameters_create()
    # corners, marker_ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict_type, parameters=parameters)

    # new code version
    corners, marker_ids, rejected = aruco_detector.detectMarkers(image=rgb_frame)

    frame_processed = rgb_frame
    logger = rcutils_logger.RcutilsLogger(name="aruco_node")

    # If markers are detected
    if len(corners) > 0:

        logger.info("Detected {} markers.".format(len(corners)))

        for i, marker_id in enumerate(marker_ids):
            # Estimate pose of each marker and return the values rvec and tvec

            # using deprecated function
            # rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners=corners[i],
            #                                                               markerLength=marker_size,
            #                                                               cameraMatrix=matrix_coefficients,
            #                                                               distCoeffs=distortion_coefficients)
            # tvec = tvec[0]

            # alternative code version using solvePnP
            tvec, rvec, quat = my_estimatePoseSingleMarkers(corners=corners[i], marker_size=marker_size,
                                                                    camera_matrix=matrix_coefficients,
                                                                    distortion=distortion_coefficients)

            # show the detected markers bounding boxes
            frame_processed = aruco_display(corners=corners, ids=marker_ids,
                                            image=frame_processed)

            # draw frame axes
            frame_processed = cv2.drawFrameAxes(image=frame_processed, cameraMatrix=matrix_coefficients,
                                                distCoeffs=distortion_coefficients, rvec=rvec, tvec=tvec,
                                                length=0.05, thickness=3)

            if (depth_frame is not None):
                # get the centroid of the pointcloud
                centroid, quat_pc = depth_to_pointcloud_centroid(depth_image=depth_frame,
                                                        intrinsic_matrix=matrix_coefficients,
                                                        corners=corners[i])

                # log comparison between depthcloud centroid and tvec estimated positions
                logger.info(f"depthcloud centroid = {centroid}")
                logger.info(f"depthcloud rotation = {quat_pc[0]} {quat_pc[1]} {quat_pc[2]} {quat_pc[3]}")
                logger.info(f"tvec = {tvec[0]} {tvec[1]} {tvec[2]}")
                logger.info(f"quat = {quat[0]} {quat[1]} {quat[2]} {quat[3]}")
            
                # use computed centroid from depthcloud as estimated pose
                pose = Pose()
                pose.position.x = float(centroid[0])
                pose.position.y = float(centroid[1])
                pose.position.z = float(centroid[2])
                pose.orientation.x = quat_pc[0]
                pose.orientation.y = quat_pc[1]
                pose.orientation.z = quat_pc[2]
                pose.orientation.w = quat_pc[3]
            else:
                # use tvec from aruco estimator as estimated pose
                pose = Pose()
                pose.position.x = float(tvec[0])
                pose.position.y = float(tvec[1])
                pose.position.z = float(tvec[2])

                pose.orientation.x = quat[0]
                pose.orientation.y = quat[1]
                pose.orientation.z = quat[2]
                pose.orientation.w = quat[3]

            # add the pose and marker id to the pose_array and markers messages
            pose_array.poses.append(pose)
            markers.poses.append(pose)
            markers.marker_ids.append(marker_id[0])

    return frame_processed, pose_array, markers


def my_estimatePoseSingleMarkers(corners, marker_size, camera_matrix, distortion) -> tuple[np.array, np.array, np.array]:
    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)

    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers in meters
    mtx - is the camera intrinsic matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    '''
    marker_points = np.array([[-marker_size / 2.0, marker_size / 2.0, 0],
                              [marker_size / 2.0, marker_size / 2.0, 0],
                              [marker_size / 2.0, -marker_size / 2.0, 0],
                              [-marker_size / 2.0, -marker_size / 2.0, 0]], dtype=np.float32)

    # solvePnP returns the rotation and translation vectors
    retval, rvec, tvec = cv2.solvePnP(objectPoints=marker_points, imagePoints=corners,
                                        cameraMatrix=camera_matrix, distCoeffs=distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    rvec = rvec.reshape(3, 1)
    tvec = tvec.reshape(3, 1)
       
    rot, jacobian = cv2.Rodrigues(rvec)
    rot_matrix = np.eye(4, dtype=np.float32)
    rot_matrix[0:3, 0:3] = rot

    # convert rotation matrix to quaternion
    quaternion = tf_transformations.quaternion_from_matrix(rot_matrix)
    norm_quat = np.linalg.norm(quaternion)
    quaternion = quaternion / norm_quat

    return tvec, rvec, quaternion


def depth_to_pointcloud_centroid(depth_image: np.array, intrinsic_matrix: np.array,
                                 corners: np.array) -> np.array:
    """
    This function takes a depth image and the corners of a quadrilateral as input,
    and returns the centroid of the corresponding pointcloud.

    Args:
        depth_image: A 2D numpy array representing the depth image.
        corners: A list of 4 tuples, each representing the (x, y) coordinates of a corner.

    Returns:
        A tuple (x, y, z) representing the centroid of the segmented pointcloud.
    """

    # Get image parameters
    height, width = depth_image.shape
    if depth_image.dtype == np.uint16:
        scale = 1000.0 # Convert from mm to meters
    elif depth_image.dtype == np.float32:
        scale = 1.0 # Already in meters

    # Check if all corners are within image bounds
    # corners has shape (1, 4, 2)
    corners_indices = np.array([(int(x), int(y)) for x, y in corners[0]])

    for x, y in corners_indices:
        if x < 0 or x >= width or y < 0 or y >= height:
            raise ValueError("One or more corners are outside the image bounds.")
    # Create a mask for the polygon
    mask = np.zeros_like(depth_image, dtype=np.uint8)
    cv2.fillPoly(mask, [corners_indices], 1)

    # Extract the depth values within the polygon
    depth_values = depth_image[mask == 1]

    # Filter out zero depth values
    depth_values = depth_values[depth_values > 0]

    # Calculate the centroid of the depth values
    if len(depth_values) == 0:
        raise ValueError("No valid depth values found within the polygon.")
    # Calculate the 3D coordinates of the centroid
    centroid_z = np.mean(depth_values) / scale  

    # Convert the 2D centroid to 3D coordinates using the intrinsic matrix
    centroid_x = (np.mean(corners_indices[:, 0]) - intrinsic_matrix[0, 2]) * centroid_z / intrinsic_matrix[0, 0]
    centroid_y = (np.mean(corners_indices[:, 1]) - intrinsic_matrix[1, 2]) * centroid_z / intrinsic_matrix[1, 1]
    centroid = np.array([centroid_x, centroid_y, centroid_z])
    
    # Rotation estimation. Should possible change this with plane fitting of the points in the polygon   
    corner_points = []
    for idx in corners_indices:
        x,y = idx
        z = depth_image[y, x] / scale
        x_3d = (x - intrinsic_matrix[0, 2]) * z / intrinsic_matrix[0, 0]
        y_3d = (y - intrinsic_matrix[1, 2]) * z / intrinsic_matrix[1, 1]
        corner_points.append([x_3d, y_3d, z])
    (topLeft, topRight, bottomRight, bottomLeft) = np.array(corner_points)
    y_mid = 0.5*(topLeft + topRight)
    x_mid = 0.5*(topRight + bottomRight)
    x_axis = x_mid - centroid
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = y_mid - centroid
    y_axis = y_axis / np.linalg.norm(y_axis)
    z_axis = np.cross(x_axis, y_axis)
    z_axis = z_axis / np.linalg.norm(z_axis)

    rotation_matrix = np.vstack([x_axis, y_axis, z_axis]).T

    return centroid, R.from_matrix(rotation_matrix).as_quat()


def is_pixel_in_polygon(pixel: tuple, corners: np.array) -> bool:
    """
    This function takes a pixel and a list of corners as input, and returns whether the pixel is inside the polygon
    defined by the corners. This function uses the ray casting algorithm to determine if the pixel is inside the polygon.
    This algorithm works by casting a ray from the pixel in the positive x-direction, and counting the number of times
    the ray intersects with the edges of the polygon. If the number of intersections is odd, the pixel is inside the
    polygon, otherwise it is outside. This algorithm works for both convex and concave polygons.

    Args:
        pixel: A tuple (x, y) representing the pixel coordinates.
        corners: A list of 4 tuples in a numpy array, each representing the (x, y) coordinates of a corner.

    Returns:
        A boolean indicating whether the pixel is inside the polygon.
    """

    # Initialize counter for number of intersections
    num_intersections = 0

    # Iterate over each edge of the polygon
    for i in range(len(corners)):
        x1, y1 = corners[i]
        x2, y2 = corners[(i + 1) % len(corners)]

        # Check if the pixel is on the same y-level as the edge
        if (y1 <= pixel[1] < y2) or (y2 <= pixel[1] < y1):
            # Calculate the x-coordinate of the intersection point
            x_intersection = (x2 - x1) * (pixel[1] - y1) / (y2 - y1) + x1

            # Check if the intersection point is to the right of the pixel
            if x_intersection > pixel[0]:
                num_intersections += 1

    # Return whether the number of intersections is odd
    return num_intersections % 2 == 1
