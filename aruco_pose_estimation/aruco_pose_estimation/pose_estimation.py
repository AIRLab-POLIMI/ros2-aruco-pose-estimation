#!/usr/bin/env python3

# Code taken and readapted from:
# https://github.com/GSNCodes/ArUCo-Markers-Pose-Estimation-Generation-Python/tree/main

# Python imports
import numpy as np
import cv2
import tf_transformations

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

        logger.debug("Detected {} markers.".format(len(corners)))

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
                centroid = depth_to_pointcloud_centroid(depth_image=depth_frame,
                                                        intrinsic_matrix=matrix_coefficients,
                                                        corners=corners[i])

                # log comparison between depthcloud centroid and tvec estimated positions
                logger.info(f"depthcloud centroid = {centroid}")
                logger.info(f"tvec = {tvec[0]} {tvec[1]} {tvec[2]}")

            # compute pose from the rvec and tvec arrays
            if (depth_frame is not None):
                # use computed centroid from depthcloud as estimated pose
                pose = Pose()
                pose.position.x = float(centroid[0])
                pose.position.y = float(centroid[1])
                pose.position.z = float(centroid[2])
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
    

    # Check if all corners are within image bounds
    # corners has shape (1, 4, 2)
    corners_indices = np.array([(int(x), int(y)) for x, y in corners[0]])

    for x, y in corners_indices:
        if x < 0 or x >= width or y < 0 or y >= height:
            raise ValueError("One or more corners are outside the image bounds.")

    # bounding box of the polygon
    x_min = int(min(corners_indices[:, 0]))
    x_max = int(max(corners_indices[:, 0]))
    y_min = int(min(corners_indices[:, 1]))
    y_max = int(max(corners_indices[:, 1]))

    # create array of pixels inside the polygon defined by the corners
    # search for pixels inside the squared bounding box of the polygon
    points = []
    for x in range(x_min, x_max):
        for y in range(y_min, y_max):
            if is_pixel_in_polygon(pixel=(x, y), corners=corners_indices):
                # add point to the list of points
                points.append([x, y, depth_image[y, x]])

    # Convert points to numpy array
    points = np.array(points, dtype=np.uint16)
   
    # convert to open3d image
    #depth_segmented = geometry.Image(points)
    # create pinhole camera model
    #pinhole_matrix = camera.PinholeCameraIntrinsic(width=width, height=height, 
    #                                               intrinsic_matrix=intrinsic_matrix)
    # Convert points to Open3D pointcloud
    #pointcloud = geometry.PointCloud.create_from_depth_image(depth=depth_segmented, intrinsic=pinhole_matrix,
    #                                                         depth_scale=1000.0)

    # apply formulas to pointcloud, where 
    # fx = intrinsic_matrix[0, 0], fy = intrinsic_matrix[1, 1]
    # cx = intrinsic_matrix[0, 2], cy = intrinsic_matrix[1, 2], 
    # u = x, v = y, d = depth_image[y, x], depth_scale = 1000.0,
    # z = d / depth_scale
    # x = (u - cx) * z / fx
    # y = (v - cy) * z / fy

    # create pointcloud
    pointcloud = []
    for x, y, d in points:
        z = d / 1000.0
        x = (x - intrinsic_matrix[0, 2]) * z / intrinsic_matrix[0, 0]
        y = (y - intrinsic_matrix[1, 2]) * z / intrinsic_matrix[1, 1]
        pointcloud.append([x, y, z])

    # Calculate centroid from pointcloud
    centroid = np.mean(np.array(pointcloud, dtype=np.uint16), axis=0)

    return centroid


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
