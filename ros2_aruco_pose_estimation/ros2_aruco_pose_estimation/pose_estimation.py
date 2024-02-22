
# Code taken and readapted from:
# https://github.com/GSNCodes/ArUCo-Markers-Pose-Estimation-Generation-Python/tree/main


# Python imports
import numpy as np
import cv2
from ros2_aruco_pose_estimation.utils import aruco_display
import tf_transformations

# ROS2 message imports
from geometry_msgs.msg import Pose
from rclpy.impl import rcutils_logger


def pose_estimation(frame, aruco_detector: cv2.aruco.ArucoDetector, marker_size,
                    matrix_coefficients, distortion_coefficients, pose_array, markers):
    '''
    frame - Frame from the video stream
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
    corners, marker_ids, rejected = aruco_detector.detectMarkers(image=frame)

    frame_processed = frame
    logger = rcutils_logger.RcutilsLogger(name="aruco_node")

    # If markers are detected
    if len(corners) > 0:

        logger.debug("Detected {} markers.".format(len(corners)))

        for i, marker_id in enumerate(marker_ids):
            # Estimate pose of each marker and return the values rvec and tvec

            # using deprecated function
            #rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners=corners[i],
            #                                                               markerLength=marker_size,
            #                                                               cameraMatrix=matrix_coefficients,
            #                                                               distCoeffs=distortion_coefficients)
            # tvec = tvec[0]

            # alternative code version using solvePnP
            rvec, tvec, markerPoints = my_estimatePoseSingleMarkers(corners=corners[i], marker_size=marker_size,
                                                                     camera_matrix=matrix_coefficients,
                                                                     distortion=distortion_coefficients)

            # show the detected markers bounding boxes
            frame_processed = aruco_display(corners=corners, ids=marker_ids,
                                            rejected=markerPoints, image=frame_processed)

            # draw frame axes
            frame_processed = cv2.drawFrameAxes(image=frame_processed, cameraMatrix=matrix_coefficients,
                                                distCoeffs=distortion_coefficients, rvec=rvec, tvec=tvec,
                                                length=0.05, thickness=3)

            # compute pose from the rvec and tvec arrays
            # when using cv2.aruco.estimatePoseSingleMarkers
            pose = Pose()
            pose.position.x = float(tvec[0][0])
            pose.position.y = float(tvec[0][1])
            pose.position.z = float(tvec[0][2])

            rot_matrix = np.eye(4)
            rot_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvec[0]))[0]
            quat = tf_transformations.quaternion_from_matrix(rot_matrix)

            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]

            # add the pose and marker id to the pose_array and markers messages
            pose_array.poses.append(pose)
            markers.poses.append(pose)
            markers.marker_ids.append(marker_id[0])

    return frame_processed, pose_array, markers



def my_estimatePoseSingleMarkers(corners, marker_size, camera_matrix, distortion):
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

    retvals = []
    rvecs = []
    tvecs = []
    for corner in corners:
        retval, rvec, tvec = cv2.solvePnP(objectPoints=marker_points, imagePoints=corner,
                                  cameraMatrix=camera_matrix, distCoeffs=distortion, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(rvec)
        tvecs.append(tvec)
        retvals.append(retval)

    rvecs = np.array(rvecs, dtype=np.float32)
    tvecs = np.array(tvecs, dtype=np.float32)
    retvals = np.array(retvals, dtype=np.float32)
    return rvecs, tvecs, retvals

