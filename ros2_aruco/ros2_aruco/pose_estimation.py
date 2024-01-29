
# Code taken and readapted from:
# https://github.com/GSNCodes/ArUCo-Markers-Pose-Estimation-Generation-Python/tree/main


# Python imports
import numpy as np
import cv2
from ros2_aruco.utils import my_estimatePoseSingleMarkers, aruco_display
import tf_transformations

# ROS2 message imports
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers
from rclpy.impl import rcutils_logger


def pose_estimation(frame, aruco_dict_type, marker_size, matrix_coefficients, distortion_coefficients, pose_array, markers):
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

	# cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
	parameters = cv2.aruco.DetectorParameters_create()

	# updated code version
	# corners, marker_ids, _ = self.aruco_detector.detectMarkers(cv_image)

	corners, marker_ids, _ = cv2.aruco.detectMarkers(
		frame, aruco_dict_type, parameters=parameters)
	frame_processed = frame

	logger = rcutils_logger.RcutilsLogger(name="aruco_node")

	# If markers are detected
	if len(corners) > 0:

		logger.debug("Detected {} markers.".format(len(corners)))
		
		for i, marker_id in enumerate(marker_ids):
			# Estimate pose of each marker and return the values rvec and tvec

			rvec, tvec, markerPoints = my_estimatePoseSingleMarkers(
				corners[i], marker_size, matrix_coefficients, distortion_coefficients)
			# rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_size, matrix_coefficients, distortion_coefficients)

			# show the detected markers bounding boxes
			frame_processed = aruco_display(
				corners, marker_ids, markerPoints, frame_processed)

			# draw frame axes
			frame_processed = cv2.drawFrameAxes(
				frame_processed, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.05, 3)

			# compute pose from the rvec and tvec arrays
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
