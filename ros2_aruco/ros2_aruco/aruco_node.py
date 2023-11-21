"""
ROS2 wrapper code taken from:
https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco/tree/main

This node locates Aruco AR markers in images and publishes their ids and poses.

Subscriptions:
   /camera/image_raw (sensor_msgs.msg.Image)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)

Published Topics:
	/aruco_poses (geometry_msgs.msg.PoseArray)
	   Pose of all detected markers (suitable for rviz visualization)

	/aruco_markers (ros2_aruco_interfaces.msg.ArucoMarkers)
	   Provides an array of all poses along with the corresponding
	   marker ids.

Parameters:
	marker_size - size of the markers in meters (default .0625)
	aruco_dictionary_id - dictionary that was used to generate markers
						  (default DICT_5X5_250)
	image_topic - image topic to subscribe to (default /camera/image_raw)
	camera_info_topic - camera info topic to subscribe to
						 (default /camera/camera_info)

Author: Nathan Sprague
Version: 10/26/2020

"""

# ROS2 imports
import rclpy
import rclpy.node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge

# Python imports
import numpy as np
import cv2

# Local imports for custom defined functions
from ros2_aruco.utils import ARUCO_DICT, aruco_display
from ros2_aruco.pose_estimation import pose_estimation

# ROS2 message imports
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from ros2_aruco_interfaces.msg import ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class ArucoNode(rclpy.node.Node):
	def __init__(self):
		super().__init__("aruco_node")

		# Declare and read parameters from aruco_params.yaml
		self.declare_parameter(
			name="marker_size",
			value=0.0625,
			descriptor=ParameterDescriptor(
				type=ParameterType.PARAMETER_DOUBLE,
				description="Size of the markers in meters.",
			),
		)

		self.declare_parameter(
			name="aruco_dictionary_id",
			value="DICT_5X5_250",
			descriptor=ParameterDescriptor(
				type=ParameterType.PARAMETER_STRING,
				description="Dictionary that was used to generate markers.",
			),
		)

		self.declare_parameter(
			name="image_topic",
			value="/camera/image_raw",
			descriptor=ParameterDescriptor(
				type=ParameterType.PARAMETER_STRING,
				description="Image topic to subscribe to.",
			),
		)

		self.declare_parameter(
			name="camera_info_topic",
			value="/camera/camera_info",
			descriptor=ParameterDescriptor(
				type=ParameterType.PARAMETER_STRING,
				description="Camera info topic to subscribe to.",
			),
		)

		self.declare_parameter(
			name="camera_frame",
			value="",
			descriptor=ParameterDescriptor(
				type=ParameterType.PARAMETER_STRING,
				description="Camera optical frame to use.",
			),
		)

		# parse values in their format
		self.marker_size = (
			self.get_parameter(
				"marker_size").get_parameter_value().double_value
		)
		self.get_logger().info(f"Marker size: {self.marker_size}")

		dictionary_id_name = (
			self.get_parameter(
				"aruco_dictionary_id").get_parameter_value().string_value
		)
		self.get_logger().info(f"Marker type: {dictionary_id_name}")

		image_topic = (
			self.get_parameter(
				"image_topic").get_parameter_value().string_value
		)
		self.get_logger().info(f"Image topic: {image_topic}")

		info_topic = (
			self.get_parameter(
				"camera_info_topic").get_parameter_value().string_value
		)
		self.get_logger().info(f"Image info topic: {info_topic}")

		self.camera_frame = (
			self.get_parameter(
				"camera_frame").get_parameter_value().string_value
		)

		# Make sure we have a valid dictionary id:
		try:
			dictionary_id = cv2.aruco.__getattribute__(dictionary_id_name)
			# if type(dictionary_id) != type(cv2.aruco.DICT_5X5_100):
			# check if the dictionary_id is a valid dictionary inside ARUCO_DICT values
			if dictionary_id not in ARUCO_DICT.values():
				raise AttributeError
		except AttributeError:
			self.get_logger().error(
				"bad aruco_dictionary_id: {}".format(dictionary_id_name)
			)
			options = "\n".join([s for s in ARUCO_DICT])
			self.get_logger().error("valid options: {}".format(options))

		# Set up subscriptions to the camera info and camera image topics
		self.info_sub = self.create_subscription(
			CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data
		)
		self.create_subscription(
			Image, image_topic, self.image_callback, qos_profile_sensor_data
		)

		# Set up publishers
		self.poses_pub = self.create_publisher(PoseArray, "aruco_poses", 10)
		self.markers_pub = self.create_publisher(ArucoMarkers, "aruco_markers", 10)
		self.image_pub = self.create_publisher(Image, "aruco_image", 10)

		# Set up fields for camera parameters
		self.info_msg = None
		self.intrinsic_mat = None
		self.distortion = None

		# code for updated version of cv2 (4.7.0)
		# self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
		# self.aruco_parameters = cv2.aruco.DetectorParameters()
		# self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)

		self.aruco_dictionary = cv2.aruco.Dictionary_get(dictionary_id)
		self.aruco_parameters = cv2.aruco.DetectorParameters_create()

		self.bridge = CvBridge()

	def info_callback(self, info_msg):
		self.info_msg = info_msg
		self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
		self.distortion = np.array(self.info_msg.d)
		
		self.get_logger().info("Camera info received.")
		self.get_logger().info("Intrinsic matrix: {}".format(self.intrinsic_mat))
		self.get_logger().info("Distortion coefficients: {}".format(self.distortion))
		self.get_logger().info("Camera frame: {}x{}".format(self.info_msg.width, self.info_msg.height))
		
		# Assume that camera parameters will remain the same...
		self.destroy_subscription(self.info_sub)

	def image_callback(self, img_msg):
		if self.info_msg is None:
			self.get_logger().warn("No camera info has been received!")
			return

		cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="rgb8")
		markers = ArucoMarkers()
		pose_array = PoseArray()

		if self.camera_frame == "":
			markers.header.frame_id = self.info_msg.header.frame_id
			pose_array.header.frame_id = self.info_msg.header.frame_id
		else:
			markers.header.frame_id = self.camera_frame
			pose_array.header.frame_id = self.camera_frame

		markers.header.stamp = img_msg.header.stamp
		pose_array.header.stamp = img_msg.header.stamp

		frame, pose_array, markers = pose_estimation(frame=cv_image, aruco_dict_type=self.aruco_dictionary, 
													 marker_size=self.marker_size, matrix_coefficients=self.intrinsic_mat,
													 distortion_coefficients=self.distortion, pose_array=pose_array, markers=markers)

		# if some markers are detected
		if len(markers.marker_ids) > 0:
			# Publish the results with the poses and markes positions
			self.poses_pub.publish(pose_array)
			self.markers_pub.publish(markers)
		
		# publish the image frame with computed markers positions over the image
		self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))



def main():
	rclpy.init()
	node = ArucoNode()
	rclpy.spin(node)

	node.destroy_node()
	rclpy.shutdown()


if __name__ == "__main__":
	main()
