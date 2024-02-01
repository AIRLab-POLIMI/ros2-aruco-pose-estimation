#include "multi_aruco_plane_detection.hpp"

/**
 * @brief constructor of the node
 * @param node_options options for the node
 */
MultiArucoPlaneDetection::MultiArucoPlaneDetection(const rclcpp::NodeOptions &node_options) : Node("multi_aruco_plane_detection", node_options) {

	// create a publisher for the corrected aruco markers
	aruco_publisher_ = this->create_publisher<ros2_aruco_interfaces::msg::ArucoMarkers>(
		"/aruco/markers/corrected", 10);

	// create a publisher for the visualization of the plane
	markers_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
		"/viz_markers", 10);

	// create a subscription to the aruco markers topic for their estimated pose and orientation
	aruco_subscription_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
		"/aruco/markers", 10, std::bind(&MultiArucoPlaneDetection::marker_callback, this, std::placeholders::_1));

	// get the name of the camera frame from the launch parameter
	camera_frame_name = this->get_parameter("camera_frame").as_string();
	if (camera_frame_name != std::string()) {
		RCLCPP_INFO(LOGGER, "Value of camera frame: %s", camera_frame_name.c_str());
	} else {
		RCLCPP_ERROR(LOGGER, "Failed to get camera frame parameter from config file");
	}
}

/**
 * @brief callback function for /aruco/markers topic subscription.
 * 		  Core thread of the node, it performs the plane detection and publishes the corrected aruco markers.
 * @param msg message containing the detected aruco markers array
 */
void MultiArucoPlaneDetection::marker_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
	// check whether the message contains at least 3 markers
	const unsigned int num_markers = msg->marker_ids.size();

	// at least 3 markers are needed to compute a valid plane
	// at most 7 markers are expected with the current setup
	if (num_markers < expected_markers_min || num_markers > expected_markers_max) {
		RCLCPP_DEBUG(LOGGER, "Received %d markers: insufficient or invalid quantity", num_markers);
		return; // not enough markers to compute a plane OR more markers than expected
	}

	//  given the set of available markers, check whether the minimum subset of markers is available
	bool required_markers_available = true;
	for (unsigned int i = 0; i < required_markers_ids.size(); i++) {
		if (std::find(msg->marker_ids.begin(), msg->marker_ids.end(), required_markers_ids[i]) == msg->marker_ids.end()) {
			required_markers_available = false;
			break;
		}
	}
	if (!required_markers_available) {
		return; // insufficient markers to compute a plane with enough confidence
	}

	RCLCPP_DEBUG(LOGGER, "Received %d markers", num_markers);

	// create a matrix of points from the markers, ordered according to the saved markers IDs
	Eigen::MatrixXd points = constructOrderedMatrixFromPoints(msg);
	// compute the normal orientation of the best fitting plane to the given points
	Eigen::Vector3d normal3d = planeFittingWithRANSAC(points);
	// compute the vector x of the plane given the aruco positions and the normal vector to the plane
	Eigen::Vector3d vector_x = lineFittingWithRegression(points, normal3d);
	// compute the vector y as the cross product between the normal and the vector x
	Eigen::Vector3d vector_y = normal3d.cross(vector_x);
	// get the quaternion from the combination of the 3 vectors
	Eigen::Quaterniond normal_quat = constructQuaternionFromVectors(vector_x, vector_y, normal3d);

	// straighten the normal vector to point upwards
	// TODO: yaw angle is derived from the normal vector orientation
	// Eigen::Quaterniond normal_quat = this->setYawZero(normal3d);

	// visualize the plane with the normal vector in the choses position (first marker registered)
	this->visualizeQuaternionWithPlane(normal_quat, msg->poses[0].position);

	// create a corrected aruco marker message and publish it with the corrected orientation
	ros2_aruco_interfaces::msg::ArucoMarkers corrected_markers(*msg);
	for (unsigned int i = 0; i < num_markers; i++) {
		// convert eigen quaterniond to geometry_msgs quaternion
		geometry_msgs::msg::Quaternion quaternion_msg;
		quaternion_msg.x = normal_quat.x();
		quaternion_msg.y = normal_quat.y();
		quaternion_msg.z = normal_quat.z();
		quaternion_msg.w = normal_quat.w();

		// create a corrected aruco marker message by changing only the orientation
		corrected_markers.poses[i].orientation = quaternion_msg;
	}
	// publish corrected markers
	aruco_publisher_->publish(corrected_markers);
}

/**
 * @brief construct a matrix of points from the detected aruco markers
 * @param msg message containing the detected aruco markers array
 * @return matrix of points, each column is a point [x,y,z]
 */
Eigen::MatrixXd MultiArucoPlaneDetection::constructOrderedMatrixFromPoints(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
	// create a matrix of points from the markers
	// order the points in the matrix according to the order of the markers IDs
	const unsigned int num_markers = msg->marker_ids.size();
	Eigen::MatrixXd points(3, num_markers);
	for (unsigned int i = 0, position = 0; i < markers_ids.size(); i++) {
		// get the index of the marker ID in the array of available markers
		int marker_index = -1;
		for (unsigned int j = 0; j < num_markers; j++) {
			if (markers_ids[i] == msg->marker_ids[j]) {
				marker_index = j;
				break;
			}
		}
		if (marker_index != -1) {
			// fill the matrix of points with the xyz coordinates of the markers
			points.col(position) << msg->poses[marker_index].position.x,
				msg->poses[marker_index].position.y, msg->poses[marker_index].position.z;
			position++;
		} else {
			// this marker is not available, so skip it
		}
	}
	return points;
}

/**
 * @brief compute the best fitting plane passing through all given points using RANSAC
 * @param points matrix of points, each column is a point [x,y,z]
 * @return the best fitting plane normal vector
 */
Eigen::Vector3d MultiArucoPlaneDetection::planeFittingWithRANSAC(const Eigen::MatrixXd &points) {

	// RANSAC algorithm for plane fitting
	// https://en.wikipedia.org/wiki/Random_sample_consensus

	// initialize the worst base model for RANSAC
	Eigen::Vector3d best_normal;
	int best_inliers = 0;
	double best_distance = 1000.0;

	// range of quantity of normal vectors to use for RANSAC
	const int num_points = points.cols(); // = points.cols()
	const int min_points = std::ceil(num_points / 2.0);

	const Eigen::Vector3d centroid = getCentroidFromPoints(points);

	// Random number generator beyween min_points and num_points
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<int> distribution(min_points, num_points);

	// create vector of indexes to select a random subset of points
	std::vector<int> subset_indices(num_points); // initialize with all the markers
	for (int k = 0; k < num_points; ++k) {
		subset_indices[k] = k;
	}

	for (int i = 0; i < max_iterations; ++i) {
		// Randomly select a subset of points, with at least min_points and at most num_points
		int subset_size = distribution(gen);
		std::shuffle(subset_indices.begin(), subset_indices.end(), gen); // select which markers to use

		// constructs subset of planes
		Eigen::MatrixXd subset(3, subset_size);
		for (int i_point = 0; i_point < subset_size; ++i_point) {
			int index = subset_indices[i_point];
			subset.col(i_point) = points.col(index);
		}

		// Fit a plane to the subset
		Eigen::Vector3d normal3d = computeNormalWithSVD(subset); // for points

		// Count inliers and compute total distance metrics
		int inlier_count = 0;
		double total_distance = 0.0;
		for (int j = 0; j < num_points; ++j) { // for every registered point
			// using xyz points as elements of the matrix
			// compute distance between plane (normal) and point
			Eigen::Vector3d point_normalized = points.col(j) - centroid;
			double distance = computeDistance(normal3d, point_normalized);
			if (distance < threshold) {
				inlier_count++;
			}
			total_distance += distance;
		}

		//  Update best plane with inlier count metric
		if (inlier_count > best_inliers) {
			best_inliers = inlier_count;
			best_normal = normal3d;
		}

		if (total_distance < best_distance) {
			best_distance = total_distance;
			// best_normal = normal3d;
		}
	}

	return best_normal;
}

/**
 * @brief compute the best fitting line passing through all given points using linear regression
 * @param points matrix of points, each column is a point [x,y,z]
 * @return the best fitting line normal vector
 */
Eigen::Vector3d MultiArucoPlaneDetection::lineFittingWithRegression(const Eigen::MatrixXd &points,
																	const Eigen::Vector3d &plane_normal) {
}

/**
 * @brief computex best fitting plane passing through all given points using SVD
 * @param points matrix of points, each column is a normal vector
 * @return the best fitting plane normal vector
 */
Eigen::Vector3d MultiArucoPlaneDetection::computeNormalWithSVD(const Eigen::MatrixXd &points) {
	// compute best fitting plane given a set of points
	// https://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points

	// Compute centroid
	Eigen::Vector3d centroid = points.rowwise().mean();
	// Subtract centroid
	Eigen::MatrixXd centeredPoints = points.colwise() - centroid;
	// Compute SVD
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(centeredPoints, Eigen::ComputeThinU | Eigen::ComputeThinV);
	// Extract the left singular vectors
	Eigen::MatrixXd leftSingularVectors = svd.matrixU();
	// Get the last column of the left singular vectors
	Eigen::Vector3d normalVector = leftSingularVectors.col(leftSingularVectors.cols() - 1);

	return normalVector;
}

/**
 * @brief compute the distance of a point [x,y,z] from a plane on its perpendicular direction
 * @param normal normal vector of the plane
 * @param point point to compute the distance from
 * @return distance of the point from the plane
 */
double MultiArucoPlaneDetection::computeDistance(const Eigen::Vector3d &normal, const Eigen::Vector3d &point) {
	// Compute the distance from a point to a plane
	// Assuming the point is part of a constellation of points where the centroid is the origin
	// https://mathinsight.org/distance_point_plane

	// Compute the dot product of the point with the vector normal to the plane (a, b, c)
	double dotProduct = normal.x() * point.x() + normal.y() * point.y() + normal.z() * point.z();
	double norm = std::sqrt(normal.x() * normal.x() + normal.y() * normal.y() + normal.z() * normal.z());

	// Compute the distance using the formula
	double distance = std::abs(dotProduct) / norm;

	return distance;
}

/**
 * @brief compute the centroid of a set of points
 * @param points matrix of points, each column is a point [x,y,z]
 * @return centroid of the points
 */
Eigen::Vector3d MultiArucoPlaneDetection::getCentroidFromPoints(const Eigen::MatrixXd &points) {
	// Compute centroid
	Eigen::Vector3d centroid = points.rowwise().mean();
	return centroid;
}

/**
 * @brief set the yaw of a quaternion to zero, given a normal vector of a plane
 * @param normal normal vector of the plane
 * @return quaternion with yaw set to zero
 */
Eigen::Quaterniond MultiArucoPlaneDetection::setYawZero(const Eigen::Vector3d &normal) {

	// convert eigen vector3d to eigen quaterniond
	Eigen::Quaterniond quaternion;
	quaternion.setFromTwoVectors(Eigen::Vector3d::UnitZ(), normal.normalized());

	// given the quaternion change the yaw angle and fix it to zero
	// Convert the quaternion to Euler angles (roll, pitch, yaw)
	Eigen::Vector3d euler = quaternion.toRotationMatrix().eulerAngles(0, 1, 2);
	// Set the yaw angle to zero
	euler[2] = 0.0;
	// Convert the modified Euler angles back to a quaternion
	Eigen::Quaterniond modifiedQuaternion(Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ()));
	return modifiedQuaternion;
}

/**
 * @brief construct a quaternion from two vectors and a normal vector
 * @param vector_x first vector = best fitting line across arucos on the bottom of the plane
 * @param vector_y second vector (must be perpendicular to the first one)
 * @param normal_z normal vector to the best fitting plane
 * @return quaternion with the given vectors as x and y axis and the normal vector as z axis
 */
Eigen::Quaterniond MultiArucoPlaneDetection::constructQuaternionFromVectors(const Eigen::Vector3d &vector_x,
																			const Eigen::Vector3d &vector_y,
																			const Eigen::Vector3d &normal_z) {
																				
}

/**
 * @brief visualize the quaternion orientation with a plane and an arrow in RViz using the /viz_markers topic
 * @param quaternion quaternion to visualize as a plane with an arrow in RViz
 * @param placement point where the center of the plane should be placed
 */
void MultiArucoPlaneDetection::visualizeQuaternionWithPlane(const Eigen::Quaterniond quaternion,
															geometry_msgs::msg::Point placement) {
	// create a visualization markery array, so to display a plane and the normal vector
	visualization_msgs::msg::Marker marker_plane;
	marker_plane.header.frame_id = camera_frame_name;
	marker_plane.header.stamp = this->now();
	marker_plane.id = 0;
	marker_plane.type = visualization_msgs::msg::Marker::CUBE;
	marker_plane.action = visualization_msgs::msg::Marker::ADD;
	marker_plane.pose.position = placement;
	marker_plane.pose.orientation.x = quaternion.x();
	marker_plane.pose.orientation.y = quaternion.y();
	marker_plane.pose.orientation.z = quaternion.z();
	marker_plane.pose.orientation.w = quaternion.w();
	marker_plane.scale.x = 0.5;
	marker_plane.scale.y = 0.5;
	marker_plane.scale.z = 0.01;
	// orange color rgb = (255, 165, 0)
	marker_plane.color.r = 255.0 / 255.0;
	marker_plane.color.g = 165.0 / 255.0;
	marker_plane.color.b = 0.0;
	marker_plane.color.a = 1.0;

	visualization_msgs::msg::Marker marker_arrow;
	marker_arrow.header.frame_id = camera_frame_name;
	marker_arrow.header.stamp = this->now();
	marker_arrow.id = 1;
	marker_arrow.type = visualization_msgs::msg::Marker::ARROW;
	marker_arrow.action = visualization_msgs::msg::Marker::ADD;
	marker_arrow.pose.position = placement;
	marker_arrow.pose.orientation.x = quaternion.x();
	marker_arrow.pose.orientation.y = quaternion.y();
	marker_arrow.pose.orientation.z = quaternion.z();
	marker_arrow.pose.orientation.w = quaternion.w();
	marker_arrow.scale.x = 0.6;
	marker_arrow.scale.y = 0.01;
	marker_arrow.scale.z = 0.01;
	// color greenish
	marker_arrow.color.r = 0.0;
	marker_arrow.color.g = 255.0 / 255.0;
	marker_arrow.color.b = 100.0 / 255.0;
	marker_arrow.color.a = 1.0;

	// create a marker array message
	visualization_msgs::msg::MarkerArray marker_array_msg;
	marker_array_msg.markers.resize(2);
	marker_array_msg.markers[0] = marker_plane;
	marker_array_msg.markers[1] = marker_arrow;

	// publish the message
	markers_publisher_->publish(marker_array_msg);
}

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	// read parameters
	rclcpp::NodeOptions node_options;
	node_options.automatically_declare_parameters_from_overrides(true);

	// Create an instance of the node
	auto node = std::make_shared<MultiArucoPlaneDetection>(node_options);

	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}